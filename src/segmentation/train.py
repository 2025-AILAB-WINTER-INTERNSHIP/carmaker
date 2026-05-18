"""Semantic segmentation 학습 entry point.

전체 흐름:
1. config를 읽는다.
2. Adapter가 image/mask pair를 찾는다.
3. Dataset/DataLoader가 batch를 만든다.
4. config에 맞는 model/loss/optimizer를 만든다.
5. train/validation을 반복하며 TensorBoard와 checkpoint를 저장한다.
"""

from __future__ import annotations

import argparse
import atexit
import importlib.util
import json
import os
import random
import re
import subprocess
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

import numpy as np
import torch
import yaml
from torch.utils.data import DataLoader

try:
    from .adapters import CarmakerSegmentationAdapter
    from .dataset import SegmentationDataset, split_dataset
    from .losses import build_loss
    from .metrics import confusion_matrix, mask_psnr, segmentation_scores
    from .models import build_model
    from .utils.visualization import make_image_grid, overlay_mask
    from .utils.visualization_gpu import (
        colorize_mask as colorize_mask_gpu,
        make_image_grid_gpu,
        overlay_mask_gpu,
    )
except ImportError:
    # `python3 src/segmentation/train.py`처럼 파일을 직접 실행할 때 사용하는 fallback import.
    from adapters import CarmakerSegmentationAdapter
    from dataset import SegmentationDataset, split_dataset
    from losses import build_loss
    from metrics import confusion_matrix, mask_psnr, segmentation_scores
    from models import build_model
    from utils.visualization import make_image_grid, overlay_mask
    from utils.visualization_gpu import (
        colorize_mask as colorize_mask_gpu,
        make_image_grid_gpu,
        overlay_mask_gpu,
    )

try:
    import lightning.pytorch as L
    from lightning.pytorch.callbacks import LearningRateMonitor, ModelCheckpoint
    from lightning.pytorch.loggers import TensorBoardLogger
    from lightning.pytorch.strategies import DDPStrategy
    from lightning.pytorch.utilities.rank_zero import rank_zero_info
except ImportError:
    try:
        import pytorch_lightning as L
        from pytorch_lightning.callbacks import LearningRateMonitor, ModelCheckpoint
        from pytorch_lightning.loggers import TensorBoardLogger
        from pytorch_lightning.strategies import DDPStrategy
        from pytorch_lightning.utilities.rank_zero import rank_zero_info
    except ImportError:
        L = None


class SegmentationDataModule(L.LightningDataModule if L else object):
    """PyTorch Lightning용 DataModule.

    Adapter 생성, Dataset Split, DataLoader 구성을 담당한다.
    """

    def __init__(self, cfg: Dict[str, Any], adapter: CarmakerSegmentationAdapter):
        super().__init__()
        self.cfg = cfg
        self.adapter = adapter
        self.train_dataset = None
        self.val_dataset = None
        self.test_dataset = None
        self.split_report = ""
        self.split_info = {}

    def setup(self, stage: str | None = None):
        # Dataset 생성
        full_dataset = SegmentationDataset(
            adapter=self.adapter,
            image_size=tuple(self.cfg.get("image_size", [512, 512])),
        )

        overfit_count = int(self.cfg.get("overfit_count", 0))
        if overfit_count > 0:
            full_dataset.samples = full_dataset.samples[:overfit_count]

        if len(full_dataset) == 0:
            raise RuntimeError("No training samples found.")

        # Train / Val / Test Split
        self.train_dataset, self.val_dataset, self.test_dataset, self.split_report, self.split_info = split_dataset(
            full_dataset,
            val_ratio=float(self.cfg.get("val_ratio", 0.2)),
            seed=int(self.cfg.get("seed", 42)),
            test_ratio=float(self.cfg.get("test_ratio", 0.1)),
            stratify_by_camera=bool(self.cfg.get("stratify_by_camera", False)),
            split_by_scenario=bool(self.cfg.get("split_by_scenario", True)),
            manual_split=self.cfg.get("manual_split"),
        )
        # HParams 기록을 위해 cfg에 상세 분할 정보 주입
        self.cfg.update(self.split_info)

    def train_dataloader(self):
        return DataLoader(
            self.train_dataset,
            batch_size=int(self.cfg.get("batch_size", 4)),
            shuffle=True,
            num_workers=int(self.cfg.get("num_workers", 2)),
            pin_memory=torch.cuda.is_available(),
        )

    def val_dataloader(self):
        return DataLoader(
            self.val_dataset,
            batch_size=int(self.cfg.get("batch_size", 4)),
            shuffle=False,
            num_workers=int(self.cfg.get("num_workers", 2)),
            pin_memory=torch.cuda.is_available(),
        )

    def test_dataloader(self):
        if not self.test_dataset:
            return None
        return DataLoader(
            self.test_dataset,
            batch_size=int(self.cfg.get("batch_size", 4)),
            shuffle=False,
            num_workers=int(self.cfg.get("num_workers", 2)),
            pin_memory=torch.cuda.is_available(),
        )


class SegmentationLightningModule(L.LightningModule if L else object):
    """PyTorch Lightning용 Module.

    모델, 손실 함수, 최적화 및 메트릭 계산 로직을 포함한다.
    """

    def __init__(
        self,
        cfg: Dict[str, Any],
        num_classes: int,
        palette: Any,
        class_names: list[str],
    ):
        super().__init__()
        self.save_hyperparameters(ignore=["palette"])
        self.cfg = cfg
        self.num_classes = num_classes
        self.palette = palette
        self.class_names = class_names

        self.model = build_model(cfg, num_classes=num_classes)
        self.criterion = build_loss(
            name=str(cfg.get("loss", "cross_entropy")),
            num_classes=num_classes,
            class_weights=cfg.get("class_weights"),
            dice_exclude_classes=cfg.get("dice_exclude_classes", [0]),
        )

        # GPU 가속 시각화를 위해 팔레트를 버퍼로 등록 (모델과 함께 이동)
        self.register_buffer(
            "palette_tensor",
            torch.tensor(palette, dtype=torch.uint8),
            persistent=False,
        )

        self.train_step_outputs = []
        self.validation_step_outputs = []
        self.test_step_outputs = []
        self.worst_samples = []
        self.num_worst = int(cfg.get("debug_worst_sample_count", 4))
        self.best_miou = 0.0
        self.best_miou_fg = 0.0

    @property
    def display_epoch(self) -> int:
        """User-facing epoch index (1-based) for logging."""
        return self.current_epoch + 1

    def forward(self, x):
        return self.model(x)

    def training_step(self, batch, batch_idx):
        image = batch["image"]
        mask = batch["mask"]
        logits = self(image)
        loss = self.criterion(logits, mask)

        # 화면(Progress Bar)용 로깅
        opt = self.trainer.optimizers[0]
        current_lr = opt.param_groups[0]["lr"]
        self.log("train/lr", current_lr, on_step=False, on_epoch=True, prog_bar=True, logger=False, batch_size=image.shape[0])

        # 실시간 스텝 로깅 (x축: Step)
        self.log(
            "train/loss_step",
            loss,
            on_step=True,
            on_epoch=False,
            prog_bar=True,
            sync_dist=True,
            batch_size=image.shape[0],
        )

        # 에폭 평균 계산을 위해 저장
        self.train_step_outputs.append(loss.detach())
        return loss

    def on_train_epoch_end(self):
        if not self.train_step_outputs:
            return

        avg_loss = torch.stack(self.train_step_outputs).mean()

        # DDP 환경에서 모든 rank의 loss를 평균내어 동기화 (정확한 통계 확보)
        if self.trainer.world_size > 1:
            avg_loss = self.all_gather(avg_loss).mean()

        if self.global_rank == 0 and not self.trainer.sanity_checking:
            writer = self.logger.experiment
            # 훈련 손실 에폭 평균 로깅 (x축: Epoch)
            writer.add_scalar("train/loss", avg_loss, self.display_epoch)

            # 학습률 에폭 로깅 (x축: Epoch)
            opt = self.trainer.optimizers[0]
            current_lr = opt.param_groups[0]["lr"]
            writer.add_scalar("train/lr", current_lr, self.display_epoch)

        self.train_step_outputs.clear()

    def validation_step(self, batch, batch_idx):
        image = batch["image"]
        mask = batch["mask"]
        logits = self(image)
        loss = self.criterion(logits, mask)

        pred = torch.argmax(logits, dim=1)
        matrix = confusion_matrix(pred, mask, self.num_classes).to(self.device)
        psnr = mask_psnr(pred, mask, max_value=float(self.num_classes - 1))

        output = {
            "val_loss": loss,
            "matrix": matrix,
            "psnr": psnr,
        }
        self.validation_step_outputs.append(output)

        # 화면(Progress Bar)용으로만 로깅하고, 텐서보드 기록(logger)은 나중에 에폭 끝에서 수동으로 수행합니다.
        self.log(
            "val/loss",
            loss,
            on_epoch=True,
            prog_bar=True,
            sync_dist=True,
            batch_size=image.shape[0],
            logger=False,
        )

        # Worst Sample 추적 로직 (모든 Rank가 동일하게 수행하여 DDP 동기화 깨짐 방지)
        # 1080p 이미지를 매번 CPU로 옮기면 RAM 폭사가 발생하므로, 점수가 나쁜 경우에만 데이터를 복사합니다.
        image_log_interval = int(self.cfg.get("image_log_interval", 5))
        if self.current_epoch == 0 or (self.current_epoch + 1) % image_log_interval == 0:
            with torch.no_grad():
                pred_cpu = pred.cpu()
                mask_cpu = mask.cpu()
                for i in range(image.shape[0]):
                    img_matrix = confusion_matrix(pred_cpu[i : i + 1], mask_cpu[i : i + 1], self.num_classes)
                    img_scores = segmentation_scores(img_matrix)
                    miou = img_scores["miou"]
                    miou_fg = img_scores["miou_fg"]

                    # 현재 리스트가 꽉 차지 않았거나, 현재 샘플이 기존 최악 중 하나보다 더 나쁜 점수일 때만 저장
                    if len(self.worst_samples) < self.num_worst or miou_fg < self.worst_samples[-1]["miou_fg"]:
                        self.worst_samples.append({
                            "miou": miou,
                            "miou_fg": miou_fg,
                            "image": image[i].cpu(),
                            "mask": mask_cpu[i],
                            "pred": pred_cpu[i],
                        })
                        # 점수 오름차순 정렬 (낮은 점수가 앞쪽)
                        self.worst_samples.sort(key=lambda x: x["miou_fg"])
                        self.worst_samples = self.worst_samples[: self.num_worst]

        return output

    def test_step(self, batch, batch_idx):
        image = batch["image"]
        mask = batch["mask"]
        logits = self(image)
        loss = self.criterion(logits, mask)

        pred = torch.argmax(logits, dim=1)
        matrix = confusion_matrix(pred, mask, self.num_classes).to(self.device)
        psnr = mask_psnr(pred, mask, max_value=float(self.num_classes - 1))

        output = {
            "test_loss": loss,
            "matrix": matrix,
            "psnr": psnr,
        }
        self.test_step_outputs.append(output)

        self.log(
            "test/loss",
            loss,
            on_epoch=True,
            prog_bar=True,
            sync_dist=True,
            batch_size=image.shape[0],
        )
        return output

    def on_test_epoch_end(self):
        if not self.test_step_outputs:
            return

        all_matrices = torch.stack([x["matrix"] for x in self.test_step_outputs])
        total_matrix = torch.sum(all_matrices, dim=0)

        if self.trainer.world_size > 1:
            total_matrix = self.all_gather(total_matrix).sum(dim=0)

        scores = segmentation_scores(total_matrix.cpu())

        # Log metrics (do not sync here; already aggregated)
        self.log("test/miou", scores["miou"], prog_bar=True, sync_dist=False)
        self.log("test/dice", scores["dice"], prog_bar=True, sync_dist=False)

        psnrs = [x["psnr"] for x in self.test_step_outputs if np.isfinite(x["psnr"])]
        if psnrs:
            avg_psnr = torch.tensor(psnrs).mean()
            self.log("test/psnr", avg_psnr, sync_dist=False)

        if self.global_rank == 0 and not self.trainer.sanity_checking:
            writer = self.logger.experiment
            _write_confusion_matrix_text(
                writer,
                "test/confusion_matrix",
                self.display_epoch,
                total_matrix,
                self.class_names,
            )

        self.test_step_outputs.clear()

    def on_validation_epoch_end(self):
        # 1-based 에폭 번호로 체크포인트 파일명을 명명하기 위해 로깅
        self.log(
            "epoch/display",
            float(self.display_epoch),
            on_epoch=True,
            sync_dist=False,
            logger=False,
            add_dataloader_idx=False,
        )

        if not self.validation_step_outputs:
            return

        all_matrices = torch.stack([x["matrix"] for x in self.validation_step_outputs])
        total_matrix = torch.sum(all_matrices, dim=0)

        if self.trainer.world_size > 1:
            total_matrix = self.all_gather(total_matrix).sum(dim=0)

        scores = segmentation_scores(total_matrix.cpu())
        miou = float(scores["miou"])
        miou_fg = float(scores["miou_fg"])

        # sanity check 기간을 제외하고 매 에폭의 최고 mIoU 실시간 업데이트
        if not self.trainer.sanity_checking:
            if miou > self.best_miou:
                self.best_miou = miou
            if miou_fg > self.best_miou_fg:
                self.best_miou_fg = miou_fg

        # 화면(Progress Bar)용 로깅 (로거 기록은 제외)
        self.log("val/miou", miou, prog_bar=True, sync_dist=False, logger=False, add_dataloader_idx=False) # Validation Dataloader Index 0
        self.log("val/miou_fg", miou_fg, prog_bar=True, sync_dist=False, logger=False, add_dataloader_idx=False)
        self.log("val/dice", scores["dice"], prog_bar=True, sync_dist=False, logger=False, add_dataloader_idx=False)

        # Val Loss 및 PSNR 동기화 (모든 Rank가 참여해야 하므로 if문 바깥에서 실행)
        avg_val_loss = torch.stack([x["val_loss"] for x in self.validation_step_outputs]).mean()
        # DDP 동기화 시 텐서 크기 불일치로 인한 크래시(RuntimeError)를 원천 차단하기 위해,
        # 각 스텝의 PSNR 값을 필터링 없이 고정 크기로 수집한 뒤 DDP gather 후 Rank 0에서 finite 값을 선별합니다.
        psnrs = torch.tensor([x["psnr"] for x in self.validation_step_outputs], device=self.device)

        if self.trainer.world_size > 1:
            avg_val_loss = self.all_gather(avg_val_loss).mean()
            psnrs = self.all_gather(psnrs).flatten()

        if self.global_rank == 0 and not self.trainer.sanity_checking:
            writer = self.logger.experiment
            writer.add_scalar("val/loss", avg_val_loss, self.display_epoch)

            # 주요 지표들 수동 에폭 로깅
            writer.add_scalar("val/miou", scores["miou"], self.display_epoch)
            writer.add_scalar("val/miou_fg", scores["miou_fg"], self.display_epoch)
            writer.add_scalar("val/dice", scores["dice"], self.display_epoch)
            writer.add_scalar("val/best_miou", self.best_miou, self.display_epoch)
            writer.add_scalar("val/best_miou_fg", self.best_miou_fg, self.display_epoch)

            # 유한한(finite) PSNR 값들만 필터링하여 평균 계산 (DDP 동기화 후 안전하게 필터링)
            valid_psnrs = psnrs[torch.isfinite(psnrs)]
            if len(valid_psnrs) > 0:
                avg_psnr = valid_psnrs.mean()
                writer.add_scalar("val/psnr", avg_psnr, self.display_epoch)

            _write_confusion_matrix_text(
                writer,
                "val/confusion_matrix",
                self.display_epoch,
                total_matrix,
                self.class_names,
            )
            for key, value in scores.items():
                if key.startswith("iou/"):
                    # x축을 에폭 단위로 기록
                    writer.add_scalar(f"val/{key}", value, self.display_epoch)

            # Worst Samples 그리드 로깅
            if self.worst_samples:
                worst_overlays = []
                for sample in self.worst_samples:
                    # GPU에서 직접 오버레이 생성 (CPU 이동 최소화)
                    img = sample["image"].to(self.device)
                    pred = sample["pred"].to(self.device)
                    worst_overlays.append(
                        overlay_mask_gpu(img, pred, self.palette_tensor)
                    )

                grid = make_image_grid_gpu(worst_overlays, columns=self.num_worst)
                writer.add_image(
                    "debug/worst_samples",
                    grid,
                    self.display_epoch,
                )

        self.validation_step_outputs.clear()
        self.worst_samples.clear()

    def on_fit_end(self):
        """학습 종료 시 모든 설정값과 최종 성능 지표를 HParams 탭에 기록합니다."""
        # 모든 rank가 작업을 마칠 때까지 대기하여 일관성 확보
        if self.trainer.world_size > 1:
            self.trainer.strategy.barrier()

        if self.global_rank == 0 and self.logger:
            # 1. 모든 설정값을 평면화 및 타입 정제 (TensorBoard 호환성 확보)
            hparams = _flatten_dict(self.cfg)
            hparams["num_classes"] = self.num_classes

            # TensorBoard hparams는 bool, str, float, int만 지원하므로 타입 체크
            hparams = {k: v if isinstance(v, (int, float, str, bool)) else str(v) for k, v in hparams.items()}

            # 2. 결과 지표 준비 및 값 정제
            val_loss = self.trainer.callback_metrics.get("val/loss", 0)
            val_miou = self.trainer.callback_metrics.get("val/miou", 0)
            val_miou_fg = self.trainer.callback_metrics.get("val/miou_fg", 0)
            val_dice = self.trainer.callback_metrics.get("val/dice", 0)
            best_miou = self.best_miou
            best_miou_fg = self.best_miou_fg

            def _to_float(val) -> float:
                if isinstance(val, torch.Tensor):
                    return float(val.item())
                return float(val)

            # Tensor/Scalar 타입을 파이썬 기본 float형으로 정제
            val_loss_f = _to_float(val_loss)
            val_miou_f = _to_float(val_miou)
            val_miou_fg_f = _to_float(val_miou_fg)
            val_dice_f = _to_float(val_dice)
            best_miou_f = _to_float(best_miou)
            best_miou_fg_f = _to_float(best_miou_fg)

            metrics = {
                "hp/val_loss": val_loss_f,
                "hp/val_miou": val_miou_f,
                "hp/val_miou_fg": val_miou_fg_f,
                "hp/val_dice": val_dice_f,
                "hp/val_best_miou": best_miou_f,
                "hp/val_best_miou_fg": best_miou_fg_f,
            }

            # 3. 텐서보드에 최종 기록 (우회 등록된 원본 메서드를 안전하게 호출)
            if hasattr(self.logger, "original_log_hyperparams"):
                self.logger.original_log_hyperparams(hparams, metrics)
            else:
                self.logger.log_hyperparams(hparams, metrics)

    def on_save_checkpoint(self, checkpoint: Dict[str, Any]) -> None:
        """PyTorch Lightning 체크포인트 저장 시 inference.py 호환 필드 추가.

        학습 시 사용된 메타 설정(cfg)을 패키징하고, 순수 PyTorch로 동작하는
        다운스트림 추론 노드(inference.py)에서 가중치 이름 불일치 크래시 없이
        곧바로 원샷 로드할 수 있도록 model 접두사를 정제한 model_state를 주입합니다.
        """
        # 1. 런타임 추론 노드(inference.py)가 모델 구조를 자급자족(Self-contained)하여 동적 빌드할 수 있도록 학습 config 주입
        checkpoint["config"] = self.cfg

        # 2. PyTorch Lightning 의존성 없이 순수 PyTorch 모델로 바로 로드할 수 있게 가중치 가공
        #    - LightningModule 내부에 선언된 원본 모델 변수명 'self.model = ...' 때문에
        #      가중치 키 이름 앞에 자동으로 붙는 'model.' 접두사(Prefix)를 전부 제거합니다.
        #      (예: 'model.conv1.weight' -> 'conv1.weight')
        #    - 이 가공을 거치지 않고 그대로 load_state_dict를 호출하면 키 불일치(Unexpected key) 런타임 에러가 발생합니다.
        state_dict = checkpoint.get("state_dict")
        if state_dict is not None:
            checkpoint["model_state"] = {
                (k[6:] if k.startswith("model.") else k): v
                for k, v in state_dict.items()
            }
        else:
            checkpoint["model_state"] = {}

    def configure_optimizers(self):
        lr = float(self.cfg.get("learning_rate", 1e-3))
        optimizer = torch.optim.AdamW(
            self.parameters(),
            lr=lr,
            weight_decay=float(self.cfg.get("weight_decay", 1e-4)),
        )

        scheduler_type = str(self.cfg.get("lr_scheduler", "none")).lower()
        warmup_enabled = bool(self.cfg.get("warmup_enabled", False))
        warmup_epochs = int(self.cfg.get("warmup_epochs", 0))

        # 1. 메인 스케줄러 결정 (워밍업 기간을 제외한 남은 에폭 동안만 코사인 곡선을 그리도록 설정)
        main_t_max = self.trainer.max_epochs
        if warmup_enabled and warmup_epochs > 0:
            main_t_max = max(1, self.trainer.max_epochs - warmup_epochs)

        if scheduler_type == "cosine":
            main_scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
                optimizer, T_max=main_t_max, eta_min=lr * 0.01
            )
        else:
            # 고정 학습률을 위한 스케줄러 (워밍업과 연결하기 위함)
            main_scheduler = torch.optim.lr_scheduler.ConstantLR(optimizer, factor=1.0)

        # 2. 워밍업 적용 여부에 따른 최종 스케줄러 구성
        if warmup_enabled and warmup_epochs > 0:
            start_factor = float(self.cfg.get("warmup_start_factor", 0.1))
            warmup_scheduler = torch.optim.lr_scheduler.LinearLR(
                optimizer, start_factor=start_factor, end_factor=1.0, total_iters=warmup_epochs
            )
            scheduler = torch.optim.lr_scheduler.SequentialLR(
                optimizer, schedulers=[warmup_scheduler, main_scheduler], milestones=[warmup_epochs]
            )
        else:
            # 워밍업이 비활성화된 경우
            if scheduler_type == "none":
                return optimizer  # 스케줄러 없이 완전 고정값 학습
            scheduler = main_scheduler

        return {
            "optimizer": optimizer,
            "lr_scheduler": {
                "scheduler": scheduler,
                "interval": "epoch",
            },
        }


class ImageLoggingCallback(L.Callback if L else object):
    """매 validation epoch마다 샘플 이미지를 TensorBoard에 로깅하는 콜백."""

    def __init__(self, debug_loaders, palette):
        super().__init__()
        self.debug_loaders = debug_loaders
        self.palette = palette

    def on_validation_epoch_end(self, trainer, pl_module):
        if pl_module.global_rank != 0:
            return

        epoch = trainer.current_epoch
        image_log_interval = int(pl_module.cfg.get("image_log_interval", 5))

        if epoch == 0 or (epoch + 1) % image_log_interval == 0:
            writer = trainer.logger.experiment
            write_debug_image_grids(
                writer,
                pl_module.display_epoch,
                pl_module.model,
                self.debug_loaders,
                pl_module.palette_tensor,
                pl_module.device,
                write_gt=(epoch == 0),
            )


SEGMENTATION_ROOT = Path(__file__).resolve().parent
SRC_ROOT = SEGMENTATION_ROOT.parent
DEFAULT_DATA_ROOT = SRC_ROOT / "carmaker_image" / "data"
DEFAULT_CONFIG = SEGMENTATION_ROOT / "config" / "segmentation_unet.yaml"


def parse_args() -> argparse.Namespace:
    """CLI argument를 정의한다.

    대부분의 실험 설정은 YAML config에 두고, 자주 바꾸는 값만 CLI로 덮어쓴다.
    """
    parser = argparse.ArgumentParser(
        description="Train U-Net segmentation on CarMaker raw/GT pairs."
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--data-root", default="")
    parser.add_argument("--manifest", default="")
    parser.add_argument(
        "--run-dir",
        default="",
        help="Explicit run directory (disables automatic naming)",
    )
    parser.add_argument(
        "--run-base", default="", help="Base directory for automatic run_dir naming"
    )
    parser.add_argument("--max-epochs", type=int, default=0)
    parser.add_argument("--batch-size", type=int, default=0)
    parser.add_argument("--num-workers", type=int, default=-1)
    parser.add_argument("--device", default="")
    parser.add_argument("--overfit-count", type=int, default=0)
    parser.add_argument("--tensorboard", action="store_true")
    parser.add_argument("--tensorboard-port", type=int, default=6006)
    parser.add_argument("--tensorboard-host", default="0.0.0.0")
    parser.add_argument("--profiler", default="")
    parser.add_argument(
        "--limit-batches",
        type=float,
        default=-1.0,
        help="limit_train_batches (float or int)",
    )
    parser.add_argument(
        "--accumulate", type=int, default=0, help="accumulate_grad_batches"
    )
    return parser.parse_args()


def load_config(path: str | Path) -> Dict[str, Any]:
    """YAML 또는 JSON config 파일을 읽어서 dict로 반환한다."""
    path = Path(path)
    if not path.exists():
        return {}
    if path.suffix.lower() == ".json":
        return json.loads(path.read_text(encoding="utf-8"))

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            "PyYAML is required for YAML config files. Install pyyaml or use JSON."
        ) from exc

    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def main() -> None:
    # RTX 3090 (Ampere) 이상의 GPU에서 Tensor Core를 활용하여 연산 속도를 최적화합니다.
    if torch.cuda.is_available():
        torch.set_float32_matmul_precision("high")

    args = parse_args()

    # 1) config 로드
    # config 안의 상대경로는 config 파일 위치 기준으로 해석한다.
    cfg = load_config(args.config)
    _resolve_config_paths(cfg, Path(args.config).expanduser().resolve().parent)
    _apply_cli_overrides(cfg, args)

    # 1.5) num_workers 최적화 (CPU 코어 및 GPU 개수 고려)
    _optimize_num_workers(cfg)

    _set_seed(int(cfg.get("seed", 42)))

    # 2) device / output directory 준비
    # CUDA가 가능하면 GPU를 쓰고, 아니면 CPU로 fallback한다.
    # Lightning에서는 accelerator='auto'로 설정하면 자동으로 선택한다.
    run_dir = _resolve_run_dir(
        cfg, explicit_run_dir=bool(args.run_dir), run_base=args.run_base
    )
    cfg["run_dir"] = str(run_dir)

    _maybe_start_tensorboard(args, run_dir)

    # 3) Adapter 생성
    adapter = CarmakerSegmentationAdapter(
        data_root=cfg.get("data_root", str(DEFAULT_DATA_ROOT)),
        manifest=cfg.get("manifest") or None,
        cameras=cfg.get("cameras"),
        use_raw_post_processed=bool(cfg.get("use_raw_post_processed", False)),
        use_mask_post_processed=True,
    )

    # 4) DataModule 및 LightningModule 생성
    data_module = SegmentationDataModule(cfg, adapter)
    # setup()을 미리 호출하여 데이터셋 정보를 가져온다.
    data_module.setup()
    rank_zero_info(data_module.split_report)

    model = SegmentationLightningModule(
        cfg=cfg,
        num_classes=adapter.num_classes,
        palette=adapter.palette,
        class_names=adapter.class_names,
    )

    # TensorBoard debug image용 loader.
    # Lightning Callback에서 사용할 수 있도록 준비한다.
    debug_image_count = int(cfg.get("debug_image_count", 4))
    debug_loaders = {
        "train": _make_debug_loader_by_camera_grid(
            data_module.train_dataset, debug_image_count, torch.device("cpu")
        ),
        "val": _make_debug_loader_by_camera_grid(
            data_module.val_dataset, debug_image_count, torch.device("cpu")
        ),
        "test": _make_debug_loader_by_camera_grid(
            data_module.test_dataset, debug_image_count, torch.device("cpu")
        ),
    }

    # 5) Lightning Trainer 설정
    # Multi-GPU 설정을 포함한다.
    logger = TensorBoardLogger(
        save_dir=str(run_dir.parent), name=run_dir.name, version=""
    )
    # PyTorch Lightning Trainer가 시작할 때 비어있는 metrics로 log_hyperparams를 호출하여
    # 텐서보드 HParams에 hp_metric만 등록되고 이후 호출이 무시되는 버그를 방지하기 위한 우회 처리
    logger.original_log_hyperparams = logger.log_hyperparams
    logger.log_hyperparams = lambda *args, **kwargs: None

    # TensorBoard Text 탭에 최종 config 기록 (마스터 프로세스만 수행)
    if int(os.environ.get("RANK", "0")) == 0:
        config_str = yaml.dump(cfg, default_flow_style=False, allow_unicode=True)
        logger.experiment.add_text("config", f"```yaml\n{config_str}\n```", 0)
        # 데이터셋 분할 리포트 기록
        if hasattr(data_module, "split_report"):
            logger.experiment.add_text("dataset/split", data_module.split_report, 0)

    # Best model checkpoint callback (checked at every validation epoch)
    best_checkpoint_callback = ModelCheckpoint(
        dirpath=run_dir / "checkpoints",
        filename="best",
        monitor="val/miou_fg",
        mode="max",
        save_top_k=1,
        save_last=True,
        every_n_epochs=1,
    )

    # Periodic checkpoint callback (saved every checkpoint_interval epochs)
    checkpoint_interval = int(cfg.get("checkpoint_interval", 1))
    periodic_checkpoint_callback = ModelCheckpoint(
        dirpath=run_dir / "checkpoints",
        filename="epoch_{epoch/display:03.0f}",
        auto_insert_metric_name=False,
        every_n_epochs=checkpoint_interval,
        save_top_k=-1,
    )

    lr_monitor = LearningRateMonitor(logging_interval="epoch")
    image_callback = ImageLoggingCallback(debug_loaders, adapter.palette)

    # 6) Trainer 실행
    t_cfg = cfg.get("trainer", {})
    profiler = t_cfg.get("profiler", cfg.get("profiler"))

    # 기본값 설정 (trainer 섹션 우선, 없으면 탑레벨 확인)
    max_epochs = int(
        t_cfg.get("max_epochs", cfg.get("max_epochs", cfg.get("epochs", 30)))
    )
    limit_batches = t_cfg.get(
        "limit_train_batches", cfg.get("limit_train_batches", 1.0)
    )
    log_every_n_steps = int(
        t_cfg.get("log_every_n_steps", cfg.get("log_every_n_steps", 50))
    )

    # 프로파일링 모드일 경우 오버라이드
    if profiler:
        p_cfg = t_cfg.get("on_profiling", {})
        max_epochs = int(p_cfg.get("max_epochs", cfg.get("profiler_max_epochs", 1)))
        limit_batches = p_cfg.get("limit_batches", cfg.get("profiler_limit_batches", 5))
        print(
            f"[profiler] enabled: limiting run to {max_epochs} epochs and {limit_batches} batches"
        )

    # Determine distributed settings before creating the Trainer
    world_size = int(os.environ.get("WORLD_SIZE", "1"))
    strategy = (
        DDPStrategy(find_unused_parameters=False)
        if (world_size > 1 or torch.cuda.device_count() > 1)
        else "auto"
    )

    trainer = L.Trainer(
        accelerator="gpu" if torch.cuda.is_available() else "cpu",
        devices="auto",
        strategy=strategy,
        max_epochs=max_epochs,
        limit_train_batches=limit_batches,
        accumulate_grad_batches=int(t_cfg.get("accumulate_grad_batches", 1)),
        profiler=profiler,
        logger=logger,
        callbacks=[best_checkpoint_callback, periodic_checkpoint_callback, lr_monitor, image_callback],
        precision="bf16-mixed" if torch.cuda.is_available() and torch.cuda.is_bf16_supported() else ("16-mixed" if torch.cuda.is_available() else 32),
        gradient_clip_val=t_cfg.get("gradient_clip_val", 1.0),
        gradient_clip_algorithm=t_cfg.get("gradient_clip_algorithm", "norm"),
        log_every_n_steps=log_every_n_steps,
    )

    rank_zero_info(
        f"[train] samples={len(data_module.train_dataset)} val={len(data_module.val_dataset)} "
        f"test={len(data_module.test_dataset)} run_dir={run_dir}"
    )

    # 7) 학습 시작
    trainer.fit(model, datamodule=data_module)

    # 8) 테스트 실행 (best checkpoint 로드)
    if data_module.test_dataset and len(data_module.test_dataset) > 0:
        trainer.test(model, datamodule=data_module, ckpt_path="best")

    print(f"[train] done run_dir={run_dir}")


def _make_debug_loader_by_camera_grid(
    dataset, debug_image_count: int, device: torch.device
):
    """TensorBoard 이미지 확인용 loader를 만든다.

    각 split에서 camera별 고정 샘플을 하나의 grid로 묶는다.
    """
    if len(dataset) == 0 or debug_image_count <= 0:
        return None

    indices_by_camera: Dict[str, list[int]] = defaultdict(list)
    for index in range(len(dataset)):
        camera = _debug_sample_camera(dataset, index)
        indices_by_camera[camera].append(index)

    cameras = sorted(camera for camera, indices in indices_by_camera.items() if indices)
    indices = []
    # 행(Row)을 카메라로, 열(Column)을 샘플 데이터로 구성하기 위해 루프 순서 변경
    for camera in cameras:
        camera_indices = indices_by_camera[camera]
        n_available = len(camera_indices)
        if n_available <= debug_image_count:
            indices.extend(camera_indices)
        else:
            # 전체 가용 샘플을 균등한 간격으로 추출하여 시나리오 전반의 다양성 확보
            step = n_available / debug_image_count
            for i in range(debug_image_count):
                indices.append(camera_indices[int(i * step)])

    if not indices:
        return None

    subset = torch.utils.data.Subset(dataset, indices)
    loader = DataLoader(
        subset,
        batch_size=1,
        shuffle=False,
        num_workers=0,
        pin_memory=device.type == "cuda",
    )
    return {"loader": loader, "columns": debug_image_count}


def _debug_sample_camera(dataset, index: int) -> str:
    """Return camera metadata for a dataset/subset item without loading images."""
    if isinstance(dataset, torch.utils.data.Subset):
        return _debug_sample_camera(dataset.dataset, int(dataset.indices[index]))

    samples = getattr(dataset, "samples", None)
    if samples is None or index >= len(samples):
        return "unknown"

    camera = getattr(samples[index], "camera", "") or "unknown"
    return str(camera)


@torch.no_grad()
def write_debug_image_grids(
    writer,
    epoch: int,
    model,
    loaders: Dict[str, Any],
    palette_tensor,
    device: torch.device,
    write_gt: bool = True,
) -> None:
    """TensorBoard Images 탭에 train/val/test별 overlay grid를 기록한다."""
    for split, spec in loaders.items():
        if not spec:
            continue
        _write_debug_image_grid(
            writer,
            epoch,
            model,
            spec["loader"],
            palette_tensor,
            device,
            split,
            columns=int(spec.get("columns", 2)),
            write_gt=write_gt,
        )


@torch.no_grad()
def _write_debug_image_grid(
    writer,
    epoch: int,
    model,
    loader,
    palette_tensor,
    device: torch.device,
    split: str,
    columns: int = 2,
    write_gt: bool = True,
) -> None:
    """한 split에서 고정 샘플 여러 장을 grid로 묶어 기록한다."""
    try:
        iterator = iter(loader)
    except TypeError:
        return

    model.eval()
    gt_overlays = []
    pred_overlays = []

    # Full-resolution images are large, so run TensorBoard debug samples one by one.
    # A 4-image batch can OOM even when the real training batch size is 1.
    for batch in iterator:
        image = batch["image"].to(device)
        try:
            pred = torch.argmax(model(image), dim=1)
        except RuntimeError as exc:
            if "out of memory" in str(exc).lower() and device.type == "cuda":
                torch.cuda.empty_cache()
                print(
                    f"[tensorboard] skipped debug/{split} image grid because CUDA ran out of memory"
                )
                return
            raise

        gt = batch["mask"].to(device)
        # GPU 상에서 오버레이 생성하여 속도 최적화
        gt_overlays.append(overlay_mask_gpu(image[0], gt[0], palette_tensor))
        pred_overlays.append(overlay_mask_gpu(image[0], pred[0], palette_tensor))

    if gt_overlays:
        if write_gt:
            grid_gt = make_image_grid_gpu(gt_overlays, columns=columns)
            writer.add_image(f"debug/{split}/gt_grid", grid_gt, epoch)

        grid_pred = make_image_grid_gpu(pred_overlays, columns=columns)
        writer.add_image(f"debug/{split}/pred_grid", grid_pred, epoch)


def _maybe_start_tensorboard(
    args: argparse.Namespace, run_dir: Path
) -> subprocess.Popen | None:
    """Start TensorBoard for this run when requested."""
    if not args.tensorboard:
        return None
    if importlib.util.find_spec("tensorboard") is None:
        print("[warn] TensorBoard is not installed. Run: uv pip install tensorboard")
        return None

    command = [
        sys.executable,
        "-m",
        "tensorboard.main",
        "--logdir",
        str(run_dir),
        "--host",
        str(args.tensorboard_host),
        "--port",
        str(args.tensorboard_port),
    ]
    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
    except OSError as exc:
        print(f"[warn] Could not start TensorBoard: {exc}")
        return None

    atexit.register(_terminate_process, process)
    print(
        f"[tensorboard] http://{args.tensorboard_host}:{args.tensorboard_port} "
        f"logdir={run_dir}"
    )
    return process


def _terminate_process(process: subprocess.Popen) -> None:
    """종료 시 백그라운드 프로세스를 정리한다."""
    if process.poll() is None:
        process.terminate()


def _resolve_run_dir(
    cfg: Dict[str, Any], explicit_run_dir: bool = False, run_base: str = ""
) -> Path:
    """실험별 run directory를 결정한다.

    --run-dir를 직접 주면 해당 폴더를 그대로 사용한다.
    --run-base가 있으면 이를 부모로 보고 그 아래 {loss}_ep{epochs}_{timestamp} 폴더를 자동 생성한다.
    config의 run_dir는 base directory로 보고, 그 아래 {loss}_ep{epochs}_{timestamp} 폴더를 자동 생성한다.
    """
    if explicit_run_dir:
        base_dir = Path(cfg.get("run_dir")).expanduser().resolve()
        base_dir.mkdir(parents=True, exist_ok=True)
        return base_dir

    if run_base:
        base_dir = Path(run_base).expanduser().resolve()
    else:
        base_dir = (
            Path(cfg.get("run_dir", SEGMENTATION_ROOT / "runs")).expanduser().resolve()
        )

    loss_name = _run_name_part(str(cfg.get("loss", "loss")))
    t_cfg = cfg.get("trainer", {})
    epochs = int(t_cfg.get("max_epochs", cfg.get("max_epochs", cfg.get("epochs", 30))))
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = base_dir / f"{loss_name}_ep{epochs}_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def _run_name_part(value: str) -> str:
    """Make a config value safe and readable as one path segment."""
    value = value.strip().lower()
    value = re.sub(r"\s+", "-", value)
    value = re.sub(r"[^a-z0-9_.+-]+", "-", value)
    return value.strip("-") or "unknown"


def _write_confusion_matrix_text(
    writer, tag: str, epoch: int, matrix, class_names
) -> None:
    """TensorBoard Text 탭에 confusion matrix를 markdown table로 기록한다."""
    if not writer or matrix is None:
        return

    if isinstance(matrix, torch.Tensor):
        values = matrix.detach().cpu().to(torch.int64).tolist()
    else:
        values = matrix

    labels = [str(name) for name in class_names]
    if len(labels) != len(values):
        labels = [f"class_{idx}" for idx in range(len(values))]

    header = "| GT \\ Pred | " + " | ".join(labels) + " |"
    separator = "|---|" + "|".join("---" for _ in labels) + "|"
    rows = [header, separator]
    for label, row in zip(labels, values):
        rows.append(
            "| " + label + " | " + " | ".join(str(int(value)) for value in row) + " |"
        )

    writer.add_text(tag, "\n".join(rows), epoch)


def _apply_cli_overrides(cfg: Dict[str, Any], args: argparse.Namespace) -> None:
    """CLI 인자가 제공되면 config 값을 덮어쓴다."""
    if args.data_root:
        cfg["data_root"] = args.data_root
    else:
        cfg.setdefault("data_root", str(DEFAULT_DATA_ROOT))

    if args.manifest:
        cfg["manifest"] = args.manifest
    if args.run_dir:
        cfg["run_dir"] = args.run_dir
    if args.max_epochs > 0:
        cfg.setdefault("trainer", {})["max_epochs"] = args.max_epochs
    if args.batch_size > 0:
        cfg["batch_size"] = args.batch_size
    if args.num_workers >= 0:
        cfg["num_workers"] = args.num_workers
    if args.device and args.device.lower() != "auto":
        cfg["device"] = args.device
    if args.profiler:
        cfg.setdefault("trainer", {})["profiler"] = args.profiler
    if args.limit_batches >= 0:
        cfg.setdefault("trainer", {})["limit_train_batches"] = args.limit_batches
    if args.accumulate > 0:
        cfg.setdefault("trainer", {})["accumulate_grad_batches"] = args.accumulate


def _resolve_config_paths(cfg: Dict[str, Any], base_dir: Path) -> None:
    """config 내부 상대경로를 config 파일 위치 기준의 절대경로로 바꾼다."""
    for key in ("data_root", "manifest", "run_dir"):
        value = cfg.get(key)
        if not value:
            continue

        path = Path(str(value)).expanduser()
        if not path.is_absolute():
            cfg[key] = str((base_dir / path).resolve())


def _set_seed(seed: int) -> None:
    """실험 재현성을 위해 Python / Numpy / PyTorch seed를 고정한다."""
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def _optimize_num_workers(cfg: Dict[str, Any]) -> None:
    """CPU 코어 수와 가용한 GPU 개수를 기반으로 DataLoader worker 수를 최적화한다."""
    requested_workers = int(cfg.get("num_workers", -1))

    # 사용자가 명시적으로 설정하지 않았거나 -1인 경우 자동 계산
    if requested_workers < 0:
        # 1. 실제 프로세스에 할당된(Affinity) 코어 수 확인 (Linux 전용, 가장 정확함)
        # 2. Slurm 환경 변수 확인 (SLURM_CPUS_PER_TASK)
        # 3. Fallback: os.cpu_count()
        try:
            cpu_count = len(os.sched_getaffinity(0))
        except (AttributeError, NotImplementedError):
            cpu_count = int(os.environ.get("SLURM_CPUS_PER_TASK", os.cpu_count() or 4))

        gpu_count = torch.cuda.device_count() if torch.cuda.is_available() else 1

        # GPU당 코어 배분 (일반적으로 GPU당 2~4개가 적당함)
        # 너무 많으면 I/O 오버헤드, 너무 적으면 GPU 대기 발생
        optimized = max(2, min(cpu_count // gpu_count, 8))
        cfg["num_workers"] = optimized
        rank_zero_info(
            f"[config] optimized num_workers to {optimized} (cpu_allocated={cpu_count}, gpu={gpu_count})"
        )


def _flatten_dict(d: Dict[str, Any], parent_key: str = "", sep: str = "/") -> Dict[str, Any]:
    """중첩된 dict를 텐서보드 기록용으로 평면화한다."""
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(_flatten_dict(v, new_key, sep=sep).items())
        elif isinstance(v, (list, tuple)):
            items.append((new_key, str(v)))
        else:
            items.append((new_key, v))
    return dict(items)


if __name__ == "__main__":
    main()
