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
except ImportError:
    # `python3 src/segmentation/train.py`처럼 파일을 직접 실행할 때 사용하는 fallback import.
    from adapters import CarmakerSegmentationAdapter
    from dataset import SegmentationDataset, split_dataset
    from losses import build_loss
    from metrics import confusion_matrix, mask_psnr, segmentation_scores
    from models import build_model
    from utils.visualization import make_image_grid, overlay_mask

try:
    import lightning.pytorch as L
    from lightning.pytorch.callbacks import LearningRateMonitor, ModelCheckpoint
    from lightning.pytorch.loggers import TensorBoardLogger
    from lightning.pytorch.strategies import DDPStrategy
except ImportError:
    try:
        import pytorch_lightning as L
        from pytorch_lightning.callbacks import LearningRateMonitor, ModelCheckpoint
        from pytorch_lightning.loggers import TensorBoardLogger
        from pytorch_lightning.strategies import DDPStrategy
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
        self.train_dataset, self.val_dataset, self.test_dataset = split_dataset(
            full_dataset,
            val_ratio=float(self.cfg.get("val_ratio", 0.2)),
            seed=int(self.cfg.get("seed", 42)),
            test_ratio=float(self.cfg.get("test_ratio", 0.1)),
            stratify_by_camera=bool(self.cfg.get("stratify_by_camera", True)),
        )

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

        # Multi-GPU 환경에서 Confusion Matrix 동기화를 위해 버퍼로 관리할 수 있으나,
        # Lightning에서는 validation_step_outputs를 모아서 처리하는 것이 일반적이다.
        self.validation_step_outputs = []
        self.test_step_outputs = []

    def forward(self, x):
        return self.model(x)

    def training_step(self, batch, batch_idx):
        image = batch["image"]
        mask = batch["mask"]
        logits = self(image)
        loss = self.criterion(logits, mask)

        self.log(
            "train/loss",
            loss,
            on_step=True,
            on_epoch=True,
            prog_bar=True,
            sync_dist=True,
            batch_size=image.shape[0],
        )
        return loss

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

        self.log(
            "val/loss",
            loss,
            on_epoch=True,
            prog_bar=True,
            sync_dist=True,
            batch_size=image.shape[0],
        )
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

        if self.global_rank == 0:
            writer = self.logger.experiment
            _write_confusion_matrix_text(
                writer,
                "test/confusion_matrix",
                self.current_epoch,
                total_matrix,
                self.class_names,
            )

        self.test_step_outputs.clear()

    def on_validation_epoch_end(self):
        if not self.validation_step_outputs:
            return

        all_matrices = torch.stack([x["matrix"] for x in self.validation_step_outputs])
        total_matrix = torch.sum(all_matrices, dim=0)

        if self.trainer.world_size > 1:
            total_matrix = self.all_gather(total_matrix).sum(dim=0)

        scores = segmentation_scores(total_matrix.cpu())

        self.log("val/miou", scores["miou"], prog_bar=True, sync_dist=False)
        self.log("val/dice", scores["dice"], prog_bar=True, sync_dist=False)

        psnrs = [
            x["psnr"] for x in self.validation_step_outputs if np.isfinite(x["psnr"])
        ]
        if psnrs:
            avg_psnr = torch.tensor(psnrs).mean()
            self.log("val/psnr", avg_psnr, sync_dist=False)

        if self.global_rank == 0:
            writer = self.logger.experiment
            _write_confusion_matrix_text(
                writer,
                "val/confusion_matrix",
                self.current_epoch,
                total_matrix,
                self.class_names,
            )
            for key, value in scores.items():
                if key.startswith("iou/"):
                    writer.add_scalar(f"val/{key}", value, self.current_epoch)

        self.validation_step_outputs.clear()

    def configure_optimizers(self):
        optimizer = torch.optim.AdamW(
            self.parameters(),
            lr=float(self.cfg.get("learning_rate", 1e-3)),
            weight_decay=float(self.cfg.get("weight_decay", 1e-4)),
        )

        scheduler_type = str(self.cfg.get("lr_scheduler", "none")).lower()
        if scheduler_type == "cosine":
            scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
                optimizer,
                T_max=self.trainer.max_epochs,
                eta_min=float(self.cfg.get("learning_rate", 1e-3)) * 0.01,
            )
            return {
                "optimizer": optimizer,
                "lr_scheduler": {
                    "scheduler": scheduler,
                    "interval": "epoch",
                },
            }

        return optimizer


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
                epoch + 1,
                pl_module.model,
                self.debug_loaders,
                self.palette,
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
    args = parse_args()

    # 1) config 로드
    # config 안의 상대경로는 config 파일 위치 기준으로 해석한다.
    cfg = load_config(args.config)
    _resolve_config_paths(cfg, Path(args.config).expanduser().resolve().parent)
    _apply_cli_overrides(cfg, args)
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

    # TensorBoard Text 탭에 최종 config 기록
    config_str = yaml.dump(cfg, default_flow_style=False, allow_unicode=True)
    logger.experiment.add_text("config", f"```yaml\n{config_str}\n```", 0)

    checkpoint_callback = ModelCheckpoint(
        dirpath=run_dir / "checkpoints",
        filename="best",
        monitor="val/miou",
        mode="max",
        save_last=True,
        every_n_epochs=1,
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

    trainer = L.Trainer(
        accelerator="gpu" if torch.cuda.is_available() else "cpu",
        devices="auto",
        strategy=DDPStrategy(find_unused_parameters=False)
        if torch.cuda.device_count() > 1
        else "auto",
        max_epochs=max_epochs,
        limit_train_batches=limit_batches,
        accumulate_grad_batches=int(t_cfg.get("accumulate_grad_batches", 1)),
        profiler=profiler,
        logger=logger,
        callbacks=[checkpoint_callback, lr_monitor, image_callback],
        precision="16-mixed" if torch.cuda.is_available() else 32,  # 혼합 정밀도 학습
        log_every_n_steps=log_every_n_steps,
    )

    print(
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
    for sample_index in range(debug_image_count):
        for camera in cameras:
            camera_indices = indices_by_camera[camera]
            if sample_index < len(camera_indices):
                indices.append(camera_indices[sample_index])

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
    return {"loader": loader, "columns": max(len(cameras), 1)}


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
    palette,
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
            palette,
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
    palette,
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
            pred = torch.argmax(model(image), dim=1).cpu()
        except RuntimeError as exc:
            if "out of memory" in str(exc).lower() and device.type == "cuda":
                torch.cuda.empty_cache()
                print(
                    f"[tensorboard] skipped debug/{split} image grid because CUDA ran out of memory"
                )
                return
            raise

        gt = batch["mask"].cpu()
        image_cpu = batch["image"].cpu()
        gt_overlays.append(overlay_mask(image_cpu[0], gt[0], palette))
        pred_overlays.append(overlay_mask(image_cpu[0], pred[0], palette))

    if gt_overlays:
        if write_gt:
            writer.add_image(
                f"debug/{split}/gt_grid",
                make_image_grid(gt_overlays, columns=columns),
                epoch,
                dataformats="HWC",
            )
        writer.add_image(
            f"debug/{split}/pred_grid",
            make_image_grid(pred_overlays, columns=columns),
            epoch,
            dataformats="HWC",
        )


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
    run_dir.mkdir(parents=True, exist_ok=False)
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


if __name__ == "__main__":
    main()
