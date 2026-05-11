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
import subprocess
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

import numpy as np
import torch
from torch.utils.data import DataLoader

try:
    from tqdm.auto import tqdm
except Exception:
    tqdm = None

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


SEGMENTATION_ROOT = Path(__file__).resolve().parent
SRC_ROOT = SEGMENTATION_ROOT.parent
DEFAULT_DATA_ROOT = SRC_ROOT / "carmaker_image" / "data"
DEFAULT_CONFIG = SEGMENTATION_ROOT / "config" / "segmentation_unet.yaml"


def parse_args() -> argparse.Namespace:
    """CLI argument를 정의한다.

    대부분의 실험 설정은 YAML config에 두고, 자주 바꾸는 값만 CLI로 덮어쓴다.
    """
    parser = argparse.ArgumentParser(description="Train U-Net segmentation on CarMaker raw/GT pairs.")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--data-root", default="")
    parser.add_argument("--manifest", default="")
    parser.add_argument("--run-dir", default="")
    parser.add_argument("--epochs", type=int, default=0)
    parser.add_argument("--batch-size", type=int, default=0)
    parser.add_argument("--num-workers", type=int, default=-1)
    parser.add_argument("--device", default="")
    parser.add_argument("--overfit-count", type=int, default=0)
    parser.add_argument("--tensorboard", action="store_true")
    parser.add_argument("--tensorboard-port", type=int, default=6006)
    parser.add_argument("--tensorboard-host", default="0.0.0.0")
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
        raise RuntimeError("PyYAML is required for YAML config files. Install pyyaml or use JSON.") from exc

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
    device = torch.device(cfg.get("device") or ("cuda" if torch.cuda.is_available() else "cpu"))
    run_dir = _resolve_run_dir(cfg, explicit_run_dir=bool(args.run_dir))
    cfg["run_dir"] = str(run_dir)
    checkpoint_dir = run_dir / "checkpoints"
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    _maybe_start_tensorboard(args, run_dir)

    # 3) Adapter 생성
    # Adapter는 CSV와 폴더 구조를 읽어서 image_path / mask_path pair 목록을 만든다.
    adapter = CarmakerSegmentationAdapter(
        data_root=cfg.get("data_root", str(DEFAULT_DATA_ROOT)),
        manifest=cfg.get("manifest") or None,
        cameras=cfg.get("cameras"),
        use_raw_post_processed=bool(cfg.get("use_raw_post_processed", False)),
        use_mask_post_processed=True,
    )

    # 4) Dataset 생성
    # Dataset은 실제 PNG를 읽고 image tensor [3,H,W], mask tensor [H,W]로 변환한다.
    dataset = SegmentationDataset(
        adapter=adapter,
        image_size=tuple(cfg.get("image_size", [512, 512])),
    )

    # overfit smoke test:
    # 전체 dataset에서 앞 N개만 사용해서 모델이 작은 데이터라도 외울 수 있는지 확인한다.
    if args.overfit_count > 0:
        dataset.samples = dataset.samples[: args.overfit_count]

    if len(dataset) == 0:
        raise RuntimeError(
            "No training samples found. Run extract_bag_images.py and apply_mask.py first, "
            "or pass --data-root/--manifest explicitly."
        )

    # 5) train / validation / test split
    # seed를 고정해서 매번 같은 split이 나오게 한다.
    train_dataset, val_dataset, test_dataset = split_dataset(
        dataset,
        val_ratio=float(cfg.get("val_ratio", 0.2)),
        seed=int(cfg.get("seed", 42)),
        test_ratio=float(cfg.get("test_ratio", 0.1)),
        stratify_by_camera=bool(cfg.get("stratify_by_camera", True)),
    )

    # 6) DataLoader 생성
    # DataLoader는 Dataset에서 sample을 가져와 batch 형태로 묶어준다.
    train_loader = DataLoader(
        train_dataset,
        batch_size=int(cfg.get("batch_size", 4)),
        shuffle=True,
        num_workers=int(cfg.get("num_workers", 2)),
        pin_memory=device.type == "cuda",
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=int(cfg.get("batch_size", 4)),
        shuffle=False,
        num_workers=int(cfg.get("num_workers", 2)),
        pin_memory=device.type == "cuda",
    )
    test_loader = DataLoader(
        test_dataset,
        batch_size=int(cfg.get("batch_size", 4)),
        shuffle=False,
        num_workers=int(cfg.get("num_workers", 2)),
        pin_memory=device.type == "cuda",
    )

    # TensorBoard debug image용 loader.
    # 학습용 train_loader는 shuffle=True지만, debug 이미지는 고정 샘플을 봐야 epoch별 변화가 보인다.
    debug_image_count = int(cfg.get("debug_image_count", 4))
    debug_loaders = {
        "train": _make_debug_loader(train_dataset, debug_image_count, device),
        "val": _make_debug_loader(val_dataset, debug_image_count, device),
        "test": _make_debug_loader(test_dataset, debug_image_count, device),
    }

    # 7) model / loss / optimizer 생성
    # build_model과 build_loss를 쓰면 config만 바꿔 모델과 loss를 교체할 수 있다.
    model = build_model(cfg, num_classes=adapter.num_classes).to(device)
    criterion = build_loss(
        name=str(cfg.get("loss", "cross_entropy")),
        num_classes=adapter.num_classes,
        class_weights=cfg.get("class_weights"),
        device=device,
    )
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=float(cfg.get("learning_rate", 1e-3)),
        weight_decay=float(cfg.get("weight_decay", 1e-4)),
    )

    # 8) TensorBoard writer / checkpoint 기준값 준비
    writer = _make_writer(run_dir)
    _write_run_metadata(writer, cfg, adapter.num_classes, device)
    best_miou = -1.0
    epochs = int(cfg.get("epochs", 30))
    print(
        f"[train] samples={len(train_dataset)} val={len(val_dataset)} "
        f"test={len(test_dataset)} device={device} run_dir={run_dir}"
    )

    # 9) epoch loop
    epoch_bar = _progress(range(1, epochs + 1), desc="epochs", total=epochs)
    for epoch in epoch_bar:
        train_loss = train_one_epoch(model, train_loader, criterion, optimizer, device, epoch)
        val = validate(model, val_loader, criterion, adapter.num_classes, device, epoch)
        _set_progress_postfix(
            epoch_bar,
            train_loss=f"{train_loss:.4f}",
            val_loss=f"{val['loss']:.4f}",
            miou=f"{val['miou']:.4f}",
            dice=f"{val['dice']:.4f}",
            acc=f"{val['pixel_accuracy']:.4f}",
        )

        # TensorBoard Scalars에 loss/metric을 기록한다.
        _write_scalars(writer, epoch, train_loss, val, optimizer)

        # TensorBoard Images에 train/val/test별 GT/pred overlay grid를 기록한다.
        if writer and (epoch == 1 or epoch % int(cfg.get("image_log_interval", 5)) == 0):
            write_debug_image_grids(writer, epoch, model, debug_loaders, adapter.palette, device)

        # validation mIoU가 최고일 때 best checkpoint를 갱신한다.
        if val["miou"] > best_miou:
            best_miou = val["miou"]
            save_checkpoint(checkpoint_dir / "best.pt", model, optimizer, epoch, cfg, best_miou)

        # 긴 학습 중간에도 복구 지점을 남기기 위한 주기적 checkpoint.
        if epoch % int(cfg.get("checkpoint_interval", 10)) == 0:
            save_checkpoint(checkpoint_dir / f"epoch_{epoch:03d}.pt", model, optimizer, epoch, cfg, best_miou)

    # 마지막 epoch 상태도 항상 저장한다.
    save_checkpoint(checkpoint_dir / "last.pt", model, optimizer, epochs, cfg, best_miou)
    if len(test_dataset) > 0:
        best_checkpoint = checkpoint_dir / "best.pt"
        if best_checkpoint.exists():
            checkpoint = _load_checkpoint(best_checkpoint, device)
            model.load_state_dict(checkpoint["model_state"])
        test = validate(model, test_loader, criterion, adapter.num_classes, device, split="test")
        _write_eval_scalars(writer, "test", epochs, test)
        print(
            f"[test] loss={test['loss']:.4f} miou={test['miou']:.4f} "
            f"dice={test['dice']:.4f} acc={test['pixel_accuracy']:.4f}"
        )
    if writer:
        writer.close()
    print(f"[train] done best_miou={best_miou:.4f} run_dir={run_dir}")


def train_one_epoch(model, loader, criterion, optimizer, device: torch.device, epoch: int = 0) -> float:
    """train_loader를 한 번 순회하며 model weight를 업데이트한다."""
    model.train()
    total_loss = 0.0
    total_items = 0

    progress = _progress(loader, desc=f"train {epoch:03d}" if epoch else "train", leave=False)
    for batch in progress:
        # DataLoader batch 형식:
        # image: [B, 3, H, W], mask: [B, H, W]
        image = batch["image"].to(device, non_blocking=True)
        mask = batch["mask"].to(device, non_blocking=True)

        optimizer.zero_grad(set_to_none=True)

        # logits: [B, C, H, W]
        # CrossEntropyLoss target은 class id mask [B, H, W] 형태를 기대한다.
        logits = model(image)
        loss = criterion(logits, mask)

        loss.backward()
        optimizer.step()

        batch_size = image.shape[0]
        total_loss += float(loss.item()) * batch_size
        total_items += batch_size
        _set_progress_postfix(progress, loss=f"{total_loss / max(total_items, 1):.4f}")

    return total_loss / max(total_items, 1)


@torch.no_grad()
def validate(
    model,
    loader,
    criterion,
    num_classes: int,
    device: torch.device,
    epoch: int = 0,
    split: str = "val",
) -> Dict[str, Any]:
    """validation set에서 loss와 segmentation metric을 계산한다."""
    model.eval()
    total_loss = 0.0
    total_items = 0
    matrix = torch.zeros((num_classes, num_classes), dtype=torch.int64, device=device)
    psnr_values = []
    camera_matrices = defaultdict(lambda: torch.zeros((num_classes, num_classes), dtype=torch.int64, device=device))
    camera_loss_totals = defaultdict(float)
    camera_items = defaultdict(int)
    camera_psnr_values = defaultdict(list)

    progress = _progress(loader, desc=f"{split} {epoch:03d}" if epoch else split, leave=False)
    for batch in progress:
        image = batch["image"].to(device, non_blocking=True)
        mask = batch["mask"].to(device, non_blocking=True)

        logits = model(image)
        loss = criterion(logits, mask)

        # 각 픽셀에서 score가 가장 높은 class를 prediction으로 선택한다.
        pred = torch.argmax(logits, dim=1)

        # confusion matrix를 누적해 mIoU / Dice / Accuracy를 안정적으로 계산한다.
        matrix += confusion_matrix(pred, mask, num_classes).to(device)
        psnr_values.append(mask_psnr(pred, mask, max_value=float(num_classes - 1)))

        cameras = [str(camera) or "unknown" for camera in batch.get("camera", [])]
        if cameras:
            for camera in sorted(set(cameras)):
                sample_indices = [idx for idx, value in enumerate(cameras) if value == camera]
                index_tensor = torch.tensor(sample_indices, dtype=torch.long, device=device)
                camera_logits = logits.index_select(0, index_tensor)
                camera_mask = mask.index_select(0, index_tensor)
                camera_pred = pred.index_select(0, index_tensor)
                camera_loss = criterion(camera_logits, camera_mask)
                camera_count = len(sample_indices)

                camera_matrices[camera] += confusion_matrix(camera_pred, camera_mask, num_classes).to(device)
                camera_loss_totals[camera] += float(camera_loss.item()) * camera_count
                camera_items[camera] += camera_count
                camera_psnr_values[camera].append(
                    mask_psnr(camera_pred, camera_mask, max_value=float(num_classes - 1))
                )

        batch_size = image.shape[0]
        total_loss += float(loss.item()) * batch_size
        total_items += batch_size
        _set_progress_postfix(progress, loss=f"{total_loss / max(total_items, 1):.4f}")

    scores = segmentation_scores(matrix.cpu())
    scores["loss"] = total_loss / max(total_items, 1)

    # pred와 GT가 완전히 같으면 PSNR은 inf가 될 수 있으므로 평균에서는 finite 값만 사용한다.
    finite_psnr = [v for v in psnr_values if np.isfinite(v)]
    scores["psnr"] = float(np.mean(finite_psnr)) if finite_psnr else float("inf")
    scores["by_camera"] = _camera_scores(camera_matrices, camera_loss_totals, camera_items, camera_psnr_values)
    return scores


def _camera_scores(camera_matrices, camera_loss_totals, camera_items, camera_psnr_values) -> Dict[str, Dict[str, float]]:
    """Build metric dictionaries for each camera."""
    by_camera: Dict[str, Dict[str, float]] = {}
    for camera in sorted(camera_matrices.keys()):
        scores = segmentation_scores(camera_matrices[camera].cpu())
        scores["loss"] = camera_loss_totals[camera] / max(camera_items[camera], 1)
        finite_psnr = [value for value in camera_psnr_values[camera] if np.isfinite(value)]
        scores["psnr"] = float(np.mean(finite_psnr)) if finite_psnr else float("inf")
        by_camera[camera] = scores
    return by_camera


def _progress(iterable, **kwargs):
    """Return a tqdm progress bar when available, otherwise the original iterable."""
    if tqdm is None:
        return iterable
    kwargs.setdefault("dynamic_ncols", True)
    return tqdm(iterable, **kwargs)


def _set_progress_postfix(progress, **values) -> None:
    """Update tqdm postfix only when the iterable is actually a tqdm instance."""
    if hasattr(progress, "set_postfix"):
        progress.set_postfix(**values)


def _make_debug_loader(dataset, debug_image_count: int, device: torch.device):
    """TensorBoard 이미지 확인용 loader를 만든다.

    각 split에서 앞쪽 고정 샘플 최대 debug_image_count개만 사용한다.
    """
    if len(dataset) == 0 or debug_image_count <= 0:
        return None

    count = min(debug_image_count, len(dataset))
    subset = torch.utils.data.Subset(dataset, list(range(count)))
    return DataLoader(
        subset,
        batch_size=count,
        shuffle=False,
        num_workers=0,
        pin_memory=device.type == "cuda",
    )


@torch.no_grad()
def write_debug_image_grids(writer, epoch: int, model, loaders: Dict[str, Any], palette, device: torch.device) -> None:
    """TensorBoard Images 탭에 train/val/test별 2x2 overlay grid를 기록한다."""
    for split, loader in loaders.items():
        if loader is None:
            continue
        _write_debug_image_grid(writer, epoch, model, loader, palette, device, split)


@torch.no_grad()
def _write_debug_image_grid(writer, epoch: int, model, loader, palette, device: torch.device, split: str) -> None:
    """한 split에서 고정 샘플 여러 장을 grid로 묶어 기록한다."""
    try:
        batch = next(iter(loader))
    except StopIteration:
        return

    model.eval()
    image = batch["image"].to(device)
    pred = torch.argmax(model(image), dim=1).cpu()
    gt = batch["mask"].cpu()
    image_cpu = batch["image"].cpu()
    max_items = min(4, image_cpu.shape[0])

    gt_overlays = []
    pred_overlays = []
    for idx in range(max_items):
        gt_overlays.append(overlay_mask(image_cpu[idx], gt[idx], palette))
        pred_overlays.append(overlay_mask(image_cpu[idx], pred[idx], palette))

    writer.add_image(f"debug/{split}/gt_grid", make_image_grid(gt_overlays, columns=2), epoch, dataformats="HWC")
    writer.add_image(f"debug/{split}/pred_grid", make_image_grid(pred_overlays, columns=2), epoch, dataformats="HWC")


def save_checkpoint(path: Path, model, optimizer, epoch: int, cfg: Dict[str, Any], best_miou: float) -> None:
    """model/optimizer/config를 checkpoint 파일로 저장한다."""
    torch.save(
        {
            "epoch": epoch,
            "model_state": model.state_dict(),
            "optimizer_state": optimizer.state_dict(),
            "config": cfg,
            "best_miou": best_miou,
        },
        path,
    )


def _load_checkpoint(path: Path, device: torch.device) -> Dict[str, Any]:
    """Load a checkpoint across PyTorch versions."""
    try:
        return torch.load(path, map_location=device, weights_only=False)
    except TypeError:
        return torch.load(path, map_location=device)


def _make_writer(run_dir: Path):
    """TensorBoard SummaryWriter를 만든다. tensorboard가 없으면 학습은 계속 진행한다."""
    try:
        from torch.utils.tensorboard import SummaryWriter
    except Exception as exc:
        print(f"[warn] TensorBoard disabled: {exc}")
        return None
    return SummaryWriter(log_dir=str(run_dir))


def _maybe_start_tensorboard(args: argparse.Namespace, run_dir: Path) -> subprocess.Popen | None:
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


def _resolve_run_dir(cfg: Dict[str, Any], explicit_run_dir: bool = False) -> Path:
    """실험별 run directory를 결정한다.

    --run-dir를 직접 주면 해당 폴더를 그대로 사용한다.
    config의 run_dir는 base directory로 보고, 그 아래 timestamp/model/loss 기반 폴더를 자동 생성한다.
    """
    base_dir = Path(cfg.get("run_dir", SEGMENTATION_ROOT / "runs")).expanduser().resolve()
    if explicit_run_dir:
        base_dir.mkdir(parents=True, exist_ok=True)
        return base_dir

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = base_dir / timestamp
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def _write_run_metadata(writer, cfg: Dict[str, Any], num_classes: int, device: torch.device) -> None:
    """TensorBoard에 run 설정을 text/hparams 형태로 기록한다."""
    if not writer:
        return

    model_cfg = cfg.get("model", {})
    model_name = model_cfg.get("name", "model") if isinstance(model_cfg, dict) else str(model_cfg)
    base_channels = model_cfg.get("base_channels", "") if isinstance(model_cfg, dict) else ""
    image_size = cfg.get("image_size", "")
    loss_name = str(cfg.get("loss", ""))

    metadata_lines = [
        f"- model: `{model_name}`",
        f"- image_size: `{image_size}`",
        f"- loss: `{loss_name}`",
        f"- base_channels: `{base_channels}`",
        f"- batch_size: `{cfg.get('batch_size', '')}`",
        f"- learning_rate: `{cfg.get('learning_rate', '')}`",
        f"- weight_decay: `{cfg.get('weight_decay', '')}`",
        f"- num_classes: `{num_classes}`",
        f"- device: `{device}`",
        f"- data_root: `{cfg.get('data_root', '')}`",
        f"- manifest: `{cfg.get('manifest', '')}`",
    ]
    writer.add_text("run/config", "\n".join(metadata_lines), 0)

    # HParams 탭에서도 핵심 설정을 빠르게 비교할 수 있게 남긴다.
    hparams = {
        "model": str(model_name),
        "image_size": str(image_size),
        "loss": loss_name,
        "base_channels": str(base_channels),
        "batch_size": str(cfg.get("batch_size", "")),
        "learning_rate": str(cfg.get("learning_rate", "")),
        "weight_decay": str(cfg.get("weight_decay", "")),
    }
    writer.add_hparams(hparams, {"hparam/placeholder": 0.0})


def _terminate_process(process: subprocess.Popen) -> None:
    if process.poll() is None:
        process.terminate()


def _write_scalars(writer, epoch: int, train_loss: float, val: Dict[str, float], optimizer) -> None:
    """TensorBoard Scalars 탭에 정량 지표를 기록한다."""
    if not writer:
        return

    writer.add_scalar("train/loss", train_loss, epoch)
    writer.add_scalar("val/loss", val["loss"], epoch)
    writer.add_scalar("val/miou", val["miou"], epoch)
    writer.add_scalar("val/dice", val["dice"], epoch)
    writer.add_scalar("val/pixel_accuracy", val["pixel_accuracy"], epoch)
    writer.add_scalar("val/psnr", val["psnr"], epoch)
    writer.add_scalar("train/lr", optimizer.param_groups[0]["lr"], epoch)

    # class별 IoU는 val/iou/class_0, val/iou/class_1처럼 따로 기록한다.
    for key, value in val.items():
        if key.startswith("iou/"):
            writer.add_scalar(f"val/{key}", value, epoch)

    _write_score_scalars(writer, "val/all", epoch, val)
    for camera, scores in val.get("by_camera", {}).items():
        _write_score_scalars(writer, f"val/{camera}", epoch, scores)


def _write_eval_scalars(writer, split: str, epoch: int, scores: Dict[str, float]) -> None:
    """Write evaluation-only metrics such as final test scores."""
    if not writer:
        return

    _write_score_scalars(writer, f"{split}/all", epoch, scores)
    for key, value in scores.items():
        if key == "by_camera":
            for camera, camera_scores in value.items():
                _write_score_scalars(writer, f"{split}/{camera}", epoch, camera_scores)
            continue
        writer.add_scalar(f"{split}/{key}", value, epoch)


def _write_score_scalars(writer, prefix: str, epoch: int, scores: Dict[str, float]) -> None:
    for key, value in scores.items():
        if key == "by_camera":
            continue
        writer.add_scalar(f"{prefix}/{key}", value, epoch)


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
    if args.epochs > 0:
        cfg["epochs"] = args.epochs
    if args.batch_size > 0:
        cfg["batch_size"] = args.batch_size
    if args.num_workers >= 0:
        cfg["num_workers"] = args.num_workers
    if args.device and args.device.lower() != "auto":
        cfg["device"] = args.device


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
