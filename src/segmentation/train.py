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
import json
import random
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
    from .utils.visualization import overlay_mask
except ImportError:
    # `python3 src/segmentation/train.py`처럼 파일을 직접 실행할 때 사용하는 fallback import.
    from adapters import CarmakerSegmentationAdapter
    from dataset import SegmentationDataset, split_dataset
    from losses import build_loss
    from metrics import confusion_matrix, mask_psnr, segmentation_scores
    from models import build_model
    from utils.visualization import overlay_mask


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
    run_dir = Path(cfg.get("run_dir", SEGMENTATION_ROOT / "runs" / "unet_carmaker")).expanduser().resolve()
    checkpoint_dir = run_dir / "checkpoints"
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

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

    # 5) train / validation split
    # seed를 고정해서 매번 같은 split이 나오게 한다.
    train_dataset, val_dataset = split_dataset(
        dataset,
        val_ratio=float(cfg.get("val_ratio", 0.2)),
        seed=int(cfg.get("seed", 42)),
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
    best_miou = -1.0
    epochs = int(cfg.get("epochs", 30))
    print(f"[train] samples={len(train_dataset)} val={len(val_dataset)} device={device} run_dir={run_dir}")

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

        # TensorBoard Images에 GT/pred overlay를 기록한다.
        if writer and (epoch == 1 or epoch % int(cfg.get("image_log_interval", 5)) == 0):
            write_debug_images(writer, epoch, model, val_loader, adapter.palette, device)

        # validation mIoU가 최고일 때 best checkpoint를 갱신한다.
        if val["miou"] > best_miou:
            best_miou = val["miou"]
            save_checkpoint(checkpoint_dir / "best.pt", model, optimizer, epoch, cfg, best_miou)

        # 긴 학습 중간에도 복구 지점을 남기기 위한 주기적 checkpoint.
        if epoch % int(cfg.get("checkpoint_interval", 10)) == 0:
            save_checkpoint(checkpoint_dir / f"epoch_{epoch:03d}.pt", model, optimizer, epoch, cfg, best_miou)

    # 마지막 epoch 상태도 항상 저장한다.
    save_checkpoint(checkpoint_dir / "last.pt", model, optimizer, epochs, cfg, best_miou)
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
def validate(model, loader, criterion, num_classes: int, device: torch.device, epoch: int = 0) -> Dict[str, float]:
    """validation set에서 loss와 segmentation metric을 계산한다."""
    model.eval()
    total_loss = 0.0
    total_items = 0
    matrix = torch.zeros((num_classes, num_classes), dtype=torch.int64, device=device)
    psnr_values = []

    progress = _progress(loader, desc=f"val {epoch:03d}" if epoch else "val", leave=False)
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

        batch_size = image.shape[0]
        total_loss += float(loss.item()) * batch_size
        total_items += batch_size
        _set_progress_postfix(progress, loss=f"{total_loss / max(total_items, 1):.4f}")

    scores = segmentation_scores(matrix.cpu())
    scores["loss"] = total_loss / max(total_items, 1)

    # pred와 GT가 완전히 같으면 PSNR은 inf가 될 수 있으므로 평균에서는 finite 값만 사용한다.
    finite_psnr = [v for v in psnr_values if np.isfinite(v)]
    scores["psnr"] = float(np.mean(finite_psnr)) if finite_psnr else float("inf")
    return scores


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


@torch.no_grad()
def write_debug_images(writer, epoch: int, model, loader, palette, device: torch.device) -> None:
    """TensorBoard Images 탭에 GT/pred overlay를 기록한다."""
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

    for idx in range(max_items):
        gt_overlay = overlay_mask(image_cpu[idx], gt[idx], palette)
        pred_overlay = overlay_mask(image_cpu[idx], pred[idx], palette)
        writer.add_image(f"debug/{idx}/gt_overlay", gt_overlay, epoch, dataformats="HWC")
        writer.add_image(f"debug/{idx}/pred_overlay", pred_overlay, epoch, dataformats="HWC")


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


def _make_writer(run_dir: Path):
    """TensorBoard SummaryWriter를 만든다. tensorboard가 없으면 학습은 계속 진행한다."""
    try:
        from torch.utils.tensorboard import SummaryWriter
    except Exception as exc:
        print(f"[warn] TensorBoard disabled: {exc}")
        return None
    return SummaryWriter(log_dir=str(run_dir))


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
