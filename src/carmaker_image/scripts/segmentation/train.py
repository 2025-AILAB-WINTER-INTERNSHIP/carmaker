"""Training entry point shared by CLI wrappers."""

from __future__ import annotations

import argparse
import json
import random
from pathlib import Path
from typing import Any, Dict

import numpy as np
import torch
from torch.utils.data import DataLoader

from .adapters import CarmakerSegmentationAdapter
from .dataset import SegmentationDataset, split_dataset
from .losses import build_loss
from .metrics import confusion_matrix, mask_psnr, segmentation_scores
from .models import build_model
from .visualization import overlay_mask


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATA_ROOT = PACKAGE_ROOT / "data"
DEFAULT_CONFIG = PACKAGE_ROOT / "config" / "segmentation_unet.yaml"


def parse_args() -> argparse.Namespace:
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
    cfg = load_config(args.config)
    _resolve_config_paths(cfg, Path(args.config).expanduser().resolve().parent)
    _apply_cli_overrides(cfg, args)
    _set_seed(int(cfg.get("seed", 42)))

    device = torch.device(cfg.get("device") or ("cuda" if torch.cuda.is_available() else "cpu"))
    run_dir = Path(cfg.get("run_dir", PACKAGE_ROOT / "runs" / "unet_carmaker")).expanduser().resolve()
    checkpoint_dir = run_dir / "checkpoints"
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    adapter = CarmakerSegmentationAdapter(
        data_root=cfg.get("data_root", str(DEFAULT_DATA_ROOT)),
        manifest=cfg.get("manifest") or None,
        cameras=cfg.get("cameras"),
        prefer_post_processed=True,
    )
    dataset = SegmentationDataset(
        adapter=adapter,
        image_size=tuple(cfg.get("image_size", [512, 512])),
    )
    if args.overfit_count > 0:
        dataset.samples = dataset.samples[: args.overfit_count]

    if len(dataset) == 0:
        raise RuntimeError(
            "No training samples found. Run extract_bag_images.py and apply_mask.py first, "
            "or pass --data-root/--manifest explicitly."
        )

    train_dataset, val_dataset = split_dataset(
        dataset,
        val_ratio=float(cfg.get("val_ratio", 0.2)),
        seed=int(cfg.get("seed", 42)),
    )

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

    writer = _make_writer(run_dir)
    best_miou = -1.0
    epochs = int(cfg.get("epochs", 30))
    print(f"[train] samples={len(train_dataset)} val={len(val_dataset)} device={device} run_dir={run_dir}")

    for epoch in range(1, epochs + 1):
        train_loss = train_one_epoch(model, train_loader, criterion, optimizer, device)
        val = validate(model, val_loader, criterion, adapter.num_classes, device)

        print(
            f"[epoch {epoch:03d}] train_loss={train_loss:.4f} "
            f"val_loss={val['loss']:.4f} miou={val['miou']:.4f} "
            f"dice={val['dice']:.4f} acc={val['pixel_accuracy']:.4f}"
        )
        _write_scalars(writer, epoch, train_loss, val, optimizer)
        if writer and (epoch == 1 or epoch % int(cfg.get("image_log_interval", 5)) == 0):
            write_debug_images(writer, epoch, model, val_loader, adapter.palette, device)

        if val["miou"] > best_miou:
            best_miou = val["miou"]
            save_checkpoint(checkpoint_dir / "best.pt", model, optimizer, epoch, cfg, best_miou)

        if epoch % int(cfg.get("checkpoint_interval", 10)) == 0:
            save_checkpoint(checkpoint_dir / f"epoch_{epoch:03d}.pt", model, optimizer, epoch, cfg, best_miou)

    save_checkpoint(checkpoint_dir / "last.pt", model, optimizer, epochs, cfg, best_miou)
    if writer:
        writer.close()


def train_one_epoch(model, loader, criterion, optimizer, device: torch.device) -> float:
    model.train()
    total_loss = 0.0
    total_items = 0
    for batch in loader:
        image = batch["image"].to(device, non_blocking=True)
        mask = batch["mask"].to(device, non_blocking=True)
        optimizer.zero_grad(set_to_none=True)
        logits = model(image)
        loss = criterion(logits, mask)
        loss.backward()
        optimizer.step()
        batch_size = image.shape[0]
        total_loss += float(loss.item()) * batch_size
        total_items += batch_size
    return total_loss / max(total_items, 1)


@torch.no_grad()
def validate(model, loader, criterion, num_classes: int, device: torch.device) -> Dict[str, float]:
    model.eval()
    total_loss = 0.0
    total_items = 0
    matrix = torch.zeros((num_classes, num_classes), dtype=torch.int64, device=device)
    psnr_values = []

    for batch in loader:
        image = batch["image"].to(device, non_blocking=True)
        mask = batch["mask"].to(device, non_blocking=True)
        logits = model(image)
        loss = criterion(logits, mask)
        pred = torch.argmax(logits, dim=1)
        matrix += confusion_matrix(pred, mask, num_classes).to(device)
        psnr_values.append(mask_psnr(pred, mask, max_value=float(num_classes - 1)))

        batch_size = image.shape[0]
        total_loss += float(loss.item()) * batch_size
        total_items += batch_size

    scores = segmentation_scores(matrix.cpu())
    scores["loss"] = total_loss / max(total_items, 1)
    finite_psnr = [v for v in psnr_values if np.isfinite(v)]
    scores["psnr"] = float(np.mean(finite_psnr)) if finite_psnr else float("inf")
    return scores


@torch.no_grad()
def write_debug_images(writer, epoch: int, model, loader, palette, device: torch.device) -> None:
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
        writer.add_image(f"debug/{idx}/gt_overlay", overlay_mask(image_cpu[idx], gt[idx], palette), epoch, dataformats="HWC")
        writer.add_image(f"debug/{idx}/pred_overlay", overlay_mask(image_cpu[idx], pred[idx], palette), epoch, dataformats="HWC")


def save_checkpoint(path: Path, model, optimizer, epoch: int, cfg: Dict[str, Any], best_miou: float) -> None:
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
    try:
        from torch.utils.tensorboard import SummaryWriter
    except Exception as exc:
        print(f"[warn] TensorBoard disabled: {exc}")
        return None
    return SummaryWriter(log_dir=str(run_dir))


def _write_scalars(writer, epoch: int, train_loss: float, val: Dict[str, float], optimizer) -> None:
    if not writer:
        return
    writer.add_scalar("train/loss", train_loss, epoch)
    writer.add_scalar("val/loss", val["loss"], epoch)
    writer.add_scalar("val/miou", val["miou"], epoch)
    writer.add_scalar("val/dice", val["dice"], epoch)
    writer.add_scalar("val/pixel_accuracy", val["pixel_accuracy"], epoch)
    writer.add_scalar("val/psnr", val["psnr"], epoch)
    writer.add_scalar("train/lr", optimizer.param_groups[0]["lr"], epoch)
    for key, value in val.items():
        if key.startswith("iou/"):
            writer.add_scalar(f"val/{key}", value, epoch)


def _apply_cli_overrides(cfg: Dict[str, Any], args: argparse.Namespace) -> None:
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
    for key in ("data_root", "manifest", "run_dir"):
        value = cfg.get(key)
        if not value:
            continue
        path = Path(str(value)).expanduser()
        if not path.is_absolute():
            cfg[key] = str((base_dir / path).resolve())


def _set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
