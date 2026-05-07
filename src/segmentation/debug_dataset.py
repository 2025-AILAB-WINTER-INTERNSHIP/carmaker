"""Dataset inspection utility."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

try:
    from .adapters import CarmakerSegmentationAdapter
    from .dataset import SegmentationDataset
    from .visualization import overlay_mask
except ImportError:
    from adapters import CarmakerSegmentationAdapter
    from dataset import SegmentationDataset
    from visualization import overlay_mask


SEGMENTATION_ROOT = Path(__file__).resolve().parent
SRC_ROOT = SEGMENTATION_ROOT.parent
DEFAULT_DATA_ROOT = SRC_ROOT / "carmaker_image" / "data"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Inspect CarMaker segmentation samples.")
    parser.add_argument("--data-root", default=str(DEFAULT_DATA_ROOT))
    parser.add_argument("--manifest", default="")
    parser.add_argument("--cameras", default="")
    parser.add_argument("--image-size", default="1920,1080")
    parser.add_argument("--count", type=int, default=8)
    parser.add_argument("--out-dir", default=str(SEGMENTATION_ROOT / "runs" / "debug_dataset"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    width, height = _parse_size(args.image_size)
    adapter = CarmakerSegmentationAdapter(
        data_root=args.data_root,
        manifest=args.manifest or None,
        cameras=args.cameras or None,
    )
    dataset = SegmentationDataset(adapter=adapter, image_size=(width, height))
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[debug] samples={len(dataset)} out_dir={out_dir}")
    if len(dataset) == 0:
        print("[debug] no samples found")
        return

    for idx in range(min(args.count, len(dataset))):
        item = dataset[idx]
        mask = item["mask"]
        values, counts = np.unique(mask.numpy(), return_counts=True)
        histogram = dict(zip(values.tolist(), counts.tolist()))
        overlay = overlay_mask(item["image"], mask, adapter.palette)
        out_path = out_dir / f"sample_{idx:04d}_{item['camera'] or 'camera'}.png"
        cv2.imwrite(str(out_path), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))
        print(f"[debug] {idx:04d} camera={item['camera']} classes={histogram} image={item['image_path']}")


def _parse_size(value: str) -> tuple[int, int]:
    parts = [int(v.strip()) for v in value.split(",")]
    if len(parts) != 2:
        raise ValueError("--image-size must be WIDTH,HEIGHT")
    return parts[0], parts[1]


if __name__ == "__main__":
    main()
