"""Dataset adapters for segmentation training.

The adapter layer hides dataset layout differences from the training loop.
For this repository the primary source is ``carmaker_image/data``:

  raw_post_processed/<scenario>/<camera>/*_raw_*_post.png
  gt_post_processed/<scenario>/<camera>/*_GT_*_post.png
  csv/manifest.csv
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class SegmentationSample:
    image_path: Path
    mask_path: Path
    camera: str = ""
    metadata: Dict[str, str] | None = None


class DatasetAdapter:
    """Base interface used by the PyTorch dataset."""

    class_names: Tuple[str, ...] = ()
    palette: Tuple[Tuple[int, int, int], ...] = ()

    def samples(self) -> Sequence[SegmentationSample]:
        raise NotImplementedError

    @property
    def num_classes(self) -> int:
        return len(self.class_names)


class CarmakerSegmentationAdapter(DatasetAdapter):
    """Adapter for the current carmaker_image extraction pipeline."""

    class_names = ("background", "lane", "landmark")
    palette = (
        (0, 0, 0),
        (0, 255, 0),
        (255, 255, 0),
    )

    def __init__(
        self,
        data_root: str | Path,
        manifest: str | Path | None = None,
        cameras: Iterable[str] | str | None = None,
        prefer_post_processed: bool = True,
    ) -> None:
        self.data_root = Path(data_root).expanduser().resolve()
        self.manifest = (
            Path(manifest).expanduser().resolve()
            if manifest
            else self.data_root / "csv" / "manifest.csv"
        )
        self.cameras = self._parse_cameras(cameras)
        self.prefer_post_processed = prefer_post_processed
        self._samples = self._load_samples()

    def samples(self) -> Sequence[SegmentationSample]:
        return self._samples

    @staticmethod
    def _parse_cameras(cameras: Iterable[str] | str | None) -> Optional[set[str]]:
        if cameras is None:
            return None
        if isinstance(cameras, str):
            values = [c.strip() for c in cameras.split(",")]
        else:
            values = [str(c).strip() for c in cameras]
        selected = {c for c in values if c}
        return selected or None

    def _load_samples(self) -> List[SegmentationSample]:
        if self.manifest.exists():
            samples = self._load_manifest_samples(self.manifest)
            if samples:
                return samples
        return self._discover_post_processed_samples()

    def _resolve_data_path(self, value: str) -> Path:
        path = Path(value)
        if path.is_absolute():
            return path
        return self.data_root / path

    def _load_manifest_samples(self, manifest_path: Path) -> List[SegmentationSample]:
        samples: List[SegmentationSample] = []
        with manifest_path.open("r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                camera = row.get("camera", "")
                if self.cameras and camera not in self.cameras:
                    continue

                raw_key_order = ("raw_post", "raw") if self.prefer_post_processed else ("raw", "raw_post")
                gt_key_order = ("gt_post",) if self.prefer_post_processed else ("gt", "gt_post")
                image_path = self._first_existing(row, raw_key_order)
                mask_path = self._first_existing(row, gt_key_order)
                if not image_path or not mask_path:
                    continue

                samples.append(
                    SegmentationSample(
                        image_path=image_path,
                        mask_path=mask_path,
                        camera=camera,
                        metadata={k: v for k, v in row.items() if v is not None},
                    )
                )
        return samples

    def _first_existing(self, row: Dict[str, str], keys: Sequence[str]) -> Optional[Path]:
        for key in keys:
            value = row.get(key, "")
            if not value:
                continue
            path = self._resolve_data_path(value)
            if path.exists():
                return path
        return None

    def _discover_post_processed_samples(self) -> List[SegmentationSample]:
        raw_root = self.data_root / "raw_post_processed"
        gt_root = self.data_root / "gt_post_processed"
        samples: List[SegmentationSample] = []
        if not raw_root.exists() or not gt_root.exists():
            return samples

        for mask_path in sorted(gt_root.rglob("*.png")):
            camera = self._camera_from_name(mask_path.name)
            if self.cameras and camera not in self.cameras:
                continue

            rel_parent = mask_path.parent.relative_to(gt_root)
            raw_name = self._mask_name_to_raw_name(mask_path.name)
            image_path = raw_root / rel_parent / raw_name
            if not image_path.exists():
                continue

            samples.append(
                SegmentationSample(
                    image_path=image_path,
                    mask_path=mask_path,
                    camera=camera,
                    metadata={"source": "discovered"},
                )
            )
        return samples

    @staticmethod
    def _camera_from_name(name: str) -> str:
        return name.split("_", 1)[0] if "_" in name else ""

    @staticmethod
    def _mask_name_to_raw_name(name: str) -> str:
        raw_name = name.replace("_GT_", "_raw_")
        raw_name = raw_name.replace("_GT", "_raw")
        return raw_name
