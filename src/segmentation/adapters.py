"""Dataset adapters for segmentation training.

The adapter layer hides dataset layout differences from the training loop.
For this repository the primary source is ``carmaker_image/data``:

  raw_images/<scenario>/<camera>/*_raw_*.png
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
    """학습 코드가 공통으로 사용하는 단일 sample 표현."""

    image_path: Path
    mask_path: Path
    camera: str = ""
    metadata: Dict[str, str] | None = None


class DatasetAdapter:
    """Dataset별 폴더 구조 차이를 숨기기 위한 base interface."""

    class_names: Tuple[str, ...] = ()
    palette: Tuple[Tuple[int, int, int], ...] = ()

    def samples(self) -> Sequence[SegmentationSample]:
        raise NotImplementedError

    @property
    def num_classes(self) -> int:
        return len(self.class_names)


class CarmakerSegmentationAdapter(DatasetAdapter):
    """carmaker_image/data 구조를 읽는 Adapter."""

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
        prefer_post_processed: bool | None = None,
        use_raw_post_processed: bool = False,
        use_mask_post_processed: bool = True,
    ) -> None:
        self.data_root = Path(data_root).expanduser().resolve()
        self.manifest = (
            Path(manifest).expanduser().resolve()
            if manifest
            else self.data_root / "csv" / "manifest.csv"
        )
        self.cameras = self._parse_cameras(cameras)
        if prefer_post_processed is not None:
            use_raw_post_processed = prefer_post_processed
            use_mask_post_processed = prefer_post_processed
        self.use_raw_post_processed = use_raw_post_processed
        self.use_mask_post_processed = use_mask_post_processed
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
        # manifest.csv가 있으면 CSV pair를 우선 신뢰하고, 없으면 폴더에서 직접 찾는다.
        if self.manifest.exists():
            samples = self._load_manifest_samples(self.manifest)
            if samples:
                return samples
        return self._discover_post_processed_samples()

    def _resolve_data_path(self, value: str) -> Path:
        # CSV에는 data_root 기준 상대경로가 들어올 수 있으므로 절대경로로 정규화한다.
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

                # 학습용 GT는 class id가 저장된 gt_post를 기본으로 사용한다.
                raw_key_order = ("raw_post", "raw") if self.use_raw_post_processed else ("raw", "raw_post")
                gt_key_order = ("gt_post",) if self.use_mask_post_processed else ("gt", "gt_post")
                image_path = self._first_existing(row, raw_key_order)
                mask_path = self._first_existing(row, gt_key_order)
                if not image_path or not mask_path:
                    continue

                # 시나리오 정보 추출 (CSV 컬럼 우선, 없으면 경로에서 추출)
                scenario = row.get("scenario")
                if not scenario:
                    # image_path structure: .../raw_images/<scenario>/<camera>/file.png
                    scenario = image_path.parent.parent.name

                metadata = {k: v for k, v in row.items() if v is not None}
                metadata["scenario"] = scenario

                samples.append(
                    SegmentationSample(
                        image_path=image_path,
                        mask_path=mask_path,
                        camera=camera,
                        metadata=metadata,
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
        # CSV가 없을 때 파일명 규칙으로 raw_post와 gt_post를 매칭하는 fallback.
        raw_root = self.data_root / ("raw_post_processed" if self.use_raw_post_processed else "raw_images")
        gt_root = self.data_root / "gt_post_processed"
        samples: List[SegmentationSample] = []
        if not raw_root.exists() or not gt_root.exists():
            return samples

        for mask_path in sorted(gt_root.rglob("*.png")):
            camera = self._camera_from_name(mask_path.name)
            if self.cameras and camera not in self.cameras:
                continue

            rel_parent = mask_path.parent.relative_to(gt_root)
            raw_name = self._mask_name_to_raw_name(mask_path.name, post_processed=self.use_raw_post_processed)
            image_path = raw_root / rel_parent / raw_name
            if not image_path.exists():
                continue

            # rel_parent structure: <scenario>/<camera>
            scenario = rel_parent.parts[0] if len(rel_parent.parts) >= 1 else "unknown"

            samples.append(
                SegmentationSample(
                    image_path=image_path,
                    mask_path=mask_path,
                    camera=camera,
                    metadata={"source": "discovered", "scenario": scenario},
                )
            )
        return samples

    @staticmethod
    def _camera_from_name(name: str) -> str:
        return name.split("_", 1)[0] if "_" in name else ""

    @staticmethod
    def _mask_name_to_raw_name(name: str, post_processed: bool = True) -> str:
        raw_name = name.replace("_GT_", "_raw_")
        raw_name = raw_name.replace("_GT", "_raw")
        if not post_processed:
            path = Path(raw_name)
            if path.stem.endswith("_post"):
                raw_name = f"{path.stem[:-5]}{path.suffix}"
        return raw_name
