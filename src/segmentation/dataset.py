"""Raw image와 class-id mask를 PyTorch tensor로 바꾸는 Dataset."""

from __future__ import annotations

from typing import Callable, Optional, Tuple

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

try:
    from .adapters import DatasetAdapter, SegmentationSample
except ImportError:
    # `python3 src/segmentation/train.py`처럼 직접 실행할 때를 위한 fallback.
    from adapters import DatasetAdapter, SegmentationSample


ImageMaskTransform = Callable[[np.ndarray, np.ndarray], Tuple[np.ndarray, np.ndarray]]


class SegmentationDataset(Dataset):
    """Adapter가 찾은 image/mask pair를 학습 가능한 tensor로 변환한다."""

    def __init__(
        self,
        adapter: DatasetAdapter,
        image_size: tuple[int, int] = (512, 512),
        transform: Optional[ImageMaskTransform] = None,
    ) -> None:
        self.adapter = adapter
        self.samples = list(adapter.samples())
        self.image_size = image_size
        self.transform = transform

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int) -> dict:
        sample = self.samples[index]

        # 원본 파일은 OpenCV로 읽고, mask는 class id를 보존해야 한다.
        image = self._read_image(sample)
        mask = self._read_mask(sample)

        # image와 mask는 반드시 같은 크기로 맞춰야 loss 계산이 가능하다.
        image, mask = self._resize_pair(image, mask)
        if self.transform:
            image, mask = self.transform(image, mask)

        # image: HWC uint8(0..255) -> CHW float32(0..1)
        image_tensor = torch.from_numpy(image.transpose(2, 0, 1)).float() / 255.0

        # mask: HW class id -> HW int64. CrossEntropyLoss target 형식이다.
        mask_tensor = torch.from_numpy(mask).long()

        return {
            "image": image_tensor,
            "mask": mask_tensor,
            "image_path": str(sample.image_path),
            "mask_path": str(sample.mask_path),
            "camera": sample.camera,
        }

    @staticmethod
    def _read_image(sample: SegmentationSample) -> np.ndarray:
        image = cv2.imread(str(sample.image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(f"Could not read image: {sample.image_path}")
        # OpenCV는 BGR로 읽기 때문에 PyTorch/TensorBoard에서 보기 좋은 RGB로 변환한다.
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    @staticmethod
    def _read_mask(sample: SegmentationSample) -> np.ndarray:
        mask = cv2.imread(str(sample.mask_path), cv2.IMREAD_UNCHANGED)
        if mask is None:
            raise FileNotFoundError(f"Could not read mask: {sample.mask_path}")
        if mask.ndim == 3:
            # GT가 3채널로 저장되어도 실제 학습에는 class id 1채널만 사용한다.
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        return mask.astype(np.int64)

    def _resize_pair(self, image: np.ndarray, mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        width, height = self.image_size
        if image.shape[1] == width and image.shape[0] == height:
            return image, mask

        # image는 자연스러운 보간을 위해 bilinear, mask는 class id 보존을 위해 nearest 사용.
        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)
        mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
        return image, mask


def split_dataset(
    dataset: Dataset,
    val_ratio: float,
    seed: int,
    test_ratio: float = 0.0,
) -> tuple[Dataset, Dataset, Dataset]:
    """Dataset을 train/validation/test로 나누되 seed로 split을 고정한다."""
    total = len(dataset)
    if val_ratio < 0 or test_ratio < 0 or val_ratio + test_ratio >= 1:
        raise ValueError("val_ratio and test_ratio must be non-negative and sum to less than 1.0")

    val_count = int(round(total * val_ratio))
    test_count = int(round(total * test_ratio))
    train_count = total - val_count - test_count
    generator = torch.Generator().manual_seed(seed)
    return torch.utils.data.random_split(
        dataset,
        [train_count, val_count, test_count],
        generator=generator,
    )
