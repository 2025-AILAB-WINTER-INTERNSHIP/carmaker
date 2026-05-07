"""PyTorch dataset for raw image and segmentation mask pairs."""

from __future__ import annotations

from typing import Callable, Optional, Tuple

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

from .adapters import DatasetAdapter, SegmentationSample


ImageMaskTransform = Callable[[np.ndarray, np.ndarray], Tuple[np.ndarray, np.ndarray]]


class SegmentationDataset(Dataset):
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
        image = self._read_image(sample)
        mask = self._read_mask(sample)

        image, mask = self._resize_pair(image, mask)
        if self.transform:
            image, mask = self.transform(image, mask)

        image_tensor = torch.from_numpy(image.transpose(2, 0, 1)).float() / 255.0
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
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    @staticmethod
    def _read_mask(sample: SegmentationSample) -> np.ndarray:
        mask = cv2.imread(str(sample.mask_path), cv2.IMREAD_UNCHANGED)
        if mask is None:
            raise FileNotFoundError(f"Could not read mask: {sample.mask_path}")
        if mask.ndim == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        return mask.astype(np.int64)

    def _resize_pair(self, image: np.ndarray, mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        width, height = self.image_size
        if image.shape[1] == width and image.shape[0] == height:
            return image, mask
        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)
        mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
        return image, mask


def split_dataset(dataset: Dataset, val_ratio: float, seed: int) -> tuple[Dataset, Dataset]:
    total = len(dataset)
    val_count = int(round(total * val_ratio))
    train_count = total - val_count
    generator = torch.Generator().manual_seed(seed)
    return torch.utils.data.random_split(dataset, [train_count, val_count], generator=generator)

