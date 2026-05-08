"""Visualization utilities for TensorBoard and dataset debugging."""

from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np
import torch


Palette = Sequence[Tuple[int, int, int]]


def colorize_mask(mask: torch.Tensor | np.ndarray, palette: Palette) -> np.ndarray:
    """class id mask를 RGB 색상 이미지로 변환한다."""
    if isinstance(mask, torch.Tensor):
        mask_np = mask.detach().cpu().numpy()
    else:
        mask_np = mask
    mask_np = mask_np.astype(np.int64)
    color = np.zeros((*mask_np.shape, 3), dtype=np.uint8)
    for class_id, rgb in enumerate(palette):
        color[mask_np == class_id] = rgb
    return color


def overlay_mask(image_chw: torch.Tensor, mask: torch.Tensor, palette: Palette, alpha: float = 0.45) -> np.ndarray:
    """원본 image 위에 colorized mask를 반투명하게 덮는다."""
    image = image_chw.detach().cpu().clamp(0.0, 1.0).numpy().transpose(1, 2, 0)
    image_u8 = (image * 255.0).astype(np.uint8)
    mask_color = colorize_mask(mask, palette)
    return ((1.0 - alpha) * image_u8 + alpha * mask_color).astype(np.uint8)
