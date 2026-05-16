"""GPU-accelerated visualization utilities using PyTorch and torchvision."""

from __future__ import annotations
import torch
import torchvision.utils as vutils

def colorize_mask(mask: torch.Tensor, palette: torch.Tensor) -> torch.Tensor:
    """class id mask를 RGB 색상 이미지(3, H, W)로 변환한다 (GPU 최적화)."""
    # mask: (H, W) or (B, H, W) - long
    # palette: (num_classes, 3) - uint8
    if mask.dim() == 2:
        return palette[mask.long()].permute(2, 0, 1) # (3, H, W)
    return palette[mask.long()].permute(0, 3, 1, 2) # (B, 3, H, W)

def overlay_mask_gpu(image_chw: torch.Tensor, mask: torch.Tensor, palette: torch.Tensor, alpha: float = 0.45) -> torch.Tensor:
    """원본 image 위에 colorized mask를 GPU 상에서 블렌딩한다."""
    # image_chw: (3, H, W) float [0, 1]
    # mask: (H, W) long
    # palette: (num_classes, 3) uint8

    # 기존 CPU 로직과 100% 동일한 거동을 위해 입력 이미지 클램핑
    image_chw = image_chw.clamp(0.0, 1.0)

    # 1. GPU 기반 색상 변환 (H, W, 3) -> (3, H, W)
    mask_rgb = palette[mask.long()].to(image_chw.dtype).permute(2, 0, 1) / 255.0

    # 2. Alpha 블렌딩
    overlay = (1.0 - alpha) * image_chw + alpha * mask_rgb

    # 3. uint8 변환 (3, H, W)
    return (overlay.clamp(0.0, 1.0) * 255.0).to(torch.uint8)

def make_image_grid_gpu(images: list[torch.Tensor], columns: int = 2, pad: int = 8) -> torch.Tensor:
    """torchvision을 사용하여 GPU 상의 Tensor 리스트를 하나의 Grid Tensor(3, H, W)로 묶는다."""
    if not images:
        raise ValueError("make_image_grid_gpu requires at least one image")

    # images: list of (3, H, W) tensors (uint8 or float)
    # vutils.make_grid expects (B, C, H, W) or a list of (C, H, W)
    # nrow는 한 행에 들어갈 이미지의 개수(열의 수)를 의미함
    return vutils.make_grid(images, nrow=columns, padding=pad)
