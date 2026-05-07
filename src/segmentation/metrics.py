"""Metrics used during segmentation training and validation."""

from __future__ import annotations

import math
from typing import Dict

import torch


def confusion_matrix(pred: torch.Tensor, target: torch.Tensor, num_classes: int) -> torch.Tensor:
    pred = pred.view(-1).to(torch.int64)
    target = target.view(-1).to(torch.int64)
    valid = (target >= 0) & (target < num_classes)
    indices = num_classes * target[valid] + pred[valid]
    matrix = torch.bincount(indices, minlength=num_classes * num_classes)
    return matrix.reshape(num_classes, num_classes)


def segmentation_scores(matrix: torch.Tensor) -> Dict[str, float]:
    matrix = matrix.float()
    tp = torch.diag(matrix)
    fp = matrix.sum(dim=0) - tp
    fn = matrix.sum(dim=1) - tp
    denom = tp + fp + fn
    iou = torch.where(denom > 0, tp / denom.clamp_min(1.0), torch.zeros_like(tp))
    dice_denom = 2.0 * tp + fp + fn
    dice = torch.where(dice_denom > 0, 2.0 * tp / dice_denom.clamp_min(1.0), torch.zeros_like(tp))
    pixel_accuracy = tp.sum() / matrix.sum().clamp_min(1.0)

    scores = {
        "miou": float(iou.mean().item()),
        "dice": float(dice.mean().item()),
        "pixel_accuracy": float(pixel_accuracy.item()),
    }
    for idx, value in enumerate(iou):
        scores[f"iou/class_{idx}"] = float(value.item())
    return scores


def mask_psnr(pred: torch.Tensor, target: torch.Tensor, max_value: float = 2.0) -> float:
    mse = torch.mean((pred.float() - target.float()) ** 2).item()
    if mse == 0.0:
        return float("inf")
    return 20.0 * math.log10(max_value) - 10.0 * math.log10(mse)

