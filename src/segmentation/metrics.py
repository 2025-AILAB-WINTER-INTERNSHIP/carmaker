"""Metrics used during segmentation training and validation."""

from __future__ import annotations

import math
from typing import Dict

import torch


def confusion_matrix(pred: torch.Tensor, target: torch.Tensor, num_classes: int) -> torch.Tensor:
    """pred/target 픽셀을 class별 confusion matrix로 누적한다."""
    pred = pred.view(-1).to(torch.int64)
    target = target.view(-1).to(torch.int64)
    valid = (target >= 0) & (target < num_classes)
    indices = num_classes * target[valid] + pred[valid]
    matrix = torch.bincount(indices, minlength=num_classes * num_classes)
    return matrix.reshape(num_classes, num_classes)


def segmentation_scores(matrix: torch.Tensor) -> Dict[str, float]:
    """confusion matrix에서 class별/전체 segmentation metric을 계산한다."""
    matrix = matrix.float()
    tp = torch.diag(matrix)
    fp = matrix.sum(dim=0) - tp
    fn = matrix.sum(dim=1) - tp

    # IoU: 예측 mask와 GT mask가 얼마나 겹치는지 보는 segmentation 대표 지표.
    denom = tp + fp + fn
    iou = torch.where(denom > 0, tp / denom.clamp_min(1.0), torch.zeros_like(tp))

    # Dice/F1(overlap): 얇은 차선처럼 작은 영역을 볼 때 IoU와 함께 확인하면 좋다.
    dice_denom = 2.0 * tp + fp + fn
    dice = torch.where(dice_denom > 0, 2.0 * tp / dice_denom.clamp_min(1.0), torch.zeros_like(tp))

    # Precision: 모델이 해당 class라고 예측한 것 중 실제로 맞은 비율.
    # 빛반사를 차선으로 잘못 칠하면 lane precision이 떨어진다.
    precision_denom = tp + fp
    precision = torch.where(
        precision_denom > 0,
        tp / precision_denom.clamp_min(1.0),
        torch.zeros_like(tp),
    )

    # Recall: GT class pixel 중 모델이 얼마나 찾아냈는지 보는 비율.
    # 차선이 끊기거나 빠지면 lane recall이 떨어진다.
    recall_denom = tp + fn
    recall = torch.where(
        recall_denom > 0,
        tp / recall_denom.clamp_min(1.0),
        torch.zeros_like(tp),
    )

    f1_denom = precision + recall
    f1 = torch.where(
        f1_denom > 0,
        2.0 * precision * recall / f1_denom.clamp_min(1e-12),
        torch.zeros_like(tp),
    )

    scores = {
        "miou": float(iou.mean().item()),
        "dice": float(dice.mean().item()),
    }
    for idx in range(matrix.shape[0]):
        scores[f"iou/class_{idx}"] = float(iou[idx].item())
        scores[f"dice/class_{idx}"] = float(dice[idx].item())
        scores[f"precision/class_{idx}"] = float(precision[idx].item())
        scores[f"recall/class_{idx}"] = float(recall[idx].item())
        scores[f"f1/class_{idx}"] = float(f1[idx].item())
    return scores


def mask_psnr(pred: torch.Tensor, target: torch.Tensor, max_value: float = 2.0) -> float:
    """예측 mask와 GT mask의 픽셀 차이를 PSNR 형태로 참고 계산한다."""
    mse = torch.mean((pred.float() - target.float()) ** 2).item()
    if mse == 0.0:
        return float("inf")
    return 20.0 * math.log10(max_value) - 10.0 * math.log10(mse)
