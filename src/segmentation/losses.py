"""Loss functions for segmentation experiments."""

from __future__ import annotations

from typing import Optional

import torch
import torch.nn.functional as F
from torch import nn


class DiceLoss(nn.Module):
    """Dice score를 loss로 변환한 segmentation loss."""

    def __init__(
        self, num_classes: int, smooth: float = 1.0, ignore_index: Optional[int] = None
    ) -> None:
        super().__init__()
        self.num_classes = num_classes
        self.smooth = smooth
        self.ignore_index = ignore_index

    def forward(self, logits: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        # logits를 class probability로 바꾸고, target은 one-hot으로 맞춘다.
        probs = torch.softmax(logits, dim=1)
        target_one_hot = (
            F.one_hot(target.clamp_min(0), self.num_classes).permute(0, 3, 1, 2).float()
        )

        if self.ignore_index is not None:
            valid = (target != self.ignore_index).unsqueeze(1)
            probs = probs * valid
            target_one_hot = target_one_hot * valid

        dims = (0, 2, 3)
        intersection = torch.sum(probs * target_one_hot, dims)
        union = torch.sum(probs + target_one_hot, dims)
        dice = (2.0 * intersection + self.smooth) / (union + self.smooth)
        return 1.0 - dice.mean()


class FocalLoss(nn.Module):
    """어려운 픽셀에 더 큰 가중치를 주는 loss."""

    def __init__(
        self,
        gamma: float = 2.0,
        weight: Optional[torch.Tensor] = None,
        ignore_index: int = -100,
    ) -> None:
        super().__init__()
        self.gamma = gamma
        self.register_buffer("weight", weight if weight is not None else None)
        self.ignore_index = ignore_index

    def forward(self, logits: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        # CE가 작은 쉬운 픽셀은 (1-pt)^gamma로 기여도를 줄인다.
        ce = F.cross_entropy(
            logits,
            target,
            weight=self.weight,
            ignore_index=self.ignore_index,
            reduction="none",
        )
        pt = torch.exp(-ce)
        # return ((1.0 - pt) ** self.gamma * ce).mean()
        focal_loss = (1.0 - pt) ** self.gamma * ce
        num_valid = (target > 0).sum().float()
        num_valid = torch.clamp(num_valid, min=1.0)
        return focal_loss.sum() / num_valid


def build_loss(
    name: str,
    num_classes: int,
    class_weights: Optional[list[float]] = None,
    device: str | torch.device = "cpu",
) -> nn.Module:
    """config의 loss 이름을 실제 nn.Module로 변환한다."""
    weight = (
        torch.tensor(class_weights, dtype=torch.float32, device=device)
        if class_weights
        else None
    )
    key = name.lower().replace("_", "-")
    if key in {"cross-entropy", "ce", "weighted-cross-entropy", "weighted-ce"}:
        return nn.CrossEntropyLoss(weight=weight)
    if key in {"dice"}:
        return DiceLoss(num_classes=num_classes)
    if key in {"ce-dice", "cross-entropy-dice"}:
        ce = nn.CrossEntropyLoss(weight=weight)
        dice = DiceLoss(num_classes=num_classes)
        return CombinedLoss(ce, dice)
    if key in {"focal", "focal-loss"}:
        return FocalLoss(weight=weight)
    raise ValueError(f"Unknown loss: {name}")


class CombinedLoss(nn.Module):
    """두 loss를 가중합으로 묶는 wrapper."""

    def __init__(
        self,
        first: nn.Module,
        second: nn.Module,
        first_weight: float = 1.0,
        second_weight: float = 1.0,
    ) -> None:
        super().__init__()
        self.first = first
        self.second = second
        self.first_weight = first_weight
        self.second_weight = second_weight

    def forward(self, logits: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        return self.first_weight * self.first(
            logits, target
        ) + self.second_weight * self.second(logits, target)
