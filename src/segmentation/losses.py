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
        # logits: [B, C, H, W] -> softmax 확률 p_c.
        # target: [B, H, W] class id -> one-hot 정답 y_c.
        probs = torch.softmax(logits, dim=1)
        target_one_hot = (
            F.one_hot(target.clamp_min(0), self.num_classes).permute(0, 3, 1, 2).float()
        )

        # ignore_index가 있으면 해당 픽셀은 예측/정답 양쪽에서 0으로 만들어 제외한다.
        if self.ignore_index is not None:
            valid = (target != self.ignore_index).unsqueeze(1)
            probs = probs * valid
            target_one_hot = target_one_hot * valid

        # class별 soft Dice:
        #   Dice_c = (2 * sum(p_c * y_c) + smooth) / (sum(p_c) + sum(y_c) + smooth)
        # DiceLoss:
        #   loss = 1 - mean_c(Dice_c)
        #
        # 현재 DiceLoss는 class_weights를 직접 쓰지 않는다.
        # 모든 class의 Dice를 같은 비중으로 평균낸다.
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
        # 픽셀별 Cross Entropy:
        #   CE = -log(p_t)
        #
        # class_weights가 있으면 여기서 CE에 곱해진다.
        # 예: background weight를 낮추고 lane/landmark weight를 높이면
        #     소수 class 픽셀의 오차를 더 크게 본다.
        ce = F.cross_entropy(
            logits,
            target,
            weight=self.weight,
            ignore_index=self.ignore_index,
            reduction="none",
        )

        # pt는 정답 class 확률 p_t와 같다. CE = -log(p_t)이므로 exp(-CE) = p_t.
        pt = torch.exp(-ce)

        # Focal Loss:
        #   FL = (1 - p_t)^gamma * CE
        #
        # p_t가 크면 이미 쉬운 픽셀이므로 (1 - p_t)^gamma가 작아져 기여도가 줄고,
        # p_t가 작으면 어려운 픽셀이므로 CE를 더 강하게 남긴다.
        focal_loss = (1.0 - pt) ** self.gamma * ce

        # background가 대부분인 segmentation에서 foreground 학습 신호가 묻히지 않도록
        # 현재 구현은 전체 픽셀 수가 아니라 target > 0 픽셀 수로 normalize한다.
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
    # class_weights는 픽셀별 분류 loss인 CrossEntropy/Focal 계열에만 직접 적용된다.
    # 이 파일의 DiceLoss는 class_weights를 받지 않고 class별 Dice를 단순 평균한다.
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
    if key in {"focal-dice", "focal-dice-loss"}:
        focal = FocalLoss(weight=weight)
        dice = DiceLoss(num_classes=num_classes)
        return CombinedLoss(focal, dice, first_weight=0.5, second_weight=1.0)
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
        # Combined Loss:
        #   loss = first_weight * first_loss + second_weight * second_loss
        #
        # ce_dice:
        #   CE(class_weights 적용 가능) + Dice(class_weights 미적용)
        # focal_dice:
        #   0.5 * Focal(class_weights 적용 가능) + 1.0 * Dice(class_weights 미적용)
        return self.first_weight * self.first(
            logits, target
        ) + self.second_weight * self.second(logits, target)
