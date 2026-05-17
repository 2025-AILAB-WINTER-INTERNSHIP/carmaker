"""Loss functions for segmentation experiments."""

from __future__ import annotations

from typing import Optional, Sequence

import torch
import torch.nn.functional as F
from torch import nn


class DiceLoss(nn.Module):
    """Dice score를 loss로 변환한 segmentation loss."""

    def __init__(
        self,
        num_classes: int,
        smooth: float = 1.0,
        ignore_index: Optional[int] = None,
        exclude_classes: Optional[Sequence[int]] = None,
    ) -> None:
        super().__init__()
        self.num_classes = num_classes
        self.smooth = smooth
        self.ignore_index = ignore_index
        self.exclude_classes = tuple(exclude_classes or ())

    def forward(self, logits: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        # logits: [B, C, H, W] -> softmax 확률 p_c.
        # target: [B, H, W] class id -> one-hot 정답 y_c.
        logits = logits.float()  # Cast to float32 for stable softmax under AMP
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
        #   loss = 1 - mean_c(Dice_c), c not in exclude_classes
        #
        # 현재 DiceLoss는 class_weights를 직접 쓰지 않는다.
        # exclude_classes를 뺀 나머지 class의 Dice를 같은 비중으로 평균낸다.
        dims = (0, 2, 3)
        intersection = torch.sum(probs * target_one_hot, dims)
        union = torch.sum(probs + target_one_hot, dims)
        dice = (2.0 * intersection + self.smooth) / (union + self.smooth)
        if self.exclude_classes:
            include_mask = torch.ones(self.num_classes, dtype=torch.bool, device=logits.device)
            for class_id in self.exclude_classes:
                if 0 <= class_id < self.num_classes:
                    include_mask[class_id] = False
            if include_mask.any():
                dice = dice[include_mask]
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
        logits = logits.float()  # Cast to float32 for stable cross_entropy under AMP
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
        # Focal Loss:
        #   FL = (1 - p_t)^gamma * CE
        #
        # p_t가 크면 이미 쉬운 픽셀이므로 (1 - p_t)^gamma가 작아져 기여도가 줄고,
        # p_t가 작으면 어려운 픽셀이므로 CE를 더 강하게 남긴다.

        # pt는 정답 class 확률 p_t와 같다. CE = -log(p_t)이므로 exp(-CE) = p_t.
        pt = torch.exp(-ce)

        # FP16 precision 환경에서 (1.0 - pt)가 미세하게 음수가 되는 것을 방지하기 위해 clamp 적용.
        # base가 0.0이하로 떨어지면 float exponent (gamma) 연산 시 gradient가 NaN이 될 수 있습니다.
        focal_coeff = torch.clamp(1.0 - pt, min=0.0, max=1.0)
        focal_loss = (focal_coeff ** self.gamma) * ce

        # background가 대부분인 segmentation에서 foreground 학습 신호가 묻히지 않도록
        # 전경(target > 0)과 배경(target == 0)을 분리하여 정규화합니다.
        #
        # [중요] mixed-precision (FP16) 및 분산 학습(DDP) 환경에서의 극도의 안정성을 위해:
        # 1. GPU-CPU 동기화를 유발하고 backward graph를 불완전하게 만드는 boolean indexing (focal_loss[mask])을 배제합니다.
        # 2. 모든 연산을 완전 벡터화(Mask multiplication)하고 제어 흐름(if) 없이 계산합니다.
        # 3. ignore_index 픽셀들을 정확하게 계산에서 배제합니다.

        valid_mask = (target != self.ignore_index).float()
        fg_mask = ((target > 0) & (target != self.ignore_index)).float()
        bg_mask = ((target == 0) & (target != self.ignore_index)).float()

        num_fg = fg_mask.sum()
        num_bg = bg_mask.sum()

        # 전경 로스: 전경 픽셀들의 평균 에러
        fg_loss = (focal_loss * fg_mask).sum() / torch.clamp(num_fg, min=1.0)
        # 배경 로스: 배경 픽셀들의 평균 에러
        bg_loss = (focal_loss * bg_mask).sum() / torch.clamp(num_bg, min=1.0)

        # 각 마스크의 실제 존재 여부에 따라 최종 로스를 선택 (분기문 없이 곱셈으로 처리)
        fg_loss = fg_loss * (num_fg > 0).float()
        bg_loss = bg_loss * (num_bg > 0).float()

        # 두 로스를 합산하여 반환.
        return fg_loss + bg_loss


def build_loss(
    name: str,
    num_classes: int,
    class_weights: Optional[list[float]] = None,
    dice_exclude_classes: Optional[Sequence[int]] = None,
    device: str | torch.device = "cpu",
) -> nn.Module:
    """config의 loss 이름을 실제 nn.Module로 변환한다."""
    # class_weights는 픽셀별 분류 loss인 CrossEntropy/Focal 계열에만 직접 적용된다.
    # DiceLoss는 class_weights 대신 dice_exclude_classes로 평균 대상 class를 고른다.
    weight = (
        torch.tensor(class_weights, dtype=torch.float32, device=device)
        if class_weights
        else None
    )
    key = name.lower().replace("_", "-")
    if key in {"cross-entropy", "ce", "weighted-cross-entropy", "weighted-ce"}:
        return nn.CrossEntropyLoss(weight=weight)
    if key in {"dice"}:
        return DiceLoss(num_classes=num_classes, exclude_classes=dice_exclude_classes)
    if key in {"ce-dice", "cross-entropy-dice"}:
        ce = nn.CrossEntropyLoss(weight=weight)
        dice = DiceLoss(num_classes=num_classes, exclude_classes=dice_exclude_classes)
        return CombinedLoss(ce, dice)
    if key in {"focal", "focal-loss"}:
        return FocalLoss(weight=weight)
    if key in {"focal-dice", "focal-dice-loss"}:
        focal = FocalLoss(weight=weight)
        dice = DiceLoss(num_classes=num_classes, exclude_classes=dice_exclude_classes)
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
