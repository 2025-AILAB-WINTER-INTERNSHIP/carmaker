"""Loss functions for segmentation experiments."""

from __future__ import annotations

from typing import Optional, Sequence

import torch
import torch.nn.functional as F
from torch import nn


def _preprocess_target(target: torch.Tensor, num_classes: int, ignore_index: Optional[int]) -> torch.Tensor:
    """타겟 라벨의 범위 이탈 방지 및 의미론적 무시(Ignore) 처리를 수행하는 공통 헬퍼 함수."""
    if ignore_index is not None:
        # 1. num_classes 이상의 비정상 라벨은 ignore_index로 변환
        # 2. ignore_index가 아닌 음수 값은 0으로 변환
        target_processed = torch.where(target >= num_classes, ignore_index, target)
        target_processed = torch.where(
            (target_processed < 0) & (target_processed != ignore_index),
            0,
            target_processed
        )
    else:
        target_processed = target.clamp(0, num_classes - 1)
    return target_processed


class DiceLoss(nn.Module):
    """Dice score를 loss로 변환한 segmentation loss (예외 방어 및 DDP 안정성 보강형)."""

    def __init__(
        self,
        num_classes: int,
        smooth: float = 1.0,
        ignore_index: Optional[int] = -100,
        exclude_classes: Optional[Sequence[int]] = None,
    ) -> None:
        super().__init__()
        self.num_classes = num_classes
        self.smooth = smooth
        self.ignore_index = ignore_index
        self.exclude_classes = tuple(exclude_classes or ())

    def forward(self, logits: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        # AMP 환경 하에서 분산 연산 시 1920x1080 대형 해상도 누적으로 인한 FP16 오버플로우(inf/inf -> NaN)를
        # 원천적으로 방지하기 위해, 전체 손실 연산을 명시적으로 FP32 공간에서만 수행하도록 강제합니다.
        with torch.cuda.amp.autocast(enabled=False):
            # logits: [B, C, H, W] -> softmax 확률 p_c.
            # target: [B, H, W] class id -> one-hot 정답 y_c.
            logits = logits.float()  # Cast to float32 for stable softmax under AMP

            # [수치적 안정성 확보 - Logits 오버플로우 방지]
            # Logits 값의 오버플로우로 인해 softmax 내부의 exp(z)가 inf가 되어 inf/inf = NaN이 터지는 현상을 방지합니다.
            logits = torch.clamp(logits, min=-50.0, max=50.0)

            probs = torch.softmax(logits, dim=1)

            # 공통 타겟 전처리 적용
            num_classes = self.num_classes
            target_processed = _preprocess_target(target, num_classes, self.ignore_index)

            # F.one_hot은 음수(-100)를 입력받을 수 없으므로, 변환 직전에만 안전하게 클램핑
            target_one_hot = (
                F.one_hot(target_processed.clamp(0, num_classes - 1), num_classes).permute(0, 3, 1, 2).float()
            )

            # ignore_index 제외를 위한 유효 픽셀 마스크 생성 (if 분기 없이 벡터화 연산 처리)
            ignore_val = self.ignore_index if self.ignore_index is not None else -9999
            valid = (target_processed != ignore_val).unsqueeze(1)
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
                else:
                    # [DDP 크래시 방지 핵심 리팩토링]
                    # 모든 클래스가 제외된 경우, NaN 발생 및 DDP 역전파 연산 단절을 방지하기 위해
                    # logits 연산 그래프에 종속된 0.0 로스 텐서를 안전하게 반환합니다.
                    return logits.sum() * 0.0

            return 1.0 - dice.mean()


class FocalLoss(nn.Module):
    """수학적 확률 분리 및 예외 안전성을 확보한 최적의 Focal Loss."""

    def __init__(
        self,
        num_classes: int,
        gamma: float = 2.0,
        weight: Optional[torch.Tensor] = None,
        ignore_index: Optional[int] = -100,
    ) -> None:
        super().__init__()
        self.num_classes = num_classes
        self.gamma = gamma
        self.ignore_index = ignore_index

        # [프레임워크 버전 안전 버퍼 등록 구현]
        if weight is not None:
            self.register_buffer("weight", weight)
        else:
            # 직접 인스턴스화 시에도 정확한 클래스 크기에 맞추어 가중치 1.0 버퍼 등록 (마법의 숫자 100 제거)
            self.register_buffer("weight", torch.ones(num_classes, dtype=torch.float32))

    def forward(self, logits: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        # AMP 환경 하에서 분산 연산 시 1920x1080 대형 해상도 누적으로 인한 FP16 오버플로우 및 기저 0.0 거듭제곱 미분 NaN 오류를
        # 방지하기 위해 전체 손실 연산을 명시적으로 FP32 공간에서만 수행하도록 강제합니다.
        with torch.cuda.amp.autocast(enabled=False):
            logits = logits.float()  # Cast to float32 for stable cross_entropy under AMP

            # [수치적 안정성 확보 1단계 - Logits 오버플로우 방지]
            # Logits 값 자체의 오버플로우/언더플로우 및 이로 인한 cross_entropy의 NaN 출력을 원천적으로 방지합니다.
            logits = torch.clamp(logits, min=-50.0, max=50.0)

            # 공통 타겟 전처리 적용
            num_classes = logits.shape[1]
            target_processed = _preprocess_target(target, num_classes, self.ignore_index)

            # [수치 왜곡 완벽 교정 및 수치적 안정성 확보]
            # Focal Loss of 초점 조절 계수 (1 - p_t)^gamma는 클래스 가중치(self.weight)가 반영되지 않은
            # '순수한 예측 확률 p_t'를 기반으로 계산되어야 수학적 정의와 완벽하게 일치합니다.
            # 따라서, 클래스 가중치가 없는 unweighted cross entropy를 먼저 계산합니다.
            ce_unweighted = F.cross_entropy(
                logits,
                target_processed,
                ignore_index=self.ignore_index if self.ignore_index is not None else -100,
                reduction="none",
            )

            # [수치적 안정성 확보 2단계 - 치명적인 NaN 방지 (inf * 0.0 = NaN)]
            # 특히 FP16 mixed precision 하에서 극도로 오판한 픽셀의 CE가 inf로 발산할 수 있습니다.
            # CE가 inf인 경우, 이후 fg/bg mask 곱셈 연산(focal_loss * fg_mask) 시 `inf * 0.0`이 발생하여
            # 최종 로스 및 그래디언트가 NaN으로 붕괴(Gradient Collapse)되는 현상이 발생합니다.
            # 이를 방지하기 위해 CE 값을 안전한 범위 내로 클램핑합니다.
            ce_unweighted = torch.clamp(ce_unweighted, min=0.0, max=50.0)

            # Focal Loss:
            #   FL = (1 - p_t)^gamma * CE
            #
            # p_t가 크면 이미 쉬운 픽셀이므로 (1 - p_t)^gamma가 작아져 기여도가 줄고,
            # p_t가 작으면 어려운 픽셀이므로 CE를 더 강하게 남긴다.

            # pt는 정답 class 확률 p_t와 같다. CE = -log(p_t)이므로 exp(-CE) = p_t.
            pt = torch.exp(-ce_unweighted)

            # FP16 precision 환경에서 (1.0 - pt)가 미세하게 음수가 되는 것을 방지하기 위해 clamp 적용.
            # base가 0.0이하로 떨어지면 float exponent (gamma) 연산 시 gradient가 NaN이 될 수 있습니다.
            focal_coeff = torch.clamp(1.0 - pt, min=0.0, max=1.0)

            # [PyTorch pow(0.0, float) 미분 NaN 방지 아키텍처]
            # base가 정확히 0.0일 때 PyTorch의 거듭제곱(pow) 미분 수치 불안정성으로 인해 그래디언트에 NaN이 전파되는 현상을 우회합니다.
            focal_coeff_safe = torch.where(focal_coeff > 0.0, focal_coeff, torch.ones_like(focal_coeff))
            focal_loss = (focal_coeff_safe ** self.gamma) * ce_unweighted
            focal_loss = torch.where(focal_coeff > 0.0, focal_loss, torch.zeros_like(focal_loss))

            # [클래스 가중치 독립 맵핑 및 가중 적용]
            # 음수 인덱스(ignore_index)가 weight 텐서 인덱싱 에러를 유발하는 것을 방지하기 위해 임시 클램핑 적용 (if 분기 제거)
            clamped_target = target_processed.clamp(0, num_classes - 1)
            pixel_weights = self.weight[clamped_target]
            focal_loss = focal_loss * pixel_weights

            # background가 대부분인 segmentation에서 foreground 학습 신호가 묻히지 않도록
            # 전경(target > 0)과 배경(target == 0)을 분리하여 정규화합니다.
            #
            # [중요] mixed-precision (FP16) 및 분산 학습(DDP) 환경에서의 극도의 안정성을 위해:
            # 1. GPU-CPU 동기화를 유발하고 backward graph를 불완전하게 만드는 boolean indexing (focal_loss[mask])을 배제합니다.
            # 2. 모든 연산을 완전 벡터화(Mask multiplication)하고 제어 흐름(if) 없이 계산합니다.
            # 3. ignore_index 픽셀들을 정확하게 계산에서 배제합니다. (ignore_index=None일 때의 런타임 방어 구현)
            # ignore_index 제외를 반영하여 전경/배경 마스크 생성 (if 분기 없이 100% 벡터화 연산 처리)
            ignore_val = self.ignore_index if self.ignore_index is not None else -9999
            fg_mask = ((target_processed > 0) & (target_processed != ignore_val)).float()
            bg_mask = ((target_processed == 0) & (target_processed != ignore_val)).float()

            num_fg = fg_mask.sum()
            num_bg = bg_mask.sum()

            # 전경 로스: 전경 픽셀들의 평균 에러
            fg_loss = (focal_loss * fg_mask).sum() / torch.clamp(num_fg, min=1.0)
            # 배경 로스: 배경 픽셀들의 평균 에러
            bg_loss = (focal_loss * bg_mask).sum() / torch.clamp(num_bg, min=1.0)

            # 각 마스크의 실제 존재 여부에 따라 최종 로스를 선택 (분기문 없이 곱셈으로 처리하여 연산 효율 극대화)
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
    # class_weights가 없더라도 모든 클래스에 1.0 가중치를 할당한 텐서를 기본 주입하여 forward 내부의 if 분기를 전면 제거합니다.
    weight = (
        torch.tensor(class_weights, dtype=torch.float32, device=device)
        if class_weights
        else torch.ones(num_classes, dtype=torch.float32, device=device)
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
        return FocalLoss(num_classes=num_classes, weight=weight)
    if key in {"focal-dice", "focal-dice-loss"}:
        focal = FocalLoss(num_classes=num_classes, weight=weight)
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
