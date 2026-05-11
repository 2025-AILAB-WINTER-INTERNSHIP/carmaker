"""Runtime helpers for semantic segmentation inference.

This module intentionally has no ROS imports. ROS nodes, offline tools, and
tests can all call the same predictor implementation.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Sequence

import cv2
import numpy as np
import torch

try:
    from .adapters import CarmakerSegmentationAdapter
    from .models import build_model
except ImportError:
    from adapters import CarmakerSegmentationAdapter
    from models import build_model


DEFAULT_CLASS_NAMES = tuple(CarmakerSegmentationAdapter.class_names)


@dataclass(frozen=True)
class SegmentationResult:
    """Outputs from one segmentation inference pass."""

    class_map: np.ndarray
    class_names: Sequence[str]


class SegmentationPredictor:
    """Load a trained segmentation checkpoint and run image inference."""

    def __init__(
        self,
        checkpoint_path: str | Path,
        config_path: str | Path | None = None,
        device: str | None = None,
        image_size: Sequence[int] | None = None,
        class_names: Sequence[str] | None = None,
        resize_output: bool = True,
    ) -> None:
        self.checkpoint_path = Path(checkpoint_path).expanduser().resolve()
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.resize_output = resize_output

        # train.py의 checkpoint에는 학습 당시 config가 들어있다.
        # 런타임 config_path는 필요한 값만 덮어쓰는 용도로 사용한다.
        checkpoint = load_checkpoint(self.checkpoint_path, self.device)
        cfg = dict(checkpoint.get("config") or {})
        if config_path:
            cfg.update(load_config(config_path))

        # class 정의는 현재 CarmakerSegmentationAdapter의 3개 class를 기본값으로 쓴다.
        # checkpoint/config에 class_names가 들어오면 그 값을 우선한다.
        self.class_names = tuple(class_names or cfg.get("class_names") or DEFAULT_CLASS_NAMES)
        self.image_size = parse_image_size(image_size or cfg.get("image_size") or (512, 512))

        # 모델 구조는 학습 config와 같은 build_model 경로를 사용해야 checkpoint weight와 맞는다.
        self.model = build_model(cfg, num_classes=len(self.class_names))
        state = checkpoint.get("model_state", checkpoint)
        self.model.load_state_dict(state)
        self.model.to(self.device)
        self.model.eval()

    @torch.inference_mode()
    def predict(self, image: np.ndarray, color_order: str = "bgr") -> SegmentationResult:
        """Run segmentation on a uint8 HWC image.

        Args:
            image: Input image as HWC uint8. ROS OpenCV images are usually BGR.
            color_order: Either ``"bgr"`` or ``"rgb"``.
        """
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("image must be an HWC uint8 3-channel array")

        original_h, original_w = image.shape[:2]

        # 학습 Dataset은 RGB, float32, CHW, 0..1 스케일을 사용했다.
        # ROS/OpenCV 입력은 보통 BGR이므로 여기서 학습 입력 형식으로 맞춘다.
        rgb = to_rgb(image, color_order)
        tensor = preprocess_rgb(rgb, self.image_size).to(self.device)

        # CrossEntropy 기반 semantic segmentation 모델의 출력은 [B, C, H, W] logits이다.
        # class_map은 각 픽셀에서 가장 큰 logit을 가진 class id이다.
        logits = self.model(tensor)
        class_map_t = torch.argmax(logits, dim=1)
        class_map = class_map_t[0].detach().cpu().numpy().astype(np.uint8)

        # 모델 입력 크기가 원본 카메라 크기와 다를 수 있으므로 class id를 nearest로 복원한다.
        # bilinear를 쓰면 class id가 섞일 수 있어서 segmentation mask에는 쓰면 안 된다.
        if self.resize_output and (class_map.shape[0] != original_h or class_map.shape[1] != original_w):
            class_map = cv2.resize(class_map, (original_w, original_h), interpolation=cv2.INTER_NEAREST)

        return SegmentationResult(
            class_map=class_map,
            class_names=self.class_names,
        )


def load_checkpoint(path: str | Path, device: torch.device) -> Dict[str, Any]:
    """Load a PyTorch checkpoint saved by ``train.py``."""
    path = Path(path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {path}")
    try:
        return torch.load(path, map_location=device, weights_only=False)
    except TypeError:
        return torch.load(path, map_location=device)


def load_config(path: str | Path) -> Dict[str, Any]:
    """Load a JSON or YAML config file."""
    path = Path(path).expanduser().resolve()
    with path.open("r", encoding="utf-8") as f:
        if path.suffix.lower() in {".yaml", ".yml"}:
            try:
                import yaml
            except ImportError as exc:
                raise RuntimeError("PyYAML is required to read YAML config files") from exc
            return yaml.safe_load(f) or {}
        return json.load(f)


def parse_image_size(value: Sequence[int] | str) -> tuple[int, int]:
    """Return image size as ``(width, height)``."""
    if isinstance(value, str):
        parts = [p.strip() for p in value.replace("x", ",").split(",") if p.strip()]
        if len(parts) != 2:
            raise ValueError(f"image_size must have width,height format: {value}")
        return int(parts[0]), int(parts[1])
    if len(value) != 2:
        raise ValueError("image_size must contain exactly two values: width, height")
    return int(value[0]), int(value[1])


def to_rgb(image: np.ndarray, color_order: str) -> np.ndarray:
    key = color_order.lower()
    if key == "rgb":
        return image.copy()
    if key == "bgr":
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    raise ValueError("color_order must be either 'bgr' or 'rgb'")


def preprocess_rgb(image_rgb: np.ndarray, image_size: tuple[int, int]) -> torch.Tensor:
    """Resize and normalize an RGB uint8 image into ``[1, 3, H, W]``."""
    width, height = image_size
    if image_rgb.shape[1] != width or image_rgb.shape[0] != height:
        # image_size는 config와 동일하게 [width, height] 순서를 따른다.
        image_rgb = cv2.resize(image_rgb, (width, height), interpolation=cv2.INTER_LINEAR)
    tensor = torch.from_numpy(image_rgb.transpose(2, 0, 1)).float() / 255.0
    return tensor.unsqueeze(0)
