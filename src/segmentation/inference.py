"""Runtime helpers for semantic segmentation inference.

This module intentionally has no ROS imports. ROS nodes, offline tools, and
tests can all call the same predictor implementation.
"""

from __future__ import annotations

import json
from contextlib import nullcontext
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
        inference_precision: str = "fp16",
        warmup_iterations: int = 1,
        use_compile: bool = True,
    ) -> None:
        self.checkpoint_path = Path(checkpoint_path).expanduser().resolve()
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.resize_output = resize_output
        self.inference_precision, self.autocast_dtype = resolve_inference_precision(
            inference_precision,
            self.device,
        )

        # train.py의 checkpoint에는 학습 당시 config가 들어있다.
        # 런타임 config_path는 필요한 값만 덮어쓰는 용도로 사용한다.
        checkpoint = load_checkpoint(self.checkpoint_path, self.device)
        state = extract_model_state(checkpoint)
        cfg = checkpoint_config(checkpoint)
        if config_path:
            cfg.update(load_config(config_path))
        cfg = infer_missing_config_from_state(cfg, state)

        # class 정의는 현재 CarmakerSegmentationAdapter의 3개 class를 기본값으로 쓴다.
        # checkpoint/config에 class_names가 들어오면 그 값을 우선한다.
        self.class_names = tuple(class_names or cfg.get("class_names") or DEFAULT_CLASS_NAMES)
        self.image_size = parse_image_size(image_size or cfg.get("image_size") or (512, 512))

        # 모델 구조는 학습 config와 같은 build_model 경로를 사용해야 checkpoint weight와 맞는다.
        self.model = build_model(cfg, num_classes=len(self.class_names))
        self.model.load_state_dict(state)

        if self.autocast_dtype is None:
            self.model.to(self.device)
        else:
            # Runtime inference에서는 optimizer/master weight가 필요 없으므로 모델
            # 파라미터와 floating buffer까지 선택 precision으로 내려 VRAM을 줄인다.
            self.model.to(device=self.device, dtype=self.autocast_dtype)
        self.model.eval()
        # cuDNN Autotuning 활성화 (입력 이미지 해상도가 고정되어 있을 때 합성곱 최적화)
        if self.device.type == "cuda":
            torch.backends.cudnn.benchmark = True

        # torch.compile 최적화 컴파일 적용 (PyTorch 2.0 이상 지원)
        if use_compile:
            try:
                # 멀티스레드(ROS worker thread) 환경에서 CUDA Graphs TLS AssertionError 방지를 위해 cudagraphs 비활성화 옵션 적용
                self.model = torch.compile(self.model, options={"triton.cudagraphs": False}, dynamic=True)
            except Exception:
                try:
                    # Fallback: 옵션 없는 기본 컴파일 시도
                    self.model = torch.compile(self.model, dynamic=True)
                except Exception:
                    pass

        self.warmup(max(0, int(warmup_iterations)), batch_sizes=(1, 4))

    @torch.inference_mode()
    def predict(self, image: np.ndarray, color_order: str = "bgr") -> tuple[SegmentationResult, float]:
        """Run segmentation on a uint8 HWC image.

        Args:
            image: Input image as HWC uint8. ROS OpenCV images are usually BGR.
            color_order: Either ``"bgr"`` or ``"rgb"``.
        """
        results, pure_ms = self.predict_batch([image], color_order=color_order)
        return results[0], pure_ms

    @torch.inference_mode()
    def predict_batch(self, images: Sequence[np.ndarray], color_order: str = "bgr") -> tuple[list[SegmentationResult], float]:
        """Run segmentation on multiple uint8 HWC images in one model forward."""
        if not images:
            return []

        original_shapes = []
        tensors = []
        for image in images:
            if image is None or image.ndim != 3 or image.shape[2] != 3:
                raise ValueError("each image must be an HWC uint8 3-channel array")

            original_shapes.append(image.shape[:2])
            # CPU에서의 transpose(2, 0, 1)는 메모리 비연속성(non-contiguous)을 유발하므로
            # transpose 없이 raw HWC 형태 그대로 CPU 텐서 뷰를 생성하여 메모리 연속성을 보존합니다.
            if not image.flags.writeable:
                image = image.copy()
            tensors.append(torch.from_numpy(image))

        # Stack 및 GPU 전송을 복사 오버헤드 없이 고속(contiguous)으로 수행합니다.
        # non_blocking=True를 지정하여 비동기 PCIe 전송을 유도합니다.
        batch = torch.stack(tensors).to(self.device, non_blocking=True)
        
        # 1. 형 변환 및 정규화를 contiguous 상태의 HWC 텐서에 대해 먼저 수행 (GPU 메모리 대역폭 효율 극대화)
        if self.autocast_dtype is not None:
            batch = batch.to(self.autocast_dtype) / 255.0
        else:
            batch = batch.float() / 255.0

        # 2. GPU 상에서 채널 뒤집기 및 차원 변경 (HWC -> CHW)을 뷰 연산으로 수행 (메모리 복사 없음)
        if color_order.lower() == "bgr":
            batch = batch[:, :, :, [2, 1, 0]].permute(0, 3, 1, 2)
        else:
            batch = batch.permute(0, 3, 1, 2)

        # 3. 모델 추론 및 Interpolate 전 단 한 번 메모리를 정렬하여 contiguous하게 만듦
        batch = batch.contiguous()

        # Resize on GPU in parallel
        width, height = self.image_size
        if batch.shape[-1] != width or batch.shape[-2] != height:
            batch = torch.nn.functional.interpolate(
                batch, size=(height, width), mode="bilinear", align_corners=False
            )

        pure_inference_ms = 0.0
        if self.device.type == "cuda":
            start_event = torch.cuda.Event(enable_timing=True)
            end_event = torch.cuda.Event(enable_timing=True)

            torch.cuda.synchronize(self.device) # GPU 준비 대기
            start_event.record()

            with self.autocast_context():
                logits = self.model(batch)

            end_event.record()
            torch.cuda.synchronize(self.device) # GPU 연산 완료 대기
            pure_inference_ms = start_event.elapsed_time(end_event)
        else:
            with self.autocast_context():
                logits = self.model(batch)

        class_maps_t = torch.argmax(logits, dim=1).to(torch.uint8)

        # Check if all images share the same original resolution
        all_same_shape = all(shape == original_shapes[0] for shape in original_shapes)

        results = []
        if self.resize_output:
            if all_same_shape:
                original_h, original_w = original_shapes[0]
                if class_maps_t.shape[-2] != original_h or class_maps_t.shape[-1] != original_w:
                    class_maps_t = torch.nn.functional.interpolate(
                        class_maps_t.unsqueeze(1).float(),
                        size=(original_h, original_w),
                        mode="nearest"
                    ).squeeze(1).to(torch.uint8)
                
                # Single host-to-device transfer for the whole batch
                class_maps_cpu = class_maps_t.cpu().numpy()
                for i in range(len(images)):
                    results.append(
                        SegmentationResult(
                            class_map=class_maps_cpu[i],
                            class_names=self.class_names,
                        )
                    )
            else:
                # Resize individually on GPU
                for i, (oh, ow) in enumerate(original_shapes):
                    m = class_maps_t[i:i+1].unsqueeze(1).float()
                    if m.shape[-2] != oh or m.shape[-1] != ow:
                        m = torch.nn.functional.interpolate(m, size=(oh, ow), mode="nearest")
                    class_map = m.squeeze(1).to(torch.uint8)[0].cpu().numpy()
                    results.append(
                        SegmentationResult(
                            class_map=class_map,
                            class_names=self.class_names,
                        )
                    )
        else:
            class_maps_cpu = class_maps_t.cpu().numpy()
            for i in range(len(images)):
                results.append(
                    SegmentationResult(
                        class_map=class_maps_cpu[i],
                        class_names=self.class_names,
                    )
                )
        return results, pure_inference_ms

    def autocast_context(self):
        if self.autocast_dtype is None:
            return nullcontext()
        return torch.autocast(device_type=self.device.type, dtype=self.autocast_dtype)

    @torch.inference_mode()
    def warmup(self, iterations: int, batch_sizes: Sequence[int] = (1, 4)) -> None:
        """Run a few dummy forwards to initialize CUDA kernels and compile graphs for expected batch sizes."""
        if iterations <= 0 or self.device.type != "cuda":
            return

        width, height = self.image_size

        # 1. 실제 predict_batch 내부에서 변환되는 정확한 dtype 사용
        dtype = self.autocast_dtype if self.autocast_dtype else torch.float32

        with self.autocast_context():
            # 2. 예상되는 모든 배치 사이즈에 대해 순차적으로 컴파일 그래프를 생성
            for b in batch_sizes:
                # predict_batch의 batch.contiguous() 결과와 100% 동일한 NCHW 메모리 구조 생성
                dummy = torch.zeros((b, 3, height, width), device=self.device, dtype=dtype)
                
                for _ in range(iterations):
                    _ = self.model(dummy)
                    
        torch.cuda.synchronize(self.device)

def load_checkpoint(path: str | Path, device: torch.device) -> Dict[str, Any]:
    """Load a PyTorch/Lightning checkpoint saved by ``train.py``.

    Lightning ``.ckpt`` files may include optimizer state and other training-only
    tensors. Keep the checkpoint on CPU so those extra tensors do not consume
    inference VRAM; only the restored model is moved to the requested device.
    """
    path = Path(path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {path}")
    try:
        return torch.load(path, map_location="cpu", weights_only=False)
    except TypeError:
        return torch.load(path, map_location="cpu")


def extract_model_state(checkpoint: Dict[str, Any]) -> Dict[str, Any]:
    """Return a plain model state dict from repository or Lightning checkpoints."""
    if "model_state" in checkpoint:
        return clean_model_state(checkpoint["model_state"])

    state = checkpoint.get("state_dict")
    if state is None:
        return clean_model_state(checkpoint)

    model_keys = [key for key in state if key.startswith("model.")]
    if model_keys:
        return clean_model_state({key[6:]: state[key] for key in model_keys})

    return clean_model_state(state)


def clean_model_state(state: Dict[str, Any]) -> Dict[str, Any]:
    """Strip Lightning/training-only keys before loading the raw segmentation model."""
    return {
        (key[6:] if key.startswith("model.") else key): value
        for key, value in dict(state).items()
        if not key.startswith("criterion.")
    }


def checkpoint_config(checkpoint: Dict[str, Any]) -> Dict[str, Any]:
    cfg = checkpoint.get("config")
    if isinstance(cfg, dict):
        return dict(cfg)

    hparams = checkpoint.get("hyper_parameters")
    if isinstance(hparams, dict):
        nested_cfg = hparams.get("cfg") or hparams.get("config")
        if isinstance(nested_cfg, dict):
            return dict(nested_cfg)
        return dict(hparams)

    return {}


def infer_missing_config_from_state(cfg: Dict[str, Any], state: Dict[str, Any]) -> Dict[str, Any]:
    """Fill small compatibility gaps for older Lightning checkpoints."""
    cfg = dict(cfg)
    model_cfg = cfg.get("model")
    if isinstance(model_cfg, str):
        model_cfg = {"name": model_cfg}
    elif isinstance(model_cfg, dict):
        model_cfg = dict(model_cfg)
    else:
        model_cfg = {}

    if "norm" not in model_cfg and "norm" not in cfg:
        has_norm_affine = any(key.endswith(".block.1.weight") for key in state)
        has_batch_stats = any(key.endswith(".running_mean") for key in state)
        if has_norm_affine and not has_batch_stats:
            model_cfg["norm"] = "group"

    if model_cfg:
        cfg["model"] = model_cfg
    return cfg


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


def resolve_inference_precision(value: str, device: torch.device) -> tuple[str, torch.dtype | None]:
    key = str(value or "fp32").strip().lower()
    aliases = {
        "32": "fp32",
        "float32": "fp32",
        "false": "fp32",
        "none": "fp32",
        "16": "fp16",
        "float16": "fp16",
        "half": "fp16",
        "bfloat16": "bf16",
    }
    key = aliases.get(key, key)

    if key == "auto":
        if device.type != "cuda":
            return "fp32", None
        if torch.cuda.is_bf16_supported():
            return "bf16", torch.bfloat16
        return "fp16", torch.float16
    if key == "fp32":
        return "fp32", None
    if key == "fp16":
        if device.type != "cuda":
            raise ValueError("inference_precision=fp16 requires a CUDA device")
        return "fp16", torch.float16
    if key == "bf16":
        if device.type != "cuda":
            raise ValueError("inference_precision=bf16 requires a CUDA device")
        if not torch.cuda.is_bf16_supported():
            raise ValueError("inference_precision=bf16 requires a GPU with bf16 support")
        return "bf16", torch.bfloat16
    raise ValueError("inference_precision must be one of: fp32, fp16, bf16, auto")


def to_rgb(image: np.ndarray, color_order: str) -> np.ndarray:
    key = color_order.lower()
    if key == "rgb":
        return image.copy()
    if key == "bgr":
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    raise ValueError("color_order must be either 'bgr' or 'rgb'")


def preprocess_rgb(image_rgb: np.ndarray, image_size: tuple[int, int]) -> torch.Tensor:
    """Resize and normalize an RGB uint8 image into ``[1, 3, H, W]``."""
    if not image_rgb.flags.writeable:
        image_rgb = image_rgb.copy()
    width, height = image_size
    if image_rgb.shape[1] != width or image_rgb.shape[0] != height:
        # image_size는 config와 동일하게 [width, height] 순서를 따른다.
        image_rgb = cv2.resize(image_rgb, (width, height), interpolation=cv2.INTER_LINEAR)
    tensor = torch.from_numpy(image_rgb.transpose(2, 0, 1)).float() / 255.0
    return tensor.unsqueeze(0)
