"""Evaluate a trained segmentation checkpoint on an external holdout dataset."""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch

if __package__ in {None, ""}:
    SEGMENTATION_ROOT = Path(__file__).resolve().parents[1]
    SRC_ROOT = SEGMENTATION_ROOT.parent
    sys.path.insert(0, str(SRC_ROOT))
    sys.path.insert(0, str(SEGMENTATION_ROOT))
else:
    SEGMENTATION_ROOT = Path(__file__).resolve().parents[1]
    SRC_ROOT = SEGMENTATION_ROOT.parent

try:
    from segmentation.adapters import CarmakerSegmentationAdapter, SegmentationSample
    from segmentation.inference import SegmentationPredictor
    from segmentation.metrics import confusion_matrix, mask_psnr, segmentation_scores
    from segmentation.utils.visualization import colorize_mask
except ImportError:
    from adapters import CarmakerSegmentationAdapter, SegmentationSample
    from inference import SegmentationPredictor
    from metrics import confusion_matrix, mask_psnr, segmentation_scores
    from utils.visualization import colorize_mask


DEFAULT_DATA_ROOT = SRC_ROOT / "carmaker_image" / "data"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run best.ckpt on an external CarMaker segmentation dataset and "
            "write metrics, per-sample scores, prediction masks, and overlays."
        )
    )
    parser.add_argument(
        "--ckpt", required=True, help="Path to best.ckpt or another checkpoint."
    )
    parser.add_argument("--data-root", default=str(DEFAULT_DATA_ROOT))
    parser.add_argument(
        "--manifest",
        default="",
        help="Path to manifest.csv. Defaults to data-root/csv/manifest.csv.",
    )
    parser.add_argument(
        "--config",
        default="",
        help="Optional config override for checkpoint inference.",
    )
    parser.add_argument(
        "--cameras", default="", help="Comma-separated camera filter, e.g. front,left."
    )
    parser.add_argument(
        "--use-raw-post-processed",
        action="store_true",
        help="Use manifest raw_post before raw.",
    )
    parser.add_argument(
        "--image-size", default="", help="Optional network input size WIDTH,HEIGHT."
    )
    parser.add_argument(
        "--eval-resolution", choices=("network", "original"), default="network"
    )
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument(
        "--max-samples",
        type=int,
        default=0,
        help="Limit sample count for quick checks. 0 means all.",
    )
    parser.add_argument(
        "--device", default="", help="Torch device override, e.g. cuda:0 or cpu."
    )
    parser.add_argument(
        "--precision", default="auto", choices=("auto", "fp32", "fp16", "bf16")
    )
    parser.add_argument(
        "--compile",
        action="store_true",
        help="Enable torch.compile for long GPU eval runs.",
    )
    parser.add_argument("--warmup-iterations", type=int, default=1)
    parser.add_argument(
        "--out-dir",
        default="",
        help="Output directory. Defaults to runs/eval_<ckpt>_<time>.",
    )
    parser.add_argument(
        "--overlay-count",
        type=int,
        default=32,
        help="Number of side-by-side overlays to save.",
    )
    parser.add_argument("--save-all-overlays", action="store_true")
    parser.add_argument("--save-pred-masks", action="store_true")
    parser.add_argument(
        "--tensorboard",
        action="store_true",
        help="Write TensorBoard scalars/images into out-dir.",
    )
    parser.add_argument("--tensorboard-image-count", type=int, default=16)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    ckpt_path = Path(args.ckpt).expanduser().resolve()
    if not ckpt_path.exists():
        raise FileNotFoundError(f"checkpoint not found: {ckpt_path}")

    out_dir = _resolve_out_dir(args.out_dir, ckpt_path)
    overlay_dir = out_dir / "overlays"
    pred_dir = out_dir / "pred_masks"
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.overlay_count > 0 or args.save_all_overlays:
        overlay_dir.mkdir(parents=True, exist_ok=True)
    if args.save_pred_masks:
        pred_dir.mkdir(parents=True, exist_ok=True)

    adapter = CarmakerSegmentationAdapter(
        data_root=args.data_root,
        manifest=args.manifest or None,
        cameras=args.cameras or None,
        use_raw_post_processed=args.use_raw_post_processed,
        use_mask_post_processed=True,
    )
    samples = list(adapter.samples())
    if args.max_samples > 0:
        samples = samples[: args.max_samples]
    if not samples:
        raise RuntimeError(
            "No evaluation samples found. Check --data-root, --manifest, and --cameras."
        )

    image_size = _parse_image_size(args.image_size) if args.image_size else None
    predictor = SegmentationPredictor(
        checkpoint_path=ckpt_path,
        config_path=args.config or None,
        device=args.device or None,
        image_size=image_size,
        resize_output=args.eval_resolution == "original",
        inference_precision=args.precision,
        warmup_iterations=args.warmup_iterations,
        use_compile=args.compile,
    )

    num_classes = len(predictor.class_names)
    palette = _palette_for_classes(adapter.palette, num_classes)
    matrix_total = torch.zeros((num_classes, num_classes), dtype=torch.int64)
    matrix_by_camera: dict[str, torch.Tensor] = defaultdict(
        lambda: torch.zeros((num_classes, num_classes), dtype=torch.int64)
    )
    matrix_by_scenario: dict[str, torch.Tensor] = defaultdict(
        lambda: torch.zeros((num_classes, num_classes), dtype=torch.int64)
    )
    sample_count_by_camera: dict[str, int] = defaultdict(int)
    sample_count_by_scenario: dict[str, int] = defaultdict(int)
    per_sample_rows: list[dict[str, Any]] = []
    tensorboard_panels: list[tuple[str, str, np.ndarray]] = []
    tensorboard_panels: list[tuple[str, str, np.ndarray]] = []
    invalid_target_pixels = 0
    total_inference_ms = 0.0

    print(f"[eval] samples={len(samples)} ckpt={ckpt_path}")
    print(f"[eval] out_dir={out_dir}")

    for start in range(0, len(samples), max(1, args.batch_size)):
        batch_samples = samples[start : start + max(1, args.batch_size)]
        batch_images_bgr = [
            _read_image_bgr(sample.image_path) for sample in batch_samples
        ]
        results, pure_ms = predictor.predict_batch(batch_images_bgr, color_order="bgr")
        total_inference_ms += float(pure_ms)

        for offset, (sample, image_bgr, result) in enumerate(
            zip(batch_samples, batch_images_bgr, results)
        ):
            sample_index = start + offset
            mask = _read_mask(sample.mask_path)
            pred = result.class_map
            pred_eval, mask_eval = _align_prediction_and_mask(
                pred, mask, args.eval_resolution, predictor.image_size
            )

            invalid = int(((mask_eval < 0) | (mask_eval >= num_classes)).sum())
            invalid_target_pixels += invalid

            sample_matrix = confusion_matrix(
                torch.from_numpy(pred_eval),
                torch.from_numpy(mask_eval),
                num_classes,
            ).to(torch.int64)
            matrix_total += sample_matrix

            camera = sample.camera or "unknown"
            scenario = _scenario_name(sample)
            matrix_by_camera[camera] += sample_matrix
            matrix_by_scenario[scenario] += sample_matrix
            sample_count_by_camera[camera] += 1
            sample_count_by_scenario[scenario] += 1

            scores = segmentation_scores(sample_matrix)
            row = {
                "index": sample_index,
                "camera": camera,
                "scenario": scenario,
                "image_path": str(sample.image_path),
                "mask_path": str(sample.mask_path),
                "valid_pixels": int(sample_matrix.sum().item()),
                "invalid_target_pixels": invalid,
                "psnr": mask_psnr(
                    torch.from_numpy(pred_eval),
                    torch.from_numpy(mask_eval),
                    max_value=float(num_classes - 1),
                ),
                **scores,
            }
            per_sample_rows.append(row)

            if args.save_pred_masks:
                pred_path = (
                    pred_dir
                    / f"{sample_index:06d}_{camera}_{_safe_stem(sample.image_path)}.png"
                )
                cv2.imwrite(str(pred_path), pred_eval.astype(np.uint8))

            save_overlay_file = (
                args.save_all_overlays or sample_index < args.overlay_count
            )
            save_tensorboard_image = args.tensorboard and len(tensorboard_panels) < max(
                0, args.tensorboard_image_count
            )
            if save_overlay_file or save_tensorboard_image:
                overlay = _make_overlay_panel(image_bgr, mask, pred, palette)
                if save_overlay_file:
                    overlay_path = (
                        overlay_dir
                        / f"{sample_index:06d}_{scenario}_{camera}_{_safe_stem(sample.image_path)}.png"
                    )
                    cv2.imwrite(
                        str(overlay_path), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
                    )
                if save_tensorboard_image:
                    tag = f"{sample_index:06d}_{scenario}_{camera}_{_safe_stem(sample.image_path)}"
                    tensorboard_panels.append((camera, tag, overlay))
                    tensorboard_panels.append((camera, tag, overlay))

        done = min(start + len(batch_samples), len(samples))
        if done == len(samples) or done % 50 == 0:
            print(f"[eval] processed {done}/{len(samples)}")

    metrics = _build_metrics_report(
        args=args,
        ckpt_path=ckpt_path,
        out_dir=out_dir,
        predictor=predictor,
        sample_count=len(samples),
        matrix_total=matrix_total,
        matrix_by_camera=matrix_by_camera,
        matrix_by_scenario=matrix_by_scenario,
        sample_count_by_camera=sample_count_by_camera,
        sample_count_by_scenario=sample_count_by_scenario,
        invalid_target_pixels=invalid_target_pixels,
        total_inference_ms=total_inference_ms,
    )

    _write_json(out_dir / "metrics.json", metrics)
    _write_per_sample_csv(out_dir / "per_sample_metrics.csv", per_sample_rows)
    if args.tensorboard:
        _write_tensorboard(out_dir, metrics, tensorboard_panels)

    overall = metrics["overall"]
    print(
        "[eval] overall "
        f"miou={overall['miou']:.4f} "
        f"miou_fg={overall['miou_fg']:.4f} "
        f"dice={overall['dice']:.4f} "
        f"f1={overall['f1']:.4f} "
        f"f1_fg={overall['f1_fg']:.4f} "
        f"lane_f1={overall.get('f1/class_1', 0.0):.4f} "
        f"landmark_f1={overall.get('f1/class_2', 0.0):.4f}"
    )
    print(f"[eval] wrote {out_dir / 'metrics.json'}")
    print(f"[eval] wrote {out_dir / 'per_sample_metrics.csv'}")
    if args.tensorboard:
        print(f"[eval] wrote TensorBoard events under {out_dir}")


def _resolve_out_dir(value: str, ckpt_path: Path) -> Path:
    if value:
        return Path(value).expanduser().resolve()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if ckpt_path.parent.name == "checkpoints" and ckpt_path.parent.parent.name:
        ckpt_name = f"{ckpt_path.parent.parent.name}_{ckpt_path.stem}"
    else:
        ckpt_name = ckpt_path.stem
    return (
        SEGMENTATION_ROOT / "runs" / f"eval_{_safe_name(ckpt_name)}_{timestamp}"
    ).resolve()


def _parse_image_size(value: str) -> tuple[int, int]:
    parts = [
        part.strip() for part in value.replace("x", ",").split(",") if part.strip()
    ]
    if len(parts) != 2:
        raise ValueError("--image-size must be WIDTH,HEIGHT")
    return int(parts[0]), int(parts[1])


def _read_image_bgr(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"could not read image: {path}")
    return image


def _read_mask(path: Path) -> np.ndarray:
    mask = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if mask is None:
        raise FileNotFoundError(f"could not read mask: {path}")
    if mask.ndim == 3:
        mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    return mask.astype(np.int64)


def _align_prediction_and_mask(
    pred: np.ndarray,
    mask: np.ndarray,
    eval_resolution: str,
    image_size: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray]:
    if eval_resolution == "network":
        width, height = image_size
        pred_eval = _resize_mask(pred, (width, height))
        mask_eval = _resize_mask(mask, (width, height))
        return pred_eval.astype(np.int64), mask_eval.astype(np.int64)

    height, width = pred.shape[:2]
    mask_eval = _resize_mask(mask, (width, height))
    return pred.astype(np.int64), mask_eval.astype(np.int64)


def _resize_mask(mask: np.ndarray, size: tuple[int, int]) -> np.ndarray:
    width, height = size
    if mask.shape[1] == width and mask.shape[0] == height:
        return mask
    return cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)


def _scenario_name(sample: SegmentationSample) -> str:
    metadata = sample.metadata or {}
    return metadata.get("scenario") or sample.image_path.parent.parent.name or "unknown"


def _palette_for_classes(
    base_palette: Any, num_classes: int
) -> list[tuple[int, int, int]]:
    palette = [tuple(int(v) for v in color) for color in base_palette]
    fallback = [
        (255, 0, 0),
        (0, 128, 255),
        (255, 0, 255),
        (0, 255, 255),
        (255, 128, 0),
    ]
    while len(palette) < num_classes:
        palette.append(fallback[(len(palette) - len(base_palette)) % len(fallback)])
    return palette[:num_classes]


def _make_overlay_panel(
    image_bgr: np.ndarray,
    mask: np.ndarray,
    pred: np.ndarray,
    palette: list[tuple[int, int, int]],
) -> np.ndarray:
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    height, width = image_rgb.shape[:2]
    gt_original = _resize_mask(mask, (width, height))
    pred_original = _resize_mask(pred, (width, height))

    gt_overlay = _overlay_hwc(image_rgb, gt_original, palette)
    pred_overlay = _overlay_hwc(image_rgb, pred_original, palette)
    error_overlay = _error_overlay(image_rgb, gt_original, pred_original)
    return _panel_with_labels(
        [
            ("raw", image_rgb),
            ("gt", gt_overlay),
            ("pred", pred_overlay),
            ("error", error_overlay),
        ]
    )


def _overlay_hwc(
    image_rgb: np.ndarray,
    mask: np.ndarray,
    palette: list[tuple[int, int, int]],
    alpha: float = 0.45,
) -> np.ndarray:
    color = colorize_mask(mask, palette)
    return ((1.0 - alpha) * image_rgb + alpha * color).astype(np.uint8)


def _error_overlay(
    image_rgb: np.ndarray, gt: np.ndarray, pred: np.ndarray
) -> np.ndarray:
    error = image_rgb.copy()
    valid = gt >= 0
    diff = (gt != pred) & valid
    highlight = np.array([255, 0, 0], dtype=np.uint8)
    error[diff] = (0.35 * error[diff] + 0.65 * highlight).astype(np.uint8)
    return error


def _panel_with_labels(items: list[tuple[str, np.ndarray]]) -> np.ndarray:
    pad = 8
    label_h = 28
    height, width, channels = items[0][1].shape
    panel = np.full(
        (height + label_h, len(items) * width + (len(items) - 1) * pad, channels),
        32,
        dtype=np.uint8,
    )
    for idx, (label, image) in enumerate(items):
        x0 = idx * (width + pad)
        cv2.putText(
            panel,
            label,
            (x0 + 8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
        panel[label_h : label_h + height, x0 : x0 + width] = image
    return panel


def _build_metrics_report(
    args: argparse.Namespace,
    ckpt_path: Path,
    out_dir: Path,
    predictor: SegmentationPredictor,
    sample_count: int,
    matrix_total: torch.Tensor,
    matrix_by_camera: dict[str, torch.Tensor],
    matrix_by_scenario: dict[str, torch.Tensor],
    sample_count_by_camera: dict[str, int],
    sample_count_by_scenario: dict[str, int],
    invalid_target_pixels: int,
    total_inference_ms: float,
) -> dict[str, Any]:
    overall = segmentation_scores(matrix_total)
    return {
        "checkpoint": str(ckpt_path),
        "data_root": str(Path(args.data_root).expanduser().resolve()),
        "manifest": str(Path(args.manifest).expanduser().resolve())
        if args.manifest
        else str(Path(args.data_root).expanduser().resolve() / "csv" / "manifest.csv"),
        "out_dir": str(out_dir),
        "samples": sample_count,
        "classes": list(predictor.class_names),
        "image_size": list(predictor.image_size),
        "eval_resolution": args.eval_resolution,
        "precision": predictor.inference_precision,
        "invalid_target_pixels": invalid_target_pixels,
        "total_inference_ms": total_inference_ms,
        "avg_inference_ms_per_batch": total_inference_ms
        / max(1, int(np.ceil(sample_count / max(1, args.batch_size)))),
        "overall": overall,
        "confusion_matrix": _matrix_to_list(matrix_total),
        "by_camera": {
            key: {
                **segmentation_scores(matrix),
                "samples": int(sample_count_by_camera.get(key, 0)),
                "valid_pixels": int(matrix.sum().item()),
                "confusion_matrix": _matrix_to_list(matrix),
            }
            for key, matrix in sorted(matrix_by_camera.items())
        },
        "by_scenario": {
            key: {
                **segmentation_scores(matrix),
                "samples": int(sample_count_by_scenario.get(key, 0)),
                "valid_pixels": int(matrix.sum().item()),
                "confusion_matrix": _matrix_to_list(matrix),
            }
            for key, matrix in sorted(matrix_by_scenario.items())
        },
    }


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")


def _write_per_sample_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _write_tensorboard(
    out_dir: Path,
    metrics: dict[str, Any],
    image_panels: list[tuple[str, str, np.ndarray]],
) -> None:
    try:
        from torch.utils.tensorboard import SummaryWriter
    except ImportError:
        print(
            "[warn] TensorBoard is not installed. Skipping TensorBoard event writing."
        )
        return

    writer = SummaryWriter(log_dir=str(out_dir))
    try:
        step = 0
        writer.add_scalar("final_test/samples", metrics["samples"], step)
        writer.add_scalar(
            "final_test/invalid_target_pixels", metrics["invalid_target_pixels"], step
        )
        writer.add_scalar(
            "final_test/avg_inference_ms_per_batch",
            metrics["avg_inference_ms_per_batch"],
            step,
        )

        _write_tensorboard_scalars(
            writer, "final_test/overall", metrics["overall"], step
        )
        for camera, values in metrics["by_camera"].items():
            _write_tensorboard_scalars(
                writer, f"final_test/camera/{_safe_tag(camera)}", values, step
            )
        for scenario, values in metrics["by_scenario"].items():
            _write_tensorboard_scalars(
                writer, f"final_test/scenario/{_safe_tag(scenario)}", values, step
            )

        _write_confusion_matrix_tensorboard(writer, metrics, step)
        _write_overlay_tensorboard(writer, image_panels, step)
        _write_confusion_matrix_tensorboard(writer, metrics, step)
        _write_overlay_tensorboard(writer, image_panels, step)

        writer.add_text(
            "final_test/summary",
            (
                f"checkpoint: `{metrics['checkpoint']}`\n\n"
                f"data_root: `{metrics['data_root']}`\n\n"
                f"manifest: `{metrics['manifest']}`\n\n"
                f"eval_resolution: `{metrics['eval_resolution']}`\n\n"
                f"classes: `{', '.join(metrics['classes'])}`"
            ),
            step,
        )
    finally:
        writer.flush()
        writer.close()


def _write_overlay_tensorboard(
    writer: Any,
    image_panels: list[tuple[str, str, np.ndarray]],
    step: int,
) -> None:
    panels_by_camera: dict[str, list[np.ndarray]] = defaultdict(list)
    camera_order = ["front", "rear", "left", "right"]

    for camera, tag, image in image_panels:
        camera_tag = _safe_tag(camera)
        writer.add_image(
            f"final_test/overlay/{camera_tag}/{_safe_tag(tag)}",
            image,
            step,
            dataformats="HWC",
        )
        panels_by_camera[camera].append(image)

    ordered_cameras = [
        camera for camera in camera_order if camera in panels_by_camera
    ] + sorted(camera for camera in panels_by_camera if camera not in camera_order)

    for camera in ordered_cameras:
        images = panels_by_camera[camera]
        if not images:
            continue
        grid = _make_image_grid(images[:4], columns=1)
        writer.add_image(
            f"final_test/overlay_grid/{_safe_tag(camera)}",
            grid,
            step,
            dataformats="HWC",
        )


def _write_confusion_matrix_tensorboard(
    writer: Any,
    metrics: dict[str, Any],
    step: int,
) -> None:
    class_names = metrics.get("classes") or []
    matrix = metrics.get("confusion_matrix")
    if matrix is not None:
        _write_confusion_matrix_text(
            writer,
            "final_test/confusion_matrix/overall",
            step,
            matrix,
            class_names,
        )
        writer.add_image(
            "final_test/confusion_matrix_image/overall",
            _confusion_matrix_image(matrix, class_names),
            step,
            dataformats="HWC",
        )

    for camera, values in metrics.get("by_camera", {}).items():
        camera_matrix = values.get("confusion_matrix")
        if camera_matrix is None:
            continue
        tag = _safe_tag(camera)
        _write_confusion_matrix_text(
            writer,
            f"final_test/confusion_matrix/camera/{tag}",
            step,
            camera_matrix,
            class_names,
        )
        writer.add_image(
            f"final_test/confusion_matrix_image/camera/{tag}",
            _confusion_matrix_image(camera_matrix, class_names),
            step,
            dataformats="HWC",
        )


def _write_confusion_matrix_text(
    writer: Any,
    tag: str,
    step: int,
    matrix: list[list[int]],
    class_names: list[str],
) -> None:
    labels = [str(name) for name in class_names]
    if len(labels) != len(matrix):
        labels = [f"class_{idx}" for idx in range(len(matrix))]

    header = "| GT \\ Pred | " + " | ".join(labels) + " |"
    separator = "|---|" + "|".join("---" for _ in labels) + "|"
    rows = [header, separator]
    for label, row in zip(labels, matrix):
        rows.append(
            "| " + label + " | " + " | ".join(str(int(value)) for value in row) + " |"
        )

    writer.add_text(tag, "\n".join(rows), step)


def _confusion_matrix_image(
    matrix: list[list[int]],
    class_names: list[str],
) -> np.ndarray:
    values = np.asarray(matrix, dtype=np.float64)
    if values.ndim != 2 or values.shape[0] == 0 or values.shape[1] == 0:
        return np.zeros((64, 64, 3), dtype=np.uint8)

    labels = [str(name) for name in class_names]
    if len(labels) != values.shape[0]:
        labels = [f"class_{idx}" for idx in range(values.shape[0])]

    cell = 96
    label_w = 150
    label_h = 54
    height = label_h + values.shape[0] * cell
    width = label_w + values.shape[1] * cell
    image = np.full((height, width, 3), 245, dtype=np.uint8)

    max_value = float(values.max()) if values.size else 0.0
    for row in range(values.shape[0]):
        for col in range(values.shape[1]):
            value = float(values[row, col])
            intensity = 0.0 if max_value <= 0.0 else value / max_value
            base = np.array([235, 245, 255], dtype=np.float64)
            hot = np.array([32, 104, 191], dtype=np.float64)
            color = ((1.0 - intensity) * base + intensity * hot).astype(np.uint8)
            y0 = label_h + row * cell
            x0 = label_w + col * cell
            image[y0 : y0 + cell, x0 : x0 + cell] = color
            cv2.rectangle(image, (x0, y0), (x0 + cell, y0 + cell), (210, 210, 210), 1)
            text = str(int(value))
            text_color = (255, 255, 255) if intensity > 0.45 else (25, 25, 25)
            _put_centered_text(image, text, (x0, y0, cell, cell), 0.65, text_color)

    for col, label in enumerate(labels):
        x0 = label_w + col * cell
        _put_centered_text(image, label, (x0, 0, cell, label_h), 0.48, (40, 40, 40))
    for row, label in enumerate(labels):
        y0 = label_h + row * cell
        _put_centered_text(image, label, (0, y0, label_w, cell), 0.48, (40, 40, 40))

    cv2.putText(
        image,
        "GT \\ Pred",
        (10, 33),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (40, 40, 40),
        1,
        cv2.LINE_AA,
    )
    return image


def _put_centered_text(
    image: np.ndarray,
    text: str,
    rect: tuple[int, int, int, int],
    scale: float,
    color: tuple[int, int, int],
) -> None:
    x, y, width, height = rect
    thickness = 1
    (tw, th), baseline = cv2.getTextSize(
        text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness
    )
    tx = x + max(4, (width - tw) // 2)
    ty = y + max(th + 4, (height + th - baseline) // 2)
    cv2.putText(
        image,
        text,
        (tx, ty),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def _make_image_grid(
    images: list[np.ndarray], columns: int = 2, pad: int = 8
) -> np.ndarray:
    if not images:
        raise ValueError("_make_image_grid requires at least one image")

    height, width, channels = images[0].shape
    rows = int(np.ceil(len(images) / max(1, columns)))
    grid_h = rows * height + max(rows - 1, 0) * pad
    grid_w = columns * width + max(columns - 1, 0) * pad
    grid = np.full((grid_h, grid_w, channels), 32, dtype=np.uint8)

    for idx, image in enumerate(images):
        row = idx // columns
        col = idx % columns
        y0 = row * (height + pad)
        x0 = col * (width + pad)
        grid[y0 : y0 + height, x0 : x0 + width] = image

    return grid


def _write_tensorboard_scalars(
    writer: Any, prefix: str, values: dict[str, Any], step: int
) -> None:
    for key, value in values.items():
        if key == "confusion_matrix" or not isinstance(value, (int, float)):
            continue
        if not np.isfinite(value):
            continue
        writer.add_scalar(f"{prefix}/{key}", float(value), step)


def _matrix_to_list(matrix: torch.Tensor) -> list[list[int]]:
    return [[int(value) for value in row] for row in matrix.detach().cpu().tolist()]


def _safe_stem(path: Path) -> str:
    return _safe_name(path.stem)


def _safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.+-]+", "_", value).strip("_") or "item"


def _safe_tag(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.+/-]+", "_", value).strip("_/") or "item"


if __name__ == "__main__":
    main()
