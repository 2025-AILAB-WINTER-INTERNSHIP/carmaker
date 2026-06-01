import gc
import json
import os
import statistics
import subprocess
import sys
from argparse import ArgumentParser
from pathlib import Path
from typing import Any, Dict, List, Literal, Tuple, cast

import torch

try:
    from ..models import UNet
except ImportError:
    # 직접 실행을 위한 Fallback (예: uv run profile_vram.py)
    SRC_ROOT = Path(__file__).resolve().parents[2]
    if str(SRC_ROOT) not in sys.path:
        sys.path.insert(0, str(SRC_ROOT))

    try:
        from segmentation.models import UNet
    except ImportError as inner_e:
        print(f"Import failed: {inner_e}")
        sys.exit(1)

Precision = Literal["fp16", "bf16", "fp32"]


def _to_mb(value_bytes: int) -> float:
    return value_bytes / (1024**2)


def profile_single_case(
    batch_size: int,
    precision: Precision,
    height: int,
    width: int,
    warmup: int,
    measured_runs: int,
    loss_name: str = "none",
) -> Dict[str, Any]:
    model = None
    dummy_input = None
    dummy_target = None
    loss_fn = None

    precision_lower = precision.lower()
    if precision_lower not in ("fp16", "bf16", "fp32"):
        return {
            "ok": False,
            "error": "Invalid precision. Use one of: fp16, bf16, fp32",
            "precision": precision,
            "batch_size": batch_size,
        }

    if precision_lower == "bf16" and not torch.cuda.is_bf16_supported():
        return {
            "ok": False,
            "error": "BF16 is not supported on this GPU",
            "precision": precision,
            "batch_size": batch_size,
            "unsupported": True,
        }

    torch.backends.cudnn.benchmark = False

    try:
        gc.collect()
        torch.cuda.empty_cache()
        torch.cuda.reset_peak_memory_stats()

        model = UNet(
            in_channels=3, num_classes=3, base_channels=32, norm_type="group"
        ).cuda()

        if loss_name != "none":
            from segmentation.losses import build_loss
            loss_fn = build_loss(name=loss_name, num_classes=3, device="cuda")
            model.train()
            dummy_target = torch.randint(0, 3, (batch_size, height, width), dtype=torch.long, device="cuda")
        else:
            model.eval()

        dummy_input = torch.randn(batch_size, 3, height, width, device="cuda")

        if precision_lower == "fp16":
            dummy_input = dummy_input.half()
            model = model.half()
            if loss_fn is not None:
                loss_fn = loss_fn.half()
        elif precision_lower == "bf16":
            dummy_input = dummy_input.bfloat16()
            model = model.bfloat16()
            if loss_fn is not None:
                loss_fn = loss_fn.bfloat16()

        try:
            model = torch.compile(model, options={"triton.cudagraphs": False})
        except Exception:
            try:
                model = torch.compile(model)
            except Exception as e:
                pass

        torch.cuda.synchronize()
        model_weights_mb = _to_mb(torch.cuda.memory_allocated())

        if loss_fn is not None:
            for _ in range(max(1, warmup)):
                outputs = model(dummy_input)
                loss = loss_fn(outputs, dummy_target)
                loss.backward()
                model.zero_grad()
        else:
            with torch.inference_mode():
                for _ in range(max(1, warmup)):
                    _ = model(dummy_input)
        torch.cuda.synchronize()

        run_peaks_allocated_mb: List[float] = []
        run_peaks_reserved_mb: List[float] = []
        run_latencies_ms: List[float] = []

        for _ in range(max(1, measured_runs)):
            torch.cuda.reset_peak_memory_stats()
            start_event = torch.cuda.Event(enable_timing=True)
            end_event = torch.cuda.Event(enable_timing=True)

            torch.cuda.synchronize()
            start_event.record()

            if loss_fn is not None:
                outputs = model(dummy_input)
                loss = loss_fn(outputs, dummy_target)
                loss.backward()
                model.zero_grad()
            else:
                with torch.inference_mode():
                    _ = model(dummy_input)

            end_event.record()
            torch.cuda.synchronize()

            run_peaks_allocated_mb.append(_to_mb(torch.cuda.max_memory_allocated()))
            run_peaks_reserved_mb.append(_to_mb(torch.cuda.max_memory_reserved()))
            run_latencies_ms.append(start_event.elapsed_time(end_event))

        baseline_allocated_mb = _to_mb(torch.cuda.memory_allocated())
        baseline_reserved_mb = _to_mb(torch.cuda.memory_reserved())

        median_latency_ms = statistics.median(run_latencies_ms)
        median_fps = (
            batch_size * 1000.0 / median_latency_ms if median_latency_ms > 0 else 0.0
        )

        return {
            "ok": True,
            "precision": precision_lower,
            "batch_size": batch_size,
            "shape": [batch_size, 3, height, width],
            "model_weights_mb": model_weights_mb,
            "baseline_allocated_mb": baseline_allocated_mb,
            "baseline_reserved_mb": baseline_reserved_mb,
            "peak_allocated_mb": max(run_peaks_allocated_mb),
            "peak_reserved_mb": max(run_peaks_reserved_mb),
            "latency_ms_median": median_latency_ms,
            "latency_ms_min": min(run_latencies_ms),
            "latency_ms_max": max(run_latencies_ms),
            "fps_median": median_fps,
            "run_peaks_allocated_mb": run_peaks_allocated_mb,
            "run_peaks_reserved_mb": run_peaks_reserved_mb,
            "run_latencies_ms": run_latencies_ms,
        }
    except torch.OutOfMemoryError as oom_error:
        return {
            "ok": False,
            "precision": precision,
            "batch_size": batch_size,
            "error": str(oom_error),
            "oom": True,
        }
    finally:
        del model
        del dummy_input
        if dummy_target is not None:
            del dummy_target
        if loss_fn is not None:
            del loss_fn
        gc.collect()
        torch.cuda.empty_cache()


def run_case_in_isolated_process(
    batch_size: int,
    precision: Precision,
    height: int,
    width: int,
    warmup: int,
    measured_runs: int,
    loss_name: str = "none",
) -> Dict[str, Any]:
    cmd = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--single-case",
        "--batch-size",
        str(batch_size),
        "--precision",
        precision,
        "--height",
        str(height),
        "--width",
        str(width),
        "--warmup",
        str(warmup),
        "--measured-runs",
        str(measured_runs),
        "--loss",
        loss_name,
        "--json",
    ]

    env = os.environ.copy()
    env.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

    completed = subprocess.run(
        cmd,
        check=False,
        env=env,
        capture_output=True,
        text=True,
    )

    payload = None
    for line in reversed(completed.stdout.splitlines()):
        line = line.strip()
        if line.startswith("{") and line.endswith("}"):
            payload = line
            break

    if payload is None:
        return {
            "ok": False,
            "precision": precision,
            "batch_size": batch_size,
            "error": "Failed to parse subprocess JSON output",
            "stdout": completed.stdout,
            "stderr": completed.stderr,
            "returncode": completed.returncode,
        }

    result = json.loads(payload)
    if completed.returncode != 0 and result.get("ok", False):
        result["ok"] = False
        result["error"] = f"Subprocess exited with code {completed.returncode}"
    return result


def summarize_case(case_results: List[Dict[str, Any]]) -> Dict[str, Any]:
    successful = [r for r in case_results if r.get("ok")]
    failed = [r for r in case_results if not r.get("ok")]

    if not successful:
        return {
            "ok": False,
            "num_trials": len(case_results),
            "num_success": 0,
            "num_failed": len(failed),
            "errors": [f.get("error", "Unknown error") for f in failed],
        }

    peaks_alloc = [float(cast(Any, r["peak_allocated_mb"])) for r in successful]
    peaks_rsrv = [float(cast(Any, r["peak_reserved_mb"])) for r in successful]
    weights = [float(cast(Any, r["model_weights_mb"])) for r in successful]
    latencies = [float(cast(Any, r["latency_ms_median"])) for r in successful]
    fps_values = [float(cast(Any, r["fps_median"])) for r in successful]

    summary = {
        "ok": True,
        "num_trials": len(case_results),
        "num_success": len(successful),
        "num_failed": len(failed),
        "model_weights_mb_median": statistics.median(weights),
        "peak_allocated_mb_median": statistics.median(peaks_alloc),
        "peak_allocated_mb_min": min(peaks_alloc),
        "peak_allocated_mb_max": max(peaks_alloc),
        "peak_reserved_mb_median": statistics.median(peaks_rsrv),
        "peak_reserved_mb_min": min(peaks_rsrv),
        "peak_reserved_mb_max": max(peaks_rsrv),
        "latency_ms_median": statistics.median(latencies),
        "latency_ms_min": min(latencies),
        "latency_ms_max": max(latencies),
        "fps_median": statistics.median(fps_values),
        "fps_min": min(fps_values),
        "fps_max": max(fps_values),
    }
    return summary


def print_case_report(
    batch_size: int, precision: Precision, summary: Dict[str, Any]
) -> None:
    precision_str = precision.upper()
    print(f"[Batch Size {batch_size} ({precision_str})]")
    if not summary.get("ok"):
        print("  - All trials failed.")
        print("-" * 50)
        return

    median_latency = summary['latency_ms_median']
    per_image_latency = median_latency / batch_size

    print(
        f"  - Trials (success/total): {summary['num_success']}/{summary['num_trials']}"
    )
    print(
        f"  - Model Weights VRAM (median): {summary['model_weights_mb_median']:.2f} MB"
    )
    print(
        "  - Peak Allocated VRAM (median/min/max): "
        f"{summary['peak_allocated_mb_median']:.2f} / "
        f"{summary['peak_allocated_mb_min']:.2f} / "
        f"{summary['peak_allocated_mb_max']:.2f} MB"
    )
    print(
        "  - Peak Reserved VRAM (median/min/max): "
        f"{summary['peak_reserved_mb_median']:.2f} / "
        f"{summary['peak_reserved_mb_min']:.2f} / "
        f"{summary['peak_reserved_mb_max']:.2f} MB"
    )
    print(
        "  - Inference Latency (Batch median/min/max): "
        f"{median_latency:.2f} / "
        f"{summary['latency_ms_min']:.2f} / "
        f"{summary['latency_ms_max']:.2f} ms"
    )
    print(
        "  - Inference Latency (Per Image median/min/max): "
        f"{per_image_latency:.2f} / "
        f"{summary['latency_ms_min']/batch_size:.2f} / "
        f"{summary['latency_ms_max']/batch_size:.2f} ms"
    )
    print(
        "  - Throughput (median/min/max): "
        f"{summary['fps_median']:.2f} / "
        f"{summary['fps_min']:.2f} / "
        f"{summary['fps_max']:.2f} frames/s"
    )

    errors = summary.get("errors", [])
    if isinstance(errors, list):
        for err in errors:
            print(f"  - Error: {err}")
    print("-" * 50)


def parse_args():
    parser = ArgumentParser(description="Stable VRAM profiler for UNet inference")
    parser.add_argument("--single-case", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument(
        "--precision",
        choices=["fp16", "bf16", "fp32"],
        default="fp16",
    )
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--width", type=int, default=720)
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--measured-runs", type=int, default=10)
    parser.add_argument("--suite-trials", type=int, default=5)
    parser.add_argument("--loss", type=str, default="none", help="Loss function name (e.g. ce, dice, focal, or none)")
    parser.add_argument("--compare-concat", action="store_true", help="Compare batch=4 (720x480) vs concatenated batch=1 sizes")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    if not torch.cuda.is_available():
        print("CUDA is not available on this environment.")
        sys.exit(1)

    if args.single_case:
        result = profile_single_case(
            batch_size=args.batch_size,
            precision=cast(Precision, args.precision),
            height=args.height,
            width=args.width,
            warmup=args.warmup,
            measured_runs=args.measured_runs,
            loss_name=args.loss,
        )
        if args.json:
            print(json.dumps(result, ensure_ascii=True))
        else:
            print(result)
        sys.exit(0 if result.get("ok") else 1)

    if args.compare_concat:
        print("=" * 80)
        print("CarMaker Semantic Segmentation U-Net Concat Performance Comparison")
        print(f"- Precision: {args.precision.upper()}, Loss: {args.loss.upper()}")
        print("- Isolated subprocess per trial")
        print("=" * 80)

        # (Label, batch_size, height, width)
        comparison_cases = [
            ("Baseline (No Concat)", 4, 480, 720),
            ("Concat Horizontal", 1, 480, 2880),
            ("Concat Vertical", 1, 1920, 720),
            ("Concat Grid (2x2)", 1, 960, 1440),
        ]

        results = []
        for label, b, h, w in comparison_cases:
            print(f"Profiling {label} (Batch {b}, {w}x{h})...")
            trials = []
            for trial_index in range(1, max(1, args.suite_trials) + 1):
                trials.append(
                    run_case_in_isolated_process(
                        batch_size=b,
                        precision=cast(Precision, args.precision),
                        height=h,
                        width=w,
                        warmup=args.warmup,
                        measured_runs=args.measured_runs,
                        loss_name=args.loss,
                    )
                )
            summary = summarize_case(trials)
            results.append((label, b, f"{w}x{h}", summary))

        print("\n" + "=" * 90)
        print(f"Concat Comparison Summary (Precision: {args.precision.upper()}, Loss: {args.loss.upper()})")
        print("=" * 90)
        print(f"{'Configuration':<25} | {'Batch':<5} | {'Resolution':<12} | {'Peak VRAM(MB)':<13} | {'Batch Lat.(ms)':<14} | {'Img Lat.(ms)':<12} | {'Throughput(FPS)':<15}")
        print("-" * 90)
        for label, b, res, summary in results:
            if summary.get("ok"):
                vram_str = f"{summary['peak_allocated_mb_median']:.2f}"
                lat_str = f"{summary['latency_ms_median']:.2f}"
                
                effective_batch = 4 if "Concat" in label else b
                img_lat_str = f"{(summary['latency_ms_median'] / effective_batch):.2f}"
                
                fps_val = summary['fps_median']
                if b == 1 and "Concat" in label:
                    fps_val *= 4
                fps_str = f"{fps_val:.2f}"
            else:
                errors = summary.get("errors", ["OOM"])
                is_oom = any("OutOfMemory" in str(e) or "OOM" in str(e) for e in errors)
                vram_str = "OOM" if is_oom else "FAILED"
                lat_str = "N/A"
                img_lat_str = "N/A"
                fps_str = "N/A"
            print(f"{label:<25} | {b:<5} | {res:<12} | {vram_str:>13} | {lat_str:>14} | {img_lat_str:>12} | {fps_str:>15}")
        print("=" * 90)
        print("* Throughput (FPS) for Concat cases is calculated as equivalent 720x480 frames processed per second (i.e. Batch 1 FPS * 4).")
        
        has_failures = any(not s.get("ok") for _, _, _, s in results)
        if has_failures:
            print("\nError Details:")
            for label, _, _, summary in results:
                if not summary.get("ok"):
                    errors = summary.get("errors", ["Unknown error"])
                    print(f"  * {label}: {', '.join(map(str, errors))}")
            print("=" * 80)
        sys.exit(0)

    print("=" * 60)
    print("CarMaker Semantic Segmentation U-Net Stable VRAM Profiler")
    print("- Isolated subprocess per trial")
    print("- Median/min/max over repeated trials")
    print("=" * 60)

    cases: List[Tuple[int, Precision]] = [
        (1, "fp16"),
        (4, "fp16"),
        (1, "bf16"),
        (4, "bf16"),
        (4, "fp32"),
    ]

    all_ok = True
    for batch_size, precision in cases:
        trials: List[Dict[str, Any]] = []
        for trial_index in range(1, max(1, args.suite_trials) + 1):
            print(
                f"Trial {trial_index}: warmup {args.warmup}, measured {args.measured_runs}"
            )
            trials.append(
                run_case_in_isolated_process(
                    batch_size=batch_size,
                    precision=precision,
                    height=args.height,
                    width=args.width,
                    warmup=args.warmup,
                    measured_runs=args.measured_runs,
                    loss_name=args.loss,
                )
            )

        summary = summarize_case(trials)
        print_case_report(batch_size, precision, summary)

        if not summary.get("ok"):
            all_ok = False

    if all_ok:
        print("Profiling session completed successfully!")
    else:
        print("Profiling session completed with failures in some cases.")
    print("=" * 60)
