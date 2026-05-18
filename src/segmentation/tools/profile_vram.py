import gc
import sys
from pathlib import Path

import torch

# Try importing the model; handle possible dependencies dynamically
try:
    from ..models import UNet
except ImportError:
    # Fallback for direct execution: uv run profile_vram.py
    SRC_ROOT = Path(__file__).resolve().parents[2]
    if str(SRC_ROOT) not in sys.path:
        sys.path.insert(0, str(SRC_ROOT))

    try:
        from segmentation.models import UNet
    except ImportError as inner_e:
        print(f"Import failed: {inner_e}")
        sys.exit(1)


def profile_inference(batch_size: int, use_amp: bool = True) -> bool:
    model = None
    dummy_input = None

    # Clear CUDA cache and reset peak memory trackers
    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats()

    precision_str = "FP16" if use_amp else "FP32"
    try:
        # 1. Instantiate the UNet model (using GroupNorm default as configured)
        model = UNet(
            in_channels=3, num_classes=3, base_channels=32, norm_type="group"
        ).cuda()
        model.eval()

        # Measure memory occupied by model weights
        mem_after_model = torch.cuda.memory_allocated() / (1024**2)

        # 2. Prepare dummy FHD [B, 3, 1080, 1920] image tensor on GPU
        dummy_input = torch.randn(batch_size, 3, 1080, 1920).cuda()

        # If mixed precision/FP16 is requested, convert to half precision
        if use_amp:
            dummy_input = dummy_input.half()
            model = model.half()

        # 3. Warmup pass to initialize CUDA kernels and internal autograd caching
        with torch.inference_mode():
            _ = model(dummy_input)

        # Reset peak memory stats to isolate active inference peak
        torch.cuda.reset_peak_memory_stats()

        # 4. Actual inference pass under inference_mode
        with torch.inference_mode():
            _ = model(dummy_input)

        peak_mem = torch.cuda.max_memory_allocated() / (1024**2)

        print(f"[Batch Size {batch_size} ({precision_str})]")
        print(f"  - Model Weights VRAM   : {mem_after_model:.2f} MB")
        print(f"  - Peak Inference VRAM  : {peak_mem:.2f} MB")
        print("-" * 50)
        return True
    except torch.OutOfMemoryError as oom_error:
        print(f"[Batch Size {batch_size} ({precision_str})]")
        print(f"  - OOM during profiling: {oom_error}")
        print("  - Suggestion: lower batch size or enable FP16.")
        print("-" * 50)
        return False
    finally:
        # Release references so each case starts from a clean state.
        del model
        del dummy_input
        gc.collect()
        torch.cuda.empty_cache()


if __name__ == "__main__":
    if not torch.cuda.is_available():
        print("CUDA is not available on this environment.")
        sys.exit(1)

    print("=" * 50)
    print("🚀 CarMaker Semantic Segmentation U-Net VRAM Profiler")
    print("=" * 50)

    results = [
        profile_inference(batch_size=1, use_amp=True),
        profile_inference(batch_size=4, use_amp=True),
        profile_inference(batch_size=4, use_amp=False),
    ]

    if all(results):
        print("Profiling session completed successfully!")
    else:
        print("Profiling session completed with OOM in some cases.")
    print("=" * 50)
