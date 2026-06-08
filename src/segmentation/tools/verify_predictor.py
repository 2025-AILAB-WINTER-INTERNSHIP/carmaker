#!/usr/bin/env python3
import os
import sys
import torch
import numpy as np
from pathlib import Path

# 1. ROS 패키지 경로를 찾아서 추가 (노드와 동일)
def add_segmentation_source_path() -> None:
    search_roots = list(Path(__file__).resolve().parents) + list(Path.cwd().resolve().parents)
    search_roots.append(Path.cwd().resolve())
    for prefix in os.environ.get("CATKIN_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            search_roots.extend(Path(prefix).resolve().parents)

    for root in search_roots:
        for candidate in (root, root / "src"):
            if (candidate / "segmentation" / "__init__.py").exists():
                candidate_str = str(candidate)
                if candidate_str not in sys.path:
                    sys.path.insert(0, candidate_str)
                return

add_segmentation_source_path()

# 2. 클래스를 통째로 가져옵니다!
from segmentation.inference import SegmentationPredictor

# Hook 발화 여부를 전역 플래그로 추적하여 최종 판단에 반영합니다.
_nan_detected = False

def check_nan_hook(module, input, output):
    """모델 내부를 감시하는 CCTV.

    단일 텐서뿐 아니라 tuple/list 출력(Attention 등)도 재귀적으로 검사합니다.
    NaN/Inf 감지 시 전역 플래그를 설정하여 최종 결론에 반영합니다.
    """
    global _nan_detected

    # 출력이 텐서인지, 텐서들의 시퀀스인지 모두 처리
    tensors: list[torch.Tensor] = []
    if isinstance(output, torch.Tensor):
        tensors = [output]
    elif isinstance(output, (tuple, list)):
        tensors = [o for o in output if isinstance(o, torch.Tensor)]

    for t in tensors:
        if torch.isnan(t).any() or torch.isinf(t).any():
            _nan_detected = True
            print(f"🔥 [오류 포착] '{module.__class__.__name__}' 레이어 연산 중 NaN/Inf 발생!")

def run_exact_verification(ckpt_path: str) -> bool:
    """FP16 추론 건전성을 검증합니다.

    Returns:
        True  – NaN/Inf 미감지 (안전)
        False – NaN/Inf 감지 또는 예외 발생 (불안전)
    """
    global _nan_detected
    _nan_detected = False  # 재실행 시 초기화

    print("🔍 SegmentationPredictor 직접 구동 검증 시작...")

    # FP16 검증은 CUDA가 필수이므로 사전에 명시적으로 확인합니다.
    if not torch.cuda.is_available():
        print("❌ FP16 검증에는 CUDA GPU가 필요합니다. CPU 환경에서는 실행할 수 없습니다.")
        return False

    # 3. ROS 노드와 100% 동일하게 클래스 생성
    # 핵심 파라미터인 inference_precision="fp16"을 넘겨줍니다.
    # 단, Hook(CCTV)이 정상 작동하도록 torch.compile 최적화만 잠시 꺼둡니다.
    predictor = SegmentationPredictor(
        checkpoint_path=ckpt_path,
        inference_precision="fp16",
        use_compile=False
    )
    print(f"✅ SegmentationPredictor 초기화 완료 (Precision: {predictor.inference_precision})")

    # 4. 생성된 모델 내부에 CCTV 부착
    for name, layer in predictor.model.named_modules():
        layer.register_forward_hook(check_nan_hook)
    print("✅ 모델 모든 레이어에 검증용 Hook 부착 완료")

    # 5. 카메라에서 들어오는 실제 데이터와 똑같은 포맷 만들기
    # 노드에서 CvBridge를 거치면 [H, W, 3] 형태의 uint8 Numpy 배열이 됩니다.
    width, height = predictor.image_size
    # 상한을 256으로 수정하여 실제 카메라 데이터의 전 범위(0~255)를 커버합니다.
    dummy_image_cv = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)

    print("\n🚀 predictor.predict() 함수 실행 중...")

    # 6. 실제 추론 실행!
    try:
        # bgr 포맷으로 추론 (노드의 기본 동작)
        result, pure_ms = predictor.predict(dummy_image_cv, color_order="bgr")

        print(f"\n🏁 추론 완료 (순수 추론 시간: {pure_ms:.2f}ms)")
        print(f"   -> 출력 클래스 맵 형태: {result.class_map.shape}")

        # 결과물 자체에도 NaN이 있는지 최종 확인
        if np.isnan(result.class_map).any():
            print("🚨 [출력 오류] 출력된 넘파이 배열에 NaN이 포함되어 있습니다!")
            _nan_detected = True

    except Exception as e:
        print(f"\n🚨 실행 중 치명적 에러 발생: {e}")
        return False

    # Hook 감지 여부를 실제 최종 결론에 반영합니다.
    #   기존: Hook 발화와 무관하게 항상 "안전" 메시지 출력 → 오탐 가능
    #   수정: 플래그 기반으로 정확한 결론을 출력하고 종료 코드를 반환합니다.
    print("\n" + "=" * 50)
    if _nan_detected:
        print("🚨 [결론] NaN/Inf가 감지되었습니다. 현재 FP16 설정은 안전하지 않습니다.")
        print("   -> 위 🔥[오류 포착] 로그를 확인하여 원인 레이어를 점검하세요.")
        return False
    else:
        print("✅ [결론] NaN/Inf 미감지. 현재 코드는 FP16에서 안전합니다.")
        return True

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="FP16 세그멘테이션 모델 NaN/Inf 검증 도구")
    parser.add_argument("--ckpt", type=str, required=True, help="체크포인트 파일 경로")
    args = parser.parse_args()

    if not os.path.exists(args.ckpt):
        print(f"❌ 체크포인트 파일을 찾을 수 없습니다: {args.ckpt}")
        sys.exit(1)

    passed = run_exact_verification(args.ckpt)
    sys.exit(0 if passed else 1)