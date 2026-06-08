#!/usr/bin/env python3
from __future__ import annotations  # Python 3.7+ 호환: 타입 힌트 지연 평가 활성화
import os
import sys
import torch
import numpy as np
from pathlib import Path

# 1. ROS 패키지 경로를 찾아서 추가 (노드와 동일)
def add_segmentation_source_path() -> None:
    # cwd 자체를 먼저 추가한 뒤 부모를 탐색해 중복 탐색을 제거합니다.
    cwd = Path.cwd().resolve()
    search_roots = (
        list(Path(__file__).resolve().parents)
        + [cwd]
        + list(cwd.parents)
    )
    for prefix in os.environ.get("CATKIN_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            p = Path(prefix).resolve()
            # prefix 자신과 그 부모를 모두 탐색합니다.
            search_roots.extend([p] + list(p.parents))

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


# ── 전역 상태 ──────────────────────────────────────────────────────────────
# Hook이 감지한 이벤트를 누적합니다.
# 형식: [(layer_name, nan_count, inf_count), ...]
#
# [설계 노트] 이 전역 상태는 스크립트 단독 실행 전용입니다.
# 멀티스레드 환경에서 재사용할 경우 threading.local()로 격리해야 합니다.
_hook_events: list[tuple[str, int, int]] = []


def _reset_hook_state() -> None:
    """시도(trial) 시작 전에 훅 누적 기록을 초기화합니다."""
    _hook_events.clear()


def _hook_detected() -> bool:
    """이번 시도에서 NaN 또는 Inf가 하나라도 감지됐는지 반환합니다."""
    return bool(_hook_events)


def check_nan_hook(module, input, output) -> None:
    """모델 내부 레이어 출력에서 NaN/Inf를 검사하는 Forward Hook.

    - 단일 텐서와 tuple/list 출력(Attention 등)을 모두 처리합니다.
    - NaN 픽셀 수와 Inf 픽셀 수를 각각 기록하여 상세 진단을 지원합니다.
    - [주의] leaf 모듈에만 등록해야 중복 검사를 방지할 수 있습니다.
    """
    tensors: list[torch.Tensor] = []
    if isinstance(output, torch.Tensor):
        tensors = [output]
    elif isinstance(output, (tuple, list)):
        tensors = [o for o in output if isinstance(o, torch.Tensor)]

    for t in tensors:
        nan_count = int(torch.isnan(t).sum())
        inf_count = int(torch.isinf(t).sum())
        if nan_count > 0 or inf_count > 0:
            layer_name = module.__class__.__name__
            _hook_events.append((layer_name, nan_count, inf_count))
            parts = []
            if nan_count:
                parts.append(f"NaN×{nan_count}")
            if inf_count:
                parts.append(f"Inf×{inf_count}")
            print(f"    🔥 [Hook] '{layer_name}' → {', '.join(parts)}")


def run_exact_verification(
    ckpt_path: str,
    batch_size: int = 1,
    trials: int = 1,
    seed: int | None = None,
) -> bool:
    """FP16 추론 건전성을 검증합니다.

    Args:
        ckpt_path:  체크포인트 파일 경로.
        batch_size: 한 번의 추론에 입력할 이미지 수.
                    배치 추론(batch_size > 1) 경로도 검증할 수 있습니다.
        trials:     추론 시도 횟수. 매 시도마다 새로운 랜덤 배치를 생성하여
                    확률적으로 나타나는 NaN을 더 신뢰성 있게 탐지합니다.
        seed:       NumPy 랜덤 시드. 지정하면 동일한 입력으로 재현 가능한
                    검증을 수행합니다. None이면 매 실행마다 다른 배치를 사용합니다.

    Returns:
        True  – 모든 시도에서 NaN/Inf 미감지 (안전)
        False – NaN/Inf 감지 또는 예외 발생 (불안전)
    """
    print(
        f"🔍 SegmentationPredictor 직접 구동 검증 시작..."
        f" (배치 사이즈: {batch_size}, 시도 횟수: {trials}회"
        + (f", 시드: {seed}" if seed is not None else "")
        + ")"
    )

    # FP16 검증은 CUDA가 필수이므로 사전에 명시적으로 확인합니다.
    if not torch.cuda.is_available():
        print("❌ FP16 검증에는 CUDA GPU가 필요합니다. CPU 환경에서는 실행할 수 없습니다.")
        return False

    # 시드를 지정하면 재현 가능한 배치를 생성합니다.
    rng = np.random.default_rng(seed)

    # 3. ROS 노드와 100% 동일하게 클래스 생성
    # 핵심 파라미터인 inference_precision="fp16"을 넘겨줍니다.
    # 단, Hook(CCTV)이 정상 작동하도록 torch.compile 최적화만 잠시 꺼둡니다.
    predictor = SegmentationPredictor(
        checkpoint_path=ckpt_path,
        inference_precision="fp16",
        use_compile=False
    )
    print(f"✅ SegmentationPredictor 초기화 완료 (Precision: {predictor.inference_precision})")

    num_classes = len(predictor.class_names)

    # 4. 생성된 모델 내부에 CCTV 부착
    # named_modules()는 상위/하위 모듈을 재귀적으로 반환하므로
    # 자식이 없는 leaf 모듈에만 Hook을 등록하여 중복 검사를 방지합니다.
    hook_count = 0
    for name, layer in predictor.model.named_modules():
        if not list(layer.children()):  # leaf 모듈만
            layer.register_forward_hook(check_nan_hook)
            hook_count += 1
    print(f"✅ Leaf 레이어 {hook_count}개에 검증용 Hook 부착 완료")

    width, height = predictor.image_size
    elapsed_times: list[float] = []

    # trial별 pass/fail 요약 (True=통과, False=실패)
    trial_results: list[bool] = []
    fatal_error: Exception | None = None

    print(f"\n🚀 predictor.predict_batch() 반복 실행 중 ({trials}회 × 배치 {batch_size}장)...")
    trial_w = len(str(trials))  # 출력 정렬용 너비

    # 5. trials 횟수만큼 반복 추론
    for trial in range(1, trials + 1):
        # 매 시도마다 서로 다른 랜덤 배치를 생성합니다.
        # (상한 256: 실제 카메라 데이터의 전 범위 0~255 커버)
        dummy_batch = [
            rng.integers(0, 256, (height, width, 3), dtype=np.uint8)
            for _ in range(batch_size)
        ]

        # Hook 누적 기록 초기화 (시도 단위로 구분)
        _reset_hook_state()
        trial_failed = False

        try:
            # bgr 포맷으로 추론 (노드의 기본 동작)
            results, pure_ms = predictor.predict_batch(dummy_batch, color_order="bgr")
            elapsed_times.append(pure_ms)

            print(f"  [{trial:{trial_w}}/{trials}] "
                  f"추론 완료 — 순수 추론 시간: {pure_ms:7.2f}ms  "
                  f"출력 형태: {results[0].class_map.shape}")

            # ── Hook 이벤트 요약 ────────────────────────────────────────────
            # (감지된 레이어/카운트는 check_nan_hook 내에서 즉시 출력됨)
            if _hook_detected():
                trial_failed = True

            # ── 각 결과물(출력 배열) 유효성 검사 ─────────────────────────
            # uint8은 IEEE 754 부동소수점이 아니므로 np.isnan은 항상 False입니다.
            # NaN은 Forward Hook이 float 단계에서 이미 포착합니다.
            # 대신 class_map의 값이 유효한 클래스 인덱스 범위 내에 있는지 검사합니다.
            for img_idx, res in enumerate(results):
                cm = res.class_map  # np.ndarray, dtype=uint8
                invalid_pixels = int((cm >= num_classes).sum())

                if invalid_pixels:
                    trial_failed = True
                    print(f"    🚨 [출력 오류] 시도 {trial}, 이미지 {img_idx}: "
                          f"class_map에 범위 초과 픽셀 {invalid_pixels}px 포함! "
                          f"(유효 범위: 0~{num_classes - 1})")
                else:
                    print(f"    ✅ [출력 OK]   시도 {trial}, 이미지 {img_idx}: "
                          f"클래스 인덱스 정상 (0~{num_classes - 1})")

        except Exception as e:
            # return False 대신 trial_results에 실패를 기록하고 break합니다.
            # 이렇게 하면 앞선 시도의 추론 시간 통계를 요약 섹션에서 확인할 수 있습니다.
            print(f"\n🚨 시도 {trial} 중 치명적 에러 발생: {e}")
            trial_results.append(False)
            fatal_error = e
            break

        trial_results.append(not trial_failed)

        # 실패가 확정되면 조기 종료 (남은 시도 불필요)
        if trial_failed:
            print(f"  -> 시도 {trial} 실패. 조기 종료합니다.")
            break

    # ── 6. 최종 요약 ──────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("📋 검증 결과 요약")
    print("=" * 60)

    # 추론 시간 통계
    if elapsed_times:
        avg_ms = sum(elapsed_times) / len(elapsed_times)
        min_ms = min(elapsed_times)
        max_ms = max(elapsed_times)
        print(f"  추론 시간   │ 평균 {avg_ms:.2f}ms │ "
              f"최소 {min_ms:.2f}ms │ 최대 {max_ms:.2f}ms")
        print(f"  [주의] Hook 활성 상태의 시간입니다. 실제 운영 시간과 다를 수 있습니다.")

    # 시도별 pass/fail 테이블
    print(f"  {'시도':<{trial_w+4}} │ 결과")
    print(f"  {'-'*(trial_w+4)}-│------")
    for i, passed_trial in enumerate(trial_results, start=1):
        mark = "✅ PASS" if passed_trial else "❌ FAIL"
        print(f"  시도 {i:{trial_w}}    │ {mark}")

    # 치명적 에러 내용 재출력
    if fatal_error is not None:
        print(f"\n  ⚠️  치명적 에러: {fatal_error}")

    # 전체 판정 — 조기 종료된 경우 미실행 시도는 이미 trial_results에 없으므로
    # 하나라도 False면 실패로 간주합니다.
    all_passed = all(trial_results) and bool(trial_results)
    print("=" * 60)
    if all_passed:
        print(f"✅ [결론] {trials}회 × 배치 {batch_size}장 전부 이상 없음."
              f" 현재 코드는 FP16에서 안전합니다.")
    else:
        fail_count = trial_results.count(False)
        print(f"🚨 [결론] {fail_count}회 실패 감지. FP16 설정이 안전하지 않습니다.")
        print("   -> 위 🔥[Hook] / 🚨[출력 오류] 로그에서 원인 레이어를 확인하세요.")
    return all_passed

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="FP16 세그멘테이션 모델 NaN/Inf 검증 도구")
    parser.add_argument("--ckpt", type=str, required=True,
                        help="체크포인트 파일 경로")
    parser.add_argument("--batch-size", type=int, default=1,
                        help="한 번의 추론에 입력할 이미지 수 (기본값: 1)")
    parser.add_argument("--trials", type=int, default=1,
                        help="추론 시도 횟수. 매번 새 랜덤 배치를 생성합니다 (기본값: 1)")
    parser.add_argument("--seed", type=int, default=None,
                        help="NumPy 랜덤 시드. 지정 시 동일한 입력으로 재현 가능한 검증을 수행합니다.")
    args = parser.parse_args()

    if not os.path.exists(args.ckpt):
        print(f"❌ 체크포인트 파일을 찾을 수 없습니다: {args.ckpt}")
        sys.exit(1)

    if args.batch_size < 1:
        print("❌ --batch-size는 1 이상이어야 합니다.")
        sys.exit(1)

    if args.trials < 1:
        print("❌ --trials는 1 이상이어야 합니다..")
        sys.exit(1)

    passed = run_exact_verification(
        ckpt_path=args.ckpt,
        batch_size=args.batch_size,
        trials=args.trials,
        seed=args.seed,
    )
    sys.exit(0 if passed else 1)