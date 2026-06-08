#!/usr/bin/env python3
"""FP32 vs FP16 세그멘테이션 결과 Bag 파일 픽셀 단위 비교 도구.

사용법:
    python compare_bags_sync.py \\
        --fp32-bag fp32_result.bag \\
        --fp16-bag fp16_result.bag \\
        [--topic /segmentation/class_map] \\
        [--threshold 99.0] \\
        [--output-dir ./diff_output]
"""
import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge


def _collect_timestamps(bag_path: str, topic: str) -> set:
    """1-pass: 타임스탬프(nsec)만 수집합니다. 메시지 데이터는 보관하지 않습니다."""
    timestamps = {}
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            ts = msg.header.stamp.to_nsec()
            # 동일 타임스탬프 중복 감지 시 경고를 출력합니다.
            if ts in timestamps:
                print(f"  ⚠️  [경고] 중복 타임스탬프 감지 (덮어쓰기): {ts} ns — {bag_path}")
            timestamps[ts] = True
    return set(timestamps.keys())


def _read_messages_for_timestamps(
    bag_path: str, topic: str, target_timestamps: set
) -> dict:
    """2-pass: 필요한 타임스탬프의 메시지만 선택적으로 읽습니다.

    모든 메시지를 메모리에 올리지 않고 필요한 것만 적재하여 OOM을 방지합니다.
    """
    messages = {}
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            ts = msg.header.stamp.to_nsec()
            if ts in target_timestamps:
                messages[ts] = msg
    return messages


def compare_synced_bags(
    fp32_bag_path: str,
    fp16_bag_path: str,
    topic: str,
    threshold: float,
    output_dir: Path,
) -> bool:
    """두 Bag 파일의 세그멘테이션 결과를 픽셀 단위로 비교합니다.

    Args:
        fp32_bag_path: 기준(FP32) Bag 파일 경로.
        fp16_bag_path: 테스트(FP16) Bag 파일 경로.
        topic:         비교 대상 ROS 토픽.
        threshold:     프레임/전체 통과 기준 픽셀 일치율(%).
        output_dir:    차이 이미지 저장 디렉터리.

    Returns:
        True  – 전체 일치율이 threshold 이상 (검증 통과)
        False – threshold 미달 또는 비교 불가
    """
    print(
        f"📂 파일 로딩 중...\n"
        f"  - 기준(FP32): {fp32_bag_path}\n"
        f"  - 테스트(FP16): {fp16_bag_path}\n"
        f"  - 토픽: {topic}\n"
        f"  - 통과 기준: {threshold:.2f}%"
    )

    # ── 1단계: 타임스탬프만 수집 (메모리 효율적인 2-pass 방식) ──────────────
    print("\n⏳ 타임스탬프 수집 중...")
    ts_fp32 = _collect_timestamps(fp32_bag_path, topic)
    ts_fp16 = _collect_timestamps(fp16_bag_path, topic)

    common_timestamps = sorted(ts_fp32.intersection(ts_fp16))

    print(f"\n📊 데이터 통계:")
    print(f"  - FP32 총 프레임: {len(ts_fp32)}")
    print(f"  - FP16 총 프레임: {len(ts_fp16)}")
    print(f"  - 🔗 타임스탬프가 일치하는 공통 프레임: {len(common_timestamps)}개")

    if not common_timestamps:
        print(
            "🚨 오류: 타임스탬프가 일치하는 프레임이 단 하나도 없습니다!\n"
            "   원본 입력 Bag이 서로 다르거나 Header가 유실되었는지 확인하세요."
        )
        return False

    # ── 2단계: 공통 타임스탬프에 해당하는 메시지만 읽기 ─────────────────────
    print("\n⏳ 공통 프레임 데이터 읽는 중 (메모리 절약 모드)...")
    common_set = set(common_timestamps)
    dict_fp32 = _read_messages_for_timestamps(fp32_bag_path, topic, common_set)
    dict_fp16 = _read_messages_for_timestamps(fp16_bag_path, topic, common_set)

    # ── 3단계: 픽셀 단위 비교 ────────────────────────────────────────────────
    bridge = CvBridge()
    total_pixels = 0
    total_matching_pixels = 0
    matching_frames = 0
    skipped_frames = 0

    # 고정 상대경로 대신 절대 경로를 사용하여 실행 위치에 무관하게 동작합니다.
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n🔍 공통 프레임 픽셀 단위 비교 시작... (diff 저장: {output_dir})")

    for i, ts in enumerate(common_timestamps):
        msg_fp32 = dict_fp32.get(ts)
        msg_fp16 = dict_fp16.get(ts)

        if msg_fp32 is None or msg_fp16 is None:
            skipped_frames += 1
            continue

        img_fp32 = bridge.imgmsg_to_cv2(msg_fp32, desired_encoding="passthrough")
        img_fp16 = bridge.imgmsg_to_cv2(msg_fp16, desired_encoding="passthrough")

        if img_fp32.shape != img_fp16.shape:
            print(f"  -> [프레임 {i}] 해상도 불일치로 건너뜀 "
                  f"(FP32:{img_fp32.shape} vs FP16:{img_fp16.shape})")
            skipped_frames += 1
            continue

        # 픽셀 일치도 계산
        match_mask = img_fp32 == img_fp16
        matching_pixels = int(np.sum(match_mask))
        frame_pixels = img_fp32.size

        total_matching_pixels += matching_pixels
        total_pixels += frame_pixels

        accuracy = (matching_pixels / frame_pixels) * 100.0

        # 임계값을 하드코딩(95)에서 인자로 받은 threshold로 대체합니다.
        if accuracy >= threshold:
            matching_frames += 1
        else:
            diff_img = np.abs(
                img_fp32.astype(np.int16) - img_fp16.astype(np.int16)
            ).astype(np.uint8)
            diff_path = output_dir / f"diff_ts_{ts}.png"
            cv2.imwrite(str(diff_path), diff_img * 50)  # 차이를 명확히 보기 위해 50배 증폭
            print(f"  -> [주의] TS:{ts} 프레임 일치율 {accuracy:.4f}% (오차 이미지 저장됨)")

    # ── 4단계: 최종 결과 산출 ─────────────────────────────────────────────────
    compared_frames = len(common_timestamps) - skipped_frames

    # 모든 프레임이 스킵된 경우 ZeroDivisionError 방지
    if total_pixels == 0:
        print(
            "\n🚨 비교 가능한 프레임이 없습니다 (모든 프레임이 해상도 불일치로 건너뜀).\n"
            "   두 Bag의 토픽 인코딩 및 해상도 설정을 확인하세요."
        )
        return False

    overall_accuracy = (total_matching_pixels / total_pixels) * 100.0

    print("\n" + "=" * 50)
    print("🎯 최종 동기화 검증 결과")
    print("=" * 50)
    print(f"  전체 평균 일치율 : {overall_accuracy:.4f}%")
    print(f"  통과 기준        : {threshold:.2f}%")
    print(f"  통과 프레임      : {matching_frames} / {compared_frames}  (건너뜀: {skipped_frames})")

    if overall_accuracy >= threshold:
        print(
            f"\n🚀 [결론] 검증 통과! "
            f"FP16 추론 속도를 누리면서 FP32와 동등한 품질을 달성했습니다."
        )
        return True
    else:
        print(
            f"\n⚠️  [경고] 일치율이 기준치({threshold:.2f}%) 미달입니다.\n"
            f"   {output_dir} 폴더의 이미지를 확인하여 오차 발생 클래스를 분석하세요."
        )
        return False


if __name__ == "__main__":
    # 하드코딩된 경로/토픽/임계값을 모두 CLI 인자로 분리합니다.
    parser = argparse.ArgumentParser(
        description="FP32 vs FP16 세그멘테이션 Bag 파일 픽셀 비교 도구"
    )
    parser.add_argument(
        "--fp32-bag",
        type=str,
        default="fp32_result.bag",
        help="기준(FP32) Bag 파일 경로 (기본값: fp32_result.bag)",
    )
    parser.add_argument(
        "--fp16-bag",
        type=str,
        default="fp16_result.bag",
        help="테스트(FP16) Bag 파일 경로 (기본값: fp16_result.bag)",
    )
    parser.add_argument(
        "--topic",
        type=str,
        default="/segmentation/class_map",
        help="비교 대상 ROS 토픽 (기본값: /segmentation/class_map)",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=99.0,
        help="픽셀 일치율 통과 기준 %% (기본값: 99.0)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="차이 이미지 저장 디렉터리 (기본값: 스크립트 위치/diff_output)",
    )
    args = parser.parse_args()

    # output_dir 미지정 시 스크립트 위치 기준 절대 경로로 설정
    if args.output_dir is None:
        resolved_output_dir = Path(__file__).resolve().parent / "diff_output"
    else:
        resolved_output_dir = Path(args.output_dir).resolve()

    for label, path in [("FP32", args.fp32_bag), ("FP16", args.fp16_bag)]:
        if not os.path.exists(path):
            print(f"❌ {label} Bag 파일을 찾을 수 없습니다: {path}")
            sys.exit(1)

    passed = compare_synced_bags(
        fp32_bag_path=args.fp32_bag,
        fp16_bag_path=args.fp16_bag,
        topic=args.topic,
        threshold=args.threshold,
        output_dir=resolved_output_dir,
    )
    sys.exit(0 if passed else 1)