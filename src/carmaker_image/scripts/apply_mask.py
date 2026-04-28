#!/usr/bin/python3
"""GT 이미지 후처리 및 마스크 생성 스크립트.

주요 기능:
    1. 일반 실행: GT 이미지에 차체 마스크를 적용하여 클래스맵(*_post.png) 생성
    2. --generate-only: 마스크 생성만 수행 (GT 이미지 처리 없음)
        make_mask.py의 기능을 완전히 대체합니다.

    사용 예:
    # GT 후처리 (일반)
    python3 apply_mask.py --suffix _post

    # 단일 JSON → 마스크 생성만
    python3 apply_mask.py --generate-only --json data/mask/front_GT.json --output data/mask/front_mask.png

    # 전체 카메라 마스크 일괄 생성
    python3 apply_mask.py --generate-only --cameras front,left,rear,right

    # 재귀 탐색 (batch_extract 이후)
    python3 apply_mask.py --suffix _post --recursive
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = PACKAGE_ROOT / "data"
DEFAULT_MASK_DIR = DATA_ROOT / "mask"
DEFAULT_GT_DIR = DATA_ROOT / "gt_images"
DEFAULT_GT_POST_DIR = DATA_ROOT / "gt_post_processed"

CLASS_BACKGROUND = 0
CLASS_LANE_BLACK = 1
CLASS_LANDMARK_YELLOW = 2


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "GT 이미지 후처리 및 카메라 마스크 생성.\n"
            "--generate-only 플래그를 사용하면 마스크 생성만 수행합니다 "
            "(구 make_mask.py 대체)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # ── 모드 선택 ──────────────────────────────────────────────────────────
    parser.add_argument(
        "--generate-only",
        action="store_true",
        help=(
            "마스크 생성만 수행하고 종료합니다 (GT 이미지 처리 없음). "
            "--json 또는 --cameras 와 함께 사용."
        ),
    )

    # ── generate-only 전용 옵션 ────────────────────────────────────────────
    gen_group = parser.add_argument_group("generate-only 전용 옵션")
    gen_group.add_argument(
        "--json",
        default="",
        help=(
            "단일 Labelme JSON 파일 경로. "
            "--generate-only 와 함께 사용 시 해당 파일 하나만 처리. "
            "비우면 --cameras 기준으로 mask-dir에서 자동 탐색."
        ),
    )
    gen_group.add_argument(
        "--output",
        default="",
        help="마스크 출력 경로 (--json 과 함께 사용 시). 비우면 mask-dir/{camera}_mask.png.",
    )
    gen_group.add_argument(
        "--image",
        default="",
        help=(
            "선택적 원본 이미지 경로. 제공되면 이미지 크기를 사용; "
            "없으면 JSON의 imageWidth/imageHeight 사용."
        ),
    )

    # ── 공통 옵션 ──────────────────────────────────────────────────────────
    parser.add_argument(
        "--input-dir",
        default=str(DEFAULT_GT_DIR),
        help=f"GT 이미지 디렉토리 (기본: {DEFAULT_GT_DIR}).",
    )
    parser.add_argument(
        "--mask-dir",
        default=str(DEFAULT_MASK_DIR),
        help=f"JSON/마스크 파일 디렉토리 (기본: {DEFAULT_MASK_DIR}).",
    )
    parser.add_argument(
        "--json-dir",
        default="",
        help="--mask-dir 의 레거시 별칭. 설정 시 --mask-dir 을 대체.",
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_GT_POST_DIR),
        help=f"후처리 결과 저장 디렉토리 (기본: {DEFAULT_GT_POST_DIR}).",
    )
    parser.add_argument(
        "--image-glob",
        default="*.png",
        help="GT 이미지 glob 패턴 (기본: *.png).",
    )
    parser.add_argument(
        "--cameras",
        default="front,left,rear,right",
        help="쉼표 구분 카메라 이름 (기본: front,left,rear,right).",
    )
    parser.add_argument(
        "--json-pattern",
        default="{camera}_GT.json",
        help="JSON 파일명 패턴 (기본: {camera}_GT.json).",
    )
    parser.add_argument(
        "--label",
        default="car_body",
        help="마스크로 채울 polygon 레이블 (기본: car_body).",
    )
    parser.add_argument(
        "--mask-pattern",
        default="{camera}_mask.png",
        help="사전 생성된 마스크 파일명 패턴 (기본: {camera}_mask.png).",
    )
    parser.add_argument(
        "--refresh-masks",
        action="store_true",
        help="기존 마스크가 있어도 JSON에서 재생성.",
    )
    parser.add_argument(
        "--suffix",
        default="_post",
        help="후처리 결과 파일명 접미사 (기본: _post).",
    )
    parser.add_argument(
        "--lane-threshold",
        type=int,
        default=40,
        help="차선(검정) 감지 임계값 0~255 (기본: 40).",
    )
    parser.add_argument(
        "--landmark-gray-min",
        type=int,
        default=160,
        help="랜드마크 grayscale 최솟값 (기본: 160).",
    )
    parser.add_argument(
        "--landmark-gray-max",
        type=int,
        default=165,
        help="랜드마크 grayscale 최댓값 (기본: 165).",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="input-dir 하위 폴더를 재귀 탐색 (batch_extract 이후 사용).",
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# 공통 헬퍼
# ---------------------------------------------------------------------------

def load_json(json_path):
    with json_path.open("r", encoding="utf-8") as f:
        return json.load(f)


def create_mask_from_labelme(data, height, width, label):
    """Labelme JSON shapes에서 지정 레이블의 polygon을 255(제거 영역)로, 나머지를 0(유효 영역)으로 만듭니다."""
    mask = np.zeros((height, width), dtype=np.uint8)
    polygon_count = 0
    for shape in data.get("shapes", []):
        if shape.get("label") != label:
            continue
        points = np.array(shape.get("points", []), dtype=np.float32)
        if points.shape[0] < 3:
            continue
        polygon = np.round(points).astype(np.int32)
        cv2.fillPoly(mask, [polygon], 255)
        polygon_count += 1
    return mask, polygon_count


def resolve_image_size(image_path_str, json_data):
    """이미지 파일 또는 JSON에서 (height, width)를 반환합니다."""
    if image_path_str:
        img = cv2.imread(image_path_str)
        if img is not None:
            return img.shape[:2]
        print(f"[warn] 이미지 읽기 실패 ({image_path_str}), JSON 크기 사용.")

    image_h = json_data.get("imageHeight")
    image_w = json_data.get("imageWidth")
    if isinstance(image_h, (int, float)) and isinstance(image_w, (int, float)):
        return int(image_h), int(image_w)

    raise ValueError(
        "마스크 크기를 결정할 수 없습니다. "
        "--image 를 제공하거나 JSON에 imageWidth/imageHeight 를 포함하세요."
    )


# ---------------------------------------------------------------------------
# generate-only 모드
# ---------------------------------------------------------------------------

def run_generate_only(args):
    """마스크 생성만 수행 (구 make_mask.py 대체)."""
    mask_dir = Path(args.mask_dir)
    cameras = [c.strip() for c in args.cameras.split(",") if c.strip()]

    # 단일 JSON 지정 모드
    if args.json:
        json_path = Path(args.json)
        if not json_path.exists():
            raise FileNotFoundError(f"JSON 파일을 찾을 수 없습니다: {json_path}")

        data = load_json(json_path)
        h, w = resolve_image_size(args.image, data)
        mask, count = create_mask_from_labelme(data, h, w, args.label)

        out_path = Path(args.output) if args.output else mask_dir / f"{json_path.stem}_mask.png"
        out_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out_path), mask)
        print(f"[ok] 마스크 저장: {out_path}")
        print(f"     label={args.label}, polygons={count}, size={w}x{h}")
        return

    # 카메라별 일괄 생성 모드
    if not mask_dir.exists():
        raise FileNotFoundError(f"mask-dir 을 찾을 수 없습니다: {mask_dir}")

    json_pattern = args.json_pattern
    mask_pattern = args.mask_pattern
    ok_count = 0
    skip_count = 0

    for camera in cameras:
        json_path = mask_dir / json_pattern.format(camera=camera)
        # fallback
        if not json_path.exists():
            fallback = mask_dir / f"{camera}.json"
            if fallback.exists():
                json_path = fallback
            else:
                print(f"[skip] JSON 없음: {json_path}")
                skip_count += 1
                continue

        mask_path = mask_dir / mask_pattern.format(camera=camera)
        if mask_path.exists() and not args.refresh_masks:
            print(f"[skip] 마스크 이미 존재 (--refresh-masks 로 재생성): {mask_path.name}")
            skip_count += 1
            continue

        data = load_json(json_path)
        # 크기는 JSON에서 읽음 (이미지 없이)
        try:
            h, w = resolve_image_size("", data)
        except ValueError as e:
            print(f"[skip] {camera}: {e}")
            skip_count += 1
            continue

        mask, count = create_mask_from_labelme(data, h, w, args.label)
        if count == 0:
            print(f"[skip] {camera}: '{args.label}' polygon 없음")
            skip_count += 1
            continue

        mask_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(mask_path), mask)
        print(f"[ok] {camera}: {mask_path.name}  (polygons={count}, size={w}x{h})")
        ok_count += 1

    print(f"done. generated={ok_count}, skipped={skip_count}")


# ---------------------------------------------------------------------------
# 일반 GT 후처리 모드
# ---------------------------------------------------------------------------

def detect_camera(stem_name, cameras):
    name = stem_name.lower()
    for camera in cameras:
        cam = camera.lower()
        if (
            name == cam
            or name.startswith(f"{cam}_")
            or name.startswith(f"{cam}-")
            or f"_{cam}_" in name
            or name.endswith(f"_{cam}")
            or name.endswith(f"-{cam}")
        ):
            return camera
    return None


def find_images(input_dir, image_glob, recursive):
    if recursive:
        return sorted(input_dir.rglob(image_glob))
    return sorted(input_dir.glob(image_glob))


def classify_post_processed_image(
    image_gray, car_mask, lane_threshold, landmark_gray_min, landmark_gray_max
):
    # 1. cv2.inRange를 활용한 빠른 임계값 필터링 (결과는 0 또는 255의 uint8 배열)
    lane_mask = cv2.inRange(image_gray, 0, lane_threshold)
    landmark_mask = cv2.inRange(image_gray, landmark_gray_min, landmark_gray_max)

    # 2. cv2.bitwise 연산을 이용한 차체 마스킹 제거
    if car_mask is not None:
        # 차체 마스크(제거 영역=255)를 반전시켜 유효 영역을 255로 만듦
        inv_car_mask = cv2.bitwise_not(car_mask)
        # 차선/랜드마크 마스크와 유효 영역을 AND 연산
        lane_mask = cv2.bitwise_and(lane_mask, inv_car_mask)
        landmark_mask = cv2.bitwise_and(landmark_mask, inv_car_mask)

    # 3. 클래스 맵에 매핑
    class_map = np.zeros(image_gray.shape[:2], dtype=np.uint8)
    class_map[lane_mask > 0] = CLASS_LANE_BLACK
    class_map[landmark_mask > 0] = CLASS_LANDMARK_YELLOW
    return class_map


def encode_class_map_to_output(class_map):
    """클래스 ID를 출력 그레이스케일 값으로 매핑.

    class 0 → background
    class 1 → lane
    class 2 → landmark
    """
    output = np.zeros(class_map.shape, dtype=np.uint8)
    output[class_map == CLASS_LANE_BLACK] = CLASS_LANE_BLACK
    output[class_map == CLASS_LANDMARK_YELLOW] = CLASS_LANDMARK_YELLOW
    return output


def create_or_load_camera_mask(
    camera, image_shape, mask_dir, json_pattern, label, mask_pattern,
    refresh_masks, json_cache,
):
    mask_path = mask_dir / mask_pattern.format(camera=camera)
    mask = None
    polygons = -1
    source = ""

    if (not refresh_masks) and mask_path.exists():
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if mask is not None:
            source = "precomputed"

    if mask is None:
        json_path = mask_dir / json_pattern.format(camera=camera)
        if not json_path.exists():
            fallback = mask_dir / f"{camera}.json"
            if fallback.exists():
                json_path = fallback
            else:
                return None, 0, f"json not found for camera={camera}: {json_path}"

        cache_key = str(json_path.resolve())
        if cache_key not in json_cache:
            json_cache[cache_key] = load_json(json_path)

        mask, polygons = create_mask_from_labelme(
            json_cache[cache_key], image_shape[0], image_shape[1], label,
        )
        if polygons == 0:
            return None, polygons, f"no '{label}' polygon in {json_path.name}"

        mask_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(mask_path), mask)
        source = "generated_from_json"

    if mask.shape[:2] != image_shape[:2]:
        mask = cv2.resize(mask, (image_shape[1], image_shape[0]), interpolation=cv2.INTER_NEAREST)

    return mask, polygons, source


def run_gt_processing(args):
    """GT 이미지 후처리 (차체 마스킹 + 클래스 분류)."""
    input_dir = Path(args.input_dir)
    mask_dir = Path(args.mask_dir)
    if args.json_dir:
        mask_dir = Path(args.json_dir)
    output_dir = Path(args.output_dir)

    if not input_dir.exists():
        raise FileNotFoundError(f"입력 디렉토리를 찾을 수 없습니다: {input_dir}")
    if not mask_dir.exists():
        raise FileNotFoundError(f"마스크 디렉토리를 찾을 수 없습니다: {mask_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    cameras = [c.strip() for c in args.cameras.split(",") if c.strip()]
    images = find_images(input_dir, args.image_glob, args.recursive)

    if not images:
        print(f"이미지를 찾을 수 없습니다: dir={input_dir}, glob={args.image_glob}, recursive={args.recursive}")
        return

    json_cache = {}
    camera_masks = {}
    processed = 0
    skipped = 0

    for image_path in images:
        image = cv2.imread(str(image_path))
        if image is None:
            print(f"[skip] 이미지 읽기 실패: {image_path}")
            skipped += 1
            continue
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        camera = detect_camera(image_path.stem, cameras)
        if camera is None:
            print(f"[skip] 파일명에서 카메라를 감지할 수 없습니다: {image_path.name}")
            skipped += 1
            continue

        if camera not in camera_masks:
            mask, polygon_count, reason = create_or_load_camera_mask(
                camera=camera,
                image_shape=image.shape,
                mask_dir=mask_dir,
                json_pattern=args.json_pattern,
                label=args.label,
                mask_pattern=args.mask_pattern,
                refresh_masks=args.refresh_masks,
                json_cache=json_cache,
            )
            if mask is None:
                print(f"[skip] {reason}")
                skipped += 1
                continue
            camera_masks[camera] = (mask, reason, polygon_count)
        else:
            base_mask, reason, polygon_count = camera_masks[camera]
            mask = base_mask
            if mask.shape[:2] != image.shape[:2]:
                mask = cv2.resize(
                    mask, (image.shape[1], image.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )

        if polygon_count == 0:
            print(f"[skip] '{args.label}' polygon 없음: camera={camera}")
            skipped += 1
            continue

        class_map = classify_post_processed_image(
            image_gray,
            car_mask=mask,
            lane_threshold=args.lane_threshold,
            landmark_gray_min=args.landmark_gray_min,
            landmark_gray_max=args.landmark_gray_max,
        )

        output_name = f"{image_path.stem}{args.suffix}{image_path.suffix}"
        output_path = output_dir / output_name
        encoded_output = encode_class_map_to_output(class_map)
        cv2.imwrite(str(output_path), encoded_output)

        print(
            f"[ok] {image_path.name} → {output_path.name} "
            f"(camera={camera}, mask={reason}, polygons={polygon_count})"
        )
        processed += 1

    print(f"done. processed={processed}, skipped={skipped}, output_dir={output_dir}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    if args.generate_only:
        run_generate_only(args)
    else:
        run_gt_processing(args)


if __name__ == "__main__":
    main()
