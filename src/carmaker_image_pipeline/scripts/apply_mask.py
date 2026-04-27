#!/usr/bin/python3
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = "/workspace/src/carmaker_image_pipeline/data"
DEFAULT_MASK_DIR = DATA_ROOT + "/mask"
DEFAULT_GT_DIR = DATA_ROOT + "/gt_images"
DEFAULT_GT_POST_DIR = DATA_ROOT + "/gt_post_processed"

CLASS_BACKGROUND = 0
CLASS_LANE_BLACK = 1
CLASS_LANDMARK_YELLOW = 2


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Automatically apply car-body masking to GT images using per-camera "
            "Labelme JSON files."
        )
    )
    parser.add_argument(
        "--input-dir",
        default=str(DEFAULT_GT_DIR),
        help=f"Directory with GT images (default: {DEFAULT_GT_DIR}).",
    )
    parser.add_argument(
        "--mask-dir",
        default=str(DEFAULT_MASK_DIR),
        help=f"Directory with camera JSON/mask files (default: {DEFAULT_MASK_DIR}).",
    )
    parser.add_argument(
        "--json-dir",
        default="",
        help="Legacy alias for --mask-dir. If set, it overrides --mask-dir.",
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_GT_POST_DIR),
        help=f"Directory to save post-processed GT images (default: {DEFAULT_GT_POST_DIR}).",
    )
    parser.add_argument(
        "--image-glob",
        default="*.png",
        help="Glob pattern for GT images (default: *.png).",
    )
    parser.add_argument(
        "--cameras",
        default="front,left,rear,right",
        help="Comma-separated camera names (default: front,left,rear,right).",
    )
    parser.add_argument(
        "--json-pattern",
        default="{camera}_GT.json",
        help="JSON filename pattern (default: {camera}_GT.json).",
    )
    parser.add_argument(
        "--label",
        default="car_body",
        help="Polygon label to fill (default: car_body).",
    )
    parser.add_argument(
        "--mask-pattern",
        default="{camera}_mask.png",
        help="Precomputed mask filename pattern in mask-dir (default: {camera}_mask.png).",
    )
    parser.add_argument(
        "--refresh-masks",
        action="store_true",
        help="Rebuild camera masks from JSON even when precomputed masks exist.",
    )
    parser.add_argument(
        "--suffix",
        default="_post",
        help="Suffix added to final post-processed output filename (default: _post).",
    )
    parser.add_argument(
        "--lane-threshold",
        type=int,
        default=40,
        help="Upper threshold for black lane detection (0~255, default: 40).",
    )
    parser.add_argument(
        "--landmark-gray-min",
        type=int,
        default=160,
        help="Lower grayscale threshold for landmark detection (default: 160).",
    )
    parser.add_argument(
        "--landmark-gray-max",
        type=int,
        default=165,
        help="Upper grayscale threshold for landmark detection (default: 165).",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Search images recursively under input-dir.",
    )
    return parser.parse_args()


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


def load_json(json_path):
    with json_path.open("r", encoding="utf-8") as file:
        return json.load(file)


def create_mask_from_labelme(data, height, width, label):
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


def find_images(input_dir, image_glob, recursive):
    if recursive:
        return sorted(input_dir.rglob(image_glob))
    return sorted(input_dir.glob(image_glob))


def classify_post_processed_image(
    image_gray,
    car_mask,
    lane_threshold,
    landmark_gray_min,
    landmark_gray_max,
):
    class_map = np.zeros(image_gray.shape[:2], dtype=np.uint8)

    lane_mask = image_gray <= lane_threshold
    landmark_mask = (image_gray >= landmark_gray_min) & (
        image_gray <= landmark_gray_max
    )

    if car_mask is not None:
        masked_region = car_mask > 0
        lane_mask[masked_region] = False
        landmark_mask[masked_region] = False

    class_map[lane_mask] = CLASS_LANE_BLACK
    class_map[landmark_mask] = CLASS_LANDMARK_YELLOW
    return class_map


def encode_class_map_to_output(class_map):
    """
    Map class ids to output grayscale values:
        class 0 -> other
        class 1 -> lane
        class 2 -> landmark
    """
    output = np.zeros(class_map.shape, dtype=np.uint8)
    output[class_map == CLASS_LANE_BLACK] = CLASS_LANE_BLACK
    output[class_map == CLASS_LANDMARK_YELLOW] = CLASS_LANDMARK_YELLOW
    return output


def create_or_load_camera_mask(
    camera,
    image_shape,
    mask_dir,
    json_pattern,
    label,
    mask_pattern,
    refresh_masks,
    json_cache,
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
            json_cache[cache_key],
            image_shape[0],
            image_shape[1],
            label,
        )

        if polygons == 0:
            return None, polygons, f"no '{label}' polygon in {json_path.name}"

        mask_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(mask_path), mask)
        source = "generated_from_json"

    if mask.shape[:2] != image_shape[:2]:
        mask = cv2.resize(
            mask, (image_shape[1], image_shape[0]), interpolation=cv2.INTER_NEAREST
        )

    return mask, polygons, source


def main():
    args = parse_args()

    input_dir = Path(args.input_dir)
    mask_dir = Path(args.mask_dir)
    if args.json_dir:
        mask_dir = Path(args.json_dir)
    output_dir = Path(args.output_dir)
    if not input_dir.exists():
        raise FileNotFoundError(f"Input directory not found: {input_dir}")
    if not mask_dir.exists():
        raise FileNotFoundError(f"Mask directory not found: {mask_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    cameras = [camera.strip() for camera in args.cameras.split(",") if camera.strip()]

    images = find_images(input_dir, args.image_glob, args.recursive)
    if not images:
        print(
            f"No images found: dir={input_dir}, glob={args.image_glob}, recursive={args.recursive}"
        )
        return

    json_cache = {}
    camera_masks = {}
    processed = 0
    skipped = 0

    for image_path in images:
        image = cv2.imread(str(image_path))
        if image is None:
            print(f"[skip] image read failed: {image_path}")
            skipped += 1
            continue
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        camera = detect_camera(image_path.stem, cameras)
        if camera is None:
            print(f"[skip] camera not detected from filename: {image_path.name}")
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
                    mask,
                    (image.shape[1], image.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )

        if polygon_count == 0:
            print(f"[skip] no '{args.label}' polygon for camera={camera}")
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
            f"[ok] {image_path.name} -> {output_path.name} "
            f"(camera={camera}, mask={reason}, polygons={polygon_count})"
        )
        processed += 1

    print(f"done. processed={processed}, skipped={skipped}, output_dir={output_dir}")


if __name__ == "__main__":
    main()
