#!/usr/bin/python3
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create a binary mask from a Labelme JSON polygon."
    )
    parser.add_argument("--json", required=True, help="Path to Labelme JSON file.")
    parser.add_argument(
        "--image",
        default="",
        help=(
            "Optional path to source GT image. If omitted or unreadable, "
            "imageWidth/imageHeight in JSON is used."
        ),
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Path to output mask image (grayscale PNG).",
    )
    parser.add_argument(
        "--label",
        default="car_body",
        help="Polygon label to use for mask filling (default: car_body).",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    json_path = Path(args.json)
    output_path = Path(args.output)
    image_path = Path(args.image) if args.image else None

    with json_path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    h = None
    w = None
    if image_path is not None:
        image = cv2.imread(str(image_path))
        if image is not None:
            h, w = image.shape[:2]
        else:
            print(
                f"warning: image read failed ({image_path}), "
                "falling back to JSON size."
            )

    if h is None or w is None:
        image_h = data.get("imageHeight")
        image_w = data.get("imageWidth")
        if isinstance(image_h, (int, float)) and isinstance(image_w, (int, float)):
            h = int(image_h)
            w = int(image_w)
        else:
            raise ValueError(
                "Cannot determine mask size. Provide --image or include "
                "imageWidth/imageHeight in JSON."
            )

    mask = np.zeros((h, w), dtype=np.uint8)

    filled_count = 0
    for shape in data.get("shapes", []):
        if shape.get("label") != args.label:
            continue

        points = np.array(shape.get("points", []), dtype=np.float32)
        if points.shape[0] < 3:
            continue

        polygon = np.round(points).astype(np.int32)
        cv2.fillPoly(mask, [polygon], 255)
        filled_count += 1

    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), mask)

    print(f"mask saved: {output_path}")
    print(f"label: {args.label}, polygons: {filled_count}, size: {w}x{h}")


if __name__ == "__main__":
    main()
