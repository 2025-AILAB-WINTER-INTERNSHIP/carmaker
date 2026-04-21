#!/usr/bin/python3
import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_color(color_text):
    parts = [p.strip() for p in color_text.split(",")]
    if len(parts) != 3:
        raise ValueError("Color must have 3 values in B,G,R format. Example: 224,224,224")

    bgr = tuple(int(value) for value in parts)
    if any(channel < 0 or channel > 255 for channel in bgr):
        raise ValueError("Each color channel must be between 0 and 255.")
    return bgr


def parse_args():
    parser = argparse.ArgumentParser(
        description="Apply a binary mask onto a GT image by replacing masked pixels."
    )
    parser.add_argument("--image", required=True, help="Path to GT image.")
    parser.add_argument("--mask", required=True, help="Path to binary mask image.")
    parser.add_argument("--output", required=True, help="Path to output masked image.")
    parser.add_argument(
        "--mode",
        choices=["fill", "or"],
        default="fill",
        help="Mask apply mode: fill or or (default: fill).",
    )
    parser.add_argument(
        "--color",
        default="255,255,255",
        help="Fill color in B,G,R (default: 255,255,255).",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    fill_color = parse_color(args.color)

    image_path = Path(args.image)
    mask_path = Path(args.mask)
    output_path = Path(args.output)

    image = cv2.imread(str(image_path))
    if image is None:
        raise FileNotFoundError(f"Image read failed: {image_path}")

    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise FileNotFoundError(f"Mask read failed: {mask_path}")

    if mask.shape[:2] != image.shape[:2]:
        mask = cv2.resize(mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)

    mask_binary = np.where(mask > 0, 255, 0).astype(np.uint8)
    if args.mode == "or":
        mask_3ch = cv2.cvtColor(mask_binary, cv2.COLOR_GRAY2BGR)
        result = cv2.bitwise_or(image, mask_3ch)
    else:
        result = image.copy()
        result[mask_binary > 0] = np.array(fill_color, dtype=np.uint8)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), result)
    print(f"masked image saved: {output_path}")


if __name__ == "__main__":
    main()
