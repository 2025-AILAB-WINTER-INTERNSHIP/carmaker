#!/usr/bin/python3
"""Extract raw/GT images from a single ROS bag file.

Usage:
    1. CLI:     python3 extract_bag_images.py --bag <path>
    2. Import:  from extract_bag_images import extract_single_bag
"""

import argparse
import csv
import re
import sys
from pathlib import Path

import cv2
import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = PACKAGE_ROOT / "data"
DEFAULT_RAW_DIR = DATA_ROOT / "raw_images"
DEFAULT_RAW_POST_DIR = DATA_ROOT / "raw_post_processed"
DEFAULT_RAW_POST_SUFFIX = "_post"
DEFAULT_GT_DIR = DATA_ROOT / "gt_images"
DEFAULT_CSV_DIR = DATA_ROOT / "csv"
DEFAULT_GT_POST_DIR = DATA_ROOT / "gt_post_processed"
DEFAULT_GT_POST_SUFFIX = "_post"


def _get_argv_for_rospy():
    """Get command-line arguments, accounting for ROS node initialization.

    When running as a ROS node, rospy.myargv() strips ROS-specific args.
    When running standalone, fall back to sys.argv.

    Returns:
        list: Arguments suitable for argparse (excludes script name)
    """
    try:
        import rospy

        return rospy.myargv()[1:]
    except Exception:
        return sys.argv[1:]


def parse_args(argv=None):
    """Parse extract_bag_images arguments.

    Args:
        argv: Command-line arguments. If None, uses ROS-aware argument detection.
              Pass explicit list to override (e.g., for testing).
    """
    p = argparse.ArgumentParser(description="Extract raw/GT images from ROS bag.")
    p.add_argument("--bag", required=True, help="Input ROS bag path.")
    p.add_argument("--raw-out-dir", default=str(DEFAULT_RAW_DIR))
    p.add_argument("--raw-post-dir", default=str(DEFAULT_RAW_POST_DIR))
    p.add_argument("--gt-out-dir", default=str(DEFAULT_GT_DIR))
    p.add_argument(
        "--gt-topic-regex",
        default=r"(image_gt|_gt/.*/image_raw|_gt/image_raw|/gt/|ground_truth|seg)",
    )
    p.add_argument("--raw-topic-regex", default=r"(image_raw|/raw/)")
    p.add_argument("--cameras", default="front,left,rear,right")
    p.add_argument("--include-unknown-camera", action="store_true")
    p.add_argument("--max-frames-per-topic", type=int, default=0)
    p.add_argument("--start-offset-sec", type=float, default=0.0)
    p.add_argument("--duration-sec", type=float, default=0.0)
    p.add_argument("--overwrite", action="store_true")
    p.add_argument("--csv-dir", default=str(DEFAULT_CSV_DIR))
    p.add_argument("--csv-prefix", default="")
    p.add_argument("--gt-post-dir", default=str(DEFAULT_GT_POST_DIR))
    p.add_argument("--gt-post-suffix", default=DEFAULT_GT_POST_SUFFIX)

    # Use ROS-aware argv detection if not explicitly provided
    if argv is None:
        argv = _get_argv_for_rospy()

    return p.parse_args(argv)


def sanitize_topic_name(topic):
    return topic.strip("/").replace("/", "_")


def detect_camera_name(topic, cameras, include_unknown_camera):
    lowered = topic.lower()
    for camera in cameras:
        if camera.lower() in lowered:
            return camera
    if include_unknown_camera:
        return sanitize_topic_name(topic)
    return None


def detect_kind(topic, gt_pattern, raw_pattern):
    if gt_pattern.search(topic):
        return "gt"
    if raw_pattern.search(topic):
        return "raw"
    return None


def convert_ros_image_to_bgr_or_gray(msg, bridge):
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception:
        return None
    encoding = (msg.encoding or "").lower()
    if cv_img.ndim == 3:
        if encoding in ("rgb8", "rgb16"):
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        elif encoding in ("rgba8", "rgba16"):
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGBA2BGR)
        elif encoding in ("bgra8", "bgra16"):
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2BGR)
    return cv_img


def convert_compressed_image(msg):
    arr = np.frombuffer(msg.data, dtype="uint8")
    if arr.size == 0:
        return None
    return cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)


def extract_message_stamp(msg, fallback_time):
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None) if header is not None else None
    if stamp is not None:
        try:
            if stamp.to_sec() > 0.0:
                return stamp
        except Exception:
            pass
    return fallback_time


def select_topics(bag, gt_pattern, raw_pattern, cameras, include_unknown):
    topics = {}
    topic_info = bag.get_type_and_topic_info().topics
    for topic, meta in topic_info.items():
        kind = detect_kind(topic, gt_pattern, raw_pattern)
        if kind is None:
            continue
        if meta.msg_type not in ("sensor_msgs/Image", "sensor_msgs/CompressedImage"):
            continue
        if meta.msg_type == "sensor_msgs/CompressedImage":
            sibling_topic = topic[:-11]
            sibling_meta = topic_info.get(sibling_topic)
            if sibling_meta and sibling_meta.msg_type == "sensor_msgs/Image":
                continue
        camera = detect_camera_name(topic, cameras, include_unknown)
        if camera is None:
            continue
        topics[topic] = {"kind": kind, "msg_type": meta.msg_type, "camera": camera}
    return topics


def count_selected_messages(bag, selected_topics):
    """Approximate total message count for progress reporting."""
    info = bag.get_type_and_topic_info().topics
    return sum(info[t].message_count for t in selected_topics if t in info)


def make_output_name(camera, kind, seq_idx):
    kind_token = "GT" if kind == "gt" else "raw"
    return f"{camera}_{kind_token}_{seq_idx}.png"


def write_csv(path, fieldnames, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)


def compute_post_path(image_path, input_dir, post_dir, post_suffix):
    """Compute post-processed image path preserving directory structure.

    Args:
        image_path: Full path to input image (e.g., /path/to/gt_images/front/image.png)
        input_dir: Base input directory where images were saved
        post_dir: Base directory for post-processed images
        post_suffix: Suffix to append to filename

    Returns:
        str: Full path to post-processed image preserving subdirectory structure
    """
    if not image_path:
        return ""

    image_p = Path(image_path)
    input_dir = Path(input_dir)

    try:
        # Get relative path from input_dir (preserves camera/timestamp dirs)
        relative_path = image_p.parent.relative_to(input_dir)
    except ValueError:
        # If input_dir not in path, use parent dir name
        relative_path = image_p.parent.name

    return str(
        Path(post_dir) / relative_path / f"{image_p.stem}{post_suffix}{image_p.suffix}"
    )


def compute_raw_post_path(raw_image_path, raw_out_dir, raw_post_dir, raw_post_suffix):
    return compute_post_path(raw_image_path, raw_out_dir, raw_post_dir, raw_post_suffix)


def compute_gt_post_path(gt_image_path, gt_out_dir, gt_post_dir, gt_post_suffix):
    return compute_post_path(gt_image_path, gt_out_dir, gt_post_dir, gt_post_suffix)


def build_minimal_rows(
    records,
    raw_out_dir,
    raw_post_dir,
    raw_post_suffix,
    gt_out_dir,
    gt_post_dir,
    gt_post_suffix,
    match_on_timestamp_only=False,
):
    """Build CSV rows pairing raw/gt images.

    By default pairs by index (preserves saved order). If
    ``match_on_timestamp_only`` is True, only produce rows where both raw and
    gt images exist with exactly the same ``stamp_sec`` value for a camera.
    """
    grouped = {}
    for rec in records:
        grouped.setdefault((rec["camera"], rec["kind"]), []).append(rec)

    rows = []
    raw_post_dir = Path(raw_post_dir)
    gt_post_dir = Path(gt_post_dir)
    cameras = sorted({cam for cam, _ in grouped.keys()})

    def make_rel(p):
        if not p:
            return ""
        try:
            return str(Path(p).relative_to(DATA_ROOT))
        except ValueError:
            return str(p)

    for camera in cameras:
        raw_list = grouped.get((camera, "raw"), [])
        gt_list = grouped.get((camera, "gt"), [])

        if not match_on_timestamp_only:
            # original index-based pairing
            pair_count = max(len(raw_list), len(gt_list))
            for idx in range(pair_count):
                raw = raw_list[idx] if idx < len(raw_list) else None
                gt = gt_list[idx] if idx < len(gt_list) else None
                raw_path = raw["output_path"] if raw else ""
                gt_path = gt["output_path"] if gt else ""
                raw_post_path = ""
                if raw_path:
                    raw_post_path = compute_raw_post_path(
                        raw_path, raw_out_dir, raw_post_dir, raw_post_suffix
                    )
                gt_post_path = ""
                if gt_path:
                    gt_post_path = compute_gt_post_path(
                        gt_path, gt_out_dir, gt_post_dir, gt_post_suffix
                    )
                ts = (raw or gt or {}).get("stamp_sec", 0.0)
                raw_stamp_sec = raw.get("stamp_sec", "") if raw else ""
                gt_stamp_sec = gt.get("stamp_sec", "") if gt else ""
                raw_stamp_ns = raw.get("stamp_ns", "") if raw else ""
                gt_stamp_ns = gt.get("stamp_ns", "") if gt else ""
                rows.append(
                    {
                        "timestamp": ts,
                        "raw_timestamp": raw_stamp_sec,
                        "gt_timestamp": gt_stamp_sec,
                        "raw_stamp_ns": raw_stamp_ns,
                        "gt_stamp_ns": gt_stamp_ns,
                        "camera": camera,
                        "raw": make_rel(raw_path),
                        "raw_post": make_rel(raw_post_path),
                        "gt": make_rel(gt_path),
                        "gt_post": make_rel(gt_post_path),
                    }
                )
        else:
            # timestamp-exact matching: only include stamps present in both
            # lists. stamp_sec is expected to be rounded consistently when
            # records were written.
            raw_map = {}
            for r in raw_list:
                ts = r.get("stamp_sec")
                if ts is None:
                    continue
                raw_map.setdefault(ts, []).append(r)

            gt_map = {}
            for g in gt_list:
                ts = g.get("stamp_sec")
                if ts is None:
                    continue
                gt_map.setdefault(ts, []).append(g)

            common_stamps = sorted(set(raw_map.keys()) & set(gt_map.keys()))
            for ts in common_stamps:
                # if multiple records share the same timestamp, take the
                # first saved one from each side.
                raw = raw_map[ts][0]
                gt = gt_map[ts][0]
                raw_path = raw["output_path"] if raw else ""
                gt_path = gt["output_path"] if gt else ""
                raw_post_path = ""
                if raw_path:
                    raw_post_path = compute_raw_post_path(
                        raw_path, raw_out_dir, raw_post_dir, raw_post_suffix
                    )
                gt_post_path = ""
                if gt_path:
                    gt_post_path = compute_gt_post_path(
                        gt_path, gt_out_dir, gt_post_dir, gt_post_suffix
                    )
                rows.append(
                    {
                        "timestamp": ts,
                        "raw_timestamp": raw.get("stamp_sec", ""),
                        "gt_timestamp": gt.get("stamp_sec", ""),
                        "raw_stamp_ns": raw.get("stamp_ns", ""),
                        "gt_stamp_ns": gt.get("stamp_ns", ""),
                        "camera": camera,
                        "raw": make_rel(raw_path),
                        "raw_post": make_rel(raw_post_path),
                        "gt": make_rel(gt_path),
                        "gt_post": make_rel(gt_post_path),
                    }
                )

    rows.sort(key=lambda x: (x["timestamp"], x["camera"]))
    return rows


def write_manifest_csv(
    csv_dir,
    csv_prefix,
    records,
    raw_out_dir,
    raw_post_dir,
    raw_post_suffix,
    gt_out_dir,
    gt_post_dir,
    gt_post_suffix,
):
    fields = [
        "bag",
        "timestamp",
        "raw_timestamp",
        "gt_timestamp",
        "raw_stamp_ns",
        "gt_stamp_ns",
        "camera",
        "raw",
        "raw_post",
        "gt",
        "gt_post",
    ]
    path = csv_dir / f"{csv_prefix}_images.csv"
    # Only include rows where raw and gt have identical timestamps.
    rows = build_minimal_rows(
        records,
        raw_out_dir,
        raw_post_dir,
        raw_post_suffix,
        gt_out_dir,
        gt_post_dir,
        gt_post_suffix,
        match_on_timestamp_only=True,
    )
    bag_name = records[0]["bag"] if records else csv_prefix
    for row in rows:
        row["bag"] = bag_name
    write_csv(path, fields, rows)
    return {"all": path, "row_count": len(rows)}


# ---------------------------------------------------------------------------
# Core extraction (importable by batch_extract.py)
# ---------------------------------------------------------------------------
def extract_single_bag(
    bag_path,
    raw_out_dir=None,
    raw_post_dir=None,
    raw_post_suffix=DEFAULT_RAW_POST_SUFFIX,
    gt_out_dir=None,
    csv_dir=None,
    csv_prefix="",
    gt_post_dir=None,
    gt_post_suffix=DEFAULT_GT_POST_SUFFIX,
    gt_topic_regex=r"(image_gt|_gt/.*/image_raw|_gt/image_raw|/gt/|ground_truth|seg)",
    raw_topic_regex=r"(image_raw|/raw/)",
    cameras=None,
    include_unknown_camera=False,
    max_frames_per_topic=0,
    start_offset_sec=0.0,
    duration_sec=0.0,
    overwrite=False,
    log_prefix="",
):
    """Extract raw/GT images from a single bag file.

    Returns:
        dict: {saved, skipped, records, manifest}.
    """
    try:
        import rosbag
        import rospy
        from cv_bridge import CvBridge
    except ImportError as exc:
        raise RuntimeError(
            "ROS dependencies missing. Source setup.bash and install rosbag/cv_bridge."
        ) from exc

    bag_path = Path(bag_path)
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag not found: {bag_path}")

    raw_out_dir = Path(raw_out_dir) if raw_out_dir else DEFAULT_RAW_DIR
    raw_post_dir = Path(raw_post_dir) if raw_post_dir else DEFAULT_RAW_POST_DIR
    gt_out_dir = Path(gt_out_dir) if gt_out_dir else DEFAULT_GT_DIR
    csv_dir = Path(csv_dir) if csv_dir else DEFAULT_CSV_DIR
    gt_post_dir = Path(gt_post_dir) if gt_post_dir else DEFAULT_GT_POST_DIR
    cameras = cameras or ["front", "left", "rear", "right"]
    csv_prefix = csv_prefix.strip() or bag_path.stem

    raw_out_dir.mkdir(parents=True, exist_ok=True)
    gt_out_dir.mkdir(parents=True, exist_ok=True)
    csv_dir.mkdir(parents=True, exist_ok=True)

    gt_pat = re.compile(gt_topic_regex, re.IGNORECASE)
    raw_pat = re.compile(raw_topic_regex, re.IGNORECASE)
    bridge = CvBridge()

    topic_counts = {}
    save_counts = {}
    total_saved = 0
    total_skipped = 0
    saved_records = []

    def _log(message):
        print(f"{log_prefix}{message}")

    with rosbag.Bag(str(bag_path), "r") as bag:
        selected = select_topics(bag, gt_pat, raw_pat, cameras, include_unknown_camera)
        if not selected:
            _log(f"No matching topics in {bag_path.name}.")
            return {"saved": 0, "skipped": 0, "records": [], "manifest": None}

        start_time = end_time = None
        bag_start = bag.get_start_time()
        if start_offset_sec > 0:
            start_time = rospy.Time.from_sec(bag_start + start_offset_sec)
        if duration_sec > 0:
            end_time = rospy.Time.from_sec(bag_start + start_offset_sec + duration_sec)

        total_msgs = count_selected_messages(bag, selected)
        _log(f"Bag: {bag_path.name}  ({total_msgs} msgs, {len(selected)} topics)")
        for t, info in selected.items():
            _log(f"  - {t} ({info['msg_type']}, {info['kind']}, {info['camera']})")

        # Two-pass approach: first pass identifies (camera, stamp_sec)
        # pairs that have both raw and gt messages (respecting
        # max_frames_per_topic). Second pass writes images only for those
        # exact-timestamp pairs.
        processed = 0
        progress_step = max(1, total_msgs // 20)

        seen = {}  # (camera, stamp_sec) -> set of kinds seen
        topic_counts_first = {}

        for entry in bag.read_messages(
            topics=list(selected.keys()),
            start_time=start_time,
            end_time=end_time,
        ):
            topic, msg, bag_t = entry[0], entry[1], entry[2]
            processed += 1
            if total_msgs > 0 and processed % progress_step == 0:
                _log(
                    f"  scan progress: {processed}/{total_msgs} ({processed * 100 // total_msgs}%)"
                )

            spec = selected[topic]
            count = topic_counts_first.get(topic, 0)
            if max_frames_per_topic > 0 and count >= max_frames_per_topic:
                continue

            msg_stamp = extract_message_stamp(msg, bag_t)
            stamp_sec = round(msg_stamp.to_sec(), 9)
            key = (spec["camera"], stamp_sec)
            seen.setdefault(key, set()).add(spec["kind"])
            topic_counts_first[topic] = count + 1

        # Determine which (camera, stamp) have both raw and gt
        desired_pairs = {k for k, v in seen.items() if v >= {"raw", "gt"}}

        # Reset for writing pass
        topic_counts = {}
        save_counts = {}
        total_saved = 0
        total_skipped = 0
        saved_records = []
        written_pairs = set()

        # Second pass: write only messages that belong to desired_pairs
        processed = 0
        for entry in bag.read_messages(
            topics=list(selected.keys()),
            start_time=start_time,
            end_time=end_time,
        ):
            topic, msg, bag_t = entry[0], entry[1], entry[2]
            processed += 1
            if total_msgs > 0 and processed % progress_step == 0:
                _log(
                    f"  write progress: {processed}/{total_msgs} ({processed * 100 // total_msgs}%)"
                )

            spec = selected[topic]
            count = topic_counts.get(topic, 0)
            if max_frames_per_topic > 0 and count >= max_frames_per_topic:
                continue

            msg_stamp = extract_message_stamp(msg, bag_t)
            stamp_sec = round(msg_stamp.to_sec(), 9)
            pair_key = (spec["camera"], stamp_sec)
            if pair_key not in desired_pairs:
                # we only save when both raw and gt exist with same stamp
                continue

            # Avoid writing duplicate images if multiple messages for same
            # (camera, kind, stamp) exist: allow each (camera,kind,stamp) once
            written_key = (spec["camera"], spec["kind"], stamp_sec)
            if written_key in written_pairs:
                # mark topic count but skip writing
                topic_counts[topic] = count + 1
                continue

            if spec["msg_type"] == "sensor_msgs/Image":
                image = convert_ros_image_to_bgr_or_gray(msg, bridge)
            else:
                image = convert_compressed_image(msg)

            if image is None:
                total_skipped += 1
                topic_counts[topic] = count + 1
                continue

            base_out_dir = gt_out_dir if spec["kind"] == "gt" else raw_out_dir
            out_dir = base_out_dir / spec["camera"]
            out_dir.mkdir(parents=True, exist_ok=True)
            save_key = (spec["camera"], spec["kind"])
            seq_idx = save_counts.get(save_key, 0) + 1
            out_name = make_output_name(spec["camera"], spec["kind"], seq_idx)
            out_path = out_dir / out_name

            if (not overwrite) and out_path.exists():
                total_skipped += 1
                topic_counts[topic] = count + 1
                continue

            if not cv2.imwrite(str(out_path), image):
                total_skipped += 1
                topic_counts[topic] = count + 1
                continue

            # successful write
            topic_counts[topic] = count + 1
            save_counts[save_key] = seq_idx
            total_saved += 1
            written_pairs.add(written_key)
            saved_records.append(
                {
                    "bag": bag_path.name,
                    "kind": spec["kind"],
                    "camera": spec["camera"],
                    "topic": topic,
                    "msg_type": spec["msg_type"],
                    "stamp_ns": msg_stamp.to_nsec(),
                    "stamp_sec": stamp_sec,
                    "height": int(image.shape[0]),
                    "width": int(image.shape[1]),
                    "output_path": str(out_path),
                }
            )

    _log(f"Done. saved={total_saved}, skipped={total_skipped}")
    for t, c in sorted(topic_counts.items()):
        _log(f"  {t}: {c}")

    manifest = None
    if saved_records:
        manifest = write_manifest_csv(
            csv_dir,
            csv_prefix,
            saved_records,
            raw_out_dir,
            raw_post_dir,
            raw_post_suffix,
            gt_out_dir,
            gt_post_dir,
            gt_post_suffix,
        )
        _log(f"CSV: {manifest['all']}  ({manifest['row_count']} rows)")

    return {
        "saved": total_saved,
        "skipped": total_skipped,
        "records": saved_records,
        "manifest": manifest,
    }


def main():
    args = parse_args()
    cameras = [x.strip() for x in args.cameras.split(",") if x.strip()]
    extract_single_bag(
        bag_path=args.bag,
        raw_out_dir=args.raw_out_dir,
        raw_post_dir=args.raw_post_dir,
        gt_out_dir=args.gt_out_dir,
        csv_dir=args.csv_dir,
        csv_prefix=args.csv_prefix,
        gt_post_dir=args.gt_post_dir,
        gt_post_suffix=args.gt_post_suffix,
        gt_topic_regex=args.gt_topic_regex,
        raw_topic_regex=args.raw_topic_regex,
        cameras=cameras,
        include_unknown_camera=args.include_unknown_camera,
        max_frames_per_topic=args.max_frames_per_topic,
        start_offset_sec=args.start_offset_sec,
        duration_sec=args.duration_sec,
        overwrite=args.overwrite,
    )


if __name__ == "__main__":
    main()
