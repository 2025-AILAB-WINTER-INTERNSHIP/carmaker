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
DEFAULT_GT_DIR = DATA_ROOT / "gt_images"
DEFAULT_CSV_DIR = DATA_ROOT / "csv"
DEFAULT_GT_POST_DIR = DATA_ROOT / "gt_post_processed"
DEFAULT_GT_POST_SUFFIX = "_post"


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Extract raw/GT images from ROS bag."
    )
    p.add_argument("--bag", required=True, help="Input ROS bag path.")
    p.add_argument("--raw-out-dir", default=str(DEFAULT_RAW_DIR))
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
    arr = np.frombuffer(msg.data, dtype=np.uint8)
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


def build_minimal_rows(records, gt_post_dir, gt_post_suffix):
    grouped = {}
    for rec in records:
        grouped.setdefault((rec["camera"], rec["kind"]), []).append(rec)

    rows = []
    gt_post_dir = Path(gt_post_dir)
    cameras = sorted({cam for cam, _ in grouped.keys()})

    def make_rel(p):
        if not p: return ""
        try:
            return str(Path(p).relative_to(DATA_ROOT))
        except ValueError:
            return str(p)

    for camera in cameras:
        raw_list = grouped.get((camera, "raw"), [])
        gt_list = grouped.get((camera, "gt"), [])
        pair_count = max(len(raw_list), len(gt_list))
        for idx in range(pair_count):
            raw = raw_list[idx] if idx < len(raw_list) else None
            gt = gt_list[idx] if idx < len(gt_list) else None
            raw_path = raw["output_path"] if raw else ""
            gt_path = gt["output_path"] if gt else ""
            gt_post_path = ""
            if gt_path:
                gf = Path(gt_path)
                gt_post_path = str(gt_post_dir / f"{gf.stem}{gt_post_suffix}{gf.suffix}")
            ts = (raw or gt or {}).get("stamp_sec", 0.0)
            rows.append({"timestamp": ts, "camera": camera,
                         "raw": make_rel(raw_path), "gt": make_rel(gt_path), "gt_post": make_rel(gt_post_path)})

    rows.sort(key=lambda x: (x["timestamp"], x["camera"]))
    return rows


def write_manifest_csv(csv_dir, csv_prefix, records, gt_post_dir, gt_post_suffix):
    fields = ["timestamp", "camera", "raw", "gt", "gt_post"]
    path = csv_dir / f"{csv_prefix}_images.csv"
    rows = build_minimal_rows(records, gt_post_dir, gt_post_suffix)
    write_csv(path, fields, rows)
    return {"all": path, "row_count": len(rows)}


# ---------------------------------------------------------------------------
# Core extraction (importable by batch_extract.py)
# ---------------------------------------------------------------------------
def extract_single_bag(
    bag_path,
    raw_out_dir=None,
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
    _log = lambda m: print(f"{log_prefix}{m}")

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

        processed = 0
        progress_step = max(1, total_msgs // 20)

        for topic, msg, bag_t in bag.read_messages(
            topics=list(selected.keys()),
            start_time=start_time,
            end_time=end_time,
        ):
            processed += 1
            if total_msgs > 0 and processed % progress_step == 0:
                _log(f"  progress: {processed}/{total_msgs} ({processed*100//total_msgs}%)")

            spec = selected[topic]
            count = topic_counts.get(topic, 0)
            if max_frames_per_topic > 0 and count >= max_frames_per_topic:
                continue

            if spec["msg_type"] == "sensor_msgs/Image":
                image = convert_ros_image_to_bgr_or_gray(msg, bridge)
            else:
                image = convert_compressed_image(msg)

            if image is None:
                total_skipped += 1
                continue

            out_dir = gt_out_dir if spec["kind"] == "gt" else raw_out_dir
            save_key = (spec["camera"], spec["kind"])
            seq_idx = save_counts.get(save_key, 0) + 1
            out_name = make_output_name(spec["camera"], spec["kind"], seq_idx)
            out_path = out_dir / out_name

            if (not overwrite) and out_path.exists():
                total_skipped += 1
                continue

            if not cv2.imwrite(str(out_path), image):
                total_skipped += 1
                continue

            msg_stamp = extract_message_stamp(msg, bag_t)
            topic_counts[topic] = count + 1
            save_counts[save_key] = seq_idx
            total_saved += 1
            saved_records.append({
                "bag": bag_path.name,
                "kind": spec["kind"],
                "camera": spec["camera"],
                "topic": topic,
                "msg_type": spec["msg_type"],
                "stamp_ns": msg_stamp.to_nsec(),
                "stamp_sec": round(msg_stamp.to_sec(), 9),
                "height": int(image.shape[0]),
                "width": int(image.shape[1]),
                "output_path": str(out_path),
            })

    _log(f"Done. saved={total_saved}, skipped={total_skipped}")
    for t, c in sorted(topic_counts.items()):
        _log(f"  {t}: {c}")

    manifest = None
    if saved_records:
        manifest = write_manifest_csv(
            csv_dir, csv_prefix, saved_records, gt_post_dir, gt_post_suffix)
        _log(f"CSV: {manifest['all']}  ({manifest['row_count']} rows)")

    return {"saved": total_saved, "skipped": total_skipped,
            "records": saved_records, "manifest": manifest}


def main():
    args = parse_args()
    cameras = [x.strip() for x in args.cameras.split(",") if x.strip()]
    extract_single_bag(
        bag_path=args.bag,
        raw_out_dir=args.raw_out_dir,
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
