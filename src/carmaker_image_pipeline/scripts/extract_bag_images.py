#!/usr/bin/python3
import argparse
import re
from pathlib import Path

import cv2
import numpy as np
import rosbag
import rospy
from cv_bridge import CvBridge, CvBridgeError

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = PACKAGE_ROOT / "data"
DEFAULT_RAW_DIR = DATA_ROOT / "raw_images"
DEFAULT_GT_DIR = DATA_ROOT / "gt_images"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Extract raw/GT images from ROS bag into carmaker_image_pipeline folders."
    )
    parser.add_argument("--bag", required=True, help="Input ROS bag path.")
    parser.add_argument(
        "--raw-out-dir",
        default=str(DEFAULT_RAW_DIR),
        help=f"Output folder for raw images (default: {DEFAULT_RAW_DIR}).",
    )
    parser.add_argument(
        "--gt-out-dir",
        default=str(DEFAULT_GT_DIR),
        help=f"Output folder for GT images (default: {DEFAULT_GT_DIR}).",
    )
    parser.add_argument(
        "--gt-topic-regex",
        default=r"(image_gt|_gt/.*/image_raw|_gt/image_raw|/gt/|ground_truth|seg)",
        help="Regex to detect GT image topics.",
    )
    parser.add_argument(
        "--raw-topic-regex",
        default=r"(image_raw|/raw/)",
        help="Regex to detect raw image topics.",
    )
    parser.add_argument(
        "--cameras",
        default="front,left,rear,right",
        help="Comma-separated camera keywords used for filename camera detection.",
    )
    parser.add_argument(
        "--include-unknown-camera",
        action="store_true",
        help="If set, save topics without front/left/rear/right keyword using topic-based camera names.",
    )
    parser.add_argument(
        "--max-frames-per-topic",
        type=int,
        default=0,
        help="Limit extracted frames per topic (0 means unlimited).",
    )
    parser.add_argument(
        "--start-offset-sec",
        type=float,
        default=0.0,
        help="Start extraction from bag start + offset seconds.",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=0.0,
        help="Extraction duration in seconds (0 means until end of bag).",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite output files when names collide.",
    )
    return parser.parse_args()


def sanitize_topic_name(topic):
    return topic.strip("/").replace("/", "_")


def detect_camera_name(topic, cameras, include_unknown_camera):
    lowered = topic.lower()
    for camera in cameras:
        cam = camera.lower()
        if cam in lowered:
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
    except CvBridgeError:
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


def select_topics(bag, gt_pattern, raw_pattern, cameras, include_unknown_camera):
    topics = {}
    topic_info = bag.get_type_and_topic_info().topics
    for topic, meta in topic_info.items():
        kind = detect_kind(topic, gt_pattern, raw_pattern)
        if kind is None:
            continue

        msg_type = meta.msg_type
        if msg_type not in ("sensor_msgs/Image", "sensor_msgs/CompressedImage"):
            continue

        camera = detect_camera_name(topic, cameras, include_unknown_camera)
        if camera is None:
            continue

        topics[topic] = {"kind": kind, "msg_type": msg_type, "camera": camera}
    return topics


def make_output_name(camera, kind, stamp_ns):
    kind_token = "GT" if kind == "gt" else "raw"
    return f"{camera}_{kind_token}_{stamp_ns}.png"


def main():
    args = parse_args()

    bag_path = Path(args.bag)
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag file not found: {bag_path}")

    raw_out_dir = Path(args.raw_out_dir)
    gt_out_dir = Path(args.gt_out_dir)
    raw_out_dir.mkdir(parents=True, exist_ok=True)
    gt_out_dir.mkdir(parents=True, exist_ok=True)

    cameras = [x.strip() for x in args.cameras.split(",") if x.strip()]
    if not cameras:
        raise ValueError("At least one camera keyword is required.")

    gt_pattern = re.compile(args.gt_topic_regex, re.IGNORECASE)
    raw_pattern = re.compile(args.raw_topic_regex, re.IGNORECASE)

    bridge = CvBridge()
    topic_counts = {}
    total_saved = 0
    total_skipped = 0

    with rosbag.Bag(str(bag_path), "r") as bag:
        selected = select_topics(
            bag=bag,
            gt_pattern=gt_pattern,
            raw_pattern=raw_pattern,
            cameras=cameras,
            include_unknown_camera=args.include_unknown_camera,
        )

        if not selected:
            print("No matching image topics found. Check regex or camera keywords.")
            return

        start_time = None
        end_time = None
        bag_start = bag.get_start_time()
        if args.start_offset_sec > 0:
            start_time = rospy.Time.from_sec(bag_start + args.start_offset_sec)
        if args.duration_sec > 0:
            start_ref = bag_start + args.start_offset_sec
            end_time = rospy.Time.from_sec(start_ref + args.duration_sec)

        print("Selected topics:")
        for topic, info in selected.items():
            print(f"  - {topic} ({info['msg_type']}, kind={info['kind']}, camera={info['camera']})")

        for topic, msg, t in bag.read_messages(
            topics=list(selected.keys()),
            start_time=start_time,
            end_time=end_time,
        ):
            spec = selected[topic]
            count = topic_counts.get(topic, 0)
            if args.max_frames_per_topic > 0 and count >= args.max_frames_per_topic:
                continue

            if spec["msg_type"] == "sensor_msgs/Image":
                image = convert_ros_image_to_bgr_or_gray(msg, bridge)
            else:
                image = convert_compressed_image(msg)

            if image is None:
                total_skipped += 1
                continue

            out_dir = gt_out_dir if spec["kind"] == "gt" else raw_out_dir
            out_name = make_output_name(spec["camera"], spec["kind"], t.to_nsec())
            out_path = out_dir / out_name

            if (not args.overwrite) and out_path.exists():
                total_skipped += 1
                continue

            ok = cv2.imwrite(str(out_path), image)
            if not ok:
                total_skipped += 1
                continue

            topic_counts[topic] = count + 1
            total_saved += 1

    print(f"done. saved={total_saved}, skipped={total_skipped}")
    for topic, count in sorted(topic_counts.items()):
        print(f"  {topic}: {count}")


if __name__ == "__main__":
    main()
