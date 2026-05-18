#!/usr/bin/env python3
"""Republish CameraBundle images as per-camera debug image topics."""

from __future__ import annotations

import re
from typing import Dict

import numpy as np
import rospy
from carmaker_msgs.msg import CameraBundle
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


DEFAULT_PALETTE_RGB = np.array(
    [
        (0, 0, 0),      # background
        (0, 255, 0),    # lane
        (255, 255, 0),  # landmark
    ],
    dtype=np.uint8,
)


class BundleImageVisualizerNode:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.bundle_topic = rospy.get_param("~bundle_topic", "/segmentation/class_map_bundle")
        self.output_prefix = rospy.get_param("~output_prefix", "/segmentation/debug").rstrip("/")
        self.publish_images = parse_bool(rospy.get_param("~publish_images", True))
        self.publish_color = parse_bool(rospy.get_param("~publish_color", True))
        self.queue_size = int(rospy.get_param("~queue_size", 1))

        self.image_pubs: Dict[str, rospy.Publisher] = {}
        self.color_pubs: Dict[str, rospy.Publisher] = {}
        self.sub = rospy.Subscriber(
            self.bundle_topic,
            CameraBundle,
            self.bundle_callback,
            queue_size=self.queue_size,
        )

        rospy.loginfo(
            "bundle_image_visualizer_node subscribed bundle=%s output_prefix=%s publish_images=%s publish_color=%s",
            self.bundle_topic,
            self.output_prefix,
            self.publish_images,
            self.publish_color,
        )

    def bundle_callback(self, msg: CameraBundle) -> None:
        for index, image_msg in enumerate(msg.images):
            name = camera_name(msg, index)
            safe_name = sanitize_topic_name(name)

            if self.publish_images:
                self.image_pub(safe_name).publish(image_msg)

            if self.publish_color and is_mono_image(image_msg):
                try:
                    color_msg = self.colorized_class_map_msg(image_msg)
                except (CvBridgeError, ValueError) as exc:
                    rospy.logwarn_throttle(
                        1.0,
                        "failed to colorize bundle image %s encoding=%s: %s",
                        name,
                        image_msg.encoding,
                        exc,
                    )
                    continue
                self.color_pub(safe_name).publish(color_msg)

    def image_pub(self, safe_name: str) -> rospy.Publisher:
        if safe_name not in self.image_pubs:
            topic = f"{self.output_prefix}/{safe_name}/image"
            self.image_pubs[safe_name] = rospy.Publisher(topic, Image, queue_size=self.queue_size)
            rospy.loginfo("publishing bundle image debug topic: %s", topic)
        return self.image_pubs[safe_name]

    def color_pub(self, safe_name: str) -> rospy.Publisher:
        if safe_name not in self.color_pubs:
            topic = f"{self.output_prefix}/{safe_name}/color"
            self.color_pubs[safe_name] = rospy.Publisher(topic, Image, queue_size=self.queue_size)
            rospy.loginfo("publishing bundle color debug topic: %s", topic)
        return self.color_pubs[safe_name]

    def colorized_class_map_msg(self, image_msg: Image) -> Image:
        class_map = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        class_map = np.asarray(class_map)
        if class_map.ndim == 3 and class_map.shape[2] == 1:
            class_map = class_map[:, :, 0]
        if class_map.ndim != 2:
            raise ValueError(f"class map must be 2D, got shape {class_map.shape}")

        class_ids = class_map.astype(np.int64, copy=False)
        color = np.zeros((*class_ids.shape, 3), dtype=np.uint8)
        valid = (class_ids >= 0) & (class_ids < len(DEFAULT_PALETTE_RGB))
        color[valid] = DEFAULT_PALETTE_RGB[class_ids[valid]]

        color_msg = self.bridge.cv2_to_imgmsg(color, encoding="rgb8")
        color_msg.header = image_msg.header
        return color_msg


def camera_name(msg: CameraBundle, index: int) -> str:
    if index < len(msg.names) and msg.names[index]:
        return msg.names[index]
    return f"camera_{index}"


def sanitize_topic_name(name: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9_]+", "_", name.strip())
    return safe.strip("_") or "camera"


def is_mono_image(msg: Image) -> bool:
    return msg.encoding.lower() in {"mono8", "8uc1"}


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def main() -> None:
    rospy.init_node("bundle_image_visualizer_node")
    BundleImageVisualizerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
