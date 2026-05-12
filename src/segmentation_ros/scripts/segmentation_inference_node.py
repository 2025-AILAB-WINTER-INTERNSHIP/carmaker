#!/usr/bin/env python3
"""ROS1 node for CarMaker semantic segmentation inference."""

from __future__ import annotations

import sys
import time
from pathlib import Path

# 이 ROS 패키지는 src/segmentation 안의 학습/추론 공용 코드를 import한다.
# catkin devel 공간에서 실행해도 source tree 기준 import가 안정적으로 잡히도록
# workspace의 src 디렉터리를 Python path 앞쪽에 추가한다.
SRC_ROOT = Path(__file__).resolve().parents[2]
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from segmentation.inference import SegmentationPredictor, parse_image_size


class SegmentationInferenceNode:
    def __init__(self) -> None:
        self.bridge = CvBridge()

        # train.py가 저장한 best.pt 또는 last.pt가 반드시 필요하다.
        # checkpoint 안에는 model_state와 학습 당시 config가 함께 들어있다.
        checkpoint_path = rospy.get_param("~checkpoint_path", "")
        if not checkpoint_path:
            raise rospy.ROSException("~checkpoint_path is required")

        # config_path/image_size/device는 checkpoint에 저장된 값을 런타임에서
        # 덮어쓰고 싶을 때만 지정한다. 비워두면 checkpoint config를 따른다.
        config_path = rospy.get_param("~config_path", None) or None
        device = rospy.get_param("~device", None) or None
        image_size_param = rospy.get_param("~image_size", None)
        image_size = parse_image_size(image_size_param) if image_size_param else None
        # cv_bridge가 ROS Image를 OpenCV 배열로 바꿀 때 사용할 encoding.
        # CarMaker/ROS 카메라 이미지는 보통 bgr8이므로 기본값도 bgr8로 둔다.
        self.input_encoding = rospy.get_param("~input_encoding", "bgr8")
        self.log_timing = parse_bool(rospy.get_param("~log_timing", True))
        self.timing_log_interval = float(rospy.get_param("~timing_log_interval", 1.0))

        # 실제 모델 로딩과 PyTorch inference는 ROS와 분리된 SegmentationPredictor가 담당한다.
        # 이 노드는 topic 입출력만 얇게 연결하는 adapter 역할을 한다.
        self.predictor = SegmentationPredictor(
            checkpoint_path=checkpoint_path,
            config_path=config_path,
            device=device,
            image_size=image_size,
        )

        image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        class_map_topic = rospy.get_param("~class_map_topic", "/segmentation/class_map")
        queue_size = int(rospy.get_param("~queue_size", 1))

        # class_map은 mono8 이미지다. 픽셀 값 자체가 class id이다.
        # 예: 0=background, 1=lane, 2=landmark.
        self.class_map_pub = rospy.Publisher(class_map_topic, Image, queue_size=queue_size)

        # queue_size=1은 오래된 이미지를 쌓지 않고 최신 프레임 위주로 처리하기 위한 설정이다.
        # inference가 카메라 FPS보다 느릴 때 지연이 계속 누적되는 것을 줄인다.
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=queue_size)

        rospy.loginfo(
            "segmentation_inference_node subscribed=%s class_map=%s device=%s classes=%s",
            image_topic,
            class_map_topic,
            self.predictor.device,
            ",".join(self.predictor.class_names),
        )

    def image_callback(self, msg: Image) -> None:
        try:
            callback_start = time.perf_counter()

            # ROS Image -> OpenCV BGR image.
            # predictor는 내부에서 학습 때와 같은 RGB/resize/tensor 변환을 수행한다.
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.input_encoding)

            inference_start = time.perf_counter()
            result = self.predictor.predict(image_bgr, color_order="bgr")
            inference_ms = (time.perf_counter() - inference_start) * 1000.0

            # class_map은 원본 입력 이미지 크기로 되돌려 publish한다.
            # 그래서 다른 노드가 같은 camera frame/header 기준으로 바로 맞춰 쓸 수 있다.
            class_msg = self.bridge.cv2_to_imgmsg(result.class_map, encoding="mono8")
            class_msg.header = msg.header
            self.class_map_pub.publish(class_msg)

            if self.log_timing:
                callback_ms = (time.perf_counter() - callback_start) * 1000.0
                fps = 1000.0 / inference_ms if inference_ms > 0.0 else 0.0
                rospy.loginfo_throttle(
                    self.timing_log_interval,
                    "segmentation timing: inference=%.2f ms callback=%.2f ms approx_fps=%.2f",
                    inference_ms,
                    callback_ms,
                    fps,
                )
        except (CvBridgeError, ValueError, RuntimeError) as exc:
            rospy.logerr_throttle(1.0, "segmentation inference failed: %s", exc)


def main() -> None:
    rospy.init_node("segmentation_inference_node")
    SegmentationInferenceNode()
    rospy.spin()


def parse_bool(value) -> bool:
    """roslaunch에서 문자열로 들어온 true/false 값을 안전하게 bool로 바꾼다."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


if __name__ == "__main__":
    main()
