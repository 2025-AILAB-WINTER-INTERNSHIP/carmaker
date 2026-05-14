#!/usr/bin/env python3
"""ROS1 node for CarMaker semantic segmentation inference."""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Tuple

# 이 ROS 패키지는 src/segmentation 안의 학습/추론 공용 코드를 import한다.
# catkin devel 공간에서 실행해도 source tree 기준 import가 안정적으로 잡히도록
# workspace의 src 디렉터리를 Python path 앞쪽에 추가한다.
SRC_ROOT = Path(__file__).resolve().parents[2]
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

import rospy
from carmaker_msgs.msg import CameraBundle
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

        # input_mode는 이 노드의 ROS 입출력 형태를 고른다.
        #
        # image:
        #   기존 방식이다. sensor_msgs/Image 한 장을 받아 sensor_msgs/Image class_map 한 장을 낸다.
        #   단일 카메라 디버깅이나 모델 단독 확인에 편하다.
        #
        # bundle:
        #   carmaker_image_synchronizer가 만든 carmaker_msgs/CameraBundle을 그대로 받아
        #   bundle 안의 모든 카메라 이미지에 inference를 수행한다.
        #   출력도 CameraBundle로 내보내되, names/infos/header는 입력 기준을 유지하고
        #   images 배열만 mono8 class_map 배열로 바꾼다.
        input_mode = str(rospy.get_param("~input_mode", "image")).strip().lower()
        image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        class_map_topic = rospy.get_param("~class_map_topic", "/segmentation/class_map")
        bundle_topic = rospy.get_param("~bundle_topic", "/synced/bundle")
        class_map_bundle_topic = rospy.get_param("~class_map_bundle_topic", "/segmentation/class_map_bundle")
        queue_size = int(rospy.get_param("~queue_size", 1))

        # class_map은 mono8 이미지다. 픽셀 값 자체가 class id이다.
        # 예: 0=background, 1=lane, 2=landmark.
        # queue_size=1은 오래된 이미지를 쌓지 않고 최신 프레임 위주로 처리하기 위한 설정이다.
        # inference가 카메라 FPS보다 느릴 때 지연이 계속 누적되는 것을 줄인다.
        if input_mode == "bundle":
            # Multi-camera path:
            # /synced/bundle 같은 토픽 하나에 front/rear/left/right 이미지가 묶여 들어온다.
            # downstream 노드가 어떤 결과가 어떤 카메라인지 계속 알 수 있도록
            # 출력 타입도 CameraBundle로 맞춘다.
            self.class_map_bundle_pub = rospy.Publisher(
                class_map_bundle_topic,
                CameraBundle,
                queue_size=queue_size,
            )
            self.bundle_sub = rospy.Subscriber(
                bundle_topic,
                CameraBundle,
                self.bundle_callback,
                queue_size=queue_size,
            )
            rospy.loginfo(
                "segmentation_inference_node subscribed bundle=%s class_map_bundle=%s device=%s classes=%s",
                bundle_topic,
                class_map_bundle_topic,
                self.predictor.device,
                ",".join(self.predictor.class_names),
            )
        elif input_mode == "image":
            # Legacy single-image path:
            # bundle synchronizer 없이 모델만 빠르게 테스트할 때 사용한다.
            self.class_map_pub = rospy.Publisher(class_map_topic, Image, queue_size=queue_size)
            self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=queue_size)
            rospy.loginfo(
                "segmentation_inference_node subscribed image=%s class_map=%s device=%s classes=%s",
                image_topic,
                class_map_topic,
                self.predictor.device,
                ",".join(self.predictor.class_names),
            )
        else:
            raise rospy.ROSException("~input_mode must be 'image' or 'bundle'")

    def image_callback(self, msg: Image) -> None:
        try:
            callback_start = time.perf_counter()

            # 단일 이미지 모드도 실제 inference 로직은 공용 helper를 쓴다.
            # 이렇게 해두면 bundle 모드와 class_map 생성 규칙이 항상 같게 유지된다.
            class_msg, inference_ms = self.predict_class_map_msg(msg)
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

    def bundle_callback(self, msg: CameraBundle) -> None:
        try:
            callback_start = time.perf_counter()

            # 정상적인 CameraBundle은 names/images/infos가 같은 순서를 공유한다.
            # 예를 들어 names[0] == "front"라면 images[0], infos[0]도 front 카메라 기준이다.
            # 아래 mismatch 체크는 메시지를 버리기보다 경고만 남긴다. 실험 중 CameraInfo가
            # 비거나 일부만 들어오는 상황에서도 segmentation 결과 자체는 낼 수 있기 때문이다.
            if not msg.images:
                rospy.logwarn_throttle(1.0, "segmentation inference skipped empty CameraBundle")
                return
            if msg.names and len(msg.names) != len(msg.images):
                rospy.logwarn_throttle(
                    1.0,
                    "CameraBundle names/images length mismatch: names=%d images=%d",
                    len(msg.names),
                    len(msg.images),
                )
            if msg.infos and len(msg.infos) != len(msg.images):
                rospy.logwarn_throttle(
                    1.0,
                    "CameraBundle infos/images length mismatch: infos=%d images=%d",
                    len(msg.infos),
                    len(msg.images),
                )

            # 출력 bundle의 의미:
            # - header: 동기화 기준 timestamp/frame_id를 그대로 유지한다.
            # - names: 입력 카메라 이름 배열을 그대로 유지한다.
            # - infos: 입력 CameraInfo 배열을 그대로 유지한다.
            # - images: 원본 RGB/BGR 이미지 대신 mono8 class_map 이미지가 들어간다.
            #
            # 즉 downstream에서는 기존 CameraBundle과 같은 index 규칙으로 접근하되,
            # images[i]를 "원본 카메라 이미지"가 아니라 "해당 카메라 segmentation map"으로 보면 된다.
            class_bundle = CameraBundle()
            class_bundle.header = msg.header
            class_bundle.names = list(msg.names)
            class_bundle.infos = list(msg.infos)

            # 현재 모델 API는 한 번에 이미지 한 장을 받으므로 bundle 안의 이미지를 순차 처리한다.
            # 나중에 batch inference를 지원하게 되면 이 루프만 batch 호출로 바꾸면 된다.
            total_inference_ms = 0.0
            for image_msg in msg.images:
                class_msg, inference_ms = self.predict_class_map_msg(image_msg)
                class_bundle.images.append(class_msg)
                total_inference_ms += inference_ms

            self.class_map_bundle_pub.publish(class_bundle)

            if self.log_timing:
                callback_ms = (time.perf_counter() - callback_start) * 1000.0
                fps = 1000.0 / total_inference_ms if total_inference_ms > 0.0 else 0.0
                rospy.loginfo_throttle(
                    self.timing_log_interval,
                    "segmentation bundle timing: images=%d inference=%.2f ms callback=%.2f ms approx_bundle_fps=%.2f",
                    len(msg.images),
                    total_inference_ms,
                    callback_ms,
                    fps,
                )
        except (CvBridgeError, ValueError, RuntimeError) as exc:
            rospy.logerr_throttle(1.0, "segmentation bundle inference failed: %s", exc)

    def predict_class_map_msg(self, msg: Image) -> Tuple[Image, float]:
        # ROS Image -> OpenCV BGR image.
        # predictor는 내부에서 학습 때와 같은 RGB/resize/tensor 변환을 수행한다.
        # desired_encoding은 input_encoding 파라미터를 따른다. 기본값은 CarMaker 카메라에서
        # 흔히 쓰는 bgr8이며, 입력 토픽이 rgb8이면 YAML/launch에서 바꿔주면 된다.
        image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.input_encoding)

        # 실제 PyTorch forward 시간만 따로 재서 로그에 남긴다.
        # bundle 모드에서는 각 카메라 inference 시간을 합산해 전체 bundle 처리량을 계산한다.
        inference_start = time.perf_counter()
        result = self.predictor.predict(image_bgr, color_order="bgr")
        inference_ms = (time.perf_counter() - inference_start) * 1000.0

        # class_map은 원본 입력 이미지 크기로 되돌려 publish한다.
        # 그래서 다른 노드가 같은 camera frame/header 기준으로 바로 맞춰 쓸 수 있다.
        # encoding은 mono8이다. 픽셀 밝기값이 시각화용 intensity가 아니라 class id이다.
        # 예: 0=background, 1=lane, 2=landmark.
        class_msg = self.bridge.cv2_to_imgmsg(result.class_map, encoding="mono8")
        class_msg.header = msg.header
        return class_msg, inference_ms


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
