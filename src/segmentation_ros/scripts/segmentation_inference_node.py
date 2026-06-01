#!/usr/bin/env python3
"""ROS1 node for CarMaker semantic segmentation inference."""

from __future__ import annotations

import os
import queue
import sys
import threading
import time
import numpy as np
from pathlib import Path
from typing import Tuple

# 이 ROS 패키지는 src/segmentation 안의 학습/추론 공용 코드를 import한다.
# devel/install 어느 쪽에서 실행해도 workspace의 src 디렉터리를 찾도록 후보 경로를 훑는다.
def add_segmentation_source_path() -> None:
    search_roots = list(Path(__file__).resolve().parents) + list(Path.cwd().resolve().parents)
    search_roots.append(Path.cwd().resolve())
    for prefix in os.environ.get("CATKIN_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            search_roots.extend(Path(prefix).resolve().parents)

    for root in search_roots:
        for candidate in (root, root / "src"):
            if (candidate / "segmentation" / "__init__.py").exists():
                candidate_str = str(candidate)
                if candidate_str not in sys.path:
                    sys.path.insert(0, candidate_str)
                return


add_segmentation_source_path()

import rospy
from carmaker_msgs.msg import CameraBundle
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from segmentation.inference import SegmentationPredictor, parse_image_size


class SegmentationInferenceNode:
    def __init__(self) -> None:
        self.bridge = CvBridge()

        # train.py가 저장한 best.ckpt 또는 last.ckpt가 반드시 필요하다.
        # checkpoint 안에는 model_state와 학습 당시 config가 함께 들어있다.
        checkpoint_path = rospy.get_param("~checkpoint_path", "")
        if not checkpoint_path:
            raise rospy.ROSException("~checkpoint_path is required")

        # config_path/image_size/device는 checkpoint에 저장된 값을 런타임에서
        # 덮어쓰고 싶을 때만 지정한다. 비워두면 checkpoint config를 따른다.
        config_path = rospy.get_param("~config_path", None) or None
        device = rospy.get_param("~device", None) or None
        inference_precision = rospy.get_param("~inference_precision", "fp16")
        cuda_warmup_iterations = int(rospy.get_param("~cuda_warmup_iterations", 1))
        image_size_param = rospy.get_param("~image_size", None)
        image_size = parse_image_size(image_size_param) if image_size_param else None
        
        # 모델 최적화 옵션 로드 (Conv-BN Fusion, torch.compile)
        use_fusion = parse_bool(rospy.get_param("~use_fusion", True))
        use_compile = parse_bool(rospy.get_param("~use_compile", True))

        # cv_bridge가 ROS Image를 OpenCV 배열로 바꿀 때 사용할 encoding.
        # CarMaker/ROS 카메라 이미지는 보통 bgr8이므로 기본값도 bgr8로 둔다.
        self.input_encoding = rospy.get_param("~input_encoding", "bgr8")
        self.log_timing = parse_bool(rospy.get_param("~log_timing", True))
        self.timing_log_interval = float(rospy.get_param("~timing_log_interval", 1.0))
        self.drop_while_busy = parse_bool(rospy.get_param("~drop_while_busy", True))
        
        # 출력 해상도 복원(원본 카메라 크기로 리사이즈) 여부를 결정하는 파라미터 (기본값: True)
        # False로 설정 시 모델 네이티브 해상도(720x480) 그대로 반환하여 호스트-디바이스 복사 오버헤드를 극적으로 아낍니다.
        resize_output = parse_bool(rospy.get_param("~resize_output", False))
        
        # 실제 모델 로딩과 PyTorch inference는 ROS와 분리된 SegmentationPredictor가 담당한다.
        self.predictor = SegmentationPredictor(
            checkpoint_path=checkpoint_path,
            config_path=config_path,
            device=device,
            image_size=image_size,
            inference_precision=inference_precision,
            warmup_iterations=cuda_warmup_iterations,
            resize_output=resize_output,
            use_fusion=use_fusion,
            use_compile=use_compile,
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
        self.input_mode = input_mode
        image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        class_map_topic = rospy.get_param("~class_map_topic", "/segmentation/class_map")
        bundle_topic = rospy.get_param("~bundle_topic", "/synced/bundle")
        class_map_bundle_topic = rospy.get_param("~class_map_bundle_topic", "/segmentation/class_map_bundle")
        queue_size = int(rospy.get_param("~queue_size", 1))

        # class_map은 mono8 이미지다. 픽셀 값 자체가 class id이다.
        # 예: 0=background, 1=lane, 2=landmark.
        # queue_size=1은 오래된 이미지를 쌓지 않고 최신 프레임 위주로 처리하기 위한 설정이다.
        # inference가 카메라 FPS보다 느릴 때 지연이 계속 누적되는 것을 줄인다.
        # 여기서는 비동기 처리 큐(Queue)와 백그라운드 워커 스레드를 사용하여 병렬성을 높입니다.
        self.msg_queue = queue.Queue(maxsize=1 if self.drop_while_busy else 0)
        self.worker_thread = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker_thread.start()

        if input_mode == "bundle":
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
                "segmentation_inference_node (async) subscribed bundle=%s class_map_bundle=%s device=%s precision=%s image_size=%sx%s warmup=%d drop_while_busy=%s classes=%s",
                bundle_topic,
                class_map_bundle_topic,
                self.predictor.device,
                self.predictor.inference_precision,
                self.predictor.image_size[0],
                self.predictor.image_size[1],
                cuda_warmup_iterations,
                self.drop_while_busy,
                ",".join(self.predictor.class_names),
            )
        elif input_mode == "image":
            self.class_map_pub = rospy.Publisher(class_map_topic, Image, queue_size=queue_size)
            self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=queue_size)
            rospy.loginfo(
                "segmentation_inference_node (async) subscribed image=%s class_map=%s device=%s precision=%s classes=%s",
                image_topic,
                class_map_topic,
                self.predictor.device,
                self.predictor.inference_precision,
                ",".join(self.predictor.class_names),
            )
        else:
            raise rospy.ROSException("~input_mode must be 'image' or 'bundle'")

    def image_callback(self, msg: Image) -> None:
        self.push_to_queue(msg)

    def bundle_callback(self, msg: CameraBundle) -> None:
        self.push_to_queue(msg)

    def push_to_queue(self, msg) -> None:
        if self.drop_while_busy:
            try:
                self.msg_queue.put_nowait(msg)
            except queue.Full:
                # 큐가 꽉 차 있으면 가장 오래된 기존 메시지를 꺼내서 버린 후 새 메시지를 넣어 최신 프레임을 유지합니다.
                try:
                    self.msg_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self.msg_queue.put_nowait(msg)
                except queue.Full:
                    pass
        else:
            self.msg_queue.put(msg)

    def worker_loop(self) -> None:
        """백그라운드에서 큐를 감시하며 전달된 프레임을 순차적으로 추론 처리하는 루프입니다."""
        while not rospy.is_shutdown():
            try:
                # 큐에 작업 메시지가 들어올 때까지 대기합니다. (타임아웃 0.1초)
                msg = self.msg_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                if self.input_mode == "bundle":
                    self.process_bundle(msg)
                else:
                    self.process_image(msg)
            except Exception as exc:
                import traceback
                rospy.logerr_throttle(1.0, "segmentation worker processing failed: %s\n%s", exc, traceback.format_exc())
            finally:
                # 작업이 성공적으로 수행되었거나 에러로 인해 끝났음을 큐에 신호로 알립니다.
                self.msg_queue.task_done()

    def process_image(self, msg: Image) -> None:
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

    def process_bundle(self, msg: CameraBundle) -> None:
        try:
            callback_start = time.perf_counter()
            if not msg.images:
                rospy.logwarn_throttle(1.0, "segmentation inference skipped empty CameraBundle")
                return

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

            # CameraBundle 안의 이미지들을 한 번의 model forward로 처리한다.
            # 기본 4채널 bundle이면 4개 이미지가 그대로 PyTorch batch로 들어간다.
            class_msgs, inference_ms = self.predict_class_map_msgs(msg.images)
            class_bundle.images.extend(class_msgs)

            self.class_map_bundle_pub.publish(class_bundle)

            if self.log_timing:
                callback_ms = (time.perf_counter() - callback_start) * 1000.0
                fps = 1000.0 / inference_ms if inference_ms > 0.0 else 0.0
                rospy.loginfo_throttle(
                    self.timing_log_interval,
                    "segmentation bundle timing: images=%d inference=%.2f ms callback=%.2f ms approx_bundle_fps=%.2f",
                    len(msg.images),
                    inference_ms,
                    callback_ms,
                    fps,
                )
        except (CvBridgeError, ValueError, RuntimeError) as exc:
            rospy.logerr_throttle(1.0, "segmentation bundle inference failed: %s", exc)

    def predict_class_map_msg(self, msg: Image) -> Tuple[Image, float]:
        # ROS Image -> OpenCV BGR/RGB image.
        # bgr8/rgb8는 빠른 처리를 위해 cv_bridge의 메모리 카피 과정 없이 바로 numpy 뷰(frombuffer)를 생성합니다.
        # predictor는 내부에서 학습 때와 같은 RGB/resize/tensor 변환을 수행한다.
        # desired_encoding은 input_encoding 파라미터를 따른다. 기본값은 CarMaker 카메라에서
        # 흔히 쓰는 bgr8이며, 입력 토픽이 rgb8이면 YAML/launch에서 바꿔주면 된다.
        if msg.encoding in {"bgr8", "rgb8"}:
            image_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        else:
            image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.input_encoding)

        # 실제 PyTorch forward 시간만 따로 재서 로그에 남긴다.
        inference_start = time.perf_counter()
        result = self.predictor.predict(image_np, color_order=msg.encoding if msg.encoding in {"bgr8", "rgb8"} else "bgr")
        inference_ms = (time.perf_counter() - inference_start) * 1000.0

        # class_map은 원본 입력 이미지 크기(또는 resize_output=False에 따른 크기)로 되돌려 publish한다.
        # 그래서 다른 노드가 같은 camera frame/header 기준으로 바로 맞춰 쓸 수 있다.
        # encoding은 mono8이다. 픽셀 밝기값이 시각화용 intensity가 아니라 class id이다.
        # 예: 0=background, 1=lane, 2=landmark.
        # 여기서는 cv_bridge 오버헤드를 우회하기 위해 직접 ROS Image 메시지를 생성하고 bytes 대입을 수행합니다.
        class_msg = Image()
        class_msg.header = msg.header
        class_msg.height = result.class_map.shape[0]
        class_msg.width = result.class_map.shape[1]
        class_msg.encoding = "mono8"
        class_msg.is_bigendian = 0
        class_msg.step = class_msg.width
        class_msg.data = result.class_map.tobytes()
        return class_msg, inference_ms

    def predict_class_map_msgs(self, msgs: list[Image]) -> Tuple[list[Image], float]:
        # ROS Image 배열을 OpenCV 배열 리스트로 바꾼 뒤 predictor의 batch API에 넘긴다.
        # 결과 순서는 입력 순서와 같으므로 CameraBundle의 names/images index 규칙이 유지된다.
        # bgr8/rgb8 인코딩은 복사 없는 고속 처리를 위해 numpy 뷰(frombuffer) 형태로 리스트를 구성합니다.
        images_np = []
        for msg in msgs:
            if msg.encoding in {"bgr8", "rgb8"}:
                images_np.append(np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3))
            else:
                images_np.append(self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.input_encoding))

        # 첫 이미지의 인코딩을 기준으로 컬러 정렬 결정
        color_order = msgs[0].encoding if msgs[0].encoding in {"bgr8", "rgb8"} else "bgr"

        inference_start = time.perf_counter()
        results = self.predictor.predict_batch(images_np, color_order=color_order)
        inference_ms = (time.perf_counter() - inference_start) * 1000.0

        class_msgs = []
        for msg, result in zip(msgs, results):
            # cv_bridge 오버헤드를 배제하기 위해 직접 ROS Image 메시지를 생성하고 bytes를 주입합니다.
            class_msg = Image()
            class_msg.header = msg.header
            class_msg.height = result.class_map.shape[0]
            class_msg.width = result.class_map.shape[1]
            class_msg.encoding = "mono8"
            class_msg.is_bigendian = 0
            class_msg.step = class_msg.width
            class_msg.data = result.class_map.tobytes()
            class_msgs.append(class_msg)
        return class_msgs, inference_ms


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
