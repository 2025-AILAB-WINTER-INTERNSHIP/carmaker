# ROS1 Segmentation Runtime Inference 구조

## 1. 목적

이 문서는 학습이 완료된 semantic segmentation 모델을 ROS1 환경에서 runtime inference node로 실행하는 구조를 정리한다.

현재 기본 목표는 CarMaker 4채널 카메라 이미지를 `CameraBundle`로 받아, 학습된 U-Net 기반 모델로 각 픽셀의 class를 예측하고, controller 또는 planner가 사용할 수 있는 `class id map`을 다시 `CameraBundle` 형태로 publish하는 것이다.

```text
train.py
  -> 모델 학습
  -> best.pt / last.pt 저장

segmentation/inference.py
  -> checkpoint 로드
  -> 학습 당시 config 기반 모델 구조 복원
  -> weight 로드
  -> 이미지 전처리
  -> 모델 forward
  -> class_map 반환

segmentation_ros/scripts/segmentation_inference_node.py
  -> ROS topic subscribe
  -> CameraBundle 또는 Image를 OpenCV 이미지로 변환
  -> SegmentationPredictor 호출
  -> class_map을 ROS topic으로 publish
```

핵심 설계는 ML 코드와 ROS 입출력 코드를 분리하는 것이다.

```text
segmentation/inference.py
  = 모델 추론 엔진

segmentation_inference_node.py
  = ROS 입출력 어댑터
```

## 2. 현재 기본 파이프라인

현재 기본 실행 모드는 `bundle`이다.

```text
bag 또는 image_synchronizer
    |
    v
/synced/bundle
  carmaker_msgs/CameraBundle
    |
    v
segmentation_inference_node.py
    |
    v
/segmentation/class_map_bundle
  carmaker_msgs/CameraBundle
```

입력 bundle에는 여러 카메라 이미지가 한 메시지 안에 들어 있다.

```text
CameraBundle
  header
  names[]   = 카메라 이름 배열
  images[]  = 카메라 이미지 배열
  infos[]   = CameraInfo 배열
```

출력 bundle도 같은 타입을 사용한다. 다만 `images[]`의 의미만 바뀐다.

```text
입력 CameraBundle.images[]
  = 원본 카메라 이미지

출력 CameraBundle.images[]
  = mono8 segmentation class map
```

`header`, `names[]`, `infos[]`는 입력 기준을 유지한다. 따라서 downstream node는 기존 CameraBundle과 같은 index 규칙으로 결과를 읽으면 된다.

```text
input.names[0] == "front"
input.images[0] == front 원본 이미지

output.names[0] == "front"
output.images[0] == front class map
```

## 3. 단일 이미지 모드

단일 이미지 디버그가 필요할 때는 `input_mode:=image`를 사용할 수 있다.

```text
/camera/image_raw
  sensor_msgs/Image
    |
    v
segmentation_inference_node.py
    |
    v
/segmentation/class_map
  sensor_msgs/Image
```

이 모드는 모델 단독 확인이나 간단한 디버깅에 유용하다. 현재 runtime 기본 경로는 bundle 모드다.

## 4. 코드 구조

```text
src/segmentation/
  train.py
  models.py
  dataset.py
  inference.py

src/segmentation_ros/
  package.xml
  CMakeLists.txt
  scripts/segmentation_inference_node.py
  config/segmentation_inference.yaml
  launch/segmentation_inference.launch
  data/bags/
  data/models/

src/carmaker_msgs/
  msg/CameraBundle.msg
```

역할은 다음과 같다.

```text
src/segmentation
  ML 코드
  모델 정의, 학습, checkpoint 복원, 이미지 추론 담당

src/segmentation_ros
  ROS 코드
  topic subscribe/publish, launch, runtime parameter 담당

src/carmaker_msgs
  CameraBundle 등 custom ROS message 정의
```

## 5. CameraBundle 메시지

`CameraBundle`은 다음 구조다.

```text
std_msgs/Header header
string[] names
sensor_msgs/Image[] images
sensor_msgs/CameraInfo[] infos
```

현재 image synchronizer 설정 기준 카메라 입력은 다음 네 개다.

```text
/mono/fisheye/front/image_raw
/mono/fisheye/rear/image_raw
/mono/fisheye/left/image_raw
/mono/fisheye/right/image_raw
```

이 입력들은 `carmaker_image_synchronizer`를 거쳐 기본적으로 다음 topic으로 묶인다.

```text
/synced/bundle
```

## 6. inference.py 역할

`segmentation/inference.py`는 ROS를 모르는 순수 모델 추론 엔진이다.

담당하는 일은 다음과 같다.

```text
best.pt / last.pt checkpoint 읽기
checkpoint 안의 config 읽기
학습 때와 같은 모델 구조 복원
model_state weight 로드
입력 이미지 전처리
model forward 실행
argmax로 class id 선택
class_map 반환
```

checkpoint 복원 흐름은 개념적으로 다음과 같다.

```python
checkpoint = torch.load(best_pt)
cfg = checkpoint["config"]
model = build_model(cfg, num_classes=len(class_names))
model.load_state_dict(checkpoint["model_state"])
model.eval()
```

반환 타입은 다음과 같다.

```python
SegmentationResult(
    class_map=np.ndarray,   # H x W, uint8
    class_names=(...)
)
```

## 7. predict() 전처리와 후처리

ROS에서 받은 이미지는 `CvBridge`를 통해 OpenCV 이미지로 변환된다. 기본 encoding은 `bgr8`이다.

`predict()`는 다음 순서로 동작한다.

```text
OpenCV BGR image
    |
    v
RGB 변환
    |
    v
학습 때 사용한 image_size로 resize
    |
    v
HWC uint8 -> CHW float32
    |
    v
0..255 -> 0..1 normalization
    |
    v
U-Net forward
    |
    v
argmax(logits, dim=class)
    |
    v
class_map 생성
    |
    v
원본 입력 이미지 크기로 nearest resize
```

class map은 class id를 보존해야 하므로 출력 resize에는 bilinear가 아니라 nearest interpolation을 사용한다.

## 8. bundle batch inference

bundle 모드에서는 기본적으로 여러 카메라 이미지를 PyTorch batch로 묶어서 한 번에 forward한다.

```text
CameraBundle.images 4장
    |
    v
OpenCV image list 4장
    |
    v
tensor [4, 3, H, W]
    |
    v
logits [4, C, H, W]
    |
    v
class maps [4, H, W]
    |
    v
CameraBundle.images에 mono8 Image 4장 저장
```

결과 순서는 입력 순서와 같다.

```text
input.images[i]  -> output.images[i]
input.names[i]   -> output.names[i]
```

`bundle_batch_inference`를 `false`로 두면 각 이미지를 batch size 1로 순차 처리한다. 출력 topic과 메시지 구조는 바뀌지 않는다.

## 9. 입력과 출력

기본 bundle 입력:

```text
topic: /synced/bundle
type:  carmaker_msgs/CameraBundle
```

기본 bundle 출력:

```text
topic: /segmentation/class_map_bundle
type:  carmaker_msgs/CameraBundle
```

단일 이미지 입력:

```text
topic: /camera/image_raw
type:  sensor_msgs/Image
```

단일 이미지 출력:

```text
topic: /segmentation/class_map
type:  sensor_msgs/Image
```

출력 class map image의 encoding은 항상 다음과 같다.

```text
mono8
```

이 값은 밝기 이미지가 아니라 픽셀별 class id map이다.

```text
class_map[y][x] = class_id
```

현재 class id는 다음 기준을 사용한다.

```text
0 = background
1 = lane
2 = landmark
```

예를 들어 다음 조건은 `(x, y)` 위치를 모델이 lane으로 판단했다는 뜻이다.

```python
class_map[y][x] == 1
```

## 10. checkpoint 사용 방식

학습이 끝나면 `train.py`가 checkpoint를 저장한다.

대표 파일:

```text
best.pt
last.pt
```

checkpoint에는 보통 다음 정보가 들어 있다.

```text
epoch
model_state
optimizer_state
config
best_miou
```

inference에서 핵심적으로 사용하는 것은 두 가지다.

```text
config
  -> 모델 구조, class 정보, image_size 복원

model_state
  -> 학습된 weight 복원
```

`optimizer_state`는 학습 재개에는 필요하지만 runtime inference에는 필요하지 않다.

`config_path`를 별도로 주면 checkpoint 안의 config 위에 추가 설정을 덮어쓸 수 있다. 일반적인 runtime에서는 비워두고 checkpoint 안의 config를 사용하는 것이 안전하다.

## 11. 설정 파일

기본 설정은 다음 파일에 있다.

```text
src/segmentation_ros/config/segmentation_inference.yaml
```

현재 주요 기본값:

```yaml
checkpoint_path: ""
config_path: ""
device: ""
image_size: ""
inference_precision: "fp16"

input_mode: "bundle"
bundle_topic: "/synced/bundle"
class_map_bundle_topic: "/segmentation/class_map_bundle"

image_topic: "/camera/image_raw"
class_map_topic: "/segmentation/class_map"

input_encoding: "bgr8"
queue_size: 1
bundle_batch_inference: true
bundle_batch_size: 4

log_timing: true
timing_log_interval: 1.0
```

대부분의 runtime 값은 YAML에 이미 선언되어 있다. 따라서 실행 시 매번 넘겨야 하는 값은 보통 `checkpoint_path`뿐이다.

## 12. 실행 방법

먼저 workspace를 빌드하고 source한다.

```bash
catkin_make
source devel/setup.bash
```

학습된 모델은 다음 위치에 두는 것을 권장한다.

```text
src/segmentation_ros/data/models/best.pt
```

bag 파일은 다음 위치에 둔다.

```text
src/segmentation_ros/data/bags/your_file.bag
```

inference node 실행:

```bash
roslaunch segmentation_ros segmentation_inference.launch \
  checkpoint_path:=$(pwd)/src/segmentation_ros/data/models/best.pt
```

GPU를 명시하고 싶으면 다음처럼 실행한다.

```bash
roslaunch segmentation_ros segmentation_inference.launch \
  checkpoint_path:=$(pwd)/src/segmentation_ros/data/models/best.pt \
  device:=cuda:0
```

단일 이미지 모드로 실행할 때는 다음처럼 지정한다.

```bash
roslaunch segmentation_ros segmentation_inference.launch \
  checkpoint_path:=$(pwd)/src/segmentation_ros/data/models/best.pt \
  input_mode:=image \
  image_topic:=/camera/image_raw
```

`rosrun`으로도 실행할 수 있지만, 일반적으로는 YAML과 launch argument가 함께 적용되는 `roslaunch`를 권장한다.

```bash
rosrun segmentation_ros segmentation_inference_node.py \
  _checkpoint_path:=$(pwd)/src/segmentation_ros/data/models/best.pt \
  _input_mode:=bundle
```

주의:

```text
roslaunch argument: checkpoint_path:=...
rosrun private parameter: _checkpoint_path:=...
```

## 13. bag 재생 시나리오

먼저 bag 안의 topic을 확인한다.

```bash
rosbag info src/segmentation_ros/data/bags/your_file.bag
```

### bag 안에 /synced/bundle이 이미 있는 경우

image synchronizer 없이 inference node만 실행하면 된다.

```bash
roslaunch segmentation_ros segmentation_inference.launch \
  checkpoint_path:=$(pwd)/src/segmentation_ros/data/models/best.pt
```

다른 터미널에서 bag을 재생한다.

```bash
rosbag play --clock src/segmentation_ros/data/bags/your_file.bag
```

### bag 안에 원본 4채널 카메라 topic만 있는 경우

먼저 synchronizer를 실행해서 `/synced/bundle`을 만들어야 한다.

```bash
roslaunch carmaker_image_synchronizer image_synchronizer.launch use_sim_time:=true
```

그 다음 inference node를 실행한다.

```bash
roslaunch segmentation_ros segmentation_inference.launch \
  checkpoint_path:=$(pwd)/src/segmentation_ros/data/models/best.pt
```

마지막으로 bag을 재생한다.

```bash
rosbag play --clock src/segmentation_ros/data/bags/your_file.bag
```

## 14. 동작 확인

topic 확인:

```bash
rostopic list | grep segmentation
```

bundle 출력 주기 확인:

```bash
rostopic hz /segmentation/class_map_bundle
```

bundle header 확인:

```bash
rostopic echo -n 1 /segmentation/class_map_bundle/header
```

카메라 순서 확인:

```bash
rostopic echo -n 1 /segmentation/class_map_bundle/names
```

출력 image encoding 확인:

```bash
rostopic echo -n 1 /segmentation/class_map_bundle/images/0/encoding
```

정상 encoding:

```text
mono8
```

단일 이미지 모드에서는 다음으로 확인한다.

```bash
rostopic hz /segmentation/class_map
rostopic echo -n 1 /segmentation/class_map/encoding
```

## 15. 로그 확인

`log_timing: true`이면 inference node가 주기적으로 timing log를 출력한다.

bundle 모드 예:

```text
segmentation bundle timing: images=4 inference=12.34 ms callback=14.20 ms approx_bundle_fps=81.04
```

단일 이미지 모드 예:

```text
segmentation timing: inference=3.21 ms callback=4.02 ms approx_fps=311.52
```

로그를 끄고 싶으면 다음처럼 실행한다.

```bash
roslaunch segmentation_ros segmentation_inference.launch \
  checkpoint_path:=$(pwd)/src/segmentation_ros/data/models/best.pt \
  log_timing:=false
```

## 16. overlay를 기본 출력하지 않는 이유

현재 runtime interface의 핵심 출력은 `class_map`이다.

overlay는 사람이 보기 위한 디버그 이미지이고, planner/controller가 실제로 필요한 값은 픽셀별 class id다. 또한 overlay 생성은 색상 변환과 blending 연산이 추가되므로 runtime 경로에서는 기본 출력으로 두지 않는다.

필요하면 별도 debug node나 optional publisher로 다음 같은 topic을 추가하는 것이 좋다.

```text
/segmentation/overlay_bundle
/segmentation/overlay
```

## 17. 최종 요약

현재 구조의 핵심은 다음이다.

```text
train.py
  -> 모델 학습 및 checkpoint 저장

segmentation/inference.py
  -> checkpoint 복원, 전처리, 모델 실행, class_map 반환

segmentation_inference_node.py
  -> ROS CameraBundle/Image subscribe, inference.py 호출, class_map publish
```

현재 기본 runtime 입출력:

```text
input:
  /synced/bundle
  carmaker_msgs/CameraBundle

output:
  /segmentation/class_map_bundle
  carmaker_msgs/CameraBundle
```

출력 bundle의 `images[]`는 원본 이미지가 아니라 `mono8` class id map이다.

```text
0 = background
1 = lane
2 = landmark
```

따라서 최종적으로 controller 또는 planner는 `/segmentation/class_map_bundle`을 subscribe해서 각 카메라별 class map을 읽으면 된다.
