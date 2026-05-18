# CarMaker Segmentation ROS Interface

이 문서는 학습된 semantic segmentation checkpoint를 ROS1에서 실시간 inference node로 사용하는 구조를 설명한다.

## 목적

학습 코드는 `src/segmentation`에 있고, ROS 입출력 코드는 `src/segmentation_ros`에 있다. 두 영역을 분리해서 모델 추론 로직은 ROS 없이도 테스트할 수 있고, ROS node는 topic subscribe/publish만 담당한다.

```text
src/segmentation
  inference.py      # checkpoint 복원, 전처리, 모델 실행, class_map 생성
  models.py         # U-Net/TinyFCN 및 build_model
  train.py          # 학습 및 checkpoint 저장

src/segmentation_ros
  scripts/segmentation_inference_node.py
  launch/segmentation_inference.launch
  config/segmentation_inference.yaml
  package.xml
  CMakeLists.txt
```

## 전체 파이프라인

기본 실행은 `input_mode: "bundle"`이다. synchronizer가 묶어 보낸 여러 카메라 이미지를 `CameraBundle`로 받고, 결과도 같은 index 순서를 유지한 `CameraBundle`로 publish한다.

```text
/synced/bundle
  carmaker_msgs/CameraBundle
  images = [front, left, rear, right]
        |
        v
segmentation_ros/scripts/segmentation_inference_node.py
  CameraBundle.images -> OpenCV BGR image list
        |
        v
segmentation/inference.py
  checkpoint load
  model restore
  each image: BGR -> RGB
  each image: resize to configured image_size if needed
  each image: HWC uint8 -> CHW float32
  each image: normalize 0..255 -> 0..1
  batch: [B, 3, H, W]
  model forward
  argmax(logits, dim=class) -> [B, H, W]
        |
        v
/segmentation/class_map_bundle
  carmaker_msgs/CameraBundle
  images = mono8 class maps
```

단일 카메라 디버깅이 필요하면 `input_mode:=image`로 실행할 수 있다.

```text
/camera/image_raw
  sensor_msgs/Image, bgr8
        |
        v
/segmentation/class_map
  sensor_msgs/Image, mono8
```

## 역할 분리

### `src/segmentation/inference.py`

`inference.py`는 ROS를 모르는 공용 추론 엔진이다.

담당하는 일:

- `best.pt` 또는 `last.pt` checkpoint 로드
- checkpoint 안의 `config` 읽기
- `models.py`의 `build_model()`로 학습 때와 같은 모델 구조 복원
- `model_state` weight 로드
- OpenCV 이미지 전처리
- PyTorch inference 실행
- 픽셀별 class id mask인 `class_map` 반환

반환 형태:

```python
SegmentationResult(
    class_map=np.ndarray,   # shape: H x W, dtype: uint8
    class_names=(...)
)
```

### `src/segmentation_ros/scripts/segmentation_inference_node.py`

ROS node는 모델 내부를 직접 다루지 않는다. 단일 이미지 모드에서는 `SegmentationPredictor.predict()`를 호출하고, bundle 모드에서는 `SegmentationPredictor.predict_batch()`를 호출한 뒤 결과를 ROS topic으로 publish한다.

담당하는 일:

- 입력 이미지 topic 또는 bundle topic subscribe
- `CvBridge`로 `sensor_msgs/Image` 배열을 OpenCV 이미지 배열로 변환
- `SegmentationPredictor` 호출
- `class_map`을 `sensor_msgs/Image` 또는 `CameraBundle`로 publish

## 입력 Topic

기본 bundle 입력:

```text
/synced/bundle
```

타입:

```text
carmaker_msgs/CameraBundle
```

`CameraBundle.images` 안의 각 이미지는 기본적으로 `bgr8`로 변환한다.

단일 이미지 모드 입력:

```text
/camera/image_raw
```

타입:

```text
sensor_msgs/Image
```

입력 topic과 mode는 launch argument 또는 ROS private parameter로 바꿀 수 있다.

## 출력 Topic

기본 bundle 출력:

```text
/segmentation/class_map_bundle
```

타입:

```text
carmaker_msgs/CameraBundle
```

의미:

```text
output.images[i] = input.images[i]에 대응하는 mono8 class_map
output.names = input.names
output.infos = input.infos
output.header = input.header
```

단일 이미지 모드 출력:

```text
/segmentation/class_map
```

타입:

```text
sensor_msgs/Image
```

각 class map image의 encoding:

```text
mono8
```

의미:

```text
class_map[y][x] = class_id
```

현재 class id:

```text
0 = background
1 = lane
2 = landmark
```

예시:

```text
0 0 0 0 0
0 1 1 1 0
0 1 1 1 0
0 0 2 0 0
```

이 결과는 색상 이미지가 아니라 픽셀별 class 번호 지도다. planner/controller가 사용할 기본 결과는 이 `class_map`이다.

## Bundle Batch Inference

bundle 모드에서는 기본적으로 bundle 안의 이미지들을 PyTorch batch로 묶어 한 번에 forward한다.

```text
CameraBundle.images 4개
  -> OpenCV image list 4개
  -> tensor [4, 3, H, W]
  -> logits [4, C, H, W]
  -> class maps [4, H, W]
  -> CameraBundle.images에 mono8 Image 4개로 다시 저장
```

입력과 출력의 index 순서는 유지된다. 예를 들어 `names[0] == "front"`라면 출력 bundle의 `images[0]`도 front 카메라의 class map이다.

`bundle_batch_inference`를 `false`로 두면 기존처럼 이미지 한 장씩 batch size 1로 순차 처리한다. 출력 topic과 메시지 구조는 바뀌지 않는다.

## Overlay를 Publish하지 않는 이유

현재 interface는 `/segmentation/overlay`를 publish하지 않는다.

이유:

- 실제 제어에 필요한 값은 `class_map`이다.
- overlay는 사람이 보는 디버깅용 이미지라서 필수 출력이 아니다.
- 실시간 inference에서는 불필요한 이미지 합성 연산을 줄이는 편이 낫다.

디버깅용 overlay가 다시 필요해지면 별도 debug node 또는 optional publisher로 추가하는 것이 좋다.

## 실행 방법

먼저 catkin workspace를 빌드하고 source한다.

```bash
catkin_make
source devel/setup.bash
```

launch 실행:

```bash
roslaunch segmentation_ros segmentation_inference.launch \
  checkpoint_path:=/absolute/path/to/best.pt \
  input_mode:=bundle \
  bundle_topic:=/synced/bundle
```

단일 이미지 모드:

```bash
roslaunch segmentation_ros segmentation_inference.launch \
  checkpoint_path:=/absolute/path/to/best.pt \
  input_mode:=image \
  image_topic:=/camera/image_raw
```

`rosrun`으로도 실행할 수 있다.

```bash
rosrun segmentation_ros segmentation_inference_node.py \
  _checkpoint_path:=/absolute/path/to/best.pt \
  _input_mode:=bundle \
  _bundle_topic:=/synced/bundle
```

주의:

```text
roslaunch: checkpoint_path:=...
rosrun:    _checkpoint_path:=...
```

## 주요 Parameter

| Parameter | 기본값 | 의미 |
|-----------|--------|------|
| `checkpoint_path` | `""` | 필수. `train.py`가 저장한 `best.pt` 또는 `last.pt` |
| `config_path` | `""` | 선택. checkpoint 안의 config를 일부 덮어쓸 때 사용 |
| `device` | `""` | 비우면 CUDA 가능 시 `cuda`, 아니면 `cpu` |
| `image_size` | `""` | 비우면 checkpoint config의 `image_size` 사용 |
| `inference_precision` | `fp16` | forward precision. `fp32`, `fp16`, `bf16`, `auto` 중 선택 |
| `input_mode` | `bundle` | `bundle`이면 `CameraBundle`, `image`면 단일 `sensor_msgs/Image` 사용 |
| `bundle_topic` | `/synced/bundle` | bundle 모드에서 subscribe할 `CameraBundle` topic |
| `class_map_bundle_topic` | `/segmentation/class_map_bundle` | bundle 모드에서 publish할 class map bundle topic |
| `image_topic` | `/camera/image_raw` | image 모드에서 subscribe할 카메라 이미지 topic |
| `class_map_topic` | `/segmentation/class_map` | image 모드에서 publish할 class map topic |
| `input_encoding` | `bgr8` | `CvBridge` 입력 변환 encoding |
| `queue_size` | `1` | 오래된 프레임 누적을 줄이기 위한 ROS queue 크기 |
| `bundle_batch_inference` | `true` | bundle 이미지를 PyTorch batch로 묶어 한 번에 forward할지 여부 |
| `bundle_batch_size` | `4` | batch inference에서 한 번에 처리할 bundle image 수 |
| `log_timing` | `true` | inference latency와 대략적인 FPS를 ROS log에 출력 |
| `timing_log_interval` | `1.0` | timing log throttle 간격, 초 단위 |

## Checkpoint와 Config

`train.py`가 저장하는 checkpoint에는 다음 정보가 들어간다.

```text
model_state
optimizer_state
config
epoch
best_miou
```

inference에서는 `optimizer_state`를 사용하지 않는다. 필요한 것은 `config`와 `model_state`다.

```text
config      -> 모델 구조와 image_size 복원
model_state -> 학습된 weight 복원
```

## 확인 방법

node가 실행 중이면 topic이 떠야 한다.

```bash
rostopic list | grep segmentation
```

header 확인:

```bash
rostopic echo /segmentation/class_map_bundle/header
```

bundle 안의 카메라 index 확인:

```bash
rostopic echo -n 1 /segmentation/class_map_bundle/names
```

출력 bundle의 `images` 배열에는 위 `names`와 같은 순서로 `mono8` class map이 들어간다. 이미지 데이터가 크기 때문에 `rostopic echo`로 `images` 전체를 직접 찍는 것은 피하는 편이 좋다.

단일 이미지 모드에서는 다음처럼 확인한다.

```bash
rostopic echo /segmentation/class_map/header
rostopic echo -n 1 /segmentation/class_map/encoding
```

class map image의 정상 encoding:

```text
mono8
```

추론 속도는 node log에서 확인한다.

```text
segmentation bundle timing: images=4 inference=12.34 ms callback=14.20 ms approx_bundle_fps=81.04
```

launch에서 끄려면 다음처럼 실행한다.

```bash
roslaunch segmentation_ros segmentation_inference.launch \
  checkpoint_path:=/absolute/path/to/best.pt \
  log_timing:=false
```

## 다음 확장

현재 interface는 가장 작은 필수 출력인 `class_map`만 제공한다.

planner/controller가 바로 쓰기 쉬운 숫자 값이 필요해지면 다음 topic을 추가할 수 있다.

```text
/segmentation/stats
```

예상 값:

```text
lane_area_ratio
lane_centroid_x
lane_centroid_y
lane_center_offset_px
has_lane
landmark_area_ratio
```

이 확장은 `class_map`을 기반으로 한 후처리 단계이므로, 현재 inference node의 기본 출력과 분리해서 추가하는 것이 좋다.
