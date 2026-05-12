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

```text
/camera/image_raw
  sensor_msgs/Image, bgr8
        |
        v
segmentation_ros/scripts/segmentation_inference_node.py
  ROS Image -> OpenCV BGR image
        |
        v
segmentation/inference.py
  checkpoint load
  model restore
  BGR -> RGB
  resize to training image_size
  HWC uint8 -> CHW float32
  normalize 0..255 -> 0..1
  model forward
  argmax(logits, dim=class)
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

ROS node는 모델 내부를 직접 다루지 않는다. `SegmentationPredictor.predict()`를 호출하고 결과를 ROS topic으로 publish한다.

담당하는 일:

- 입력 이미지 topic subscribe
- `CvBridge`로 `sensor_msgs/Image`를 OpenCV 이미지로 변환
- `SegmentationPredictor` 호출
- `class_map`을 `sensor_msgs/Image`로 publish

## 입력 Topic

기본 입력:

```text
/camera/image_raw
```

타입:

```text
sensor_msgs/Image
```

기본 encoding:

```text
bgr8
```

입력 topic은 launch argument 또는 ROS private parameter로 바꿀 수 있다.

## 출력 Topic

기본 출력:

```text
/segmentation/class_map
```

타입:

```text
sensor_msgs/Image
```

encoding:

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
  image_topic:=/camera/image_raw
```

`rosrun`으로도 실행할 수 있다.

```bash
rosrun segmentation_ros segmentation_inference_node.py \
  _checkpoint_path:=/absolute/path/to/best.pt \
  _image_topic:=/camera/image_raw
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
| `image_topic` | `/camera/image_raw` | subscribe할 카메라 이미지 topic |
| `class_map_topic` | `/segmentation/class_map` | publish할 class map topic |
| `input_encoding` | `bgr8` | `CvBridge` 입력 변환 encoding |
| `queue_size` | `1` | 오래된 프레임 누적을 줄이기 위한 ROS queue 크기 |
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
rostopic echo /segmentation/class_map/header
```

이미지 형태 확인:

```bash
rostopic echo -n 1 /segmentation/class_map/encoding
```

정상 값:

```text
mono8
```

추론 속도는 node log에서 확인한다.

```text
segmentation timing: inference=12.34 ms callback=14.20 ms approx_fps=81.04
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
