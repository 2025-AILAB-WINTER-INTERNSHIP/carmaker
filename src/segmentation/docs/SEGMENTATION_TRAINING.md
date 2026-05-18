# CarMaker Semantic Segmentation Training

이 문서는 CarMaker에서 추출한 raw 이미지와 GT 이미지를 사용해 semantic segmentation 모델을 학습하는 방법을 설명한다.

현재 구조에서는 `carmaker_image` 패키지가 rosbag 이미지 추출과 GT 후처리를 담당하고, `src/segmentation` 패키지가 dataset 구성, 모델 학습, 검증, TensorBoard 기록을 담당한다. 학습된 checkpoint를 ROS topic으로 연결하는 runtime interface는 `src/segmentation_ros` 패키지가 담당한다.

```text
src/carmaker_image      # rosbag 이미지 추출, GT 후처리
src/segmentation        # dataset, model, loss, metric, train loop
src/segmentation_ros    # 학습 checkpoint를 사용하는 ROS1 inference node
```

ROS inference 구조와 topic interface는 `docs/SEGMENTATION_ROS_INTERFACE.md`에 정리되어 있다.

이미 `src/carmaker_image/data` 아래에 필요한 데이터가 준비되어 있다면 rosbag 추출과 후처리 단계는 다시 실행하지 않아도 된다. 바로 dataset debug 또는 학습 단계부터 시작하면 된다.

## 전체 흐름

```text
rosbag
  |
  | carmaker_image/batch_extract.py
  v
raw_images + gt_images + csv/manifest.csv
  |
  | carmaker_image/apply_mask.py
  v
raw_post_processed + gt_post_processed
  |
  | src/segmentation/adapters.py
  v
SegmentationSample(image_path, mask_path, camera)
  |
  | src/segmentation/dataset.py
  v
image tensor: [B, 3, H, W]
mask tensor:  [B, H, W]
  |
  | src/segmentation/models.py
  v
logits: [B, C, H, W]
  |
  | losses.py + metrics.py
  v
checkpoint + TensorBoard logs
  |
  | src/segmentation/inference.py
  v
restored model + class_map
  |
  | src/segmentation_ros/scripts/segmentation_inference_node.py
  v
/segmentation/class_map
```

역할 분리는 다음과 같다.

```text
Adapter:
  데이터 폴더 구조와 manifest.csv를 읽고 image/mask pair 목록을 만든다.

Dataset:
  실제 PNG 파일을 읽고 resize 후 PyTorch tensor로 변환한다.

Model:
  RGB image tensor를 입력받아 pixel별 class logits를 출력한다.

Train loop:
  train/validation/test split, 학습, 검증, checkpoint, TensorBoard 기록을 수행한다.
```

## 입력 이미지와 GT Mask

현재 학습 입력 이미지는 기본적으로 **후처리 전 raw 이미지**를 사용한다.

```text
input image:
  src/carmaker_image/data/raw_images/...

target mask:
  src/carmaker_image/data/gt_post_processed/...
```

즉 모델이 보는 RGB 입력은 원본 raw 이미지이고, loss 계산에 사용하는 target은 `apply_mask.py`가 만든 class-id mask이다.

현재 class id는 다음과 같다.

```text
0 = background
1 = lane
2 = landmark
```

config의 기본값은 다음과 같다.

```yaml
use_raw_post_processed: false
```

이 값이 `false`이면 manifest에서 `raw` 컬럼을 우선 사용한다.

```text
raw -> raw_post
```

후처리된 raw 이미지를 입력으로 쓰고 싶다면 다음처럼 바꾼다.

```yaml
use_raw_post_processed: true
```

이 경우 manifest에서 `raw_post` 컬럼을 우선 사용한다.

```text
raw_post -> raw
```

GT mask는 학습용 class id가 저장된 `gt_post`를 계속 사용한다.

## 실행 순서

### 1. rosbag에서 Raw/GT 이미지 추출

이미 데이터가 있다면 이 단계는 건너뛰어도 된다.

Docker/ROS 환경 안에서 실행한다.

```bash
roslaunch carmaker_image batch_extract.launch
```

기본 입력 bag 경로:

```text
src/carmaker_image/data/bags/
```

결과:

```text
src/carmaker_image/data/raw_images/
src/carmaker_image/data/gt_images/
src/carmaker_image/data/csv/manifest.csv
```

bag 목록만 먼저 확인하려면 dry run을 사용한다.

```bash
roslaunch carmaker_image batch_extract.launch dry_run:=true
```

bag directory를 직접 지정하려면 다음처럼 실행한다.

```bash
roslaunch carmaker_image batch_extract.launch \
  bag_dirs:=/path/to/bags
```

### 2. GT를 Class-ID Mask로 후처리

이미 `gt_post_processed`가 있다면 이 단계도 건너뛰어도 된다.

```bash
roslaunch carmaker_image apply_mask.launch recursive:=true
```

결과:

```text
src/carmaker_image/data/raw_post_processed/
src/carmaker_image/data/gt_post_processed/
```

mask를 JSON에서 다시 강제로 만들고 싶으면 다음처럼 실행한다.

```bash
roslaunch carmaker_image apply_mask.launch \
  recursive:=true \
  refresh_masks:=true
```

### 3. Dataset Debug

학습 전에 image/mask pair가 정상인지 확인한다.

```bash
python3 src/segmentation/tools/debug_dataset.py \
  --count 10 \
  --image-size 1920,1080
```

결과 overlay는 다음 위치에 저장된다.

```text
src/segmentation/runs/debug_dataset/
```

확인할 항목:

```text
raw image와 GT mask가 같은 장면인지
front/left/rear/right camera가 제대로 들어오는지
mask 위치가 이미지와 맞는지
mask 값이 0, 1, 2로만 구성되는지
```

다른 데이터 경로를 사용할 때:

```bash
python3 src/segmentation/tools/debug_dataset.py \
  --data-root /path/to/carmaker_image/data \
  --manifest /path/to/carmaker_image/data/csv/manifest.csv \
  --count 10 \
  --image-size 1920,1080
```

### 4. Overfit Smoke Test

전체 학습 전에 작은 데이터만 사용해서 모델이 외울 수 있는지 확인한다.

```bash
python3 src/segmentation/train.py \
  --overfit-count 10 \
  --epochs 50
```

정상이라면 다음 경향이 보여야 한다.

```text
train/loss가 빠르게 내려간다.
val/all/miou가 올라간다.
pred_overlay가 gt_overlay와 점점 비슷해진다.
```

10장도 외우지 못하면 다음을 먼저 의심한다.

```text
raw image와 GT mask가 잘못 매칭됨
mask class id가 잘못됨
resize 방식 문제
model output channel 수 문제
loss 입력 shape 문제
learning rate 문제
```

### 5. 전체 학습

기본 config로 학습한다.

```bash
python3 src/segmentation/train.py
```

학습과 동시에 TensorBoard를 띄우려면 다음처럼 실행한다.

```bash
uv run src/segmentation/train.py \
  --tensorboard \
  --tensorboard-port 6006
```

다른 데이터 경로를 사용하려면 다음처럼 지정한다.

```bash
python3 src/segmentation/train.py \
  --data-root /path/to/carmaker_image/data \
  --manifest /path/to/carmaker_image/data/csv/manifest.csv
```

실험별로 run directory를 분리하고 싶으면 `--run-dir`를 지정한다.

```bash
python3 src/segmentation/train.py \
  --run-dir src/segmentation/runs/unet_raw_seed42_exp01
```

`--run-dir`를 직접 지정하지 않으면 config의 `run_dir`를 base directory로 사용하고,
그 아래에 `{loss}_ep{epochs}_{timestamp}` 형식의 run directory를 자동 생성한다.

결과:

```text
src/segmentation/runs/focal_ep16_20260514_175830/
  checkpoints/
    best.ckpt
    last.ckpt
    epoch_010.ckpt
    ...
  events.out.tfevents...
```

### 6. ROS Inference Node 실행

학습이 끝나면 `best.ckpt` 또는 `last.ckpt`를 ROS inference node 입력으로 사용한다. 일반적으로 validation mIoU가 가장 좋았던 `best.ckpt`를 먼저 사용한다.

```bash
catkin_make
source devel/setup.bash
```

```bash
roslaunch segmentation_ros segmentation_inference.launch \
  checkpoint_path:=/absolute/path/to/best.ckpt \
  image_topic:=/camera/image_raw
```

출력 topic:

```text
/segmentation/class_map
```

출력 형태:

```text
sensor_msgs/Image
encoding: mono8
pixel value: class id
```

현재 class id:

```text
0 = background
1 = lane
2 = landmark
```

자세한 ROS interface 구조는 `docs/SEGMENTATION_ROS_INTERFACE.md`를 참고한다.

## Dataset Split

현재 dataset은 학습 시 다음 비율로 나뉜다.

```text
train 70%
val   20%
test  10%
```

config:

```yaml
val_ratio: 0.2
test_ratio: 0.1
seed: 42
```

`seed`는 random split을 고정하기 위한 값이다. 같은 dataset 목록과 같은 seed를 쓰면 매번 같은 train/val/test 분배가 나온다.

```text
seed: 42  -> 항상 같은 split
seed: 123 -> 다른 split
```

숫자 `42` 자체가 특별한 의미를 갖는 것은 아니고, 실험 재현성을 위해 고정해 둔 값이다.

### Camera-Stratified Split

현재 split은 단순 전체 랜덤 split이 아니라 카메라별 stratified split이다.

config:

```yaml
stratify_by_camera: true
```

이 값이 `true`이면 각 카메라 그룹 안에서 따로 70/20/10 split을 수행한 뒤 합친다.

```text
front -> train 70 / val 20 / test 10
rear  -> train 70 / val 20 / test 10
left  -> train 70 / val 20 / test 10
right -> train 70 / val 20 / test 10
```

이렇게 하면 train/val/test 각각에 `front`, `rear`, `left`, `right`가 균형 있게 들어간다. 카메라 위치별 시야 차이가 크기 때문에 전체 랜덤 split보다 안정적으로 성능을 비교할 수 있다.

주의할 점은 이 split이 여전히 이미지 단위 split이라는 점이다. 더 엄밀한 최종 평가는 bag 또는 scenario 단위 split을 고려하는 것이 좋다. 같은 scenario의 연속 frame이 train과 val/test에 섞이면 validation/test 성능이 실제보다 좋게 보일 수 있다.

### 각 Split의 역할

```text
train:
  model weight를 직접 업데이트하는 데이터

validation:
  매 epoch마다 평가
  best.ckpt 선택 기준
  hyperparameter 선택 기준

test:
  학습이 끝난 뒤 best.ckpt로 최종 1회 평가
  튜닝에는 사용하지 않는 최종 보고용 데이터
```

현재 `train.py`는 validation mIoU가 가장 높은 checkpoint를 `best.ckpt`로 저장한다.

```text
src/segmentation/runs/unet_carmaker/checkpoints/best.ckpt
```

학습이 끝나면 `best.ckpt`를 다시 load해서 test set을 평가한다.

```text
[test] loss=... miou=... dice=... acc=...
```

마지막 epoch 상태는 항상 `last.ckpt`로 저장된다.

```text
src/segmentation/runs/unet_carmaker/checkpoints/last.ckpt
```

## tqdm 진행 표시

학습 진행은 `tqdm` progress bar로 표시된다.

```text
epochs: ...
train 001: ...
val 001: ...
```

epoch마다 여러 줄의 print를 남기는 대신, 현재 진행률과 주요 metric을 한 줄에서 갱신한다.

예시:

```text
epochs:  20%|██        | 10/50 [..., train_loss=..., val_loss=..., miou=...]
```

`tqdm`이 설치되어 있지 않으면 progress bar 없이 일반 iterable처럼 동작한다. 필요하면 컨테이너 안에서 다음을 실행한다.

```bash
uv pip install tqdm
```

## TensorBoard

TensorBoard 로그는 `run_dir` 아래에 저장된다.

config:

```yaml
run_dir: ../runs
```

`--run-dir`를 직접 지정하지 않으면 `train.py`가 `{loss}_ep{epochs}_{timestamp}` 형식의
하위 폴더를 만들고, 그 안에 event 파일을 저장한다.

```text
src/segmentation/runs/focal_ep16_20260514_175830/events.out.tfevents...
```

학습이 끝난 뒤에도 TensorBoard를 다시 실행하면 과거 로그를 볼 수 있다.

```bash
tensorboard --logdir src/segmentation/runs --host 0.0.0.0 --port 6006
```

브라우저:

```text
http://localhost:6006
```

Docker 환경에서 host 브라우저로 접근하려면 compose 또는 docker run에서 `6006:6006` 포트가 열려 있어야 한다.

### TensorBoard Scalar Tags

#### Train

```text
train/loss
train/lr
```

#### Validation 통합 Metric

기존 통합 tag:

```text
val/loss
val/miou
val/dice
val/pixel_accuracy
val/psnr
val/iou/class_0
val/iou/class_1
val/iou/class_2
```

카메라별 metric과 같은 구조로 비교하기 위한 통합 alias:

```text
val/all/loss
val/all/miou
val/all/dice
val/all/pixel_accuracy
val/all/psnr
val/all/iou/class_0
val/all/iou/class_1
val/all/iou/class_2
```

#### Validation 카메라별 Metric

각 카메라별로 아래 metric이 기록된다.

```text
val/front/loss
val/front/miou
val/front/dice
val/front/pixel_accuracy
val/front/psnr
val/front/iou/class_0
val/front/iou/class_1
val/front/iou/class_2
```

```text
val/rear/loss
val/rear/miou
val/rear/dice
val/rear/pixel_accuracy
val/rear/psnr
val/rear/iou/class_0
val/rear/iou/class_1
val/rear/iou/class_2
```

```text
val/left/loss
val/left/miou
val/left/dice
val/left/pixel_accuracy
val/left/psnr
val/left/iou/class_0
val/left/iou/class_1
val/left/iou/class_2
```

```text
val/right/loss
val/right/miou
val/right/dice
val/right/pixel_accuracy
val/right/psnr
val/right/iou/class_0
val/right/iou/class_1
val/right/iou/class_2
```

#### Test 통합 Metric

학습 종료 후 `best.ckpt` 기준으로 한 번 기록된다.

```text
test/loss
test/miou
test/dice
test/pixel_accuracy
test/psnr
test/iou/class_0
test/iou/class_1
test/iou/class_2
```

통합 alias:

```text
test/all/loss
test/all/miou
test/all/dice
test/all/pixel_accuracy
test/all/psnr
test/all/iou/class_0
test/all/iou/class_1
test/all/iou/class_2
```

#### Test 카메라별 Metric

```text
test/front/*
test/rear/*
test/left/*
test/right/*
```

각 camera tag 아래에는 validation과 동일하게 다음 metric이 들어간다.

```text
loss
miou
dice
pixel_accuracy
psnr
iou/class_0
iou/class_1
iou/class_2
```

### TensorBoard Image Tags

TensorBoard Images 탭에는 고정 debug sample overlay가 기록된다.

```text
debug/train/gt_grid
debug/train/pred_grid
debug/val/gt_grid
debug/val/pred_grid
debug/test/gt_grid
debug/test/pred_grid
```

각 split마다 하나의 grid로 기록된다. grid 내부는 camera별 column으로 묶이며, 예를 들어 `debug_image_count: 8`이고 `front/left/rear/right`가 모두 있으면 8행 x 4열 형태로 최대 32장이 들어간다. `gt_grid`는 정답 mask overlay이고, `pred_grid`는 모델 예측 mask overlay이다.

debug sample은 고정되어 있으므로 `gt_grid`는 첫 epoch에만 기록한다. `pred_grid`는 모델 예측 변화를 보기 위해 첫 epoch과 `image_log_interval`마다 기록한다.

### 어떤 Metric을 우선 볼 것인가

초기 학습 확인:

```text
train/loss
val/loss
val/all/miou
```

실제 segmentation 품질 확인:

```text
val/all/miou
val/all/iou/class_1
val/all/iou/class_2
```

카메라별 문제 확인:

```text
val/front/miou
val/rear/miou
val/left/miou
val/right/miou
```

차선과 landmark를 더 자세히 보려면:

```text
val/front/iou/class_1
val/front/iou/class_2
val/rear/iou/class_1
val/rear/iou/class_2
val/left/iou/class_1
val/left/iou/class_2
val/right/iou/class_1
val/right/iou/class_2
```

`pixel_accuracy`는 참고용으로만 보는 것이 좋다. 배경 pixel이 많으면 배경만 잘 맞춰도 높게 나올 수 있다.

## Config

기본 설정 파일:

```text
src/segmentation/config/segmentation_unet.yaml
```

현재 주요 설정:

```yaml
data_root: ../../carmaker_image/data
manifest: ../../carmaker_image/data/csv/manifest.csv
cameras: front,left,rear,right
use_raw_post_processed: false

image_size: [1920, 1080]
num_workers: 4
batch_size: 1

val_ratio: 0.2
test_ratio: 0.1
stratify_by_camera: true
seed: 42

epochs: 16
learning_rate: 0.001
weight_decay: 0.0001

model:
  name: unet
  in_channels: 3
  base_channels: 32
  activation: relu
  weight_init: he_normal

loss: focal
class_weights: [5.0, 15.0, 10.0]
checkpoint_interval: 4
image_log_interval: 1
run_dir: ../runs
```

원본 1920x1080 이미지를 그대로 사용하므로 GPU memory가 부족할 수 있다. 부족하면 먼저 `base_channels`를 줄인다.

```yaml
model:
  base_channels: 16
```

그래도 부족하면 16:9 비율을 유지하면서 `image_size`를 줄인다.

```yaml
image_size: [1280, 720]
```

또는:

```yaml
image_size: [960, 540]
```

## 모델 구조

기본 모델은 U-Net이다.

```yaml
model:
  name: unet
```

현재 model registry:

| Config name | Class | 설명 |
|-------------|-------|------|
| `unet` | `UNet` | 기본 baseline 모델 |
| `tiny_fcn` | `TinyFCN` | smoke test 또는 구조 확인용 작은 모델 |

모델을 교체하려면 config의 `model.name`을 바꾼다.

```yaml
model:
  name: tiny_fcn
```

activation function도 config에서 바꿀 수 있다. 기본값은 `relu`이다.

```yaml
model:
  activation: relu
```

지원 activation:

```text
relu
leaky_relu
elu
gelu
silu
```

`weight_init: he_normal` 또는 `he_uniform`을 사용할 때 `leaky_relu`는 Kaiming gain도 `leaky_relu` 기준으로 맞춘다. `gelu`, `silu`, `elu`는 기존 ReLU gain을 사용한다.

새 모델을 추가하려면 다음 순서로 수정한다.

```text
1. src/segmentation/models.py에 nn.Module class 추가
2. MODEL_REGISTRY에 등록
3. config의 model.name 변경
```

## Loss 선택

기본:

```yaml
loss: focal
```

지원 loss:

```text
cross_entropy
dice
ce_dice
focal
focal_dice
```

추천 순서:

```text
1. cross_entropy
2. weighted cross_entropy
3. ce_dice
4. focal
5. focal_dice
```

class imbalance가 심하면 class weight를 사용할 수 있다.

```yaml
class_weights: [5.0, 15.0, 10.0]
```

의미:

```text
background weight = 5.0
lane weight       = 15.0
landmark weight   = 10.0
```

`class_weights`는 모든 loss에 똑같이 적용되지 않는다.

```text
class_weights 적용:
- cross_entropy
- ce_dice의 CrossEntropy 항
- focal
- focal_dice의 Focal 항

class_weights 미적용:
- dice
- ce_dice의 Dice 항
- focal_dice의 Dice 항
```

현재 DiceLoss는 `dice_exclude_classes`에 지정된 class를 제외하고, 남은 class별 Dice를 같은 비중으로 평균낸다.
기본 config는 배경 class 0을 제외해 lane/landmark overlap 신호를 더 크게 본다.

```text
Dice_c = (2 * sum(p_c * y_c) + smooth) / (sum(p_c) + sum(y_c) + smooth)
DiceLoss = 1 - mean_c(Dice_c), c not in dice_exclude_classes
```

```yaml
dice_exclude_classes: [0]
```

FocalLoss는 정답 class 확률이 낮은 어려운 픽셀의 기여도를 더 크게 남긴다.

```text
CE = -log(p_t)
FocalLoss = (1 - p_t)^gamma * CE
```

focal_dice는 두 loss를 가중합으로 묶는다.

```text
focal_dice = 0.5 * FocalLoss + 1.0 * DiceLoss
```

추천 실험 순서:

```text
1. focal_dice + class_weights
2. focal_dice + no class_weights
3. ce_dice + class_weights
4. focal + class_weights
```

## 주요 파일 역할

| 파일 | 역할 |
|------|------|
| `src/segmentation/adapters.py` | CarMaker 데이터 구조와 manifest를 읽고 image/mask pair 목록 생성 |
| `src/segmentation/dataset.py` | PNG 이미지와 mask를 읽고 PyTorch tensor로 변환, train/val/test split |
| `src/segmentation/models.py` | U-Net, TinyFCN, model registry, `build_model` |
| `src/segmentation/losses.py` | CrossEntropy, Dice, CE+Dice, Focal, Focal+Dice loss |
| `src/segmentation/metrics.py` | mIoU, class IoU, Dice, Pixel Accuracy, PSNR |
| `src/segmentation/utils/visualization.py` | mask colorize, overlay image 생성 |
| `src/segmentation/tools/debug_dataset.py` | dataset pair와 mask 값 확인용 debug tool |
| `src/segmentation/train.py` | 학습, 검증, test 평가, TensorBoard, checkpoint 저장 |
| `src/segmentation/inference.py` | checkpoint 복원, 전처리, 모델 추론, class map 생성 |
| `src/segmentation/config/segmentation_unet.yaml` | 학습 설정 파일 |
| `src/segmentation_ros/scripts/segmentation_inference_node.py` | ROS Image subscribe, class map publish |
| `src/segmentation_ros/launch/segmentation_inference.launch` | ROS inference node 실행 설정 |

## 확장 기준

| 바꾸고 싶은 것 | 수정 위치 |
|----------------|-----------|
| 새 데이터셋 추가 | `src/segmentation/adapters.py` |
| augmentation 추가 | `src/segmentation/dataset.py` 또는 transform 모듈 |
| 새 모델 추가 | `src/segmentation/models.py` |
| loss 추가 | `src/segmentation/losses.py` |
| metric 추가 | `src/segmentation/metrics.py` |
| 학습 방식 변경 | `src/segmentation/train.py` |
| 실험 설정 변경 | `src/segmentation/config/segmentation_unet.yaml` |
| ROS inference topic 변경 | `src/segmentation_ros/config/segmentation_inference.yaml` 또는 launch argument |
