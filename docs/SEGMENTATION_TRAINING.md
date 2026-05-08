# CarMaker Semantic Segmentation Training

이 문서는 CarMaker raw/GT 이미지로 semantic segmentation을 학습하는 방법을 설명합니다.

현재 학습 코드는 ROS launch가 아니라 독립 Python 폴더에서 직접 실행하는 방식을 기본으로 합니다.

```text
src/segmentation/
```

`carmaker_image` 패키지는 여전히 rosbag 이미지 추출과 GT 후처리를 담당하고, `src/segmentation`은 학습과 검증을 담당합니다.

이미 실제 환경에서 rosbag 수집과 후처리가 끝나서 `src/carmaker_image/data` 아래에 데이터가 있다면, 1번과 2번은 다시 실행하지 않아도 됩니다. 바로 Dataset debug부터 시작하면 됩니다.

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
```

핵심 원칙:

```text
데이터 구조는 Adapter가 숨긴다.
이미지/tensor 변환은 Dataset이 담당한다.
모델 선택은 config와 model registry가 담당한다.
학습 루프는 공통으로 유지한다.
```

## 실행 순서

### 1. rosbag에서 raw/GT 이미지 추출

이미 데이터가 있으면 이 단계는 건너뜁니다.

이 단계는 아직 ROS launch를 사용합니다.

```bash
roslaunch carmaker_image batch_extract.launch
```

결과:

```text
src/carmaker_image/data/raw_images/
src/carmaker_image/data/gt_images/
src/carmaker_image/data/csv/manifest.csv
```

### 2. GT를 class-id mask로 후처리

이미 `raw_post_processed`와 `gt_post_processed`가 있으면 이 단계는 건너뜁니다.

```bash
roslaunch carmaker_image apply_mask.launch recursive:=true
```

결과:

```text
src/carmaker_image/data/raw_post_processed/
src/carmaker_image/data/gt_post_processed/
```

현재 class id:

```text
0 = background
1 = lane
2 = landmark
```

### 3. Dataset debug

학습 전에 image/mask pair가 정상인지 확인합니다.

이미 데이터가 확보된 상태라면 여기부터 실행하면 됩니다.

```bash
python3 src/segmentation/tools/debug_dataset.py --count 10 --image-size 1920,1080
```

결과:

```text
src/segmentation/runs/debug_dataset/
```

확인할 것:

```text
raw image와 GT mask가 같은 장면인가?
front/left/rear/right 카메라가 섞이지 않았는가?
mask 위치가 이미지와 맞는가?
mask 값이 0, 1, 2로 잘 들어있는가?
```

기본으로 읽는 데이터 위치:

```text
src/carmaker_image/data
```

다른 위치에 데이터가 있으면 직접 지정합니다.

```bash
python3 src/segmentation/tools/debug_dataset.py \
  --data-root /path/to/carmaker_image/data \
  --manifest /path/to/carmaker_image/data/csv/manifest.csv \
  --count 10
```

### 4. Overfit smoke test

작은 데이터만 사용해서 모델이 외울 수 있는지 확인합니다.

```bash
python3 src/segmentation/train.py --overfit-count 10 --epochs 50
```

`--overfit-count 10`은 전체 dataset에서 앞의 10개 sample만 자동으로 골라 사용합니다. 따로 10장을 복사해서 준비할 필요는 없습니다.

정상이라면:

```text
train/loss가 빠르게 내려간다.
pred_overlay가 gt_overlay와 점점 비슷해진다.
```

10장도 못 외우면 의심할 것:

```text
raw와 GT가 매칭되지 않음
mask class id가 잘못됨
resize 방식 문제
모델 output channel 수 문제
loss 입력 shape 문제
learning rate 문제
```

### 5. 전체 학습

```bash
python3 src/segmentation/train.py
```

다른 데이터 위치를 사용하려면:

```bash
python3 src/segmentation/train.py \
  --data-root /path/to/carmaker_image/data \
  --manifest /path/to/carmaker_image/data/csv/manifest.csv
```

결과:

```text
src/segmentation/runs/unet_carmaker/
  checkpoints/
    best.pt
    last.pt
  events.out.tfevents...
```

### 6. TensorBoard

다른 터미널 탭에서 실행합니다.

```bash
tensorboard --logdir src/segmentation/runs --host 0.0.0.0
```

확인 항목:

```text
train/loss
val/loss
val/miou
val/dice
val/pixel_accuracy
val/psnr
val/iou/class_0
val/iou/class_1
val/iou/class_2
debug/*/gt_overlay
debug/*/pred_overlay
```

## 주요 파일 역할

| 파일 | 역할 |
|------|------|
| `src/segmentation/adapters.py` | CarMaker 데이터 폴더와 CSV를 읽어서 image/mask pair 목록을 만든다 |
| `src/segmentation/dataset.py` | PNG 이미지를 읽고 PyTorch tensor로 변환한다 |
| `src/segmentation/models.py` | U-Net, 임시 TinyFCN, model registry, `build_model`을 관리한다 |
| `src/segmentation/losses.py` | CrossEntropy, Dice, CE+Dice, Focal loss를 만든다 |
| `src/segmentation/metrics.py` | mIoU, class별 IoU, Dice, Pixel Accuracy, PSNR을 계산한다 |
| `src/segmentation/utils/visualization.py` | mask colorize와 overlay 이미지를 만든다 |
| `src/segmentation/tools/debug_dataset.py` | 학습 전 dataset pair와 mask 값을 확인한다 |
| `src/segmentation/train.py` | 학습, 검증, TensorBoard, checkpoint 저장을 수행한다 |
| `src/segmentation/config/segmentation_unet.yaml` | 학습 설정 파일 |

## Config

기본 설정 파일:

```text
src/segmentation/config/segmentation_unet.yaml
```

주요 설정:

```yaml
data_root: ../../carmaker_image/data
manifest: ../../carmaker_image/data/csv/manifest.csv
cameras: front,left,rear,right

image_size: [1920, 1080]
batch_size: 1
epochs: 30
learning_rate: 0.001

model:
  name: unet
  in_channels: 3
  base_channels: 32

loss: cross_entropy
run_dir: ../runs/unet_carmaker
```

원본 1920x1080 이미지를 통으로 사용합니다. GPU 메모리가 부족하면 먼저 아래 값을 줄입니다.

```yaml
model:
  base_channels: 16
```

그래도 부족하면 `image_size`를 16:9 비율로 낮춥니다.

```yaml
image_size: [1280, 720]
```

또는:

```yaml
image_size: [960, 540]
```

## 모델 교체

`train.py`는 직접 `UNet()`을 만들지 않고 아래 함수를 사용합니다.

```python
model = build_model(cfg, num_classes=adapter.num_classes)
```

현재 model registry:

| Config name | Class | 설명 |
|-------------|-------|------|
| `unet` | `UNet` | 기본 baseline 모델 |
| `tiny_fcn` | `TinyFCN` | 임시 모델, smoke test / 모델 교체 구조 확인용 |

`TinyFCN`은 정식 실험용 모델이 아니라 임시 예시용 모델입니다.

모델을 바꾸려면 config에서:

```yaml
model:
  name: tiny_fcn
```

새 모델을 추가하려면:

```text
1. src/segmentation/models.py에 nn.Module class 추가
2. MODEL_REGISTRY에 등록
3. config의 model.name 변경
```

## Loss 선택

기본:

```yaml
loss: cross_entropy
```

지원:

```text
cross_entropy
dice
ce_dice
focal
```

추천 순서:

```text
1. cross_entropy
2. weighted cross_entropy
3. ce_dice
4. focal
```

class imbalance가 심하면:

```yaml
class_weights: [0.2, 2.0, 3.0]
```

## 확장 기준

| 바꾸고 싶은 것 | 수정할 위치 |
|----------------|-------------|
| 새 데이터셋 추가 | `src/segmentation/adapters.py` |
| augmentation 추가 | `src/segmentation/dataset.py` 또는 transform 모듈 |
| 새 모델 추가 | `src/segmentation/models.py` |
| loss 추가 | `src/segmentation/losses.py` |
| metric 추가 | `src/segmentation/metrics.py` |
| 학습 방식 변경 | `src/segmentation/train.py` |
| 실험 설정 변경 | `src/segmentation/config/segmentation_unet.yaml` |
