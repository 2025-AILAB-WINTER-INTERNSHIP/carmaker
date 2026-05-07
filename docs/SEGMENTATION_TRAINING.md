# CarMaker Semantic Segmentation Training

이 문서는 `carmaker_image` 패키지에서 **fisheye semantic segmentation 학습 파이프라인이 어떻게 흘러가는지** 설명합니다.

목표는 다음과 같습니다.

```text
입력: CarMaker raw/fisheye image
출력: 픽셀별 class id segmentation mask
```

현재 baseline은 U-Net을 기본 모델로 사용합니다. 다만 학습 루프가 U-Net에 고정되지 않도록 `build_model(config)` 구조로 분리해두었기 때문에, 나중에 다른 segmentation 모델도 config만 바꿔서 실험할 수 있습니다.

## 1. 전체 그림

전체 파이프라인은 아래처럼 흘러갑니다.

```text
rosbag
  |
  | batch_extract.py
  v
raw_images + gt_images + csv manifest
  |
  | apply_mask.py
  v
raw_post_processed + gt_post_processed
  |
  | CarmakerSegmentationAdapter
  v
SegmentationSample(image_path, mask_path, camera)
  |
  | SegmentationDataset
  v
torch tensors
  image: [B, 3, H, W], float32, 0..1
  mask:  [B, H, W], int64 class id
  |
  | build_model(config)
  v
model logits: [B, C, H, W]
  |
  | loss + metrics
  v
checkpoint + TensorBoard logs
```

핵심 설계 원칙은 이것입니다.

```text
데이터 폴더 구조는 Adapter가 책임진다.
이미지/tensor 변환은 Dataset이 책임진다.
모델 선택은 Model Factory가 책임진다.
학습 루프는 최대한 공통으로 유지한다.
```

즉, 데이터셋이 바뀌어도 `train.py`를 크게 건드리지 않고, 모델이 바뀌어도 학습 루프를 다시 쓰지 않는 구조입니다.

## 2. 실행 순서

### 2.1 rosbag에서 raw/GT 이미지 추출

```bash
roslaunch carmaker_image batch_extract.launch
```

실행되는 ROS node:

```text
batch_extract
  -> scripts/batch_extract.py
     -> 내부에서 extract_bag_images.py 기능 사용
```

역할:

```text
rosbag 안의 image_raw / image_gt topic을 읽는다.
카메라별 raw image와 GT image를 PNG로 저장한다.
raw/GT pair 정보를 CSV로 저장한다.
```

주요 출력:

```text
src/carmaker_image/data/raw_images/
src/carmaker_image/data/gt_images/
src/carmaker_image/data/csv/manifest.csv
```

`manifest.csv`는 학습에서 매우 중요합니다. 어떤 raw 이미지와 어떤 GT 이미지가 한 쌍인지 기록하는 파일입니다.

### 2.2 GT 이미지를 학습용 class-id mask로 변환

```bash
roslaunch carmaker_image apply_mask.launch recursive:=true
```

실행되는 ROS node:

```text
apply_mask
  -> scripts/apply_mask.py
```

역할:

```text
GT 이미지를 읽는다.
차체 영역 mask를 적용한다.
lane / landmark / background를 class id로 변환한다.
raw image에도 동일한 후처리 mask를 적용한다.
```

주요 출력:

```text
src/carmaker_image/data/raw_post_processed/
src/carmaker_image/data/gt_post_processed/
```

학습에 실제로 쓰는 입력/정답은 보통 아래 두 경로입니다.

```text
input image: data/raw_post_processed/.../*_raw_*_post.png
GT mask:     data/gt_post_processed/.../*_GT_*_post.png
```

### 2.3 Dataset debug

```bash
roslaunch carmaker_image segmentation_debug_dataset.launch
```

실행되는 ROS node:

```text
segmentation_debug_dataset
  -> scripts/segmentation_debug_dataset.py
     -> segmentation/debug_dataset.py
```

역할:

```text
Adapter로 image/mask pair를 찾는다.
Dataset으로 실제 이미지를 읽는다.
mask class 분포를 출력한다.
raw image 위에 GT mask를 overlay해서 저장한다.
```

출력:

```text
src/carmaker_image/runs/debug_dataset/
```

이 단계는 학습 전에 꼭 확인하는 것이 좋습니다.

확인할 것:

```text
raw image와 GT mask가 같은 장면인가?
front/left/rear/right 카메라가 섞이지 않았는가?
mask 위치가 이미지와 맞는가?
mask 값이 0, 1, 2로 잘 들어있는가?
```

### 2.4 학습 실행

```bash
roslaunch carmaker_image segmentation_train.launch
```

실행되는 ROS node:

```text
segmentation_train
  -> scripts/segmentation_train.py
     -> segmentation/train.py
```

역할:

```text
config를 읽는다.
Adapter / Dataset / DataLoader를 만든다.
config에 맞는 모델을 만든다.
loss와 optimizer를 만든다.
epoch 단위로 학습한다.
validation metric을 계산한다.
TensorBoard log와 checkpoint를 저장한다.
```

출력:

```text
src/carmaker_image/runs/unet_carmaker/
  checkpoints/
    best.pt
    last.pt
  events.out.tfevents...
```

`best.pt`는 validation mIoU가 가장 좋았던 checkpoint입니다.  
`last.pt`는 마지막 epoch의 checkpoint입니다.

### 2.5 TensorBoard 확인

```bash
tensorboard --logdir src/carmaker_image/runs
```

TensorBoard에서 확인할 항목:

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

숫자만 보지 말고 overlay 이미지를 꼭 같이 봐야 합니다. Loss가 줄어도 실제 예측 mask가 이상하면 데이터 매칭이나 mask 처리에 문제가 있을 수 있습니다.

## 3. 현재 Class 정의

`apply_mask.py`가 만든 GT mask는 픽셀값 자체가 class id입니다.

| Id | Class | 설명 |
|----|-------|------|
| 0 | background | 배경, 차체 제거 영역 포함 |
| 1 | lane | 차선 |
| 2 | landmark | landmark 영역 |

모델 출력 channel 수 `C`는 class 개수와 같습니다.

```text
현재 C = 3
```

따라서 모델 출력 shape은:

```text
logits: [B, 3, H, W]
```

정답 mask shape은:

```text
mask: [B, H, W]
```

입니다.

## 4. 주요 Python 파일 역할

### 4.1 `adapters.py`

파일:

```text
src/carmaker_image/scripts/segmentation/adapters.py
```

역할:

```text
현재 데이터셋의 폴더 구조를 읽어서
학습 코드가 공통으로 이해할 수 있는 sample 목록으로 변환한다.
```

현재 핵심 클래스:

```python
CarmakerSegmentationAdapter
```

읽는 대상:

```text
data/csv/manifest.csv
data/raw_post_processed/
data/gt_post_processed/
```

반환하는 sample 형태:

```python
SegmentationSample(
    image_path=...,
    mask_path=...,
    camera="front",
)
```

왜 Adapter가 필요한가?

```text
CarMaker 데이터와 WoodScape 데이터는 폴더 구조가 다를 수 있다.
하지만 학습 코드는 같은 형태의 sample만 받으면 된다.
그래서 데이터셋별 차이는 Adapter 안에 숨긴다.
```

나중에 WoodScape를 추가한다면:

```text
WoodscapeSegmentationAdapter를 새로 만든다.
train.py는 최대한 그대로 둔다.
```

### 4.2 `dataset.py`

파일:

```text
src/carmaker_image/scripts/segmentation/dataset.py
```

역할:

```text
Adapter가 준 image_path / mask_path를 실제로 읽는다.
OpenCV image를 PyTorch tensor로 변환한다.
resize를 적용한다.
DataLoader가 사용할 수 있는 Dataset 객체를 제공한다.
```

입력:

```text
image png
mask png
```

출력:

```text
image tensor: [3, H, W], float32, 0..1
mask tensor:  [H, W], int64
```

중요한 점:

```text
image resize는 bilinear interpolation 사용
mask resize는 nearest interpolation 사용
```

mask는 class id이기 때문에 bilinear를 쓰면 안 됩니다. 예를 들어 원래 mask 값이 `0`, `1`, `2`뿐이어야 하는데 bilinear resize를 하면 중간값이 생길 수 있습니다.

### 4.3 `models.py`

파일:

```text
src/carmaker_image/scripts/segmentation/models.py
```

역할:

```text
segmentation model 정의
모델 registry 관리
config를 보고 모델 생성
```

현재 들어있는 모델:

| Config name | Class | 설명 |
|-------------|-------|------|
| `unet` | `UNet` | 기본 baseline 모델 |
| `tiny_fcn` | `TinyFCN` | 임시 모델, smoke test / 모델 교체 구조 확인용 |

중요: `TinyFCN`은 정식 실험용으로 넣은 모델이 아니라 **임시 예시용 모델**입니다.

`TinyFCN`을 넣은 이유:

```text
U-Net만 고정되어 있지 않고,
config의 model.name을 바꾸면 다른 모델도 실행되는지 확인하기 위한 간단한 예시입니다.
```

따라서 실제 비교 실험에서는 다음과 같은 모델을 추가하는 것이 더 의미 있습니다.

```text
UNet + ResNet encoder
DeepLabV3
DeepLabV3+
SegFormer
BiSeNet
Fast-SCNN
```

모델 선택은 `MODEL_REGISTRY`에서 합니다.

```python
MODEL_REGISTRY = {
    "unet": UNet,
    "tiny_fcn": TinyFCN,
}
```

학습 코드는 직접 `UNet()`을 만들지 않고:

```python
model = build_model(cfg, num_classes=adapter.num_classes)
```

를 호출합니다.

즉, 모델을 바꾸려면 config만 바꾸면 됩니다.

```yaml
model:
  name: unet
  in_channels: 3
  base_channels: 32
```

임시 모델로 바꾸려면:

```yaml
model:
  name: tiny_fcn
  in_channels: 3
  base_channels: 32
```

새 모델 추가 순서:

```text
1. models.py에 새 nn.Module class 작성
2. MODEL_REGISTRY에 등록
3. segmentation_unet.yaml의 model.name 변경
```

예시:

```python
MODEL_REGISTRY = {
    "unet": UNet,
    "tiny_fcn": TinyFCN,
    "deeplabv3": DeepLabV3Model,
}
```

```yaml
model:
  name: deeplabv3
```

### 4.4 `losses.py`

파일:

```text
src/carmaker_image/scripts/segmentation/losses.py
```

역할:

```text
loss 함수 정의
config의 loss 이름을 실제 PyTorch loss 객체로 변환
```

현재 지원:

```text
cross_entropy
dice
ce_dice
focal
```

처음 추천:

```yaml
loss: cross_entropy
```

class imbalance가 심하면:

```yaml
loss: cross_entropy
class_weights: [0.2, 2.0, 3.0]
```

대략적인 의미:

```text
background는 덜 중요하게
lane은 더 중요하게
landmark는 더 중요하게
```

loss 선택 추천 순서:

```text
1. cross_entropy
2. weighted cross_entropy
3. ce_dice
4. focal
```

처음부터 focal loss를 쓰면 디버깅이 어려울 수 있으므로, 먼저 cross entropy로 데이터와 모델 흐름이 정상인지 확인하는 것이 좋습니다.

### 4.5 `metrics.py`

파일:

```text
src/carmaker_image/scripts/segmentation/metrics.py
```

역할:

```text
validation 단계에서 segmentation 품질을 수치로 평가한다.
```

현재 metric:

```text
mIoU
class별 IoU
Dice
Pixel Accuracy
PSNR
```

가장 중요하게 볼 metric:

```text
mIoU
class별 IoU
```

예시:

```text
val/iou/class_0 -> background IoU
val/iou/class_1 -> lane IoU
val/iou/class_2 -> landmark IoU
```

해석 예시:

```text
background IoU만 높고 lane IoU가 낮다
-> 모델이 대부분 background로 예측하고 있을 가능성이 있음

lane IoU는 오르는데 landmark IoU가 낮다
-> landmark 데이터가 부족하거나 GT 변환이 불안정할 수 있음
```

PSNR은 segmentation의 핵심 평가지표는 아닙니다. mask 이미지 유사도를 참고용으로 보기 위해 넣어둔 debug 성격의 metric입니다.

### 4.6 `visualization.py`

파일:

```text
src/carmaker_image/scripts/segmentation/visualization.py
```

역할:

```text
class id mask를 색상 이미지로 변환한다.
raw image 위에 mask를 overlay한다.
TensorBoard와 dataset debug에서 시각화 이미지를 만든다.
```

예:

```text
GT overlay
Prediction overlay
```

segmentation에서는 이 시각화가 매우 중요합니다. 숫자 metric이 좋아 보여도 실제 overlay가 틀어져 있으면 데이터 pairing이나 후처리 문제가 있을 수 있습니다.

### 4.7 `debug_dataset.py`

파일:

```text
src/carmaker_image/scripts/segmentation/debug_dataset.py
```

역할:

```text
학습 전에 dataset이 정상인지 검사한다.
```

내부 흐름:

```text
CarmakerSegmentationAdapter 생성
SegmentationDataset 생성
앞에서부터 몇 개 sample 읽기
mask class histogram 출력
overlay image 저장
```

실행:

```bash
roslaunch carmaker_image segmentation_debug_dataset.launch
```

이 단계에서 문제가 보이면 학습으로 넘어가지 말고 먼저 데이터 파이프라인을 고치는 것이 좋습니다.

### 4.8 `train.py`

파일:

```text
src/carmaker_image/scripts/segmentation/train.py
```

역할:

```text
학습 전체를 제어하는 main training loop
```

내부 흐름:

```text
1. config 읽기
2. random seed 설정
3. device 선택
4. Adapter 생성
5. Dataset 생성
6. train/validation split
7. DataLoader 생성
8. build_model로 모델 생성
9. build_loss로 loss 생성
10. optimizer 생성
11. epoch 반복
12. train loss 계산
13. validation metric 계산
14. TensorBoard 기록
15. checkpoint 저장
```

학습 batch 하나의 흐름:

```text
image, mask batch
  |
  v
model(image)
  |
  v
logits [B, C, H, W]
  |
  v
loss(logits, mask)
  |
  v
backward
  |
  v
optimizer.step()
```

validation batch 하나의 흐름:

```text
image, mask batch
  |
  v
model(image)
  |
  v
argmax(logits)
  |
  v
pred mask [B, H, W]
  |
  v
GT mask와 비교
  |
  v
IoU / Dice / Accuracy / PSNR 계산
```

## 5. Config 파일

파일:

```text
src/carmaker_image/config/segmentation_unet.yaml
```

주요 설정:

```yaml
data_root: ../data
manifest: ../data/csv/manifest.csv
cameras: front,left,rear,right

image_size: [512, 512]
num_workers: 2
batch_size: 4
val_ratio: 0.2
seed: 42

epochs: 30
learning_rate: 0.001
weight_decay: 0.0001

model:
  name: unet
  in_channels: 3
  base_channels: 32

loss: cross_entropy
run_dir: ../runs/unet_carmaker
```

자주 바꾸는 값:

```text
batch_size
epochs
learning_rate
model.name
loss
class_weights
image_size
```

## 6. Overfit Smoke Test

처음에는 전체 데이터로 바로 학습하지 말고 작은 overfit test를 권장합니다.

```bash
roslaunch carmaker_image segmentation_train.launch overfit_count:=10 epochs:=50
```

의미:

```text
데이터 10장만 사용해서 모델이 거의 외울 수 있는지 확인한다.
```

정상적인 경우:

```text
train loss가 빠르게 내려간다.
GT overlay와 pred overlay가 점점 비슷해진다.
```

10장도 못 외우면 의심할 것:

```text
raw와 GT가 매칭되지 않음
mask class id가 잘못됨
mask resize 방식 문제
모델 output channel 수 문제
loss 입력 shape 문제
learning rate 문제
```

## 7. 확장 기준

앞으로 코드를 확장할 때는 아래 기준으로 나누면 됩니다.

| 바꾸고 싶은 것 | 수정할 위치 |
|----------------|-------------|
| 새 데이터셋 추가 | `adapters.py`에 Adapter 추가 |
| augmentation 추가 | `dataset.py` 또는 transform 모듈 추가 |
| 새 모델 추가 | `models.py`에 모델 class 추가 후 `MODEL_REGISTRY` 등록 |
| loss 추가 | `losses.py`에 loss 추가 후 `build_loss`에 등록 |
| metric 추가 | `metrics.py`에 metric 추가 |
| 학습 방식 변경 | `train.py` 수정 |
| 실행 옵션 변경 | launch 파일 또는 `segmentation_unet.yaml` 수정 |

가장 중요한 기준:

```text
데이터셋이 바뀌면 Adapter를 바꾼다.
모델이 바뀌면 models.py + config만 바꾼다.
loss가 바뀌면 losses.py + config만 바꾼다.
학습 루프는 공통으로 유지한다.
```

