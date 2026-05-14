# Google Colab Segmentation Training

이 폴더는 CarMaker segmentation 학습을 Google Colab에서 돌리기 위한 묶음이다.

## Colab에서 열 파일

```text
src/segmentation_colab/segmentation_colab.ipynb
```

노트북은 GitHub에서 repo 전체를 clone/update한 뒤 `src/segmentation/train.py`를 실행한다.

## Drive에 올릴 데이터

Google Drive에는 학습 데이터만 올리면 된다.

```text
MyDrive/carmaker_segmentation/
  data/
    raw_images/
    gt_post_processed/
    csv/
      manifest.csv
  runs/
```

`manifest.csv`까지 올리는 편이 image/mask pair 매칭이 가장 안전하다.

## 포함된 파일

```text
src/segmentation_colab/
  SEGMENTATION_COLAB.md
  segmentation_colab.ipynb
  segmentation_unet_colab.yaml
```

`segmentation_unet_colab.yaml`은 Colab용 설정 템플릿이다. 노트북은 실행 중 Drive 경로에 맞는 config를 자동 생성해서 사용한다.

## 현재 Colab 학습 설정

노트북 자동 생성 config에도 현재 로컬 학습 설정을 반영했다.

```yaml
loss: focal
class_weights: [5.0, 15.0, 10.0]
dice_exclude_classes: [0]
debug_image_count: 8
run_dir: /content/drive/MyDrive/carmaker_segmentation/runs
```

`dice_exclude_classes: [0]`은 Dice 계열 loss를 쓸 때 background를 Dice 평균에서 제외한다. 현재 `loss: focal`만 쓰면 Dice 항은 없지만, `focal_dice`나 `ce_dice`로 바꿔도 같은 config를 그대로 사용할 수 있다.

`run_dir`는 base directory로 사용된다. `--run-dir`를 직접 지정하지 않으면 `train.py`가 그 아래에 `{loss}_ep{epochs}_{timestamp}` 형식의 폴더를 자동 생성한다.

```text
MyDrive/carmaker_segmentation/runs/focal_ep16_20260514_175830/
```

TensorBoard는 `runs`를 logdir로 열기 때문에 Colab에서도 로컬과 같은 run 이름이 보인다.

TensorBoard debug image는 split별 하나의 grid로 기록된다.

```text
debug/train/gt_grid
debug/train/pred_grid
debug/val/gt_grid
debug/val/pred_grid
debug/test/gt_grid
debug/test/pred_grid
```

grid 내부는 camera별 column으로 묶인다. `debug_image_count: 8`이고 `front/left/rear/right`가 모두 있으면 각 grid에 최대 8행 x 4열, 즉 32장이 들어간다.

## 자주 바꾸는 값

노트북의 설정 셀에서 보통 아래 값만 바꾸면 된다.

```python
DRIVE_PROJECT_DIR = "/content/drive/MyDrive/carmaker_segmentation"
IMAGE_SIZE = [1920, 1080]
EPOCHS = 16
BATCH_SIZE = 1
NUM_WORKERS = 4
BASE_CHANNELS = 32
LOSS = "focal"
```

Colab 기본값은 로컬 학습 config와 맞춘다. 무료 Colab GPU에서 OOM이 나면 `IMAGE_SIZE`를 `[960, 540]`으로 줄이고, 그래도 부족하면 `BASE_CHANNELS`를 `16`으로 낮춘다.

학습 셀은 `python -u`와 `TQDM_POSITION=-1` 환경으로 실행한다. Colab 출력에서 tqdm이 한 줄 갱신 형태로 보이지 않을 때도 epoch/step, loss, ETA가 로그에 남도록 하기 위한 설정이다.
