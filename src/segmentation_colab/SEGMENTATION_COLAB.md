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

필수:

```text
data/raw_images/
data/gt_post_processed/
```

권장:

```text
data/csv/manifest.csv
```

`manifest.csv`까지 올리는 편이 image/mask pair 매칭이 가장 안전하다.

## 포함된 파일

```text
src/segmentation_colab/
  README.md
  segmentation_colab.ipynb
  segmentation_unet_colab.yaml
```

`segmentation_unet_colab.yaml`은 Colab용 설정 템플릿이다. 노트북은 실행 중 Drive 경로에 맞는 config를 자동 생성해서 사용한다.

## 자주 바꾸는 값

노트북의 설정 셀에서 보통 아래 값만 바꾸면 된다.

```python
DRIVE_PROJECT_DIR = "/content/drive/MyDrive/carmaker_segmentation"
IMAGE_SIZE = [960, 540]
EPOCHS = 16
BATCH_SIZE = 1
BASE_CHANNELS = 32
```

무료 Colab GPU에서 OOM이 나면 `IMAGE_SIZE`를 먼저 줄이고, 그래도 부족하면 `BASE_CHANNELS`를 `16`으로 낮춘다.
