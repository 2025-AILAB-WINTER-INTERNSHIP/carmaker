# carmaker_image (ROS1)

CarMaker GT 이미지 파이프라인 패키지입니다.
rosbag에서 raw/GT 이미지를 추출하고, 차체 마스킹과 클래스 분류를 거쳐 학습용 GT 데이터를 생성합니다.

---

## 파이프라인 개요

```
data/bags/
└── 2026-04-01/
    └── scenario_001.bag
          │
          │  [1단계] bag → 이미지 추출
          │  batch_extract.launch  (또는 extract_bag_images.launch)
          ▼
data/raw_images/scenario_001/      data/gt_images/scenario_001/
  front_raw_1.png  ...               front_GT_1.png  ...
data/csv/
  scenario_001_images.csv
  batch_manifest.csv
          │
          │  [2단계] GT 이미지 후처리
          │  apply_mask.launch  (--recursive)
          ▼
data/gt_post_processed/
  front_GT_1_post.png  ...    ← 픽셀값: 0=background, 1=lane, 2=landmark
```

> **`apply_mask.py --generate-only`** 는 파이프라인 시작 전 사전 준비 단계로,
> `data/mask/{camera}_mask.png` 를 Labelme JSON으로부터 생성합니다.

---

## 폴더 구조

```
carmaker_image/
├── config/
│   └── camera_config.yaml         # 카메라 내부/외부 파라미터, BEV 범위
├── data/
│   ├── bags/                      # 입력 rosbag — 날짜별 서브디렉토리로 관리
│   │   ├── 2026-04-01/
│   │   │   ├── scenario_001.bag
│   │   │   └── scenario_002.bag
│   │   └── 2026-04-15/
│   │       └── highway_test.bag
│   ├── raw_images/                # [1단계 출력] rosbag 추출 raw 이미지
│   ├── gt_images/                 # [1단계 출력] rosbag 추출 GT 이미지
│   ├── gt_post_processed/         # [2단계 출력] 최종 후처리 결과 (*_post.png)
│   ├── mask/                      # 카메라별 Labelme JSON + 마스크 PNG
│   └── csv/                       # 추출 CSV 매니페스트
├── launch/
│   ├── extract_bag_images.launch  # 단일 bag 추출
│   ├── batch_extract.launch       # 다중 bag 배치 추출
│   └── apply_mask.launch          # GT 후처리
└── scripts/
    ├── extract_bag_images.py      # [1단계] 단일 bag → 이미지 추출
    ├── batch_extract.py           # [1단계] 다중 bag 배치 추출
    └── apply_mask.py              # [2단계] GT 후처리 + 마스크 생성
```

---

## 1단계: rosbag → 이미지 추출

`CarMaker_Rosbridge` 기준으로 `image_raw` (raw), `image_gt` (GT) 토픽을 사용합니다.

### 단일 Bag 추출 (`extract_bag_images.py`)

```bash
# rosrun
rosrun carmaker_image extract_bag_images.py \
  --bag /path/to/recorded.bag \
  --cameras front,left,rear,right

# Python 직접 실행
python3 scripts/extract_bag_images.py \
  --bag /path/to/recorded.bag

# roslaunch
roslaunch carmaker_image extract_bag_images.launch \
  bag:=/path/to/recorded.bag
```

주요 옵션:

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--bag` | **(필수)** | 입력 bag 경로 |
| `--raw-out-dir` | `data/raw_images` | raw 이미지 저장 디렉토리 |
| `--gt-out-dir` | `data/gt_images` | GT 이미지 저장 디렉토리 |
| `--csv-dir` | `data/csv` | CSV 매니페스트 저장 디렉토리 |
| `--cameras` | `front,left,rear,right` | 카메라 키워드 (쉼표 구분) |
| `--gt-topic-regex` | `(image_gt\|...)` | GT 토픽 분류 정규식 |
| `--raw-topic-regex` | `(image_raw\|/raw/)` | raw 토픽 분류 정규식 |
| `--max-frames-per-topic` | `0` (무제한) | 토픽별 최대 추출 프레임 수 |
| `--start-offset-sec` | `0.0` | bag 시작 기준 오프셋 (초) |
| `--duration-sec` | `0.0` (끝까지) | 추출 구간 길이 (초) |
| `--overwrite` | `false` | 기존 파일 덮어쓰기 |
| `--include-unknown-camera` | `false` | 알 수 없는 카메라 토픽도 저장 |
| `--csv-prefix` | bag 파일명 | CSV 파일명 prefix |

**CSV 출력 형식** — `{prefix}_images.csv`:

| 컬럼 | 설명 |
|------|------|
| `timestamp` | 메시지 헤더 타임스탬프 (초, float) |
| `kind` | 카메라 이름 (예: `front`) |
| `raw` | 추출된 raw 이미지 절대 경로 |
| `gt` | 추출된 GT 이미지 절대 경로 |
| `gt_post` | 후처리 결과 예상 경로 (`apply_mask.py` 실행 후 생성됨) |

### 배치 추출 (`batch_extract.py`)

여러 bag 파일을 한 번에 처리합니다.
각 bag의 출력은 **bag stem 이름의 서브디렉토리**에 격리되어 파일명 충돌을 방지합니다.

```bash
# roslaunch (기본: data/bags 재귀 탐색)
roslaunch carmaker_image batch_extract.launch

# 커스텀 디렉토리 지정
roslaunch carmaker_image batch_extract.launch \
  bag_dirs:=/path/to/bags

# CLI — 디렉토리 기반
rosrun carmaker_image batch_extract.py \
  --bag-dirs data/bags \
  --cameras front,left,rear,right

# CLI — 파일 직접 지정
python3 scripts/batch_extract.py \
  --bag-files a.bag b.bag c.bag

# dry-run: 추출 없이 대상 목록만 확인
python3 scripts/batch_extract.py --bag-dirs data/bags --dry-run
```

주요 옵션:

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--bag-dirs` | — | bag 검색 디렉토리 (`--bag-files`와 상호 배타) |
| `--bag-files` | — | 명시적 bag 파일 경로 목록 |
| `--no-recursive` | `false` | 하위 디렉토리 탐색 비활성화 |
| `--pattern` | `*.bag` | 파일 검색 glob 패턴 |
| `--dry-run` | `false` | 추출 없이 대상 목록만 출력 |
| `--force` | `false` | 이미 처리된 bag도 강제 재추출 |
| 나머지 | — | `extract_bag_images.py`와 동일 (모두 passthrough) |

배치 추출 후 출력 구조:

```
data/
├── raw_images/
│   ├── scenario_001/       ← bag stem 이름으로 자동 격리
│   │   ├── front/
│   │   │   └── front_raw_1.png
│   │   └── left/
│   │       └── left_raw_1.png
│   └── scenario_002/
│       └── ...
├── gt_images/
│   ├── scenario_001/
│   │   └── front/
│   │       └── front_GT_1.png
│   └── ...
└── csv/
    ├── scenario_001_images.csv
    ├── scenario_002_images.csv
    └── batch_manifest.csv      ← 전체 bag 통합 CSV
```

---

## 2단계: GT 후처리 (`apply_mask.py`)

GT 이미지에 차체 마스크를 적용하여 픽셀을 lane / landmark / background로 분류하고
`*_post.png`로 저장합니다.

`data/mask/{camera}_mask.png`가 없으면 JSON에서 자동 생성합니다.

```bash
# 기본 실행
roslaunch carmaker_image apply_mask.launch

# 배치 추출 이후: 서브디렉토리 재귀 탐색 (기본값)
roslaunch carmaker_image apply_mask.launch recursive:=true

# 마스크 강제 재생성
roslaunch carmaker_image apply_mask.launch refresh_masks:=true

# CLI
rosrun carmaker_image apply_mask.py --suffix _post --recursive
```

주요 옵션:

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--input-dir` | `data/gt_images` | GT 입력 디렉토리 |
| `--mask-dir` | `data/mask` | JSON/마스크 파일 디렉토리 |
| `--output-dir` | `data/gt_post_processed` | 결과 저장 디렉토리 |
| `--suffix` | `_post` | 출력 파일명 접미사 |
| `--cameras` | `front,left,rear,right` | 카메라 이름 목록 |
| `--recursive` | `true` | 하위 디렉토리 재귀 탐색 (배치 추출 후 기본 동작) |
| `--refresh-masks` | `false` | 기존 마스크 무시하고 JSON에서 강제 재생성 |
| `--lane-threshold` | `40` | 차선(검정) 감지 임계값 (0~255) |
| `--landmark-gray-min` | `160` | 랜드마크 grayscale 범위 최솟값 |
| `--landmark-gray-max` | `165` | 랜드마크 grayscale 범위 최댓값 |

### 클래스 정의

| 클래스 | 픽셀값 | 설명 |
|--------|--------|------|
| background | `0` | 배경 및 차체 영역 |
| lane | `1` | 검은 차선 |
| landmark | `2` | 노란 랜드마크 |

`*_post.png` 픽셀값이 위 클래스 ID와 1:1 대응합니다.

### 마스크 생성 (`--generate-only`)

GT 이미지 처리 없이 `data/mask/{camera}_mask.png` 생성만 수행합니다.
파이프라인 시작 전 마스크가 없을 때 먼저 실행합니다.

```bash
# 전체 카메라 마스크 일괄 생성
python3 scripts/apply_mask.py --generate-only \
  --cameras front,left,rear,right

# 단일 JSON → 마스크 (경로 직접 지정)
python3 scripts/apply_mask.py --generate-only \
  --json data/mask/front_GT.json \
  --output data/mask/front_mask.png

# 기존 마스크 강제 재생성
python3 scripts/apply_mask.py --generate-only --refresh-masks
```

> `--generate-only` 는 launch 파일을 지원하지 않습니다. CLI로만 실행합니다.

---

## 빌드

```bash
cd /workspace
cbp carmaker_image
source /workspace/devel/setup.bash
```
