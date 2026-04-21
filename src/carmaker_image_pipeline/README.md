# carmaker_image_pipeline (ROS1)

CarMaker GT 이미지 후처리 파이프라인 패키지입니다.

## 0) 작업 순서 (가장 중요)

1. rosbag에서 raw/GT 이미지를 먼저 추출
2. 추출된 GT 이미지를 후처리해서 최종 `*_post.png` 생성

즉, **항상 `extract_bag_images.py` -> `auto_mask_gt.py` 순서**로 진행합니다.

## 1) 폴더 구조

- `data/mask`
- `data/raw_images`
- `data/gt_images`
- `data/gt_post_processed`

각 폴더 역할:

- `data/raw_images`: rosbag에서 추출한 raw 이미지
- `data/gt_images`: rosbag에서 추출한 GT 이미지 (후처리 입력)
- `data/mask`: 카메라별 Labelme JSON + 재사용 마스크 PNG
- `data/gt_post_processed`: 최종 후처리 결과

## 2) 1단계: rosbag -> raw/GT 이미지 추출 (먼저 수행)

`CarMaker_Rosbridge` 기준으로 보통 `image_raw`, `image_gt` 토픽을 사용합니다.

기본 실행:

```bash
rosrun carmaker_image_pipeline extract_bag_images.py \
  --bag /path/to/recorded.bag \
  --cameras front,left,rear,right
```

launch 실행:

```bash
roslaunch carmaker_image_pipeline extract_bag_images.launch \
  bag:=/path/to/recorded.bag
```

주요 옵션:

- `--raw-out-dir`: raw 이미지 저장 폴더 (기본: `data/raw_images`)
- `--gt-out-dir`: GT 이미지 저장 폴더 (기본: `data/gt_images`)
- `--gt-topic-regex`, `--raw-topic-regex`: 토픽 분류 정규식
- `--include-unknown-camera`: front/left/rear/right 키워드 없는 토픽도 저장
- `--max-frames-per-topic`: 토픽별 최대 저장 프레임
- `--start-offset-sec`, `--duration-sec`: bag 구간 추출
- `--overwrite`: 기존 파일 덮어쓰기

## 3) 2단계: GT 후처리 (`auto_mask_gt.py`)

입력 규칙:

- 파일명에 카메라 이름 포함: `front`, `left`, `rear`, `right`
- 예: `front_GT_...png`
- `data/mask`에 카메라별 JSON 존재
  - 기본 패턴: `{camera}_GT.json`

처리 흐름:

1. `data/gt_images` 스캔
2. 카메라 이름 판별
3. `data/mask/{camera}_mask.png` 로드
4. 없으면 `{camera}_GT.json`에서 `car_body` polygon으로 1회 생성 후 저장
5. GT 분류 시 car-mask 영역 제외
6. 최종 결과를 `data/gt_post_processed/*_post.png`로 저장

중요:

- 중간 마스킹 이미지 저장 없음
- 최종 결과 1장(`*_post.png`)만 저장

기본 실행:

```bash
rosrun carmaker_image_pipeline auto_mask_gt.py --suffix _post
```

launch 실행:

```bash
roslaunch carmaker_image_pipeline auto_mask_gt.launch
```

주요 옵션:

- `--input-dir`: GT 입력 폴더 (기본: `data/gt_images`)
- `--mask-dir`: JSON/마스크 폴더 (기본: `data/mask`)
- `--output-dir`: 결과 폴더 (기본: `data/gt_post_processed`)
- `--suffix`: 출력 suffix (기본: `_post`)
- `--refresh-masks`: JSON 기준 마스크 재생성
- `--recursive`: 하위 폴더 재귀 스캔
- `--lane-threshold`: 차선 분류 임계값
- `--yellow-r-min`, `--yellow-g-min`, `--yellow-b-max`: 랜드마크 분류 임계값

호환성:

- `--json-dir`는 기존 명령 호환용 별칭으로 유지됨

## 4) 클래스 정의 / 최종 저장값

내부 클래스 ID:

- `0`: background
- `1`: lane
- `2`: yellow landmark

최종 `*_post.png` 값:

- `0` -> background
- `255` -> lane
- `127` -> yellow landmark

요약:

- 차선 = `255`
- 랜드마크 = `127`

## 5) 빌드

```bash
cd /workspace
cbp carmaker_image_pipeline
source /workspace/install/setup.bash
```
