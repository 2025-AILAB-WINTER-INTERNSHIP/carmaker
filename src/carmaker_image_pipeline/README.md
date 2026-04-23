# carmaker_image_pipeline (ROS1)

CarMaker GT 이미지 후처리 파이프라인 패키지입니다.

## 0) 작업 순서 (가장 중요)

1. rosbag에서 raw/GT 이미지를 먼저 추출
2. 추출된 GT 이미지를 후처리해서 최종 `*_post.png` 생성

즉, **항상 `extract_bag_images.py` -> `apply_mask.py` 순서**로 진행합니다.

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

직접 Python 실행:

```bash
python3 scripts/extract_bag_images.py \
  --bag /path/to/recorded.bag \
  --cameras front,left,rear,right
```

주의:

- `extract_bag_images.py`는 `rosbag`, `rospy`, `cv_bridge` 의존성이 있으므로 ROS 환경에서 실행해야 합니다.

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
- `--csv-dir`: CSV 저장 폴더 (스크립트 기본: `data/csv`, launch 기본: `data`)
- `--csv-prefix`: CSV 파일명 prefix (빈 문자열이면 bag 파일명 stem 사용)

CSV 출력 파일:

- `{prefix}_images.csv`: 전체(raw+GT)
- `{prefix}_raw_images.csv`: raw만
- `{prefix}_gt_images.csv`: GT만
- `{prefix}_raw_gt_pairs.csv`: 카메라별 저장 순서 기준 pair 목록

## 3) 2단계: GT 후처리 (`apply_mask.py`)

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

- 후처리 중간 결과 이미지는 저장하지 않음
- 단, `data/mask/{camera}_mask.png`가 없으면 JSON에서 생성하여 저장(재사용)함
- 최종 결과 1장(`*_post.png`)만 저장

기본 실행:

```bash
rosrun carmaker_image_pipeline apply_mask.py --suffix _post
```

직접 Python 실행:

```bash
python3 scripts/apply_mask.py --suffix _post
```

launch 실행:

```bash
roslaunch carmaker_image_pipeline apply_mask.launch
```

주요 옵션:

- `--input-dir`: GT 입력 폴더 (기본: `data/gt_images`)
- `--mask-dir`: JSON/마스크 폴더 (기본: `data/mask`)
- `--output-dir`: 결과 폴더 (기본: `data/gt_post_processed`)
- `--suffix`: 출력 suffix (기본: `_post`)
- `--refresh-masks`: JSON 기준 마스크 재생성
- `--recursive`: 하위 폴더 재귀 스캔
- `--lane-threshold`: 차선 분류 임계값
- `--landmark-gray-min`, `--landmark-gray-max`: 랜드마크 grayscale 분류 임계값

호환성:

- `--json-dir`는 기존 명령 호환용 별칭으로 유지됨

## 4) 클래스 정의 / 최종 저장값

내부 클래스 ID:

- `2`: lane
- `1`: yellow landmark
- `0`: background

최종 `*_post.png` 값:

- `255` -> lane
- `127` -> yellow landmark
- `0` -> background


요약:

- 차선 = `255`
- 랜드마크 = `127`

## 5) 빌드

```bash
cd /workspace
cbp carmaker_image_pipeline
source /workspace/install/setup.bash
```
