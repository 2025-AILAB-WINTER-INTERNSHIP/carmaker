# carmaker_image_pipeline (ROS1)

CarMaker GT 이미지 후처리 파이프라인 패키지입니다.

현재 목표:

- GT 이미지 입력 처리
- 카메라별 차량 하단(car body) 영역 제외
- 최종 결과를 이미지 1장(`*_post.png`)으로 저장

## 1) 최종 폴더 구조

- `data/mask`
- `data/raw_images`
- `data/gt_images`
- `data/gt_post_processed`

각 폴더 역할:

- `data/raw_images`: rosbag에서 추출한 원본 이미지 (참고/보관용)
- `data/gt_images`: 상위 단계에서 생성된 GT 이미지 (본 파이프라인 입력)
- `data/mask`: 카메라별 Labelme JSON + 재사용 마스크 PNG
- `data/gt_post_processed`: 최종 후처리 결과 저장 위치

## 2) 입력 파일 규칙

- 파일명에 카메라 이름이 포함되어야 합니다: `front`, `left`, `rear`, `right`
- 예: `front_GT.png`, `left_GT.png`, ...
- `data/mask` 안에는 카메라별 JSON 파일이 있어야 합니다.
  - 기본 패턴: `{camera}_GT.json`
  - 예: `front_GT.json`, `left_GT.json`, `rear_GT.json`, `right_GT.json`

## 3) 전체 처리 흐름

`auto_mask_gt.py` 실행 시:

1. `data/gt_images`에서 GT 이미지들을 스캔합니다.
2. 파일명에서 카메라 이름을 판별합니다.
3. `data/mask/{camera}_mask.png` 마스크를 먼저 로드합니다.
4. 마스크가 없으면 `data/mask/{camera}_GT.json`의 `car_body` polygon으로 마스크를 1회 생성하고 저장합니다.
5. GT 이미지에서 차선/랜드마크 픽셀을 분류하되, car-mask 영역은 분류에서 제외합니다.
6. 클래스맵을 최종 인코딩해서 `data/gt_post_processed/*_post.png`로 저장합니다.

중요:

- 중간 마스킹 이미지는 저장하지 않습니다.
- 최종 결과 이미지 1장만 저장합니다. (고속 처리에 유리)

## 4) 클래스 정의 및 최종 저장값

내부 클래스 ID:

- `0`: background
- `1`: lane (black lane)
- `2`: yellow landmark

최종 `*_post.png` 픽셀값:

- `0` -> background
- `255` -> lane
- `127` -> yellow landmark

요약:

- 차선 = `255`
- 랜드마크 = `127`

## 5) 빌드 및 실행

빌드:

```bash
cd /workspace
cbp carmaker_image_pipeline
source /workspace/install/setup.bash
```

기본 실행:

```bash
rosrun carmaker_image_pipeline auto_mask_gt.py --suffix _post
```

launch 실행:

```bash
roslaunch carmaker_image_pipeline auto_mask_gt.launch
```

## 6) 주요 옵션

- `--input-dir`: GT 입력 폴더 (기본: `data/gt_images`)
- `--mask-dir`: JSON/마스크 폴더 (기본: `data/mask`)
- `--output-dir`: 결과 출력 폴더 (기본: `data/gt_post_processed`)
- `--suffix`: 출력 파일 suffix (기본: `_post`)
- `--refresh-masks`: JSON 기준으로 마스크 재생성
- `--recursive`: 하위 폴더 재귀 스캔
- `--lane-threshold`: 검정 차선 분류 임계값
- `--yellow-r-min`, `--yellow-g-min`, `--yellow-b-max`: 노란 랜드마크 분류 임계값

호환성:

- `--json-dir`는 기존 명령 호환용 별칭으로 유지되어 있으며, 설정 시 `--mask-dir` 대신 사용됩니다.

## 7) 보조 스크립트

- `scripts/make_mask.py`: JSON 1개로 mask PNG 1개 생성
- `scripts/apply_mask.py`: 단일 이미지에 단일 mask 적용 (디버그/검증용)

운영 파이프라인 진입점은 `scripts/auto_mask_gt.py`입니다.

## 8) 지금까지 작업 요약

- 패키지명 통합: `carmaker_image_pipeline`
- 데이터 구조 정리: `raw_images / gt_images / mask / gt_post_processed`
- `json` 폴더명 변경 -> `mask`
- 카메라별 마스크 1회 생성 후 재사용 로직 적용
- 중간 마스킹 이미지 저장 제거
- 최종 산출물을 단일 `*_post.png`로 단순화
- 최종 인코딩 고정: `background=0, lane=255, landmark=127`
