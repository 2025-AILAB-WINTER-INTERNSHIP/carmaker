# carmaker svm 기반 자동 주차 및 정렬

## 🚀 빠른 시작 가이드 (Quick Start)

### 📑 시스템 아키텍처 요구사항

| 구성 요소 | 역할 (Role) | 아키텍처적 필요성 |
| :--- | :---: | :--- |
| **Docker Engine** | **Runtime & Isolation** | 모든 개발 도구와 의존성을 격리된 컨테이너 내부에 가둡니다. |
| **GNU Make** | **Command Center** | 복잡한 도커 명령을 단일 워크플로우(`make ros` 등)로 추상화하고 자동화합니다. |
| **NVIDIA Toolkit** | **HW Acceleration** | 호스트의 GPU 자원을 컨테이너 내부로 손실 없이 패스스루(Pass-through)합니다. |


### 깃 클론

```bash
git clone 
cd 
```

### 환경 변수 초기화 및 설정

`make setup`을 통해 프로젝트 전용격리 환경을 구성하기 위한 `.env` 파일을 생성합니다.

```bash
make setup
nano .env
```

### 개발 환경 시작 및 상태 확인

```bash
make status         # 현재 프로젝트 설정 및 GPU/아키텍처/툴킷 자동 감지
make build-ros      # ROS 이미지 빌드 (Multi-Arch 자동 대응)
make ros            # ROS 컨테이너 시작
```

### 개발 환경 접속

```bash
make ros-shell    # shell로 컨테이너 접속
make ros-term     # 테미네이터로 컨테이너 접속
```

### 개발 환경 설정

```bash
# 1. 원스텝 통합 초기화
mksync --share  # 시스템 패키지 공유
```

### 실행

```bash
s

roslaunch carmaker_bringup bringup.launch checkpoint_path:=/workspace/src/segmentation_ros/data/models/best.ckpt

rostopic pub -1 /planning/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "Fr0"}, pose: {position: {x: 0, y: -4.33, z: 0}, orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}}}'
```
