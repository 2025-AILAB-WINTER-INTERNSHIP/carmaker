# CarMaker Control Pipeline

이 문서는 `carmaker_control` 패키지를 실행하는 방법과, planning 결과가 control 입력으로 들어와 최종 CarMaker 제어 명령으로 변환되는 과정을 정리한다.

## 전체 파이프라인

현재 의도한 전체 흐름은 다음과 같다.

```text
segmentation
  -> localization
  -> planning
  -> control
  -> CarMaker
```

`carmaker_control`은 앞단 전체를 다시 보지 않는다. control이 직접 사용하는 입력은 두 개뿐이다.

```text
/planning/trajectory   carmaker_msgs/TrajectoryPath
/localization/odom     nav_msgs/Odometry
```

그리고 최종 출력은 하나다.

```text
/carmaker/control_signal   carmaker_msgs/Control_Signal
```

즉 control 기준에서 역할을 나누면 다음과 같다.

| 단계 | 담당 패키지 | control과의 관계 |
| --- | --- | --- |
| segmentation | `carmaker_image`, `segmentation` 계열 | control이 직접 보지 않음 |
| localization | `carmaker_localization` | `/localization/odom`으로 현재 pose, velocity 제공 |
| planning | `carmaker_planning` | `/planning/trajectory`로 추종할 경로 제공 |
| control | `carmaker_control` | PID 속도 제어, Stanley 조향 제어, 기어 전환 처리 |
| vehicle interface | CarMaker bridge | `/carmaker/control_signal`을 받아 차량에 적용 |

## 실행 순서

Ubuntu ROS PC에서 실행한다고 가정한다.

### 1. 빌드

```bash
cd ~/catkin_ws
catkin_make --pkg carmaker_msgs carmaker_localization carmaker_planning carmaker_control
source devel/setup.bash
```

워크스페이스 전체를 빌드하는 경우:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. CarMaker 및 센서/인지 노드 실행

CarMaker bridge가 최소한 다음 토픽을 publish해야 한다.

```text
/carmaker/dynamic_info   carmaker_msgs/DynamicsInfo
```

segmentation/localization이 사용하는 카메라, 동역학, map 입력도 이 시점에 준비되어야 한다.

### 3. Localization 실행

localization은 control에 필요한 현재 상태를 `/localization/odom`으로 내보낸다.

```bash
roslaunch carmaker_localization localization.launch
```

확인할 토픽:

```bash
rostopic echo /localization/odom
```

`/localization/odom`에는 다음 값이 포함된다.

| 필드 | 의미 |
| --- | --- |
| `pose.pose.position.x/y` | 현재 차량 위치 |
| `pose.pose.orientation` | 현재 차량 yaw |
| `twist.twist.linear.x` | 차량 진행 방향 기준 속도 |
| `twist.twist.angular.z` | yaw rate |

또한 localization은 planning에서 사용할 TF도 publish한다.

```text
Fr0 -> Fr1A_pred
```

### 4. Planning 실행

```bash
roslaunch carmaker_planning global_planner.launch
```

실제 localization 기반 파이프라인에서는 `src/carmaker_planning/config/global_planner_params.yaml`에서 다음 설정을 확인해야 한다.

```yaml
setting:
  use_gt_pose: false
  use_manual_pose:
    enabled: false
```

`use_manual_pose.enabled`가 `true`이면 planning은 localization TF가 아니라 `/planning/start` 수동 pose를 시작점으로 사용한다. RViz 테스트에는 유용하지만, 실제 파이프라인에서는 `false`가 자연스럽다.

목표점은 다음 토픽으로 들어간다.

```text
/planning/goal   geometry_msgs/PoseStamped
```

planning 성공 시 control 입력 경로가 publish된다.

```text
/planning/trajectory   carmaker_msgs/TrajectoryPath
```

확인:

```bash
rostopic echo /planning/trajectory
```

### 5. Control 실행

```bash
roslaunch carmaker_control control.launch
```

기본 설정 파일은 다음이다.

```text
src/carmaker_control/config/control_params.yaml
```

실행 후 확인할 출력:

```bash
rostopic echo /carmaker/control_signal
```

## Control 패키지 파일 역할

| 파일 | 역할 |
| --- | --- |
| `src/carmaker_control/src/control_node.cpp` | ROS node, trajectory 수신, odometry 수신, segment 분할, PID/Stanley 실행, control signal publish |
| `src/carmaker_control/src/pid.cpp` | 속도 PID 구현 |
| `src/carmaker_control/include/carmaker_control/pid.hpp` | PID 클래스 선언 |
| `src/carmaker_control/src/stanley.cpp` | Stanley 조향 계산 구현 |
| `src/carmaker_control/include/carmaker_control/stanley.hpp` | Stanley 클래스 선언 |
| `src/carmaker_control/config/control_params.yaml` | 토픽, PID gain, Stanley gain, 기어, 제한값 설정 |
| `src/carmaker_control/launch/control.launch` | control node 실행 및 파라미터 로드 |

`ControlNode`는 `control_node.cpp` 안에만 있다. 다른 `.cpp`에서 재사용하지 않는 실행 파일 내부 클래스이기 때문에 별도 헤더를 두지 않는다.

`PID`, `Stanley`는 `control_node.cpp`에서 사용하고 각각의 `.cpp`에서 구현되므로, 클래스 선언을 공유하기 위한 `.hpp` 헤더가 필요하다.

## Planning 결과가 Control로 이어지는 방식

planning 내부의 경로 표현은 `carmaker_planning::PathPoint`이다.

중요 필드는 다음이다.

| 필드 | 의미 |
| --- | --- |
| `x`, `y`, `theta` | 경로점 pose |
| `kappa` | 곡률 |
| `v` | 속도 크기 |
| `a` | 가속도 |
| `direction` | `1`: 전진, `-1`: 후진 |
| `t` | 시작 후 시간 |

Hybrid A*는 전진과 후진 후보를 모두 확장한다.

```cpp
const std::vector<int> directions = {1, -1};
```

후진에는 비용 penalty가 붙는다.

```yaml
global_planner:
  weights:
    reverse: 5.0
    change_dir: 15.0
```

planning 후처리는 direction이 같은 구간 단위로 수행된다.

```text
raw Hybrid A* path
  -> direction별 smoothing
  -> direction별 resampling
  -> direction별 velocity profiling
```

특히 velocity profiling은 전진/후진 전환 지점에서 속도를 0으로 만든다. 따라서 planning 결과는 control에게 “여기서 멈춘 뒤 기어를 바꿔야 한다”는 정보를 속도 프로파일로 전달한다.

마지막으로 planning은 `/planning/trajectory`를 publish할 때 direction을 별도 필드로 보내지 않고, `longitudinal_velocity` 부호에 실어 보낸다.

```cpp
tp.longitudinal_velocity = pt.v * pt.direction;
```

control은 이 값을 다음처럼 해석한다.

| `longitudinal_velocity` | control 해석 |
| --- | --- |
| `> 0` | 전진 |
| `< 0` | 후진 |
| `약 0` | 정지점 또는 기어 전환점 |

## Control 내부 처리 순서

control loop는 다음 순서로 동작한다.

```text
1. /planning/trajectory 수신
2. longitudinal_velocity 부호 기준으로 trajectory segment 분할
3. /localization/odom에서 현재 pose, speed 수신
4. 현재 active segment 안에서 nearest point 검색
5. lookahead point에서 target speed 선택
6. PID로 accel command 계산
7. Stanley로 steer command 계산
8. Control_Signal로 gas/brake/accel/steerangle/gear publish
9. segment 끝에 도착하면 정지
10. 속도가 충분히 낮아지면 다음 gear segment로 전환
```

### Segment 분할

control은 planning trajectory를 전체 경로로 한 번에 추종하지 않는다.

예를 들어 planning 결과가 다음과 같다면:

```text
전진 -> 전진 -> 정지 -> 후진 -> 후진 -> 정지 -> 전진
```

control은 다음 segment로 나눈다.

```text
Segment 0: forward
Segment 1: reverse
Segment 2: forward
```

각 segment는 독립적으로 추종된다. segment가 끝나면 control은 먼저 멈추고, 차량 속도가 `gear_switch_speed` 이하가 된 뒤 다음 segment로 넘어간다.

### 현재 위치 기준점

control은 `/localization/odom`을 현재 상태의 단일 출처로 사용한다.

```text
pose.x = odom.pose.pose.position.x
pose.y = odom.pose.pose.position.y
pose.yaw = odom.pose.pose.orientation
speed = odom.twist.twist.linear.x
```

GT pose, `/carmaker/dynamic_info`, TF lookup은 control에서 직접 사용하지 않는다. 이 덕분에 control은 localization 이후 단계로만 동작하며, 앞단 추정 방식이 바뀌어도 `/localization/odom` 인터페이스만 유지하면 된다.

### 속도 제어

속도 제어는 PID를 사용한다.

```text
target_speed = trajectory lookahead point의 속도 크기
current_speed = abs(/localization/odom.twist.twist.linear.x)
accel_command = PID(target_speed, current_speed)
```

PID 출력은 바로 `gas`가 아니라 가속도 명령이다. 이후 `publishControl()`에서 다음으로 변환된다.

```text
accel_command > 0 -> gas
accel_command < 0 -> brake
```

후진은 gear가 reverse이므로 속도 목표는 항상 절댓값으로 다루고, 최종 `accel` 필드에 direction 부호를 반영한다.

### 조향 제어

조향은 Stanley controller를 사용한다.

입력:

```text
cross track error
heading error
current speed
```

출력:

```text
tire steer angle
```

후진에서는 같은 조향각이 차량 yaw를 반대 방향으로 변화시키므로 Stanley 결과 부호를 뒤집는다.

```text
forward: steer = stanley(...)
reverse: steer = -reverse_steering_scale * stanley(...)
```

이후 `steering_ratio`와 `max_steer_command`로 CarMaker 입력 범위에 맞춘다.

## 주요 토픽

| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
| --- | --- | --- | --- | --- |
| `/localization/odom` | `nav_msgs/Odometry` | `carmaker_localization` | `carmaker_control` | 현재 pose와 속도 |
| `/planning/goal` | `geometry_msgs/PoseStamped` | RViz 또는 goal publisher | `carmaker_planning` | 목표 pose |
| `/planning/trajectory` | `carmaker_msgs/TrajectoryPath` | `carmaker_planning` | `carmaker_control` | 추종 경로 |
| `/carmaker/control_signal` | `carmaker_msgs/Control_Signal` | `carmaker_control` | CarMaker bridge | 최종 제어 명령 |

## 주요 파라미터

### 토픽 설정

```yaml
topics:
  subscribe:
    trajectory: "/planning/trajectory"
    odom: "/localization/odom"
  publish:
    control: "/carmaker/control_signal"
```

### 안전 timeout

```yaml
control:
  odom_timeout: 0.5
  trajectory_timeout: 30.0
```

`odom_timeout` 안에 localization odometry가 갱신되지 않으면 정지 명령을 낸다.

`trajectory_timeout` 안에 새 planning trajectory가 없으면 경로가 오래되었다고 보고 정지한다.

### 전진/후진 segment 판단

```yaml
control:
  direction_velocity_epsilon: 0.02
```

이 값보다 작은 속도는 정지점으로 본다. 너무 작게 두면 노이즈로 segment가 잘게 쪼개질 수 있고, 너무 크게 두면 저속 후진 구간을 정지점으로 잘못 볼 수 있다.

### 기어 전환 속도

```yaml
control:
  gear_switch_speed: 0.08
```

segment 끝에 도달해도 현재 속도가 이 값보다 크면 다음 segment로 넘어가지 않는다. 먼저 brake를 걸고 충분히 느려진 뒤 기어를 바꾼다.

### PID

```yaml
pid:
  speed:
    kp: 1.0
    ki: 0.0
    kd: 0.05
```

튜닝 방향:

| 증상 | 조정 |
| --- | --- |
| 목표 속도까지 너무 늦게 올라감 | `kp` 증가 |
| 속도 오차가 오래 남음 | `ki` 소량 증가 |
| 가감속이 출렁임 | `kd` 증가 또는 `kp` 감소 |
| 출발이 과격함 | `max_accel`, `max_gas`, `kp` 감소 |

### Stanley

```yaml
stanley:
  k: 1.0
  k_soft: 0.3
  cte_gain: 1.0
  heading_gain: 1.0
  max_steer_angle_deg: 28.4
  reverse_steering_scale: 1.0
```

`max_steer_angle_deg`는 Ioniq 5 설정값인 `wheelbase=2.97m`,
`min_turning_radius=5.5m`에서 계산한 타이어 조향각 한계다.

```text
atan(2.97 / 5.5) = 28.4 deg
```

튜닝 방향:

| 증상 | 조정 |
| --- | --- |
| 경로 옆으로 밀려도 복귀가 약함 | `k` 또는 `cte_gain` 증가 |
| 헤딩 정렬이 느림 | `heading_gain` 증가 |
| 저속에서 조향이 튐 | `k_soft` 증가 |
| 후진 조향이 과함 | `reverse_steering_scale` 감소 |
| 후진 조향이 부족함 | `reverse_steering_scale` 증가 |

### CarMaker 조향 입력 범위

```yaml
vehicle:
  wheelbase: 2.97
  min_turning_radius: 5.5
  steering_ratio: 1.0
  max_steer_command: 0.4951
```

Stanley는 타이어 조향각을 계산한다. CarMaker가 steering wheel angle을 기대한다면 `steering_ratio`를 키우거나 `max_steer_command`를 프로젝트 입력 스펙에 맞춰 조정해야 한다.

## 실행 확인 체크리스트

### localization 확인

```bash
rostopic hz /localization/odom
rostopic echo /localization/odom
```

확인할 것:

- pose가 NaN이 아닌지
- 차량 이동 시 position이 변하는지
- `twist.twist.linear.x`가 실제 속도와 비슷한지

### planning 확인

```bash
rostopic echo /planning/trajectory
```

확인할 것:

- `points`가 비어 있지 않은지
- `longitudinal_velocity`가 전진 구간에서 양수인지
- 후진 구간이 있으면 음수인지
- 전진/후진 전환 근처에서 속도가 0에 가까워지는지

### control 확인

```bash
rostopic echo /carmaker/control_signal
```

확인할 것:

- 경로가 없을 때 `gear = neutral_gear`, `brake > 0`인지
- 전진 segment에서 `gear = drive_gear`인지
- 후진 segment에서 `gear = reverse_gear`인지
- `steerangle`이 너무 큰 값으로 포화되지 않는지
- `gas`와 `brake`가 동시에 과하게 나오지 않는지

## 흔한 문제

### planning이 localization을 안 쓰는 경우

`global_planner_params.yaml`에서 manual pose가 켜져 있으면 planning은 `/planning/start`를 시작점으로 쓴다.

실제 파이프라인에서는 다음처럼 둔다.

```yaml
setting:
  use_gt_pose: false
  use_manual_pose:
    enabled: false
```

### control이 계속 정지 명령만 내는 경우

다음을 확인한다.

```bash
rostopic hz /localization/odom
rostopic hz /planning/trajectory
```

가능한 원인:

- localization odometry timeout
- trajectory timeout
- trajectory points가 비어 있음
- 현재 차량이 segment 끝으로 판정되어 정지 중

### 후진 전환이 안 되는 경우

다음을 확인한다.

- `/planning/trajectory`의 후진 구간 velocity가 음수인지
- `gear_switch_speed`보다 차량 속도가 충분히 내려가는지
- `reverse_gear` 값이 CarMaker 프로젝트의 후진 gear 값과 맞는지

### 조향 방향이 반대로 보이는 경우

가능한 원인:

- CarMaker의 `steerangle` 부호 규약이 현재 teleop/control 가정과 다름
- localization yaw frame과 trajectory yaw frame이 맞지 않음
- 후진 조향 부호 반전이 차량 모델과 맞지 않음

먼저 아주 낮은 속도로 전진 직선 경로를 테스트하고, 그다음 좌/우 곡선, 마지막으로 후진 곡선을 확인한다.
