# CarMaker Control Pipeline

이 문서는 `carmaker_control` 패키지의 역할, 실행 방법, planning 결과가 control 입력으로 연결되는 방식, 그리고 controller 내부 코드가 어떤 순서로 동작하는지 정리한다.

## 문서 범위와 가정

- ROS1 catkin workspace에서 실행한다고 가정한다.
- CarMaker bridge는 `/carmaker/dynamic_info`를 publish하고, `/carmaker/control_signal`을 받아 차량에 적용한다고 가정한다.
- 실제 localization 기반 파이프라인에서는 control 현재 상태 입력으로 `/localization/odom`을 쓰는 구성을 기준으로 설명한다.
- localization 없이 planning/control 성능만 확인할 때는 control 현재 상태 입력을 `/carmaker/dynamic_info` GT로 바꿀 수 있다.
- RViz 단독 테스트처럼 `/planning/start` 수동 pose를 쓰는 경우는 별도 테스트 모드로 본다.
- 이 문서는 `carmaker_control` 중심 설명이다. segmentation, localization, planning 내부 알고리즘은 control과 연결되는 지점만 다룬다.

## 전체 파이프라인

전체 주행 흐름은 다음과 같다.

```text
segmentation
  -> localization
  -> planning
  -> control
  -> CarMaker
```

topic 기준으로 보면 다음 구조다.

```text
/carmaker/dynamic_info + segmentation 결과
        -> carmaker_localization
        -> /localization/odom

/planning/goal + 현재 시작 pose
        -> carmaker_planning
        -> /planning/trajectory

/planning/trajectory + /localization/odom 또는 /carmaker/dynamic_info
        -> carmaker_control
        -> /carmaker/control_signal
        -> CarMaker 차량 시뮬레이션
```

즉 localization은 현재 차량 상태를 추정하고, planning은 차량이 따라갈 trajectory를 생성하며, control은 그 trajectory를 실제 차량 명령으로 변환하는 마지막 단계다.

`carmaker_control`이 직접 사용하는 입력은 기본적으로 두 개다.

```text
/planning/trajectory   carmaker_msgs/TrajectoryPath
/localization/odom     nav_msgs/Odometry
```

localization 없이 GT로 테스트할 때는 현재 상태 입력만 다음으로 바꾼다.

```text
/planning/trajectory   carmaker_msgs/TrajectoryPath
/carmaker/dynamic_info carmaker_msgs/DynamicsInfo
```

최종 출력은 하나다.

```text
/carmaker/control_signal   carmaker_msgs/Control_Signal
```

control 기준에서 각 단계의 역할은 다음과 같다.

| 단계 | 담당 패키지 | control과의 관계 |
| --- | --- | --- |
| segmentation | `carmaker_image`, `segmentation` 계열 | control이 직접 보지 않음 |
| localization | `carmaker_localization` | 기본 모드에서 `/localization/odom`으로 현재 pose, velocity 제공 |
| planning | `carmaker_planning` | `/planning/trajectory`로 추종할 경로 제공 |
| control | `carmaker_control` | PID 속도 제어, Stanley 조향 제어, 기어 전환 처리. 테스트 모드에서는 `/carmaker/dynamic_info` GT 사용 가능 |
| vehicle interface | CarMaker bridge | `/carmaker/control_signal`을 받아 차량에 적용 |

중요한 점은 control이 segmentation 결과를 직접 보지 않는다는 것이다. 현재 차량 상태는 기본적으로 localization이 만든 `/localization/odom`을 사용하고, localization 없는 성능 확인에서는 CarMaker GT인 `/carmaker/dynamic_info`를 선택할 수 있다. 경로는 두 모드 모두 planning이 만든 `/planning/trajectory`를 사용한다.

## Controller 노드의 역할

`carmaker_control` 노드는 planning이 만든 trajectory를 차량이 실제로 따라가도록 만드는 trajectory follower다.

| 입력 또는 출력 | topic | control에서 사용하는 정보 |
| --- | --- | --- |
| Localization 입력 | `/localization/odom` | 현재 차량 위치, 자세, 속도 |
| GT 테스트 입력 | `/carmaker/dynamic_info` | CarMaker GT 위치, yaw, 속도 |
| Planning 입력 | `/planning/trajectory` | 따라갈 경로, 목표 속도, 곡률, 시간 정보 |
| Control 출력 | `/carmaker/control_signal` | 조향, gas, brake, accel, gear 명령 |

control 노드는 다음 질문에 매 제어 tick마다 답한다.

```text
현재 차량 상태에서 planning 경로를 따라가려면
조향을 얼마나 꺾고,
가속할지 브레이크를 밟을지,
기어를 drive/reverse/neutral 중 무엇으로 둘지
```

이 역할 때문에 control은 perception이나 planning 내부 알고리즘을 다시 계산하지 않는다. 이미 만들어진 현재 상태와 목표 경로를 받아 차량 제어 명령으로 바꾸는 노드다.

## 실행 방법

Ubuntu ROS PC에서 실행한다고 가정한다. 각 launch는 보통 별도 터미널에서 실행하고, 터미널마다 workspace setup을 먼저 source한다.

### 1. 빌드

필요 패키지만 빌드하는 경우:

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

### 2. CarMaker 및 bridge 실행

CarMaker bridge가 최소한 다음 동역학 토픽을 publish해야 한다.

```text
/carmaker/dynamic_info   carmaker_msgs/DynamicsInfo
```

segmentation과 localization이 사용하는 카메라, 동역학, map 입력도 이 시점에 준비되어야 한다. control은 이 입력들을 직접 구독하지 않지만, localization과 planning이 정상 동작하려면 앞단 입력이 먼저 살아 있어야 한다.

### 3. Localization 실행

localization은 control에 필요한 현재 상태를 `/localization/odom`으로 내보낸다.

```bash
roslaunch carmaker_localization localization.launch
```

확인할 토픽:

```bash
rostopic hz /localization/odom
rostopic echo /localization/odom
```

`/localization/odom`에서 control이 사용하는 값은 다음과 같다.

| 필드 | 의미 |
| --- | --- |
| `pose.pose.position.x/y` | 현재 차량 위치 |
| `pose.pose.orientation` | 현재 차량 yaw |
| `twist.twist.linear.x` | 차량 진행 방향 기준 속도 |
| `twist.twist.angular.z` | yaw rate |

localization은 planning에서 사용할 TF도 publish한다.

```text
Fr0 -> Fr1A_pred
```

### 4. Planning 실행

```bash
roslaunch carmaker_planning global_planner.launch
```

실제 localization 기반 파이프라인에서는 `src/carmaker_planning/config/global_planner_params.yaml`에서 다음 설정을 확인한다.

```yaml
setting:
  use_gt_pose: false
  use_manual_pose:
    enabled: false
```

`use_manual_pose.enabled`가 `true`이면 planning은 localization TF가 아니라 `/planning/start` 수동 pose를 시작점으로 사용한다. RViz 테스트에는 유용하지만, 실제 파이프라인에서는 `false`로 두는 구성이 자연스럽다.

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
rostopic hz /carmaker/control_signal
rostopic echo /carmaker/control_signal
```

기본값은 localization odometry를 현재 상태 입력으로 쓰는 모드다.

```yaml
control:
  state_source: "odom"
```

localization이 들어오지 않은 상태에서 planning/control 성능만 확인하려면 `src/carmaker_control/config/control_params.yaml`에서 다음처럼 바꾼다.

```yaml
control:
  state_source: "dynamics"
```

이 모드에서는 control이 `/localization/odom` 대신 `/carmaker/dynamic_info`를 구독한다.

planning의 trajectory publisher는 latched publisher이므로, control node를 planning 이후에 켜도 마지막으로 publish된 trajectory를 받을 수 있다. 다만 오래된 경로를 계속 추종하지 않도록 control에는 `trajectory_timeout` 안전장치가 있다.

### 전체 실행 순서 요약

```text
1. CarMaker bridge 및 센서 입력 준비
2. roslaunch carmaker_localization localization.launch
3. roslaunch carmaker_planning global_planner.launch
4. /planning/goal 입력
5. roslaunch carmaker_control control.launch
6. /carmaker/control_signal 확인
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
| `src/carmaker_control/docs/Hyundai_Ioniq_5_ailab.md` | Ioniq 5 차량 제원과 조향 한계 메모 |

`ControlNode`는 `control_node.cpp` 안에만 있다. 다른 `.cpp`에서 재사용하지 않는 실행 파일 내부 클래스이기 때문에 별도 헤더를 두지 않는다.

`PID`, `Stanley`는 `control_node.cpp`에서 사용하고 각각의 `.cpp`에서 구현되므로, 클래스 선언을 공유하기 위한 `.hpp` 헤더가 있다.

## ROS 연결 구조

`ControlNode`는 생성자에서 다음 연결을 만든다.

```text
subscribe: /planning/trajectory
subscribe: /localization/odom 또는 /carmaker/dynamic_info
publish:   /carmaker/control_signal
timer:     controlTimerCallback(), 기본 30 Hz
```

trajectory callback이나 odom callback에서 바로 제어 명령을 계산하지 않는다. callback은 최신 데이터를 저장만 하고, 실제 제어 계산은 timer loop에서 주기적으로 수행한다.

```text
trajectoryCallback()
  -> 최신 trajectory 저장
  -> 전진/후진 segment 분할
  -> active index와 PID 상태 초기화

odomCallback()
  -> 최신 odom 저장
  -> odom 수신 시각 갱신

dynamicsCallback()
  -> 최신 DynamicsInfo GT 저장
  -> GT 수신 시각 갱신

controlTimerCallback()
  -> 30 Hz 주기로 실제 제어 계산
  -> PID/Stanley 결과 publish
```

이 구조의 장점은 planning topic이나 localization topic의 수신 주기가 일정하지 않아도, control loop는 일정한 주기로 동작할 수 있다는 점이다.

## Planning 결과가 Control로 이어지는 방식

planning 내부의 경로 표현은 `carmaker_planning::PathPoint`이다.

중요 필드는 다음이다.

| 필드 | 의미 |
| --- | --- |
| `x`, `y`, `theta` | 경로점 pose |
| `kappa` | 곡률 |
| `v` | 속도 크기 |
| `a` | 가속도 |
| `s` | 경로 누적 거리 |
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

특히 velocity profiling은 전진/후진 전환 지점에서 속도를 0으로 만든다. 따라서 planning 결과는 control에게 "여기서 멈춘 뒤 기어를 바꿔야 한다"는 정보를 속도 프로파일로 전달한다.

현재 tight parking 기본 profile은 `src/carmaker_planning/config/global_planner_params.yaml`에서 `global_post_processing/profiler/max_vel: 0.7`로 둔다. 이는 약 `2.5 km/h`이며, 조향 추종이 안정될 때까지 속도와 횡가속을 낮추기 위한 기준값이다.

마지막으로 planning은 `/planning/trajectory`를 publish할 때 direction을 별도 필드로 보내지 않고, `longitudinal_velocity` 부호에 실어 보낸다.

```cpp
tp.longitudinal_velocity = pt.v * pt.direction;
tp.longitudinal_acceleration = pt.a * pt.direction;
tp.curvature = pt.kappa;
tp.time_from_start = ros::Duration(pt.t);
```

control은 `longitudinal_velocity`를 다음처럼 해석한다.

| `longitudinal_velocity` | control 해석 |
| --- | --- |
| `> 0` | 전진 |
| `< 0` | 후진 |
| `약 0` | 정지점 또는 기어 전환점 |

## 메시지 구조

### `carmaker_msgs/TrajectoryPath`

```text
std_msgs/Header header
carmaker_msgs/TrajectoryPoint[] points
```

### `carmaker_msgs/TrajectoryPoint`

```text
geometry_msgs/Pose pose
float64 longitudinal_velocity
float64 longitudinal_acceleration
float64 curvature
duration time_from_start
```

### `carmaker_msgs/Control_Signal`

```text
std_msgs/Header header
float32 steerangle
float32 brake
float32 gas
float32 accel
int32 gear
```

### `carmaker_msgs/DynamicsInfo` 중 control GT mode에서 쓰는 필드

```text
float64 RearAxle_x
float64 RearAxle_y
float64 RearAxle_z
float64 Car_Yaw
float64 Car_vx
```

`RearAxle_x`, `RearAxle_y`, `Car_Yaw`는 rear axle 기준 GT pose로 사용하고, `Car_vx`는 signed longitudinal speed로 사용한다. `RearAxle_z`는 메시지에 포함되지만 2D control pose에서는 사용하지 않는다.

planning은 rear axle 기준으로 경로를 만들고 `/planning/trajectory`도 rear axle 기준점으로 내보낸다. 따라서 control도 현재 pose를 rear axle 기준으로 맞춰서 nearest point, segment 완료 판정, Stanley 오차 계산을 수행한다.

## Control 내부 처리 순서

control loop는 다음 순서로 동작한다.

```text
1. /planning/trajectory 수신
2. longitudinal_velocity 부호 기준으로 trajectory segment 분할
3. /localization/odom에서 현재 pose, speed 수신
4. timer loop에서 현재 active segment 선택
5. active segment 안에서 nearest point 검색
6. lookahead point에서 target speed 선택
7. PID로 accel command 계산
8. Stanley로 steer command 계산
9. Control_Signal로 gas/brake/accel/steerangle/gear publish
10. segment 끝에 도착하면 정지
11. 속도가 충분히 낮아지면 다음 gear segment로 전환
12. 정지 상태에서 다음 segment 조향각을 먼저 보낸 뒤 출발
```

control은 단순히 trajectory를 받아 그대로 내보내는 노드가 아니다. 매 순간 현재 차량 위치와 경로를 비교하면서 속도 제어와 조향 제어를 동시에 수행한다.

### 1. Trajectory 수신과 segment 분할

planning 결과에는 전진과 후진이 섞여 있을 수 있다.

예를 들어 trajectory의 속도 부호가 다음과 같다고 하면:

```text
+ + + + 0 - - - 0 + + +
```

control 내부에서는 다음과 같이 나눈다.

```text
Segment 0: forward
Segment 1: reverse
Segment 2: forward
```

이 작업을 하는 함수가 `splitTrajectory()`다.

분할 기준은 `longitudinal_velocity` 부호다.

- `direction_velocity_epsilon`보다 큰 양수: 전진
- `direction_velocity_epsilon`보다 작은 음수: 후진
- 그 사이 값: 정지점 또는 전환점

정지점 근처의 0 속도 point는 단순히 버리지 않는다. 현재 진행 중인 direction에 붙이고, direction이 바뀌는 경계에서는 이전 segment의 마지막 point를 다음 segment의 시작 boundary로 복사한다. 이 덕분에 전진/후진 전환 지점에서 두 segment가 같은 기준점을 공유한다.

control이 전체 trajectory에서 바로 nearest point를 찾지 않고 현재 active segment 안에서만 nearest point를 찾는 이유는, 전진/후진 경로가 가까이 붙어 있을 때 잘못된 segment를 따라가는 문제를 막기 위해서다.

```text
전체 trajectory에서 nearest point 탐색 X
현재 active segment 안에서 nearest point 탐색 O
```

즉 control은 전체 경로를 한 번에 따라가는 것이 아니라, 다음 순서로 안정적으로 경로를 추종한다.

```text
전진 segment 추종
  -> 정지
  -> 후진 segment 추종
  -> 정지
  -> 다음 segment 추종
```

### 2. 현재 상태 수신

control은 `control/state_source` 값에 따라 현재 상태 입력을 선택한다. 기본값은 `/localization/odom`이다.

```text
pose.x = odom.pose.pose.position.x
pose.y = odom.pose.pose.position.y
pose.yaw = odom.pose.pose.orientation
speed = odom.twist.twist.linear.x
```

localization 없이 GT로 테스트할 때는 `/carmaker/dynamic_info`에서 다음 값을 사용한다.

```text
pose.x = dynamics.RearAxle_x
pose.y = dynamics.RearAxle_y
pose.yaw = dynamics.Car_Yaw
speed = dynamics.Car_vx
```

`DynamicsInfo.Car_x/y`가 bumper 기준으로 들어오더라도 control GT mode는 `RearAxle_x/y`를 직접 사용하므로 내부에서 별도 기준점 보정을 하지 않는다.

이 정보가 있어야 planning이 만든 trajectory 중 현재 차량과 가장 가까운 지점을 찾을 수 있고, 차량이 경로에서 얼마나 벗어났는지도 계산할 수 있다.

정식 localization 기반 폐루프에서는 `/localization/odom`을 쓰는 것이 맞다. `/carmaker/dynamic_info` GT 모드는 localization이 아직 없거나 불안정할 때 planning/control 자체의 성능을 분리해서 확인하기 위한 테스트 모드다.

### 3. Nearest point 검색

`findNearestIndex()`는 현재 active segment 안에서 차량 위치와 가장 가까운 trajectory point를 찾는다.

검색 범위는 전체 segment가 아니라 이전 nearest index 주변으로 제한된다.

```yaml
control:
  nearest_search_back: 20
  nearest_search_ahead: 120
```

이 제한은 차량이 경로가 가까이 겹치는 구간을 지날 때 index가 갑자기 먼 지점으로 튀는 문제를 줄인다.

### 4. Lookahead point 선택

속도 목표는 nearest point가 아니라 일정 거리 앞의 lookahead point에서 읽는다.

```text
lookahead = lookahead_distance + lookahead_time * current_speed
```

그리고 다음 범위로 제한한다.

```yaml
control:
  min_lookahead_distance: 0.2
  max_lookahead_distance: 0.8
```

속도가 높을수록 조금 더 앞을 보고, 너무 가까이 보거나 너무 멀리 보는 상황은 min/max로 막는다.

### 5. 속도 제어

속도 제어는 PID가 담당한다.

```text
target_speed = trajectory lookahead point의 속도 크기
current_speed = abs(/localization/odom.twist.twist.linear.x)
accel_command = PID(target_speed, current_speed)
```

PID는 전진/후진 자체를 결정하는 역할보다는, 현재 segment 안에서 얼마나 빠르게 움직일 것인지를 제어하는 역할에 가깝다. 그래서 target speed와 current speed는 절댓값으로 비교한다.

segment 완료 판정이 나기 전에는 lookahead point가 기어 전환점의 0 속도를 읽더라도 `min_tracking_speed` 이상의 속도 목표를 유지한다. 이렇게 해야 차량이 segment 끝점에 도착하기 전에 멈춰 서서 다음 전진/후진 segment로 넘어가지 못하는 상황을 피할 수 있다.

```text
target_speed > current_speed
  -> 가속 필요
  -> gas 증가

target_speed < current_speed
  -> 감속 필요
  -> brake 증가
```

PID 출력은 바로 `gas`가 아니라 가속도 명령이다. 이후 `publishControl()`에서 다음으로 변환된다.

```text
accel_command > 0 -> gas
accel_command < 0 -> brake
```

`pid.cpp`의 PID는 출력 범위를 `-max_decel`부터 `max_accel` 사이로 제한하고, saturation 상태에서 integral wind-up이 커지지 않도록 적분 후보를 조건부로 반영한다.

### 6. 조향 제어

조향은 Stanley controller가 담당한다.

Stanley controller는 차량이 경로 중심선을 따라가도록 조향각을 계산한다. 이때 주로 두 가지 오차를 사용한다.

```text
heading error = path yaw - vehicle yaw
cross-track error = 차량이 경로 중심선에서 좌우로 벗어난 정도
```

`control_node.cpp`에서는 nearest point 기준으로 다음 값을 만든다.

```text
dx = vehicle.x - reference.x
dy = vehicle.y - reference.y
cte = sin(reference.yaw) * dx - cos(reference.yaw) * dy
heading_error = normalize(reference.yaw - vehicle.yaw)
```

`stanley.cpp`의 feedback 계산식은 개념적으로 다음과 같다.

```text
cte_term = cte_gain * atan2(k * cte, abs(velocity) + k_soft)
feedback_steer = heading_gain * heading_error + cte_term
```

여기에 lookahead/target point에서 읽은 trajectory `curvature` 기반 feedforward를 더한다.

```text
curvature_ff = curvature_ff_gain * atan(wheelbase * trajectory_curvature)
steer = feedback_steer + curvature_ff
```

feedback은 nearest point 기준 경로 오차가 생긴 뒤 보정하는 성격이고, curvature feedforward는 lookahead point의 path 곡률을 보고 미리 꺾는 성격이다. 주차처럼 곡률이 큰 path에서는 feedforward가 조향 반응 지연을 줄인다. 최종 조향각은 타이어 조향 한계 안으로 clamp된다.

### 7. 후진 segment 처리

후진 segment에서는 같은 조향각이 yaw를 전진과 반대로 변화시킨다. 그래서 후진 구간에서는 Stanley의 heading error 항을 후진 yaw dynamics에 맞게 반대로 적용한다.

```text
forward: steer = heading_error_term + lateral_error_term
reverse: steer = -heading_error_term + reverse_steering_scale * lateral_error_term
```

반대로 lateral error 항까지 뒤집으면 후진 path 좌우 기준이 반대로 해석되어 후진 path와 반대 방향으로 조향할 수 있다. 따라서 trajectory pose yaw는 차량 자세 기준으로 유지하고, 후진에서는 heading error 항만 부호를 바꾸며 lateral error 항은 path 좌우 기준을 유지한다.

이후 `steering_ratio`와 `max_steer_command`로 CarMaker 입력 범위에 맞춘다.

### 8. Segment 끝 처리와 기어 전환

control은 segment가 끝났다고 바로 다음 segment로 넘어가지 않는다.

segment 완료는 두 조건을 함께 본다.

- nearest index가 segment 끝 근처인가
- 차량 위치가 segment 마지막 point와 충분히 가까운가

이렇게 하는 이유는 U-turn이나 후진 경로처럼 경로가 공간상 가까이 붙은 경우, 단순 거리 조건만으로 너무 일찍 segment 완료가 되는 일을 막기 위해서다.

segment 끝에 도달하면 먼저 brake를 유지한다. 전진에서 후진으로 바뀌거나 후진에서 전진으로 바뀌는 경우, 차량이 충분히 감속한 뒤 다음 segment로 넘어간다.

```text
segment 끝 접근
  -> brake 유지
  -> 현재 속도가 gear_switch_speed 이하인지 확인
  -> 충분히 느려지면 다음 segment로 전환
  -> presteer_duration 동안 다음 segment 조향각 유지
  -> PID/Stanley 추종 재개
```

예를 들어 전진 segment 다음에 후진 segment가 있다면 다음과 같이 동작한다.

```text
전진 segment 추종
  -> segment 끝 도달
  -> 브레이크로 감속
  -> gear_switch_speed 이하로 감소
  -> reverse gear 전환
  -> 후진 segment 첫 조향각으로 pre-steer
  -> 후진 segment 추종
```

이 구조는 차량이 아직 움직이고 있는데 바로 반대 방향 gear로 바뀌는 것을 막고, 정지 상태에서 다음 segment 곡률에 맞춰 조향을 먼저 잡아 출발 초반 경로 이탈을 줄이기 위한 안정화 로직이다.

### 9. 최종 Control_Signal publish

`publishControl()`은 PID와 Stanley 결과를 CarMaker가 받을 수 있는 `Control_Signal` 메시지로 변환한다.

| 출력 필드 | 의미 |
| --- | --- |
| `steerangle` | Stanley 기반 조향각 |
| `gas` | PID 결과 기반 가속 페달 명령 |
| `brake` | PID 결과 기반 브레이크 명령 |
| `accel` | direction 부호까지 반영한 가속도 명령 |
| `gear` | drive, reverse, neutral 중 선택 |

기어는 현재 active segment 방향에 따라 결정된다.

```text
전진 segment -> drive gear
후진 segment -> reverse gear
정지/완료/대기 -> neutral gear
```

후진은 gear가 reverse이므로 속도 목표는 항상 절댓값으로 다루고, 최종 `accel` 필드에 direction 부호를 반영한다.

## 안전 정지 동작

control은 입력 데이터가 유효하지 않으면 정지 명령을 낸다. `publishStop()`은 다음 상황에서 호출된다.

```text
trajectory가 없음
odom이 없음
입력 데이터 timeout
segment 완료 후 정지 필요
전체 trajectory 완료
비정상 상태
```

정지 명령은 대략 다음 형태다.

```text
gear = neutral 또는 현재/다음 segment gear
gas = 0
brake = 양수
accel = 0 또는 감속
steerangle = 0 또는 지정된 steer 값
```

`direction = 0`이면 경로가 없거나 완료된 상태이므로 neutral gear로 brake를 유지한다. `direction`이 있으면 현재 또는 다음 gear를 유지한 채 brake를 걸어 기어 전환 타이밍을 안정화한다.

## 주요 토픽

| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
| --- | --- | --- | --- | --- |
| `/localization/odom` | `nav_msgs/Odometry` | `carmaker_localization` | `carmaker_control` | 현재 pose와 속도 |
| `/planning/goal` | `geometry_msgs/PoseStamped` | RViz 또는 goal publisher | `carmaker_planning` | 목표 pose |
| `/planning/trajectory` | `carmaker_msgs/TrajectoryPath` | `carmaker_planning` | `carmaker_control` | 추종 경로 |
| `/carmaker/control_signal` | `carmaker_msgs/Control_Signal` | `carmaker_control` | CarMaker bridge | 최종 제어 명령 |

## Control debug 토픽

`debug/publish: true`이면 control node가 PlotJuggler/RViz 진단용 토픽을 추가로 publish한다. 기본 prefix는 `/control/debug`다.

### RViz용 토픽

| 토픽 | 타입 | 설명 |
| --- | --- | --- |
| `/control/debug/current_pose` | `geometry_msgs/PoseStamped` | control이 현재 상태로 사용 중인 차량 pose. `odom` 모드면 localization pose, `dynamics` 모드면 GT pose |
| `/control/debug/current_control_pose` | `geometry_msgs/PoseStamped` | Stanley 조향 오차 계산에 쓰는 rear axle pose. 현재는 `/control/debug/current_pose`와 동일 |
| `/control/debug/nearest_pose` | `geometry_msgs/PoseStamped` | 현재 active segment에서 차량과 가장 가까운 trajectory point |
| `/control/debug/nearest_control_pose` | `geometry_msgs/PoseStamped` | Stanley 조향 오차 계산에 쓰는 rear axle 기준 nearest pose. 현재는 `/control/debug/nearest_pose`와 동일 |
| `/control/debug/lookahead_pose` | `geometry_msgs/PoseStamped` | 목표 속도를 읽는 lookahead point |
| `/control/debug/active_segment_path` | `nav_msgs/Path` | 현재 추종 중인 전진/후진 segment |

RViz 설정 예:

```text
Fixed Frame: Fr0

Add -> Pose
  Topic: /control/debug/current_pose

Add -> Pose
  Topic: /control/debug/current_control_pose

Add -> Pose
  Topic: /control/debug/nearest_pose

Add -> Pose
  Topic: /control/debug/nearest_control_pose

Add -> Pose
  Topic: /control/debug/lookahead_pose

Add -> Path
  Topic: /control/debug/active_segment_path
```

### PlotJuggler용 토픽

| 토픽 | 타입 | 설명 |
| --- | --- | --- |
| `/control/debug/current_speed` | `std_msgs/Float64` | 현재 속도 크기 |
| `/control/debug/target_speed` | `std_msgs/Float64` | lookahead point의 목표 속도 |
| `/control/debug/speed_error` | `std_msgs/Float64` | `target_speed - current_speed` |
| `/control/debug/steer_command` | `std_msgs/Float64` | 최종 `/carmaker/control_signal/steerangle`로 나가는 조향 명령 |
| `/control/debug/curvature_feedforward` | `std_msgs/Float64` | lookahead trajectory curvature에서 미리 더한 조향 명령 성분 |
| `/control/debug/steer_saturated` | `std_msgs/Int32` | `1`: 조향 명령이 `max_steer_command` 근처에서 포화됨, `0`: 비포화 |
| `/control/debug/cross_track_error` | `std_msgs/Float64` | nearest point 기준 lateral error |
| `/control/debug/heading_error` | `std_msgs/Float64` | nearest point 기준 yaw error |
| `/control/debug/lookahead_distance` | `std_msgs/Float64` | 현재 tick에서 사용한 lookahead 거리 |
| `/control/debug/segment_index` | `std_msgs/Int32` | 현재 active segment index |
| `/control/debug/segment_count` | `std_msgs/Int32` | 현재 trajectory가 전진/후진 부호 기준으로 나뉜 segment 개수 |
| `/control/debug/nearest_index` | `std_msgs/Int32` | active segment 안에서 현재 pose와 가장 가까운 point index. 추종 중이 아니면 `-1` |
| `/control/debug/target_index` | `std_msgs/Int32` | target speed를 읽는 lookahead point index. 추종 중이 아니면 `-1` |
| `/control/debug/distance_to_segment_end` | `std_msgs/Float64` | 현재 pose에서 active segment 마지막 point까지 거리. 추종 중이 아니면 `-1` |
| `/control/debug/trajectory_age` | `std_msgs/Float64` | 마지막 `/planning/trajectory` 수신 후 지난 시간. trajectory를 받은 적 없으면 `-1` |
| `/control/debug/trajectory_completed` | `std_msgs/Int32` | `1`: trajectory 완료 상태, `0`: 아직 완료 아님 |
| `/control/debug/direction` | `std_msgs/Int32` | `1`: 전진, `-1`: 후진, `0`: 추종 중 아님 |
| `/control/debug/tracking_state` | `std_msgs/Int32` | `0`: idle/no trajectory, `1`: tracking, `2`: stopping at segment end, `3`: pre-steering before next segment |
| `/control/debug/stop_reason` | `std_msgs/Int32` | `0`: none/tracking, `1`: current state unavailable, `2`: no valid trajectory or timeout, `3`: segment arrived/braking, `4`: pre-steering hold, `5`: final trajectory arrived |

PlotJuggler에서 함께 보면 좋은 기본 제어 출력:

```text
/carmaker/control_signal/gear
/carmaker/control_signal/gas
/carmaker/control_signal/brake
/carmaker/control_signal/steerangle
/carmaker/control_signal/accel
/control/debug/steer_command
/control/debug/curvature_feedforward
/control/debug/steer_saturated
/control/debug/current_speed
/control/debug/target_speed
/control/debug/speed_error
/control/debug/cross_track_error
/control/debug/heading_error
/control/debug/tracking_state
/control/debug/stop_reason
/control/debug/direction
/control/debug/segment_index
/control/debug/segment_count
/control/debug/distance_to_segment_end
/control/debug/trajectory_age
/control/debug/trajectory_completed
```

정상 패턴:

- 직선 전진에서는 `cross_track_error`, `heading_error`, `steerangle`이 0 근처로 수렴한다.
- `current_speed`가 `target_speed`보다 낮으면 `gas`가 증가하고, 높으면 `brake`가 증가한다.
- `steer_saturated = 1`이 계속 유지되면 controller가 더 꺾고 싶지만 `max_steer_command` 한계에 걸린 상태다.
- segment 끝에서는 `tracking_state = 2`, `brake > 0`이 되고, 속도가 `gear_switch_speed` 이하로 내려가면 다음 segment로 넘어간다.
- segment 전환 직후에는 `tracking_state = 3`, `brake > 0` 상태에서 다음 segment 조향각이 먼저 나간다.
- segment 전환이 정상이라면 `segment_index`가 증가하고 `direction`이 다음 segment 방향으로 바뀐다.
- `stop_reason = 3`이면 segment 끝점에 도착해 감속 중이고, `stop_reason = 5`이면 전체 trajectory 최종 도착이다.
- `stop_reason = 1` 또는 `2`이면 도착이 아니라 입력 상태나 trajectory freshness 문제로 정지한 것이다.
- segment 도착/최종 도착 시에는 ROS_INFO에도 segment 번호, 속도, 끝점 거리가 남는다.
- 전진 segment에서는 `direction = 1`, `gear = drive_gear`다.
- 후진 segment에서는 `direction = -1`, `gear = reverse_gear`다.

## 주요 파라미터

### 토픽 설정

```yaml
topics:
  subscribe:
    trajectory: "/planning/trajectory"
    odom: "/localization/odom"
    dynamics: "/carmaker/dynamic_info"
  publish:
    control: "/carmaker/control_signal"
```

### 현재 상태 입력 선택

```yaml
control:
  state_source: "odom"
```

사용 가능한 값:

| 값 | 입력 토픽 | 용도 |
| --- | --- | --- |
| `odom` | `/localization/odom` | 실제 localization 기반 control |
| `dynamics` | `/carmaker/dynamic_info` | localization 없이 GT로 planning/control 성능 확인 |

`gt`는 `dynamics`, `localization`은 `odom`의 alias로 처리된다.

### GT 입력 기준점

`/planning/trajectory`는 rear axle 기준점으로 publish된다. GT mode에서도 control은 `DynamicsInfo.RearAxle_x/y`를 그대로 사용해 `/control/debug/current_pose`를 rear axle 기준으로 둔다.

따라서 control에는 GT 기준점 보정용 `rear_axle_offset`이나 `dynamics_pose_reference` 파라미터가 없다.

### 안전 timeout

```yaml
control:
  odom_timeout: 0.5
  dynamics_timeout: 0.5
  trajectory_timeout: 300.0
```

`odom_timeout` 안에 localization odometry가 갱신되지 않으면 정지 명령을 낸다.

`dynamics_timeout` 안에 CarMaker GT DynamicsInfo가 갱신되지 않으면 정지 명령을 낸다.

`trajectory_timeout` 안에 새 planning trajectory가 없으면 경로가 오래되었다고 보고 정지한다. 저속 주차 trajectory는 30초 이상 걸릴 수 있으므로 기본값은 300초로 둔다.

### 전진/후진 segment 판단

```yaml
control:
  direction_velocity_epsilon: 0.02
```

이 값보다 작은 속도는 정지점으로 본다. 너무 작게 두면 노이즈로 segment가 잘게 쪼개질 수 있고, 너무 크게 두면 저속 후진 구간을 정지점으로 잘못 볼 수 있다.

### Lookahead

```yaml
control:
  min_tracking_speed: 0.20
  max_target_speed: 0.7
  lookahead_distance: 0.35
  lookahead_time: 0.2
  min_lookahead_distance: 0.2
  max_lookahead_distance: 0.8
```

lookahead가 너무 짧으면 조향과 속도 목표가 자주 흔들릴 수 있고, 너무 길면 코너나 정지점 반영이 늦어질 수 있다.

`min_tracking_speed`는 segment 완료 판정이 나기 전까지 적용되는 최소 속도 목표다. `/control/debug/target_speed`가 0이고 `/control/debug/tracking_state`가 1이면 차량이 끝점에 도착하기 전에 멈출 수 있으므로 이 값이 필요하다. 완료 판정 후에는 `tracking_state = 2`가 되고 stop/gear switch 로직이 brake와 다음 gear 전환을 처리한다.

조향 추종이 안 되는 tight parking 상황에서는 `max_target_speed: 0.7`을 기본 상한으로 둔다. 먼저 path 추종이 안정되는지 확인하고, 안정화 이후에만 planning의 `max_vel`과 control의 `max_target_speed`를 함께 올린다.

### Segment 완료와 기어 전환

```yaml
control:
  segment_finish_distance: 0.5
  segment_finish_index_margin: 3
  gear_switch_speed: 0.08
  presteer_enabled: true
  presteer_duration: 0.6
```

segment 끝에 도달해도 현재 속도가 `gear_switch_speed`보다 크면 다음 segment로 넘어가지 않는다. 먼저 brake를 걸고 충분히 느려진 뒤 기어를 바꾼다.

`presteer_enabled`가 켜져 있으면 다음 segment로 넘어간 직후 `presteer_duration` 동안 brake를 유지하면서 다음 segment의 nearest/lookahead point 기준 Stanley 조향각을 먼저 보낸다. 시간이 지나면 PID 속도 제어를 다시 허용해 출발한다.

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

### Gas, brake, accel 제한

```yaml
control:
  max_accel: 1.0
  max_decel: 1.5
  max_gas: 0.45
  max_brake: 1.0
  stop_brake: 0.4
```

PID 출력은 `[-max_decel, max_accel]` 범위의 가속도 명령이다. 이 값이 `publishControl()`에서 `gas`, `brake`, `accel` 필드로 변환된다.

### Gear mapping

```yaml
control:
  drive_gear: 1
  neutral_gear: 0
  reverse_gear: -1
```

CarMaker 프로젝트마다 gear 숫자 규약이 다를 수 있으므로, 후진이 되지 않거나 neutral 상태가 이상하면 이 값을 먼저 확인한다.

### Stanley

```yaml
stanley:
  k: 2.5
  k_soft: 0.15
  cte_gain: 2.0
  heading_gain: 1.8
  max_steer_angle_deg: 28.4
  reverse_steering_scale: 1.3
  curvature_ff_gain: 0.6
  reverse_curvature_ff_sign: -1.0
```

`max_steer_angle_deg`는 Ioniq 5 설정값인 `wheelbase=2.97m`, `min_turning_radius=5.5m`에서 계산한 타이어 조향각 한계다.

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
| 코너 진입에서 늦게 꺾음 | `curvature_ff_gain` 증가 |
| 코너 진입에서 너무 먼저/과하게 꺾음 | `curvature_ff_gain` 감소 |

후진 조향은 전진 Stanley 결과 전체를 단순히 뒤집지 않는다. trajectory pose yaw는 차량 자세 기준으로 유지하고, 후진에서는 heading error 항만 반대로 적용한다. lateral error 항은 path 좌우 기준을 유지하며, `reverse_steering_scale`은 후진 lateral error 보정 강도를 조절하는 값으로 보면 된다.

`curvature_ff_gain`은 lookahead trajectory curvature를 이용한 선제 조향 비율이다. `0`이면 순수 feedback Stanley에 가깝고, 값을 키우면 큰 곡률에서 path를 따라 미리 꺾는다. 후진 feedforward는 차량 yaw dynamics가 전진과 반대이므로 `reverse_curvature_ff_sign: -1.0`을 기본값으로 둔다.

### CarMaker 조향 입력 범위

```yaml
vehicle:
  wheelbase: 2.97
  min_turning_radius: 5.5
  steering_ratio: 9.0
  max_steer_command: 4.5
  steering_command_sign: 1.0
```

Stanley는 타이어 조향각을 계산한다. CarMaker `control_signal/steerangle`이 steering wheel command 범위를 기대하는 프로젝트라면 `steering_ratio`와 `max_steer_command`를 프로젝트 입력 범위와 맞춰야 한다. 현재 tight parking 기본값은 조향 부족을 줄이기 위해 `max_steer_command: 4.5`까지 허용한다.

`steering_command_sign`은 조향 부호 테스트용이다. RViz/PlotJuggler에서 `cross_track_error`가 줄지 않고 커지거나, 좌회전해야 하는데 우회전으로 반응하면 `-1.0`으로 바꿔서 바로 비교한다.

Stanley 오차 계산은 `/planning/trajectory`와 current pose가 공유하는 rear axle 기준에서 바로 수행한다.

### Debug publish

```yaml
debug:
  publish: true
  topic_prefix: "/control/debug"
  frame_id: "Fr0"
  path_publish_period: 1.0
```

`publish`를 `false`로 두면 control debug 토픽을 끌 수 있다. `path_publish_period`는 `/control/debug/active_segment_path` publish 주기이며, 0 이하로 두면 매 control tick마다 publish한다.

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
- yaw가 차량 진행 방향과 일관되는지

### GT DynamicsInfo 확인

GT mode를 쓸 때 확인한다.

```bash
rostopic hz /carmaker/dynamic_info
rostopic echo /carmaker/dynamic_info
```

확인할 것:

- `RearAxle_x`, `RearAxle_y`가 후륜축 위치와 맞는지
- `Car_Yaw`가 radian 기준 yaw로 들어오는지
- `Car_vx`가 차량 전후방 속도와 맞는지
- 후진 시 `Car_vx` 부호가 실제 후진 방향과 일관되는지
- RViz에서 `/control/debug/current_pose`가 `/control/debug/active_segment_path` 위에 놓이는지
- current pose가 경로보다 진행 방향으로 어긋나면 `/carmaker/dynamic_info`의 `RearAxle_x/y` publish 값과 planning/control이 보는 dynamic info 토픽이 같은지 확인한다.

### planning 확인

```bash
rostopic echo /planning/trajectory
```

확인할 것:

- `points`가 비어 있지 않은지
- `longitudinal_velocity`가 전진 구간에서 양수인지
- 후진 구간이 있으면 음수인지
- 전진/후진 전환 근처에서 속도가 0에 가까워지는지
- `pose.orientation`의 yaw가 경로 진행 방향과 맞는지

### control 확인

```bash
rostopic echo /carmaker/control_signal
```

확인할 것:

- 경로가 없을 때 `gear = neutral_gear`, `brake > 0`인지
- 전진 segment에서 `gear = drive_gear`인지
- 후진 segment에서 `gear = reverse_gear`인지
- `steerangle`이 너무 큰 값으로 계속 포화되지 않는지
- `/control/debug/steer_saturated`가 계속 `1`이면 조향 한계에 걸린 상태인지
- `gas`와 `brake`가 동시에 과하게 나오지 않는지
- segment 끝에서 먼저 감속한 뒤 다음 gear로 넘어가는지
- segment 전환 직후 `tracking_state = 3` 동안 brake를 유지하면서 다음 segment 조향각이 먼저 나가는지

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

RViz에서 offline으로 planning만 확인할 때는 `use_manual_pose.enabled: true`가 편할 수 있다. 이 경우 control의 현재 상태는 여전히 `/localization/odom`이므로, 실제 폐루프 주행과는 기준점이 달라질 수 있다.

### control이 계속 정지 명령만 내는 경우

다음을 확인한다.

```bash
rostopic hz /localization/odom
rostopic hz /planning/trajectory
rostopic echo /carmaker/control_signal
```

가능한 원인:

- localization odometry timeout
- GT mode에서 DynamicsInfo timeout
- trajectory timeout
- trajectory points가 비어 있음
- 현재 차량이 segment 끝으로 판정되어 정지 중
- `publish_stop_without_path`가 켜져 있고 trajectory를 아직 받지 못함

### 후진 전환이 안 되는 경우

다음을 확인한다.

- `/planning/trajectory`의 후진 구간 velocity가 음수인지
- `gear_switch_speed`보다 차량 속도가 충분히 내려가는지
- `reverse_gear` 값이 CarMaker 프로젝트의 후진 gear 값과 맞는지
- `/control/debug/current_pose`와 `/control/debug/active_segment_path`의 기준점이 같은지
- segment 완료 조건인 `segment_finish_distance`, `segment_finish_index_margin`이 너무 엄격하지 않은지

### 조향 방향이 반대로 보이는 경우

가능한 원인:

- CarMaker의 `steerangle` 부호 규약이 현재 teleop/control 가정과 다름
- localization yaw frame과 trajectory yaw frame이 맞지 않음
- 후진 조향 부호 반전이 차량 모델과 맞지 않음
- CarMaker가 타이어 조향각이 아니라 steering wheel angle을 기대함
- Stanley 오차 계산 기준점이 rear bumper라서 곡률이 큰 주차 경로에서 늦게 반응함

먼저 아주 낮은 속도로 전진 직선 경로를 테스트하고, 그다음 좌/우 곡선, 마지막으로 후진 곡선을 확인한다.

### 속도는 맞는데 경로를 크게 벗어나는 경우

가능한 원인:

- `/localization/odom` pose frame과 `/planning/trajectory` frame이 맞지 않음
- `heading_gain`, `cte_gain`, `k`가 너무 작음
- `lookahead_distance`가 너무 커서 가까운 곡률 변화를 늦게 반영함
- `curvature_ff_gain`이 작아서 큰 곡률을 미리 반영하지 못함
- `max_steer_command`가 너무 작아 조향이 일찍 포화됨
- `steering_command_sign`이 차량 모델과 반대임

### 출발이나 정지가 거친 경우

가능한 원인:

- `kp`가 너무 큼
- `max_accel` 또는 `max_gas`가 큼
- `stop_brake`가 큼
- planning velocity profile의 목표 속도가 control 제한보다 공격적임

조향 추종이 안 되는 tight parking 상황에서는 `global_post_processing/profiler/max_vel`과 control의 `max_target_speed`를 둘 다 `0.7 m/s`로 맞추고 시작한다. 추종이 안정적인데 시간이 너무 오래 걸리면 두 값을 함께 조금씩 올리고, 경로 이탈이나 조향 포화가 커지면 먼저 `lookahead_distance`와 조향 gain을 본다.
