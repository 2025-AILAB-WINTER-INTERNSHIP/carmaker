# Hyundai Ioniq 5 AILAB 차량 제원 반영 기준

이 문서는 `carmaker_planning`과 `carmaker_control`에서 공통으로 사용하는
Hyundai Ioniq 5 차량 제원 기준을 정리한다.

## 현재 적용값

| 항목 | 값 | 적용 위치 |
| --- | ---: | --- |
| 전폭 | `1.89 m` | planning collision / vehicle geometry |
| 전장 | `4.635 m` | planning collision / vehicle geometry |
| 휠베이스 | `2.97 m` | planning kinematics, control steering limit 계산 |
| 후방 범퍼-후륜축 거리 | `0.82 m` | planning vehicle footprint |
| 최소 회전 반경 | `5.5 m` | Hybrid A* / Reeds-Shepp / control steering limit |
| 타이어 최대 조향각 | `28.4 deg` | planning steering candidate, Stanley steering limit |

## 조향각 계산

controller의 Stanley 제어기는 타이어 조향각 기준으로 명령을 계산한다.
따라서 차량의 휠베이스와 최소 회전 반경을 이용해 조향각 한계를 다음처럼 계산한다.

```text
max_steer_angle = atan(wheelbase / min_turning_radius)
                = atan(2.97 / 5.5)
                = 28.4 deg
                = 0.4951 rad
```

`/carmaker/control_signal`의 `steerangle` 입력이 타이어 조향각이면
`steering_ratio: 1.0`을 유지한다. 만약 CarMaker 모델이 steering wheel angle을
기대한다면 `vehicle/steering_ratio`와 `vehicle/max_steer_command`를 모델 입력
스펙에 맞춰 다시 조정해야 한다.

## 가감속 제한 해석

planning의 `max_accel/max_decel`은 속도 프로파일 생성용 제한이고,
control의 `max_accel/max_decel`은 PID 출력과 gas/brake 변환용 제한이다.
두 값은 같은 이름을 쓰지만 역할이 다르므로 반드시 동일할 필요는 없다.

현재 planning profiler는 저속 주행 안정성을 우선해 `0.8 m/s^2`를 사용한다.
controller 제한은 이 프로파일을 추종할 수 있는 범위 안에서 시뮬레이션 응답을 보며
조정한다.
