# EKF 위치 추정(Localization)

이 문서는 현재 `EkfCore`가 사용하는 2D pose-only EKF 구조를 설명한다.

## 1. 상태 벡터

EKF 상태는 차량 후륜축 중심 기준 3차원 포즈다.

$$
x =
\begin{bmatrix}
X & Y & \theta
\end{bmatrix}^T
$$

- **$X, Y$:** 글로벌 프레임(`Fr0`)에서의 후륜축 중심 위치
- **$\theta$:** 글로벌 요각(`YAW`)

종방향 속도, yaw rate, IMU bias는 더 이상 EKF 상태로 추정하지 않는다. 휠/IMU 기반 모션 정보는 prediction 입력으로만 사용한다.

## 2. Motion Input

Prediction 입력은 다음 2차원 벡터다.

$$
u =
\begin{bmatrix}
v & \omega
\end{bmatrix}^T
$$

- **$v$:** 후륜 휠 속도 평균으로 계산한 종방향 속도
- **$\omega$:** IMU z축 gyro yaw rate

`LocalizationNodelet`은 `DynamicsInfo` 메시지가 들어올 때마다 해당 메시지 timestamp 기준으로 `prediction()`을 수행한다. 따라서 EKF prediction 주기는 별도 타이머가 아니라 dynamics 센서 주기와 같다.

## 3. Prediction

한 prediction 구간 동안 입력 $v, \omega$가 일정하다고 가정한다.

$$
X_k = X_{k-1} + v\cos\theta_{k-1}dt
$$

$$
Y_k = Y_{k-1} + v\sin\theta_{k-1}dt
$$

$$
\theta_k = \theta_{k-1} + \omega dt
$$

예측 후 yaw는 항상 `[-pi, pi]` 범위로 정규화한다.

상태 자코비안은 다음과 같다.

$$
F =
\begin{bmatrix}
1 & 0 & -v\sin\theta dt \\
0 & 1 &  v\cos\theta dt \\
0 & 0 & 1
\end{bmatrix}
$$

입력 자코비안은 다음과 같다.

$$
G =
\begin{bmatrix}
\cos\theta dt & 0 \\
\sin\theta dt & 0 \\
0 & dt
\end{bmatrix}
$$

공분산 예측은 다음 형태다.

$$
P_k^- = F P_{k-1}^+ F^T + Q_{pose}dt + G R_u G^T
$$

- **$Q_{pose}$:** motion input 모델이 설명하지 못하는 residual pose process noise
- **$R_u$:** velocity/yaw-rate 입력 노이즈

가감속이 크면 velocity 입력 노이즈를 키우고, 급선회 중이면 yaw 관련 노이즈를 키운다.

## 4. Pose Correction

카메라/ICP 정합 결과만 EKF measurement correction으로 사용한다.

$$
z =
\begin{bmatrix}
X_{obs} & Y_{obs} & \theta_{obs}
\end{bmatrix}^T
$$

측정 모델은 상태를 직접 관측하는 형태다.

$$
h(x) =
\begin{bmatrix}
X & Y & \theta
\end{bmatrix}^T
$$

따라서

$$
H = I_3
$$

innovation의 yaw 항은 원형 변수이므로 `[-pi, pi]`로 wrap한 뒤 Kalman update를 수행한다. 공분산 갱신은 수치 안정성을 위해 Joseph form을 사용한다.

## 5. 제거된 보정

3차원 pose-only 모델에서는 다음 보정 함수가 존재하지 않는다.

- `correctWheel()`
- `correctImu()`

휠 속도와 IMU yaw rate는 상태를 보정하지 않고 prediction 입력으로 들어간다. 즉 입력 오차는 Kalman correction이 아니라 `R_u`와 `Q_{pose}`를 통해 pose 불확실성으로 전파된다.

## 6. 시간 정책

- `DynamicsInfo` timestamp가 단조 증가할 때만 prediction을 수행한다.
- dynamics cycleno가 뒤로 가거나 큰 시간 점프가 감지되면 localization을 reset한다.
- camera correction은 현재 EKF state timestamp 기준으로 지연 보상한다.
- correction timestamp가 현재 state보다 미래인 경우 `EkfCore`는 마지막 motion input으로 해당 시각까지 advance한다.
- state history buffer가 없으므로 out-of-sequence measurement는 적용하지 않는다.
