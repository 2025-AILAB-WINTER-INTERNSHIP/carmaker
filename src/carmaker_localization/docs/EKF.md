# EKF 위치 추정(Localization)

이 문서는 `EkfCore`가 어떤 이산시간 EKF(discrete-time extended Kalman filter) 흐름으로 동작하는지, 그리고 일반식

$$
x_k = f_{k-1}(x_{k-1}, u_{k-1}, w_{k-1}), \quad
y_k = h_k(x_k, v_k)
$$

$$
w_k \sim (0, Q_k), \quad v_k \sim (0, R_k)
$$

을 현재 차량 위치 추정 코드에 어떻게 대응시켰는지 설명한다.

---

## 1. 상태 벡터

EKF 상태는 차량 후륜축 중심을 기준으로 한 6차원 벡터다.

$$
x =
\begin{bmatrix}
X & Y & \theta & V_x & \omega & b_\omega
\end{bmatrix}^T
$$

- **$X, Y$:** 글로벌 프레임(`Fr0`)에서의 후륜축 중심 위치
- **$\theta$:** 글로벌 요각(`YAW`)
- **$V_x$:** 차량 로컬 좌표계의 종방향 속도
- **$\omega$:** 차량 로컬 좌표계의 yaw rate
- **$b_\omega$:** IMU z축 자이로 bias

이 상태 정의는 저속 주차/충전 패드 정렬 환경에서 횡방향 속도 상태를 따로 추정하지 않도록 설계했다. 횡방향 슬립이 작다는 물리 가정을 상태 모델 자체에 넣어, 센서 노이즈가 횡방향 위치 드리프트로 누적되는 것을 줄인다.

---

## 2. 전체 EKF 흐름

`EkfCore`는 다음 순서로 움직인다.

1. **초기화:** `initialize()`에서 $x_0^+$, $P_0^+$를 설정한다. yaw는 즉시 `[-pi, pi]`로 정규화한다.
2. **시간 전파:** `prediction(timestamp, vx_wheel, ax_imu)` 또는 각 `correct*()` 내부의 timestamp 처리에서 현재 상태를 목표 시각까지 전파한다.
3. **공분산 전파:** 선형화 행렬 $F$와 process noise $Q_d$로 $P_k^-$를 계산한다.
4. **측정 보정:** pose, wheel, IMU 측정마다 $z_k$, $h(x_k^-)$, $H$를 구성한다.
5. **innovation covariance 계산:** $S = H P^- H^T + R$를 만든다.
6. **Kalman gain 계산:** 교재 수식과 동일하게 $K = P^-H^TS^{-1}$ 형태로 계산한다.
7. **상태 갱신:** $x_k^+ = x_k^- + K(z_k - h(x_k^-))$를 적용하고 yaw를 다시 정규화한다.
8. **공분산 갱신:** Joseph form으로 $P_k^+$를 갱신하고 마지막에 대칭화한다.

중요한 시간 정책은 다음과 같다.

- 측정 timestamp가 현재 EKF state time보다 미래이면, correction 전에 그 시각까지 prediction을 먼저 수행한다.
- 측정 timestamp가 이미 지나간 과거라면, 현재 코어에는 state history buffer가 없으므로 해당 보정을 적용하지 않는다.
- 카메라/ICP 보정은 nodelet에서 범퍼 기준 측정값을 후륜축 기준으로 변환하고, 지연 시간만큼 선도 보상한 뒤 현재 시각 기준 측정으로 `correctPose()`에 전달한다.

---

## 3. 공식과 현재 구현의 대응

이산시간 EKF 예측식은 다음과 같다.

$$
P_k^- = F_{k-1} P_{k-1}^+ F_{k-1}^T + L_{k-1} Q_{k-1} L_{k-1}^T
$$

$$
\hat{x}_k^- = f_{k-1}(\hat{x}_{k-1}^+, u_{k-1}, 0)
$$

현재 구현은 process noise가 상태 공간에 직접 더해지는 additive noise라고 두므로

$$
L_{k-1} = I
$$

로 단순화한다. 또한 `Q_`는 continuous-time PSD 성격의 행렬로 두고, prediction 구간 길이 $dt$를 곱해 이산 covariance로 근사한다.

$$
Q_{d,k} = Q_{dyn} \cdot dt
$$

따라서 코드의 공분산 예측은 다음 형태다.

$$
P_k^- = F_{k-1} P_{k-1}^+ F_{k-1}^T + Q_{dyn} \cdot dt
$$

측정 보정식은 교재의

$$
K_k =
P_k^- H_k^T
\left(H_k P_k^- H_k^T + M_k R_k M_k^T\right)^{-1}
$$

에 대응한다. 현재 pose, wheel, IMU 측정 모델은 모두 additive measurement noise이므로

$$
M_k = I
$$

로 단순화한다. 즉 코드에서는

$$
S_k = H_k P_k^- H_k^T + R_k
$$

를 만든다. 구현에서는 교재 공식과 바로 대응되도록 `S.inverse()` 형태로 Kalman gain을 계산한다.

---

## 4. 상태 예측 모델

### 4.1 Euler 기반 상태 전파

상태 예측은 후륜축 기준 Euler 1차 적분 모델을 사용한다. 한 prediction 구간 $dt$ 동안 yaw와 yaw rate는 일정하다고 보고, IMU 종방향 가속도 $a_x$로 종방향 속도 $V_x$를 먼저 예측한 뒤 평균 속도로 위치를 전파한다. 휠 속도 $v_{x,wheel}$은 예측 속도가 감속 중 물리적으로 반대 부호로 넘어가지 않도록 제한하는 방향 기준으로 사용한다.

$$
V_{x,k}^{pred} = V_{x,k-1} + a_x dt
$$

전진/정차 중인 경우 $v_{x,wheel} \ge -0.01$이면 $V_{x,k}^{pred}$가 음수가 되지 않도록 제한하고, 후진 중인 경우에는 양수가 되지 않도록 제한한다.

$$
\bar{V}_{x,k} = \frac{1}{2}\left(V_{x,k-1} + V_{x,k}^{pred}\right)
$$

$$
X_k =
X_{k-1}
+ \bar{V}_{x,k}\cos\theta_{k-1}\,dt
$$

$$
Y_k =
Y_{k-1}
+ \bar{V}_{x,k}\sin\theta_{k-1}\,dt
$$

$$
\theta_k = \theta_{k-1} + \omega_{k-1}dt
$$

$$
V_{x,k} = V_{x,k}^{pred}, \quad
\omega_k = \omega_{k-1}, \quad
b_{\omega,k} = b_{\omega,k-1}
$$

예측 후 yaw는 항상 다음처럼 정규화한다.

$$
\theta_k \leftarrow \operatorname{atan2}(\sin\theta_k, \cos\theta_k)
$$

### 4.2 시스템 자코비안 $F$

코드의 $F$는 위 Euler 모델을 상태 변수에 대해 선형화한 행렬이다.

$$
F = \frac{\partial f}{\partial x}
$$

Euler 모델의 자코비안은 다음과 같다.

$$
F =
\begin{bmatrix}
1 & 0 & -\bar{V}_x\sin\theta\,dt & \cos\theta\,dt & 0 & 0 \\
0 & 1 & \bar{V}_x\cos\theta\,dt & \sin\theta\,dt & 0 & 0 \\
0 & 0 & 1 & 0 & dt & 0 \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

코드의 $F$는 위치 전파에 평균 속도 $\bar{V}_x$를 사용한다. 단, $a_x$와 wheel sign clamp는 EKF 상태 벡터에 포함된 상태가 아니라 외부 입력이므로, 현재 구현의 자코비안은 상태 변수에 대한 주요 민감도만 반영한다.

위치는 시작 yaw 기준으로 1차 적분하므로 $X/Y$가 yaw rate에 직접 의존하는 항은 두지 않는다. yaw rate는 yaw 상태를 통해 다음 prediction step의 위치 전파에 간접적으로 반영된다.

### 4.3 공정 노이즈 $Q_{dyn}$

nodelet은 매 prediction 주기마다 차량 가속도와 yaw rate 크기를 보고 동적으로 $Q_{dyn}$을 만든 뒤 `setProcessNoise()`로 전달한다.

$$
Q_{dyn} =
\operatorname{diag}
\left(
\sigma_{pos}^2 s_{dyn},
\sigma_{pos}^2 s_{dyn},
(\sigma_{yaw}s_{yaw})^2,
(\sigma_{vx}s_{vel})^2,
(\sigma_\omega s_{yaw})^2,
\sigma_{bias}^2
\right)
$$

- **$\sigma_{pos}$:** 위치 예측 노이즈 표준편차 (`pos_std`, 기본 0.05 m)
- **$\sigma_{yaw}$:** 자세 예측 노이즈 표준편차 (`yaw_std`, 기본 0.03 rad)
- **$\sigma_{vx}$:** 속도 예측 노이즈 표준편차 (`vel_std`, 기본 0.15 m/s)
- **$\sigma_{\omega}$:** yaw rate 예측 노이즈 표준편차 (`yaw_rate_std`, 기본 0.01 rad/s)
- **$\sigma_{bias}$:** gyro bias 변화 노이즈 표준편차 (`bias_std`, 설정 파일 기본 0.01 rad/s)
- **$s_{vel}$:** 가감속이 클 때 속도 상태의 불확실성을 키우는 scale
- **$s_{yaw}$:** 선회가 클 때 yaw/yaw rate 불확실성을 키우는 scale
- **$s_{dyn}$:** 위치 불확실성에 적용하는 합성 scale

`EkfCore`는 단순한 공식 대응을 위해 `Q_dyn`의 PSD 검사를 별도로 수행하지 않는다. 대신 외부에서 아주 작은 비대칭이 섞여 들어온 경우 공분산 의미를 유지하도록 `Q = (Q + Q^T) / 2`로 대칭화한 뒤 사용한다.

---

## 5. 측정 모델

### 5.1 Pose 보정: `correctPose`

Pose 측정값은 후륜축 기준 위치와 yaw다.

$$
z_{pose} =
\begin{bmatrix}
X_{pose} & Y_{pose} & \theta_{pose}
\end{bmatrix}^T
$$

관측 모델은 다음과 같다.

$$
h_{pose}(x) =
\begin{bmatrix}
X & Y & \theta
\end{bmatrix}^T
$$

$$
H_{pose} =
\begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0
\end{bmatrix}
$$

ICP 정합 결과는 nodelet에서 다음 처리를 거친 뒤 core로 들어온다.

1. ICP 결과는 범퍼 기준 pose로 나온다.
2. `transformPose()`로 후륜축 기준 pose로 변환한다.
3. ICP covariance는 레버암 자코비안으로 후륜축 기준 covariance로 전파한다.
4. 카메라 정합 지연 시간에 따른 추가 불확실성 $Q_{latency}$를 더한다.
5. 짧은 지연 시간만큼 자전거 모델로 현재 시각까지 선도 보상한다.

후륜축 변환 covariance는 다음 형태다.

$$
R_{pose} = J R_{reg} J^T + Q_{latency}
$$

$$
J =
\begin{bmatrix}
1 & 0 & -x_{offset}\sin\theta \\
0 & 1 & x_{offset}\cos\theta \\
0 & 0 & 1
\end{bmatrix}
$$

`correctPose()` 내부에서는 yaw residual을 항상 wrap한다.

$$
y_\theta =
\operatorname{atan2}
\left(
\sin(\theta_{pose}-\theta^-),
\cos(\theta_{pose}-\theta^-)
\right)
$$

또한 pose update에는 실차 안정용 rate limiter가 있다. 이 limiter는 EKF 이론식의 일부가 아니므로, clipping이 발생하면 공분산을 Joseph form으로 줄이지 않고 prior covariance를 유지한다. 이렇게 하면 상태는 부드럽게 따라가되, 필터가 clipping된 측정을 완전히 반영했다고 과신하지 않는다.

### 5.2 Wheel 보정: `correctWheel`

휠 측정값은 종방향 속도와 좌우 휠 속도차 기반 yaw rate다.

$$
z_{wheel} =
\begin{bmatrix}
v_{x,wheel} & \omega_{wheel}
\end{bmatrix}^T
$$

관측 모델은 다음과 같다.

$$
h_{wheel}(x) =
\begin{bmatrix}
V_x & \omega
\end{bmatrix}^T
$$

$$
H_{wheel} =
\begin{bmatrix}
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0
\end{bmatrix}
$$

휠 관측은 gyro bias를 직접 보지 않는다. 따라서 IMU의 raw yaw rate와 휠 yaw rate가 함께 들어올 때, EKF는 `YAW_RATE`와 `B_YAW_RATE`를 분리할 근거를 얻는다.

휠 관측 covariance는 다음과 같다.

$$
R_{wheel} =
\begin{bmatrix}
\sigma_{wheel}^2 & 0 \\
0 & 2.0\frac{\sigma_{wheel}^2}{L_{track}^2}
\end{bmatrix}
$$

종방향 slip이 감지되면 nodelet이 $R_{wheel}(0,0)$을 키워 휠 속도 측정의 영향력을 낮춘다.

### 5.3 IMU 보정: `correctImu`

IMU 측정은 raw yaw rate 하나만 사용한다.

$$
z_{imu} = \omega_{raw}
$$

관측 모델은 gyro bias를 포함한다.

$$
h_{imu}(x) = \omega + b_\omega
$$

$$
H_{imu} =
\begin{bmatrix}
0 & 0 & 0 & 0 & 1 & 1
\end{bmatrix}
$$

IMU 단독으로는 $\omega$와 $b_\omega$를 분리할 수 없다. bias 추정은 wheel yaw rate, pose yaw 변화, 정지 상태 pseudo measurement 같은 추가 정보가 함께 있을 때 안정된다.

---

## 6. Innovation Covariance와 Kalman Gain

각 센서 update는 먼저 innovation covariance를 만든다.

$$
S_k = H_k P_k^- H_k^T + R_k
$$

innovation residual은 다음과 같다.

$$
y_k = z_k - h_k(x_k^-)
$$

`EkfCore`는 별도의 통계적 outlier gate나 pose/wheel 측정의 $S_k$ 분해 검사를 두지 않는다. 입력 측정 공분산 $R_k$가 정상적으로 설정된다는 전제에서 교재식 그대로 update를 수행한다.

Kalman gain은 수식상 다음과 같다.

$$
K_k = P_k^- H_k^T S_k^{-1}
$$

코드도 같은 형태를 사용한다.

```cpp
K = P * H.transpose() * S.inverse();
```

pose update에서는 $S$가 $3 \times 3$, wheel update에서는 $2 \times 2$라 계산량은 작다. 단, `S.inverse()`는 입력 covariance가 비정상적이면 결과가 불안정해질 수 있으므로, $R$ 튜닝과 입력 covariance 관리가 중요하다.

---

## 7. 상태와 공분산 보정

상태 보정은 다음과 같다.

$$
x_k^+ = x_k^- + K_k y_k
$$

보정 후 yaw는 반드시 정규화한다.

$$
\theta_k^+ \leftarrow \operatorname{atan2}(\sin\theta_k^+, \cos\theta_k^+)
$$

공분산은 Joseph form을 사용한다.

$$
P_k^+
=
(I - K_k H_k)P_k^-(I - K_k H_k)^T
+ K_k R_k K_k^T
$$

Joseph form은 단순식

$$
P_k^+ = (I - K_k H_k) P_k^-
$$

보다 수치적 대칭성과 양의 정부호성을 더 잘 보존한다. 다만 잘못된 $Q/R$, 잘못된 timestamp, 비정상 측정값까지 완전히 해결하는 것은 아니므로, 코드에서는 최소한의 방어 로직만 함께 둔다.

- IMU scalar update에서 `R_gyro`와 scalar $S$가 0에 너무 가까우면 update 거부
- 과거 timestamp 측정은 state buffer가 없으므로 update 거부
- 보정 후 yaw 정규화
- 보정 후 covariance 대칭화

---

## 8. Timestamp와 비동기 센서 처리 한계

현재 코어는 correction 함수의 timestamp를 실제로 사용한다. 측정 시각이 현재 상태보다 미래이면 correction 전에 그 시각까지 prediction을 수행한다.

하지만 state history buffer는 아직 없다. 따라서 이미 지나간 과거 측정(out-of-sequence measurement)을 받아 과거 상태를 고치고 현재까지 재전파하는 기능은 지원하지 않는다.

실전 비동기 fusion에서 더 높은 정확도가 필요하면 다음 구조가 추가로 필요하다.

- 센서 메시지를 timestamp 기준 queue로 정렬
- 상태 history buffer 저장
- 과거 상태 update 후 현재 시각까지 재전파
- 센서별 latency 모델과 covariance inflation 정교화

현재 카메라 보정은 nodelet에서 현재 시각 기준으로 선도 보상한 뒤 core에 전달하는 방식으로 이 문제를 완화한다.

---

## 9. 평가 지표

### 9.1 위치 및 자세 RMSE

GT 후륜축 pose와 EKF 후륜축 상태를 비교한다.

$$
\operatorname{RMSE}_{pos}
=
\sqrt{
\frac{1}{N}
\sum_{k=1}^{N}
\left[
(X_{GT,k}-X_k)^2 + (Y_{GT,k}-Y_k)^2
\right]
}
$$

$$
\operatorname{RMSE}_{yaw}
=
\sqrt{
\frac{1}{N}
\sum_{k=1}^{N}
e_{\theta,k}^2
}
$$

$$
e_{\theta,k}
=
\operatorname{atan2}
\left(
\sin(\theta_{GT,k}-\theta_k),
\cos(\theta_{GT,k}-\theta_k)
\right)
$$

### 9.2 NEES

NEES는 실제 pose error가 EKF covariance와 통계적으로 잘 맞는지 보는 지표다.

$$
\epsilon_k = e_k^T P_{pose,k}^{-1} e_k
$$

$$
e_k =
\begin{bmatrix}
X_{GT,k}-X_k &
Y_{GT,k}-Y_k &
\operatorname{WrapToPi}(\theta_{GT,k}-\theta_k)
\end{bmatrix}^T
$$

3차원 pose error 기준이므로 평균 NEES는 이상적으로 자유도 3 근처에서 움직인다.

- **NEES가 3보다 매우 큼:** 실제 오차에 비해 $P$가 너무 작아 필터가 과신 중일 가능성
- **NEES가 3보다 매우 작음:** 실제 오차에 비해 $P$가 너무 커 필터가 지나치게 보수적일 가능성

### 9.3 지표 추출 도구

`evaluate_ekf.py`는 저장된 rosbag 또는 실시간 ROS topic에서 Position RMSE, Yaw RMSE, NEES를 계산한다. CSV는 기본적으로 `src/carmaker_localization/data/csv/` 아래 timestamp 파일명으로 저장되며, 기존 파일이 있으면 `_001`, `_002` suffix를 붙여 덮어쓰지 않는다. 같은 경로를 강제로 덮어쓰려면 `--overwrite`를 사용한다.

Bag 후처리:

```bash
rosrun carmaker_localization evaluate_ekf.py bag ekf_eval.bag \
  --summary-output
```

실시간 누적 평가:

```bash
rosrun carmaker_localization evaluate_ekf.py live --csv
```

또는 launch 파일로 실행할 수 있다.

```bash
roslaunch carmaker_localization ekf_metrics_live.launch
```

기본 입력 topic은 `/carmaker/dynamic_info`와 `/localization/odom`이며, GT와 EKF odom은 기본 `0.05 s` 이내의 가장 가까운 timestamp끼리 매칭한다. 실시간 모드는 `/localization/eval/position_rmse`, `/localization/eval/yaw_rmse_deg`, `/localization/eval/mean_nees` 등의 누적 지표 topic도 함께 발행한다.

---

## 10. 현재 모델의 남은 한계

현재 구현은 실시간성과 단순성을 우선한 6차원 EKF다. 다음 개선은 별도 단계로 검토할 수 있다.

- acceleration noise, yaw acceleration noise, gyro bias random walk를 명시한 $G Q_c G^T$ 형태의 process noise 모델
- out-of-sequence measurement를 위한 state buffer
- 정지 감지 시 $V_x=0$, $\omega=0$ pseudo measurement
- IMU bias 초기 calibration
- 센서 extrinsic 오차까지 포함한 covariance propagation
- 더 큰 비선형성이 있는 환경에서 UKF 검토
