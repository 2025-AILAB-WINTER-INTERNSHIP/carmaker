## 1. 상태 벡터 (State Vector)

EKF는 시스템 오차 축소, 수치적 강건성 및 저속 환경에서의 횡방향 슬립 차단을 위해 다음과 같은 **6차원 상태 벡터**를 사용

$$
x = \begin{bmatrix} X & Y & \theta & V_x & \omega & b_\omega \end{bmatrix}^T
$$

- **$X, Y$:** 글로벌 위치 (Global Frame, `Fr0` 기준) - 차량 후륜축(Rear Axle) 중심점 기준
- **$\theta$:** 글로벌 요각 (Heading, $YAW$)
- **$V_x$:** 차량 로컬 종방향 속도 (Longitudinal Velocity, 후륜축 기준)
- **$\omega$:** 차량 로컬 요레이트 (Yaw Rate)
- **$b_\omega$:** IMU 자이로 센서 바이어스 (Gyro Bias)

---

## 2. 상태 예측 모델 (State Prediction Model)

### 2.1 이산 시간 시스템 방정식 (State Propagation)

상태 예측은 차량 후륜축 중심 기준 기구학 등속/등요레이트 모델(기구학 모델)을 사용하여 시간 간격 $dt$ 마다 다음과 같이 비선형 적분식으로 전파:

$$
x_k = f(x_{k-1}) = \begin{bmatrix} X_{k-1} + V_{x, k-1} \cos\theta_{k-1} \cdot dt \\ Y_{k-1} + V_{x, k-1} \sin\theta_{k-1} \cdot dt \\ \theta_{k-1} + \omega_{k-1} \cdot dt \\ V_{x, k-1} \\ \omega_{k-1} \\ b_{\omega, k-1} \end{bmatrix}
$$

- 요각 예측치 $\theta_k$는 물리법칙에 맞추어 상시 $[-\pi, \pi]$ 범위로 정규화(Normalization)

### 2.2 시스템 자코비안 행렬 $F$ (6x6)

시스템 천이 모델식 $f(x)$를 각 상태 변수로 편미분한 자코비안 행렬 $F = \frac{\partial f(x)}{\partial x}$:

$$
F = \begin{bmatrix}
1 & 0 & -V_x \sin\theta \cdot dt & \cos\theta \cdot dt & 0 & 0 \\
0 & 1 & V_x \cos\theta \cdot dt & \sin\theta \cdot dt & 0 & 0 \\
0 & 0 & 1 & 0 & dt & 0 \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

### 2.3 시스템 공정 노이즈 공분산 행렬 $Q$ (6x6)

시스템 상태 전파에 따른 노이즈의 강도를 결정하는 공정 노이즈 행렬 $Q$는 대각 행렬로 정의되며, 실제 코드 상에서 파라미터를 반영하여 다음과 같이 적용:

$$
Q = \text{diag}\left(\sigma_{pos}^2, \sigma_{pos}^2, \sigma_{yaw}^2, \sigma_{vx}^2, \sigma_{gyro}^2, \sigma_{bias}^2\right)
$$

- **$\sigma_{pos}^2$:** 위치 예측 노이즈 분산 ($1\text{e-}4$)
- **$\sigma_{yaw}^2$:** 자세 예측 노이즈 분산 ($1\text{e-}4$)
- **$\sigma_{vx}^2$:** 속도 예측 노이즈 분산 (속도 인코더 신뢰도 기반: $\text{wheel\_speed\_std}^2 = 0.0025$)
- **$\sigma_{gyro}^2$:** 요레이트 예측 노이즈 분산 (IMU 사양 기반: $\text{imu\_gyro\_std}^2 = 0.0001$)
- **$\sigma_{bias}^2$:** 자이로 바이어스 변화율 노이즈 분산 ($1\text{e-}6$)

상태 예측에 따른 오차 공분산 전파 수식은 다음과 같으며, 연산 후 수치적 대칭성을 보장:

$$
P_{k|k-1} = F P_{k-1} F^T + Q \cdot dt
$$

- 공정 노이즈 행렬 $Q$는 연속 시간(Continuous-time)의 파워 스펙트럼 밀도(PSD) 행렬이며, 오차 공분산 전파 시 $dt$를 곱하여 이산 시간(Discrete-time) 노이즈 공분산으로 **오일러 1차 근사(Euler 1st-order Integration)**하여 적용

$$
P_{k|k-1} = \frac{P_{k|k-1} + P_{k|k-1}^T}{2}
$$

---

## 3. 센서 관측 모델 (Sensor Measurement Model)

### 3.1 센서 1: 비전 ICP 위치/자세 보정 (`correctPose`)

- **측정값 벡터 $z_{pose}$ (3x1):**

    $$
    z_{pose} = \begin{bmatrix} X_{pose} & Y_{pose} & \theta_{pose} \end{bmatrix}^T
    $$

    (비전 ICP 결과는 후륜축 변환 및 카메라 정합 지연 시간 $dt$만큼 자전거 기구학 모델 전방 적분으로 보상된 후 입력)

- **비선형 관측 모델식 $h_{pose}(x)$:**

    $$
    h_{pose}(x) = \begin{bmatrix} X & Y & \theta \end{bmatrix}^T
    $$

- **관측 자코비안 행렬 $H_{pose}$ (3x6):**

    $$
    H_{pose} = \begin{bmatrix}
    1 & 0 & 0 & 0 & 0 & 0 \\
    0 & 1 & 0 & 0 & 0 & 0 \\
    0 & 0 & 1 & 0 & 0 & 0
    \end{bmatrix}
    $$

- **관측 노이즈 공분산 행렬 $R_{pose}$ (3x3):**
정합 매칭 공분산 $R_{reg}$을 후륜축으로 변환(J 자코비안)하고, 지연 시간에 비례한 공분산 인플레이션 항 $Q_{latency}$를 합산:

    $$
    R_{pose} = J R_{reg} J^T + Q_{latency}
    $$

    $$
    J = \begin{bmatrix} 1 & 0 & -x_{offset} \sin\theta_{pose} \\ 0 & 1 & x_{offset} \cos\theta_{pose} \\ 0 & 0 & 1 \end{bmatrix}
    $$

- 오프셋  마이너스 부호($-x_{offset}$): ICP 정합 측정 기준점(후방 범퍼)에서 차량의 고유 회전 중심인 **후륜축(Rear Axle)으로 좌표계 기준을 역변환 및 전파**하는 물리적 관계가 반영

### 3.2 센서 2: 휠 오도메트리 속도/요레이트 보정 (`correctWheel`)

- **측정값 벡터 $z_{wheel}$ (2x1):**

    $$
    z_{wheel} = \begin{bmatrix} v_{x, wheel} & \omega_{wheel} \end{bmatrix}^T
    $$

    - 종방향 평균 휠속도 및 좌우 속도차 기반 요레이트 성분
- **관측 모델식 $h_{wheel}(x)$:**

    $$
    h_{wheel}(x) = \begin{bmatrix} V_x & \omega \end{bmatrix}^T
    $$

- **관측 자코비안 행렬 $H_{wheel}$ (2x6):**

    $$
    H_{wheel} = \begin{bmatrix}
    0 & 0 & 0 & 1 & 0 & 0 \\
    0 & 0 & 0 & 0 & 1 & 0
    \end{bmatrix}
    $$

- **관측 노이즈 공분산 행렬 $R_{wheel}$ (2x2):**

    $$
    R_{wheel} = \begin{bmatrix}
    \sigma_{vx}^2 & 0 \\
    0 & 2.0\frac{\sigma_{vx}^2}{L_{track}^2}
    \end{bmatrix}
    $$

    (종방향 바퀴 슬립 감지 시 $R_{wheel}(0,0)$ 원소값은 100배 팽창됨)


### 3.3 센서 3: IMU 요레이트 보정 (`correctImu`)

- **측정값 벡터 $z_{imu}$ (1x1):**

    $$
    z_{imu} = \begin{bmatrix} \omega_{raw} \end{bmatrix}
    $$

    - (저속 주행 환경에 무의미하고 노이즈가 큰 가속도 센서 보정을 차단하여 모델 무결성 확보)
- **관측 모델식 $h_{imu}(x)$:**

    자이로 실측값에 EKF 내부에서 실시간 추정하는 바이어스 오차 성분을 결합하여 모델링:

    $$
    h_{imu}(x) = \omega + b_\omega
    $$

- **관측 자코비안 행렬 $H_{imu}$ (1x6):**

    $$
    H_{imu} = \begin{bmatrix} 0 & 0 & 0 & 0 & 1 & 1 \end{bmatrix}
    $$

- **관측 노이즈 공분산 행렬 $R_{imu}$ (1x1):**

    $$
    R_{imu} = \begin{bmatrix} \sigma_{gyro}^2 \end{bmatrix}
    $$

    ($\sigma_{gyro} = 0.01 \text{ rad/s}$)


---

## 4. 공분산 업데이트 (Covariance Update)

오차 공분산 행렬 $P$의 대칭성(Symmetry) 및 양의 정정치(Positive-Definite) 수치 안정성을 엄밀하게 강제하여 필터 발산을 완벽히 억제하기 위해, 모든 센서 보정 업데이트 단계에서 **Joseph Form** 공식을 이용해 오차 공분산을 갱신:

$$
P_k = (I - K_k H_k) P_{k|k-1} (I - K_k H_k)^T + K_k R_k K_k^T
$$

- **$K_k$:** 칼만 이득 (Kalman Gain) 행렬 [6xM]

    $$
    K_k = P_{k|k-1} H_k^T (H_k P_{k|k-1} H_k^T + R_k)^{-1}
    $$

- **$H_k$:** 업데이트 단계별 관측 자코비안 행렬 ($H_{pose}$ [3x6], $H_{wheel}$ [2x6], $H_{imu}$ [1x6])
- **$R_k$:** 업데이트 단계별 관측 노이즈 공분산 행렬 ($R_{pose}$ [3x3], $R_{wheel}$ [2x2], $R_{imu}$ [1x1])
- **$I$:** 6차원 단위 행렬 (Identity Matrix, [6x6])

---

## 5. 평가

- 위치 추정 오차
    - 주행 간 Ego의 GT 위치와 추정 위치 간의 누적 RMSE 측정
- 자세 추정 오차
    - 주행 간 Ego의 GT 자세와 추정 자세 간의 누적 RMSE 측정
- NESS
    -
- 충전 패드 정렬도
    - 주차 후 충전 패드와의 위치 및 각도의 RMSE 측정

---

## 6. Appendix

1. CA 및 다른 물리 모델 대신 키네마틱 사용 이유
2. 충전패드 랜드마크의 면 정합 → 선 정합 이유
