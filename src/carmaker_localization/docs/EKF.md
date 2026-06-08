# EKF 위치 추정(Localization)

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
    

### 3.4 관측 및 공정 공분산 설계 사양 (Covariance Design Specification)

1. **시스템 공정 노이즈 공분산 $Q$ (6x6):**
    - 위치 및 요각 대각 성분: $1\text{e-}4$ (기구학 예측 연속성을 유지하기 위해 수치 안정성 확보)
    - 속도 $V_x$ 성분: $\sigma_{vx}^2 = 0.05^2 = 0.0025$ (가감속을 충분히 수종하도록 열어둠)
    - 요레이트 $\omega$ 성분: $\sigma_{gyro}^2 = 0.01^2 = 0.0001$ (IMU 자이로의 물리적 속성을 매핑)
    - 자이로 바이어스 $b_\omega$ 성분: $1\text{e-}4$ (바이어스 추적 및 수렴 속도를 위해 설정)
2. **휠 오도메트리 관측 노이즈 공분산 $R_{wheel}$ (2x2):**
    - 종방향 속도 분산 $R_{wheel}(0,0)$: $\sigma_{vx}^2 = 0.05^2 = 0.0025$
        - *종방향 바퀴 슬립 감지 시:* $|v_{wheel} - V_{x, state}| > 1.0\text{m/s}$ 일 때, 해당 분산을 100배 팽창시킴($\times 100.0$)으로써 EKF 휠 신뢰도를 일시 정지시키고 IMU/비전 정보로 위치를 제어
    - 요레이트 분산 $R_{wheel}(1,1)$: 차동 조향 오차 전파식 $2.0\frac{\sigma_{vx}^2}{L_{track}^2}$을 적용하여 좌우 휠 속도 오차를 트랙 너비의 제곱에 맞춰 물리적으로 엄밀하게 스케일링
3. **IMU 요레이트 관측 노이즈 공분산 $R_{imu}$ (1x1):**
    - 자이로 단일 성분 분산: $\sigma_{gyro}^2 = 0.01^2 = 0.0001$ (MEMS 자이로 원초 노이즈 수준 매핑)
4. **비전 ICP 관측 공분산 $R_{pose}$ (3x3):**
    - **특징점 개별 공분산 ($C_{obs}$):** 카메라 원근 왜곡을 모사하여 Sweet Spot(렌즈 광축 바닥 투영점)으로부터의 거리에 비례하여 방사 방향($r^2$에 비례) 및 접선 방향($r$에 비례)의 오차를 2D Cartesian 좌표로 투영해 계산
    - **포즈 정보 행렬(Hessian, $H$) 구성:** 개별 특징점 공분산의 역행렬을 가중치로 한 정합 쌍의 자코비안 $J_i$ 곱인 $H = \sum J_i^T W_i J_i$ 로 계산되어 특징점 공간 배치 분포(GDOP)를 직접 지표화
    - **Max Capping 보호 기법:** 특징점이 한 축 방향으로 결여된 직선도로/터널 환경에서 공분산이 무한히 튀는 수치 비특이성을 예방하기 위해 고유값 분해(Eigenvalue Decomposition)를 통해 최대 한계 분산값인 `max_covariance` = `10.0`으로 설정

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

## 5. 평가 (Evaluation Metrics)

### 5.1 위치 및 자세 추정 오차 (RMSE)

실주행 궤적(Ground Truth, GT) 대비 EKF 추정값의 전체 시간 평균 제곱근 오차(RMSE)를 실시간 및 사후 분석으로 측정

- **위치 추정 RMSE (Position RMSE, m):**
    
    $$
    \text{RMSE}{pos} = \sqrt{\frac{1}{N}\sum{k=1}^N \left((X_{GT,k} - X_k)^2 + (Y_{GT,k} - Y_k)^2\right)}
    $$
    
    - 실시간 진단 및 수렴 확인을 위해 `/localization/data/rmse_position` 토픽으로 instantaneous 위치 오차를 실시간 발행
- **자세 추정 RMSE (Yaw RMSE, deg):**
    
    $$
    \text{RMSE}{yaw} = \sqrt{\frac{1}{N}\sum{k=1}^N (e_{\theta, k})^2}
    $$
    
    - $e_{\theta, k} = \text{WrapToPi}(\theta_{GT, k} - \theta_k)$
    - 실시간 오차 판단을 위해 `/localization/data/rmse_orientation` 토픽으로 instantaneous 각도 편차를 실시간 발행

### 5.2 수치적 일관성 검증 (NEES, Normalized Estimation Error Squared)

필터가 계산한 오차 공분산 $P_k$가 실제 발생하는 추정 오차 범위를 투명하게 지탱하고 있는지 확인하기 위한 통계적 검증 지표

$$
\epsilon_k = e_k^T P_{pose, k}^{-1} e_k
$$

- $e_k = \begin{bmatrix} X_{GT,k} - X_k & Y_{GT,k} - Y_k & \text{WrapToPi}(\theta_{GT,k} - \theta_k) \end{bmatrix}^T$
- $P_{pose, k}$: EKF 오차 공분산 행렬 $P$에서 추출한 X, Y, Yaw 상태에 해당하는 3x3 부분 행렬
- **모니터링 방식:** 매 스텝마다 실시간 계산된 $\epsilon_k$를 `/localization/data/nees` 토픽으로 퍼블리시하여 Plotjuggler 등을 통해 공분산 정합성 튜닝 추이를 시각화하며, 진단(/diagnostics) 토픽 내 `Cumulative Average NEES` 및 `Latest Instantaneous NEES`로 누적 상태를 수집
- **판정 기준:** 3차원 포즈 오차이므로 평균 NEES가 자유도 수치인 **`3.0` 근방**에서 안정적으로 변동해야 이상적으로 튜닝된 상태
    - **NEES $\gg 3.0$ (**실제 오차($e$)에 비해 필터가 계산한 공분산 $P$가 너무 작은 경우): 필터가 자기 상태를 지나치게 신뢰하여 센서 관측을 무시하다가 발산 위험성 → 시스템 노이즈 $Q$ 증가 or 센서 노이즈 $R$ 감쇠
    - **NEES $\ll 3.0$ (실제 오차($e$)에 비해 필터가 계산한 공분산 $P$가 너무 큰 경우): 필터가 자기 상태를 불신하여 센서 잡음에 흔들림** → 시스템 노이즈 $Q$ 감쇠 or 센서 노이즈 $R$ 증가

### 5.3 충전 패드 정렬도 (Charging Pad Alignment Accuracy)

최종 목적지인 충전 패드 상에 주차를 완료한 시점에서, 차량의 고유 기준축과 충전 패드 중심점 간의 상대 위치/자세 정밀도를 측정

- **기준:** 자율 주차 완료 시점에서의 최종 정적 정밀도 RMSE 평가
    - 횡방향 주차 오차: **3cm 이하**
    - 요각 정렬 오차: **1도 이하**

---

## 6. Appendix

### 6.1 CA(Constant Acceleration) 및 동역학 모델 대신 저속 기구학 모델을 사용하는 이유

1. **횡슬립(Drift) 차단 및 강건성 극대화:**
    - 저속(0m/s ~ 2m/s) 주행 환경에서는 타이어 슬립각이 극도로 작기 때문에 횡방향 가속도($A_Y$)나 횡속도($V_Y$) 성분을 EKF 상태 변수에 넣고 추정할 경우, 센서 잡음으로 인해 상태 변수가 흔들리면서 차량이 게처럼 옆으로 기어가는 드리프트 현상이 발생
    - 등속/등요레이트 기구학 모델은 횡방향 구속 조건($V_Y = 0$, Non-Holonomic Constraint)을 물리적 상태 구조 단계에서 완벽히 강제함으로써 저속 및 정지 시 시스템 위치 추정의 강건성을 원천적으로 향상
2. **센서 드리프트 노이즈 누적 배제:**
    - CA 모델 또는 IMU의 선가속도계 적분 방식을 활용하면 센서 자체의 바이어스 잡음이 속도와 위치로 이중 누적(Double Integration)되어 무제한 드리프트가 발생.
    - 기구학 등속/등요레이트 모델을 적용하면 IMU는 가속도 수치를 차단하고 요레이트 자이로 센서만 바이어스 보정을 거쳐 깔끔하게 각도 적분에 사용하므로, 무의미한 중력 보정 계산이나 레버암 편심 미분 연산이 사라져 수치적 해석 안정성이 크게 향상
3. **경량화 및 연산량 대폭 절감:**
    - 12차원에서 6차원으로 상태 차원이 축소됨에 따라, 오차 공분산 예측을 위한 행렬 곱 연산($F P F^T$) 횟수가 획기적으로 줄어들어 임베디드 자율주행 보드나 실시간 제어 스텝(100Hz 이상)에서의 CPU 로드를 대폭 낮춰 시스템 지연을 최소화

### 6.2 충전패드 랜드마크의 면(Area) 정합 대신 선(Edge/Line) 정합을 사용하는 이유

1. **특징점 분포의 기하학적 명확성 확보 (DoP 향상):**
    - 충전 패드 영역 전체를 2D 면(Area) 데이터(바이너리 꽉 찬 마스크)로 인식하여 ICP를 돌리게 되면, 면 내부의 평평하고 넓은 픽셀 정보들은 차량 이동에 따른 기하학적 구속(Gradient)을 주지 못해 슬라이딩 오차 발생 가능
    - 충전 패드의 외각 경계선(Edge/Line) 정보만 모폴로지(Erosion) 연산으로 추출하여 정합할 경우, 특징점의 위치 정보가 경계선을 따라 명확한 1차원 선형 기울기를 형성하므로 정합 시 종/횡방향 및 헤딩각 방향에 대한 수학적 구속력이 극대화되어 수렴 속도와 정합 정밀도가 현격하게 상승
2. **특징점 매칭 개수의 축소와 연산 부하 절감:**
    - 면 내부의 무수한 점들을 KD-Tree 탐색 및 Mahalanobis 평가에 투입하는 것 대비, 경계선 픽셀만 필터링하여 정합을 시도하면 처리해야 할 특징점 수(Point Cloud size)가 감소
    - 이는 ICP 루프당 최근접 이웃 탐색(Nearest Neighbor Search) 연산량을 급감시켜 실시간 위치 제어가 가능하도록 병목 현상을 해결
3. **노면 조도 및 그림자 강건성 확보:**
    - 실제 조명이나 가려짐(오클루전) 조건 하에서 면 전체의 이진화 임계값은 주변 밝기에 따라 찌그러지기 쉽지만, 경계선(Edge)의 경우 필터링 연산을 통해 비교적 강건하게 살아남으므로 실차 환경에서의 조도 강건성 향상
