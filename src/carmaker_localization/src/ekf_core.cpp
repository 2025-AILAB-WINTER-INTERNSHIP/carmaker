#include "carmaker_localization/ekf_core.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace carmaker_localization {

EkfCore::EkfCore()
    : is_initialized_(false), last_time_(0.0), wheelbase_(2.97), rear_axle_offset_(0.82) {
    x_ = Eigen::VectorXd::Zero(STATE_DIM);
    P_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 1.0;
    Q_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.01;

    // 위치, 속도, 가속도, 요각 및 편향 상태 전파에 따른 공정 노이즈 튜닝
    Q_.block<2, 2>(X, X) *= 0.01;         // 위치 상태는 매우 부드럽게 유지되도록 설정
    Q_.block<2, 2>(VX, VX) *= 0.1;        // 속도는 완만하게 변화하도록 설정
    Q_.block<2, 2>(AX, AX) *= 1.0;        // 가속도는 급격한 변화를 감안해 크게 설정
    Q_(YAW, YAW) *= 0.01;
    Q_(YAW_RATE, YAW_RATE) *= 0.1;
    Q_.block<3, 3>(B_AX, B_AX) *= 0.0001; // 센서 바이어스는 아주 느리게 변화하도록 설정
}

void EkfCore::initialize(double x, double y, double yaw, double timestamp, double vx, double vy) {
    std::lock_guard<std::mutex> lock(mutex_);
    x_.setZero();
    x_(X) = x;
    x_(Y) = y;
    x_(YAW) = yaw;
    x_(VX) = vx;
    x_(VY) = vy;

    P_.setIdentity();
    P_.block<2, 2>(X, X) *= 0.1;
    P_(YAW, YAW) *= 0.05;

    last_time_ = timestamp;
    is_initialized_ = true;
}

void EkfCore::setProcessNoise(const Eigen::MatrixXd& Q) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (Q.rows() == STATE_DIM && Q.cols() == STATE_DIM) {
        Q_ = Q;
    }
}

void EkfCore::setWheelbase(double wheelbase) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (wheelbase > 0.0) {
        wheelbase_ = wheelbase;
        std::cout << "[EkfCore] Wheelbase set to: " << wheelbase_ << " m" << std::endl;
    }
}

void EkfCore::setRearAxleOffset(double offset) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (offset >= 0.0) {
        rear_axle_offset_ = offset;
        std::cout << "[EkfCore] Rear axle offset set to: " << rear_axle_offset_ << " m" << std::endl;
    }
}

void EkfCore::prediction(double timestamp, const PredictionInput& u) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    double dt = timestamp - last_time_;

    // 시간 도약 감지 (전방 1초 초과 또는 후방 시간 도약 시 필터 상태 초기화 방지용 분기)
    if (dt > 1.0 || dt < -0.05) {
        handleTimeJump(timestamp);
        return;
    }

    if (dt <= 1e-4) return; // 너무 작은 시간 간격은 연산 생략

    // --- 후륜 축 기준 기구학 자전거 모델 기반 예측 (상태는 차량 원점/범퍼 기준 전파) ---
    double vx = x_(VX);
    double yaw = x_(YAW);
    double ax = x_(AX);
    double ay = x_(AY);
    double delta = u.steering_angle;

    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    // 조향각을 승용차 물리적 한계로 제한 (약 +/-40도 = 0.7 rad)
    double delta_clamped = std::max(-0.7, std::min(0.7, delta));
    double tan_delta = std::tan(delta_clamped);
    
    // 후륜 축 기준 회전 각속도 계산 (v/L * tan(delta))
    double yaw_rate_bicycle = vx * tan_delta / wheelbase_;
    // 후륜 축 기준 원심 가속도 계산 (v * omega)
    double ay_bicycle = vx * yaw_rate_bicycle;
    
    // 상태 벡터를 차량 원점(Fr1A, 후방 범퍼)으로 일치화하되,
    // 뒷바퀴 축 오프셋(rear_axle_offset_)을 사용한 비홀로노믹 제약(NHC)을 투영하여 물리 연산 수행
    // 범퍼 기준 횡속도: vy = -yaw_rate * rear_axle_offset_
    double vy = -yaw_rate_bicycle * rear_axle_offset_;

    // 상태 전파 (위치 상태에 대한 2차 적분 적용)
    x_(X) += (vx * cos_yaw - vy * sin_yaw) * dt + 0.5 * (ax * cos_yaw - ay * sin_yaw) * dt * dt;
    x_(Y) += (vx * sin_yaw + vy * cos_yaw) * dt + 0.5 * (ax * sin_yaw + ay * cos_yaw) * dt * dt;
    x_(VX) += ax * dt;
    x_(VY) = vy; // 범퍼 기준 횡속도 업데이트
    x_(AX) = ax; // 등가속도 종방향 전파
    x_(AY) = ay_bicycle; // 뒷바퀴 축 원심가속도로 가속도 상태 제한
    x_(YAW) += yaw_rate_bicycle * dt;
    x_(YAW_RATE) = yaw_rate_bicycle;

    // 자코비안 F 행렬 [11x11]
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);

    // 위치 상태 변수에 대한 미분
    // 비홀로노믹 제약으로 인해 vy가 vx에 종속됨에 따른 편미분 계수 C 반영
    // vy = -vx * (tan_delta / L) * d
    double C = (tan_delta / wheelbase_) * rear_axle_offset_;
    F(X, VX)  = (cos_yaw + C * sin_yaw) * dt;
    F(X, VY)  = -sin_yaw * dt;
    F(X, AX)  = 0.5 * cos_yaw * dt * dt;
    F(X, AY)  = -0.5 * sin_yaw * dt * dt;
    F(X, YAW) = (-vx * sin_yaw - vy * cos_yaw) * dt - 0.5 * (ax * sin_yaw + ay * cos_yaw) * dt * dt;

    F(Y, VX)  = (sin_yaw - C * cos_yaw) * dt;
    F(Y, VY)  = cos_yaw * dt;
    F(Y, AX)  = 0.5 * sin_yaw * dt * dt;
    F(Y, AY)  = 0.5 * cos_yaw * dt * dt;
    F(Y, YAW) = (vx * cos_yaw - vy * sin_yaw) * dt + 0.5 * (ax * cos_yaw - ay * sin_yaw) * dt * dt;

    // 속도 상태 변수에 대한 미분
    F(VX, AX) = dt;

    // 제약된 상태 변수에 대한 항등 행렬 대각 성분 오버라이드 (매 스텝 예측치로 구속되므로 이전 상태 미분값 0)
    F(VY, VY) = 0.0;
    F(AY, AY) = 0.0;
    F(YAW_RATE, YAW_RATE) = 0.0;

    // 기구학적 제약 조건에 따른 속도 관련 미분 (d(VY)/d(VX), d(AY)/d(VX) 등)
    F(VY, VX) = -tan_delta / wheelbase_ * rear_axle_offset_;
    F(AY, VX) = 2.0 * vx * tan_delta / wheelbase_;
    F(YAW, VX) = tan_delta / wheelbase_ * dt;
    F(YAW_RATE, VX) = tan_delta / wheelbase_;

    // 요각 정규화 (모델 독립적)
    while (x_(YAW) > M_PI) x_(YAW) -= 2.0 * M_PI;
    while (x_(YAW) < -M_PI) x_(YAW) += 2.0 * M_PI;

    // --- 오차 공분산 전파 ---
    P_ = F * P_ * F.transpose() + Q_ * dt;

    // 수치적 대칭성 보장
    P_ = (P_ + P_.transpose()) * 0.5;

    last_time_ = timestamp;
}

void EkfCore::correctPose(double x, double y, double yaw, const Eigen::Matrix3d& R, double timestamp) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    // 주의: 상태 버퍼를 통한 과거 시점 소급 보정은 의도적으로 제외
    // CarMaker 시뮬레이션 환경에서는 비전 정합 지연이 100ms 이내로 매우 낮아,
    // 과거 포즈를 역추적하는 것보다 현재 시점 상태에 직접 보정하는 것이 주행 시 안정성에 더 유리
    // 지연 시간이 200ms를 초과하게 될 경우에만 소급 보정 버퍼의 재활성화 검토

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, STATE_DIM);
    H(0, X) = 1.0; H(1, Y) = 1.0; H(2, YAW) = 1.0;

    Eigen::Vector3d z(x, y, yaw);
    Eigen::Vector3d h(x_(X), x_(Y), x_(YAW));

    // 잔차 계산 및 요각 범위 제한
    Eigen::Vector3d y_res = z - h;
    while (y_res(2) > M_PI) y_res(2) -= 2.0 * M_PI;
    while (y_res(2) < -M_PI) y_res(2) += 2.0 * M_PI;

    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ += K * y_res;

    // 수치적 안정성을 위한 Joseph Form 공분산 업데이트: P = (I - KH) * P * (I - KH)^T + K * R * K^T
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

void EkfCore::correctWheel(double vx, double vy, double yaw_rate, const Eigen::Matrix3d& R, double timestamp) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, STATE_DIM);
    H(0, VX) = 1.0; H(1, VY) = 1.0; H(2, YAW_RATE) = 1.0;

    Eigen::Vector3d z(vx, vy, yaw_rate);
    Eigen::Vector3d h(x_(VX), x_(VY), x_(YAW_RATE));

    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ += K * (z - h);

    // 수치적 안정성을 위한 Joseph Form 공분산 업데이트: P = (I - KH) * P * (I - KH)^T + K * R * K^T
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

void EkfCore::correctImu(double ax_raw, double ay_raw, double yaw_rate_raw, const Eigen::Matrix3d& R, double timestamp) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    // 관측 모델: z_raw = 상태 + 바이어스
    // 즉, h(x) = [ax + b_ax, ay + b_ay, yaw_rate + b_yaw_rate]
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, STATE_DIM);
    H(0, AX) = 1.0; H(0, B_AX) = 1.0;
    H(1, AY) = 1.0; H(1, B_AY) = 1.0;
    H(2, YAW_RATE) = 1.0; H(2, B_YAW_RATE) = 1.0;

    Eigen::Vector3d z(ax_raw, ay_raw, yaw_rate_raw);
    Eigen::Vector3d h(x_(AX) + x_(B_AX), x_(AY) + x_(B_AY), x_(YAW_RATE) + x_(B_YAW_RATE));

    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ += K * (z - h);

    // 수치적 안정성을 위한 Joseph Form 공분산 업데이트: P = (I - KH) * P * (I - KH)^T + K * R * K^T
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

StateFrame EkfCore::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {last_time_, x_, P_};
}

void EkfCore::handleTimeJump(double timestamp) {
    std::cout << "\033[1;33m[EkfCore] Time jump detected (dt: " << (timestamp - last_time_) << "s). Resetting filter state.\033[0m" << std::endl;

    last_time_ = timestamp;

    // 재수렴을 허용하기 위해 공분산을 초기 상태로 리셋
    P_.setIdentity();
    P_.block<2, 2>(X, X) *= 0.5;
    P_(YAW, YAW) *= 0.1;

    // 타임점프 후 현재 상태는 유지하되 속도와 바이어스는 클리어
    x_(VX) = 0.0; x_(VY) = 0.0;
    x_(AX) = 0.0; x_(AY) = 0.0;
    x_(YAW_RATE) = 0.0;
    // IMU 바이어스 리셋: 시간 점프(백 루프 / 시뮬레이션 재시작)는 이전 바이어스 추정치를 무효화하므로 처음부터 다시 시작
    x_(B_AX) = 0.0; x_(B_AY) = 0.0; x_(B_YAW_RATE) = 0.0;
}

} // namespace carmaker_localization
