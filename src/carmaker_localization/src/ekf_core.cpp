#include "carmaker_localization/ekf_core.h"
#include <cmath>
#include <algorithm>
#include <sstream>

namespace carmaker_localization {

EkfCore::EkfCore()
    : is_initialized_(false), last_time_(0.0) {
    x_.setZero();
    P_.setIdentity();
    Q_.setZero();

    // 위치, 자세, 속도, 요레이트 및 바이어스 공정 노이즈 튜닝
    Q_(X, X) = 1e-4;
    Q_(Y, Y) = 1e-4;
    Q_(YAW, YAW) = 1e-4;
    Q_(VX, VX) = 1e-4;
    Q_(YAW_RATE, YAW_RATE) = 1e-4;
    Q_(B_YAW_RATE, B_YAW_RATE) = 1e-4;
}

void EkfCore::initialize(double x, double y, double yaw, double timestamp, double vx) {
    std::lock_guard<std::mutex> lock(mutex_);
    x_.setZero();
    x_(X) = x;
    x_(Y) = y;
    x_(YAW) = yaw;
    x_(VX) = vx;

    P_.setIdentity();
    P_.block<2, 2>(X, X) *= 0.1;
    P_(YAW, YAW) *= 0.05;

    last_time_ = timestamp;
    is_initialized_ = true;
}

void EkfCore::setProcessNoise(const StateMatrix& Q) {
    std::lock_guard<std::mutex> lock(mutex_);
    Q_ = Q;
}

void EkfCore::setLogCallbacks(std::function<void(const std::string&)> info,
                               std::function<void(const std::string&)> warn) {
    std::lock_guard<std::mutex> lock(mutex_);
    log_info_ = std::move(info);
    log_warn_ = std::move(warn);
}

void EkfCore::prediction(double timestamp) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    double dt = timestamp - last_time_;

    // 시간 도약 감지 (전방 1초 초과 또는 후방 시간 도약 시 필터 상태 초기화 방지용 분기)
    if (dt > 1.0 || dt < -0.05) {
        handleTimeJump(timestamp);
        return;
    }

    if (dt <= 1e-4) return; // 너무 작은 시간 간격은 연산 생략

    double vx = x_(VX);
    double yaw = x_(YAW);
    double yaw_rate = x_(YAW_RATE);

    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    // 6차원 상태 기반 등속/등요레이트(YAW_RATE) 오도메트리 전파
    x_(X) += vx * cos_yaw * dt;
    x_(Y) += vx * sin_yaw * dt;
    x_(YAW) += yaw_rate * dt;
    // vx, yaw_rate, b_yaw_rate는 등속/등요레이트(YAW_RATE) 예측 모델에 따라 유지됨

    // 자코비안 F 행렬 [6x6]
    StateMatrix F = StateMatrix::Identity();
    F(X, YAW) = -vx * sin_yaw * dt;
    F(X, VX) = cos_yaw * dt;
    F(Y, YAW) = vx * cos_yaw * dt;
    F(Y, VX) = sin_yaw * dt;
    F(YAW, YAW_RATE) = dt;

    // 요각 정규화
    while (x_(YAW) > M_PI) x_(YAW) -= 2.0 * M_PI;
    while (x_(YAW) < -M_PI) x_(YAW) += 2.0 * M_PI;

    // --- 오차 공분산 전파 ---
    P_ = F * P_ * F.transpose() + Q_ * dt;

    // 수치적 대칭성 보장
    P_ = (P_ + P_.transpose()) * 0.5;

    last_time_ = timestamp;
}

void EkfCore::correctPose(double x, double y, double yaw, const Eigen::Matrix3d& R, double timestamp,
                          double max_pos_step, double max_yaw_step) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Matrix<double, 3, STATE_DIM> H = Eigen::Matrix<double, 3, STATE_DIM>::Zero();
    H(0, X) = 1.0; H(1, Y) = 1.0; H(2, YAW) = 1.0;

    Eigen::Vector3d z(x, y, yaw);
    Eigen::Vector3d h(x_(X), x_(Y), x_(YAW));

    // 잔차 계산 및 요각 범위 제한
    Eigen::Vector3d y_res = z - h;
    while (y_res(2) > M_PI) y_res(2) -= 2.0 * M_PI;
    while (y_res(2) < -M_PI) y_res(2) += 2.0 * M_PI;

    Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, STATE_DIM, 3> K = P_ * H.transpose() * S.inverse();

    StateVector dx_state = K * y_res;

    // 최대 단일 보정 한계치 설정을 통해 급격한 차량 튀어오름 방지
    double pos_step = dx_state.segment<2>(X).norm();
    if (pos_step > max_pos_step) {
        dx_state.segment<2>(X) *= (max_pos_step / pos_step);
    }
    double yaw_step = std::abs(dx_state(YAW));
    if (yaw_step > max_yaw_step) {
        dx_state(YAW) *= (max_yaw_step / yaw_step);
    }

    x_ += dx_state;

    // 수치적 안정성을 위한 Joseph Form 공분산 업데이트
    StateMatrix I_KH = StateMatrix::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

void EkfCore::correctWheel(double vx, double yaw_rate, const Eigen::Matrix2d& R, double timestamp) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    // 6D EKF: 종방향 속도(VX)와 요레이트(YAW_RATE)만 관측으로 업데이트
    Eigen::Matrix<double, 2, STATE_DIM> H = Eigen::Matrix<double, 2, STATE_DIM>::Zero();
    H(0, VX) = 1.0;
    H(1, YAW_RATE) = 1.0;

    Eigen::Vector2d z(vx, yaw_rate);
    Eigen::Vector2d h(x_(VX), x_(YAW_RATE));

    Eigen::Matrix2d S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, STATE_DIM, 2> K = P_ * H.transpose() * S.inverse();

    x_ += K * (z - h);

    // 수치적 안정성을 위한 Joseph Form 공분산 업데이트
    StateMatrix I_KH = StateMatrix::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

void EkfCore::correctImu(double yaw_rate_raw, double R_gyro, double timestamp) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    // 6D EKF 저속 환경 모델: 요레이트 자이로 관측(YAW_RATE + B_YAW_RATE)만 수행
    Eigen::Matrix<double, 1, STATE_DIM> H = Eigen::Matrix<double, 1, STATE_DIM>::Zero();
    H(0, YAW_RATE) = 1.0;
    H(0, B_YAW_RATE) = 1.0;

    double z = yaw_rate_raw;
    double h = x_(YAW_RATE) + x_(B_YAW_RATE);

    double S = (H * P_ * H.transpose())(0, 0) + R_gyro;
    Eigen::Matrix<double, STATE_DIM, 1> K = P_ * H.transpose() / S;

    x_ += K * (z - h);

    // 수치적 안정성을 위한 Joseph Form 공분산 업데이트
    StateMatrix I_KH = StateMatrix::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_gyro * K.transpose();
}

StateFrame EkfCore::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {last_time_, x_, P_};
}

void EkfCore::handleTimeJump(double timestamp) {
    if (log_warn_) {
        std::ostringstream oss;
        oss << "[EkfCore] Time jump detected (dt: " << (timestamp - last_time_) << "s). Resetting filter state.";
        log_warn_(oss.str());
    }

    last_time_ = timestamp;

    // 재수렴을 허용하기 위해 공분산을 초기 상태로 리셋
    P_.setIdentity();
    P_.block<2, 2>(X, X) *= 0.5;
    P_(YAW, YAW) *= 0.1;

    x_(VX) = 0.0;
    x_(YAW_RATE) = 0.0;
    x_(B_YAW_RATE) = 0.0;
}

} // namespace carmaker_localization
