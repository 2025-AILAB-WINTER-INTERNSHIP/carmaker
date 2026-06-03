#include "carmaker_localization/ekf_core.h"
#include <cmath>
#include <algorithm>
#include <sstream>

namespace carmaker_localization {

EkfCore::EkfCore()
    : is_initialized_(false), last_time_(0.0), wheelbase_(2.97), rear_axle_offset_(0.82), imu_offset_x_(0.0), imu_offset_y_(0.0) {
    x_.setZero();
    P_.setIdentity();
    Q_.setIdentity();
    Q_ *= 0.01;

    // 위치, 속도, 가속도, 요각 및 편향 상태 전파에 따른 공정 노이즈 튜닝
    Q_.block<2, 2>(X, X) *= 0.01;         // 위치 상태는 매우 부드럽게 유지되도록 설정
    Q_.block<2, 2>(VX, VX) *= 0.1;        // 속도는 완만하게 변화하도록 설정
    Q_.block<2, 2>(AX, AX) *= 1.0;        // 가속도는 급격한 변화를 감안해 크게 설정
    Q_(YAW, YAW) *= 0.01;
    Q_(YAW_RATE, YAW_RATE) *= 0.1;
    Q_(YAW_ACC, YAW_ACC) *= 10.0;         // 각가속도는 민첩하게 반응하도록 크게 설정
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

void EkfCore::setProcessNoise(const StateMatrix& Q) {
    std::lock_guard<std::mutex> lock(mutex_);
    Q_ = Q;
}

void EkfCore::setWheelbase(double wheelbase) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (wheelbase > 0.0) {
        wheelbase_ = wheelbase;
        if (log_info_) {
            std::ostringstream oss;
            oss << "[EkfCore] Wheelbase set to: " << wheelbase_ << " m";
            log_info_(oss.str());
        }
    }
}

void EkfCore::setRearAxleOffset(double offset) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (offset >= 0.0) {
        rear_axle_offset_ = offset;
        if (log_info_) {
            std::ostringstream oss;
            oss << "[EkfCore] Rear axle offset set to: " << rear_axle_offset_ << " m";
            log_info_(oss.str());
        }
    }
}

void EkfCore::setImuOffsets(double offset_x, double offset_y) {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_offset_x_ = offset_x;
    imu_offset_y_ = offset_y;
    if (log_info_) {
        std::ostringstream oss;
        oss << "[EkfCore] IMU offsets set to: x=" << imu_offset_x_ << " m, y=" << imu_offset_y_ << " m";
        log_info_(oss.str());
    }
}

void EkfCore::setVyDecayTimeConst(double tau) {
    std::lock_guard<std::mutex> lock(mutex_);
    vy_decay_time_const_ = tau;
}

void EkfCore::setLogCallbacks(std::function<void(const std::string&)> info,
                               std::function<void(const std::string&)> warn) {
    std::lock_guard<std::mutex> lock(mutex_);
    log_info_ = std::move(info);
    log_warn_ = std::move(warn);
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

    // --- 12차원 차량 기구학 모델 기반 예측 ---
    double vx = x_(VX);
    double vy = x_(VY);
    double ax = x_(AX);
    double ay = x_(AY);
    double yaw = x_(YAW);
    double yaw_rate = x_(YAW_RATE);
    double yaw_acc = x_(YAW_ACC);

    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    // VY 소프트 NHC 감쇠: 후륜축 접지점에서 횡속도 = 0 (비홀로노믹 구속)
    // EKF 상태 (X, Y, VX, VY)는 후륜축 기준이므로 vy_NHC = 0
    // decay 파라미터: VY의 슬립 성분을 0으로 수렴시키는 감쇠 인자
    const double vy_decay = (vy_decay_time_const_ > 1e-4) ? std::max(0.0, std::min(1.0, std::exp(-dt / vy_decay_time_const_))) : 1.0;
    const double vy_nhc = 0.0;

    // 상태 전파 (위치 상태에 대한 2차 적분 적용 및 횡속도/각가속도 전파)
    x_(X) += (vx * cos_yaw - vy * sin_yaw) * dt + 0.5 * (ax * cos_yaw - ay * sin_yaw) * dt * dt;
    x_(Y) += (vx * sin_yaw + vy * cos_yaw) * dt + 0.5 * (ax * sin_yaw + ay * cos_yaw) * dt * dt;
    x_(VX) += ax * dt;
    x_(VY) = vy_nhc + (vy - vy_nhc) * vy_decay + ay * dt;
    x_(AX) = ax;
    x_(AY) = ay;
    x_(YAW) += yaw_rate * dt + 0.5 * yaw_acc * dt * dt;
    x_(YAW_RATE) += yaw_acc * dt;
    x_(YAW_ACC) = yaw_acc;

    // 자코비안 F 행렬 [12x12]
    StateMatrix F = StateMatrix::Identity();

    // 위치 상태 변수에 대한 미분
    F(X, VX)  = cos_yaw * dt;
    F(X, VY)  = -sin_yaw * dt;
    F(X, AX)  = 0.5 * cos_yaw * dt * dt;
    F(X, AY)  = -0.5 * sin_yaw * dt * dt;
    F(X, YAW) = (-vx * sin_yaw - vy * cos_yaw) * dt - 0.5 * (ax * sin_yaw + ay * cos_yaw) * dt * dt;

    F(Y, VX)  = sin_yaw * dt;
    F(Y, VY)  = cos_yaw * dt;
    F(Y, AX)  = 0.5 * sin_yaw * dt * dt;
    F(Y, AY)  = 0.5 * cos_yaw * dt * dt;
    F(Y, YAW) = (vx * cos_yaw - vy * sin_yaw) * dt + 0.5 * (ax * cos_yaw - ay * sin_yaw) * dt * dt;

    // 속도 상태 변수에 대한 미분
    F(VX, AX) = dt;
    // VY: 소프트 NHC 감쇠에 의한 F 행렬 업데이트
    // vy(t+dt) = vy * decay + ay * dt  (vy_nhc=0, yaw_rate cross-term 없음)
    // → F(VY, VY) = decay,  F(VY, AY) = dt
    F(VY, VY)       = vy_decay;
    F(VY, AY)       = dt;

    // 요 및 각속도 상태 변수에 대한 미분
    F(YAW, YAW_RATE) = dt;
    F(YAW, YAW_ACC) = 0.5 * dt * dt;
    F(YAW_RATE, YAW_ACC) = dt;

    // 요각 정규화 (모델 독립적)
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

    // 주의: 상태 버퍼를 통한 과거 시점 소급 보정은 의도적으로 제외
    // CarMaker 시뮬레이션 환경에서는 비전 정합 지연이 100ms 이내로 매우 낮아,
    // 과거 포즈를 역추적하는 것보다 현재 시점 상태에 직접 보정하는 것이 주행 시 안정성에 더 유리
    // 지연 시간이 200ms를 초과하게 될 경우에만 소급 보정 버퍼의 재활성화 검토

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

    // 최대 단일 보정 한계치 설정 (설정 파라미터 적용)을 통해 급격한 차량 튐(Jump) 제어
    double pos_step = dx_state.segment<2>(X).norm();
    if (pos_step > max_pos_step) {
        dx_state.segment<2>(X) *= (max_pos_step / pos_step);
    }
    double yaw_step = std::abs(dx_state(YAW));
    if (yaw_step > max_yaw_step) {
        dx_state(YAW) *= (max_yaw_step / yaw_step);
    }

    x_ += dx_state;

    // 수치적 안정성을 위한 Joseph Form 공분산 업데이트: P = (I - KH) * P * (I - KH)^T + K * R * K^T
    StateMatrix I_KH = StateMatrix::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

void EkfCore::correctWheel(double vx, double vy_nhc, double yaw_rate, const Eigen::Matrix3d& R, double timestamp) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    // 후륜축 기준 휠 속도 관측 및 비홀로노믹 구속(NHC) 결합 모델
    // h(x) = [ vx, vy, w_z ]  (후륜 접지점 기준: VY_rear_axle = 0)
    Eigen::Matrix<double, 3, STATE_DIM> H = Eigen::Matrix<double, 3, STATE_DIM>::Zero();
    H(0, VX) = 1.0;
    H(1, VY) = 1.0;
    H(2, YAW_RATE) = 1.0;

    Eigen::Vector3d z(vx, vy_nhc, yaw_rate);
    Eigen::Vector3d h(x_(VX), x_(VY), x_(YAW_RATE));

    Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, STATE_DIM, 3> K = P_ * H.transpose() * S.inverse();

    x_ += K * (z - h);

    // 수치적 안정성을 위한 Joseph Form 공분산 업데이트: P = (I - KH) * P * (I - KH)^T + K * R * K^T
    StateMatrix I_KH = StateMatrix::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

void EkfCore::correctImu(double ax_raw, double ay_raw, double yaw_rate_raw, const Eigen::Matrix3d& R, double timestamp) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    // 관측 모델: 레버암 변환 및 코리올리 가속도가 결합된 원시 IMU 데이터 직접 관측
    // h(x) = [ ax - w * vy - alpha * ry - w^2 * rx + b_ax,
    //          ay + w * vx + alpha * rx - w^2 * ry + b_ay,
    //          w + b_yaw_rate ]
    double rx = imu_offset_x_ - rear_axle_offset_;
    double ry = imu_offset_y_;
    double w = x_(YAW_RATE);
    double alpha = x_(YAW_ACC);
    double vx = x_(VX);
    double vy = x_(VY);

    Eigen::Vector3d z(ax_raw, ay_raw, yaw_rate_raw);
    Eigen::Vector3d h(
        x_(AX) - w * vy - alpha * ry - w * w * rx + x_(B_AX),
        x_(AY) + w * vx + alpha * rx - w * w * ry + x_(B_AY),
        w + x_(B_YAW_RATE)
    );

    Eigen::Matrix<double, 3, STATE_DIM> H = Eigen::Matrix<double, 3, STATE_DIM>::Zero();
    H(0, AX) = 1.0;
    H(0, VY) = -w;
    H(0, YAW_RATE) = -vy - 2.0 * w * rx;
    H(0, YAW_ACC) = -ry;
    H(0, B_AX) = 1.0;

    H(1, AY) = 1.0;
    H(1, VX) = w;
    H(1, YAW_RATE) = vx - 2.0 * w * ry;
    H(1, YAW_ACC) = rx;
    H(1, B_AY) = 1.0;

    H(2, YAW_RATE) = 1.0;
    H(2, B_YAW_RATE) = 1.0;

    Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, STATE_DIM, 3> K = P_ * H.transpose() * S.inverse();

    x_ += K * (z - h);

    // 수치적 안정성을 위한 Joseph Form 공분산 업데이트: P = (I - KH) * P * (I - KH)^T + K * R * K^T
    StateMatrix I_KH = StateMatrix::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
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

    // 타임점프 후 현재 상태는 유지하되 속도와 바이어스는 클리어
    x_(VX) = 0.0; x_(VY) = 0.0;
    x_(AX) = 0.0; x_(AY) = 0.0;
    x_(YAW_RATE) = 0.0; x_(YAW_ACC) = 0.0;
    // IMU 바이어스 리셋: 시간 점프(백 루프 / 시뮬레이션 재시작)는 이전 바이어스 추정치를 무효화하므로 처음부터 다시 시작
    x_(B_AX) = 0.0; x_(B_AY) = 0.0; x_(B_YAW_RATE) = 0.0;
}

} // namespace carmaker_localization
