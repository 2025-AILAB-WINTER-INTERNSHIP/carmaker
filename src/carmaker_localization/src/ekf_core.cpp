#include "carmaker_localization/ekf_core.h"
#include <cmath>
#include <sstream>
#include <utility>

namespace carmaker_localization {

namespace {
// 파일 내부 전용 상수 모음.
// 시간 처리와 스칼라 분산 검사 하한값을 EkfCore 구현 안에서만 사용

// EKF 예측 단계에서 dt가 이 값보다 작으면 예측을 생략
constexpr double kMinPredictionDt = 1e-4;

// EKF 예측 단계에서 dt가 이 값보다 크면 시간 도약으로 간주
constexpr double kMaxPredictionDt = 1.0;

// EKF 예측 단계에서 timestamp가 현재 state 시간보다 이 값 이상 과거이면 시간 도약으로 간주
constexpr double kMaxBackwardTimeJump = 0.05;

// EKF 측정 업데이트에서 timestamp가 현재 state 시간보다 이 값 이상 과거이면 out-of-sequence로 간주
constexpr double kOutOfSequenceTolerance = 1e-4;

// IMU scalar update에서 분산과 innovation covariance를 검사할 때 쓰는 하한값
constexpr double kMinVariance = 1e-12;

} // namespace

// EKF Core 클래스 구현
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

// 초기 상태 설정
void EkfCore::initialize(double x, double y, double yaw, double timestamp, double vx) {
    std::lock_guard<std::mutex> lock(mutex_);
    x_.setZero();
    x_(X) = x;
    x_(Y) = y;
    x_(YAW) = normalizeAngle(yaw);
    x_(VX) = vx;

    P_.setIdentity();
    P_.block<2, 2>(X, X) *= 0.1;
    P_(YAW, YAW) *= 0.05;

    last_time_ = timestamp;
    is_initialized_ = true;
}
// EKF 초기화 여부 반환
bool EkfCore::isInitialized() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return is_initialized_;
}

void EkfCore::setProcessNoise(const StateMatrix& Q) {
    std::lock_guard<std::mutex> lock(mutex_);

    // P^- = FPF^T + LQL^T에서 이 구현은 L=I로 단순화
    // Q는 continuous-time PSD 행렬이므로, prediction 단계에서는 Q * dt로 이산화
    Q_ = (Q + Q.transpose()) * 0.5;
}

void EkfCore::setWarnLogCallback(std::function<void(const std::string&)> warn) {
    std::lock_guard<std::mutex> lock(mutex_);
    log_warn_ = std::move(warn);
}

void EkfCore::prediction(double timestamp, double vx_wheel, double ax_imu) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_initialized_) return;

    predictUnlocked(timestamp, vx_wheel, ax_imu);
}

bool EkfCore::predictUnlocked(double timestamp, double vx_wheel, double ax_imu) {
    if (!std::isfinite(timestamp)) {
        if (log_warn_) log_warn_("[EkfCore] Prediction skipped: timestamp is not finite.");
        return false;
    }
    double dt = timestamp - last_time_;

    // 시간 도약 감지: dt가 너무 크거나 음수로 크게 떨어지는 경우, 예측을 건너뛰고 상태를 리셋
    if (dt > kMaxPredictionDt || dt < -kMaxBackwardTimeJump) {
        handleTimeJump(timestamp);
        return false;
    }

    if (dt <= kMinPredictionDt) return true; // 너무 작은 시간 간격은 연산 생략

    // --- 1) 상태 예측: x^-_k = f(x^+_{k-1}) ---
    //
    // 상태는 후륜축 기준 [X, Y, YAW, VX, YAW_RATE, B_YAW_RATE]
    // 한 prediction 구간 동안 yaw와 YAW_RATE가 일정하다고 보고,
    // IMU 종방향 가속도로 VX를 1차 적분한 뒤 평균 속도로 위치를 전파
    double vx = x_(VX);
    double yaw = x_(YAW);
    double yaw_rate = x_(YAW_RATE);

    // 종방향 가속도 입력을 통합하여 예측 속도 계산
    double vx_next = vx + ax_imu * dt;

    // 주행 방향(휠 속도 부호 기준)에 따른 속도 제한 (감속 시 속도 부호 역전 방지)
    if (vx_wheel >= -0.01) {
        if (vx_next < 0.0) vx_next = 0.0; // 전진/정차 중인 경우 음의 속도로 흐르지 않도록 제한
    } else {
        if (vx_next > 0.0) vx_next = 0.0; // 후진 중인 경우 양의 속도로 흐르지 않도록 제한
    }
    double vx_avg = 0.5 * (vx + vx_next);

    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    // 6차원 상태 기반 등가속/등요레이트(YAW_RATE) 오도메트리 전파.
    x_(X) += vx_avg * cos_yaw * dt;
    x_(Y) += vx_avg * sin_yaw * dt;
    x_(YAW) += yaw_rate * dt;
    x_(VX) = vx_next;
    // yaw_rate, b_yaw_rate는 등요레이트(YAW_RATE) 예측 모델에 따라 유지


    // 자코비안 F = df/dx 행렬 [6x6]
    StateMatrix F = StateMatrix::Identity();
    F(X, YAW) = -vx_avg * sin_yaw * dt;
    F(X, VX) = cos_yaw * dt;
    F(Y, YAW) = vx_avg * cos_yaw * dt;
    F(Y, VX) = sin_yaw * dt;
    F(YAW, YAW_RATE) = dt;

    // 예측된 yaw는 원형 변수이므로 정규화하여 -pi ~ pi 범위로 유지
    x_(YAW) = normalizeAngle(x_(YAW));

    // --- 2) 공분산 예측: P^-_k = F P^+_{k-1} F^T + L Q L^T ---
    //
    // process noise는 상태에 직접 더해지는 형태로 모델링하므로 L=I로 단순화
    // Q_는 continuous-time PSD 행렬이므로, 예측 단계에서는 Q_ * dt로 이산화
    P_ = F * P_ * F.transpose() + Q_ * dt;
    // 수치적 안정성을 위한 공분산 대칭화
    stabilizeCovariance();

    last_time_ = timestamp;
    return true;
}

bool EkfCore::advanceToMeasurementTime(double timestamp, const char* source) {
    if (!std::isfinite(timestamp)) {
        if (log_warn_) {
            std::ostringstream oss;
            oss << "[EkfCore] " << source << " skipped: measurement timestamp is not finite.";
            log_warn_(oss.str());
        }
        return false;
    }

    const double dt = timestamp - last_time_;
    // 측정이 현재 state보다 과거이면 state history가 없어 보정하지 않고 건너뛴다.
    if (dt < -kOutOfSequenceTolerance) {
        if (log_warn_) {
            std::ostringstream oss;
            oss << "[EkfCore] " << source << " skipped: out-of-sequence measurement. "
                << "measurement_time=" << timestamp << ", state_time=" << last_time_;
            log_warn_(oss.str());
        }
        return false;
    }

    // 측정이 현재 state보다 미래이면 그 시각까지 prediction하여 x^-를 만든다.
    return predictUnlocked(timestamp, x_(VX), 0.0);
}

double EkfCore::normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

void EkfCore::stabilizeCovariance() {
    P_ = (P_ + P_.transpose()) * 0.5;
}

// 센서1: 외부 위치/자세 관측 보정
void EkfCore::correctPose(double x, double y, double yaw, const Eigen::Matrix3d& R, double timestamp,
                          double max_pos_step, double max_yaw_step) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_initialized_) return;
    if (!advanceToMeasurementTime(timestamp, "correctPose")) return;

    // --- 3) 측정 모델: z_pose = h_pose(x) + v, v ~ N(0, R_pose) ---
    //
    // h_pose(x) = [X, Y, YAW]^T 이므로 H = dh/dx는 상태에서 해당 성분을 꺼내는 선택 행렬
    // M은 additive measurement noise이므로 M=I.
    Eigen::Matrix<double, 3, STATE_DIM> H = Eigen::Matrix<double, 3, STATE_DIM>::Zero();
    H(0, X) = 1.0; H(1, Y) = 1.0; H(2, YAW) = 1.0;

    Eigen::Vector3d z(x, y, normalizeAngle(yaw));
    Eigen::Vector3d h(x_(X), x_(Y), x_(YAW));

    // innovation y = z - h(x^-). yaw는 원형 변수라 단순 뺄셈 후 wrap한다.
    Eigen::Vector3d y_res = z - h;
    y_res(2) = normalizeAngle(y_res(2));

    // S = H P^- H^T + R, K = P^- H^T S^-1.
    Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, STATE_DIM, 3> K = P_ * H.transpose() * S.inverse();
    StateVector dx_state = K * y_res;

    // 최대 허용 편차 검증 및 클리핑: EKF 상태가 관측 쪽으로 급격히 튀는 것을 방지
    bool clipped = false;
    double pos_step = dx_state.segment<2>(X).norm();
    if (std::isfinite(max_pos_step) && max_pos_step > 0.0 && pos_step > max_pos_step) {
        dx_state.segment<2>(X) *= (max_pos_step / pos_step);
        clipped = true;
    }
    double yaw_step = std::abs(dx_state(YAW));
    if (std::isfinite(max_yaw_step) && max_yaw_step > 0.0 && yaw_step > max_yaw_step) {
        dx_state(YAW) *= (max_yaw_step / yaw_step);
        clipped = true;
    }

    const StateMatrix P_prior = P_;
    x_ += dx_state;
    x_(YAW) = normalizeAngle(x_(YAW));

    if (clipped) {
        P_ = P_prior;
    } else {
        // Joseph form: P^+ = (I-KH)P^-(I-KH)^T + K R K^T.
        // 단순 (I-KH)P보다 대칭성과 양의 정부호성을 더 잘 보존
        StateMatrix I_KH = StateMatrix::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    }
    stabilizeCovariance();
}
// 센서2: 휠 속도 및 요레이트 관측 보정
void EkfCore::correctWheel(double vx, double yaw_rate, const Eigen::Matrix2d& R, double timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_initialized_) return;
    if (!advanceToMeasurementTime(timestamp, "correctWheel")) return;

    // z_wheel = [v_x,wheel, yaw_rate,wheel]^T.
    // h_wheel(x) = [VX, YAW_RATE]^T 이므로 H = dh/dx는 상태에서 해당 성분을 꺼내는 선택 행렬
    Eigen::Matrix<double, 2, STATE_DIM> H = Eigen::Matrix<double, 2, STATE_DIM>::Zero();
    H(0, VX) = 1.0;
    H(1, YAW_RATE) = 1.0;

    Eigen::Vector2d z(vx, yaw_rate);
    Eigen::Vector2d h(x_(VX), x_(YAW_RATE));
    Eigen::Vector2d y_res = z - h;

    Eigen::Matrix2d S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, STATE_DIM, 2> K = P_ * H.transpose() * S.inverse();
    x_ += K * y_res;
    x_(YAW) = normalizeAngle(x_(YAW));

    // 수치적 안정성을 위한 Joseph Form 공분산 업데이트
    StateMatrix I_KH = StateMatrix::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    stabilizeCovariance();
}
// 센서3: IMU 요레이트 관측 보정 (YAW_RATE + B_YAW_RATE)
void EkfCore::correctImu(double yaw_rate_raw, double R_gyro, double timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_initialized_) return;

    // 유효하지 않은 측정 분산으로 state time이 advance되지 않도록 먼저 검사한다.
    if (!std::isfinite(R_gyro) || R_gyro <= kMinVariance) {
        if (log_warn_) log_warn_("[EkfCore] correctImu skipped: invalid gyro variance.");
        return;
    }

    if (!advanceToMeasurementTime(timestamp, "correctImu")) return;

    // z_imu = yaw_rate_raw = yaw_rate_vehicle + gyro_bias + noise.
    // h_imu(x) = YAW_RATE + B_YAW_RATE 이므로 H는 두 상태 성분의 합을 관측한다.
    Eigen::Matrix<double, 1, STATE_DIM> H = Eigen::Matrix<double, 1, STATE_DIM>::Zero();
    H(0, YAW_RATE) = 1.0;
    H(0, B_YAW_RATE) = 1.0;

    double z = yaw_rate_raw;
    double h = x_(YAW_RATE) + x_(B_YAW_RATE);
    double y_res = z - h;

    double S = (H * P_ * H.transpose())(0, 0) + R_gyro;
    if (!std::isfinite(S) || S <= kMinVariance) {
        if (log_warn_) log_warn_("[EkfCore] correctImu skipped: invalid innovation covariance S.");
        return;
    }

    Eigen::Matrix<double, STATE_DIM, 1> K = P_ * H.transpose() / S;

    x_ += K * y_res;
    x_(YAW) = normalizeAngle(x_(YAW));

    // 수치적 안정성을 위한 Joseph Form 공분산 업데이트
    StateMatrix I_KH = StateMatrix::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_gyro * K.transpose();
    stabilizeCovariance();
}

StateFrame EkfCore::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {last_time_, x_, P_};
}
// 시간 도약 감지 시 상태와 공분산을 완화하여 재수렴을 허용
void EkfCore::handleTimeJump(double timestamp) {
    if (log_warn_) {
        std::ostringstream oss;
        oss << "[EkfCore] Time jump detected (dt: " << (timestamp - last_time_)
            << "s). Resetting covariance and dynamic states.";
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
