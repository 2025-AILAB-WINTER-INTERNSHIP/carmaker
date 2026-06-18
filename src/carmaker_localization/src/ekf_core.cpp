#include "carmaker_localization/ekf_core.h"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace carmaker_localization {

namespace {
// 파일 내부 전용 상수 모음.
// 시간 처리 기준값을 EkfCore 구현 안에서만 사용

// EKF 예측 단계에서 dt가 이 값보다 작으면 예측을 생략
constexpr double kMinPredictionDt = 1e-4;

// EKF 예측 단계에서 dt가 이 값보다 크면 시간 도약으로 간주
constexpr double kMaxPredictionDt = 1.0;

// EKF 예측 단계에서 timestamp가 현재 state 시간보다 이 값 이상 과거이면 시간 도약으로 간주
constexpr double kMaxBackwardTimeJump = 0.05;

// EKF 측정 업데이트에서 timestamp가 현재 state 시간보다 이 값 이상 과거이면 out-of-sequence로 간주
constexpr double kOutOfSequenceTolerance = 1e-4;

} // namespace

// EKF Core 클래스 구현
EkfCore::EkfCore()
    : last_velocity_(0.0), last_yaw_rate_(0.0), last_time_(0.0), is_initialized_(false) {
    x_.setZero();
    P_.setIdentity();
    Q_.setZero();
    last_input_noise_.setZero();

    // 위치와 자세 공정 노이즈 튜닝
    Q_(X, X) = 1e-4;
    Q_(Y, Y) = 1e-4;
    Q_(YAW, YAW) = 1e-4;
}

// 초기 상태 설정
void EkfCore::initialize(double x, double y, double yaw, double timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    x_.setZero();
    x_(X) = x;
    x_(Y) = y;
    x_(YAW) = normalizeAngle(yaw);
    last_velocity_ = 0.0;
    last_yaw_rate_ = 0.0;
    last_input_noise_.setZero();

    // 초기 공분산 설정: 위치와 자세는 비교적 확실하게 시작
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

    // Q_는 pose 상태에 직접 더해지는 continuous-time residual process noise다.
    // prediction 단계에서는 Q_ * dt로 이산화한다.
    Q_ = (Q + Q.transpose()) * 0.5;
}

void EkfCore::setWarnLogCallback(std::function<void(const std::string&)> warn) {
    std::lock_guard<std::mutex> lock(mutex_);
    log_warn_ = std::move(warn);
}

void EkfCore::prediction(double timestamp, double velocity, double yaw_rate, const Eigen::Matrix2d& R_input) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_initialized_) return;

    predictUnlocked(timestamp, velocity, yaw_rate, R_input);
}

bool EkfCore::predictUnlocked(double timestamp, double velocity, double yaw_rate, const Eigen::Matrix2d& R_input) {
    if (!std::isfinite(timestamp)) {
        if (log_warn_) log_warn_("[EkfCore] Prediction skipped: timestamp is not finite.");
        return false;
    } 
    // velocity/yaw_rate가 정상 숫자인지 확인
    if (!std::isfinite(velocity) || !std::isfinite(yaw_rate)) {
        if (log_warn_) log_warn_("[EkfCore] Prediction skipped: motion input is not finite.");
        return false;
    }
    
    // 입력 공분산은 이론적으로 대칭이어야 하므로 작은 비대칭을 보정한다.
    Eigen::Matrix2d input_noise = (R_input + R_input.transpose()) * 0.5;
    for (int r = 0; r < input_noise.rows(); ++r) {
        for (int c = 0; c < input_noise.cols(); ++c) {
            if (!std::isfinite(input_noise(r, c))) {
                if (log_warn_) log_warn_("[EkfCore] Prediction skipped: input covariance is not finite.");
                return false;
            }
        }
    }
    // 음수 분산을 0으로 보정
    input_noise(0, 0) = std::max(0.0, input_noise(0, 0));
    input_noise(1, 1) = std::max(0.0, input_noise(1, 1));

    double dt = timestamp - last_time_;

    // 시간 도약 감지: dt가 너무 크거나 음수로 크게 떨어지는 경우, 예측을 건너뛰고 상태를 리셋
    if (dt > kMaxPredictionDt || dt < -kMaxBackwardTimeJump) {
        handleTimeJump(timestamp);
        return false;
    }
    
    last_velocity_ = velocity;
    last_yaw_rate_ = yaw_rate;
    last_input_noise_ = input_noise;

    if (dt <= kMinPredictionDt) return true; // 너무 작은 시간 간격은 연산 생략

    // --- 1) 상태 예측: x^-_k = f(x^+_{k-1}) ---
    //
    // 상태는 후륜축 기준 [X, Y, YAW].
    // motion input [velocity, yaw_rate]을 한 prediction 구간 동안 일정하다고 보고 전파한다.
    double yaw = x_(YAW);

    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    x_(X) += velocity * cos_yaw * dt;
    x_(Y) += velocity * sin_yaw * dt;
    x_(YAW) += yaw_rate * dt;

    // 자코비안 F = df/dx 행렬 [3x3]
    StateMatrix F = StateMatrix::Identity();
    F(X, YAW) = -velocity * sin_yaw * dt;
    F(Y, YAW) = velocity * cos_yaw * dt;

    // 입력 자코비안 G = df/du, u=[velocity, yaw_rate]
    Eigen::Matrix<double, STATE_DIM, 2> G = Eigen::Matrix<double, STATE_DIM, 2>::Zero();
    G(X, 0) = cos_yaw * dt;
    G(Y, 0) = sin_yaw * dt;
    G(YAW, 1) = dt;

    // 예측된 yaw는 원형 변수이므로 정규화하여 -pi ~ pi 범위로 유지
    x_(YAW) = normalizeAngle(x_(YAW));

    // --- 2) 공분산 예측 ---
    //
    // 일반식은 P^- = F P^+ F^T + L Q L^T.
    // 여기서는 process noise를 두 종류로 나눠서 구성한다.
    //
    //   L = [I, G]
    //   Q = blockdiag(Q_pose * dt, R_input)
    //
    // 따라서
    //
    //   L Q L^T = Q_pose * dt + G R_input G^T
    //
    // Q_는 pose 상태에 직접 더해지는 continuous-time residual process noise이고,
    // input_noise는 motion input [velocity, yaw_rate]의 covariance다.
    P_ = F * P_ * F.transpose() + Q_ * dt + G * input_noise * G.transpose();
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

    // 측정이 현재 state보다 미래이면 마지막 motion input으로 그 시각까지 prediction하여 x^-를 만든다.
    return predictUnlocked(timestamp, last_velocity_, last_yaw_rate_, last_input_noise_);
}

double EkfCore::normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

void EkfCore::stabilizeCovariance() {
    P_ = (P_ + P_.transpose()) * 0.5;
}

// 외부 위치/자세 관측 보정
void EkfCore::correctPose(double x, double y, double yaw, const Eigen::Matrix3d& R, double timestamp,
                          double max_pos_step, double max_yaw_step) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_initialized_) return;
    if (!advanceToMeasurementTime(timestamp, "correctPose")) return;

    // --- 3) 측정 모델: z_pose = h_pose(x) + v, v ~ N(0, R_pose) ---
    //
    // h_pose(x) = [X, Y, YAW]^T 이므로 H = dh/dx는 3x3 identity와 같다.
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
StateFrame EkfCore::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {last_time_, x_, P_};
}
// 시간 도약 감지 시 상태와 공분산을 완화하여 재수렴을 허용
void EkfCore::handleTimeJump(double timestamp) {
    if (log_warn_) {
        std::ostringstream oss;
        oss << "[EkfCore] Time jump detected (dt: " << (timestamp - last_time_)
            << "s). Resetting covariance and motion input.";
        log_warn_(oss.str());
    }

    last_time_ = timestamp;

    // 재수렴을 허용하기 위해 공분산을 초기 상태로 리셋
    P_.setIdentity();
    P_.block<2, 2>(X, X) *= 0.5;
    P_(YAW, YAW) *= 0.1;
    last_velocity_ = 0.0;
    last_yaw_rate_ = 0.0;
    last_input_noise_.setZero();
}

} // namespace carmaker_localization
