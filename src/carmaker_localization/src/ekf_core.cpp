#include "carmaker_localization/ekf_core.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace carmaker_localization {

EkfCore::EkfCore()
    : is_initialized_(false), last_time_(0.0), wheelbase_(2.97) {
    x_ = Eigen::VectorXd::Zero(STATE_DIM);
    P_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 1.0;
    Q_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.01;

    // Process noise tuning for motion vs bias
    Q_.block<2, 2>(X, X) *= 0.01;         // Position is very smooth
    Q_.block<2, 2>(VX, VX) *= 0.1;        // Velocity can change
    Q_.block<2, 2>(AX, AX) *= 1.0;        // Accel can change rapidly
    Q_(YAW, YAW) *= 0.01;
    Q_(YAW_RATE, YAW_RATE) *= 0.1;
    Q_.block<3, 3>(B_AX, B_AX) *= 0.0001; // Biases change very slowly
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

void EkfCore::prediction(double timestamp, const PredictionInput& u) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    double dt = timestamp - last_time_;

    // Detect Time Jump (Forward > 1s or any Backward jump)
    if (dt > 1.0 || dt < -0.05) {
        handleTimeJump(timestamp);
        return;
    }

    if (dt <= 1e-4) return; // Ignore too small intervals

    // --- Kinematic Bicycle Model Prediction ---
    double v = x_(VX);
    double yaw = x_(YAW);
    double ax = x_(AX);
    double ay = x_(AY);
    double delta = u.steering_angle;

    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    // Clamp steering angle to physical passenger vehicle limits (approx +/-40 deg = 0.7 rad)
    double delta_clamped = std::max(-0.7, std::min(0.7, delta));
    double tan_delta = std::tan(delta_clamped);
    double yaw_rate_bicycle = v * tan_delta / wheelbase_;
    double ay_bicycle = v * yaw_rate_bicycle;

    // State propagation (second-order integration for position)
    x_(X) += v * cos_yaw * dt + 0.5 * (ax * cos_yaw - ay * sin_yaw) * dt * dt;
    x_(Y) += v * sin_yaw * dt + 0.5 * (ax * sin_yaw + ay * cos_yaw) * dt * dt;
    x_(VX) += ax * dt;
    x_(VY) = 0.0; // Non-holonomic constraint: lateral velocity is zero at rear axle
    x_(AX) = ax;  // Constant longitudinal acceleration propagation
    x_(AY) = ay_bicycle; // Lateral acceleration is constrained to centripetal acceleration
    x_(YAW) += yaw_rate_bicycle * dt;
    x_(YAW_RATE) = yaw_rate_bicycle;

    // Jacobian F
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);

    // Position derivatives
    F(X, VX)  = cos_yaw * dt;
    F(X, AX)  = 0.5 * cos_yaw * dt * dt;
    F(X, AY)  = -0.5 * sin_yaw * dt * dt;
    F(X, YAW) = -v * sin_yaw * dt - 0.5 * (ax * sin_yaw + ay * cos_yaw) * dt * dt;

    F(Y, VX)  = sin_yaw * dt;
    F(Y, AX)  = 0.5 * sin_yaw * dt * dt;
    F(Y, AY)  = 0.5 * cos_yaw * dt * dt;
    F(Y, YAW) = v * cos_yaw * dt + 0.5 * (ax * cos_yaw - ay * sin_yaw) * dt * dt;

    // Velocity derivatives
    F(VX, AX) = dt;

    // Override Identity diagonal for constrained states
    F(VY, VY) = 0.0;
    F(AY, AY) = 0.0;
    F(YAW_RATE, YAW_RATE) = 0.0;

    // Kinematic constraints derivatives
    F(AY, VX) = 2.0 * v * tan_delta / wheelbase_;
    F(YAW, VX) = tan_delta / wheelbase_ * dt;
    F(YAW_RATE, VX) = tan_delta / wheelbase_;

    // Yaw normalization (model-agnostic)
    while (x_(YAW) > M_PI) x_(YAW) -= 2.0 * M_PI;
    while (x_(YAW) < -M_PI) x_(YAW) += 2.0 * M_PI;

    // --- Covariance Propagation ---
    P_ = F * P_ * F.transpose() + Q_ * dt;

    // Symmetry check
    P_ = (P_ + P_.transpose()) * 0.5;

    last_time_ = timestamp;
}

void EkfCore::correctPose(double x, double y, double yaw, const Eigen::Matrix3d& R, double timestamp) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    // NOTE: Retroactive correction via state buffer was intentionally removed.
    // The CarMaker simulation pipeline delivers vision measurements with
    // sub-100ms latency, well within one 100 Hz prediction cycle. Applying
    // measurements to the current state is empirically more stable than
    // rewinding history, which introduced oscillation at higher vehicle speeds.
    // If sustained latency ever exceeds ~200 ms, reconsider this decision.

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, STATE_DIM);
    H(0, X) = 1.0; H(1, Y) = 1.0; H(2, YAW) = 1.0;

    Eigen::Vector3d z(x, y, yaw);
    Eigen::Vector3d h(x_(X), x_(Y), x_(YAW));

    // Residual with yaw wrapping
    Eigen::Vector3d y_res = z - h;
    while (y_res(2) > M_PI) y_res(2) -= 2.0 * M_PI;
    while (y_res(2) < -M_PI) y_res(2) += 2.0 * M_PI;

    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ += K * y_res;

    // Joseph Form covariance update for numerical stability: P = (I - KH) * P * (I - KH)^T + K * R * K^T
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

    // Joseph Form covariance update for numerical stability: P = (I - KH) * P * (I - KH)^T + K * R * K^T
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

void EkfCore::correctImu(double ax_raw, double ay_raw, double yaw_rate_raw, const Eigen::Matrix3d& R, double timestamp) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    // Observation model: z_raw = state + bias
    // So h(x) = [ax + b_ax, ay + b_ay, yaw_rate + b_yaw_rate]
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, STATE_DIM);
    H(0, AX) = 1.0; H(0, B_AX) = 1.0;
    H(1, AY) = 1.0; H(1, B_AY) = 1.0;
    H(2, YAW_RATE) = 1.0; H(2, B_YAW_RATE) = 1.0;

    Eigen::Vector3d z(ax_raw, ay_raw, yaw_rate_raw);
    Eigen::Vector3d h(x_(AX) + x_(B_AX), x_(AY) + x_(B_AY), x_(YAW_RATE) + x_(B_YAW_RATE));

    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ += K * (z - h);

    // Joseph Form covariance update for numerical stability: P = (I - KH) * P * (I - KH)^T + K * R * K^T
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

    // Reset covariance to initial state to allow re-convergence
    P_.setIdentity();
    P_.block<2, 2>(X, X) *= 0.5;
    P_(YAW, YAW) *= 0.1;

    // Maintain current state but clear velocities and biases after the jump
    x_(VX) = 0.0; x_(VY) = 0.0;
    x_(AX) = 0.0; x_(AY) = 0.0;
    x_(YAW_RATE) = 0.0;
    // Reset IMU biases: a time jump (bag-loop / sim restart) invalidates
    // previously converged bias estimates, so start fresh.
    x_(B_AX) = 0.0; x_(B_AY) = 0.0; x_(B_YAW_RATE) = 0.0;
}

} // namespace carmaker_localization
