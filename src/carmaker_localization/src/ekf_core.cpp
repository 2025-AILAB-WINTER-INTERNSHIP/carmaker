#include "carmaker_localization/ekf_core.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace carmaker_localization {

EkfCore::EkfCore(double buffer_duration)
    : buffer_duration_(buffer_duration), is_initialized_(false), last_time_(0.0) {
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
    state_buffer_.clear();
    updateBuffer(timestamp);
}

void EkfCore::setParameters(double tire_radius, double wheelbase, double track_width, double rear_axle_x) {
    tire_radius_ = tire_radius;
    wheelbase_ = wheelbase;
    track_width_ = track_width;
    rear_axle_x_ = rear_axle_x;
}

void EkfCore::setProcessNoise(const Eigen::MatrixXd& Q) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (Q.rows() == STATE_DIM && Q.cols() == STATE_DIM) {
        Q_ = Q;
    }
}

void EkfCore::prediction(double timestamp) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    double dt = timestamp - last_time_;

    // Detect Time Jump (Forward > 1s or any Backward jump)
    if (dt > 1.0 || dt < 0.0) {
        handleTimeJump(timestamp);
        return;
    }

    if (dt <= 1e-4) return; // Ignore too small intervals

    // --- State Transition (Constant Acceleration Model) ---
    double yaw = x_(YAW);
    double vx = x_(VX);
    double vy = x_(VY);
    double ax = x_(AX);
    double ay = x_(AY);
    double yaw_rate = x_(YAW_RATE);

    // Global Position Update
    double cos_y = std::cos(yaw);
    double sin_y = std::sin(yaw);
    double v_global_x = vx * cos_y - vy * sin_y;
    double v_global_y = vx * sin_y + vy * cos_y;

    x_(X) += v_global_x * dt + 0.5 * (ax * cos_y - ay * sin_y) * dt * dt;
    x_(Y) += v_global_y * dt + 0.5 * (ax * sin_y + ay * cos_y) * dt * dt;

    // Local Velocity Update
    x_(VX) += ax * dt;
    x_(VY) += ay * dt;

    // Heading Update
    x_(YAW) += yaw_rate * dt;
    // Yaw normalization
    while (x_(YAW) > M_PI) x_(YAW) -= 2.0 * M_PI;
    while (x_(YAW) < -M_PI) x_(YAW) += 2.0 * M_PI;

    // --- Jacobian Matrix F [11x11] ---
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    F(X, YAW) = (-vx * sin_y - vy * cos_y) * dt;
    F(X, VX) = cos_y * dt;
    F(X, VY) = -sin_y * dt;
    F(X, AX) = 0.5 * cos_y * dt * dt;
    F(X, AY) = -0.5 * sin_y * dt * dt;

    F(Y, YAW) = (vx * cos_y - vy * sin_y) * dt;
    F(Y, VX) = sin_y * dt;
    F(Y, VY) = cos_y * dt;
    F(Y, AX) = 0.5 * sin_y * dt * dt;
    F(Y, AY) = 0.5 * cos_y * dt * dt;

    F(VX, AX) = dt;
    F(VY, AY) = dt;
    F(YAW, YAW_RATE) = dt;

    // --- Covariance Propagation ---
    P_ = F * P_ * F.transpose() + Q_ * dt;

    // Symmetry check
    P_ = (P_ + P_.transpose()) * 0.5;

    last_time_ = timestamp;
    updateBuffer(timestamp);
}

void EkfCore::correctPose(double x, double y, double yaw, const Eigen::Matrix3d& R, double timestamp) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    // 1. Handle Latency: Find the frame and update current state using historical residual
    if (timestamp < last_time_) {
        auto it = std::lower_bound(state_buffer_.begin(), state_buffer_.end(), timestamp,
            [](const StateFrame& f, double t) { return f.timestamp < t; });

        if (it != state_buffer_.end()) {
            Eigen::Vector3d z(x, y, yaw);
            Eigen::Vector3d h(it->x(X), it->x(Y), it->x(YAW));
            Eigen::Vector3d y_res = z - h;
            while (y_res(2) > M_PI) y_res(2) -= 2.0 * M_PI;
            while (y_res(2) < -M_PI) y_res(2) += 2.0 * M_PI;

            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, STATE_DIM);
            H(0, X) = 1.0; H(1, Y) = 1.0; H(2, YAW) = 1.0;

            Eigen::MatrixXd S = H * it->P * H.transpose() + R;
            Eigen::MatrixXd K = it->P * H.transpose() * S.inverse();

            x_ += K * y_res;
            P_ = (Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H) * P_;
            P_ = (P_ + P_.transpose()) * 0.5;
            return;
        }
    }

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
    P_ = (Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H) * P_;
    P_ = (P_ + P_.transpose()) * 0.5;
}

void EkfCore::correctVelocity(double vx, double vy, const Eigen::Matrix2d& R, double timestamp) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    if (timestamp < last_time_) {
        auto it = std::lower_bound(state_buffer_.begin(), state_buffer_.end(), timestamp,
            [](const StateFrame& f, double t) { return f.timestamp < t; });
        if (it != state_buffer_.end()) {
            Eigen::Vector2d z(vx, vy);
            Eigen::Vector2d h(it->x(VX), it->x(VY));
            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, STATE_DIM);
            H(0, VX) = 1.0; H(1, VY) = 1.0;
            Eigen::MatrixXd S = H * it->P * H.transpose() + R;
            Eigen::MatrixXd K = it->P * H.transpose() * S.inverse();
            x_ += K * (z - h);
            P_ = (Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H) * P_;
            P_ = (P_ + P_.transpose()) * 0.5;
            return;
        }
    }

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, STATE_DIM);
    H(0, VX) = 1.0; H(1, VY) = 1.0;

    Eigen::Vector2d z(vx, vy);
    Eigen::Vector2d h(x_(VX), x_(VY));

    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ += K * (z - h);
    P_ = (Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H) * P_;
    P_ = (P_ + P_.transpose()) * 0.5;
}

void EkfCore::correctImu(double ax_raw, double ay_raw, double yaw_rate_raw, const Eigen::Matrix3d& R, double timestamp) {
    if (!is_initialized_) return;
    std::lock_guard<std::mutex> lock(mutex_);

    if (timestamp < last_time_) {
        auto it = std::lower_bound(state_buffer_.begin(), state_buffer_.end(), timestamp,
            [](const StateFrame& f, double t) { return f.timestamp < t; });
        if (it != state_buffer_.end()) {
            Eigen::Vector3d z(ax_raw, ay_raw, yaw_rate_raw);
            Eigen::Vector3d h(it->x(AX) + it->x(B_AX), it->x(AY) + it->x(B_AY), it->x(YAW_RATE) + it->x(B_YAW_RATE));
            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, STATE_DIM);
            H(0, AX) = 1.0; H(0, B_AX) = 1.0;
            H(1, AY) = 1.0; H(1, B_AY) = 1.0;
            H(2, YAW_RATE) = 1.0; H(2, B_YAW_RATE) = 1.0;
            Eigen::MatrixXd S = H * it->P * H.transpose() + R;
            Eigen::MatrixXd K = it->P * H.transpose() * S.inverse();
            x_ += K * (z - h);
            P_ = (Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H) * P_;
            P_ = (P_ + P_.transpose()) * 0.5;
            return;
        }
    }

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
    P_ = (Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H) * P_;
    P_ = (P_ + P_.transpose()) * 0.5;
}

StateFrame EkfCore::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {last_time_, x_, P_};
}

void EkfCore::updateBuffer(double timestamp) {
    state_buffer_.push_back({timestamp, x_, P_});
    while (!state_buffer_.empty() && (timestamp - state_buffer_.front().timestamp > buffer_duration_)) {
        state_buffer_.pop_front();
    }
}

void EkfCore::handleTimeJump(double timestamp) {
    std::cout << "\033[1;33m[EkfCore] Time jump detected (dt: " << (timestamp - last_time_) << "s). Resetting filter state and buffer.\033[0m" << std::endl;

    last_time_ = timestamp;
    state_buffer_.clear();

    // Reset covariance to initial state to allow re-convergence
    P_.setIdentity();
    P_.block<2, 2>(X, X) *= 0.5;
    P_(YAW, YAW) *= 0.1;

    // Maintain current state but clear velocities if it was a big jump
    x_(VX) = 0.0; x_(VY) = 0.0;
    x_(AX) = 0.0; x_(AY) = 0.0;
    x_(YAW_RATE) = 0.0;

    updateBuffer(timestamp);
}

} // namespace carmaker_localization
