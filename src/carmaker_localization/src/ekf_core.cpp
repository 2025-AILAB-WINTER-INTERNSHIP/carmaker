#include "carmaker_localization/ekf_core.h"

namespace carmaker_localization {

EkfCore::EkfCore(double buffer_duration) : buffer_(buffer_duration), last_time_(0.0) {
}

void EkfCore::init(const Eigen::Matrix<double, STATE_DIM, 1>& x0,
                    const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& P0) {
    x_ = x0;
    P_ = P0;
    P_init_ = P0; // Save for recovery
    buffer_.clear();
}

void EkfCore::predict(double dt, const Eigen::Matrix<double, 6, 1>& u) {
    predictInternal(dt, u, false);
}

void EkfCore::predictInternal(double dt, const Eigen::Matrix<double, 6, 1>& u, bool skip_buffer) {
    // 1. IMU Integration (Dead Reckoning)
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> F = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
    double ax = u(0);
    double ay = u(1);
    double wz = u(2);
    double v_rl = u(3);
    double v_rr = u(4);
    double steer = u(5);

    double c = std::cos(x_(PSI));
    double s = std::sin(x_(PSI));
    double psidot_eff = wz - x_(BGZ);

    // IMU Lever-arm correction (Centripetal acceleration compensation from Fr1A to IMU)
    // a_base = a_imu + w^2 * r
    double ax_base = ax + psidot_eff * psidot_eff * imu_offset_x_;
    double ay_base = ay + psidot_eff * psidot_eff * imu_offset_y_;

    x_(X) += (x_(VX) * c - x_(VY) * s) * dt;
    x_(Y) += (x_(VX) * s + x_(VY) * c) * dt;
    x_(PSI) = normalizeAngle(x_(PSI) + psidot_eff * dt);

    x_(VX) += (ax_base - x_(BAX) + x_(VY) * psidot_eff) * dt;
    x_(VY) += (ay_base - x_(BAY) - x_(VX) * psidot_eff) * dt;
    x_(PSIDOT) = psidot_eff;
    x_(DELTA) = steer;

    F(X, PSI) = (-x_(VX) * s - x_(VY) * c) * dt;
    F(X, VX)  = c * dt;
    F(X, VY)  = -s * dt;
    F(Y, PSI) = (x_(VX) * c - x_(VY) * s) * dt;
    F(Y, VX)  = s * dt;
    F(Y, VY)  = c * dt;
    F(PSI, BGZ) = -dt;
    F(VX, VY)  = psidot_eff * dt;
    F(VX, BAX) = -dt;
    F(VX, BGZ) = (-2.0 * psidot_eff * imu_offset_x_ - x_(VY)) * dt;
    F(VY, VX)  = -psidot_eff * dt;
    F(VY, BAY) = -dt;
    F(VY, BGZ) = (-2.0 * psidot_eff * imu_offset_y_ + x_(VX)) * dt;
    F(PSIDOT, PSIDOT) = 0.0;
    F(PSIDOT, BGZ)    = -1.0;
    F(DELTA, DELTA)   = 0.0;

    P_ = F * P_ * F.transpose() + Q_;

    // 2. Wheel Odometry Update
    double avg_rotv = 0.5 * (v_rl + v_rr);
    double v_raw = tire_radius_ * avg_rotv;

    // Numerical safety: Ensure SW (Slip Weight) doesn't approach zero to avoid division by zero
    const double MIN_SW = 0.1;
    if (x_(SW) < MIN_SW) x_(SW) = MIN_SW;

    // Kinematic Lever-arm correction
    // Since Fr1A is on the centerline (Y=0) of the rear bumper, and rear axle is also on the centerline (Y=0),
    // there is NO Lever-arm effect on the X-velocity between Fr1A and rear axle!
    double expected_v_rear = x_(VX);
    double h_vx = expected_v_rear / x_(SW);
    double y = v_raw - h_vx;

    Eigen::Matrix<double, 1, STATE_DIM> H = Eigen::Matrix<double, 1, STATE_DIM>::Zero();
    H(VX) = 1.0 / x_(SW);
    H(SW) = -expected_v_rear / (x_(SW) * x_(SW));

    double v_wheel_scaled = v_raw * x_(SW);
    double slip_diff = std::abs(v_wheel_scaled - x_(VX));
    double R_wheel = wheel_speed_std_ * wheel_speed_std_;
    if (slip_diff > slip_threshold_) {
        R_wheel *= 10.0;
    }

    Eigen::Matrix<double, 1, 1> R_mat;
    R_mat(0, 0) = R_wheel;

    Eigen::Matrix<double, 1, 1> S = H * P_ * H.transpose() + R_mat;
    Eigen::Matrix<double, STATE_DIM, 1> K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity() - K * H) * P_;

    last_time_ += dt;

    if (!skip_buffer) {
        StateFrame frame;
        frame.timestamp = last_time_;
        frame.x = x_;
        frame.P = P_;
        frame.u = u;
        buffer_.addFrame(frame);
    }
}

void EkfCore::updateVision(double timestamp,
                            const Eigen::Vector3d& z,
                            const Eigen::Matrix3d& R) {
    StateFrame frame;
    if (!buffer_.getFrameAt(timestamp, frame)) {
        ROS_WARN_THROTTLE(1.0, "Could not find state in buffer for timestamp %f", timestamp);
        return;
    }

    // 1. Vision Update on old state
    Eigen::Matrix<double, 3, STATE_DIM> H = Eigen::Matrix<double, 3, STATE_DIM>::Zero();
    H(0, X) = 1.0;
    H(1, Y) = 1.0;
    H(2, PSI) = 1.0;

    Eigen::Vector3d h_x;
    h_x << frame.x(X), frame.x(Y), frame.x(PSI);
    Eigen::Vector3d y = z - h_x;
    y(2) = normalizeAngle(y(2));

    Eigen::Matrix3d S = H * frame.P * H.transpose() + R;
    Eigen::Matrix<double, STATE_DIM, 3> K = frame.P * H.transpose() * S.inverse();

    frame.x = frame.x + K * y;
    frame.P = (Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity() - K * H) * frame.P;

    // Divergence detection: Check trace of covariance
    if (frame.P.trace() > divergence_threshold_) {
        ROS_WARN("EKF Divergence! Re-initializing with latest Vision pose.");
        frame.x(X) = z(0);
        frame.x(Y) = z(1);
        frame.x(PSI) = z(2);
        // Keep velocity states as they are to avoid sudden stops, but reset covariance
        frame.P = P_init_;
    }

    // 2. Repropagate to current time
    std::vector<StateFrame> future_frames = buffer_.getFramesSince(frame.timestamp);

    x_ = frame.x;
    P_ = frame.P;
    last_time_ = frame.timestamp;

    // Clear and rebuild buffer starting with the corrected frame
    buffer_.clear();
    buffer_.addFrame(frame);

    for (const auto& f : future_frames) {
        double dt = f.timestamp - last_time_;
        if (dt > 0.0) {
            predictInternal(dt, f.u, true);

            StateFrame new_f = f;
            new_f.x = x_;
            new_f.P = P_;
            buffer_.addFrame(new_f);
        }
    }
}

} // namespace carmaker_localization
