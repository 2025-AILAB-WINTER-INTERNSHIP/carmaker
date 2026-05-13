#ifndef CARMAKER_LOCALIZATION_EKF_CORE_H
#define CARMAKER_LOCALIZATION_EKF_CORE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <cmath>
#include "carmaker_localization/state_buffer.h"

namespace carmaker_localization {

/**
 * @brief State Index for 11-state 2D EKF
 * x = [x, y, psi, vx, vy, psidot, bax, bay, bgz, delta, sw]^T
 */
enum StateIdx {
    X = 0, Y = 1, PSI = 2,
    VX = 3, VY = 4, PSIDOT = 5,
    BAX = 6, BAY = 7, BGZ = 8,
    DELTA = 9, SW = 10,
    STATE_DIM = 11
};

/**
 * @brief Utility for angle normalization to [-PI, PI]
 */
inline double normalizeAngle(double angle) {
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

class EkfCore {
public:
    explicit EkfCore(double buffer_duration = 0.5);
    ~EkfCore() = default;

    void init(const Eigen::Matrix<double, STATE_DIM, 1>& x0,
                const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& P0);

    /**
     * @brief Prediction step using IMU and Wheel Odometry
     * @param dt Time step
     * @param u Input [ax, ay, wz, v_rl, v_rr, steer]
     */
    void prediction(double dt, const Eigen::Matrix<double, 6, 1>& u);

    void setParameters(double tire_radius, double slip_threshold, double wheel_speed_std, const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& Q, 
                        double rear_axle_offset_x = 0.5, double imu_offset_x = 1.35, double imu_offset_y = 0.0, double imu_offset_z = 0.5) {
        tire_radius_ = tire_radius;
        slip_threshold_ = slip_threshold;
        wheel_speed_std_ = wheel_speed_std;
        Q_ = Q;
        rear_axle_offset_x_ = rear_axle_offset_x;
        imu_offset_x_ = imu_offset_x;
        imu_offset_y_ = imu_offset_y;
        imu_offset_z_ = imu_offset_z;
    }

    void setDivergenceThreshold(double threshold) {
        divergence_threshold_ = threshold;
    }

    /**
     * @brief Update step using Absolute Pose from MapMatcher
     */
    void correction(double timestamp,
                    const Eigen::Vector3d& z,
                    const Eigen::Matrix3d& R);

    // Getters
    Eigen::Matrix<double, STATE_DIM, 1> getState() const { return x_; }
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> getCovariance() const { return P_; }
    double getTimestamp() const { return last_time_; }

private:
    Eigen::Matrix<double, STATE_DIM, 1> x_;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q_;

    double last_time_;
    StateBuffer buffer_;

    // Parameters
    double tire_radius_ = 0.31;
    double slip_threshold_ = 0.5;
    double wheel_speed_std_ = 0.05;
    double divergence_threshold_ = 100.0; // Trace(P) limit
    double rear_axle_offset_x_ = 0.5;
    double imu_offset_x_ = 1.35;
    double imu_offset_y_ = 0.0;
    double imu_offset_z_ = 0.5;

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_init_; // Store for recovery

    void predictInternal(double dt, const Eigen::Matrix<double, 6, 1>& u, bool skip_buffer);
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_EKF_CORE_H
