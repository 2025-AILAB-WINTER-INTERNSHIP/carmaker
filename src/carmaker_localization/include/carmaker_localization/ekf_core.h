#ifndef CARMAKER_LOCALIZATION_EKF_CORE_H
#define CARMAKER_LOCALIZATION_EKF_CORE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
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

class EkfCore {
public:
    EkfCore();
    ~EkfCore() = default;

    void init(const Eigen::Matrix<double, STATE_DIM, 1>& x0,
              const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& P0);

    /**
     * @brief Prediction step using IMU and Wheel Odometry
     * @param dt Time step
     * @param u Input [ax, ay, wz, v_rl, v_rr, steer]
     */
    void predict(double dt, const Eigen::Matrix<double, 6, 1>& u);

    /**
     * @brief Update step using Absolute Pose from MapMatcher
     */
    void updateVision(double timestamp,
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
    double imu_acc_std_;
    double imu_gyro_std_;
    double wheel_speed_std_;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_EKF_CORE_H
