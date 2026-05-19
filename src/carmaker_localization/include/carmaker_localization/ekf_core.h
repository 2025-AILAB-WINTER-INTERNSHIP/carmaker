#ifndef CARMAKER_LOCALIZATION_EKF_CORE_H
#define CARMAKER_LOCALIZATION_EKF_CORE_H

#include <Eigen/Dense>
#include <vector>
#include <mutex>

namespace carmaker_localization {

/**
 * @brief 11-Dimensional Full State EKF for Vehicle Localization
 * State Vector [11x1]: [x, y, vx, vy, ax, ay, yaw, yaw_rate, b_ax, b_ay, b_yaw_rate]
 */
enum StateIdx {
    X = 0, Y,          // Position (Global Frame)
    VX, VY,            // Velocity (Vehicle Frame)
    AX, AY,            // Acceleration (Vehicle Frame)
    YAW, YAW_RATE,     // Heading & Turn Rate (Global/Vehicle Frame)
    B_AX, B_AY,        // IMU Acceleration Bias
    B_YAW_RATE,        // IMU Gyro Bias
    STATE_DIM
};

struct StateFrame {
    double timestamp;
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
};

class EkfCore {
public:
    EkfCore();
    ~EkfCore() = default;

    // Initialization
    void initialize(double x, double y, double yaw, double timestamp, double vx = 0.0, double vy = 0.0);
    bool isInitialized() const { return is_initialized_; }

    // Parameters
    void setProcessNoise(const Eigen::MatrixXd& Q);

    // EKF Core Cycle
    void prediction(double timestamp);

    // Multi-Sensor Corrections
    void correctPose(double x, double y, double yaw, const Eigen::Matrix3d& R, double timestamp);
    void correctVelocity(double vx, double vy, const Eigen::Matrix2d& R, double timestamp);
    void correctImu(double ax, double ay, double yaw_rate, const Eigen::Matrix3d& R, double timestamp);

    // Getters
    StateFrame getState() const;

private:
    void handleTimeJump(double timestamp);

    // State
    Eigen::VectorXd x_; // [11x1]
    Eigen::MatrixXd P_; // [11x11]
    Eigen::MatrixXd Q_; // Process Noise [11x11]

    // Timing & History
    double last_time_;
    bool is_initialized_;
    mutable std::mutex mutex_;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_EKF_CORE_H
