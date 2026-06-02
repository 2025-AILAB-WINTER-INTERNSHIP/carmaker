#ifndef CARMAKER_LOCALIZATION_EKF_CORE_H
#define CARMAKER_LOCALIZATION_EKF_CORE_H

#include <Eigen/Dense>
#include <vector>
#include <mutex>
#include <memory>

namespace carmaker_localization {

/**
 * @brief 11-Dimensional Full State EKF for Vehicle Localization
 * State Vector [11x1]: [x, y, vx, vy, ax, ay, yaw, yaw_rate, b_ax, b_ay, b_yaw_rate]
 */
/**
 * @brief EKF 상태 변수별 기준 좌표 프레임 정의
 * - 위치(X, Y), 요각(YAW), 선속도(VX, VY)는 차량 기준점인 Fr1A(후방 범퍼) 기준
 * - 가속도 상태(AX, AY)는 센서 보정(Lever-arm) 및 자전거 기구학 예측식과의 정합성 (Rigid Body의 각가속도 dwz 항 배제를 통한 수치적 안정성 확보)을 위해 후륜 축(Rear Axle) 중심 기준
 */
enum StateIdx {
    X = 0, Y,          // Position (Global Frame) - Fr1A(후방 범퍼) 기준
    VX, VY,            // Velocity (Vehicle Frame) - Fr1A(후방 범퍼) 기준 (vy = -yaw_rate * offset)
    AX, AY,            // Acceleration (Vehicle Frame) - 후륜 축(Rear Axle) 기준 (ay = vx * yaw_rate)
    YAW, YAW_RATE,     // Heading & Turn Rate (강체 전체 공통 - 모든 위치에서 동일)
    B_AX, B_AY,        // IMU Acceleration Bias
    B_YAW_RATE,        // IMU Gyro Bias
    STATE_DIM
};

struct StateFrame {
    double timestamp;
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
};

/**
 * @brief Control inputs for the EKF prediction step.
 */
struct PredictionInput {
    double dt = 0.0;                ///< Time step [s] (filled by EkfCore)
    double steering_angle = 0.0;    ///< Front tire road-wheel angle δ [rad]
    double rear_wheel_speed = 0.0;  ///< Rear axle longitudinal speed v [m/s]
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
    void setWheelbase(double wheelbase);
    void setRearAxleOffset(double offset);

    // EKF Core Cycle
    void prediction(double timestamp, const PredictionInput& u = {});

    // Multi-Sensor Corrections
    void correctPose(double x, double y, double yaw, const Eigen::Matrix3d& R, double timestamp);
    void correctImu(double ax, double ay, double yaw_rate, const Eigen::Matrix3d& R, double timestamp);
    void correctWheel(double vx, double vy, double yaw_rate, const Eigen::Matrix3d& R, double timestamp);

    // Getters
    StateFrame getState() const;

private:
    void handleTimeJump(double timestamp);

    // Configuration
    double wheelbase_;
    double rear_axle_offset_;

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
