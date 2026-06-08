#ifndef CARMAKER_LOCALIZATION_EKF_CORE_H
#define CARMAKER_LOCALIZATION_EKF_CORE_H

#include <Eigen/Dense>
#include <functional>
#include <string>
#include <vector>
#include <mutex>
#include <memory>

namespace carmaker_localization {

/**
 * @brief 12-Dimensional Full State EKF for Vehicle Localization
 * State Vector [12x1]: [x, y, vx, vy, ax, ay, yaw, yaw_rate, yaw_acc, b_ax, b_ay, b_yaw_rate]
 */
/**
 * @brief EKF 상태 변수별 기준 좌표 프레임 정의
 * - 위치(X, Y), 요각(YAW), 선속도(VX, VY)는 차량 기준점인 Rear Axle (후륜축) 기준
 * - 가속도 상태(AX, AY)는 센서 보정(Lever-arm) 및 자전거 기구학 예측식과의 정합성 (Rigid Body의 각가속도 dwz 항 배제를 통한 수치적 안정성 확보)을 위해 후륜 축(Rear Axle) 중심 기준
 */
enum StateIdx {
    X = 0, Y,          // Position (Global Frame) - Rear Axle (후륜축) 기준
    YAW,               // Heading (Global Frame)
    VX,                // Velocity (Vehicle Frame) - Rear Axle (후륜축) 기준
    YAW_RATE,          // Turn Rate (Vehicle Frame)
    B_YAW_RATE,        // IMU Gyro Bias
    STATE_DIM
};

using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
using StateMatrix = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;

struct StateFrame {
    double timestamp;
    StateVector x;
    StateMatrix P;
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
    void setProcessNoise(const StateMatrix& Q);
    void setWheelbase(double wheelbase);
    void setRearAxleOffset(double offset);
    void setImuOffsets(double offset_x, double offset_y);


    /**
     * @brief ROS 의존성 격리용 로그 콜백 등록.
     * @param info INFO 레벨 로그 핸들러 (예: [](const std::string& s){ NODELET_INFO_STREAM(s); })
     * @param warn WARN 레벨 로그 핸들러
     */
    void setLogCallbacks(std::function<void(const std::string&)> info,
                         std::function<void(const std::string&)> warn);

    // EKF Core Cycle
    void prediction(double timestamp, const PredictionInput& u = {});

    // Multi-Sensor Corrections
    void correctPose(double x, double y, double yaw, const Eigen::Matrix3d& R, double timestamp,
                     double max_pos_step = 999.0, double max_yaw_step = 999.0);
    void correctImu(double yaw_rate, double R_gyro, double timestamp);
    void correctWheel(double vx, double yaw_rate, const Eigen::Matrix2d& R, double timestamp);

    // Getters
    StateFrame getState() const;

private:
    void handleTimeJump(double timestamp);

    // Log callbacks (optional; no-op if unset — keeps EkfCore ROS-free)
    std::function<void(const std::string&)> log_info_;
    std::function<void(const std::string&)> log_warn_;

    // Configuration
    double wheelbase_;
    double rear_axle_offset_;
    double imu_offset_x_;
    double imu_offset_y_;


    // State
    StateVector x_; // [12x1]
    StateMatrix P_; // [12x12]
    StateMatrix Q_; // Process Noise [12x12]

    // Timing & History
    double last_time_;
    bool is_initialized_;
    mutable std::mutex mutex_;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_EKF_CORE_H
