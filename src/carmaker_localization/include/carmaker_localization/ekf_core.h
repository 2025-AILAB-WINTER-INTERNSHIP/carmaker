#ifndef CARMAKER_LOCALIZATION_EKF_CORE_H
#define CARMAKER_LOCALIZATION_EKF_CORE_H

#include <Eigen/Dense>
#include <functional>
#include <string>
#include <mutex>

namespace carmaker_localization {

/**
 * @brief 2D 차량 위치 추정을 위한 6차원 상태 기반 EKF.
 * 상태 벡터: [X, Y, YAW, VX, YAW_RATE, B_YAW_RATE]
 * 위치와 속도 상태는 후륜축 기준으로 표현한다.
 */
/**
 * @brief EKF 상태 변수별 기준 좌표 프레임 정의
 * - 위치(X, Y), 요각(YAW), 종방향 속도(VX)는 후륜축(Rear Axle) 기준
 * - YAW_RATE와 B_YAW_RATE는 차량 좌표계의 z축 회전율 기준
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

class EkfCore {
public:
    EkfCore();
    ~EkfCore() = default;

    // Initialization
    void initialize(double x, double y, double yaw, double timestamp, double vx = 0.0);
    bool isInitialized() const { return is_initialized_; }

    // Parameters
    void setProcessNoise(const StateMatrix& Q);

    /**
     * @brief ROS 의존성 격리용 로그 콜백 등록.
     * @param info INFO 레벨 로그 핸들러 (예: [](const std::string& s){ NODELET_INFO_STREAM(s); })
     * @param warn WARN 레벨 로그 핸들러
     */
    void setLogCallbacks(std::function<void(const std::string&)> info,
                         std::function<void(const std::string&)> warn);

  // EKF Core Cycle
    void prediction(double timestamp);

    // Multi-Sensor Corrections
    void correctPose(double x, double y, double yaw, const Eigen::Matrix3d& R, double timestamp,
                     double max_pos_step = 999.0, double max_yaw_step = 999.0);
    void correctImu(double yaw_rate, double R_gyro, double timestamp);
    void correctWheel(double vx, double yaw_rate, const Eigen::Matrix2d& R, double timestamp);

    // 상태 조회
    StateFrame getState() const;

private:
    void handleTimeJump(double timestamp);

    // 로그 콜백(선택 사항). 설정하지 않으면 아무 동작도 하지 않아 EkfCore의 ROS 비의존성을 유지한다.
    std::function<void(const std::string&)> log_info_;
    std::function<void(const std::string&)> log_warn_;

    // State
    StateVector x_;
    StateMatrix P_;
    StateMatrix Q_;

   // Timing & History
    double last_time_;
    bool is_initialized_;
    mutable std::mutex mutex_;
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_EKF_CORE_H
