#ifndef CARMAKER_LOCALIZATION_EKF_CORE_H
#define CARMAKER_LOCALIZATION_EKF_CORE_H

#include <Eigen/Dense>
#include <functional>
#include <string>
#include <mutex>

namespace carmaker_localization {

/**
 * @brief EKF 상태 변수별 기준 좌표 프레임 정의
 *
 * 상태 벡터: [X, Y, YAW, VX, YAW_RATE, B_YAW_RATE]
 * - 위치(X, Y), 요각(YAW), 종방향 속도(VX)는 후륜축 기준
 * - YAW_RATE는 차량 좌표계 z축 회전율, B_YAW_RATE는 IMU z축 자이로 bias
 */

// EKF 상태 인덱스
enum StateIdx {
    X = 0, Y,          // Position (Global Frame) - Rear Axle (후륜축) 기준
    YAW,               // Heading (Global Frame)
    VX,                // Velocity (Vehicle Frame) - Rear Axle (후륜축) 기준
    YAW_RATE,          // Turn Rate (Vehicle Frame)
    B_YAW_RATE,        // IMU Gyro Bias
    STATE_DIM
};

// EKF 상태(6x1) 및 공분산(6x6) 타입 정의
using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
using StateMatrix = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;

// EKF 상태 프레임 구조체
struct StateFrame {
    double timestamp;
    StateVector x;
    StateMatrix P;
};

// EKF Core 클래스 정의
class EkfCore {
public:
    EkfCore();
    ~EkfCore() = default;

    // Initialization
    void initialize(double x, double y, double yaw, double timestamp, double vx = 0.0);
    bool isInitialized() const;

    // Parameters
    void setProcessNoise(const StateMatrix& Q);

    /**
     * @brief ROS 의존성 격리용 warning 로그 콜백 등록.
     * @param warn WARN 레벨 로그 핸들러
     */
    void setWarnLogCallback(std::function<void(const std::string&)> warn);

    // EKF Core Cycle
    void prediction(double timestamp, double vx_wheel = 0.0, double ax_imu = 0.0);

    // Multi-Sensor Corrections
    void correctPose(double x, double y, double yaw, const Eigen::Matrix3d& R, double timestamp,
                     double max_pos_step = 999.0, double max_yaw_step = 999.0);
    void correctImu(double yaw_rate, double R_gyro, double timestamp);
    void correctWheel(double vx, double yaw_rate, const Eigen::Matrix2d& R, double timestamp);

    // 상태 조회
    StateFrame getState() const;

private:
    // mutex_를 잡은 상태에서만 호출하는 내부 예측 루틴.
    // public prediction()과 각 correction()이 동일한 시간 전파 로직을 공유한다.
    bool predictUnlocked(double timestamp, double vx_wheel = 0.0, double ax_imu = 0.0);

    // correction timestamp가 현재 상태보다 미래이면 그 시각까지 prediction을 수행한다.
    // 이미 지나간 측정(out-of-sequence)은 state buffer가 없으므로 적용하지 않는다.
    bool advanceToMeasurementTime(double timestamp, const char* source);

    // EKF에서 각도 상태는 원형 변수이므로 상태 갱신 후 항상 [-pi, pi]로 접는다.
    static double normalizeAngle(double angle);

    // P는 이론적으로 대칭이어야 하지만 부동소수점 연산 후 작은 비대칭이 생길 수 있다.
    void stabilizeCovariance();

    void handleTimeJump(double timestamp);

    // 로그 콜백(선택 사항). 설정하지 않으면 아무 동작도 하지 않아 EkfCore의 ROS 비의존성을 유지한다.
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
