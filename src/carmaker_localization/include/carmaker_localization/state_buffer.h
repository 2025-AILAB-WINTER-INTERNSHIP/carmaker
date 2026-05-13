#ifndef CARMAKER_LOCALIZATION_STATE_BUFFER_H
#define CARMAKER_LOCALIZATION_STATE_BUFFER_H

#include <Eigen/Core>
#include <map>
#include <vector>

namespace carmaker_localization {

struct StateFrame {
    double timestamp;
    Eigen::Matrix<double, 11, 1> x;
    Eigen::Matrix<double, 11, 11> P;
    Eigen::Matrix<double, 6, 1> u; // IMU (acc_x, acc_y, gyro_z) + Wheel (v_rl, v_rr, steer)
};

/**
 * @brief Ring buffer for state history to support delay compensation.
 */
class StateBuffer {
public:
    explicit StateBuffer(double duration_sec = 0.5);

    void addFrame(const StateFrame& frame);

    /**
     * @brief Find the closest frame at or before timestamp
     */
    bool getFrameAt(double timestamp, StateFrame& frame) const;

    /**
     * @brief Get all frames after specific timestamp (for re-propagation)
     */
    std::vector<StateFrame> getFramesSince(double timestamp) const;

    void clear();

private:
    double duration_;
    std::map<double, StateFrame> buffer_; // Ordered by timestamp
};

} // namespace carmaker_localization

#endif // CARMAKER_LOCALIZATION_STATE_BUFFER_H
