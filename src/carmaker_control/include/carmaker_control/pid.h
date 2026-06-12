#ifndef CARMAKER_CONTROL_PID_H
#define CARMAKER_CONTROL_PID_H

namespace carmaker_control {

/**
 * @brief PID controller for longitudinal velocity tracking.
 * Outputs desired acceleration (m/s^2), bounded by output limits.
 */
class PID {
public:
  PID() = default;
  PID(double kp, double ki, double kd, double output_min, double output_max);

  void configure(double kp, double ki, double kd, double output_min, double output_max);
  double calculate(double target, double current, double dt);
  void reset();

private:
  double kp_{0.0};
  double ki_{0.0};
  double kd_{0.0};
  double output_min_{0.0};
  double output_max_{0.0};

  double error_{0.0};
  double prev_error_{0.0};
  double integral_{0.0};
};

}  // namespace carmaker_control

#endif  // CARMAKER_CONTROL_PID_H
