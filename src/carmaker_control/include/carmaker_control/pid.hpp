#ifndef CARMAKER_CONTROL_PID_HPP
#define CARMAKER_CONTROL_PID_HPP

namespace carmaker_control {

// 속도 제어용 PID 컨트롤러.
// 출력은 가속도 명령[m/s^2]으로 쓰며, output_min/output_max로
// 최대 감속/가속 범위를 제한한다.
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

#endif  // CARMAKER_CONTROL_PID_HPP
