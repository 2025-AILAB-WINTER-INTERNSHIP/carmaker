#include "carmaker_control/pid.hpp"

#include <algorithm>
#include <cmath>

namespace carmaker_control {

PID::PID(double kp, double ki, double kd, double output_min, double output_max)
{
  configure(kp, ki, kd, output_min, output_max);
}

void PID::configure(double kp, double ki, double kd, double output_min, double output_max)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  output_min_ = output_min;
  output_max_ = output_max;
  reset();
}

double PID::calculate(double target, double current, double dt)
{
  // ROS timer가 처음 돌거나 시간이 튀는 경우 dt가 0에 가까울 수 있다.
  // 이때 미분항이 과도하게 커지는 것을 막기 위해 명령을 0으로 둔다.
  if (dt <= 1e-4) {
    return 0.0;
  }

  error_ = target - current;
  const double p_term = kp_ * error_;
  const double d_term = kd_ * (error_ - prev_error_) / dt;

  // 적분항은 먼저 후보값을 계산한 뒤, 출력 제한을 더 밀어붙이는 방향이면
  // 누적하지 않는다. 정지 직전이나 포화 상태에서 wind-up이 쌓이는 것을 막는다.
  const double integral_candidate = integral_ + error_ * dt;
  const double i_term_candidate = ki_ * integral_candidate;
  const double output_candidate = p_term + i_term_candidate + d_term;

  const bool saturating_high = output_candidate > output_max_;
  const bool saturating_low = output_candidate < output_min_;
  const bool pushes_deeper =
      (saturating_high && error_ > 0.0) || (saturating_low && error_ < 0.0);

  if (!pushes_deeper) {
    integral_ = integral_candidate;
  }

  // ki가 0이 아닐 때 적분 상태 자체도 출력 범위 안에서 의미 있는 값으로 제한한다.
  // 이렇게 하면 포화 이후 다시 정상 구간으로 돌아올 때 복귀가 빨라진다.
  if (std::abs(ki_) > 1e-9) {
    const double integral_min = std::min(output_min_ / ki_, output_max_ / ki_);
    const double integral_max = std::max(output_min_ / ki_, output_max_ / ki_);
    integral_ = std::clamp(integral_, integral_min, integral_max);
  }

  const double output = std::clamp(p_term + ki_ * integral_ + d_term,
                                   output_min_, output_max_);
  prev_error_ = error_;
  return output;
}

void PID::reset()
{
  // 새 trajectory를 받거나 기어 segment를 바꿀 때 이전 오차 기억을 버린다.
  // 전진에서 후진으로 넘어갈 때 남아 있던 적분항이 튀는 것을 방지한다.
  error_ = 0.0;
  prev_error_ = 0.0;
  integral_ = 0.0;
}

}  // namespace carmaker_control
