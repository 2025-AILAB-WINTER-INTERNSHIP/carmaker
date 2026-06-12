#include "carmaker_control/pid.h"

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
  // Avoid division by zero/extreme values in D-term if dt is very small
  if (dt <= 1e-4) {
    return 0.0;
  }

  error_ = target - current;
  const double p_term = kp_ * error_;
  const double d_term = kd_ * (error_ - prev_error_) / dt;

  // Anti-windup: do not accumulate integral term if output candidate is saturated
  // and integrating further would push it deeper into saturation.
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

  // Bound integral state itself to speed up recovery from saturation
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
  // Reset error memory to prevent integral/derivative spikes during state transitions
  error_ = 0.0;
  prev_error_ = 0.0;
  integral_ = 0.0;
}

}  // namespace carmaker_control
