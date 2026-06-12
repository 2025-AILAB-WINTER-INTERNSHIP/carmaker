#include "carmaker_control/stanley.h"

#include <algorithm>
#include <cmath>

namespace carmaker_control {

Stanley::Stanley(double k, double k_soft, double max_steer_angle,
                 double cte_gain, double heading_gain)
{
  configure(k, k_soft, max_steer_angle, cte_gain, heading_gain);
}

void Stanley::configure(double k, double k_soft, double max_steer_angle,
                        double cte_gain, double heading_gain)
{
  k_ = k;
  k_soft_ = k_soft;
  max_steer_angle_ = max_steer_angle;
  cte_gain_ = cte_gain;
  heading_gain_ = heading_gain;
}

double Stanley::calculate(double cte, double heading_error, double velocity,
                          int direction, double reverse_cte_scale) const
{
  // Cross-track error term scale (attenuated in reverse if needed)
  const double cte_scale = (direction < 0) ? reverse_cte_scale : 1.0;
  
  // atan2 denominator includes k_soft to avoid extreme outputs at near-zero velocity
  const double cte_term = cte_scale * cte_gain_ * std::atan2(k_ * cte, std::abs(velocity) + k_soft_);

  // Reverse heading error sign to compensate for reverse kinematics
  const double heading_sign = (direction < 0) ? -1.0 : 1.0;

  const double steer_angle = (heading_sign * heading_gain_ * heading_error) + cte_term;
  return std::clamp(steer_angle, -max_steer_angle_, max_steer_angle_);
}

}  // namespace carmaker_control
