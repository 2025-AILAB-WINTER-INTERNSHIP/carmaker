#include "carmaker_control/stanley.hpp"

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

double Stanley::calculate(double cte, double heading_error, double velocity) const
{
  return calculate(cte, heading_error, velocity, 1, 1.0);
}

double Stanley::calculate(double cte, double heading_error, double velocity,
                          int direction, double reverse_cte_scale) const
{
  // Stanley의 횡오차 항은 속도가 낮을수록 커진다.
  // k_soft를 더해 정지 근처에서 atan2 분모가 0에 가까워지는 상황을 완화한다.
  const double cte_scale = direction < 0 ? reverse_cte_scale : 1.0;
  const double cte_term = cte_scale * cte_gain_ *
                          std::atan2(k_ * cte, std::abs(velocity) + k_soft_);
  const double heading_sign = direction < 0 ? -1.0 : 1.0;

  // heading_error는 차량 yaw와 trajectory pose yaw를 맞추는 항이다.
  // 후진에서는 같은 조향각이 yaw를 전진과 반대로 변화시키므로 heading 항을 뒤집는다.
  // lateral error 항은 path 좌우 기준을 유지해야 후진 path 반대 방향으로 꺾지 않는다.
  const double steer_angle = heading_sign * heading_gain_ * heading_error + cte_term;
  return std::clamp(steer_angle, -max_steer_angle_, max_steer_angle_);
}

}  // namespace carmaker_control
