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
  // Stanley의 횡오차 항은 속도가 낮을수록 커진다.
  // k_soft를 더해 정지 근처에서 atan2 분모가 0에 가까워지는 상황을 완화한다.
  const double cte_term = cte_gain_ * std::atan2(k_ * cte, std::abs(velocity) + k_soft_);

  // heading_error는 경로 방향과 차량 yaw의 차이, cte_term은 경로 좌우 오차 보정이다.
  // 최종 조향각은 차량 물리 한계 안으로 제한한다.
  const double steer_angle = heading_gain_ * heading_error + cte_term;
  return std::clamp(steer_angle, -max_steer_angle_, max_steer_angle_);
}

}  // namespace carmaker_control
