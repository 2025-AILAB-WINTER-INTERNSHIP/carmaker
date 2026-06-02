#ifndef CARMAKER_CONTROL_STANLEY_HPP
#define CARMAKER_CONTROL_STANLEY_HPP

namespace carmaker_control {

// 횡방향 오차와 헤딩 오차를 이용해 조향각을 계산하는 Stanley 컨트롤러.
// 여기서 반환되는 값은 타이어 조향각 기준이며, 실제 CarMaker 명령으로
// 보낼 때는 control_node에서 steering_ratio와 max_steer_command를 적용한다.
class Stanley {
public:
  Stanley() = default;
  Stanley(double k, double k_soft, double max_steer_angle,
          double cte_gain, double heading_gain);

  void configure(double k, double k_soft, double max_steer_angle,
                 double cte_gain, double heading_gain);
  double calculate(double cte, double heading_error, double velocity) const;
  double calculate(double cte, double heading_error, double velocity,
                   int direction, double reverse_cte_scale) const;

private:
  double k_{1.0};
  double k_soft_{0.1};
  double max_steer_angle_{0.6};
  double cte_gain_{1.0};
  double heading_gain_{1.0};
};

}  // namespace carmaker_control

#endif  // CARMAKER_CONTROL_STANLEY_HPP
