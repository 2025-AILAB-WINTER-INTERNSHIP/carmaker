#ifndef CARMAKER_CONTROL_STANLEY_H
#define CARMAKER_CONTROL_STANLEY_H

namespace carmaker_control {

/**
 * @brief Stanley lateral controller.
 * Computes steer angle at the tire level based on cross-track error and heading error.
 */
class Stanley {
public:
  Stanley() = default;
  Stanley(double k, double k_soft, double max_steer_angle,
          double cte_gain, double heading_gain);

  void configure(double k, double k_soft, double max_steer_angle,
                 double cte_gain, double heading_gain);

  double calculate(double cte, double heading_error, double velocity,
                   int direction = 1, double reverse_cte_scale = 1.0) const;

private:
  double k_{1.0};
  double k_soft_{0.1};
  double max_steer_angle_{0.6};
  double cte_gain_{1.0};
  double heading_gain_{1.0};
};

}  // namespace carmaker_control

#endif  // CARMAKER_CONTROL_STANLEY_H
