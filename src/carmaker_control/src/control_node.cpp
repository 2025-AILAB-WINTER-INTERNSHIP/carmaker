#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <carmaker_msgs/Control_Signal.h>
#include <carmaker_msgs/DynamicsInfo.h>
#include <carmaker_msgs/TrajectoryPath.h>
#include <carmaker_msgs/TrajectoryPoint.h>

#include "carmaker_control/pid.hpp"
#include "carmaker_control/stanley.hpp"

namespace carmaker_control {
namespace {
constexpr double kPi = 3.14159265358979323846;

// 설정 파일에서는 조향 한계를 사람이 읽기 쉬운 degree로 두고,
// 실제 제어 계산은 radian으로 통일한다.
double deg2rad(double degree)
{
  return degree * kPi / 180.0;
}

double rad2deg(double radian)
{
  return radian * 180.0 / kPi;
}

// 내부 direction 표현(1/-1)을 CarMaker gear 값으로 변환한다.
// gear 숫자는 프로젝트마다 다를 수 있으므로 yaml에서 바꿀 수 있게 둔다.
int directionToGear(int direction, int drive_gear, int reverse_gear)
{
  return direction >= 0 ? drive_gear : reverse_gear;
}

double normalizeAngle(double angle)
{
  // yaw 오차는 항상 [-pi, pi]로 제한해 Stanley heading term이 가장 짧은 방향을 보게 한다.
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double quaternionToYaw(const geometry_msgs::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

enum SpeedSelectionReason {
  kSpeedReasonAllZeroStop = 0,
  kSpeedReasonLookaheadSpeed = 1,
  kSpeedReasonMinTrackingSpeed = 2,
  kSpeedReasonMinCreepSpeed = 3,
  kSpeedReasonEndpointStop = 4
};

const char* speedSelectionReasonString(int reason)
{
  switch (reason) {
    case kSpeedReasonAllZeroStop: return "all_zero_stop";
    case kSpeedReasonLookaheadSpeed: return "lookahead_speed";
    case kSpeedReasonMinTrackingSpeed: return "min_tracking_speed";
    case kSpeedReasonMinCreepSpeed: return "min_creep_speed";
    case kSpeedReasonEndpointStop: return "endpoint_stop";
  }
  return "unknown";
}
}  // namespace

// /planning/trajectory와 현재 차량 상태를 받아 /carmaker/control_signal을 내보내는 제어 노드.
//
// 실제 파이프라인은 segmentation -> localization -> planning -> control 순서다.
// 기본 current state source는 localization이 publish한 Odometry이며,
// localization 없이 planning/control 성능만 확인할 때는 CarMaker DynamicsInfo GT를 선택할 수 있다.
class ControlNode {
public:
  ControlNode();

private:
  struct Pose2D {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  // controller 내부에서 쓰는 trajectory point.
  // target_speed는 항상 절댓값이며, direction이 실제 주행 기어를 표현한다.
  struct PathPoint {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double target_speed{0.0};
    double curvature{0.0};
    int direction{1};
  };

  struct ActiveTrajectory {
    std::vector<PathPoint> points;

    bool empty() const { return points.empty(); }
    std::size_t size() const { return points.size(); }
    int direction() const {
      if (points.empty()) return 1;
      return points.front().direction < 0 ? -1 : 1;
    }
  };

  struct TargetSpeedDecision {
    double target_speed{0.0};
    double raw_target_speed{0.0};
    int reason{kSpeedReasonEndpointStop};
  };

  void loadParameters();
  void trajectoryCallback(const carmaker_msgs::TrajectoryPathConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void dynamicsCallback(const carmaker_msgs::DynamicsInfoConstPtr& msg);
  void controlTimerCallback(const ros::TimerEvent& event);

  PathPoint makePathPoint(const carmaker_msgs::TrajectoryPoint& msg, int direction) const;
  int trajectoryDirection(const carmaker_msgs::TrajectoryPath& msg) const;

  bool getCurrentState(Pose2D& pose, double& signed_speed) const;
  std::size_t findNearestIndex(const ActiveTrajectory& trajectory,
                               const Pose2D& pose,
                               std::size_t previous_index) const;
  std::size_t findLookaheadIndex(const ActiveTrajectory& trajectory,
                                 std::size_t nearest_index,
                                 double lookahead_distance) const;
  double computePreviewCurvature(const ActiveTrajectory& trajectory,
                                 std::size_t nearest_index,
                                 std::size_t& preview_index) const;
  double computeCurvatureFeedforward(double preview_curvature, int direction) const;
  double computeSteeringCommand(const Pose2D& pose,
                                const PathPoint& feedback_reference,
                                double preview_curvature,
                                double speed,
                                int direction) const;
  TargetSpeedDecision selectTargetSpeed(const ActiveTrajectory& trajectory,
                                        std::size_t nearest_index,
                                        std::size_t target_index,
                                        double distance_to_end) const;
  double computeRemainingDistance(const std::vector<PathPoint>& path,
                                  std::size_t nearest_index,
                                  const Pose2D& pose,
                                  int direction) const;
  static TargetSpeedDecision selectTargetSpeedForPath(const std::vector<PathPoint>& path,
                                                      std::size_t nearest_index,
                                                      std::size_t target_index,
                                                      double distance_to_end,
                                                      double min_tracking_speed,
                                                      double min_creep_speed,
                                                      double max_target_speed,
                                                      double stop_velocity_epsilon,
                                                      double arrival_slow_distance);

  void advertiseDebugTopics();
  void publishDebugTelemetry(const Pose2D& pose,
                             const Pose2D& tracking_pose,
                             double signed_speed,
                             const ActiveTrajectory* trajectory,
                             std::size_t nearest_index_rear,
                             std::size_t nearest_index_tracking,
                             std::size_t target_index,
                             double target_speed,
                             double lookahead,
                             int tracking_state,
                             double steer_command,
                             int stop_reason = 0,
                             double raw_target_speed = 0.0,
                             int speed_selection_reason = kSpeedReasonEndpointStop,
                             std::size_t preview_index = std::numeric_limits<std::size_t>::max(),
                             double preview_distance = 0.0,
                             double preview_curvature = 0.0);
  void publishStopReason(int stop_reason);
  void publishPoseDebug(const ros::Publisher& publisher,
                        double x,
                        double y,
                        double yaw,
                        const ros::Time& stamp) const;
  void publishActiveSegmentPath(const ActiveTrajectory& trajectory, const ros::Time& stamp);
  void publishControl(int direction, double steer_command, double accel_command);
  void publishStop(int direction, double steer_command = 0.0);
  void resetIdleRelease();
  bool publishIdleStopUntilRelease(const ros::Time& now);

  static double distance2D(const Pose2D& pose, const PathPoint& point);
  static double distance2D(const PathPoint& a, const PathPoint& b);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber trajectory_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber dynamics_sub_;
  ros::Publisher control_pub_;
  ros::Timer control_timer_;

  ros::Publisher debug_current_pose_pub_;
  ros::Publisher debug_current_control_pose_pub_;
  ros::Publisher debug_nearest_pose_pub_;
  ros::Publisher debug_nearest_control_pose_pub_;
  ros::Publisher debug_lookahead_pose_pub_;
  ros::Publisher debug_curvature_preview_pose_pub_;
  ros::Publisher debug_active_path_pub_;
  ros::Publisher debug_current_speed_pub_;
  ros::Publisher debug_target_speed_pub_;
  ros::Publisher debug_raw_target_speed_pub_;
  ros::Publisher debug_speed_error_pub_;
  ros::Publisher debug_steer_command_pub_;
  ros::Publisher debug_curvature_ff_pub_;
  ros::Publisher debug_curvature_preview_pub_;
  ros::Publisher debug_curvature_preview_distance_pub_;
  ros::Publisher debug_steer_saturated_pub_;
  ros::Publisher debug_cte_pub_;
  ros::Publisher debug_heading_error_pub_;
  ros::Publisher debug_lookahead_pub_;
  ros::Publisher debug_segment_index_pub_;
  ros::Publisher debug_segment_count_pub_;
  ros::Publisher debug_nearest_index_pub_;
  ros::Publisher debug_target_index_pub_;
  ros::Publisher debug_curvature_preview_index_pub_;
  ros::Publisher debug_distance_to_end_pub_;
  ros::Publisher debug_trajectory_age_pub_;
  ros::Publisher debug_direction_pub_;
  ros::Publisher debug_tracking_state_pub_;
  ros::Publisher debug_stop_reason_pub_;
  ros::Publisher debug_speed_selection_reason_pub_;
  ros::Publisher debug_speed_selection_reason_text_pub_;

  mutable std::mutex odom_mutex_;
  nav_msgs::Odometry latest_odom_;
  ros::Time last_odom_time_;
  bool odom_received_{false};

  mutable std::mutex dynamics_mutex_;
  carmaker_msgs::DynamicsInfo latest_dynamics_;
  ros::Time last_dynamics_time_;
  bool dynamics_received_{false};

  mutable std::mutex trajectory_mutex_;
  ActiveTrajectory active_trajectory_;
  ros::Time last_trajectory_time_;
  std::size_t nearest_index_{0};
  bool trajectory_received_{false};

  PID speed_pid_;
  Stanley stanley_;

  std::string trajectory_topic_;
  std::string odom_topic_;
  std::string dynamics_topic_;
  std::string control_topic_;
  std::string state_source_{"odom"};
  std::string debug_topic_prefix_{"/control/debug"};
  std::string debug_frame_id_{"Fr0"};

  bool publish_stop_without_path_{true};
  bool publish_debug_{true};
  double control_rate_{30.0};
  double odom_timeout_{0.5};
  double dynamics_timeout_{0.5};
  double active_trajectory_timeout_{1.0};
  double stop_trajectory_velocity_epsilon_{1e-6};

  double min_tracking_speed_{0.2};
  double min_creep_speed_{0.1};
  double arrival_slow_distance_{0.5};
  double max_target_speed_{0.7};
  double lookahead_distance_{0.35};
  double lookahead_time_{0.2};
  double min_lookahead_distance_{0.2};
  double max_lookahead_distance_{0.8};
  double curvature_preview_distance_{0.8};
  double curvature_preview_window_{0.4};
  int nearest_search_back_{20};
  int nearest_search_ahead_{120};

  double max_accel_{0.8};
  double max_decel_{1.5};
  double max_gas_{0.35};
  double max_brake_{1.0};
  double stop_brake_{0.4};

  int drive_gear_{1};
  int neutral_gear_{0};
  int reverse_gear_{-1};

  double wheelbase_{2.97};
  double steering_ratio_{9.0};
  double max_steer_command_{4.5};
  double steering_command_sign_{1.0};
  double reverse_steering_scale_{1.0};
  double curvature_ff_gain_{0.6};
  double reverse_curvature_ff_sign_{-1.0};
  double last_steer_command_{0.0};
  double idle_release_initial_steer_{0.0};
  bool steer_release_after_idle_{false};
  bool idle_release_active_{false};
  bool idle_control_released_{false};
  double release_duration_{1.0};
  ros::Time idle_release_start_time_;

  ros::Time last_control_time_;
  ros::Time last_debug_path_time_;
  double debug_path_period_{1.0};
};

ControlNode::ControlNode()
  : nh_(),
    pnh_("~")
{
  loadParameters();

  // planning 결과와 현재 차량 상태를 받고 최종 CarMaker 제어 명령을 publish한다.
  trajectory_sub_ = nh_.subscribe(trajectory_topic_, 1, &ControlNode::trajectoryCallback, this);
  if (state_source_ == "dynamics") {
    dynamics_sub_ = nh_.subscribe(dynamics_topic_, 30, &ControlNode::dynamicsCallback, this);
  } else {
    odom_sub_ = nh_.subscribe(odom_topic_, 30, &ControlNode::odomCallback, this);
  }
  control_pub_ = nh_.advertise<carmaker_msgs::Control_Signal>(control_topic_, 10);
  advertiseDebugTopics();

  // 제어 루프는 trajectory callback과 독립적으로 일정 주기로 돈다.
  // trajectory가 latched publisher이므로 노드를 나중에 켜도 마지막 계획 경로를 받을 수 있다.
  control_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, control_rate_)),
                                   &ControlNode::controlTimerCallback, this);

  const std::string state_topic = state_source_ == "dynamics" ? dynamics_topic_ : odom_topic_;
  ROS_INFO("carmaker_control_node ready. trajectory=%s state_source=%s state_topic=%s control=%s",
           trajectory_topic_.c_str(), state_source_.c_str(),
           state_topic.c_str(), control_topic_.c_str());
}

void ControlNode::loadParameters()
{
  // 기본 current state source는 localization odometry다.
  // localization 없이 planning/control만 확인할 때 control/state_source=dynamics로 두면
  // /carmaker/dynamic_info의 GT pose/speed를 직접 사용한다.
  pnh_.param<std::string>("topics/subscribe/trajectory", trajectory_topic_, "/planning/trajectory");
  pnh_.param<std::string>("topics/subscribe/odom", odom_topic_, "/localization/odom");
  pnh_.param<std::string>("topics/subscribe/dynamics", dynamics_topic_, "/carmaker/dynamic_info");
  pnh_.param<std::string>("topics/publish/control", control_topic_, "/carmaker/control_signal");

  pnh_.param("setting/publish_stop_without_path", publish_stop_without_path_, true);
  pnh_.param<std::string>("control/state_source", state_source_, "odom");
  if (state_source_ == "gt") {
    state_source_ = "dynamics";
  } else if (state_source_ == "localization") {
    state_source_ = "odom";
  } else if (state_source_ != "odom" && state_source_ != "dynamics") {
    ROS_WARN("Unknown control/state_source='%s'. Falling back to 'odom'.",
             state_source_.c_str());
    state_source_ = "odom";
  }

  // active_trajectory_timeout은 local planner가 active trajectory publish를 멈춘 뒤
  // control이 neutral stop으로 돌아가기 위한 안전장치다.
  pnh_.param("control/rate", control_rate_, 30.0);
  pnh_.param("control/odom_timeout", odom_timeout_, 0.5);
  pnh_.param("control/dynamics_timeout", dynamics_timeout_, 0.5);
  pnh_.param("control/active_trajectory_timeout", active_trajectory_timeout_, 1.0);
  pnh_.param("control/stop_trajectory_velocity_epsilon",
             stop_trajectory_velocity_epsilon_, 1e-6);
  active_trajectory_timeout_ = std::max(0.0, active_trajectory_timeout_);
  stop_trajectory_velocity_epsilon_ = std::max(0.0, stop_trajectory_velocity_epsilon_);

  pnh_.param("control/min_tracking_speed", min_tracking_speed_, 0.2);
  pnh_.param("control/min_creep_speed", min_creep_speed_, 0.1);
  pnh_.param("control/arrival_slow_distance", arrival_slow_distance_, 0.5);
  pnh_.param("control/max_target_speed", max_target_speed_, 0.7);
  min_tracking_speed_ = std::max(0.0, min_tracking_speed_);
  min_creep_speed_ = std::max(0.0, min_creep_speed_);
  arrival_slow_distance_ = std::max(0.0, arrival_slow_distance_);
  max_target_speed_ = std::max(0.0, max_target_speed_);
  if (min_creep_speed_ > min_tracking_speed_) {
    ROS_WARN("control/min_creep_speed=%.3f is greater than min_tracking_speed=%.3f; "
             "near-endpoint creep may exceed the normal tracking floor.",
             min_creep_speed_, min_tracking_speed_);
  }
  if (max_target_speed_ < min_tracking_speed_) {
    ROS_WARN("control/max_target_speed=%.3f is lower than min_tracking_speed=%.3f; "
             "target speed will be clamped below the configured tracking floor.",
             max_target_speed_, min_tracking_speed_);
  }

  // lookahead는 속도가 높을수록 조금 멀리 보도록 base + time * speed 형태로 계산한다.
  // 단, 너무 가까워 떨리거나 너무 멀어 코너를 뭉개지 않도록 min/max로 제한한다.
  pnh_.param("control/lookahead_distance", lookahead_distance_, 0.35);
  pnh_.param("control/lookahead_time", lookahead_time_, 0.2);
  pnh_.param("control/min_lookahead_distance", min_lookahead_distance_, 0.2);
  pnh_.param("control/max_lookahead_distance", max_lookahead_distance_, 0.8);
  pnh_.param("control/curvature_preview_distance", curvature_preview_distance_, 0.8);
  pnh_.param("control/curvature_preview_window", curvature_preview_window_, 0.4);
  curvature_preview_distance_ = std::max(0.0, curvature_preview_distance_);
  curvature_preview_window_ = std::max(0.0, curvature_preview_window_);

  // nearest search는 이전 index 주변만 훑어 경로가 겹칠 때 뒤쪽 segment로 튀는 것을 줄인다.
  pnh_.param("control/nearest_search_back", nearest_search_back_, 20);
  pnh_.param("control/nearest_search_ahead", nearest_search_ahead_, 120);

  pnh_.param("control/max_accel", max_accel_, 0.8);
  pnh_.param("control/max_decel", max_decel_, 1.5);
  pnh_.param("control/max_gas", max_gas_, 0.35);
  pnh_.param("control/max_brake", max_brake_, 1.0);
  pnh_.param("control/stop_brake", stop_brake_, 0.4);
  pnh_.param("control/steer_release_after_idle", steer_release_after_idle_, false);
  pnh_.param("control/release_duration", release_duration_, 1.0);
  release_duration_ = std::max(0.0, release_duration_);
  pnh_.param("control/drive_gear", drive_gear_, 1);
  pnh_.param("control/neutral_gear", neutral_gear_, 0);
  pnh_.param("control/reverse_gear", reverse_gear_, -1);

  // Stanley는 타이어 조향각을 계산한다.
  // 기본 조향 한계는 차량 제원의 wheelbase와 최소 회전 반경으로 계산한다.
  //   tire_steer_limit = atan(wheelbase / min_turning_radius)
  // 실제 CarMaker 입력이 steering wheel angle이면 vehicle/steering_ratio 또는
  // vehicle/max_steer_command로 최종 명령 범위를 조정한다.
  double min_turning_radius = 5.5;
  pnh_.param("vehicle/wheelbase", wheelbase_, wheelbase_);
  pnh_.param("vehicle/min_turning_radius", min_turning_radius, min_turning_radius);

  double max_tire_steer_deg = 28.4;
  if (wheelbase_ > 1e-6 && min_turning_radius > 1e-6) {
    max_tire_steer_deg = rad2deg(std::atan(wheelbase_ / min_turning_radius));
  }
  pnh_.param("stanley/max_steer_angle_deg", max_tire_steer_deg, max_tire_steer_deg);
  const double max_tire_steer = deg2rad(max_tire_steer_deg);

  double stanley_k = 2.5;
  double stanley_k_soft = 0.15;
  double stanley_cte_gain = 2.0;
  double stanley_heading_gain = 1.8;
  pnh_.param("stanley/k", stanley_k, stanley_k);
  pnh_.param("stanley/k_soft", stanley_k_soft, stanley_k_soft);
  pnh_.param("stanley/cte_gain", stanley_cte_gain, stanley_cte_gain);
  pnh_.param("stanley/heading_gain", stanley_heading_gain, stanley_heading_gain);
  pnh_.param("stanley/reverse_steering_scale", reverse_steering_scale_, 1.3);
  pnh_.param("stanley/curvature_ff_gain", curvature_ff_gain_, 0.6);
  pnh_.param("stanley/reverse_curvature_ff_sign", reverse_curvature_ff_sign_, -1.0);
  reverse_curvature_ff_sign_ = reverse_curvature_ff_sign_ < 0.0 ? -1.0 : 1.0;
  stanley_.configure(stanley_k, stanley_k_soft, max_tire_steer,
                     stanley_cte_gain, stanley_heading_gain);

  // PID 출력은 gas/brake가 아니라 desired acceleration이다.
  // publishControl에서 이 값을 gas/brake/accel 필드로 다시 변환한다.
  double speed_kp = 1.0;
  double speed_ki = 0.0;
  double speed_kd = 0.05;
  pnh_.param("pid/speed/kp", speed_kp, speed_kp);
  pnh_.param("pid/speed/ki", speed_ki, speed_ki);
  pnh_.param("pid/speed/kd", speed_kd, speed_kd);
  speed_pid_.configure(speed_kp, speed_ki, speed_kd, -max_decel_, max_accel_);

  pnh_.param("vehicle/steering_ratio", steering_ratio_, 9.0);
  max_steer_command_ = max_tire_steer * steering_ratio_;
  pnh_.param("vehicle/max_steer_command", max_steer_command_, max_steer_command_);
  pnh_.param("vehicle/steering_command_sign", steering_command_sign_, 1.0);
  steering_command_sign_ = steering_command_sign_ < 0.0 ? -1.0 : 1.0;

  pnh_.param("debug/publish", publish_debug_, true);
  pnh_.param<std::string>("debug/topic_prefix", debug_topic_prefix_, "/control/debug");
  pnh_.param<std::string>("debug/frame_id", debug_frame_id_, "Fr0");
  pnh_.param("debug/path_publish_period", debug_path_period_, 1.0);
}

void ControlNode::trajectoryCallback(const carmaker_msgs::TrajectoryPathConstPtr& msg)
{
  if (!msg) {
    ROS_WARN_THROTTLE(1.0, "Received null trajectory message.");
    return;
  }

  if (msg->points.size() < 2) {
    ROS_WARN("Received trajectory with fewer than 2 points.");
    return;
  }

  const int new_direction = trajectoryDirection(*msg);
  if (new_direction == 0) {
    ROS_WARN_THROTTLE(1.0, "Active trajectory has no valid direction field; ignoring.");
    return;
  }

  ActiveTrajectory new_trajectory;
  new_trajectory.points.reserve(msg->points.size());
  bool mixed_direction = false;
  for (const auto& point : msg->points) {
    if (point.direction == 0 || (point.direction < 0 ? -1 : 1) != new_direction) {
      mixed_direction = true;
    }
    new_trajectory.points.push_back(makePathPoint(point, new_direction));
  }

  if (mixed_direction) {
    ROS_WARN_THROTTLE(1.0,
                      "Active trajectory violates single-direction contract; ignoring.");
    return;
  }

  const bool new_stop_trajectory =
      std::all_of(new_trajectory.points.begin(), new_trajectory.points.end(),
                  [this](const PathPoint& point) {
                    return point.target_speed <= stop_trajectory_velocity_epsilon_;
                  });

  std::size_t preserved_nearest_index = 0;
  bool preserved_index = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    const bool had_trajectory = trajectory_received_ && !active_trajectory_.empty();
    preserved_index =
        !new_stop_trajectory && had_trajectory &&
        active_trajectory_.direction() == new_direction;
    if (preserved_index) {
      preserved_nearest_index =
          std::min(nearest_index_, new_trajectory.points.size() - 1);
    }
    active_trajectory_ = std::move(new_trajectory);
    nearest_index_ = preserved_nearest_index;
    trajectory_received_ = !active_trajectory_.empty();
    last_trajectory_time_ = ros::Time::now();
  }

  resetIdleRelease();

  ROS_INFO_THROTTLE(1.0,
                    "Active trajectory received: %zu points direction=%s nearest_index=%zu (%s)",
                    msg->points.size(),
                    new_direction > 0 ? "D" : "R",
                    preserved_nearest_index,
                    preserved_index ? "preserved" : "reset");
}

void ControlNode::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  if (!msg) {
    ROS_WARN_THROTTLE(1.0, "Received null Odometry message.");
    return;
  }

  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = *msg;
  last_odom_time_ = ros::Time::now();
  odom_received_ = true;
}

void ControlNode::dynamicsCallback(const carmaker_msgs::DynamicsInfoConstPtr& msg)
{
  if (!msg) {
    ROS_WARN_THROTTLE(1.0, "Received null DynamicsInfo message.");
    return;
  }

  std::lock_guard<std::mutex> lock(dynamics_mutex_);
  latest_dynamics_ = *msg;
  last_dynamics_time_ = ros::Time::now();
  dynamics_received_ = true;
}

ControlNode::PathPoint
ControlNode::makePathPoint(const carmaker_msgs::TrajectoryPoint& msg, int direction) const
{
  // controller에서는 속도 크기와 direction을 분리해서 다룬다.
  // 이렇게 해야 후진에서도 PID는 "얼마나 빠르게 움직일지"만 보고,
  // gear/accel 부호 처리는 publish 단계에서 일관되게 처리할 수 있다.
  PathPoint point;
  point.x = msg.pose.position.x;
  point.y = msg.pose.position.y;
  point.yaw = quaternionToYaw(msg.pose.orientation);
  point.target_speed = std::abs(msg.longitudinal_velocity);
  point.curvature = std::isfinite(msg.curvature) ? msg.curvature : 0.0;
  point.direction = direction;
  return point;
}

int ControlNode::trajectoryDirection(const carmaker_msgs::TrajectoryPath& msg) const
{
  for (const auto& point : msg.points) {
    if (point.direction != 0) {
      return point.direction < 0 ? -1 : 1;
    }
  }
  return 0;
}

bool ControlNode::getCurrentState(Pose2D& pose, double& signed_speed) const
{
  if (state_source_ == "dynamics") {
    carmaker_msgs::DynamicsInfo dynamics;
    ros::Time last_dynamics_time;
    bool has_dynamics = false;
    {
      std::lock_guard<std::mutex> lock(dynamics_mutex_);
      has_dynamics = dynamics_received_;
      dynamics = latest_dynamics_;
      last_dynamics_time = last_dynamics_time_;
    }

    if (!has_dynamics) {
      ROS_WARN_THROTTLE(1.0, "Waiting for DynamicsInfo on %s.", dynamics_topic_.c_str());
      return false;
    }

    if ((ros::Time::now() - last_dynamics_time).toSec() > dynamics_timeout_) {
      ROS_WARN_THROTTLE(1.0, "CarMaker DynamicsInfo timeout.");
      return false;
    }

    // DynamicsInfo는 rear axle 기준 GT를 직접 제공한다.
    // Controller 내부 pose와 /planning/trajectory도 같은 rear axle 기준으로 맞춘다.
    const double yaw = normalizeAngle(dynamics.Car_Yaw);
    pose.x = dynamics.RearAxle_x;
    pose.y = dynamics.RearAxle_y;
    pose.yaw = yaw;
    signed_speed = dynamics.Car_vx;
    return true;
  }

  nav_msgs::Odometry odom;
  ros::Time last_odom_time;
  bool has_odom = false;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    has_odom = odom_received_;
    odom = latest_odom_;
    last_odom_time = last_odom_time_;
  }

  if (!has_odom) {
    ROS_WARN_THROTTLE(1.0, "Waiting for Odometry on %s.", odom_topic_.c_str());
    return false;
  }

  if ((ros::Time::now() - last_odom_time).toSec() > odom_timeout_) {
    ROS_WARN_THROTTLE(1.0, "Localization odometry timeout.");
    return false;
  }

  // localization odom은 pose와 twist를 같은 추정 상태에서 내보낸다.
  // control은 이 값을 현재 상태의 단일 출처로 사용해 planning 좌표계와 맞춘다.
  pose.x = odom.pose.pose.position.x;
  pose.y = odom.pose.pose.position.y;
  pose.yaw = normalizeAngle(quaternionToYaw(odom.pose.pose.orientation));
  signed_speed = odom.twist.twist.linear.x;
  return true;
}

void ControlNode::controlTimerCallback(const ros::TimerEvent& event)
{
  // PID 계산용 dt. ROS time이 멈추거나 처음 호출된 경우 wall time으로 보정한다.
  const ros::Time now = ros::Time::now();
  double dt = (now - last_control_time_).toSec();
  if (last_control_time_.isZero() || dt <= 1e-4 || dt > 1.0) {
    const double wall_dt = event.last_real.isZero()
                               ? 1.0 / std::max(1.0, control_rate_)
                               : (event.current_real - event.last_real).toSec();
    dt = wall_dt > 1e-4 ? wall_dt : 1.0 / std::max(1.0, control_rate_);
  }
  last_control_time_ = now;

  if (idle_control_released_) {
    return;
  }

  Pose2D pose;
  double signed_speed = 0.0;
  if (!getCurrentState(pose, signed_speed)) {
    // pose/speed를 모르면 제어를 계속 내는 것보다 정지 명령을 유지하는 편이 안전하다.
    publishStopReason(1);
    publishStop(0);
    return;
  }

  // callback에서 갱신될 수 있는 trajectory 상태를 짧게 lock해서 복사한다.
  // 실제 제어 계산은 lock 밖에서 수행해 callback 지연을 줄인다.
  ActiveTrajectory active_trajectory;
  std::size_t previous_nearest_index = 0;
  bool should_track = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    const bool trajectory_fresh =
        trajectory_received_ &&
        (now - last_trajectory_time_).toSec() <= active_trajectory_timeout_ &&
        !active_trajectory_.empty();

    if (trajectory_fresh) {
      active_trajectory = active_trajectory_;
      previous_nearest_index = nearest_index_;
      should_track = true;
    }
  }

  if (!should_track) {
    // 경로가 없거나 timeout이면 release_duration 동안만 stop command를 유지한다.
    publishDebugTelemetry(pose, pose, signed_speed, nullptr, 0, 0, 0, 0.0, 0.0, 0, 0.0, 2);
    publishIdleStopUntilRelease(now);
    speed_pid_.reset();
    return;
  }

  resetIdleRelease();

  const double current_speed = std::abs(signed_speed);
  const std::size_t nearest_index_rear =
      findNearestIndex(active_trajectory, pose, previous_nearest_index);
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (active_trajectory.direction() == active_trajectory_.direction()) {
      nearest_index_ = nearest_index_rear;
    }
  }

  Pose2D tracking_pose = pose;
  if (active_trajectory.direction() > 0) {
    tracking_pose.x = pose.x + wheelbase_ * std::cos(pose.yaw);
    tracking_pose.y = pose.y + wheelbase_ * std::sin(pose.yaw);
  }

  const std::size_t nearest_index_tracking =
      findNearestIndex(active_trajectory, tracking_pose, previous_nearest_index);

  // Use arc-length along path from nearest index to the end instead of
  // Euclidean distance. On curved paths, Euclidean distance underestimates the
  // actual remaining travel distance, causing late deceleration and overshoot.
  const double distance_to_end = computeRemainingDistance(active_trajectory.points, nearest_index_rear, pose, active_trajectory.direction());

  // 속도 목표와 curvature feedforward는 lookahead point에서 읽고,
  // Stanley feedback 오차는 nearest point 기준으로 계산한다.
  // 코너에서 너무 앞 점만 보면 lateral error가 늦게 반영될 수 있어서 둘을 분리한다.
  const double lookahead = clamp(lookahead_distance_ + lookahead_time_ * current_speed, min_lookahead_distance_, max_lookahead_distance_);
  const std::size_t target_index = findLookaheadIndex(active_trajectory, nearest_index_rear, lookahead);
  std::size_t preview_index = target_index;
  const double preview_curvature = computePreviewCurvature(active_trajectory, nearest_index_rear, preview_index);

  const TargetSpeedDecision speed_decision = selectTargetSpeed(active_trajectory, nearest_index_rear, target_index, distance_to_end);
  const double target_speed = speed_decision.target_speed;
  const double steer_command = computeSteeringCommand(tracking_pose, active_trajectory.points[nearest_index_tracking],
                                preview_curvature,
                                std::max(current_speed, target_speed),
                                active_trajectory.direction());

  const bool stop_trajectory = std::all_of(active_trajectory.points.begin(), active_trajectory.points.end(),
                                [this](const PathPoint& point) {
                                    return point.target_speed <= stop_trajectory_velocity_epsilon_;
                                });
  if (stop_trajectory) {
    publishDebugTelemetry(pose, tracking_pose, signed_speed, &active_trajectory,
                          nearest_index_rear, nearest_index_tracking, target_index, 0.0, lookahead, 2, steer_command,
                          3, 0.0, kSpeedReasonAllZeroStop,
                          preview_index, curvature_preview_distance_, preview_curvature);
    speed_pid_.reset();
    publishStop(active_trajectory.direction(), steer_command);
    return;
  }

  const double accel_command = speed_pid_.calculate(target_speed, current_speed, dt);

  publishDebugTelemetry(pose, tracking_pose, signed_speed, &active_trajectory,
                        nearest_index_rear, nearest_index_tracking, target_index, target_speed, lookahead, 1, steer_command,
                        0, speed_decision.raw_target_speed, speed_decision.reason,
                        preview_index, curvature_preview_distance_, preview_curvature);
  publishControl(active_trajectory.direction(), steer_command, accel_command);
}

std::size_t ControlNode::findNearestIndex(const ActiveTrajectory& trajectory,
                                          const Pose2D& pose,
                                          std::size_t previous_index) const
{
  const auto& path = trajectory.points;
  if (path.empty()) {
    return 0;
  }

  std::size_t start = 0;
  std::size_t end = path.size() - 1;
  if (previous_index < path.size()) {
    // 이전 nearest 주변만 검색해 경로가 가까이 겹치는 상황에서 index가 뒤로 크게 튀는 것을 줄인다.
    const std::size_t back = static_cast<std::size_t>(std::max(0, nearest_search_back_));
    const std::size_t ahead = static_cast<std::size_t>(std::max(0, nearest_search_ahead_));
    start = previous_index > back ? previous_index - back : 0;
    end = std::min(path.size() - 1, previous_index + ahead);
  }

  std::size_t best_index = start;
  double best_distance = std::numeric_limits<double>::max();
  for (std::size_t i = start; i <= end; ++i) {
    const double distance = distance2D(pose, path[i]);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

std::size_t ControlNode::findLookaheadIndex(const ActiveTrajectory& trajectory,
                                            std::size_t nearest_index,
                                            double lookahead_distance) const
{
  const auto& path = trajectory.points;
  if (path.empty()) {
    return 0;
  }

  std::size_t index = std::min(nearest_index, path.size() - 1);
  double accumulated = 0.0;
  // trajectory point 간 실제 거리 누적으로 lookahead index를 찾는다.
  // planning resampling 간격이 바뀌어도 m 단위 lookahead가 유지된다.
  while (index + 1 < path.size() && accumulated < lookahead_distance) {
    accumulated += distance2D(path[index], path[index + 1]);
    ++index;
  }
  return index;
}

double ControlNode::computePreviewCurvature(const ActiveTrajectory& trajectory,
                                            std::size_t nearest_index,
                                            std::size_t& preview_index) const
{
  const auto& path = trajectory.points;
  if (path.empty()) {
    preview_index = 0;
    return 0.0;
  }

  const double preview_distance = std::max(0.0, curvature_preview_distance_);
  preview_index = findLookaheadIndex(trajectory, nearest_index, preview_distance);

  const double half_window = 0.5 * std::max(0.0, curvature_preview_window_);
  const std::size_t start_index = findLookaheadIndex(trajectory, nearest_index, std::max(0.0, preview_distance - half_window));
  const std::size_t end_index = findLookaheadIndex(trajectory, nearest_index, preview_distance + half_window);

  const std::size_t first = std::min(start_index, end_index);
  const std::size_t last = std::max(start_index, end_index);

  double curvature_sum = 0.0;
  std::size_t curvature_count = 0;
  for (std::size_t i = first; i <= last && i < path.size(); ++i) {
    const double curvature = path[i].curvature;
    if (!std::isfinite(curvature)) {
      continue;
    }
    curvature_sum += curvature;
    ++curvature_count;
  }

  if (curvature_count > 0) {
    return curvature_sum / static_cast<double>(curvature_count);
  }

  const double fallback_curvature =
      path[std::min(preview_index, path.size() - 1)].curvature;
  return std::isfinite(fallback_curvature) ? fallback_curvature : 0.0;
}

double ControlNode::computeCurvatureFeedforward(double preview_curvature, int direction) const
{
  if (!std::isfinite(preview_curvature)) {
    return 0.0;
  }

  const double ff_direction_sign =
      direction < 0 ? reverse_curvature_ff_sign_ : 1.0;
  return curvature_ff_gain_ * ff_direction_sign *
         std::atan(wheelbase_ * preview_curvature);
}

double ControlNode::computeSteeringCommand(const Pose2D& pose,
                                           const PathPoint& feedback_reference,
                                           double preview_curvature,
                                           double speed,
                                           int direction) const
{
  // cte 부호 규약:
  // 경로 yaw 기준 차량이 왼쪽에 있으면 cte가 음수가 된다.
  // CarMaker/teleop 기준 positive steer가 좌회전이므로, 왼쪽으로 벗어난 차량은 음수 조향으로
  // 오른쪽 복귀를 하게 된다.
  const double dx = pose.x - feedback_reference.x;
  const double dy = pose.y - feedback_reference.y;
  const double cte = std::sin(feedback_reference.yaw) * dx -
                     std::cos(feedback_reference.yaw) * dy;
  const double heading_error = normalizeAngle(feedback_reference.yaw - pose.yaw);

  const double tire_steer = stanley_.calculate(cte, heading_error, speed, direction, reverse_steering_scale_);
  const double curvature_ff = computeCurvatureFeedforward(preview_curvature, direction);
  const double steer_command = steering_command_sign_ * (tire_steer + curvature_ff) * steering_ratio_;

  return clamp(steer_command, -max_steer_command_, max_steer_command_);
}

ControlNode::TargetSpeedDecision ControlNode::selectTargetSpeed(
    const ActiveTrajectory& trajectory,
    std::size_t nearest_index,
    std::size_t target_index,
    double distance_to_end) const
{
  return selectTargetSpeedForPath(trajectory.points,
                                  nearest_index,
                                  target_index,
                                  distance_to_end,
                                  min_tracking_speed_,
                                  min_creep_speed_,
                                  max_target_speed_,
                                  stop_trajectory_velocity_epsilon_,
                                  arrival_slow_distance_);
}

double ControlNode::computeRemainingDistance(const std::vector<PathPoint>& path,
                                             std::size_t nearest_index,
                                             const Pose2D& pose,
                                             int direction) const
{
  if (path.empty()) {
    return 0.0;
  }
  const std::size_t idx = std::min(nearest_index, path.size() - 1);
  double dist_to_end = distance2D(pose, path[idx]);
  for (std::size_t i = idx; i + 1 < path.size(); ++i) {
    dist_to_end += distance2D(path[i], path[i + 1]);
  }
  if (direction > 0) {
    dist_to_end = std::max(0.0, dist_to_end - wheelbase_);
  }
  return dist_to_end;
}

ControlNode::TargetSpeedDecision ControlNode::selectTargetSpeedForPath(
    const std::vector<PathPoint>& path,
    std::size_t nearest_index,
    std::size_t target_index,
    double distance_to_end,
    double min_tracking_speed,
    double min_creep_speed,
    double max_target_speed,
    double stop_velocity_epsilon,
    double arrival_slow_distance)
{
  TargetSpeedDecision decision;
  if (path.empty()) {
    return decision;
  }

  const double eps = std::max(0.0, stop_velocity_epsilon);
  nearest_index = std::min(nearest_index, path.size() - 1);
  target_index = std::min(target_index, path.size() - 1);

  const bool stopped_segment = std::all_of(path.begin(), path.end(),
                                [eps](const PathPoint& point) {
                                    return !std::isfinite(point.target_speed) ||
                                        point.target_speed <= eps;
                                });
  if (stopped_segment) {
    decision.reason = kSpeedReasonAllZeroStop;
    return decision;
  }

  decision.raw_target_speed = std::isfinite(path[target_index].target_speed)
                                  ? path[target_index].target_speed
                                  : 0.0;
  double target_speed = decision.raw_target_speed;

  const bool has_future_tracking_speed = std::any_of(path.begin() + nearest_index, path.end(),
                                            [min_tracking_speed](const PathPoint& point) {
                                                return std::isfinite(point.target_speed) &&
                                                    point.target_speed > min_tracking_speed;
                                            });

  if (distance_to_end <= eps) {
    decision.reason = kSpeedReasonEndpointStop;
    return decision;
  }

  if (distance_to_end > arrival_slow_distance && has_future_tracking_speed) {
    decision.reason = kSpeedReasonMinTrackingSpeed;
    target_speed = std::max(target_speed, min_tracking_speed);
  } else if (target_speed <= eps) {
    decision.reason = kSpeedReasonMinCreepSpeed;
    target_speed = min_creep_speed;
  } else {
    decision.reason = kSpeedReasonLookaheadSpeed;
  }

  decision.target_speed = clamp(target_speed, 0.0, std::max(0.0, max_target_speed));
  return decision;
}

void ControlNode::advertiseDebugTopics()
{
  if (!publish_debug_) {
    return;
  }

  if (!debug_topic_prefix_.empty() && debug_topic_prefix_.back() == '/') {
    debug_topic_prefix_.pop_back();
  }

  debug_current_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(debug_topic_prefix_ + "/current_pose", 10);
  debug_current_control_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(debug_topic_prefix_ + "/current_control_pose", 10);
  debug_nearest_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(debug_topic_prefix_ + "/nearest_pose", 10);
  debug_nearest_control_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(debug_topic_prefix_ + "/nearest_control_pose", 10);
  debug_lookahead_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(debug_topic_prefix_ + "/lookahead_pose", 10);
  debug_curvature_preview_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(debug_topic_prefix_ + "/curvature_preview_pose", 10);
  debug_active_path_pub_ =
      nh_.advertise<nav_msgs::Path>(debug_topic_prefix_ + "/active_path", 1, true);

  debug_current_speed_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/current_speed", 10);
  debug_target_speed_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/target_speed", 10);
  debug_raw_target_speed_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/raw_lookahead_speed", 10);
  debug_speed_error_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/speed_error", 10);
  debug_steer_command_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/steer_command", 10);
  debug_curvature_ff_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/curvature_feedforward", 10);
  debug_curvature_preview_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/curvature_preview", 10);
  debug_curvature_preview_distance_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/curvature_preview_distance", 10);
  debug_steer_saturated_pub_ =
      nh_.advertise<std_msgs::Int32>(debug_topic_prefix_ + "/steer_saturated", 10);
  debug_cte_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/cross_track_error", 10);
  debug_heading_error_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/heading_error", 10);
  debug_lookahead_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/lookahead_distance", 10);
  debug_segment_index_pub_ =
      nh_.advertise<std_msgs::Int32>(debug_topic_prefix_ + "/segment_index", 10);
  debug_segment_count_pub_ =
      nh_.advertise<std_msgs::Int32>(debug_topic_prefix_ + "/segment_count", 10);
  debug_nearest_index_pub_ =
      nh_.advertise<std_msgs::Int32>(debug_topic_prefix_ + "/nearest_index", 10);
  debug_target_index_pub_ =
      nh_.advertise<std_msgs::Int32>(debug_topic_prefix_ + "/target_index", 10);
  debug_curvature_preview_index_pub_ =
      nh_.advertise<std_msgs::Int32>(debug_topic_prefix_ + "/curvature_preview_index", 10);
  debug_distance_to_end_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/distance_to_segment_end", 10);
  debug_trajectory_age_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/trajectory_age", 10);
  debug_direction_pub_ =
      nh_.advertise<std_msgs::Int32>(debug_topic_prefix_ + "/direction", 10);
  debug_tracking_state_pub_ =
      nh_.advertise<std_msgs::Int32>(debug_topic_prefix_ + "/tracking_state", 10);
  debug_stop_reason_pub_ =
      nh_.advertise<std_msgs::Int32>(debug_topic_prefix_ + "/stop_reason", 10);
  debug_speed_selection_reason_pub_ =
      nh_.advertise<std_msgs::Int32>(debug_topic_prefix_ + "/speed_selection_reason", 10);
  debug_speed_selection_reason_text_pub_ =
      nh_.advertise<std_msgs::String>(debug_topic_prefix_ + "/speed_selection_reason_text", 10);

  ROS_INFO("Control debug topics enabled at %s (frame=%s)",
           debug_topic_prefix_.c_str(), debug_frame_id_.c_str());
}

void ControlNode::publishDebugTelemetry(const Pose2D& pose,
                                        const Pose2D& tracking_pose,
                                        double signed_speed,
                                        const ActiveTrajectory* trajectory,
                                        std::size_t nearest_index_rear,
                                        std::size_t nearest_index_tracking,
                                        std::size_t target_index,
                                        double target_speed,
                                        double lookahead,
                                        int tracking_state,
                                        double steer_command,
                                        int stop_reason,
                                        double raw_target_speed,
                                        int speed_selection_reason,
                                        std::size_t preview_index,
                                        double preview_distance,
                                        double preview_curvature)
{
  if (!publish_debug_) {
    return;
  }

  const ros::Time stamp = ros::Time::now();
  const double current_speed = std::abs(signed_speed);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  double cte = nan;
  double heading_error = nan;
  double curvature_ff = nan;
  double preview_curvature_value = nan;
  double preview_distance_value = nan;
  double distance_to_end = -1.0;
  double trajectory_age = -1.0;
  int direction = 0;
  int segment_index_value = trajectory ? 0 : -1;
  int nearest_index_value = -1;
  int target_index_value = -1;
  int preview_index_value = -1;
  int segment_count = 0;
  int steer_saturated = 0;

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    segment_count = trajectory_received_ && !active_trajectory_.empty() ? 1 : 0;
    if (trajectory_received_) {
      trajectory_age = (stamp - last_trajectory_time_).toSec();
    }
  }

  publishPoseDebug(debug_current_pose_pub_, pose.x, pose.y, pose.yaw, stamp);
  publishPoseDebug(debug_current_control_pose_pub_, tracking_pose.x, tracking_pose.y, tracking_pose.yaw, stamp);

  if (trajectory && !trajectory->empty()) {
    const auto& path = trajectory->points;
    direction = trajectory->direction();
    const std::size_t nearest_rear = std::min(nearest_index_rear, path.size() - 1);
    const std::size_t nearest_tracking = std::min(nearest_index_tracking, path.size() - 1);
    const std::size_t target = std::min(target_index, path.size() - 1);
    const bool has_preview =
        preview_index != std::numeric_limits<std::size_t>::max();
    const std::size_t preview =
        has_preview ? std::min(preview_index, path.size() - 1) : target;
    nearest_index_value = static_cast<int>(nearest_rear);
    target_index_value = static_cast<int>(target);
    preview_index_value = has_preview ? static_cast<int>(preview) : -1;
    const PathPoint& nearest_rear_point = path[nearest_rear];
    const PathPoint& nearest_tracking_point = path[nearest_tracking];
    const PathPoint& target_point = path[target];
    const PathPoint& preview_point = path[preview];
    
    distance_to_end = computeRemainingDistance(path, nearest_rear, pose, direction);

    if (has_preview) {
      preview_curvature_value = preview_curvature;
      preview_distance_value = preview_distance;
    }

    publishPoseDebug(debug_nearest_pose_pub_, nearest_rear_point.x, nearest_rear_point.y, nearest_rear_point.yaw, stamp);
    publishPoseDebug(debug_nearest_control_pose_pub_, nearest_tracking_point.x, nearest_tracking_point.y, nearest_tracking_point.yaw, stamp);
    publishPoseDebug(debug_lookahead_pose_pub_, target_point.x, target_point.y, target_point.yaw, stamp);
    if (has_preview) {
      publishPoseDebug(debug_curvature_preview_pose_pub_, preview_point.x, preview_point.y, preview_point.yaw, stamp);
    }

    const double dx = tracking_pose.x - nearest_tracking_point.x;
    const double dy = tracking_pose.y - nearest_tracking_point.y;
    cte = std::sin(nearest_tracking_point.yaw) * dx -
          std::cos(nearest_tracking_point.yaw) * dy;
    heading_error = normalizeAngle(nearest_tracking_point.yaw - tracking_pose.yaw);
    curvature_ff = steering_command_sign_ * steering_ratio_ * computeCurvatureFeedforward(preview_curvature, direction);
    steer_saturated = std::abs(steer_command) >= 0.98 * std::max(1e-6, max_steer_command_) ? 1 : 0;

    const bool should_publish_path = last_debug_path_time_.isZero() ||
                                        debug_path_period_ <= 0.0 ||
                                        (stamp - last_debug_path_time_).toSec() >= debug_path_period_;
    if (should_publish_path) {
      publishActiveSegmentPath(*trajectory, stamp);
    }
  } else {
    target_speed = 0.0;
    raw_target_speed = 0.0;
    lookahead = nan;
    speed_selection_reason = kSpeedReasonEndpointStop;
  }

  std_msgs::Float64 float_msg;
  float_msg.data = current_speed;
  debug_current_speed_pub_.publish(float_msg);
  float_msg.data = target_speed;
  debug_target_speed_pub_.publish(float_msg);
  float_msg.data = raw_target_speed;
  debug_raw_target_speed_pub_.publish(float_msg);
  float_msg.data = target_speed - current_speed;
  debug_speed_error_pub_.publish(float_msg);
  float_msg.data = steer_command;
  debug_steer_command_pub_.publish(float_msg);
  float_msg.data = curvature_ff;
  debug_curvature_ff_pub_.publish(float_msg);
  float_msg.data = preview_curvature_value;
  debug_curvature_preview_pub_.publish(float_msg);
  float_msg.data = preview_distance_value;
  debug_curvature_preview_distance_pub_.publish(float_msg);
  float_msg.data = cte;
  debug_cte_pub_.publish(float_msg);
  float_msg.data = heading_error;
  debug_heading_error_pub_.publish(float_msg);
  float_msg.data = lookahead;
  debug_lookahead_pub_.publish(float_msg);
  float_msg.data = distance_to_end;
  debug_distance_to_end_pub_.publish(float_msg);
  float_msg.data = trajectory_age;
  debug_trajectory_age_pub_.publish(float_msg);

  std_msgs::Int32 int_msg;
  int_msg.data = segment_index_value;
  debug_segment_index_pub_.publish(int_msg);
  int_msg.data = segment_count;
  debug_segment_count_pub_.publish(int_msg);
  int_msg.data = nearest_index_value;
  debug_nearest_index_pub_.publish(int_msg);
  int_msg.data = target_index_value;
  debug_target_index_pub_.publish(int_msg);
  int_msg.data = preview_index_value;
  debug_curvature_preview_index_pub_.publish(int_msg);
  int_msg.data = steer_saturated;
  debug_steer_saturated_pub_.publish(int_msg);
  int_msg.data = direction;
  debug_direction_pub_.publish(int_msg);
  int_msg.data = tracking_state;
  debug_tracking_state_pub_.publish(int_msg);
  int_msg.data = stop_reason;
  debug_stop_reason_pub_.publish(int_msg);
  int_msg.data = speed_selection_reason;
  debug_speed_selection_reason_pub_.publish(int_msg);

  std_msgs::String string_msg;
  string_msg.data = speedSelectionReasonString(speed_selection_reason);
  debug_speed_selection_reason_text_pub_.publish(string_msg);
}

void ControlNode::publishStopReason(int stop_reason)
{
  if (!publish_debug_) {
    return;
  }

  std_msgs::Int32 msg;
  msg.data = stop_reason;
  debug_stop_reason_pub_.publish(msg);
}

void ControlNode::publishPoseDebug(const ros::Publisher& publisher,
                                   double x,
                                   double y,
                                   double yaw,
                                   const ros::Time& stamp) const
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = debug_frame_id_;
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = 0.0;
  msg.pose.orientation = yawToQuaternion(yaw);
  publisher.publish(msg);
}

void ControlNode::publishActiveSegmentPath(const ActiveTrajectory& trajectory,
                                           const ros::Time& stamp)
{
  nav_msgs::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = debug_frame_id_;
  path.poses.reserve(trajectory.points.size());

  for (const PathPoint& point : trajectory.points) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = yawToQuaternion(point.yaw);
    path.poses.push_back(pose);
  }

  debug_active_path_pub_.publish(path);
  last_debug_path_time_ = stamp;
}

void ControlNode::publishControl(int direction, double steer_command, double accel_command)
{
  // PID 출력 accel_command를 gas/brake로 변환한다.
  // accel_command > 0: 가속, accel_command < 0: 감속.
  // 후진은 gear가 reverse이므로 msg.accel 부호만 direction에 맞춰 뒤집어 기록한다.
  const double gas_norm = clamp(accel_command / std::max(1e-6, max_accel_), 0.0, 1.0);
  const double brake_norm = clamp(-accel_command / std::max(1e-6, max_decel_), 0.0, 1.0);

  last_steer_command_ = clamp(steer_command, -max_steer_command_, max_steer_command_);
  resetIdleRelease();

  carmaker_msgs::Control_Signal msg;
  msg.header.stamp = ros::Time::now();
  msg.steerangle = static_cast<float>(last_steer_command_);
  msg.gas = static_cast<float>(gas_norm * max_gas_);
  msg.brake = static_cast<float>(brake_norm * max_brake_);
  msg.accel = static_cast<float>(direction * (gas_norm * max_accel_ - brake_norm * max_decel_));
  msg.gear = directionToGear(direction, drive_gear_, reverse_gear_);
  control_pub_.publish(msg);
}

void ControlNode::publishStop(int direction, double steer_command)
{
  // direction=0이면 active trajectory가 없으므로 neutral로 대기하되 마지막 조향각은 유지한다.
  // direction이 있으면 현재/다음 gear를 유지한 채 멈춤 명령을 내 기어 전환 타이밍을 안정화한다.
  if (direction != 0) {
    last_steer_command_ = clamp(steer_command, -max_steer_command_, max_steer_command_);
  }

  carmaker_msgs::Control_Signal msg;
  msg.header.stamp = ros::Time::now();
  msg.gas = 0.0F;
  msg.steerangle = static_cast<float>(last_steer_command_);
  if (direction == 0) {
    msg.brake = 0.0F;
    msg.gear = neutral_gear_;
    msg.accel = 0.0F;
  } else {
    msg.brake = static_cast<float>(clamp(stop_brake_, 0.0, max_brake_));
    msg.gear = directionToGear(direction, drive_gear_, reverse_gear_);
    msg.accel = static_cast<float>(direction * -msg.brake * max_decel_);
  }
  control_pub_.publish(msg);
}

void ControlNode::resetIdleRelease()
{
  idle_release_active_ = false;
  idle_control_released_ = false;
  idle_release_start_time_ = ros::Time();
}

bool ControlNode::publishIdleStopUntilRelease(const ros::Time& now)
{
  if (idle_control_released_) {
    return false;
  }

  if (!idle_release_active_) {
    idle_release_active_ = true;
    idle_release_start_time_ = now;
    idle_release_initial_steer_ = last_steer_command_;
  }

  const double elapsed = (now - idle_release_start_time_).toSec();
  if (elapsed >= release_duration_) {
    if (steer_release_after_idle_) {
      last_steer_command_ = 0.0;
    }
    idle_control_released_ = true;
    ROS_INFO_THROTTLE(1.0, "No active trajectory: released control_signal publisher.");
    return false;
  }

  if (steer_release_after_idle_) {
    const double ratio = release_duration_ <= 1e-6
                             ? 1.0
                             : clamp(elapsed / release_duration_, 0.0, 1.0);
    last_steer_command_ = idle_release_initial_steer_ * (1.0 - ratio);
  }

  if (publish_stop_without_path_) {
    publishStop(0);
    return true;
  }
  return false;
}
double ControlNode::distance2D(const Pose2D& pose, const PathPoint& point)
{
  return std::hypot(pose.x - point.x, pose.y - point.y);
}

double ControlNode::distance2D(const PathPoint& a, const PathPoint& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace carmaker_control

int main(int argc, char** argv)
{
  ros::init(argc, argv, "carmaker_control_node");
  carmaker_control::ControlNode node;
  ros::spin();
  return 0;
}
