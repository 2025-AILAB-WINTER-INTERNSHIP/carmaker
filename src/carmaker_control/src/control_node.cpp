#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

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

  // 같은 direction을 갖는 연속 trajectory 구간.
  // 후진이 섞인 전체 경로도 이 단위로 나누어 하나씩 제어한다.
  struct PathSegment {
    std::vector<PathPoint> points;
    int direction{1};
  };

  void loadParameters();
  void trajectoryCallback(const carmaker_msgs::TrajectoryPathConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void dynamicsCallback(const carmaker_msgs::DynamicsInfoConstPtr& msg);
  void controlTimerCallback(const ros::TimerEvent& event);

  std::vector<PathSegment> splitTrajectory(const carmaker_msgs::TrajectoryPath& msg) const;
  PathPoint makePathPoint(const carmaker_msgs::TrajectoryPoint& msg, int direction) const;

  bool getCurrentState(Pose2D& pose, double& signed_speed) const;
  std::size_t findNearestIndex(const PathSegment& segment,
                               const Pose2D& pose,
                               std::size_t previous_index) const;
  std::size_t findLookaheadIndex(const PathSegment& segment,
                                 std::size_t nearest_index,
                                 double lookahead_distance) const;
  bool isSegmentComplete(const PathSegment& segment,
                         const Pose2D& pose,
                         std::size_t nearest_index) const;
  Pose2D toSteeringControlPose(const Pose2D& pose) const;
  PathPoint toSteeringControlPoint(const PathPoint& point) const;
  double computeSteeringCommand(const Pose2D& pose,
                                const PathPoint& feedback_reference,
                                const PathPoint& feedforward_reference,
                                double speed,
                                int direction) const;
  double selectTargetSpeed(const PathSegment& segment, std::size_t target_index) const;

  void advanceOrStop(const PathSegment& active_segment,
                     std::size_t active_segment_index,
                     const Pose2D& pose,
                     double current_speed,
                     const ros::Time& now);
  void advertiseDebugTopics();
  void publishDebugTelemetry(const Pose2D& pose,
                             double signed_speed,
                             const PathSegment* segment,
                             std::size_t segment_index,
                             std::size_t nearest_index,
                             std::size_t target_index,
                             double target_speed,
                             double lookahead,
                             int tracking_state,
                             double steer_command);
  void publishPoseDebug(const ros::Publisher& publisher,
                        double x,
                        double y,
                        double yaw,
                        const ros::Time& stamp) const;
  void publishActiveSegmentPath(const PathSegment& segment, const ros::Time& stamp);
  void publishControl(int direction, double steer_command, double accel_command);
  void publishStop(int direction, double steer_command = 0.0);

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
  ros::Publisher debug_active_path_pub_;
  ros::Publisher debug_current_speed_pub_;
  ros::Publisher debug_target_speed_pub_;
  ros::Publisher debug_speed_error_pub_;
  ros::Publisher debug_steer_command_pub_;
  ros::Publisher debug_curvature_ff_pub_;
  ros::Publisher debug_steer_saturated_pub_;
  ros::Publisher debug_cte_pub_;
  ros::Publisher debug_heading_error_pub_;
  ros::Publisher debug_lookahead_pub_;
  ros::Publisher debug_segment_index_pub_;
  ros::Publisher debug_segment_count_pub_;
  ros::Publisher debug_nearest_index_pub_;
  ros::Publisher debug_target_index_pub_;
  ros::Publisher debug_distance_to_end_pub_;
  ros::Publisher debug_trajectory_age_pub_;
  ros::Publisher debug_trajectory_completed_pub_;
  ros::Publisher debug_direction_pub_;
  ros::Publisher debug_tracking_state_pub_;

  mutable std::mutex odom_mutex_;
  nav_msgs::Odometry latest_odom_;
  ros::Time last_odom_time_;
  bool odom_received_{false};

  mutable std::mutex dynamics_mutex_;
  carmaker_msgs::DynamicsInfo latest_dynamics_;
  ros::Time last_dynamics_time_;
  bool dynamics_received_{false};

  mutable std::mutex trajectory_mutex_;
  std::vector<PathSegment> segments_;
  ros::Time last_trajectory_time_;
  std::size_t active_segment_index_{0};
  std::size_t nearest_index_{0};
  bool presteer_active_{false};
  std::size_t presteer_segment_index_{0};
  ros::Time presteer_until_;
  double presteer_steer_command_{0.0};
  bool trajectory_received_{false};
  bool trajectory_completed_{false};

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
  double trajectory_timeout_{5.0};

  double direction_velocity_epsilon_{0.02};
  double min_tracking_speed_{0.2};
  double max_target_speed_{0.7};
  double lookahead_distance_{0.35};
  double lookahead_time_{0.2};
  double min_lookahead_distance_{0.2};
  double max_lookahead_distance_{0.8};
  int nearest_search_back_{20};
  int nearest_search_ahead_{120};

  double segment_finish_distance_{0.5};
  int segment_finish_index_margin_{3};
  double gear_switch_speed_{0.08};
  bool presteer_enabled_{true};
  double presteer_duration_{0.6};

  double max_accel_{0.8};
  double max_decel_{1.5};
  double max_gas_{0.35};
  double max_brake_{1.0};
  double stop_brake_{0.4};

  int drive_gear_{1};
  int neutral_gear_{0};
  int reverse_gear_{-1};

  double wheelbase_{2.97};
  double rear_axle_offset_{0.82};
  std::string dynamics_pose_reference_{"rear_bumper"};
  double steering_ratio_{9.0};
  double max_steer_command_{4.5};
  double steering_command_sign_{1.0};
  double reverse_steering_scale_{1.0};
  double curvature_ff_gain_{0.6};
  double reverse_curvature_ff_sign_{-1.0};
  std::string steering_control_point_{"rear_axle"};

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

  // trajectory_timeout은 새 경로가 너무 오래된 경우 차량을 정지시키기 위한 안전장치다.
  // Ubuntu ROS PC에서 시뮬레이션 clock을 쓸 경우 ROS time 기준으로 동작한다.
  pnh_.param("control/rate", control_rate_, 30.0);
  pnh_.param("control/odom_timeout", odom_timeout_, 0.5);
  pnh_.param("control/dynamics_timeout", dynamics_timeout_, 0.5);
  pnh_.param("control/trajectory_timeout", trajectory_timeout_, 5.0);

  // direction_velocity_epsilon보다 작은 속도는 정지점으로 보고 이전 기어 구간에 붙인다.
  // Reeds-Shepp cusp 근처에서 velocity가 0으로 떨어져도 불필요하게 segment가 쪼개지지 않는다.
  pnh_.param("control/direction_velocity_epsilon", direction_velocity_epsilon_, 0.02);
  pnh_.param("control/min_tracking_speed", min_tracking_speed_, 0.2);
  pnh_.param("control/max_target_speed", max_target_speed_, 0.7);

  // lookahead는 속도가 높을수록 조금 멀리 보도록 base + time * speed 형태로 계산한다.
  // 단, 너무 가까워 떨리거나 너무 멀어 코너를 뭉개지 않도록 min/max로 제한한다.
  pnh_.param("control/lookahead_distance", lookahead_distance_, 0.35);
  pnh_.param("control/lookahead_time", lookahead_time_, 0.2);
  pnh_.param("control/min_lookahead_distance", min_lookahead_distance_, 0.2);
  pnh_.param("control/max_lookahead_distance", max_lookahead_distance_, 0.8);

  // nearest search는 이전 index 주변만 훑어 경로가 겹칠 때 뒤쪽 segment로 튀는 것을 줄인다.
  pnh_.param("control/nearest_search_back", nearest_search_back_, 20);
  pnh_.param("control/nearest_search_ahead", nearest_search_ahead_, 120);

  // segment 종료는 index가 끝에 가깝고 실제 위치도 끝점 근처일 때만 인정한다.
  // 후진 전환 지점에서 가까운 다른 경로점으로 잘못 넘어가는 상황을 피하기 위함이다.
  pnh_.param("control/segment_finish_distance", segment_finish_distance_, 0.5);
  pnh_.param("control/segment_finish_index_margin", segment_finish_index_margin_, 3);
  pnh_.param("control/gear_switch_speed", gear_switch_speed_, 0.08);
  pnh_.param("control/presteer_enabled", presteer_enabled_, true);
  pnh_.param("control/presteer_duration", presteer_duration_, 0.6);
  presteer_duration_ = std::max(0.0, presteer_duration_);

  pnh_.param("control/max_accel", max_accel_, 0.8);
  pnh_.param("control/max_decel", max_decel_, 1.5);
  pnh_.param("control/max_gas", max_gas_, 0.35);
  pnh_.param("control/max_brake", max_brake_, 1.0);
  pnh_.param("control/stop_brake", stop_brake_, 0.4);
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
  pnh_.param("vehicle/rear_axle_offset", rear_axle_offset_, 0.82);
  pnh_.param<std::string>("vehicle/dynamics_pose_reference",
                          dynamics_pose_reference_, "rear_bumper");
  if (dynamics_pose_reference_ != "rear_bumper" &&
      dynamics_pose_reference_ != "rear_axle") {
    ROS_WARN("Unknown vehicle/dynamics_pose_reference='%s'. Falling back to 'rear_bumper'.",
             dynamics_pose_reference_.c_str());
    dynamics_pose_reference_ = "rear_bumper";
  }

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
  pnh_.param<std::string>("vehicle/steering_control_point",
                          steering_control_point_, "rear_axle");
  if (steering_control_point_ != "rear_bumper" &&
      steering_control_point_ != "rear_axle") {
    ROS_WARN("Unknown vehicle/steering_control_point='%s'. Falling back to 'rear_axle'.",
             steering_control_point_.c_str());
    steering_control_point_ = "rear_axle";
  }

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

  // planning은 전체 경로를 하나의 TrajectoryPath로 준다.
  // 이 노드는 signed longitudinal_velocity를 보고 같은 기어 구간끼리 분리한다.
  std::vector<PathSegment> new_segments = splitTrajectory(*msg);
  if (new_segments.empty()) {
    ROS_WARN("Received trajectory with no usable segments.");
  }

  // 로그에 D:points, R:points 형태로 남겨 전진/후진 segment 분할 결과를 바로 확인한다.
  std::ostringstream summary;
  for (std::size_t i = 0; i < new_segments.size(); ++i) {
    if (i > 0) {
      summary << ", ";
    }
    summary << (new_segments[i].direction > 0 ? "D" : "R")
            << ":" << new_segments[i].points.size();
  }

  const std::size_t segment_count = new_segments.size();
  {
    // 새 trajectory를 받으면 진행 index와 PID 기억을 모두 초기화한다.
    // 이전 경로의 index가 새 경로에 남아 있으면 엉뚱한 점을 추종할 수 있다.
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    segments_ = std::move(new_segments);
    active_segment_index_ = 0;
    nearest_index_ = 0;
    presteer_active_ = false;
    presteer_segment_index_ = 0;
    presteer_until_ = ros::Time();
    presteer_steer_command_ = 0.0;
    trajectory_received_ = !segments_.empty();
    trajectory_completed_ = false;
    last_trajectory_time_ = ros::Time::now();
  }

  speed_pid_.reset();
  ROS_INFO("Trajectory received: %zu points -> %zu segment(s) [%s]",
           msg->points.size(), segment_count, summary.str().c_str());
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

std::vector<ControlNode::PathSegment>
ControlNode::splitTrajectory(const carmaker_msgs::TrajectoryPath& msg) const
{
  std::vector<PathSegment> result;
  const std::size_t count = msg.points.size();
  if (count < 2) {
    return result;
  }

  // 1단계: velocity 부호만 먼저 읽는다.
  // +: 전진, -: 후진, 0: 정지/전환점으로 둔다.
  std::vector<int> raw_direction(count, 0);
  for (std::size_t i = 0; i < count; ++i) {
    const double velocity = msg.points[i].longitudinal_velocity;
    if (velocity > direction_velocity_epsilon_) {
      raw_direction[i] = 1;
    } else if (velocity < -direction_velocity_epsilon_) {
      raw_direction[i] = -1;
    }
  }

  // 2단계: 경로가 정지 속도만 갖는 앞부분이라도 첫 non-zero direction을 알 수 있게
  // 뒤에서부터 다음 유효 direction을 미리 계산한다.
  std::vector<int> next_non_zero(count, 0);
  int upcoming = 0;
  for (std::size_t reverse_i = count; reverse_i-- > 0;) {
    if (raw_direction[reverse_i] != 0) {
      upcoming = raw_direction[reverse_i];
    }
    next_non_zero[reverse_i] = upcoming;
  }

  // 3단계: 0 속도 점은 현재 진행 중인 direction에 붙인다.
  // cusp에서 멈추는 점들은 이전 segment의 끝점이 되고, 새 segment는 그 점을 복사해 시작한다.
  std::vector<int> direction(count, 1);
  int active_direction = next_non_zero.front() == 0 ? 1 : next_non_zero.front();
  for (std::size_t i = 0; i < count; ++i) {
    if (raw_direction[i] != 0) {
      active_direction = raw_direction[i];
    }
    direction[i] = active_direction;
  }

  PathSegment current;
  current.direction = direction.front();

  // 4단계: direction이 바뀌는 순간 segment를 닫는다.
  // 새 segment에는 이전 끝점을 속도 0인 boundary로 복사해 기어 전환 기준점을 공유한다.
  for (std::size_t i = 0; i < count; ++i) {
    if (i > 0 && direction[i] != current.direction) {
      if (current.points.size() >= 2) {
        result.push_back(current);
      }

      PathSegment next_segment;
      next_segment.direction = direction[i];
      if (!current.points.empty()) {
        PathPoint boundary = current.points.back();
        boundary.direction = next_segment.direction;
        boundary.target_speed = 0.0;
        next_segment.points.push_back(boundary);
      }
      current = next_segment;
    }

    current.points.push_back(makePathPoint(msg.points[i], current.direction));
  }

  if (current.points.size() >= 2) {
    result.push_back(current);
  }

  return result;
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

    // DynamicsInfo의 Car_* 값은 CarMaker GT 상태다.
    // Controller 내부 pose와 /planning/trajectory는 rear axle 기준으로 맞춘다.
    // CarMaker GT가 Fr1A/rear bumper 기준이면 후륜축 중심으로 변환한다.
    const double yaw = normalizeAngle(dynamics.Car_Yaw);
    pose.x = dynamics.Car_x;
    pose.y = dynamics.Car_y;
    if (dynamics_pose_reference_ == "rear_bumper") {
      pose.x += rear_axle_offset_ * std::cos(yaw);
      pose.y += rear_axle_offset_ * std::sin(yaw);
    }
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

  Pose2D pose;
  double signed_speed = 0.0;
  if (!getCurrentState(pose, signed_speed)) {
    // pose/speed를 모르면 제어를 계속 내는 것보다 정지 명령을 유지하는 편이 안전하다.
    publishStop(0);
    return;
  }

  // callback에서 갱신될 수 있는 trajectory 상태를 짧게 lock해서 복사한다.
  // 실제 제어 계산은 lock 밖에서 수행해 callback 지연을 줄인다.
  PathSegment active_segment;
  std::size_t active_segment_index = 0;
  std::size_t previous_nearest_index = 0;
  bool presteer_active = false;
  ros::Time presteer_until;
  double presteer_steer_command = 0.0;
  bool should_track = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    const bool trajectory_fresh =
        trajectory_received_ &&
        !trajectory_completed_ &&
        (now - last_trajectory_time_).toSec() <= trajectory_timeout_ &&
        active_segment_index_ < segments_.size();

    if (trajectory_fresh) {
      active_segment = segments_[active_segment_index_];
      active_segment_index = active_segment_index_;
      previous_nearest_index = nearest_index_;
      presteer_active =
          presteer_active_ && presteer_segment_index_ == active_segment_index_;
      presteer_until = presteer_until_;
      presteer_steer_command = presteer_steer_command_;
      should_track = true;
    } else {
      presteer_active_ = false;
    }
  }

  if (!should_track) {
    // 경로가 없거나 timeout이면 neutral + brake로 대기한다.
    publishDebugTelemetry(pose, signed_speed, nullptr, 0, 0, 0, 0.0, 0.0, 0, 0.0);
    if (publish_stop_without_path_) {
      publishStop(0);
    }
    speed_pid_.reset();
    return;
  }

  const double current_speed = std::abs(signed_speed);
  // 현재 segment 안에서만 nearest point를 찾는다.
  // 이것이 "전체 경로를 한 번에 추종하지 않고 같은 기어 단위로 제어"하는 핵심이다.
  const std::size_t nearest_index = findNearestIndex(active_segment, pose, previous_nearest_index);
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (active_segment_index == active_segment_index_) {
      nearest_index_ = nearest_index;
    }
  }

  if (presteer_active) {
    const double presteer_lookahead =
        clamp(lookahead_distance_, min_lookahead_distance_, max_lookahead_distance_);
    const std::size_t presteer_target_index =
        findLookaheadIndex(active_segment, nearest_index, presteer_lookahead);
    if (now < presteer_until || current_speed > gear_switch_speed_) {
      publishDebugTelemetry(pose, signed_speed, &active_segment, active_segment_index,
                            nearest_index, presteer_target_index, 0.0,
                            presteer_lookahead, 3, presteer_steer_command);
      publishStop(active_segment.direction, presteer_steer_command);
      speed_pid_.reset();
      return;
    }

    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (presteer_segment_index_ == active_segment_index) {
      presteer_active_ = false;
    }
  }

  if (isSegmentComplete(active_segment, pose, nearest_index)) {
    // segment 끝에 도착하면 바로 다음 segment로 넘어가지 않고 먼저 충분히 감속한다.
    // 전진/후진 기어 전환을 움직이는 중에 하지 않기 위한 처리다.
    publishDebugTelemetry(pose, signed_speed, &active_segment, active_segment_index,
                          nearest_index, nearest_index, 0.0, 0.0, 2, 0.0);
    advanceOrStop(active_segment, active_segment_index, pose, current_speed, now);
    return;
  }

  // 속도 목표와 curvature feedforward는 lookahead point에서 읽고,
  // Stanley feedback 오차는 nearest point 기준으로 계산한다.
  // 코너에서 너무 앞 점만 보면 lateral error가 늦게 반영될 수 있어서 둘을 분리한다.
  const double lookahead =
      clamp(lookahead_distance_ + lookahead_time_ * current_speed,
            min_lookahead_distance_, max_lookahead_distance_);
  const std::size_t target_index = findLookaheadIndex(active_segment, nearest_index, lookahead);

  const double target_speed = selectTargetSpeed(active_segment, target_index);
  const double accel_command = speed_pid_.calculate(target_speed, current_speed, dt);
  const double steer_command =
      computeSteeringCommand(pose, active_segment.points[nearest_index],
                             active_segment.points[target_index],
                             std::max(current_speed, target_speed), active_segment.direction);

  publishDebugTelemetry(pose, signed_speed, &active_segment, active_segment_index,
                        nearest_index, target_index, target_speed, lookahead, 1, steer_command);
  publishControl(active_segment.direction, steer_command, accel_command);
}

std::size_t ControlNode::findNearestIndex(const PathSegment& segment,
                                          const Pose2D& pose,
                                          std::size_t previous_index) const
{
  if (segment.points.empty()) {
    return 0;
  }

  std::size_t start = 0;
  std::size_t end = segment.points.size() - 1;
  if (previous_index < segment.points.size()) {
    // 이전 nearest 주변만 검색해 경로가 가까이 겹치는 상황에서 index가 뒤로 크게 튀는 것을 줄인다.
    const std::size_t back = static_cast<std::size_t>(std::max(0, nearest_search_back_));
    const std::size_t ahead = static_cast<std::size_t>(std::max(0, nearest_search_ahead_));
    start = previous_index > back ? previous_index - back : 0;
    end = std::min(segment.points.size() - 1, previous_index + ahead);
  }

  std::size_t best_index = start;
  double best_distance = std::numeric_limits<double>::max();
  for (std::size_t i = start; i <= end; ++i) {
    const double distance = distance2D(pose, segment.points[i]);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

std::size_t ControlNode::findLookaheadIndex(const PathSegment& segment,
                                            std::size_t nearest_index,
                                            double lookahead_distance) const
{
  if (segment.points.empty()) {
    return 0;
  }

  std::size_t index = std::min(nearest_index, segment.points.size() - 1);
  double accumulated = 0.0;
  // trajectory point 간 실제 거리 누적으로 lookahead index를 찾는다.
  // planning resampling 간격이 바뀌어도 m 단위 lookahead가 유지된다.
  while (index + 1 < segment.points.size() && accumulated < lookahead_distance) {
    accumulated += distance2D(segment.points[index], segment.points[index + 1]);
    ++index;
  }
  return index;
}

bool ControlNode::isSegmentComplete(const PathSegment& segment,
                                    const Pose2D& pose,
                                    std::size_t nearest_index) const
{
  if (segment.points.empty()) {
    return true;
  }

  const std::size_t last_index = segment.points.size() - 1;
  const bool near_end_index =
      nearest_index + static_cast<std::size_t>(std::max(0, segment_finish_index_margin_)) >= last_index;
  const bool near_end_position = distance2D(pose, segment.points.back()) <= segment_finish_distance_;
  // index 조건과 위치 조건을 모두 만족해야 segment 완료로 본다.
  // 단순 거리만 쓰면 U-turn/후진 경로에서 끝점 근처를 지나갈 때 조기 전환될 수 있다.
  return near_end_index && near_end_position;
}

ControlNode::Pose2D ControlNode::toSteeringControlPose(const Pose2D& pose) const
{
  if (steering_control_point_ != "rear_bumper") {
    return pose;
  }

  Pose2D control_pose = pose;
  control_pose.x -= rear_axle_offset_ * std::cos(pose.yaw);
  control_pose.y -= rear_axle_offset_ * std::sin(pose.yaw);
  return control_pose;
}

ControlNode::PathPoint ControlNode::toSteeringControlPoint(const PathPoint& point) const
{
  if (steering_control_point_ != "rear_bumper") {
    return point;
  }

  PathPoint control_point = point;
  control_point.x -= rear_axle_offset_ * std::cos(point.yaw);
  control_point.y -= rear_axle_offset_ * std::sin(point.yaw);
  return control_point;
}

double ControlNode::computeSteeringCommand(const Pose2D& pose,
                                           const PathPoint& feedback_reference,
                                           const PathPoint& feedforward_reference,
                                           double speed,
                                           int direction) const
{
  const Pose2D control_pose = toSteeringControlPose(pose);
  const PathPoint control_feedback_reference = toSteeringControlPoint(feedback_reference);
  const PathPoint control_feedforward_reference = toSteeringControlPoint(feedforward_reference);

  // cte 부호 규약:
  // 경로 yaw 기준 차량이 왼쪽에 있으면 cte가 음수가 된다.
  // CarMaker/teleop 기준 positive steer가 좌회전이므로, 왼쪽으로 벗어난 차량은 음수 조향으로
  // 오른쪽 복귀를 하게 된다.
  const double dx = control_pose.x - control_feedback_reference.x;
  const double dy = control_pose.y - control_feedback_reference.y;
  const double cte = std::sin(control_feedback_reference.yaw) * dx -
                     std::cos(control_feedback_reference.yaw) * dy;
  const double heading_error = normalizeAngle(control_feedback_reference.yaw - control_pose.yaw);

  const double tire_steer =
      stanley_.calculate(cte, heading_error, speed, direction, reverse_steering_scale_);
  const double ff_direction_sign =
      direction < 0 ? reverse_curvature_ff_sign_ : 1.0;
  const double curvature_ff =
      curvature_ff_gain_ * ff_direction_sign *
      std::atan(wheelbase_ * control_feedforward_reference.curvature);
  const double steer_command =
      steering_command_sign_ * (tire_steer + curvature_ff) * steering_ratio_;

  return clamp(steer_command, -max_steer_command_, max_steer_command_);
}

double ControlNode::selectTargetSpeed(const PathSegment& segment, std::size_t target_index) const
{
  if (segment.points.empty()) {
    return 0.0;
  }

  target_index = std::min(target_index, segment.points.size() - 1);
  double target_speed = segment.points[target_index].target_speed;
  if (!std::isfinite(target_speed)) {
    target_speed = 0.0;
  }

  // segment 완료 판정은 controlTimerCallback()에서 이미 먼저 수행된다.
  // 아직 완료가 아니라면 lookahead가 끝점의 0 속도를 읽더라도 최소 속도를 유지해야
  // 끝점 근처에서 멈춘 채 다음 segment로 넘어가지 못하는 상황을 피할 수 있다.
  target_speed = std::max(target_speed, min_tracking_speed_);

  return clamp(target_speed, 0.0, max_target_speed_);
}

void ControlNode::advanceOrStop(const PathSegment& active_segment,
                                std::size_t active_segment_index,
                                const Pose2D& pose,
                                double current_speed,
                                const ros::Time& now)
{
  if (current_speed > gear_switch_speed_) {
    // 아직 움직이는 중이면 현재 gear를 유지한 채 brake를 걸어 segment 끝에서 정지시킨다.
    publishStop(active_segment.direction);
    return;
  }

  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (active_segment_index != active_segment_index_) {
    return;
  }

  speed_pid_.reset();
  nearest_index_ = 0;
  presteer_active_ = false;
  if (active_segment_index_ + 1 < segments_.size()) {
    // 충분히 느려진 뒤 다음 segment로 넘어간다.
    // 다음 segment 조향을 정지 상태에서 먼저 보낸 뒤 PID/Stanley 추종을 재개한다.
    ++active_segment_index_;
    PathSegment& next_segment = segments_[active_segment_index_];
    double next_steer_command = 0.0;
    if (presteer_enabled_ && presteer_duration_ > 0.0 && !next_segment.points.empty()) {
      const std::size_t next_nearest_index = findNearestIndex(next_segment, pose, 0);
      const double presteer_lookahead =
          clamp(lookahead_distance_, min_lookahead_distance_, max_lookahead_distance_);
      const std::size_t next_target_index =
          findLookaheadIndex(next_segment, next_nearest_index, presteer_lookahead);
      next_steer_command =
          computeSteeringCommand(pose, next_segment.points[next_nearest_index],
                                 next_segment.points[next_target_index],
                                 min_tracking_speed_, next_segment.direction);
      presteer_active_ = true;
      presteer_segment_index_ = active_segment_index_;
      presteer_until_ = now + ros::Duration(presteer_duration_);
      presteer_steer_command_ = next_steer_command;
    }

    ROS_INFO("Switching trajectory segment: %zu/%zu, direction=%s",
             active_segment_index_ + 1, segments_.size(),
             next_segment.direction > 0 ? "forward" : "reverse");
    publishStop(next_segment.direction, next_steer_command);
  } else {
    trajectory_completed_ = true;
    presteer_active_ = false;
    ROS_INFO("Trajectory complete.");
    publishStop(0);
  }
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
  debug_active_path_pub_ =
      nh_.advertise<nav_msgs::Path>(debug_topic_prefix_ + "/active_segment_path", 1, true);

  debug_current_speed_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/current_speed", 10);
  debug_target_speed_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/target_speed", 10);
  debug_speed_error_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/speed_error", 10);
  debug_steer_command_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/steer_command", 10);
  debug_curvature_ff_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/curvature_feedforward", 10);
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
  debug_distance_to_end_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/distance_to_segment_end", 10);
  debug_trajectory_age_pub_ =
      nh_.advertise<std_msgs::Float64>(debug_topic_prefix_ + "/trajectory_age", 10);
  debug_trajectory_completed_pub_ =
      nh_.advertise<std_msgs::Int32>(debug_topic_prefix_ + "/trajectory_completed", 10);
  debug_direction_pub_ =
      nh_.advertise<std_msgs::Int32>(debug_topic_prefix_ + "/direction", 10);
  debug_tracking_state_pub_ =
      nh_.advertise<std_msgs::Int32>(debug_topic_prefix_ + "/tracking_state", 10);

  ROS_INFO("Control debug topics enabled at %s (frame=%s)",
           debug_topic_prefix_.c_str(), debug_frame_id_.c_str());
}

void ControlNode::publishDebugTelemetry(const Pose2D& pose,
                                        double signed_speed,
                                        const PathSegment* segment,
                                        std::size_t segment_index,
                                        std::size_t nearest_index,
                                        std::size_t target_index,
                                        double target_speed,
                                        double lookahead,
                                        int tracking_state,
                                        double steer_command)
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
  double distance_to_end = -1.0;
  double trajectory_age = -1.0;
  int direction = 0;
  int segment_index_value = segment ? static_cast<int>(segment_index) : -1;
  int nearest_index_value = -1;
  int target_index_value = -1;
  int segment_count = 0;
  int trajectory_completed = 0;
  int steer_saturated = 0;

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    segment_count = static_cast<int>(segments_.size());
    trajectory_completed = trajectory_completed_ ? 1 : 0;
    if (trajectory_received_) {
      trajectory_age = (stamp - last_trajectory_time_).toSec();
    }
  }

  publishPoseDebug(debug_current_pose_pub_, pose.x, pose.y, pose.yaw, stamp);
  const Pose2D control_pose = toSteeringControlPose(pose);
  publishPoseDebug(debug_current_control_pose_pub_,
                   control_pose.x, control_pose.y, control_pose.yaw, stamp);

  if (segment && !segment->points.empty()) {
    direction = segment->direction;
    const std::size_t nearest = std::min(nearest_index, segment->points.size() - 1);
    const std::size_t target = std::min(target_index, segment->points.size() - 1);
    nearest_index_value = static_cast<int>(nearest);
    target_index_value = static_cast<int>(target);
    const PathPoint& nearest_point = segment->points[nearest];
    const PathPoint& target_point = segment->points[target];
    const PathPoint control_nearest_point = toSteeringControlPoint(nearest_point);
    const PathPoint control_target_point = toSteeringControlPoint(target_point);
    distance_to_end = distance2D(pose, segment->points.back());

    publishPoseDebug(debug_nearest_pose_pub_,
                     nearest_point.x, nearest_point.y, nearest_point.yaw, stamp);
    publishPoseDebug(debug_nearest_control_pose_pub_,
                     control_nearest_point.x, control_nearest_point.y,
                     control_nearest_point.yaw, stamp);
    publishPoseDebug(debug_lookahead_pose_pub_,
                     target_point.x, target_point.y, target_point.yaw, stamp);

    const double dx = control_pose.x - control_nearest_point.x;
    const double dy = control_pose.y - control_nearest_point.y;
    cte = std::sin(control_nearest_point.yaw) * dx -
          std::cos(control_nearest_point.yaw) * dy;
    heading_error = normalizeAngle(control_nearest_point.yaw - control_pose.yaw);
    const double ff_direction_sign =
        direction < 0 ? reverse_curvature_ff_sign_ : 1.0;
    curvature_ff =
        steering_command_sign_ * steering_ratio_ * curvature_ff_gain_ *
        ff_direction_sign * std::atan(wheelbase_ * control_target_point.curvature);
    steer_saturated =
        std::abs(steer_command) >= 0.98 * std::max(1e-6, max_steer_command_) ? 1 : 0;

    const bool should_publish_path =
        last_debug_path_time_.isZero() ||
        debug_path_period_ <= 0.0 ||
        (stamp - last_debug_path_time_).toSec() >= debug_path_period_;
    if (should_publish_path) {
      publishActiveSegmentPath(*segment, stamp);
    }
  } else {
    target_speed = 0.0;
    lookahead = nan;
  }

  std_msgs::Float64 float_msg;
  float_msg.data = current_speed;
  debug_current_speed_pub_.publish(float_msg);
  float_msg.data = target_speed;
  debug_target_speed_pub_.publish(float_msg);
  float_msg.data = target_speed - current_speed;
  debug_speed_error_pub_.publish(float_msg);
  float_msg.data = steer_command;
  debug_steer_command_pub_.publish(float_msg);
  float_msg.data = curvature_ff;
  debug_curvature_ff_pub_.publish(float_msg);
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
  int_msg.data = trajectory_completed;
  debug_trajectory_completed_pub_.publish(int_msg);
  int_msg.data = steer_saturated;
  debug_steer_saturated_pub_.publish(int_msg);
  int_msg.data = direction;
  debug_direction_pub_.publish(int_msg);
  int_msg.data = tracking_state;
  debug_tracking_state_pub_.publish(int_msg);
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

void ControlNode::publishActiveSegmentPath(const PathSegment& segment, const ros::Time& stamp)
{
  nav_msgs::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = debug_frame_id_;
  path.poses.reserve(segment.points.size());

  for (const PathPoint& point : segment.points) {
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

  carmaker_msgs::Control_Signal msg;
  msg.header.stamp = ros::Time::now();
  msg.steerangle = static_cast<float>(steer_command);
  msg.gas = static_cast<float>(gas_norm * max_gas_);
  msg.brake = static_cast<float>(brake_norm * max_brake_);
  msg.accel = static_cast<float>(direction * (gas_norm * max_accel_ - brake_norm * max_decel_));
  msg.gear = directionToGear(direction, drive_gear_, reverse_gear_);
  control_pub_.publish(msg);
}

void ControlNode::publishStop(int direction, double steer_command)
{
  // direction=0이면 경로가 없거나 완료된 상태이므로 neutral gear로 brake를 유지한다.
  // direction이 있으면 현재/다음 gear를 유지한 채 멈춤 명령을 내 기어 전환 타이밍을 안정화한다.
  carmaker_msgs::Control_Signal msg;
  msg.header.stamp = ros::Time::now();
  msg.steerangle = static_cast<float>(steer_command);
  msg.gas = 0.0F;
  msg.brake = static_cast<float>(clamp(stop_brake_, 0.0, max_brake_));
  msg.gear = direction == 0 ? neutral_gear_ : directionToGear(direction, drive_gear_, reverse_gear_);
  msg.accel = direction == 0
                  ? 0.0F
                  : static_cast<float>(direction * -msg.brake * max_decel_);
  control_pub_.publish(msg);
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
