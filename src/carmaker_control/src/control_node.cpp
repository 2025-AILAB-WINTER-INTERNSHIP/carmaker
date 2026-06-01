#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <carmaker_msgs/Control_Signal.h>
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

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}
}  // namespace

// /planning/trajectory와 /localization/odom을 받아 /carmaker/control_signal을 내보내는 제어 노드.
//
// 실제 파이프라인은 segmentation -> localization -> planning -> control 순서다.
// 따라서 control은 CarMaker GT pose나 별도 TF lookup을 다시 보지 않고,
// localization이 publish한 Odometry를 현재 상태의 단일 출처로 사용한다.
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
  double computeSteeringCommand(const Pose2D& pose,
                                const PathPoint& reference,
                                double speed,
                                int direction) const;
  double selectTargetSpeed(const PathSegment& segment, std::size_t target_index) const;

  void advanceOrStop(const PathSegment& active_segment,
                     std::size_t active_segment_index,
                     double current_speed);
  void publishControl(int direction, double steer_command, double accel_command);
  void publishStop(int direction, double steer_command = 0.0);

  static double distance2D(const Pose2D& pose, const PathPoint& point);
  static double distance2D(const PathPoint& a, const PathPoint& b);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber trajectory_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher control_pub_;
  ros::Timer control_timer_;

  mutable std::mutex odom_mutex_;
  nav_msgs::Odometry latest_odom_;
  ros::Time last_odom_time_;
  bool odom_received_{false};

  mutable std::mutex trajectory_mutex_;
  std::vector<PathSegment> segments_;
  ros::Time last_trajectory_time_;
  std::size_t active_segment_index_{0};
  std::size_t nearest_index_{0};
  bool trajectory_received_{false};
  bool trajectory_completed_{false};

  PID speed_pid_;
  Stanley stanley_;

  std::string trajectory_topic_;
  std::string odom_topic_;
  std::string control_topic_;

  bool publish_stop_without_path_{true};
  double control_rate_{30.0};
  double odom_timeout_{0.5};
  double trajectory_timeout_{5.0};

  double direction_velocity_epsilon_{0.02};
  double min_tracking_speed_{0.2};
  double max_target_speed_{3.0};
  double lookahead_distance_{0.8};
  double lookahead_time_{0.4};
  double min_lookahead_distance_{0.4};
  double max_lookahead_distance_{3.0};
  int nearest_search_back_{20};
  int nearest_search_ahead_{120};

  double segment_finish_distance_{0.5};
  int segment_finish_index_margin_{3};
  double gear_switch_speed_{0.08};

  double max_accel_{2.0};
  double max_decel_{2.5};
  double max_gas_{0.7};
  double max_brake_{1.0};
  double stop_brake_{0.4};

  int drive_gear_{1};
  int neutral_gear_{0};
  int reverse_gear_{-1};

  double steering_ratio_{1.0};
  double max_steer_command_{0.61};
  double reverse_steering_scale_{1.0};

  ros::Time last_control_time_;
};

ControlNode::ControlNode()
  : nh_(),
    pnh_("~")
{
  loadParameters();

  // planning 결과와 localization 결과를 받고 최종 CarMaker 제어 명령을 publish한다.
  trajectory_sub_ = nh_.subscribe(trajectory_topic_, 1, &ControlNode::trajectoryCallback, this);
  odom_sub_ = nh_.subscribe(odom_topic_, 30, &ControlNode::odomCallback, this);
  control_pub_ = nh_.advertise<carmaker_msgs::Control_Signal>(control_topic_, 10);

  // 제어 루프는 trajectory callback과 독립적으로 일정 주기로 돈다.
  // trajectory가 latched publisher이므로 노드를 나중에 켜도 마지막 계획 경로를 받을 수 있다.
  control_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, control_rate_)),
                                   &ControlNode::controlTimerCallback, this);

  ROS_INFO("carmaker_control_node ready. trajectory=%s odom=%s control=%s",
           trajectory_topic_.c_str(), odom_topic_.c_str(), control_topic_.c_str());
}

void ControlNode::loadParameters()
{
  // control은 localization 이후 단계이므로 현재 pose/speed는 /localization/odom에서 받는다.
  pnh_.param<std::string>("topics/subscribe/trajectory", trajectory_topic_, "/planning/trajectory");
  pnh_.param<std::string>("topics/subscribe/odom", odom_topic_, "/localization/odom");
  pnh_.param<std::string>("topics/publish/control", control_topic_, "/carmaker/control_signal");

  pnh_.param("setting/publish_stop_without_path", publish_stop_without_path_, true);

  // trajectory_timeout은 새 경로가 너무 오래된 경우 차량을 정지시키기 위한 안전장치다.
  // Ubuntu ROS PC에서 시뮬레이션 clock을 쓸 경우 ROS time 기준으로 동작한다.
  pnh_.param("control/rate", control_rate_, 30.0);
  pnh_.param("control/odom_timeout", odom_timeout_, 0.5);
  pnh_.param("control/trajectory_timeout", trajectory_timeout_, 5.0);

  // direction_velocity_epsilon보다 작은 속도는 정지점으로 보고 이전 기어 구간에 붙인다.
  // Reeds-Shepp cusp 근처에서 velocity가 0으로 떨어져도 불필요하게 segment가 쪼개지지 않는다.
  pnh_.param("control/direction_velocity_epsilon", direction_velocity_epsilon_, 0.02);
  pnh_.param("control/min_tracking_speed", min_tracking_speed_, 0.2);
  pnh_.param("control/max_target_speed", max_target_speed_, 3.0);

  // lookahead는 속도가 높을수록 조금 멀리 보도록 base + time * speed 형태로 계산한다.
  // 단, 너무 가까워 떨리거나 너무 멀어 코너를 뭉개지 않도록 min/max로 제한한다.
  pnh_.param("control/lookahead_distance", lookahead_distance_, 0.8);
  pnh_.param("control/lookahead_time", lookahead_time_, 0.4);
  pnh_.param("control/min_lookahead_distance", min_lookahead_distance_, 0.4);
  pnh_.param("control/max_lookahead_distance", max_lookahead_distance_, 3.0);

  // nearest search는 이전 index 주변만 훑어 경로가 겹칠 때 뒤쪽 segment로 튀는 것을 줄인다.
  pnh_.param("control/nearest_search_back", nearest_search_back_, 20);
  pnh_.param("control/nearest_search_ahead", nearest_search_ahead_, 120);

  // segment 종료는 index가 끝에 가깝고 실제 위치도 끝점 근처일 때만 인정한다.
  // 후진 전환 지점에서 가까운 다른 경로점으로 잘못 넘어가는 상황을 피하기 위함이다.
  pnh_.param("control/segment_finish_distance", segment_finish_distance_, 0.5);
  pnh_.param("control/segment_finish_index_margin", segment_finish_index_margin_, 3);
  pnh_.param("control/gear_switch_speed", gear_switch_speed_, 0.08);

  pnh_.param("control/max_accel", max_accel_, 2.0);
  pnh_.param("control/max_decel", max_decel_, 2.5);
  pnh_.param("control/max_gas", max_gas_, 0.7);
  pnh_.param("control/max_brake", max_brake_, 1.0);
  pnh_.param("control/stop_brake", stop_brake_, 0.4);
  pnh_.param("control/drive_gear", drive_gear_, 1);
  pnh_.param("control/neutral_gear", neutral_gear_, 0);
  pnh_.param("control/reverse_gear", reverse_gear_, -1);

  // Stanley는 타이어 조향각을 계산한다. 실제 CarMaker 입력이 steering wheel angle이면
  // vehicle/steering_ratio 또는 vehicle/max_steer_command로 변환 범위를 조정한다.
  double max_tire_steer_deg = 35.0;
  pnh_.param("stanley/max_steer_angle_deg", max_tire_steer_deg, max_tire_steer_deg);
  const double max_tire_steer = deg2rad(max_tire_steer_deg);

  double stanley_k = 1.0;
  double stanley_k_soft = 0.3;
  double stanley_cte_gain = 1.0;
  double stanley_heading_gain = 1.0;
  pnh_.param("stanley/k", stanley_k, stanley_k);
  pnh_.param("stanley/k_soft", stanley_k_soft, stanley_k_soft);
  pnh_.param("stanley/cte_gain", stanley_cte_gain, stanley_cte_gain);
  pnh_.param("stanley/heading_gain", stanley_heading_gain, stanley_heading_gain);
  pnh_.param("stanley/reverse_steering_scale", reverse_steering_scale_, 1.0);
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

  pnh_.param("vehicle/steering_ratio", steering_ratio_, 1.0);
  max_steer_command_ = max_tire_steer * steering_ratio_;
  pnh_.param("vehicle/max_steer_command", max_steer_command_, max_steer_command_);
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
  point.direction = direction;
  return point;
}

bool ControlNode::getCurrentState(Pose2D& pose, double& signed_speed) const
{
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
  bool should_track = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    const bool trajectory_fresh =
        trajectory_received_ &&
        !trajectory_completed_ &&
        (ros::Time::now() - last_trajectory_time_).toSec() <= trajectory_timeout_ &&
        active_segment_index_ < segments_.size();

    if (trajectory_fresh) {
      active_segment = segments_[active_segment_index_];
      active_segment_index = active_segment_index_;
      previous_nearest_index = nearest_index_;
      should_track = true;
    }
  }

  if (!should_track) {
    // 경로가 없거나 timeout이면 neutral + brake로 대기한다.
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

  if (isSegmentComplete(active_segment, pose, nearest_index)) {
    // segment 끝에 도착하면 바로 다음 segment로 넘어가지 않고 먼저 충분히 감속한다.
    // 전진/후진 기어 전환을 움직이는 중에 하지 않기 위한 처리다.
    advanceOrStop(active_segment, active_segment_index, current_speed);
    return;
  }

  // 속도 목표는 lookahead point에서 읽고, Stanley 조향은 nearest point 기준 오차로 계산한다.
  // 코너에서 너무 앞 점만 보면 lateral error가 늦게 반영될 수 있어서 둘을 분리한다.
  const double lookahead =
      clamp(lookahead_distance_ + lookahead_time_ * current_speed,
            min_lookahead_distance_, max_lookahead_distance_);
  const std::size_t target_index = findLookaheadIndex(active_segment, nearest_index, lookahead);

  const double target_speed = selectTargetSpeed(active_segment, target_index);
  const double accel_command = speed_pid_.calculate(target_speed, current_speed, dt);
  const double steer_command =
      computeSteeringCommand(pose, active_segment.points[nearest_index],
                             std::max(current_speed, target_speed), active_segment.direction);

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

double ControlNode::computeSteeringCommand(const Pose2D& pose,
                                           const PathPoint& reference,
                                           double speed,
                                           int direction) const
{
  // cte 부호 규약:
  // 경로 yaw 기준 차량이 왼쪽에 있으면 cte가 음수가 된다.
  // CarMaker/teleop 기준 positive steer가 좌회전이므로, 왼쪽으로 벗어난 차량은 음수 조향으로
  // 오른쪽 복귀를 하게 된다.
  const double dx = pose.x - reference.x;
  const double dy = pose.y - reference.y;
  const double cte = std::sin(reference.yaw) * dx - std::cos(reference.yaw) * dy;
  const double heading_error = normalizeAngle(reference.yaw - pose.yaw);

  double tire_steer = stanley_.calculate(cte, heading_error, speed);
  if (direction < 0) {
    // 후진은 같은 조향각이 yaw를 반대 방향으로 변화시키므로 Stanley 결과의 부호를 뒤집는다.
    // 실제 차량/CarMaker 모델에서 후진 조향 반응이 과하거나 부족하면 reverse_steering_scale로 튜닝한다.
    tire_steer = -reverse_steering_scale_ * tire_steer;
  }

  return clamp(tire_steer * steering_ratio_, -max_steer_command_, max_steer_command_);
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

  const bool close_to_segment_end =
      target_index + static_cast<std::size_t>(std::max(0, segment_finish_index_margin_)) >=
      segment.points.size() - 1;
  if (!close_to_segment_end) {
    // segment 중간에서 profiler 속도가 0인 점을 만나면 차량이 멈춰 버릴 수 있다.
    // 끝점 근처가 아닐 때만 최소 추종 속도를 적용한다.
    target_speed = std::max(target_speed, min_tracking_speed_);
  }

  return clamp(target_speed, 0.0, max_target_speed_);
}

void ControlNode::advanceOrStop(const PathSegment& active_segment,
                                std::size_t active_segment_index,
                                double current_speed)
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
  if (active_segment_index_ + 1 < segments_.size()) {
    // 충분히 느려진 뒤 다음 segment로 넘어간다.
    // 여기서 다음 segment의 direction에 맞는 gear로 stop 명령을 한 번 내고,
    // 다음 timer tick부터 PID/Stanley 추종을 재개한다.
    ++active_segment_index_;
    ROS_INFO("Switching trajectory segment: %zu/%zu, direction=%s",
             active_segment_index_ + 1, segments_.size(),
             segments_[active_segment_index_].direction > 0 ? "forward" : "reverse");
    publishStop(segments_[active_segment_index_].direction);
  } else {
    trajectory_completed_ = true;
    ROS_INFO("Trajectory complete.");
    publishStop(0);
  }
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
