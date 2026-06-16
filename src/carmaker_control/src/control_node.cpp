#include "carmaker_control/control_node.h"

#include <algorithm>
#include <atomic>
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
#include <boost/bind/bind.hpp>

#include "carmaker_control/pid.h"
#include "carmaker_control/stanley.h"

namespace carmaker_control {
namespace {
constexpr double kPi = 3.14159265358979323846;

double deg2rad(double degree)
{
  return degree * kPi / 180.0;
}

double rad2deg(double radian)
{
  return radian * 180.0 / kPi;
}

int directionToGear(int direction, int drive_gear, int reverse_gear)
{
  return direction >= 0 ? drive_gear : reverse_gear;
}

double normalizeAngle(double angle)
{
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

ControlNode::ControlNode()
  : nh_(),
    pnh_("~")
{
  loadParameters();

  trajectory_sub_ = nh_.subscribe(trajectory_topic_, 1, &ControlNode::trajectoryCallback, this);

  ros::TransportHints hints;
  hints.tcpNoDelay();
  if (state_source_ == "dynamics") {
    dynamics_sub_ = nh_.subscribe(dynamics_topic_, 2, &ControlNode::dynamicsCallback, this, hints);
  } else {
    odom_sub_ = nh_.subscribe(odom_topic_, 2, &ControlNode::odomCallback, this, hints);
  }
  control_pub_ = nh_.advertise<carmaker_msgs::Control_Signal>(control_topic_, 10);
  advertiseDebugTopics();

  // Initialize Dynamic Reconfigure Server
  reconfigure_server_ = std::make_unique<dynamic_reconfigure::Server<carmaker_control::CarmakerControlConfig>>(pnh_);
  dynamic_reconfigure::Server<carmaker_control::CarmakerControlConfig>::CallbackType cb =
      boost::bind(&ControlNode::reconfigureCallback, this, _1, _2);
  reconfigure_server_->setCallback(cb);

  control_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, control_rate_)),
                                   &ControlNode::controlTimerCallback, this);

  const std::string state_topic = state_source_ == "dynamics" ? dynamics_topic_ : odom_topic_;
  ROS_INFO("carmaker_control_node ready. trajectory=%s state_source=%s state_topic=%s control=%s",
           trajectory_topic_.c_str(), state_source_.c_str(),
           state_topic.c_str(), control_topic_.c_str());
}

void ControlNode::loadParameters()
{
  pnh_.param<std::string>("topics/subscribe/trajectory", trajectory_topic_, "/planning/trajectory");
  pnh_.param<std::string>("topics/subscribe/odom", odom_topic_, "/localization/odom");
  pnh_.param<std::string>("topics/subscribe/dynamics", dynamics_topic_, "/carmaker/dynamic_info");
  pnh_.param<std::string>("topics/publish/control", control_topic_, "/carmaker/control_signal");

  pnh_.param("setting/publish_stop_without_path", publish_stop_without_path_, true);
  pnh_.param<std::string>("control/state_source", state_source_, "odom");
  if (state_source_ != "odom" && state_source_ != "dynamics") {
    ROS_WARN("Unknown control/state_source='%s'. Falling back to 'odom'.", state_source_.c_str());
    state_source_ = "odom";
  }

  pnh_.param("control/rate", control_rate_, 30.0);
  pnh_.param("control/odom_timeout", odom_timeout_, 0.5);
  pnh_.param("control/dynamics_timeout", dynamics_timeout_, 0.5);
  pnh_.param("control/active_trajectory_timeout", active_trajectory_timeout_, 1.0);
  pnh_.param("control/stop_trajectory_velocity_epsilon", stop_trajectory_velocity_epsilon_, 1e-6);
  active_trajectory_timeout_ = std::max(0.0, active_trajectory_timeout_);
  stop_trajectory_velocity_epsilon_ = std::max(0.0, stop_trajectory_velocity_epsilon_);

  pnh_.param("control/min_tracking_speed", min_tracking_speed_, 0.2);
  pnh_.param("control/min_creep_speed", min_creep_speed_, 0.1);
  pnh_.param("control/arrival_slow_distance", arrival_slow_distance_, 0.5);
  pnh_.param("control/alignment_fade_distance", alignment_fade_distance_, 2.0);
  pnh_.param("control/max_target_speed", max_target_speed_, 0.7);
  min_tracking_speed_ = std::max(0.0, min_tracking_speed_);
  min_creep_speed_ = std::max(0.0, min_creep_speed_);
  arrival_slow_distance_ = std::max(0.0, arrival_slow_distance_);
  alignment_fade_distance_ = std::max(0.1, alignment_fade_distance_);
  max_target_speed_ = std::max(0.0, max_target_speed_);

  pnh_.param("control/front_curvature_weight", front_curvature_weight_, 0.5);
  front_curvature_weight_ = std::max(0.0, std::min(1.0, front_curvature_weight_));
  pnh_.param("control/forward/control_lookahead", forward_control_lookahead_, 0.0);
  pnh_.param("control/reverse/control_lookahead", reverse_control_lookahead_, 0.82);

  // Load structured lookahead parameters
  forward_lookahead_ = loadLookaheadParams("forward");
  reverse_lookahead_ = loadLookaheadParams("reverse");

  pnh_.param("control/nearest_search_back", nearest_search_back_, 20);
  pnh_.param("control/nearest_search_ahead", nearest_search_ahead_, 120);

  pnh_.param("control/max_accel", max_accel_, 0.8);
  pnh_.param("control/max_decel", max_decel_, 1.5);
  pnh_.param("control/max_gas", max_gas_, 0.35);
  pnh_.param("control/max_brake", max_brake_, 1.0);
  pnh_.param("control/stop_brake", stop_brake_, 0.4);

  bool release_steer = false;
  pnh_.param("control/steer_release_after_idle", release_steer, false);
  steer_release_after_idle_ = release_steer;
  pnh_.param("control/release_duration", release_duration_, 1.0);
  release_duration_ = std::max(0.0, release_duration_);
  pnh_.param("control/drive_gear", drive_gear_, 1);
  pnh_.param("control/neutral_gear", neutral_gear_, 0);
  pnh_.param("control/reverse_gear", reverse_gear_, -1);

  double min_turning_radius = 5.5;
  pnh_.param("vehicle/wheelbase", wheelbase_, wheelbase_);
  pnh_.param("vehicle/min_turning_radius", min_turning_radius, min_turning_radius);

  double max_tire_steer_deg = 28.4;
  if (wheelbase_ > 1e-6 && min_turning_radius > 1e-6) {
    max_tire_steer_deg = rad2deg(std::atan(wheelbase_ / min_turning_radius));
  }
  pnh_.param("stanley/max_steer_angle_deg", max_tire_steer_deg, max_tire_steer_deg);
  max_tire_steer_ = deg2rad(max_tire_steer_deg);

  // Load structured Stanley parameters
  StanleyParams f_defaults{3.0, 0.3, 2.0, 1.4, 1.0};
  forward_stanley_params_ = loadStanleyParams("forward", f_defaults);
  reverse_stanley_params_ = loadStanleyParams("reverse", forward_stanley_params_);

  pnh_.param("stanley/reverse/reverse_curvature_ff_sign", reverse_curvature_ff_sign_, -1.0);
  reverse_curvature_ff_sign_ = reverse_curvature_ff_sign_ < 0.0 ? -1.0 : 1.0;

  forward_stanley_.configure(forward_stanley_params_.k, forward_stanley_params_.k_soft, max_tire_steer_,
                             forward_stanley_params_.cte_gain, forward_stanley_params_.heading_gain);

  reverse_stanley_.configure(reverse_stanley_params_.k, reverse_stanley_params_.k_soft, max_tire_steer_,
                             reverse_stanley_params_.cte_gain, reverse_stanley_params_.heading_gain);

  double speed_kp = 1.0, speed_ki = 0.0, speed_kd = 0.05;
  pnh_.param("pid/speed/kp", speed_kp, speed_kp);
  pnh_.param("pid/speed/ki", speed_ki, speed_ki);
  pnh_.param("pid/speed/kd", speed_kd, speed_kd);
  speed_pid_.configure(speed_kp, speed_ki, speed_kd, -max_decel_, max_accel_);

  pnh_.param("vehicle/steering_ratio", steering_ratio_, 9.0);
  max_steer_command_ = max_tire_steer_ * steering_ratio_;
  pnh_.param("vehicle/max_steer_command", max_steer_command_, max_steer_command_);
  pnh_.param("vehicle/steering_command_sign", steering_command_sign_, 1.0);
  steering_command_sign_ = steering_command_sign_ < 0.0 ? -1.0 : 1.0;

  pnh_.param("debug/publish", publish_debug_, true);
  pnh_.param<std::string>("debug/topic_prefix", debug_topic_prefix_, "/control/debug");
  pnh_.param<std::string>("debug/frame_id", debug_frame_id_, "Fr0");
  pnh_.param("debug/path_publish_period", debug_path_period_, 1.0);
}

ControlNode::LookaheadParams ControlNode::loadLookaheadParams(const std::string& direction)
{
  LookaheadParams p;
  pnh_.param("control/" + direction + "/lookahead_distance", p.distance, 0.35);
  pnh_.param("control/" + direction + "/lookahead_time", p.time, 0.2);
  pnh_.param("control/" + direction + "/min_lookahead_distance", p.min_distance, 0.2);
  pnh_.param("control/" + direction + "/max_lookahead_distance", p.max_distance, 0.8);
  pnh_.param("control/" + direction + "/curvature_preview_distance", p.curvature_preview_distance, 0.6);
  pnh_.param("control/" + direction + "/curvature_preview_window", p.curvature_preview_window, 0.4);
  return p;
}

ControlNode::StanleyParams ControlNode::loadStanleyParams(const std::string& direction, const StanleyParams& defaults)
{
  StanleyParams p;
  pnh_.param("stanley/" + direction + "/k", p.k, defaults.k);
  pnh_.param("stanley/" + direction + "/k_soft", p.k_soft, defaults.k_soft);
  pnh_.param("stanley/" + direction + "/cte_gain", p.cte_gain, defaults.cte_gain);
  pnh_.param("stanley/" + direction + "/heading_gain", p.heading_gain, defaults.heading_gain);
  pnh_.param("stanley/" + direction + "/curvature_ff_gain", p.curvature_ff_gain, defaults.curvature_ff_gain);
  return p;
}

void ControlNode::trajectoryCallback(const carmaker_msgs::TrajectoryPathConstPtr& msg)
{
  if (!msg) {
    ROS_WARN_THROTTLE(1.0, "Received null trajectory message.");
    return;
  }
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  latest_trajectory_msg_ = msg;
  trajectory_received_ = true;
  last_trajectory_time_ = ros::Time::now();
  idle_control_released_ = false;
}

void ControlNode::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  if (!msg) return;
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = *msg;
  last_odom_time_ = ros::Time::now();
  odom_received_ = true;
}

void ControlNode::dynamicsCallback(const carmaker_msgs::DynamicsInfoConstPtr& msg)
{
  if (!msg) return;
  std::lock_guard<std::mutex> lock(dynamics_mutex_);
  latest_dynamics_ = *msg;
  last_dynamics_time_ = ros::Time::now();
  dynamics_received_ = true;
}

bool ControlNode::processTrajectory(const carmaker_msgs::TrajectoryPathConstPtr& msg)
{
  if (msg->points.size() < 2) {
    ROS_WARN_THROTTLE(1.0, "Received trajectory with fewer than 2 points.");
    return false;
  }

  const int new_direction = trajectoryDirection(*msg);
  if (new_direction == 0) {
    ROS_WARN_THROTTLE(1.0, "Active trajectory has no valid direction field; ignoring.");
    return false;
  }

  ActiveTrajectory new_trajectory;
  new_trajectory.points.reserve(msg->points.size());
  bool mixed_direction = false;
  double s_accumulated = 0.0;
  for (std::size_t i = 0; i < msg->points.size(); ++i) {
    const auto& point = msg->points[i];
    if (point.direction == 0 || (point.direction < 0 ? -1 : 1) != new_direction) {
      mixed_direction = true;
    }
    PathPoint pt = makePathPoint(point, new_direction);
    if (i > 0) {
      s_accumulated += distance2D(new_trajectory.points.back(), pt);
    }
    pt.s = s_accumulated;
    new_trajectory.points.push_back(pt);
  }

  if (mixed_direction) {
    ROS_WARN_THROTTLE(1.0, "Active trajectory violates single-direction contract; ignoring.");
    return false;
  }

  const bool new_stop_trajectory = isStopTrajectory(new_trajectory.points);
  const bool had_trajectory = !active_trajectory_.empty();
  std::size_t preserved_nearest_index = 0;
  const bool preserved_index = (!new_stop_trajectory && had_trajectory && active_trajectory_.direction() == new_direction);
  if (preserved_index) {
    preserved_nearest_index = std::min(nearest_index_, new_trajectory.points.size() - 1);
  }

  active_trajectory_ = std::move(new_trajectory);
  nearest_index_ = preserved_nearest_index;
  resetIdleRelease();

  ROS_INFO_THROTTLE(1.0,
                    "Active trajectory processed: %zu points direction=%s nearest_index=%zu (%s)",
                    msg->points.size(),
                    new_direction > 0 ? "D" : "R",
                    preserved_nearest_index,
                    preserved_index ? "preserved" : "reset");
  return true;
}

ControlNode::PathPoint
ControlNode::makePathPoint(const carmaker_msgs::TrajectoryPoint& msg, int direction) const
{
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

bool ControlNode::getCurrentState(VehicleState& state) const
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

    state.pose.x = dynamics.RearAxle_x;
    state.pose.y = dynamics.RearAxle_y;
    state.pose.yaw = normalizeAngle(dynamics.Car_Yaw);
    state.signed_speed = dynamics.Car_vx;
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

  state.pose.x = odom.pose.pose.position.x;
  state.pose.y = odom.pose.pose.position.y;
  state.pose.yaw = normalizeAngle(quaternionToYaw(odom.pose.pose.orientation));
  state.signed_speed = odom.twist.twist.linear.x;
  return true;
}

void ControlNode::controlTimerCallback(const ros::TimerEvent& event)
{
  const ros::Time now = ros::Time::now();
  double dt = (now - last_control_time_).toSec();

  // Time Jump Reset
  if (dt < 0.0) {
    ROS_WARN_THROTTLE(1.0, "Time jump backward detected (dt = %.4f s). Resetting state.", dt);
    speed_pid_.reset();
    resetIdleRelease();
    nearest_index_ = 0;
    dt = 1.0 / std::max(1.0, control_rate_);
  } else if (last_control_time_.isZero() || dt <= 1e-4 || dt > 1.0) {
    const double wall_dt = event.last_real.isZero()
                               ? 1.0 / std::max(1.0, control_rate_)
                               : (event.current_real - event.last_real).toSec();
    dt = wall_dt > 1e-4 ? wall_dt : 1.0 / std::max(1.0, control_rate_);
  }
  last_control_time_ = now;

  if (idle_control_released_) {
    return;
  }

  VehicleState state;
  if (!getCurrentState(state)) {
    publishStopReason(1);
    publishStop(0);
    return;
  }

  carmaker_msgs::TrajectoryPathConstPtr current_msg;
  ros::Time trajectory_time;
  bool has_traj_msg = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    has_traj_msg = trajectory_received_;
    if (has_traj_msg) {
      current_msg = latest_trajectory_msg_;
      trajectory_time = last_trajectory_time_;
    }
  }

  if (has_traj_msg && current_msg) {
    if (current_msg != processed_trajectory_msg_) {
      processTrajectory(current_msg);
      processed_trajectory_msg_ = current_msg;
    }
  }

  const bool trajectory_fresh =
      has_traj_msg &&
      (now - trajectory_time).toSec() <= active_trajectory_timeout_ &&
      !active_trajectory_.empty();

  if (!trajectory_fresh) {
    publishDebugTelemetry(state.pose, state.pose, state.signed_speed, nullptr, 0, 0, 0, 0.0, 0.0, 0, 0.0, 2);
    publishIdleStopUntilRelease(now);
    speed_pid_.reset();
    return;
  }

  resetIdleRelease();

  const double current_speed = std::abs(state.signed_speed);
  nearest_index_ = findNearestIndex(active_trajectory_, state.pose, nearest_index_);

  Pose2D tracking_pose = state.pose;
  if (active_trajectory_.direction() > 0) {
    tracking_pose.x = state.pose.x + (wheelbase_ + forward_control_lookahead_) * std::cos(state.pose.yaw);
    tracking_pose.y = state.pose.y + (wheelbase_ + forward_control_lookahead_) * std::sin(state.pose.yaw);
  } else {
    tracking_pose.x = state.pose.x - reverse_control_lookahead_ * std::cos(state.pose.yaw);
    tracking_pose.y = state.pose.y - reverse_control_lookahead_ * std::sin(state.pose.yaw);
  }


  const double distance_to_end = computeRemainingDistance(active_trajectory_.points, nearest_index_, state.pose, active_trajectory_.direction());

  const int dir = active_trajectory_.direction();
  const LookaheadParams& lp = (dir >= 0) ? forward_lookahead_ : reverse_lookahead_;

  const double lookahead = clamp(lp.distance + lp.time * current_speed, lp.min_distance, lp.max_distance);

  const std::size_t nearest_index_tracking = findNearestIndex(active_trajectory_, tracking_pose, nearest_index_);
  const std::size_t target_index = findLookaheadIndex(active_trajectory_, nearest_index_tracking, lookahead);
  const std::size_t target_index_speed = findLookaheadIndex(active_trajectory_, nearest_index_, lookahead);
  std::size_t preview_index = 0;
  const double preview_curvature = computePreviewCurvature(active_trajectory_, nearest_index_tracking, preview_index);

  const TargetSpeedDecision speed_decision = selectTargetSpeed(active_trajectory_, nearest_index_, target_index_speed, distance_to_end);
  const double target_speed = speed_decision.target_speed;
  const double steer_command = computeSteeringCommand(tracking_pose, active_trajectory_.points[nearest_index_tracking],
                                preview_curvature,
                                std::max(current_speed, target_speed),
                                active_trajectory_.direction(),
                                active_trajectory_.points[nearest_index_].curvature,
                                distance_to_end);

  if (isStopTrajectory(active_trajectory_.points)) {
    publishDebugTelemetry(state.pose, tracking_pose, state.signed_speed, &active_trajectory_,
                          nearest_index_, nearest_index_tracking, target_index, 0.0, lookahead, 2, steer_command,
                          3, 0.0, kSpeedReasonAllZeroStop,
                          preview_index, lp.curvature_preview_distance, preview_curvature);
    speed_pid_.reset();
    publishStop(active_trajectory_.direction(), steer_command);
    return;
  }

  const double accel_command = speed_pid_.calculate(target_speed, current_speed, dt);

  publishDebugTelemetry(state.pose, tracking_pose, state.signed_speed, &active_trajectory_,
                        nearest_index_, nearest_index_tracking, target_index, target_speed, lookahead, 1, steer_command,
                        0, speed_decision.raw_target_speed, speed_decision.reason,
                        preview_index, lp.curvature_preview_distance, preview_curvature);
  publishControl(active_trajectory_.direction(), steer_command, accel_command);
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

  const int direction = trajectory.direction();
  const LookaheadParams& lp = (direction >= 0) ? forward_lookahead_ : reverse_lookahead_;

  const double preview_distance = std::max(0.0, lp.curvature_preview_distance);
  preview_index = findLookaheadIndex(trajectory, nearest_index, preview_distance);

  const double half_window = 0.5 * std::max(0.0, lp.curvature_preview_window);
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

  const double fallback_curvature = path[std::min(preview_index, path.size() - 1)].curvature;
  return std::isfinite(fallback_curvature) ? fallback_curvature : 0.0;
}

double ControlNode::computeCurvatureFeedforward(double preview_curvature, int direction, double ff_gain) const
{
  if (!std::isfinite(preview_curvature)) {
    return 0.0;
  }
  const double ff_direction_sign = direction < 0 ? reverse_curvature_ff_sign_ : 1.0;
  return ff_gain * ff_direction_sign * std::atan(wheelbase_ * preview_curvature);
}

double ControlNode::computeOffTrackingOffset(double wheelbase, double effective_curvature) const
{
  double off_tracking_offset = 0.0;
  const double abs_kappa = std::abs(effective_curvature);
  const double L2 = wheelbase * wheelbase;
  if (abs_kappa >= 1e-4) {
    off_tracking_offset = std::copysign(std::sqrt(1.0 / (abs_kappa * abs_kappa) + L2) - 1.0 / abs_kappa, effective_curvature);
  } else {
    const double term1 = L2 * effective_curvature / 2.0;
    const double term3 = L2 * L2 * effective_curvature * effective_curvature * effective_curvature / 8.0;
    off_tracking_offset = term1 - term3;
  }
  return off_tracking_offset;
}

double ControlNode::computeSteeringCommand(const Pose2D& pose,
                                           const PathPoint& feedback_reference,
                                           double preview_curvature,
                                           double speed,
                                           int direction,
                                           double rear_curvature,
                                           double distance_to_end) const
{
  const double dx = pose.x - feedback_reference.x;
  const double dy = pose.y - feedback_reference.y;
  double cte = std::sin(feedback_reference.yaw) * dx - std::cos(feedback_reference.yaw) * dy;

  // Apply geometric off-tracking compensation for forward driving.
  // Use min(|κ_rear|, |κ_front|) to prevent over-compensation at curve transitions:
  //   - Corner entry  (κ_rear≈0, κ_front>0): min→0, no premature offset
  //   - Steady curve  (κ_rear≈κ_front):      min→κ, full compensation
  //   - Corner exit   (κ_rear>0, κ_front≈0): min→0, no lingering offset
  if (direction >= 0) {
    // const double front_path_curvature = feedback_reference.curvature;
    // const double min_abs_kappa = std::min(std::abs(rear_curvature), std::abs(front_path_curvature));
    // const double effective_curvature = std::copysign(min_abs_kappa, rear_curvature);
    // cte -= computeOffTrackingOffset(wheelbase_, effective_curvature);

    // cte -= computeOffTrackingOffset(wheelbase_, rear_curvature);

    const double front_path_curvature = feedback_reference.curvature;
    const double effective_curvature = (1.0 - front_curvature_weight_) * rear_curvature + front_curvature_weight_ * front_path_curvature;
    const double off_tracking_offset = computeOffTrackingOffset(wheelbase_, effective_curvature);

    // 최종 지점 정렬을 위해 남은 거리에 비례하여 오프셋 선형 감쇄(Fade-out)
    double fade_factor = 1.0;
    if (alignment_fade_distance_ > 0.0) {
      fade_factor = clamp(distance_to_end / alignment_fade_distance_, 0.0, 1.0);
    }
    cte -= off_tracking_offset * fade_factor;
  }

  const double heading_error = normalizeAngle(feedback_reference.yaw - pose.yaw);

  double tire_steer = 0.0;
  double curvature_ff = 0.0;

  if (direction >= 0) {
    tire_steer = forward_stanley_.calculate(cte, heading_error, speed, 1, 1.0);
    curvature_ff = computeCurvatureFeedforward(preview_curvature, direction, forward_stanley_params_.curvature_ff_gain);
  } else {
    tire_steer = reverse_stanley_.calculate(cte, heading_error, speed, -1, 1.0);
    curvature_ff = computeCurvatureFeedforward(preview_curvature, direction, reverse_stanley_params_.curvature_ff_gain);
  }

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
  (void)direction;
  if (path.empty()) {
    return 0.0;
  }
  const std::size_t idx = std::min(nearest_index, path.size() - 1);
  const double s_goal = path.back().s - wheelbase_ * 1.5;
  double dist_to_end = std::hypot(pose.x - path[idx].x, pose.y - path[idx].y) + (s_goal - path[idx].s);
  return std::max(0.0, dist_to_end);
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
                                             return !std::isfinite(point.target_speed) || point.target_speed <= eps;
                                           });
  if (stopped_segment) {
    decision.reason = kSpeedReasonAllZeroStop;
    return decision;
  }

  decision.raw_target_speed = std::isfinite(path[target_index].target_speed) ? path[target_index].target_speed : 0.0;
  double target_speed = decision.raw_target_speed;

  const bool has_future_tracking_speed = std::any_of(path.begin() + nearest_index, path.end(),
                                                     [min_tracking_speed](const PathPoint& point) {
                                                       return std::isfinite(point.target_speed) && point.target_speed > min_tracking_speed;
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

bool ControlNode::isStopTrajectory(const std::vector<PathPoint>& points) const
{
  return std::all_of(points.begin(), points.end(),
                     [this](const PathPoint& pt) {
                       return pt.target_speed <= stop_trajectory_velocity_epsilon_;
                     });
}

template <typename T>
ros::Publisher ControlNode::advertiseDebug(const std::string& suffix, uint32_t queue_size, bool latch)
{
  return nh_.advertise<T>(debug_topic_prefix_ + "/" + suffix, queue_size, latch);
}

void ControlNode::advertiseDebugTopics()
{
  if (!publish_debug_) {
    return;
  }

  if (!debug_topic_prefix_.empty() && debug_topic_prefix_.back() == '/') {
    debug_topic_prefix_.pop_back();
  }

  debug_pubs_.rear_axle_pose = advertiseDebug<geometry_msgs::PoseStamped>("rear_axle_pose");
  debug_pubs_.front_axle_pose = advertiseDebug<geometry_msgs::PoseStamped>("front_axle_pose");
  debug_pubs_.tracking_pose = advertiseDebug<geometry_msgs::PoseStamped>("tracking_pose");
  debug_pubs_.nearest_pose = advertiseDebug<geometry_msgs::PoseStamped>("nearest_pose");
  debug_pubs_.control_pose = advertiseDebug<geometry_msgs::PoseStamped>("control_pose");
  debug_pubs_.lookahead_pose = advertiseDebug<geometry_msgs::PoseStamped>("lookahead_pose");
  debug_pubs_.feedforward_pose = advertiseDebug<geometry_msgs::PoseStamped>("feedforward_pose");
  debug_pubs_.active_path = advertiseDebug<nav_msgs::Path>("active_path", 1, true);

  debug_pubs_.current_speed = advertiseDebug<std_msgs::Float64>("current_speed");
  debug_pubs_.target_speed = advertiseDebug<std_msgs::Float64>("target_speed");
  debug_pubs_.raw_lookahead_speed = advertiseDebug<std_msgs::Float64>("raw_lookahead_speed");
  debug_pubs_.speed_error = advertiseDebug<std_msgs::Float64>("speed_error");
  debug_pubs_.steer_command = advertiseDebug<std_msgs::Float64>("steer_command");
  debug_pubs_.curvature_feedforward = advertiseDebug<std_msgs::Float64>("curvature_feedforward");
  debug_pubs_.curvature_preview = advertiseDebug<std_msgs::Float64>("curvature_preview");
  debug_pubs_.curvature_preview_distance = advertiseDebug<std_msgs::Float64>("curvature_preview_distance");
  debug_pubs_.steer_saturated = advertiseDebug<std_msgs::Int32>("steer_saturated");
  debug_pubs_.cross_track_error = advertiseDebug<std_msgs::Float64>("cross_track_error");
  debug_pubs_.heading_error = advertiseDebug<std_msgs::Float64>("heading_error");
  debug_pubs_.lookahead_distance = advertiseDebug<std_msgs::Float64>("lookahead_distance");
  debug_pubs_.segment_index = advertiseDebug<std_msgs::Int32>("segment_index");
  debug_pubs_.segment_count = advertiseDebug<std_msgs::Int32>("segment_count");
  debug_pubs_.nearest_index = advertiseDebug<std_msgs::Int32>("nearest_index");
  debug_pubs_.target_index = advertiseDebug<std_msgs::Int32>("target_index");
  debug_pubs_.curvature_preview_index = advertiseDebug<std_msgs::Int32>("curvature_preview_index");
  debug_pubs_.distance_to_segment_end = advertiseDebug<std_msgs::Float64>("distance_to_segment_end");
  debug_pubs_.trajectory_age = advertiseDebug<std_msgs::Float64>("trajectory_age");
  debug_pubs_.direction = advertiseDebug<std_msgs::Int32>("direction");
  debug_pubs_.tracking_state = advertiseDebug<std_msgs::Int32>("tracking_state");
  debug_pubs_.stop_reason = advertiseDebug<std_msgs::Int32>("stop_reason");
  debug_pubs_.speed_selection_reason = advertiseDebug<std_msgs::Int32>("speed_selection_reason");
  debug_pubs_.speed_selection_reason_text = advertiseDebug<std_msgs::String>("speed_selection_reason_text");

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
  double cte = nan, heading_error = nan, curvature_ff = nan;
  double preview_curvature_value = nan, preview_distance_value = nan, distance_to_end = -1.0, trajectory_age = -1.0;
  int direction = 0, segment_index_value = trajectory ? 0 : -1, nearest_index_value = -1, target_index_value = -1, preview_index_value = -1;
  int segment_count = 0, steer_saturated = 0;

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    segment_count = trajectory_received_ && !active_trajectory_.empty() ? 1 : 0;
    if (trajectory_received_) {
      trajectory_age = (stamp - last_trajectory_time_).toSec();
    }
  }
  publishPoseDebug(debug_pubs_.rear_axle_pose, pose.x, pose.y, pose.yaw, stamp);
  const double front_x = pose.x + wheelbase_ * std::cos(pose.yaw);
  const double front_y = pose.y + wheelbase_ * std::sin(pose.yaw);
  publishPoseDebug(debug_pubs_.front_axle_pose, front_x, front_y, pose.yaw, stamp);
  publishPoseDebug(debug_pubs_.tracking_pose, tracking_pose.x, tracking_pose.y, tracking_pose.yaw, stamp);

  if (trajectory && !trajectory->empty()) {
    const auto& path = trajectory->points;
    direction = trajectory->direction();
    const std::size_t nearest_rear = std::min(nearest_index_rear, path.size() - 1);
    const std::size_t nearest_tracking = std::min(nearest_index_tracking, path.size() - 1);
    const std::size_t target = std::min(target_index, path.size() - 1);
    const bool has_preview = preview_index != std::numeric_limits<std::size_t>::max();
    const std::size_t preview = has_preview ? std::min(preview_index, path.size() - 1) : target;

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

    publishPoseDebug(debug_pubs_.nearest_pose, nearest_rear_point.x, nearest_rear_point.y, nearest_rear_point.yaw, stamp);
    publishPoseDebug(debug_pubs_.control_pose, nearest_tracking_point.x, nearest_tracking_point.y, nearest_tracking_point.yaw, stamp);
    publishPoseDebug(debug_pubs_.lookahead_pose, target_point.x, target_point.y, target_point.yaw, stamp);
    if (has_preview) {
      publishPoseDebug(debug_pubs_.feedforward_pose, preview_point.x, preview_point.y, preview_point.yaw, stamp);
    }

    const double dx = tracking_pose.x - nearest_tracking_point.x;
    const double dy = tracking_pose.y - nearest_tracking_point.y;
    cte = std::sin(nearest_tracking_point.yaw) * dx - std::cos(nearest_tracking_point.yaw) * dy;
    heading_error = normalizeAngle(nearest_tracking_point.yaw - tracking_pose.yaw);
    const double ff_gain = (direction >= 0) ? forward_stanley_params_.curvature_ff_gain : reverse_stanley_params_.curvature_ff_gain;
    curvature_ff = steering_command_sign_ * steering_ratio_ * computeCurvatureFeedforward(preview_curvature, direction, ff_gain);
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
  debug_pubs_.current_speed.publish(float_msg);
  float_msg.data = target_speed;
  debug_pubs_.target_speed.publish(float_msg);
  float_msg.data = raw_target_speed;
  debug_pubs_.raw_lookahead_speed.publish(float_msg);
  float_msg.data = target_speed - current_speed;
  debug_pubs_.speed_error.publish(float_msg);
  float_msg.data = steer_command;
  debug_pubs_.steer_command.publish(float_msg);
  float_msg.data = curvature_ff;
  debug_pubs_.curvature_feedforward.publish(float_msg);
  float_msg.data = preview_curvature_value;
  debug_pubs_.curvature_preview.publish(float_msg);
  float_msg.data = preview_distance_value;
  debug_pubs_.curvature_preview_distance.publish(float_msg);
  float_msg.data = cte;
  debug_pubs_.cross_track_error.publish(float_msg);
  float_msg.data = heading_error;
  debug_pubs_.heading_error.publish(float_msg);
  float_msg.data = lookahead;
  debug_pubs_.lookahead_distance.publish(float_msg);
  float_msg.data = distance_to_end;
  debug_pubs_.distance_to_segment_end.publish(float_msg);
  float_msg.data = trajectory_age;
  debug_pubs_.trajectory_age.publish(float_msg);

  std_msgs::Int32 int_msg;
  int_msg.data = segment_index_value;
  debug_pubs_.segment_index.publish(int_msg);
  int_msg.data = segment_count;
  debug_pubs_.segment_count.publish(int_msg);
  int_msg.data = nearest_index_value;
  debug_pubs_.nearest_index.publish(int_msg);
  int_msg.data = target_index_value;
  debug_pubs_.target_index.publish(int_msg);
  int_msg.data = preview_index_value;
  debug_pubs_.curvature_preview_index.publish(int_msg);
  int_msg.data = steer_saturated;
  debug_pubs_.steer_saturated.publish(int_msg);
  int_msg.data = direction;
  debug_pubs_.direction.publish(int_msg);
  int_msg.data = tracking_state;
  debug_pubs_.tracking_state.publish(int_msg);
  int_msg.data = stop_reason;
  debug_pubs_.stop_reason.publish(int_msg);
  int_msg.data = speed_selection_reason;
  debug_pubs_.speed_selection_reason.publish(int_msg);

  std_msgs::String string_msg;
  string_msg.data = speedSelectionReasonString(speed_selection_reason);
  debug_pubs_.speed_selection_reason_text.publish(string_msg);
}

void ControlNode::publishStopReason(int stop_reason)
{
  if (!publish_debug_) return;
  std_msgs::Int32 msg;
  msg.data = stop_reason;
  debug_pubs_.stop_reason.publish(msg);
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

  debug_pubs_.active_path.publish(path);
  last_debug_path_time_ = stamp;
}

void ControlNode::publishControl(int direction, double steer_command, double accel_command)
{
  const double gas_norm = clamp(accel_command / std::max(1e-6, max_accel_), 0.0, 1.0);
  const double brake_norm = clamp(-accel_command / std::max(1e-6, max_decel_), 0.0, 1.0);

  last_steer_command_ = clamp(steer_command, -max_steer_command_, max_steer_command_);
  resetIdleRelease();

  carmaker_msgs::Control_Signal msg;
  msg.header.stamp = ros::Time::now();
  msg.steerangle = static_cast<float>(last_steer_command_);
  msg.gas = static_cast<float>(gas_norm * max_gas_);
  msg.brake = static_cast<float>(brake_norm * max_brake_);
  msg.accel = static_cast<float>(direction * (accel_command >= 0.0 ? gas_norm * max_accel_ : -brake_norm * max_decel_));
  msg.gear = directionToGear(direction, drive_gear_, reverse_gear_);
  control_pub_.publish(msg);
}

void ControlNode::publishStop(int direction, double steer_command)
{
  if (direction != 0) {
    last_steer_command_ = clamp(steer_command, -max_steer_command_, max_steer_command_);
  }

  carmaker_msgs::Control_Signal msg;
  msg.header.stamp = ros::Time::now();
  msg.gas = 0.0F;
  msg.steerangle = static_cast<float>(last_steer_command_);
  if (direction == 0) {
    msg.brake = static_cast<float>(clamp(stop_brake_, 0.0, max_brake_));
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
    if (publish_stop_without_path_) {
      publishStop(0);
      return true;
    }
    idle_control_released_ = true;
    ROS_INFO_THROTTLE(1.0, "No active trajectory: released control_signal publisher.");
    return false;
  }

  if (steer_release_after_idle_) {
    const double ratio = release_duration_ <= 1e-6 ? 1.0 : clamp(elapsed / release_duration_, 0.0, 1.0);
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

void ControlNode::reconfigureCallback(carmaker_control::CarmakerControlConfig& config, uint32_t level)
{
  (void)level;

  // Update PID Configuration
  speed_pid_.configure(config.pid_kp, config.pid_ki, config.pid_kd, -max_decel_, max_accel_);

  // Update Stanley Forward Configuration
  forward_stanley_params_.k = config.stanley_forward_k;
  forward_stanley_params_.k_soft = config.stanley_forward_k_soft;
  forward_stanley_params_.cte_gain = config.stanley_forward_cte_gain;
  forward_stanley_params_.heading_gain = config.stanley_forward_heading_gain;
  forward_stanley_params_.curvature_ff_gain = config.stanley_forward_curvature_ff_gain;

  forward_stanley_.configure(forward_stanley_params_.k, forward_stanley_params_.k_soft, max_tire_steer_,
                             forward_stanley_params_.cte_gain, forward_stanley_params_.heading_gain);

  // Update Stanley Reverse Configuration
  reverse_stanley_params_.k = config.stanley_reverse_k;
  reverse_stanley_params_.k_soft = config.stanley_reverse_k_soft;
  reverse_stanley_params_.cte_gain = config.stanley_reverse_cte_gain;
  reverse_stanley_params_.heading_gain = config.stanley_reverse_heading_gain;
  reverse_stanley_params_.curvature_ff_gain = config.stanley_reverse_curvature_ff_gain;

  reverse_stanley_.configure(reverse_stanley_params_.k, reverse_stanley_params_.k_soft, max_tire_steer_,
                             reverse_stanley_params_.cte_gain, reverse_stanley_params_.heading_gain);

  // Update Forward Lookahead parameters
  forward_lookahead_.distance = config.forward_lookahead_distance;
  forward_lookahead_.time = config.forward_lookahead_time;
  forward_lookahead_.min_distance = config.forward_min_lookahead_distance;
  forward_lookahead_.max_distance = config.forward_max_lookahead_distance;
  forward_lookahead_.curvature_preview_distance = config.forward_curvature_preview_distance;
  forward_lookahead_.curvature_preview_window = config.forward_curvature_preview_window;

  // Update Reverse Lookahead parameters
  reverse_lookahead_.distance = config.reverse_lookahead_distance;
  reverse_lookahead_.time = config.reverse_lookahead_time;
  reverse_lookahead_.min_distance = config.reverse_min_lookahead_distance;
  reverse_lookahead_.max_distance = config.reverse_max_lookahead_distance;
  reverse_lookahead_.curvature_preview_distance = config.reverse_curvature_preview_distance;
  reverse_lookahead_.curvature_preview_window = config.reverse_curvature_preview_window;

  // Update Alignment Fade-out Parameter
  alignment_fade_distance_ = config.alignment_fade_distance;

  // Update Steering Lookahead & Curvature Weight parameters
  forward_control_lookahead_ = config.forward_control_lookahead;
  reverse_control_lookahead_ = config.reverse_control_lookahead;
  front_curvature_weight_ = config.front_curvature_weight;

  ROS_INFO("CarmakerControl dynamic parameters updated.");
}

}  // namespace carmaker_control

int main(int argc, char** argv)
{
  ros::init(argc, argv, "carmaker_control_node");
  carmaker_control::ControlNode node;
  ros::spin();
  return 0;
}
