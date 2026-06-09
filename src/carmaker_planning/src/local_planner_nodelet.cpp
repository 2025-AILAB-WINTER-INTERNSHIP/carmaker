/**
 * @file local_planner_nodelet.cpp
 * @brief ROS1 Nodelet: cusp-segment local planner with quintic polynomial paths.
 *
 * RULE §1 — SRP Callbacks: callbacks only store data under lock; processing delegated
 *            to processNewGlobalPath() and publishActiveTrajectory().
 * RULE §3 — Fine-grained Locks: lock scope restricted to data copy only; heavy
 *            operations (conversion, fitting, profiling) run outside the lock.
 * RULE §4 — YAML Config: all parameters loaded via NodeHandle::param().
 */

#include "carmaker_planning/local_planner_nodelet.h"
#include "carmaker_planning/math.h"
#include <pluginlib/class_list_macros.h>
#include <carmaker_msgs/TrajectoryPoint.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace carmaker_planning {

// ── Initialization ────────────────────────────────────────────────────────────

void LocalPlannerNodelet::onInit() {
  nh_  = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  loadLocalPlannerConfig(pnh_, cfg_);

  pnh_.param<std::string>("frames/global",    global_frame_,    "Fr0");
  pnh_.param<std::string>("frames/rear_axle", rear_axle_frame_, "Fr1A_Rear_Axle_Pred");
  pnh_.param<bool>("setting/use_gt_pose", use_gt_pose_, true);
  pnh_.param<bool>("setting/test_pose/enabled", use_test_pose_, false);
  pnh_.param<std::string>("local_planner/mode", mode_, "local");
  if (mode_ != "local" && mode_ != "global") {
    NODELET_WARN("[LocalPlanner] Unknown local_planner/mode='%s'. Falling back to 'local'.", mode_.c_str());
    mode_ = "local";
  }

  pose_timeout_ = std::max(0.0, pnh_.param<double>("local_planner/pose_timeout", 0.5));

  final_approach_max_vel_ = std::max(0.0, pnh_.param<double>("local_planner/final_approach/max_vel", 0.15));
  state_machine_config_.mode = TrajectoryStateMachine::parseMode(mode_);
  state_machine_config_.endpoint_xy_tol = cfg_.endpoint_xy_tol;
  state_machine_config_.endpoint_yaw_tol = cfg_.endpoint_yaw_tol;
  state_machine_config_.stop_vel_tol = cfg_.stop_vel_tol;
  state_machine_config_.precision_zone_distance = cfg_.precision_zone_distance;
  state_machine_config_.stop_duration =
      pnh_.param<double>("local_planner/transition/stop_duration", 0.6);
  state_machine_config_.presteer_duration =
      pnh_.param<double>("local_planner/transition/presteer_duration", 0.6);
  state_machine_config_.idle_after_finish_duration =
      pnh_.param<double>("local_planner/transition/idle_after_finish_duration", 0.5);
  trajectory_state_machine_.configure(state_machine_config_);

  // KinematicLimits assembled from local_planner config
  wheelbase_ = pnh_.param<double>("vehicle/wheelbase", 2.97);
  min_turning_radius_ = pnh_.param<double>("vehicle/min_turning_radius", 5.2);

  kinematic_limits_.max_vel       = cfg_.max_vel;
  kinematic_limits_.max_accel     = cfg_.max_accel;
  kinematic_limits_.max_decel     = cfg_.max_decel;
  kinematic_limits_.max_jerk      = cfg_.max_jerk;
  kinematic_limits_.max_steer_vel = pnh_.param<double>("vehicle/limits/max_steer_vel", 0.6108);
  kinematic_limits_.max_lat_acc   = cfg_.max_lat_acc;
  kinematic_limits_.min_vel_denom = pnh_.param<double>(
    "local_planner/profiler/min_velocity_denominator", 0.02);

  // Topics (RULE §4 — no hardcoding)
  const std::string traj_in  = pnh_.param<std::string>(
      "topics/subscribe/trajectory",     "/planning/global/trajectory");
  const std::string traj_out = pnh_.param<std::string>(
      "topics/publish/trajectory", "/planning/trajectory");

  // RULE §1 SRP: trajectory_sub_ callback only stores raw message pointer.
  trajectory_sub_       = nh_.subscribe(traj_in, 1,
                              &LocalPlannerNodelet::trajectoryCallback, this);
  trajectory_pub_ = nh_.advertise<carmaker_msgs::TrajectoryPath>(traj_out, 1, true);
  visualizer_ = std::make_unique<Visualizer>(nh_, pnh_, false);

  if (use_test_pose_) {
    const std::string topic = pnh_.param<std::string>(
        "setting/test_pose/topic", "/planning/start");
    test_pose_sub_ = nh_.subscribe(topic, 1,
        &LocalPlannerNodelet::testPoseCallback, this);
    NODELET_INFO("[LocalPlanner] Pose source: offline test pose (%s)", topic.c_str());
  } else if (use_gt_pose_) {
    const std::string dyn = pnh_.param<std::string>(
        "topics/subscribe/dynamics", "/carmaker/dynamic_info");
    dynamics_sub_ = nh_.subscribe(dyn, 10,
                       &LocalPlannerNodelet::dynamicsCallback, this);
    NODELET_INFO("[LocalPlanner] Pose source: GT dynamics (%s)", dyn.c_str());
  } else {
    const std::string odom = pnh_.param<std::string>(
        "topics/subscribe/odom", "/localization/odom");
    odom_sub_ = nh_.subscribe(odom, 10,
                    &LocalPlannerNodelet::odomCallback, this);
    NODELET_INFO("[LocalPlanner] Pose source: EKF odom (%s)", odom.c_str());
  }

  const double period = 1.0 / std::max(cfg_.replanning_rate_hz, 1.0);
  publish_timer_ = nh_.createTimer(ros::Duration(period), &LocalPlannerNodelet::timerCallback, this);

  NODELET_INFO("[LocalPlanner] Initialized (%.1f Hz).", cfg_.replanning_rate_hz);
  NODELET_INFO("[LocalPlanner] mode=%s trajectory_in=%s trajectory_out=%s",
               mode_.c_str(), traj_in.c_str(), traj_out.c_str());
  NODELET_DEBUG("[LocalPlanner] limits: max_vel=%.2f max_accel=%.2f max_decel=%.2f "
                "max_jerk=%.2f max_lat_acc=%.2f max_steer_vel=%.2f",
                kinematic_limits_.max_vel,
                kinematic_limits_.max_accel,
                kinematic_limits_.max_decel,
                kinematic_limits_.max_jerk,
                kinematic_limits_.max_lat_acc,
                kinematic_limits_.max_steer_vel);
}

// ── Callbacks (RULE §1 — store only, no processing) ──────────────────────────

void LocalPlannerNodelet::trajectoryCallback(
    const carmaker_msgs::TrajectoryPath::ConstPtr& msg) {
  if (!msg || msg->points.empty()) {
    NODELET_WARN_THROTTLE(1.0, "[LocalPlanner] Empty global trajectory received; ignoring.");
    return;
  }
  std::lock_guard<std::mutex> lock(raw_trajectory_mutex_);
  pending_trajectory_  = msg;
  new_trajectory_flag_ = true;
}

void LocalPlannerNodelet::dynamicsCallback(
    const carmaker_msgs::DynamicsInfoConstPtr& msg) {
  if (!msg) return;
  std::lock_guard<std::mutex> lock(dynamics_mutex_);
  latest_dynamics_   = *msg;
  last_dynamics_time_ = ros::Time::now();
  dynamics_received_ = true;
}

void LocalPlannerNodelet::odomCallback(
    const nav_msgs::Odometry::ConstPtr& msg) {
  if (!msg) return;
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_    = *msg;
  last_odom_time_ = ros::Time::now();
  odom_received_  = true;
}

void LocalPlannerNodelet::testPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  if (!msg) return;
  {
    std::lock_guard<std::mutex> lock(test_pose_mutex_);
    latest_test_pose_.header = msg->header;
    latest_test_pose_.pose   = msg->pose.pose;
    test_pose_received_      = true;
  }
  NODELET_INFO("[LocalPlanner] Offline test pose received: (%.2f, %.2f, %.2f deg)",
               msg->pose.pose.position.x,
               msg->pose.pose.position.y,
               quaternionToYaw(msg->pose.pose.orientation) * 180.0 / M_PI);
}

// ── Timer: main replanning loop ───────────────────────────────────────────────

void LocalPlannerNodelet::timerCallback(const ros::TimerEvent&) {
  if (resetOnTimeJump()) return;

  // Step 1: Consume pending global path (RULE §3 — lock only for flag/pointer swap)
  processNewGlobalPath();

  // Step 2: Gate check
  {
    std::lock_guard<std::mutex> lock(state_machine_mutex_);
    if (!trajectory_state_machine_.hasPath()) {
      NODELET_DEBUG_THROTTLE(2.0, "[LocalPlanner] Waiting for global trajectory.");
      return;
    }
    if (trajectory_state_machine_.isIdle()) {
      NODELET_DEBUG_THROTTLE(2.0, "[LocalPlanner] Idle after completing active path.");
      return;
    }
  }

  // Step 3: Get ego state
  State ego;
  if (!getEgoState(ego)) {
    publishTimeoutStop();
    return;
  }

  // Step 4: Publish active trajectory
  publishActiveTrajectory(ego);
}

// ── Core logic ────────────────────────────────────────────────────────────────

bool LocalPlannerNodelet::resetOnTimeJump() {
  const ros::Time now = ros::Time::now();
  if (now.isZero()) return false;

  if (last_timer_time_valid_ && now < last_timer_time_) {
    {
      std::lock_guard<std::mutex> lock(raw_trajectory_mutex_);
      pending_trajectory_.reset();
      new_trajectory_flag_ = false;
    }
    {
      std::lock_guard<std::mutex> lock(state_machine_mutex_);
      trajectory_state_machine_.setGlobalPath(Path{});
      last_decision_log_valid_ = false;
      last_logged_finished_ = false;
    }
    last_timer_time_ = now;
    NODELET_WARN("[LocalPlanner] Time jump detected. Cleared pending and active trajectory state.");
    return true;
  }

  last_timer_time_ = now;
  last_timer_time_valid_ = true;
  return false;
}

void LocalPlannerNodelet::processNewGlobalPath() {
  carmaker_msgs::TrajectoryPath::ConstPtr msg;
  {
    // RULE §3 — minimal lock scope: only swap the pointer
    std::lock_guard<std::mutex> lock(raw_trajectory_mutex_);
    if (!new_trajectory_flag_) return;
    msg = pending_trajectory_;
    new_trajectory_flag_ = false;
    pending_trajectory_.reset();
  }

  // Heavy conversion happens OUTSIDE the lock (RULE §3 — scoped locks)
  Path gp = fromTrajectoryMsg(*msg);

  {
    std::lock_guard<std::mutex> lock(state_machine_mutex_);
    trajectory_state_machine_.setGlobalPath(gp);
    last_decision_log_valid_ = false;
    last_logged_finished_ = false;
  }

  size_t segment_count = 0;
  {
    std::lock_guard<std::mutex> lock(state_machine_mutex_);
    segment_count = trajectory_state_machine_.segmentCount();
  }
  if (!gp.empty()) {
    NODELET_INFO("[LocalPlanner] Global trajectory received: %zu points, %zu segment(s), length=%.2f m, "
                 "start=(%.2f, %.2f, %.2f rad), end=(%.2f, %.2f, %.2f rad)",
                 gp.size(), segment_count, gp.back().s,
                 gp.front().x, gp.front().y, gp.front().theta,
                 gp.back().x, gp.back().y, gp.back().theta);
  }
}

bool LocalPlannerNodelet::getEgoState(State& ego) {
  if (use_test_pose_) {
    if (!test_pose_received_) {
      NODELET_WARN_THROTTLE(2.0, "[LocalPlanner] Offline test pose not yet received.");
      return false;
    }
    std::lock_guard<std::mutex> lock(test_pose_mutex_);
    ego.x     = latest_test_pose_.pose.position.x;
    ego.y     = latest_test_pose_.pose.position.y;
    ego.theta = quaternionToYaw(latest_test_pose_.pose.orientation);
    ego.v     = 0.0;
  } else if (use_gt_pose_) {
    if (!dynamics_received_) {
      NODELET_WARN_THROTTLE(2.0, "[LocalPlanner] GT dynamics not yet received.");
      return false;
    }
    std::lock_guard<std::mutex> lock(dynamics_mutex_);
    if (pose_timeout_ > 0.0 && (ros::Time::now() - last_dynamics_time_).toSec() > pose_timeout_) {
      NODELET_WARN_THROTTLE(1.0, "[LocalPlanner] GT dynamics timeout.");
      return false;
    }
    ego.x     = latest_dynamics_.RearAxle_x;
    ego.y     = latest_dynamics_.RearAxle_y;
    ego.theta = latest_dynamics_.Car_Yaw;
    ego.v     = latest_dynamics_.Car_vx;
  } else {
    if (!odom_received_) {
      NODELET_WARN_THROTTLE(2.0, "[LocalPlanner] EKF odom not yet received.");
      return false;
    }
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (pose_timeout_ > 0.0 && (ros::Time::now() - last_odom_time_).toSec() > pose_timeout_) {
      NODELET_WARN_THROTTLE(1.0, "[LocalPlanner] EKF odom timeout.");
      return false;
    }
    ego.x     = latest_odom_.pose.pose.position.x;
    ego.y     = latest_odom_.pose.pose.position.y;
    ego.theta = quaternionToYaw(latest_odom_.pose.pose.orientation);
    ego.v     = latest_odom_.twist.twist.linear.x;
  }
  return true;
}

void LocalPlannerNodelet::publishActiveTrajectory(const State& ego) {
  TrajectoryStateMachine::TrajectoryDecision decision;
  {
    std::lock_guard<std::mutex> lock(state_machine_mutex_);
    decision = trajectory_state_machine_.update(ego, ros::Time::now().toSec());
  }

  logDecisionTransition(decision, ego);

  if (!decision.publish) {
    NODELET_DEBUG_THROTTLE(1.0,
                           "[LocalPlanner] No publish decision: state=%s intent=%s source=%s",
                           trajectoryStateString(decision.state),
                           trajectoryIntentString(decision.intent),
                           trajectorySourceString(decision.path_source));
    return;
  }

  Path path = buildPublishPath(decision, ego);
  if (path.empty()) {
    NODELET_WARN_THROTTLE(1.0,
                          "[LocalPlanner] Built an empty trajectory: state=%s intent=%s source=%s",
                          trajectoryStateString(decision.state),
                          trajectoryIntentString(decision.intent),
                          trajectorySourceString(decision.path_source));
    return;
  }

  trajectory_pub_.publish(toTrajectoryMsg(path, global_frame_));
  publishDebug(path);

  const double endpoint_dist = dist(ego.x, ego.y, decision.endpoint.x, decision.endpoint.y);
  NODELET_DEBUG_THROTTLE(1.0,
                         "[LocalPlanner] Publish: state=%s intent=%s source=%s seg=%zu "
                         "dir=%d pts=%zu len=%.2f m duration=%.2f s "
                         "ego=(%.2f, %.2f, %.2f, v=%.2f) endpoint_dist=%.2f m final_cap=%s",
                         trajectoryStateString(decision.state),
                         trajectoryIntentString(decision.intent),
                         trajectorySourceString(decision.path_source),
                         decision.active_segment_index,
                         decision.active_direction,
                         path.size(),
                         path.back().s,
                         path.back().t,
                         ego.x,
                         ego.y,
                         ego.theta,
                         ego.v,
                         endpoint_dist,
                         decision.use_final_approach_speed_cap ? "true" : "false");

}

void LocalPlannerNodelet::logDecisionTransition(
    const TrajectoryStateMachine::TrajectoryDecision& decision,
    const State& ego) {
  const bool first_log = !last_decision_log_valid_;
  const bool segment_changed =
      last_decision_log_valid_ &&
      decision.active_segment_index != last_logged_segment_index_;
  const bool state_changed =
      first_log || decision.state != last_logged_state_ ||
      decision.intent != last_logged_intent_;

  if (segment_changed) {
    NODELET_INFO("[LocalPlanner] Segment transition: %zu -> %zu, state=%s, intent=%s, "
                 "dir=%d, ego=(%.2f, %.2f, %.2f), endpoint=(%.2f, %.2f, %.2f)",
                 last_logged_segment_index_,
                 decision.active_segment_index,
                 trajectoryStateString(decision.state),
                 trajectoryIntentString(decision.intent),
                 decision.active_direction,
                 ego.x,
                 ego.y,
                 ego.theta,
                 decision.endpoint.x,
                 decision.endpoint.y,
                 decision.endpoint.theta);
  } else if (state_changed) {
    NODELET_DEBUG("[LocalPlanner] State transition: state=%s, intent=%s, segment=%zu",
                  trajectoryStateString(decision.state),
                  trajectoryIntentString(decision.intent),
                  decision.active_segment_index);
  }

  if (decision.finished && !last_logged_finished_) {
    NODELET_INFO("[LocalPlanner] Final goal reached.");
  }

  last_decision_log_valid_ = true;
  last_logged_finished_ = decision.finished;
  last_logged_segment_index_ = decision.active_segment_index;
  last_logged_state_ = decision.state;
  last_logged_intent_ = decision.intent;
}

void LocalPlannerNodelet::publishTimeoutStop() {
  TrajectoryStateMachine::TrajectoryDecision decision;
  {
    std::lock_guard<std::mutex> lock(state_machine_mutex_);
    decision = trajectory_state_machine_.stopForTimeout(ros::Time::now().toSec());
  }
  if (!decision.publish) return;
  State ego;
  ego.v = 0.0;
  Path path = buildPublishPath(decision, ego);
  if (path.empty()) {
    NODELET_WARN_THROTTLE(1.0, "[LocalPlanner] Timeout stop produced an empty trajectory.");
    return;
  }
  trajectory_pub_.publish(toTrajectoryMsg(path, global_frame_));
  publishDebug(path);
  NODELET_WARN_THROTTLE(1.0,
                        "[LocalPlanner] Timeout stop publish: state=%s source=%s pts=%zu",
                        trajectoryStateString(decision.state),
                        trajectorySourceString(decision.path_source),
                        path.size());
}

// ── Debug Visualization ───────────────────────────────────────────────────────

void LocalPlannerNodelet::publishDebug(const Path& path) {
  if (!visualizer_) return;
  visualizer_->visualize(path, {}, {}, global_frame_, min_turning_radius_);
}

Path LocalPlannerNodelet::buildPublishPath(const TrajectoryStateMachine::TrajectoryDecision& decision,
                                           const State& ego) const {
  Path path;
  if (decision.path_source == TrajectoryStateMachine::TrajectorySource::kActiveSegment) {
    path = decision.active_segment;
  } else if (decision.path_source == TrajectoryStateMachine::TrajectorySource::kLocalToEndpoint) {
    QuinticPathFitter fitter;
    path = fitter.fit(ego,
                      getEgoKappa(ego, decision.global_path),
                      decision.endpoint,
                      cfg_.sample_resolution);
    if (path.empty()) return {};

    normalizePath(path, decision.endpoint.direction);
    KinematicLimits limits = kinematic_limits_;
    if (decision.use_final_approach_speed_cap) {
      limits.max_vel = std::min(limits.max_vel, final_approach_max_vel_);
    }
    profilePath(path, limits, wheelbase_, std::abs(ego.v));
  } else {
    return {};
  }

  normalizePath(path, decision.active_direction);
  if (decision.use_final_approach_speed_cap) {
    capVelocity(path, final_approach_max_vel_);
  }
  applyTrajectoryIntent(path, decision.intent);
  return path;
}

double LocalPlannerNodelet::getEgoKappa(const State& ego, const Path& global_path) const {
  if (global_path.empty()) return 0.0;

  size_t best_idx = 0;
  double best_d = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_path.size(); ++i) {
    const double d = dist(ego.x, ego.y, global_path[i].x, global_path[i].y);
    if (d < best_d) {
      best_d = d;
      best_idx = i;
    }
  }
  return global_path[best_idx].kappa;
}

void LocalPlannerNodelet::applyTrajectoryIntent(
    Path& path,
    TrajectoryStateMachine::TrajectoryIntent intent) const {
  if (intent != TrajectoryStateMachine::TrajectoryIntent::kStopCurrent &&
      intent != TrajectoryStateMachine::TrajectoryIntent::kPresteerNext &&
      intent != TrajectoryStateMachine::TrajectoryIntent::kEmergencyStop &&
      intent != TrajectoryStateMachine::TrajectoryIntent::kFinishedHold) {
    return;
  }

  for (auto& pt : path) {
    pt.v = 0.0;
    pt.a = 0.0;
    pt.t = 0.0;
  }
}

void LocalPlannerNodelet::normalizePath(Path& path, int direction) const {
  if (path.empty()) return;
  direction = direction < 0 ? -1 : 1;
  path[0].s = 0.0;
  path[0].direction = direction;
  for (size_t i = 1; i < path.size(); ++i) {
    path[i].s = path[i - 1].s + dist(path[i - 1], path[i]);
    path[i].direction = direction;
  }
}

void LocalPlannerNodelet::capVelocity(Path& path, double max_velocity) const {
  const double cap = std::max(0.0, max_velocity);
  for (auto& pt : path) {
    pt.v = std::min(std::max(0.0, pt.v), cap);
    if (std::abs(pt.a) > cap && cap > 0.0) {
      pt.a = std::copysign(cap, pt.a);
    }
  }
}

// ── Message Conversion (static helpers) ──────────────────────────────────────

Path LocalPlannerNodelet::fromTrajectoryMsg(const carmaker_msgs::TrajectoryPath& msg) {
  Path path;
  path.reserve(msg.points.size());
  std::vector<int> raw_direction(msg.points.size(), 0);
  for (size_t i = 0; i < msg.points.size(); ++i) {
    const double sv = msg.points[i].longitudinal_velocity;
    if (sv > 1e-3) {
      raw_direction[i] = 1;
    } else if (sv < -1e-3) {
      raw_direction[i] = -1;
    }
  }

  std::vector<int> next_non_zero(msg.points.size(), 0);
  int upcoming = 0;
  for (size_t i = msg.points.size(); i-- > 0;) {
    if (raw_direction[i] != 0) upcoming = raw_direction[i];
    next_non_zero[i] = upcoming;
  }

  int active_direction =
      next_non_zero.empty() || next_non_zero.front() == 0 ? 1 : next_non_zero.front();
  for (size_t i = 0; i < msg.points.size(); ++i) {
    const auto& tp = msg.points[i];
    if (tp.direction != 0) {
      active_direction = tp.direction < 0 ? -1 : 1;
    } else if (raw_direction[i] != 0) {
      active_direction = raw_direction[i];
    }
    PathPoint pt;
    pt.x         = tp.pose.position.x;
    pt.y         = tp.pose.position.y;
    pt.theta     = quaternionToYaw(tp.pose.orientation);
    pt.kappa     = tp.curvature;
    pt.t         = tp.time_from_start.toSec();
    const double sv = tp.longitudinal_velocity;
    pt.direction = active_direction;
    pt.v         = std::abs(sv);
    pt.a         = tp.longitudinal_acceleration * pt.direction;
    path.push_back(pt);
  }
  if (!path.empty()) {
    path[0].s = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
      path[i].s = path[i-1].s + dist(path[i-1], path[i]);
  }
  return path;
}

carmaker_msgs::TrajectoryPath LocalPlannerNodelet::toTrajectoryMsg(
    const Path& path, const std::string& frame_id) {
  carmaker_msgs::TrajectoryPath msg;
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = frame_id;
  msg.points.reserve(path.size());
  for (const auto& pt : path) {
    carmaker_msgs::TrajectoryPoint tp;
    tp.pose.position.x    = pt.x;
    tp.pose.position.y    = pt.y;
    tp.pose.position.z    = 0.0;
    const double hy       = pt.theta * 0.5;
    tp.pose.orientation.z = std::sin(hy);
    tp.pose.orientation.w = std::cos(hy);
    tp.longitudinal_velocity     = pt.v * pt.direction;
    tp.longitudinal_acceleration = pt.a * pt.direction;
    tp.curvature                 = pt.kappa;
    tp.direction                 = pt.direction < 0 ? -1 : 1;
    tp.time_from_start           = ros::Duration(pt.t);
    msg.points.push_back(tp);
  }
  return msg;
}

} // namespace carmaker_planning

PLUGINLIB_EXPORT_CLASS(carmaker_planning::LocalPlannerNodelet, nodelet::Nodelet)
