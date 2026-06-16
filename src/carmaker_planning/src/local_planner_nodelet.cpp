/**
 * @file local_planner_nodelet.cpp
 * @brief ROS1 Nodelet: cusp-segment local planner with quintic polynomial paths.
 *
 * RULE §1 — SRP Callbacks: callbacks only store data under lock; processing delegated
 *            to consumePendingTrajectory() and publishTrackingTrajectory().
 * RULE §3 — Fine-grained Locks: lock scope restricted to data copy only; heavy
 *            operations (conversion, fitting, profiling) run outside the lock.
 * RULE §4 — YAML Config: all parameters loaded via NodeHandle::param().
 */

#include "carmaker_planning/local_planner_nodelet.h"
#include "carmaker_planning/math.h"
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace carmaker_planning {

namespace {

size_t findSegmentEndpointIndex(const Path& path, size_t target_seg_idx) {
  if (path.empty()) return 0;
  size_t current_seg_idx = 0;
  size_t start = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    if (path[i].direction != path[start].direction) {
      if (current_seg_idx == target_seg_idx) {
        return i - 1;
      }
      current_seg_idx++;
      start = i;
    }
  }
  return path.size() - 1;
}

} // namespace

// ── Initialization ────────────────────────────────────────────────────────────

void LocalPlannerNodelet::onInit() {
  nh_  = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  loadLocalPlannerConfig(pnh_, cfg_);

  if (cfg_.min_creep_speed > cfg_.stop_vel_tol) {
    NODELET_WARN("[LocalPlanner] local_planner/velocity/min_creep_speed=%.3f is greater than "
                 "arrival/stop_vel_tol=%.3f. Endpoint creep may prevent the state machine "
                 "from entering stop/finish states until vehicle speed falls below stop_vel_tol.",
                 cfg_.min_creep_speed,
                 cfg_.stop_vel_tol);
  }

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
  pnh_.param("local_planner/fitting_lookahead", fitting_lookahead_, 2.0);

  wheelbase_ = pnh_.param<double>("vehicle/wheelbase", 2.97);
  state_machine_config_.mode = TrajectoryStateMachine::parseMode(mode_);
  state_machine_config_.endpoint_xy_tol = cfg_.endpoint_xy_tol;
  state_machine_config_.endpoint_yaw_tol = cfg_.endpoint_yaw_tol;
  state_machine_config_.segment_transition_xy_tol = cfg_.segment_transition_xy_tol;
  state_machine_config_.segment_transition_yaw_tol = cfg_.segment_transition_yaw_tol;
  state_machine_config_.stop_vel_tol = cfg_.stop_vel_tol;
  state_machine_config_.stop_duration =
      pnh_.param<double>("local_planner/transition/stop_duration", 0.6);
  state_machine_config_.presteer_duration =
      pnh_.param<double>("local_planner/transition/presteer_duration", 0.6);
  trajectory_state_machine_.configure(state_machine_config_);
  min_turning_radius_ = pnh_.param<double>("vehicle/min_turning_radius", 5.2);
  local_trajectory_planner_.configure(cfg_.post_process,
                                      wheelbase_,
                                      min_turning_radius_);

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

  diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(nh_, pnh_);
  diagnostic_updater_->setHardwareID("carmaker_local_planner");
  diagnostic_updater_->add("Planner Status", this, &LocalPlannerNodelet::produceDiagnostics);
  pnh_.param<double>("diagnostic_period", diag_period_, 1.0);
  diag_timer_ = nh_.createWallTimer(ros::WallDuration(diag_period_),
                                    &LocalPlannerNodelet::diagTimerCallback,
                                    this);

  NODELET_INFO("[LocalPlanner] Initialized (%.1f Hz).", cfg_.replanning_rate_hz);
  NODELET_INFO("[LocalPlanner] mode=%s trajectory_in=%s trajectory_out=%s",
               mode_.c_str(), traj_in.c_str(), traj_out.c_str());
  NODELET_DEBUG("[LocalPlanner] limits: max_vel=%.2f max_accel=%.2f max_decel=%.2f "
                "max_jerk=%.2f max_lat_acc=%.2f max_steer_vel=%.2f",
                cfg_.post_process.profiler.max_vel,
                cfg_.post_process.profiler.max_accel,
                cfg_.post_process.profiler.max_decel,
                cfg_.post_process.profiler.max_jerk,
                cfg_.post_process.profiler.max_lat_acc,
                cfg_.post_process.profiler.max_steer_vel);
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
  if (handleTimeJump()) return;

  // Step 1: Consume pending global path (RULE §3 — lock only for flag/pointer swap)
  consumePendingTrajectory();

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
  if (!readEgoState(ego)) {
    publishTimeoutStopTrajectory();
    return;
  }

  // Step 4: Publish active trajectory
  publishTrackingTrajectory(ego);
}

void LocalPlannerNodelet::diagTimerCallback(const ros::WallTimerEvent&) {
  processDiagnostics();
}

// ── Core logic ────────────────────────────────────────────────────────────────

bool LocalPlannerNodelet::handleTimeJump() {
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
    path_locked_ = false;
    locked_path_.clear();
    last_timer_time_ = now;
    NODELET_WARN("[LocalPlanner] Time jump detected. Cleared pending and active trajectory state.");
    return true;
  }

  last_timer_time_ = now;
  last_timer_time_valid_ = true;
  return false;
}

bool LocalPlannerNodelet::consumePendingTrajectory() {
  carmaker_msgs::TrajectoryPath::ConstPtr msg;
  {
    // RULE §3 — minimal lock scope: only swap the pointer
    std::lock_guard<std::mutex> lock(raw_trajectory_mutex_);
    if (!new_trajectory_flag_) return false;
    msg = pending_trajectory_;
    new_trajectory_flag_ = false;
    pending_trajectory_.reset();
  }

  if (!msg) {
    NODELET_WARN_THROTTLE(1.0, "[LocalPlanner] Pending trajectory flag was set without a message.");
    return false;
  }

  // Heavy conversion happens OUTSIDE the lock (RULE §3 — scoped locks)
  Path gp = trajectoryMsgToPath(*msg);

  {
    std::lock_guard<std::mutex> lock(state_machine_mutex_);
    trajectory_state_machine_.setGlobalPath(gp);
    last_decision_log_valid_ = false;
    last_logged_finished_ = false;
  }
  path_locked_ = false;
  locked_path_.clear();

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
  return true;
}

bool LocalPlannerNodelet::readEgoState(State& ego) {
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

void LocalPlannerNodelet::processDiagnostics() {
  if (diagnostic_updater_) {
    diagnostic_updater_->update();
  }
}

void LocalPlannerNodelet::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  LocalPlanningDiagnostic diag_snapshot;
  double observed_publish_rate_hz = 0.0;
  double last_publish_period_sec = -1.0;
  double last_endpoint_distance = -1.0;
  bool last_publish_zero_speed = false;
  TrajectoryStateMachine::PlannerState last_diag_state =
      TrajectoryStateMachine::PlannerState::kIdle;
  TrajectoryStateMachine::TrajectoryIntent last_diag_intent =
      TrajectoryStateMachine::TrajectoryIntent::kNone;
  TrajectoryStateMachine::TrajectorySource last_diag_source =
      TrajectoryStateMachine::TrajectorySource::kNone;
  {
    std::lock_guard<std::mutex> lock(diag_mutex_);
    diag_snapshot = last_diag_;
    observed_publish_rate_hz = observed_publish_rate_hz_;
    last_publish_period_sec = last_publish_period_sec_;
    last_endpoint_distance = last_endpoint_distance_;
    last_publish_zero_speed = last_publish_zero_speed_;
    last_diag_state = last_diag_state_;
    last_diag_intent = last_diag_intent_;
    last_diag_source = last_diag_source_;
  }

  bool has_path = false;
  bool idle = true;
  size_t segment_count = 0;
  size_t active_segment = 0;
  TrajectoryStateMachine::PlannerState state = TrajectoryStateMachine::PlannerState::kIdle;
  {
    std::lock_guard<std::mutex> lock(state_machine_mutex_);
    has_path = trajectory_state_machine_.hasPath();
    idle = trajectory_state_machine_.isIdle();
    segment_count = trajectory_state_machine_.segmentCount();
    active_segment = trajectory_state_machine_.activeSegmentIndex();
    state = trajectory_state_machine_.state();
  }

  if (!has_path) {
    stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Waiting for global trajectory");
  } else if (diag_snapshot.success) {
    stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Publishing trajectory");
  } else if (idle) {
    stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Idle");
  } else {
    stat.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Last local trajectory failed");
  }

  stat.add("Has Global Path", has_path ? "True" : "False");
  stat.add("Mode", mode_);
  stat.add("State", trajectoryStateString(state));
  stat.add("Last Snapshot State", trajectoryStateString(last_diag_state));
  stat.add("Last Snapshot Intent", trajectoryIntentString(last_diag_intent));
  stat.add("Last Snapshot Source", trajectorySourceString(last_diag_source));
  stat.add("Active Segment Index", active_segment);
  stat.add("Segment Count", segment_count);
  stat.add("Total Trajectories", total_trajectories_.load());
  stat.add("Successful Trajectories", successful_trajectories_.load());
  stat.add("Failed Trajectories", failed_trajectories_.load());
  stat.add("Configured Replanning Rate (Hz)", cfg_.replanning_rate_hz);
  stat.add("Observed Planning Publish Rate (Hz)", observed_publish_rate_hz);
  stat.add("Last Planning Publish Period (s)", last_publish_period_sec);
  stat.add("Last Endpoint Distance (m)", last_endpoint_distance);
  stat.add("Last Snapshot Zero Speed", last_publish_zero_speed ? "True" : "False");
  stat.add("Snapshot Success", diag_snapshot.success ? "True" : "False");
  stat.add("Snapshot Path Points", static_cast<int>(diag_snapshot.path_size));
  stat.add("Snapshot Path Length (m)", diag_snapshot.path_length);
  stat.add("Snapshot Total Time (s)", diag_snapshot.total_time);
  stat.add("Snapshot Smoothing Time (s)", diag_snapshot.smoothing_time);
  stat.add("Snapshot Resampling Time (s)", diag_snapshot.resampling_time);
  stat.add("Snapshot Velocity Profiling Time (s)", diag_snapshot.profiling_time);
}

void LocalPlannerNodelet::updateLocalDiagnostics(const LocalPlanningResult& result) {
  total_trajectories_++;
  if (result.success) {
    successful_trajectories_++;
  } else {
    failed_trajectories_++;
  }

  std::lock_guard<std::mutex> lock(diag_mutex_);
  last_diag_ = LocalPlanningDiagnostic(result);
}

void LocalPlannerNodelet::publishTrackingTrajectory(const State& ego) {
  TrajectoryStateMachine::TrajectoryDecision decision;
  {
    std::lock_guard<std::mutex> lock(state_machine_mutex_);
    decision = trajectory_state_machine_.update(ego, ros::Time::now().toSec());
  }

  logDecisionTransition(decision, ego);

  if (!decision.publish) {
    clearDebugTrajectory();
    NODELET_DEBUG_THROTTLE(1.0,
                           "[LocalPlanner] No publish decision: state=%s intent=%s source=%s",
                           trajectoryStateString(decision.state),
                           trajectoryIntentString(decision.intent),
                           trajectorySourceString(decision.path_source));
    return;
  }

  Path path;
  if (!composeTrajectoryForDecision(decision, ego, path)) {
    clearDebugTrajectory();
    NODELET_WARN_THROTTLE(1.0,
                          "[LocalPlanner] Failed to build trajectory: state=%s intent=%s source=%s",
                          trajectoryStateString(decision.state),
                          trajectoryIntentString(decision.intent),
                          trajectorySourceString(decision.path_source));
    return;
  }

  trajectory_pub_.publish(pathToTrajectoryMsg(path, global_frame_));
  publishDebugTrajectory(path);

  const double endpoint_dist = dist(ego.x, ego.y, decision.endpoint.x, decision.endpoint.y);
  {
    std::lock_guard<std::mutex> lock(diag_mutex_);
    const ros::WallTime now_wall = ros::WallTime::now();
    if (!last_publish_wall_time_.isZero()) {
      last_publish_period_sec_ = (now_wall - last_publish_wall_time_).toSec();
      observed_publish_rate_hz_ = last_publish_period_sec_ > 1e-6
                                      ? 1.0 / last_publish_period_sec_
                                      : 0.0;
    }
    last_publish_wall_time_ = now_wall;
    last_endpoint_distance_ = endpoint_dist;
    last_publish_zero_speed_ = isZeroSpeedTrajectory(path);
    last_diag_state_ = decision.state;
    last_diag_intent_ = decision.intent;
    last_diag_source_ = decision.path_source;
  }
  NODELET_DEBUG_THROTTLE(1.0,
                         "[LocalPlanner] Publish: state=%s intent=%s source=%s seg=%zu "
                         "dir=%d pts=%zu len=%.2f m duration=%.2f s "
                         "ego=(%.2f, %.2f, %.2f, v=%.2f) endpoint_dist=%.2f m",
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
                         endpoint_dist);

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
  const bool has_active_endpoint =
      decision.path_source != TrajectoryStateMachine::TrajectorySource::kNone;

  if (segment_changed && has_active_endpoint) {
    double err_xy = 0.0;
    double err_yaw = 0.0;
    double err_v = std::abs(ego.v);
    PathPoint prev_endpoint;
    const auto& gp = trajectory_state_machine_.globalPath();
    if (!gp.empty()) {
      size_t ep_idx = findSegmentEndpointIndex(gp, last_logged_segment_index_);
      prev_endpoint = gp[ep_idx];
      err_xy = dist(ego.x, ego.y, prev_endpoint.x, prev_endpoint.y);
      err_yaw = std::abs(wrap_to_pi(ego.theta - prev_endpoint.theta));
    }
    if (decision.failed) {
      NODELET_WARN("[LocalPlanner] Segment transition %zu -> %zu (ARRIVAL FAILED on transition, precise tolerances not met): state=%s, intent=%s, dir=%d\n"
                   "Ego: (%.2f, %.2f, %.2f) | Completed Endpoint: (%.2f, %.2f, %.2f) | Target Endpoint: (%.2f, %.2f, %.2f)\n"
                   "Tolerances: xy=%.3f, yaw=%.3f deg, vel=%.3f | Errors: xy=%.3f, yaw=%.3f deg, vel=%.3f",
                   last_logged_segment_index_,
                   decision.active_segment_index,
                   trajectoryStateString(decision.state),
                   trajectoryIntentString(decision.intent),
                   decision.active_direction,
                   ego.x,
                   ego.y,
                   ego.theta,
                   prev_endpoint.x,
                   prev_endpoint.y,
                   prev_endpoint.theta,
                   decision.endpoint.x,
                   decision.endpoint.y,
                   decision.endpoint.theta,
                   cfg_.segment_transition_xy_tol,
                   rad2deg(cfg_.segment_transition_yaw_tol),
                   cfg_.stop_vel_tol,
                   err_xy,
                   rad2deg(err_yaw),
                   err_v);
    } else {
      NODELET_INFO("[LocalPlanner] Segment transition: %zu -> %zu, state=%s, intent=%s, dir=%d\n"
                   "Ego: (%.2f, %.2f, %.2f) | Completed Endpoint: (%.2f, %.2f, %.2f) | Target Endpoint: (%.2f, %.2f, %.2f)\n"
                   "Tolerances: xy=%.3f, yaw=%.3f deg, vel=%.3f | Errors: xy=%.3f, yaw=%.3f deg, vel=%.3f",
                   last_logged_segment_index_,
                   decision.active_segment_index,
                   trajectoryStateString(decision.state),
                   trajectoryIntentString(decision.intent),
                   decision.active_direction,
                   ego.x,
                   ego.y,
                   ego.theta,
                   prev_endpoint.x,
                   prev_endpoint.y,
                   prev_endpoint.theta,
                   decision.endpoint.x,
                   decision.endpoint.y,
                   decision.endpoint.theta,
                   cfg_.segment_transition_xy_tol,
                   rad2deg(cfg_.segment_transition_yaw_tol),
                   cfg_.stop_vel_tol,
                   err_xy,
                   rad2deg(err_yaw),
                   err_v);
    }
  } else if (state_changed) {
    NODELET_DEBUG("[LocalPlanner] State transition: state=%s, intent=%s, segment=%zu",
                  trajectoryStateString(decision.state),
                  trajectoryIntentString(decision.intent),
                  decision.active_segment_index);
  }

  if (decision.finished && !last_logged_finished_) {
    double err_xy = 0.0;
    double err_yaw = 0.0;
    double err_v = std::abs(ego.v);
    PathPoint target;
    if (!trajectory_state_machine_.globalPath().empty()) {
      target = trajectory_state_machine_.globalPath().back();
      err_xy = dist(ego.x, ego.y, target.x, target.y);
      err_yaw = std::abs(wrap_to_pi(ego.theta - target.theta));
    }
    if (decision.failed) {
      NODELET_WARN("[LocalPlanner] Final goal reached (ARRIVAL FAILED at final destination, precise tolerances not met): last_segment=%zu, state=%s, intent=%s\n"
                   "Ego: (%.2f, %.2f, %.2f) | Target Endpoint: (%.2f, %.2f, %.2f)\n"
                   "Tolerances: xy=%.3f, yaw=%.3f deg, vel=%.3f | Errors: xy=%.3f, yaw=%.3f deg, vel=%.3f",
                   last_logged_segment_index_,
                   trajectoryStateString(decision.state),
                   trajectoryIntentString(decision.intent),
                   ego.x,
                   ego.y,
                   ego.theta,
                   target.x,
                   target.y,
                   target.theta,
                   cfg_.endpoint_xy_tol, rad2deg(cfg_.endpoint_yaw_tol), cfg_.stop_vel_tol,
                   err_xy, rad2deg(err_yaw), err_v);
    } else {
      NODELET_INFO("[LocalPlanner] Final goal reached (Active path finished): last_segment=%zu, state=%s, intent=%s\n"
                   "Ego: (%.2f, %.2f, %.2f) | Target Endpoint: (%.2f, %.2f, %.2f)\n"
                   "Tolerances: xy=%.3f, yaw=%.3f deg, vel=%.3f | Errors: xy=%.3f, yaw=%.3f deg, vel=%.3f",
                   last_logged_segment_index_,
                   trajectoryStateString(decision.state),
                   trajectoryIntentString(decision.intent),
                   ego.x,
                   ego.y,
                   ego.theta,
                   target.x,
                   target.y,
                   target.theta,
                   cfg_.endpoint_xy_tol, rad2deg(cfg_.endpoint_yaw_tol), cfg_.stop_vel_tol,
                   err_xy, rad2deg(err_yaw), err_v);
    }
  }

  last_decision_log_valid_ = true;
  last_logged_finished_ = decision.finished;
  last_logged_segment_index_ = decision.active_segment_index;
  last_logged_state_ = decision.state;
  last_logged_intent_ = decision.intent;
}

void LocalPlannerNodelet::publishTimeoutStopTrajectory() {
  TrajectoryStateMachine::TrajectoryDecision decision;
  {
    std::lock_guard<std::mutex> lock(state_machine_mutex_);
    decision = trajectory_state_machine_.stopForTimeout(ros::Time::now().toSec());
  }
  if (!decision.publish) return;
  State ego;
  ego.v = 0.0;
  Path path;
  if (!composeTrajectoryForDecision(decision, ego, path)) {
    clearDebugTrajectory();
    NODELET_WARN_THROTTLE(1.0, "[LocalPlanner] Timeout stop failed to build a trajectory.");
    return;
  }
  trajectory_pub_.publish(pathToTrajectoryMsg(path, global_frame_));
  publishDebugTrajectory(path);
  NODELET_WARN_THROTTLE(1.0,
                        "[LocalPlanner] Timeout stop publish: state=%s source=%s pts=%zu",
                        trajectoryStateString(decision.state),
                        trajectorySourceString(decision.path_source),
                        path.size());
}

// ── Debug Visualization ───────────────────────────────────────────────────────

void LocalPlannerNodelet::publishDebugTrajectory(const Path& path) {
  if (!visualizer_) return;
  visualizer_->visualize(path, {}, {}, global_frame_, min_turning_radius_);
  {
    std::lock_guard<std::mutex> lock(diag_mutex_);
    debug_visualization_cleared_ = false;
  }
}

void LocalPlannerNodelet::clearDebugTrajectory() {
  if (!visualizer_) return;

  bool should_clear = false;
  {
    std::lock_guard<std::mutex> lock(diag_mutex_);
    should_clear = !debug_visualization_cleared_;
    debug_visualization_cleared_ = true;
  }

  if (should_clear) {
    visualizer_->clear(global_frame_);
  }
}

bool LocalPlannerNodelet::composeTrajectoryForDecision(
    const TrajectoryStateMachine::TrajectoryDecision& decision,
    const State& ego,
    Path& path) {
  path.clear();

  if (decision.state != TrajectoryStateMachine::PlannerState::kTracking ||
      decision.intent != TrajectoryStateMachine::TrajectoryIntent::kTrack) {
    path_locked_ = false;
    locked_path_.clear();
  }

  if (decision.path_source == TrajectoryStateMachine::TrajectorySource::kActiveSegment) {
    path = decision.active_segment;
    LocalPlanningResult result;
    result.success = !path.empty();
    result.path = path;
    updateLocalDiagnostics(result);
  } else if (decision.path_source == TrajectoryStateMachine::TrajectorySource::kLocalToEndpoint) {
    if (path_locked_ && !locked_path_.empty()) {
      path = locked_path_;
      total_trajectories_++;
      successful_trajectories_++;
    } else {
      double start_kappa = 0.0;
      if (!estimateStartKappa(ego, decision.active_segment, start_kappa)) {
        NODELET_WARN_THROTTLE(1.0, "[LocalPlanner] Cannot estimate ego curvature: active segment is empty.");
        LocalPlanningResult result;
        result.error("LocalPlannerNodelet: cannot estimate start curvature because active segment is empty.");
        updateLocalDiagnostics(result);
        return false;
      }
      LocalTrajectoryPlanner::PlanRequest request;
      request.ego = ego;
      request.start_kappa = start_kappa;
      request.start_vel = ego.v;

      const double dist_to_endpoint = dist(ego.x, ego.y, decision.endpoint.x, decision.endpoint.y);

      if (dist_to_endpoint <= cfg_.trajectory_lock_distance) {
        // 락 구간 내부: 차량의 시작 위치를 뒤로 trajectory_lock_distance 만큼 투영하여 가상의 긴 피팅 구간 확보
        double dx = cfg_.trajectory_lock_distance;
        request.ego.x -= dx * cos(ego.theta) * decision.active_direction;
        request.ego.y -= dx * sin(ego.theta) * decision.active_direction;
        request.target = decision.endpoint;
      } else {
        // 락 구간 외부: 현재 차량 위치(best_idx) 기준 룩어헤드 거리만큼 앞의 지점을 타겟으로 설정
        size_t ego_idx = 0;
        double best_d = std::numeric_limits<double>::max();
        for (size_t i = 0; i < decision.active_segment.size(); ++i) {
          const double d = dist(ego.x, ego.y, decision.active_segment[i].x, decision.active_segment[i].y);
          if (d < best_d) {
            best_d = d;
            ego_idx = i;
          }
        }

        size_t target_idx = ego_idx;
        double accumulated = 0.0;
        while (target_idx + 1 < decision.active_segment.size() && accumulated < fitting_lookahead_) {
          accumulated += dist(decision.active_segment[target_idx], decision.active_segment[target_idx + 1]);
          target_idx++;
        }

        request.target = decision.active_segment[target_idx];
        request.stitching_path.assign(decision.active_segment.begin() + static_cast<std::ptrdiff_t>(target_idx + 1),
                                      decision.active_segment.end());
      }

      LocalPlanningResult result = local_trajectory_planner_.planToEndpoint(request);
      logPlannerMessages("[LocalPlanner]", result.logs);
      updateLocalDiagnostics(result);
      if (!result.success) {
        return false;
      }
      path = std::move(result.path);
    }
  } else {
    return false;
  }

  // 최종 경로 끝단에 오프셋 일괄 연장 적용 (락 구간 피팅 궤적도 자동으로 포함됨)
  if (!path.empty()) {
    double offset = getControlLookaheadOffset(decision.active_direction);
    if (offset > 0.0) {
      appendOffset(path, offset);
    }
  }

  // 락 상태 저장 (연장 처리가 완료된 완성본이 locked_path_에 안전하게 저장됨)
  if (decision.path_source == TrajectoryStateMachine::TrajectorySource::kLocalToEndpoint && !path_locked_) {
    const double dist_to_endpoint = dist(ego.x, ego.y, decision.endpoint.x, decision.endpoint.y);
    if (dist_to_endpoint <= cfg_.trajectory_lock_distance) {
      path_locked_ = true;
      locked_path_ = path;
      NODELET_INFO("[LocalPlanner] Distance to endpoint (%.3fm) <= lock threshold. Trajectory LOCKED & EXTENDED.", dist_to_endpoint);
    }
  }

  applyStopIntentIfNeeded(path, decision.intent);
  if (applyTrackingCreepIfNeeded(path, decision.intent)) {
    recomputeTimingAndAcceleration(path);
  }
  return !path.empty();
}

bool LocalPlannerNodelet::estimateStartKappa(
    const State& ego,
    const Path& active_segment,
    double& start_kappa) const {
  start_kappa = 0.0;
  if (active_segment.empty()) return false;

  size_t best_idx = 0;
  double best_d = std::numeric_limits<double>::max();
  for (size_t i = 0; i < active_segment.size(); ++i) {
    const double d = dist(ego.x, ego.y, active_segment[i].x, active_segment[i].y);
    if (d < best_d) {
      best_d = d;
      best_idx = i;
    }
  }
  start_kappa = active_segment[best_idx].kappa;
  return true;
}

bool LocalPlannerNodelet::isZeroSpeedTrajectory(const Path& path) const {
  return !path.empty() &&
         std::all_of(path.begin(), path.end(), [](const PathPoint& pt) {
           return std::abs(pt.v) <= 1e-6;
         });
}

bool LocalPlannerNodelet::applyTrackingCreepIfNeeded(
    Path& path,
    TrajectoryStateMachine::TrajectoryIntent intent) const {
  if (intent != TrajectoryStateMachine::TrajectoryIntent::kTrack || path.size() < 2) {
    return false;
  }

  const double min_creep_speed = std::max(0.0, cfg_.min_creep_speed);
  const double creep_distance = std::max(0.0, cfg_.creep_distance);
  if (min_creep_speed <= 1e-6 || creep_distance <= 1e-6) {
    return false;
  }

  const double offset = getControlLookaheadOffset(path.back().direction);

  bool changed = false;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const double distance_to_end = std::max(0.0, path.back().s - offset - path[i].s);
    if (distance_to_end <= creep_distance && path[i].s < path.back().s - offset) {
      if (path[i].v < min_creep_speed) {
        path[i].v = min_creep_speed;
        changed = true;
      }
    }
  }

  for (size_t i = 0; i < path.size(); ++i) {
    if (path[i].s >= path.back().s - offset - 1e-3) {
      if (std::abs(path[i].v) > 1e-6 || std::abs(path[i].a) > 1e-6) {
        path[i].v = 0.0;
        path[i].a = 0.0;
        changed = true;
      }
    }
  }
  return changed;
}

void LocalPlannerNodelet::recomputeTimingAndAcceleration(Path& path) const {
  if (path.empty()) {
    return;
  }

  const double min_velocity_denominator =
      std::max(1e-6, cfg_.post_process.profiler.min_velocity_denominator);
  path.front().t = 0.0;
  path.front().a = 0.0;

  for (size_t i = 1; i < path.size(); ++i) {
    double ds = path[i].s - path[i - 1].s;
    if (!std::isfinite(ds) || ds < 0.0) {
      ds = dist(path[i - 1], path[i]);
    }

    const double v_prev = std::max(0.0, std::abs(path[i - 1].v));
    const double v_curr = std::max(0.0, std::abs(path[i].v));
    double dt = 0.0;
    if (v_prev + v_curr > 1e-3) {
      const double denom = std::max(v_prev + v_curr, min_velocity_denominator);
      dt = 2.0 * std::max(0.0, ds) / denom;
    } else {
      dt = 0.0;
    }

    path[i].t = path[i - 1].t + dt;
    path[i].a = dt > 1e-6 ? (path[i].v - path[i - 1].v) / dt : 0.0;
  }
}

double LocalPlannerNodelet::getControlLookaheadOffset(int direction) const {
  if (direction == 1 || direction == -1) {
    return wheelbase_ * 1.5;
  }
  return 0.0;
}

void LocalPlannerNodelet::appendOffset(Path& path, double offset) const {
  if (path.empty() || offset <= 0.0) return;
  double resolution = cfg_.post_process.resampler.resolution;
  if (resolution <= 1e-4) {
    resolution = 0.1;
  }
  const auto& last_pt = path.back();
  const double theta = last_pt.theta;
  const double time_from_start = last_pt.t;
  const double start_s = last_pt.s;

  int num_offset_points = static_cast<int>(std::ceil(offset / resolution));
  path.reserve(path.size() + num_offset_points);

  for (int i = 1; i <= num_offset_points; ++i) {
    double d = std::min(offset, i * resolution);
    PathPoint p;
    p.x = last_pt.x + d * std::cos(theta) * last_pt.direction;
    p.y = last_pt.y + d * std::sin(theta) * last_pt.direction;
    p.theta = theta;
    // 0.0까지 선형적으로 감쇠하도록 처리하여 부드러운 핸들 정렬 유도
    double ratio = (offset <= 1e-6) ? 1.0 : std::max(0.0, std::min(1.0, d / offset));
    p.kappa = last_pt.kappa * (1.0 - ratio);
    p.v = 0.0;
    p.a = 0.0;
    p.direction = last_pt.direction;
    p.s = start_s + d;
    p.t = time_from_start;
    path.push_back(p);
  }
}

void LocalPlannerNodelet::applyStopIntentIfNeeded(
    Path& path,
    TrajectoryStateMachine::TrajectoryIntent intent) const {
  if (intent != TrajectoryStateMachine::TrajectoryIntent::kStopCurrent &&
      intent != TrajectoryStateMachine::TrajectoryIntent::kPresteerNext &&
      intent != TrajectoryStateMachine::TrajectoryIntent::kEmergencyStop) {
    return;
  }

  for (auto& pt : path) {
    pt.v = 0.0;
    pt.a = 0.0;
    pt.t = 0.0;
  }
}

} // namespace carmaker_planning

PLUGINLIB_EXPORT_CLASS(carmaker_planning::LocalPlannerNodelet, nodelet::Nodelet)
