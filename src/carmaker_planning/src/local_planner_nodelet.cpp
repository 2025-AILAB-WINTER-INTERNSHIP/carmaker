/**
 * @file local_planner_nodelet.cpp
 * @brief ROS1 Nodelet: cusp-segment local planner with quintic polynomial paths.
 *
 * RULE §1 — SRP Callbacks: callbacks only store data under lock; processing delegated
 *            to processNewGlobalPath() and replan().
 * RULE §3 — Fine-grained Locks: lock scope restricted to data copy only; heavy
 *            operations (conversion, fitting, profiling) run outside the lock.
 * RULE §4 — YAML Config: all parameters loaded via NodeHandle::param().
 */

#include "carmaker_planning/local_planner_nodelet.h"
#include "carmaker_planning/math.h"
#include <pluginlib/class_list_macros.h>
#include <carmaker_msgs/TrajectoryPoint.h>
#include <cmath>
#include <limits>

namespace carmaker_planning {

// ── Initialization ────────────────────────────────────────────────────────────

void LocalPlannerNodelet::onInit() {
  nh_  = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  loadLocalPlannerConfig(pnh_, cfg_);

  // Frame / pose-source (mirrors GlobalPlannerNodelet for consistency)
  pnh_.param<std::string>("frames/global",    global_frame_,    "Fr0");
  pnh_.param<std::string>("frames/rear_axle", rear_axle_frame_, "Fr1A_Rear_Axle_Pred");
  pnh_.param<bool>("setting/use_gt_pose",             use_gt_pose_,    false);
  pnh_.param<bool>("setting/use_manual_pose/enabled", use_manual_pose_, false);

  wheelbase_ = pnh_.param<double>("vehicle/wheelbase", 2.97);

  // KinematicLimits assembled from local_planner config
  kinematic_limits_.max_vel       = cfg_.max_vel;
  kinematic_limits_.max_accel     = cfg_.max_accel;
  kinematic_limits_.max_decel     = cfg_.max_decel;
  kinematic_limits_.max_jerk      = cfg_.max_jerk;
  kinematic_limits_.max_steer_vel = pnh_.param<double>("vehicle/limits/max_steer_vel", 0.6108);
  kinematic_limits_.max_lat_acc   = cfg_.max_lat_acc;
  kinematic_limits_.min_vel_denom = pnh_.param<double>(
      "profiler/min_velocity_denominator", 0.02);

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Topics (RULE §4 — no hardcoding)
  const std::string traj_in  = pnh_.param<std::string>(
      "topics/subscribe/trajectory",      "/planning/trajectory");
  const std::string traj_out = pnh_.param<std::string>(
      "topics/publish/local_trajectory",  "/planning/local_trajectory");

  // RULE §1 SRP: trajectory_sub_ callback only stores raw message pointer.
  trajectory_sub_       = nh_.subscribe(traj_in, 1,
                              &LocalPlannerNodelet::trajectoryCallback, this);
  local_trajectory_pub_ = nh_.advertise<carmaker_msgs::TrajectoryPath>(traj_out, 1, true);

  if (use_gt_pose_) {
    const std::string dyn = pnh_.param<std::string>(
        "topics/subscribe/dynamics", "/carmaker/dynamic_info");
    dynamics_sub_ = nh_.subscribe(dyn, 10,
                       &LocalPlannerNodelet::dynamicsCallback, this);
    NODELET_INFO("LocalPlanner pose source: GT dynamics (%s)", dyn.c_str());
  } else if (use_manual_pose_) {
    const std::string mp = pnh_.param<std::string>(
        "setting/use_manual_pose/topic", "/planning/start");
    manual_pose_sub_ = nh_.subscribe(mp, 1,
                          &LocalPlannerNodelet::manualPoseCallback, this);
    NODELET_INFO("LocalPlanner pose source: manual pose (%s)", mp.c_str());
  } else {
    NODELET_INFO("LocalPlanner pose source: TF (%s → %s)",
                 global_frame_.c_str(), rear_axle_frame_.c_str());
  }

  const double period = 1.0 / std::max(cfg_.replanning_rate_hz, 1.0);
  replan_timer_ = nh_.createTimer(ros::Duration(period),
                                  &LocalPlannerNodelet::timerCallback, this);

  NODELET_INFO("LocalPlannerNodelet initialized (%.1f Hz).", cfg_.replanning_rate_hz);
}

// ── Callbacks (RULE §1 — store only, no processing) ──────────────────────────

void LocalPlannerNodelet::trajectoryCallback(
    const carmaker_msgs::TrajectoryPath::ConstPtr& msg) {
  if (!msg || msg->points.empty()) {
    NODELET_WARN_THROTTLE(1.0, "Empty global trajectory received — ignoring.");
    return;
  }
  // Store raw message; heavy conversion runs in processNewGlobalPath() below.
  {
    std::lock_guard<std::mutex> lock(raw_trajectory_mutex_);
    pending_trajectory_ = msg;
    new_trajectory_flag_ = true;
  }
}

void LocalPlannerNodelet::dynamicsCallback(
    const carmaker_msgs::DynamicsInfoConstPtr& msg) {
  if (!msg) return;
  std::lock_guard<std::mutex> lock(dynamics_mutex_);
  latest_dynamics_   = *msg;
  dynamics_received_ = true;
}

void LocalPlannerNodelet::manualPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  if (!msg) return;
  std::lock_guard<std::mutex> lock(manual_pose_mutex_);
  latest_manual_pose_.header = msg->header;
  latest_manual_pose_.pose   = msg->pose.pose;
  manual_pose_received_      = true;
}

// ── Timer: main replanning loop ───────────────────────────────────────────────

void LocalPlannerNodelet::timerCallback(const ros::TimerEvent&) {
  // Step 1: Consume pending global path (RULE §3 — lock only for flag/pointer swap)
  processNewGlobalPath();

  // Step 2: Gate check
  {
    std::lock_guard<std::mutex> lock(segment_mutex_);
    if (!segment_manager_.hasPath() || segment_manager_.isFinished()) return;
  }

  // Step 3: Get ego state
  State ego;
  if (!getEgoState(ego)) return;

  // Step 4: Replan
  replan(ego);
}

// ── Core logic ────────────────────────────────────────────────────────────────

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
    std::lock_guard<std::mutex> lock(segment_mutex_);
    segment_manager_.setGlobalPath(gp);
  }

  NODELET_INFO("Global path processed: %zu points, %zu segment(s).",
               gp.size(), segment_manager_.segmentCount());
}

bool LocalPlannerNodelet::getEgoState(State& ego) {
  if (use_gt_pose_) {
    if (!dynamics_received_) {
      NODELET_WARN_THROTTLE(2.0, "GT dynamics not yet received.");
      return false;
    }
    // RULE §3 — copy data quickly, exit lock before returning
    std::lock_guard<std::mutex> lock(dynamics_mutex_);
    ego.x     = latest_dynamics_.RearAxle_x;
    ego.y     = latest_dynamics_.RearAxle_y;
    ego.theta = latest_dynamics_.Car_Yaw;
    ego.v     = latest_dynamics_.Car_vx;
  } else if (use_manual_pose_) {
    if (!manual_pose_received_) {
      NODELET_WARN_THROTTLE(2.0, "Manual pose not yet received.");
      return false;
    }
    std::lock_guard<std::mutex> lock(manual_pose_mutex_);
    ego.x     = latest_manual_pose_.pose.position.x;
    ego.y     = latest_manual_pose_.pose.position.y;
    ego.theta = quaternionToYaw(latest_manual_pose_.pose.orientation);
    ego.v     = 0.0;
  } else {
    try {
      auto tf = tf_buffer_->lookupTransform(global_frame_, rear_axle_frame_, ros::Time(0));
      ego.x     = tf.transform.translation.x;
      ego.y     = tf.transform.translation.y;
      ego.theta = quaternionToYaw(tf.transform.rotation);
      ego.v     = 0.0;
    } catch (const tf2::TransformException& ex) {
      NODELET_WARN_THROTTLE(2.0, "TF lookup failed: %s", ex.what());
      return false;
    }
  }
  return true;
}

double LocalPlannerNodelet::getEgoKappa(const State& ego) {
  // Copy global path under a scoped lock (RULE §3), then search outside.
  Path gp_copy;
  {
    std::lock_guard<std::mutex> lock(segment_mutex_);
    gp_copy = segment_manager_.globalPath();
  }
  if (gp_copy.empty()) return 0.0;

  size_t best_idx = 0;
  double best_d   = std::numeric_limits<double>::max();
  for (size_t i = 0; i < gp_copy.size(); ++i) {
    const double d = dist(ego.x, ego.y, gp_copy[i].x, gp_copy[i].y);
    if (d < best_d) { best_d = d; best_idx = i; }
  }
  return gp_copy[best_idx].kappa;
}

void LocalPlannerNodelet::replan(const State& ego) {
  // Arrival check + endpoint copy (minimal lock scope)
  PathPoint endpoint;
  {
    std::lock_guard<std::mutex> lock(segment_mutex_);
    const bool finished = segment_manager_.updateArrival(
        ego, cfg_.arrival_xy_tol, cfg_.arrival_yaw_tol, cfg_.arrival_vel_tol);
    if (finished) {
      NODELET_INFO_ONCE("LocalPlanner: final goal reached.");
      return;
    }
    const PathPoint* ep = segment_manager_.activeEndpoint();
    if (!ep) return;
    endpoint = *ep;  // copy before releasing lock
  }

  // Path fitting and profiling — all outside the lock (RULE §3)
  const double ego_kappa = getEgoKappa(ego);
  QuinticPathFitter fitter;
  Path local_path = fitter.fit(ego, ego_kappa, endpoint, cfg_.sample_resolution);

  if (local_path.empty()) {
    NODELET_WARN_THROTTLE(1.0, "QuinticPathFitter: degenerate path (ego too close to endpoint).");
    return;
  }

  // Accumulate arc-length s (used by velocity profiler)
  local_path[0].s = 0.0;
  for (size_t i = 1; i < local_path.size(); ++i)
    local_path[i].s = local_path[i-1].s + dist(local_path[i-1], local_path[i]);

  profilePath(local_path, kinematic_limits_, wheelbase_, std::abs(ego.v));

  local_trajectory_pub_.publish(toTrajectoryMsg(local_path, global_frame_));
}

// ── Message Conversion (static helpers) ──────────────────────────────────────

Path LocalPlannerNodelet::fromTrajectoryMsg(const carmaker_msgs::TrajectoryPath& msg) {
  Path path;
  path.reserve(msg.points.size());
  for (const auto& tp : msg.points) {
    PathPoint pt;
    pt.x         = tp.pose.position.x;
    pt.y         = tp.pose.position.y;
    pt.theta     = quaternionToYaw(tp.pose.orientation);
    pt.kappa     = tp.curvature;
    pt.t         = tp.time_from_start.toSec();
    const double sv = tp.longitudinal_velocity;
    pt.direction = (sv >= 0.0) ? 1 : -1;
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
    tp.time_from_start           = ros::Duration(pt.t);
    msg.points.push_back(tp);
  }
  return msg;
}

} // namespace carmaker_planning

PLUGINLIB_EXPORT_CLASS(carmaker_planning::LocalPlannerNodelet, nodelet::Nodelet)
