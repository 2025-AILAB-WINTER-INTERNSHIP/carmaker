/**
 * @file global_planner_nodelet.cpp
 * @brief ROS1 Nodelet implementation for the carmaker_planning global planner.
 */

#include "carmaker_planning/global_planner_nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <carmaker_msgs/TrajectoryPath.h>
#include <carmaker_msgs/TrajectoryPoint.h>

namespace carmaker_planning {

void GlobalPlannerNodelet::onInit() {
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  // Load configs using private nodehandle
  loadGlobalMainConfig(pnh_, config_);

  // Load frame ID parameters
  pnh_.param<std::string>("frames/global", global_frame_, "Fr0");
  pnh_.param<std::string>("frames/ego", ego_frame_, "Fr1A_pred");
  pnh_.param<bool>("setting/use_gt_pose", use_gt_pose_, false);
  pnh_.param<bool>("setting/use_manual_pose/enabled", use_manual_pose_, false);

  // Initialize time trackers for diagnostics
  last_sim_time_ = ros::Time(0);
  last_wall_time_ = ros::WallTime::now();

  // Initialize planner, map, and visualizer
  planner_ = std::make_unique<GlobalPlanner>(config_, "global_planner_nodelet");
  global_map_ = std::make_unique<GlobalMap>(config_);
  visualizer_ = std::make_unique<Visualizer>(nh_);

  // TF2 setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Debug occupancy grid publisher
  std::string debug_map_topic = pnh_.param("topics/publish/debug/map", std::string("/planning/debug/occupancy_grid"));
  debug_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(debug_map_topic, 1, true);

  // Trajectory publisher
  std::string trajectory_topic = pnh_.param("topics/publish/trajectory", std::string("/planning/trajectory"));
  trajectory_pub_ = nh_.advertise<carmaker_msgs::TrajectoryPath>(trajectory_topic, 1, true);

  // Retrieve static grid map (OccupancyGrid) via ROS service
  if (!loadMapFromService()) {
    NODELET_FATAL("Failed to load map from service!");
    return;
  }

  std::string dynamics_topic = pnh_.param("topics/subscribe/dynamics", std::string("/carmaker/dynamic_info"));
  std::string goal_topic = pnh_.param("topics/subscribe/goal", std::string("/planning/goal"));

  if (use_gt_pose_) {
    dynamics_sub_ = nh_.subscribe(dynamics_topic, 10, &GlobalPlannerNodelet::dynamicsCallback, this);
    NODELET_INFO("Ego Pose Source: Ground Truth (%s) active", dynamics_topic.c_str());
  } else if (use_manual_pose_) {
    std::string manual_topic = pnh_.param("setting/use_manual_pose/topic", std::string("/planning/start"));
    manual_pose_sub_ = nh_.subscribe(manual_topic, 1, &GlobalPlannerNodelet::manualPoseCallback, this);
    NODELET_INFO("Ego Pose Source: Manual Pose Topic (%s) active - simulation-free test mode", manual_topic.c_str());
  } else {
    NODELET_INFO("Ego Pose Source: TF Lookup (global: %s -> ego: %s) active", global_frame_.c_str(), ego_frame_.c_str());
  }
  goal_sub_ = nh_.subscribe(goal_topic, 1, &GlobalPlannerNodelet::goalCallback, this);

  // Diagnostics Setup
  diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(nh_, pnh_);
  diagnostic_updater_->setHardwareID("carmaker_global_planner");
  diagnostic_updater_->add("Planner Status", this, &GlobalPlannerNodelet::produceDiagnostics);

  pnh_.param<double>("diagnostic_period", diag_period_, 1.0);
  diag_timer_ = nh_.createWallTimer(ros::WallDuration(diag_period_), &GlobalPlannerNodelet::diagTimerCallback, this);

  NODELET_INFO("GlobalPlannerNodelet initialized.");
}



bool GlobalPlannerNodelet::getStartState(State& start_state) {
  if (use_gt_pose_) {
    if (!dynamics_received_) {
      NODELET_WARN("GT DynamicsInfo not received yet. Cannot plan path.");
      return false;
    }
    carmaker_msgs::DynamicsInfo dyn;
    {
      std::lock_guard<std::mutex> lock(dynamics_mutex_);
      dyn = latest_dynamics_;
    }
    // Translate start pose from rear bumper forward to rear axle center
    start_state.x = dyn.Car_x + config_.vehicle.rear_axle_offset * std::cos(dyn.Car_Yaw);
    start_state.y = dyn.Car_y + config_.vehicle.rear_axle_offset * std::sin(dyn.Car_Yaw);
    start_state.theta = dyn.Car_Yaw;
  }
  else if (use_manual_pose_) {
    if (!manual_pose_received_) {
      NODELET_WARN("Manual start pose not received yet. Cannot plan path.");
      return false;
    }
    geometry_msgs::PoseStamped pose;
    {
      std::lock_guard<std::mutex> lock(manual_pose_mutex_);
      pose = latest_manual_pose_;
    }
    double qx = pose.pose.orientation.x;
    double qy = pose.pose.orientation.y;
    double qz = pose.pose.orientation.z;
    double qw = pose.pose.orientation.w;
    double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    // Translate from rear bumper to rear axle center
    start_state.x = pose.pose.position.x + config_.vehicle.rear_axle_offset * std::cos(yaw);
    start_state.y = pose.pose.position.y + config_.vehicle.rear_axle_offset * std::sin(yaw);
    start_state.theta = yaw;
  }
  else {
    try {
      // Look up latest transform from global to ego frame
      geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(global_frame_, ego_frame_, ros::Time(0));
      double tx = transform.transform.translation.x;
      double ty = transform.transform.translation.y;

      // Extract yaw from quaternion
      double qx_rot = transform.transform.rotation.x;
      double gq_y_rot = transform.transform.rotation.y;
      double qz_rot = transform.transform.rotation.z;
      double qw_rot = transform.transform.rotation.w;
      double yaw = std::atan2(2.0 * (qw_rot * qz_rot + qx_rot * gq_y_rot), 1.0 - 2.0 * (gq_y_rot * gq_y_rot + qz_rot * qz_rot));

      // Translate start pose from rear bumper (Fr1A_pred/Fr1A) forward to rear axle center
      start_state.x = tx + config_.vehicle.rear_axle_offset * std::cos(yaw);
      start_state.y = ty + config_.vehicle.rear_axle_offset * std::sin(yaw);
      start_state.theta = yaw;
    }
    catch (tf2::TransformException &ex) {
      NODELET_WARN("Could not look up ego pose from TF: %s. Using origin (0, 0, 0) as start.", ex.what());
      start_state.x = 0.0;
      start_state.y = 0.0;
      start_state.theta = 0.0;
    }
  }
  return true;
}

State GlobalPlannerNodelet::getGoalState(const geometry_msgs::PoseStamped& msg) {
  State goal_state;
  double gqx = msg.pose.orientation.x;
  double gqy = msg.pose.orientation.y;
  double gqz = msg.pose.orientation.z;
  double gqw = msg.pose.orientation.w;
  double goal_yaw = std::atan2(2.0 * (gqw * gqz + gqx * gqy), 1.0 - 2.0 * (gqy * gqy + gqz * gqz));

  goal_state.x = msg.pose.position.x + config_.vehicle.rear_axle_offset * std::cos(goal_yaw);
  goal_state.y = msg.pose.position.y + config_.vehicle.rear_axle_offset * std::sin(goal_yaw);
  goal_state.theta = goal_yaw;
  return goal_state;
}

void GlobalPlannerNodelet::publishVisualization(const Path& path) {
  visualizer_->visualize(path, planner_->getSearchTree(), planner_->getTreeBranches(),
                         global_frame_, config_.vehicle.rear_axle_offset, config_.vehicle.min_turning_radius);
}

void GlobalPlannerNodelet::publishTrajectory(const Path& path) {
  carmaker_msgs::TrajectoryPath traj_msg;
  traj_msg.header.stamp = ros::Time::now();
  traj_msg.header.frame_id = global_frame_;
  traj_msg.points.reserve(path.size());

  for (const auto& pt : path) {
    carmaker_msgs::TrajectoryPoint tp;

    // Translate planned path point (rear axle center) backward to rear bumper
    double bx = pt.x - config_.vehicle.rear_axle_offset * std::cos(pt.theta);
    double by = pt.y - config_.vehicle.rear_axle_offset * std::sin(pt.theta);

    tp.pose.position.x = bx;
    tp.pose.position.y = by;
    tp.pose.position.z = 0.0;

    double half_theta = pt.theta * 0.5;
    tp.pose.orientation.x = 0.0;
    tp.pose.orientation.y = 0.0;
    tp.pose.orientation.z = std::sin(half_theta);
    tp.pose.orientation.w = std::cos(half_theta);

    tp.longitudinal_velocity = pt.v * pt.direction;
    tp.longitudinal_acceleration = pt.a * pt.direction;
    tp.curvature = pt.kappa;
    tp.time_from_start = ros::Duration(pt.t);

    traj_msg.points.push_back(tp);
  }
  trajectory_pub_.publish(traj_msg);
}

void GlobalPlannerNodelet::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // 1. Check incoming goal message pointer validity
  if (!msg) {
    NODELET_WARN_THROTTLE(1.0, "Received null goal message pointer.");
    return;
  }

  NODELET_INFO("Goal pose received: (%.2f, %.2f, %.2f deg)",
               msg->pose.position.x, msg->pose.position.y,
               quaternionToYaw(msg->pose.orientation) * 180.0 / M_PI);

  // 2. Verify that map data is loaded and ready
  if (!has_map_) {
    NODELET_WARN_THROTTLE(1.0, "Map not received yet, dropping goal request.");
    return;
  }

  // 3. Verify dynamics readiness if Ground Truth pose source is active
  if (use_gt_pose_ && !dynamics_received_) {
    NODELET_WARN_THROTTLE(1.0, "Ego pose (GT dynamics) not received yet, dropping goal request.");
    return;
  }

  // 4. Verify manual pose readiness if manual pose mode is active
  if (use_manual_pose_ && !manual_pose_received_) {
    NODELET_WARN_THROTTLE(1.0, "Manual start pose (/planning/start) not received yet, dropping goal request.");
    return;
  }

  processGoal(*msg);
}

void GlobalPlannerNodelet::processGoal(const geometry_msgs::PoseStamped& goal_msg) {
  if (!has_map_) {
    NODELET_WARN("Map not received yet, cannot plan path.");
    return;
  }

  State start_state;
  if (!getStartState(start_state)) {
    return;
  }

  State goal_state = getGoalState(goal_msg);

  NODELET_INFO("Plan requested from (%.2f, %.2f, %.2f rad) to (%.2f, %.2f, %.2f rad)",
               start_state.x, start_state.y, start_state.theta,
               goal_state.x, goal_state.y, goal_state.theta);

  GlobalPlanningResult result;
  {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    result = planner_->plan(start_state, goal_state, *global_map_, 0.0);
  }

  total_plans_++;
  {
    GlobalPlanningDiagnostic diag;
    diag.status = result.status;
    diag.total_time = result.total_time;
    diag.path_length = result.path.empty() ? 0.0 : result.path.back().s;
    diag.planning_time = result.planning_time;
    diag.smoothing_time = result.smoothing_time;
    diag.resampling_time = result.resampling_time;
    diag.profiling_time = result.profiling_time;
    diag.expanded_nodes = result.expanded_nodes;
    diag.search_iterations = result.search_iterations;

    std::lock_guard<std::mutex> d_lock(diag_mutex_);
    last_diag_ = diag;
  }

  // Log bubbled diagnostic messages from core planner & post-processor via Nodelet logger
  for (const auto& log : result.logs) {
    if (log.first == "WARN") {
      NODELET_WARN("%s", log.second.c_str());
    } else if (log.first == "INFO") {
      NODELET_INFO("%s", log.second.c_str());
    } else if (log.first == "ERROR") {
      NODELET_ERROR("%s", log.second.c_str());
    } else if (log.first == "DEBUG") {
      NODELET_DEBUG("%s", log.second.c_str());
    }
  }

  if (result.success()) {
    successful_plans_++;

    double len = result.path.empty() ? 0.0 : result.path.back().s;

    NODELET_INFO("Plan success! Planning: %.3f s, Smoothing: %.3f s, Total: %.3f s, Path Length: %.2f m",
                 result.planning_time, result.smoothing_time, result.total_time, len);

    // Publish path and search tree
    publishVisualization(result.path);

    // Publish TrajectoryPath msg
    publishTrajectory(result.path);
  }
  else {
    failed_plans_++;
    NODELET_ERROR("Planning failed: %s", result.statusString());
  }
}

void GlobalPlannerNodelet::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  GlobalPlanningDiagnostic last_res;
  {
    std::lock_guard<std::mutex> d_lock(diag_mutex_);
    last_res = last_diag_;
  }

  std::string status = "NO_PLAN";
  if (last_res.status != PlanningStatus::INVALID) {
    status = last_res.statusString();
  }

  double p_time = last_res.total_time;
  double p_len = last_res.path_length;

  if (status == "SUCCESS_OPTIMAL" || status == "SUCCESS_BEST_EFFORT") {
    stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Planning Active");
  } else if (status == "NO_PLAN") {
    stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Idle, waiting for plan request");
  } else {
    stat.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Last planning failed: " + status);
  }

  stat.add("Has Map", has_map_.load() ? "True" : "False");
  stat.add("Total Plans Requested", total_plans_.load());
  stat.add("Successful Plans", successful_plans_.load());
  stat.add("Failed Plans", failed_plans_.load());
  stat.add("Last Planning Time (s)", p_time);
  stat.add("Last Path Length (m)", p_len);
  stat.add("Last Status", status);
  stat.add("A* Search Time (s)", last_res.planning_time);
  stat.add("Smoothing Time (s)", last_res.smoothing_time);
  stat.add("Resampling Time (s)", last_res.resampling_time);
  stat.add("Velocity Profiling Time (s)", last_res.profiling_time);
  stat.add("Expanded Nodes", last_res.expanded_nodes);
  stat.add("Search Iterations", last_res.search_iterations);
}

void GlobalPlannerNodelet::diagTimerCallback(const ros::WallTimerEvent&) {
  processDiagnostics();
}

void GlobalPlannerNodelet::processDiagnostics() {
  ros::Time now_sim = ros::Time::now();
  ros::WallTime now_wall = ros::WallTime::now();

  if (!now_sim.isZero() && !last_sim_time_.isZero()) {
    double sim_diff = (now_sim - last_sim_time_).toSec();
    double wall_diff = (now_wall - last_wall_time_).toSec();

    // Reset statistics and clear TF buffer if a time jump is detected
    if (sim_diff < -1.0 || (wall_diff > 0.0 && sim_diff > 20.0 * wall_diff)) {
      NODELET_WARN("[Time Jump Detected in Timer] Sim Diff: %.2f sec, Wall Diff: %.2f sec. Resetting planner statistics and clearing TF buffer...", sim_diff, wall_diff);
      total_plans_ = 0;
      successful_plans_ = 0;
      failed_plans_ = 0;
      {
        std::lock_guard<std::mutex> d_lock(diag_mutex_);
        last_diag_ = GlobalPlanningDiagnostic();
      }
      if (tf_buffer_) {
        tf_buffer_->clear();
      }
      diagnostic_updater_->force_update();
    }
  }
  last_sim_time_ = now_sim;
  last_wall_time_ = now_wall;

  if (diagnostic_updater_) {
    diagnostic_updater_->update();
  }
}

void GlobalPlannerNodelet::dynamicsCallback(const carmaker_msgs::DynamicsInfoConstPtr& msg) {
  // 1. Check incoming dynamics message pointer validity
  if (!msg) {
    NODELET_WARN_THROTTLE(1.0, "Received null dynamics message pointer.");
    return;
  }
  // 2. Safely copy data and update initialization flag under mutex lock
  std::lock_guard<std::mutex> lock(dynamics_mutex_);
  latest_dynamics_ = *msg;
  dynamics_received_ = true;
}

void GlobalPlannerNodelet::manualPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  if (!msg) {
    NODELET_WARN_THROTTLE(1.0, "Received null manual pose message pointer.");
    return;
  }
  std::lock_guard<std::mutex> lock(manual_pose_mutex_);
  latest_manual_pose_.header = msg->header;
  latest_manual_pose_.pose = msg->pose.pose;
  manual_pose_received_ = true;
  NODELET_INFO("Manual start pose received: (%.2f, %.2f, %.2f deg)",
               msg->pose.pose.position.x, msg->pose.pose.position.y,
               quaternionToYaw(msg->pose.pose.orientation) * 180.0 / M_PI);
}

// ── Receive static map via ROS service ────────────────────────────────────────

bool GlobalPlannerNodelet::loadMapFromService() {
  ros::NodeHandle& pnh = getPrivateNodeHandle();
  std::string service_name = pnh.param("services/get_map", std::string("/localization/get_map"));

  ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetMap>(service_name);

  NODELET_INFO("[GlobalPlannerNodelet] Waiting for map service: %s ...", service_name.c_str());
  if (!client.waitForExistence(ros::Duration(10.0))) {
    NODELET_ERROR("[GlobalPlannerNodelet] Map service '%s' is not available after 10 seconds.", service_name.c_str());
    return false;
  }

  nav_msgs::GetMap srv;
  if (!client.call(srv)) {
    NODELET_ERROR("[GlobalPlannerNodelet] Failed to call map service: %s", service_name.c_str());
    return false;
  }

  nav_msgs::OccupancyGrid grid = srv.response.map;
  if (grid.data.empty()) {
    NODELET_ERROR("[GlobalPlannerNodelet] Received empty map data from service.");
    return false;
  }

  const int w = grid.info.width;
  const int h = grid.info.height;
  const double res = grid.info.resolution;
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;

  // Grid 데이터를 사용하여 planner map 업데이트
  {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    global_map_->updateMap(w, h, res, origin_x, origin_y, grid.data);
    has_map_ = true;
  }

  // Debug OccupancyGrid 퍼블리시 (frame_id와 timestamp 갱신하여 송신)
  grid.header.frame_id = global_frame_;
  grid.header.stamp = ros::Time::now();
  debug_map_pub_.publish(grid);

  NODELET_INFO("[GlobalPlannerNodelet] OccupancyGrid received and configured via ROS service: %dx%d cells, resolution=%.3fm, origin=(%.2f, %.2f)",
               w, h, res, origin_x, origin_y);
  return true;
}

} // namespace carmaker_planning

PLUGINLIB_EXPORT_CLASS(carmaker_planning::GlobalPlannerNodelet, nodelet::Nodelet)
