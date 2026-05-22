/**
 * @file global_planner_nodelet.h
 * @brief ROS1 Nodelet for the carmaker_planning global planner.
 */
#ifndef CARMAKER_PLANNING_GLOBAL_PLANNER_NODELET_H
#define CARMAKER_PLANNING_GLOBAL_PLANNER_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <mutex>
#include <atomic>
#include <memory>

#include "carmaker_planning/global_planner.h"
#include "carmaker_planning/global_map.h"
#include "carmaker_planning/visualizer.h"
#include <carmaker_msgs/DynamicsInfo.h>

namespace carmaker_planning {

class GlobalPlannerNodelet : public nodelet::Nodelet {
public:
  GlobalPlannerNodelet() = default;
  virtual ~GlobalPlannerNodelet() = default;

  virtual void onInit() override;

private:
  // ROS Callbacks (strictly for receiving data, delegating computation to separate methods)
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void diagTimerCallback(const ros::WallTimerEvent& event);
  void dynamicsCallback(const carmaker_msgs::DynamicsInfoConstPtr& msg);
  void manualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // Dedicated Processing / Computation Methods
  void processGoal(const geometry_msgs::PoseStamped& goal_msg);
  void processDiagnostics();

  // ROS 서비스를 통해 static OccupancyGrid 맵을 가져와 GlobalMap에 주입한다.
  bool loadMapFromService();

  // Helper methods
  bool getStartState(State& start_state);
  State getGoalState(const geometry_msgs::PoseStamped& msg);
  void publishVisualization(const Path& path);
  void publishTrajectory(const Path& path);

  // ROS Infrastructure
  ros::NodeHandle nh_, pnh_;
  ros::Publisher trajectory_pub_;
  ros::Publisher debug_map_pub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber dynamics_sub_;
  ros::Subscriber manual_pose_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string global_frame_;
  std::string ego_frame_;

  // Diagnostics
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  ros::WallTimer diag_timer_;
  ros::Time last_sim_time_;
  ros::WallTime last_wall_time_;
  double diag_period_ = 1.0;

  // Planner configurations and engines
  GlobalMainConfig config_;
  std::unique_ptr<GlobalPlanner> planner_;
  std::unique_ptr<GlobalMap> global_map_;
  std::unique_ptr<Visualizer> visualizer_;
  
  std::mutex planner_mutex_;
  std::atomic<bool> has_map_{false};

  // Ground Truth Dynamics pose state
  bool use_gt_pose_ = false;
  std::mutex dynamics_mutex_;
  carmaker_msgs::DynamicsInfo latest_dynamics_;
  std::atomic<bool> dynamics_received_{false};

  // Manual start pose state (for offline testing without simulator)
  bool use_manual_pose_ = false;
  std::mutex manual_pose_mutex_;
  geometry_msgs::PoseStamped latest_manual_pose_;
  std::atomic<bool> manual_pose_received_{false};

  // Diagnostic metrics
  std::atomic<uint64_t> total_plans_{0};
  std::atomic<uint64_t> successful_plans_{0};
  std::atomic<uint64_t> failed_plans_{0};
  
  std::mutex diag_mutex_;
  double last_planning_time_{0.0};
  double last_path_length_{0.0};
  std::string last_status_{"NO_PLAN"};
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_GLOBAL_PLANNER_NODELET_H
