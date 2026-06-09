/**
 * @file local_planner_nodelet.h
 * @brief ROS1 Nodelet for cusp-segment local planning with quintic polynomial paths.
 *
 * Subscribes:
 *   topics/subscribe/trajectory  (carmaker_msgs/TrajectoryPath)   — global path
 *   topics/subscribe/dynamics    (carmaker_msgs/DynamicsInfo)     — GT ego pose
 *   topics/subscribe/odom        (nav_msgs/Odometry)              — EKF ego pose
 *   setting/test_pose/topic      (geometry_msgs/PoseWithCovarianceStamped) — offline fit test pose
 *
 * Publishes:
 *   topics/publish/trajectory          (carmaker_msgs/TrajectoryPath)
 *
 * RULE §1 — SRP Callbacks: callbacks only store raw data under a lock.
 *            All processing is delegated to processNewGlobalPath() and publishActiveTrajectory().
 * RULE §3 — Fine-grained Locks: each resource has its own dedicated mutex.
 *            Lock scope restricted to data copy; heavy operations run outside.
 */
#ifndef CARMAKER_PLANNING_LOCAL_PLANNER_NODELET_H
#define CARMAKER_PLANNING_LOCAL_PLANNER_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <memory>
#include <atomic>

#include "carmaker_planning/types.h"
#include "carmaker_planning/local_planner.h"
#include "carmaker_planning/trajectory_state_machine.h"
#include "carmaker_planning/post_processing.h"  // KinematicLimits, profilePath
#include "carmaker_planning/visualizer.h"
#include <carmaker_msgs/TrajectoryPath.h>
#include <carmaker_msgs/DynamicsInfo.h>

namespace carmaker_planning {

class LocalPlannerNodelet : public nodelet::Nodelet {
public:
  LocalPlannerNodelet() = default;
  ~LocalPlannerNodelet() override = default;
  void onInit() override;

private:
  // ── Callbacks (RULE §1 — store only) ───────────────────────────────────────
  void trajectoryCallback(const carmaker_msgs::TrajectoryPath::ConstPtr& msg);
  void dynamicsCallback(const carmaker_msgs::DynamicsInfoConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void testPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void timerCallback(const ros::TimerEvent&);

  // ── Processing methods ──────────────────────────────────────────────────────
  void   processNewGlobalPath();           ///< Consumes pending_trajectory_ outside lock
  bool   getEgoState(State& ego);
  void   publishActiveTrajectory(const State& ego);
  void   publishTimeoutStop();
  Path   buildPublishPath(const TrajectoryStateMachine::TrajectoryDecision& decision,
                          const State& ego) const;
  double getEgoKappa(const State& ego, const Path& global_path) const;
  void   applyTrajectoryIntent(Path& path,
                               TrajectoryStateMachine::TrajectoryIntent intent) const;
  void   normalizePath(Path& path, int direction) const;
  void   capVelocity(Path& path, double max_velocity) const;

  // ── Visualization ───────────────────────────────────────────────────────────
  void publishDebug(const Path& path);

  // ── Message conversion (static helpers) ─────────────────────────────────────
  static Path fromTrajectoryMsg(const carmaker_msgs::TrajectoryPath& msg);
  static carmaker_msgs::TrajectoryPath toTrajectoryMsg(const Path& path,
                                                        const std::string& frame_id);

  // ── ROS infrastructure ──────────────────────────────────────────────────────
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber trajectory_sub_, dynamics_sub_, odom_sub_, test_pose_sub_;
  ros::Publisher  trajectory_pub_;
  ros::Timer      publish_timer_;

  // ── Configuration ───────────────────────────────────────────────────────────
  LocalPlannerConfig cfg_;
  std::string        global_frame_;
  std::string        rear_axle_frame_;
  bool               use_gt_pose_ = true;
  bool               use_test_pose_ = false;
  std::string        mode_ = "local";
  double             min_turning_radius_ = 5.2;
  double             pose_timeout_ = 0.5;
  double             wheelbase_ = 2.97;
  double             final_approach_max_vel_ = 0.15;
  KinematicLimits    kinematic_limits_;
  TrajectoryStateMachine::Config state_machine_config_;
  std::unique_ptr<Visualizer> visualizer_;

  // ── Shared state (each protected by its own mutex — RULE §3) ────────────────

  // Raw incoming trajectory (store-only in callback; consumed in processNewGlobalPath)
  std::mutex raw_trajectory_mutex_;
  carmaker_msgs::TrajectoryPath::ConstPtr pending_trajectory_;
  bool new_trajectory_flag_ = false;

  // Active trajectory planner
  std::mutex state_machine_mutex_;
  TrajectoryStateMachine trajectory_state_machine_;

  // GT dynamics pose
  std::mutex                  dynamics_mutex_;
  carmaker_msgs::DynamicsInfo latest_dynamics_;
  ros::Time                   last_dynamics_time_;
  std::atomic<bool>           dynamics_received_{false};

  // EKF odometry pose (use_gt_pose_ = false)
  std::mutex         odom_mutex_;
  nav_msgs::Odometry latest_odom_;
  ros::Time         last_odom_time_;
  std::atomic<bool>  odom_received_{false};

  // Offline path-fitting test pose (use_test_pose_ = true)
  std::mutex                test_pose_mutex_;
  geometry_msgs::PoseStamped latest_test_pose_;
  std::atomic<bool>          test_pose_received_{false};
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_LOCAL_PLANNER_NODELET_H
