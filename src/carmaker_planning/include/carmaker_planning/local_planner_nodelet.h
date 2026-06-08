/**
 * @file local_planner_nodelet.h
 * @brief ROS1 Nodelet for cusp-segment local planning with quintic polynomial paths.
 *
 * Subscribes:
 *   topics/subscribe/trajectory       (carmaker_msgs/TrajectoryPath) — global path
 *   topics/subscribe/dynamics         (carmaker_msgs/DynamicsInfo)   — GT ego pose
 *
 * Publishes:
 *   topics/publish/local_trajectory   (carmaker_msgs/TrajectoryPath) — local path
 *
 * RULE §1 — SRP Callbacks: callbacks only store raw data under a lock.
 *            All processing is delegated to processNewGlobalPath() and replan().
 * RULE §3 — Fine-grained Locks: each resource has its own dedicated mutex.
 *            Lock scope restricted to data copy; heavy operations run outside.
 */
#ifndef CARMAKER_PLANNING_LOCAL_PLANNER_NODELET_H
#define CARMAKER_PLANNING_LOCAL_PLANNER_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <mutex>
#include <memory>
#include <atomic>

#include "carmaker_planning/types.h"
#include "carmaker_planning/local_planner.h"
#include "carmaker_planning/post_processing.h"  // KinematicLimits, profilePath
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
  void manualPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void timerCallback(const ros::TimerEvent&);

  // ── Processing methods ──────────────────────────────────────────────────────
  void   processNewGlobalPath();          ///< Consumes pending_trajectory_ outside lock
  bool   getEgoState(State& ego);
  double getEgoKappa(const State& ego);   ///< Nearest global-path κ to ego (lock-safe)
  void   replan(const State& ego);

  // ── Message conversion (static helpers) ─────────────────────────────────────
  static Path fromTrajectoryMsg(const carmaker_msgs::TrajectoryPath& msg);
  static carmaker_msgs::TrajectoryPath toTrajectoryMsg(const Path& path,
                                                        const std::string& frame_id);

  // ── ROS infrastructure ──────────────────────────────────────────────────────
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber trajectory_sub_, dynamics_sub_, manual_pose_sub_;
  ros::Publisher  local_trajectory_pub_;
  ros::Timer      replan_timer_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── Configuration ───────────────────────────────────────────────────────────
  LocalPlannerConfig cfg_;
  std::string        global_frame_;
  std::string        rear_axle_frame_;
  bool               use_gt_pose_     = false;
  bool               use_manual_pose_ = false;
  double             wheelbase_       = 2.97;
  KinematicLimits    kinematic_limits_;

  // ── Shared state (each protected by its own mutex — RULE §3) ────────────────

  // Raw incoming trajectory (store-only in callback; consumed in processNewGlobalPath)
  std::mutex raw_trajectory_mutex_;
  carmaker_msgs::TrajectoryPath::ConstPtr pending_trajectory_;
  bool new_trajectory_flag_ = false;

  // Segment manager
  std::mutex     segment_mutex_;
  SegmentManager segment_manager_;

  // GT dynamics pose
  std::mutex                  dynamics_mutex_;
  carmaker_msgs::DynamicsInfo latest_dynamics_;
  std::atomic<bool>           dynamics_received_{false};

  // Manual pose (for offline testing)
  std::mutex                manual_pose_mutex_;
  geometry_msgs::PoseStamped latest_manual_pose_;
  std::atomic<bool>          manual_pose_received_{false};
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_LOCAL_PLANNER_NODELET_H
