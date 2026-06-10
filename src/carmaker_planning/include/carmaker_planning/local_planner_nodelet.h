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
 *            All processing is delegated to consumePendingTrajectory() and publishTrackingTrajectory().
 * RULE §3 — Fine-grained Locks: each resource has its own dedicated mutex.
 *            Lock scope restricted to data copy; heavy operations run outside.
 */
#ifndef CARMAKER_PLANNING_LOCAL_PLANNER_NODELET_H
#define CARMAKER_PLANNING_LOCAL_PLANNER_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <mutex>
#include <memory>
#include <atomic>

#include "carmaker_planning/types.h"
#include "carmaker_planning/local_planner.h"
#include "carmaker_planning/ros_utils.h"
#include "carmaker_planning/trajectory_state_machine.h"
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
  void diagTimerCallback(const ros::WallTimerEvent&);
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // ── Processing methods ──────────────────────────────────────────────────────
  bool   consumePendingTrajectory();       ///< Consumes pending_trajectory_ outside lock
  bool   handleTimeJump();
  bool   readEgoState(State& ego);
  void   publishTrackingTrajectory(const State& ego);
  void   publishTimeoutStopTrajectory();
  void   processDiagnostics();
  void   updateLocalDiagnostics(const LocalPlanningResult& result);
  void   logDecisionTransition(const TrajectoryStateMachine::TrajectoryDecision& decision,
                               const State& ego);
  bool   composeTrajectoryForDecision(const TrajectoryStateMachine::TrajectoryDecision& decision,
                                      const State& ego,
                                      Path& path);
  bool   estimateStartKappa(const State& ego,
                            const Path& global_path,
                            double& start_kappa) const;
  void   applyStopIntentIfNeeded(Path& path,
                                 TrajectoryStateMachine::TrajectoryIntent intent) const;
  bool   applyTrackingCreepIfNeeded(Path& path,
                                    TrajectoryStateMachine::TrajectoryIntent intent) const;
  void   recomputeTimingAndAcceleration(Path& path) const;
  bool   isZeroSpeedTrajectory(const Path& path) const;

  // ── Visualization ───────────────────────────────────────────────────────────
  void publishDebugTrajectory(const Path& path);
  void clearDebugTrajectory();

  // ── ROS infrastructure ──────────────────────────────────────────────────────
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber trajectory_sub_, dynamics_sub_, odom_sub_, test_pose_sub_;
  ros::Publisher  trajectory_pub_;
  ros::Timer      publish_timer_;
  ros::WallTimer  diag_timer_;
  ros::Time       last_timer_time_;
  bool            last_timer_time_valid_ = false;

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
  double             diag_period_ = 1.0;
  LocalTrajectoryPlanner local_trajectory_planner_;
  TrajectoryStateMachine::Config state_machine_config_;
  std::unique_ptr<Visualizer> visualizer_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;

  // ── Shared state (each protected by its own mutex — RULE §3) ────────────────

  // Raw incoming trajectory (store-only in callback; consumed in consumePendingTrajectory)
  std::mutex raw_trajectory_mutex_;
  carmaker_msgs::TrajectoryPath::ConstPtr pending_trajectory_;
  bool new_trajectory_flag_ = false;

  // Active trajectory planner
  std::mutex state_machine_mutex_;
  TrajectoryStateMachine trajectory_state_machine_;
  bool last_decision_log_valid_ = false;
  bool last_logged_finished_ = false;
  size_t last_logged_segment_index_ = 0;
  TrajectoryStateMachine::PlannerState last_logged_state_ =
      TrajectoryStateMachine::PlannerState::kIdle;
  TrajectoryStateMachine::TrajectoryIntent last_logged_intent_ =
      TrajectoryStateMachine::TrajectoryIntent::kNone;

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

  // Diagnostics
  std::atomic<uint64_t> total_trajectories_{0};
  std::atomic<uint64_t> successful_trajectories_{0};
  std::atomic<uint64_t> failed_trajectories_{0};
  std::mutex diag_mutex_;
  LocalPlanningDiagnostic last_diag_;
  double observed_publish_rate_hz_ = 0.0;
  double last_publish_period_sec_ = -1.0;
  ros::WallTime last_publish_wall_time_;
  double last_endpoint_distance_ = -1.0;
  bool last_publish_zero_speed_ = false;
  bool debug_visualization_cleared_ = false;
  TrajectoryStateMachine::PlannerState last_diag_state_ =
      TrajectoryStateMachine::PlannerState::kIdle;
  TrajectoryStateMachine::TrajectoryIntent last_diag_intent_ =
      TrajectoryStateMachine::TrajectoryIntent::kNone;
  TrajectoryStateMachine::TrajectorySource last_diag_source_ =
      TrajectoryStateMachine::TrajectorySource::kNone;
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_LOCAL_PLANNER_NODELET_H
