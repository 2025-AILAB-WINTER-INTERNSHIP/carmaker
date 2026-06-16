#ifndef CARMAKER_CONTROL_CONTROL_NODE_H
#define CARMAKER_CONTROL_CONTROL_NODE_H

#include <atomic>
#include <cstddef>
#include <limits>
#include <mutex>
#include <string>
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

#include "carmaker_control/pid.h"
#include "carmaker_control/stanley.h"

#include <dynamic_reconfigure/server.h>
#include <carmaker_control/CarmakerControlConfig.h>

namespace carmaker_control {

enum SpeedSelectionReason {
  kSpeedReasonAllZeroStop = 0,
  kSpeedReasonLookaheadSpeed = 1,
  kSpeedReasonMinTrackingSpeed = 2,
  kSpeedReasonMinCreepSpeed = 3,
  kSpeedReasonEndpointStop = 4
};

class ControlNode {
public:
  ControlNode();

private:
  struct Pose2D {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct VehicleState {
    Pose2D pose;
    double signed_speed{0.0};
  };

  struct PathPoint {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double target_speed{0.0};
    double curvature{0.0};
    int direction{1};
    double s{0.0};
  };

  struct ActiveTrajectory {
    std::vector<PathPoint> points;

    bool empty() const { return points.empty(); }
    std::size_t size() const { return points.size(); }
    int direction() const {
      if (points.empty()) return 1;
      return points.front().direction < 0 ? -1 : 1;
    }
  };

  struct TargetSpeedDecision {
    double target_speed{0.0};
    double raw_target_speed{0.0};
    int reason{kSpeedReasonEndpointStop};
  };

  struct LookaheadParams {
    double distance{0.35};
    double time{0.2};
    double min_distance{0.2};
    double max_distance{0.8};
    double curvature_preview_distance{0.6};
    double curvature_preview_window{0.4};
  };

  struct StanleyParams {
    double k{3.0};
    double k_soft{0.3};
    double cte_gain{2.0};
    double heading_gain{1.4};
    double curvature_ff_gain{1.0};
  };

  struct DebugPublishers {
    ros::Publisher rear_axle_pose;
    ros::Publisher front_axle_pose;
    ros::Publisher tracking_pose;
    ros::Publisher nearest_pose;
    ros::Publisher control_pose;
    ros::Publisher lookahead_pose;
    ros::Publisher feedforward_pose;
    ros::Publisher active_path;

    ros::Publisher current_speed;
    ros::Publisher target_speed;
    ros::Publisher raw_lookahead_speed;
    ros::Publisher speed_error;
    ros::Publisher steer_command;
    ros::Publisher curvature_feedforward;
    ros::Publisher curvature_preview;
    ros::Publisher curvature_preview_distance;
    ros::Publisher steer_saturated;
    ros::Publisher cross_track_error;
    ros::Publisher heading_error;
    ros::Publisher lookahead_distance;
    ros::Publisher segment_index;
    ros::Publisher segment_count;
    ros::Publisher nearest_index;
    ros::Publisher target_index;
    ros::Publisher curvature_preview_index;
    ros::Publisher distance_to_segment_end;
    ros::Publisher trajectory_age;
    ros::Publisher direction;
    ros::Publisher tracking_state;
    ros::Publisher stop_reason;
    ros::Publisher speed_selection_reason;
    ros::Publisher speed_selection_reason_text;
  } debug_pubs_;

  void loadParameters();
  LookaheadParams loadLookaheadParams(const std::string& direction);
  StanleyParams loadStanleyParams(const std::string& direction, const StanleyParams& defaults);

  void trajectoryCallback(const carmaker_msgs::TrajectoryPathConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void dynamicsCallback(const carmaker_msgs::DynamicsInfoConstPtr& msg);
  void controlTimerCallback(const ros::TimerEvent& event);

  bool processTrajectory(const carmaker_msgs::TrajectoryPathConstPtr& msg);
  PathPoint makePathPoint(const carmaker_msgs::TrajectoryPoint& msg, int direction) const;
  int trajectoryDirection(const carmaker_msgs::TrajectoryPath& msg) const;

  bool getCurrentState(VehicleState& state) const;
  std::size_t findNearestIndex(const ActiveTrajectory& trajectory,
                               const Pose2D& pose,
                               std::size_t previous_index) const;
  std::size_t findLookaheadIndex(const ActiveTrajectory& trajectory,
                                 std::size_t nearest_index,
                                 double lookahead_distance) const;
  double computePreviewCurvature(const ActiveTrajectory& trajectory,
                                 std::size_t nearest_index,
                                 std::size_t& preview_index) const;
  double computeCurvatureFeedforward(double preview_curvature, int direction, double ff_gain) const;
  double computeOffTrackingOffset(double wheelbase, double effective_curvature) const;
  double computeSteeringCommand(const Pose2D& pose,
                                const PathPoint& feedback_reference,
                                double preview_curvature,
                                double speed,
                                int direction,
                                double rear_curvature = 0.0,
                                double distance_to_end = 999.0) const;
  TargetSpeedDecision selectTargetSpeed(const ActiveTrajectory& trajectory,
                                        std::size_t nearest_index,
                                        std::size_t target_index,
                                        double distance_to_end) const;
  double computeRemainingDistance(const std::vector<PathPoint>& path,
                                  std::size_t nearest_index,
                                  const Pose2D& pose,
                                  int direction) const;
  static TargetSpeedDecision selectTargetSpeedForPath(const std::vector<PathPoint>& path,
                                                      std::size_t nearest_index,
                                                      std::size_t target_index,
                                                      double distance_to_end,
                                                      double min_tracking_speed,
                                                      double min_creep_speed,
                                                      double max_target_speed,
                                                      double stop_velocity_epsilon,
                                                      double arrival_slow_distance);

  bool isStopTrajectory(const std::vector<PathPoint>& points) const;

  void advertiseDebugTopics();
  template <typename T>
  ros::Publisher advertiseDebug(const std::string& suffix, uint32_t queue_size = 10, bool latch = false);

  void publishDebugTelemetry(const Pose2D& pose,
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
                             int stop_reason = 0,
                             double raw_target_speed = 0.0,
                             int speed_selection_reason = kSpeedReasonEndpointStop,
                             std::size_t preview_index = std::numeric_limits<std::size_t>::max(),
                             double preview_distance = 0.0,
                             double preview_curvature = 0.0);
  void publishStopReason(int stop_reason);
  void publishPoseDebug(const ros::Publisher& publisher,
                        double x,
                        double y,
                        double yaw,
                        const ros::Time& stamp) const;
  void publishActiveSegmentPath(const ActiveTrajectory& trajectory, const ros::Time& stamp);
  void publishControl(int direction, double steer_command, double accel_command);
  void publishStop(int direction, double steer_command = 0.0);
  void resetIdleRelease();
  bool publishIdleStopUntilRelease(const ros::Time& now);

  static double distance2D(const Pose2D& pose, const PathPoint& point);
  static double distance2D(const PathPoint& a, const PathPoint& b);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber trajectory_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber dynamics_sub_;
  ros::Publisher control_pub_;
  ros::Timer control_timer_;

  // Thread-safe state inputs
  mutable std::mutex odom_mutex_;
  nav_msgs::Odometry latest_odom_;
  ros::Time last_odom_time_;
  std::atomic<bool> odom_received_{false};

  mutable std::mutex dynamics_mutex_;
  carmaker_msgs::DynamicsInfo latest_dynamics_;
  ros::Time last_dynamics_time_;
  std::atomic<bool> dynamics_received_{false};

  // Trajectory input (callback thread writes, timer thread reads)
  mutable std::mutex trajectory_mutex_;
  carmaker_msgs::TrajectoryPathConstPtr latest_trajectory_msg_;
  ros::Time last_trajectory_time_;
  std::atomic<bool> trajectory_received_{false};

  // Trajectory processing state (strictly timer thread-confined)
  ActiveTrajectory active_trajectory_;
  carmaker_msgs::TrajectoryPathConstPtr processed_trajectory_msg_;
  std::size_t nearest_index_{0};

  PID speed_pid_;
  Stanley forward_stanley_;
  Stanley reverse_stanley_;

  std::string trajectory_topic_;
  std::string odom_topic_;
  std::string dynamics_topic_;
  std::string control_topic_;
  std::string state_source_{"odom"};
  std::string debug_topic_prefix_{"/control/debug"};
  std::string debug_frame_id_{"Fr0"};

  bool publish_stop_without_path_{true};
  bool publish_debug_{true};
  double control_rate_{30.0};
  double odom_timeout_{0.5};
  double dynamics_timeout_{0.5};
  double active_trajectory_timeout_{1.0};
  double stop_trajectory_velocity_epsilon_{1e-6};

  double min_tracking_speed_{0.2};
  double min_creep_speed_{0.1};
  double arrival_slow_distance_{0.5};
  double alignment_fade_distance_{2.0};
  double front_curvature_weight_{0.5};
  int off_tracking_mode_{3};
  double forward_control_lookahead_{0.0};
  double reverse_control_lookahead_{0.82};
  double max_target_speed_{0.7};

  LookaheadParams forward_lookahead_;
  LookaheadParams reverse_lookahead_;

  int nearest_search_back_{20};
  int nearest_search_ahead_{120};

  double max_accel_{0.8};
  double max_decel_{1.5};
  double max_gas_{0.35};
  double max_brake_{1.0};
  double stop_brake_{0.4};

  int drive_gear_{1};
  int neutral_gear_{0};
  int reverse_gear_{-1};

  double wheelbase_{2.97};
  double steering_ratio_{9.0};
  double max_steer_command_{4.5};
  double steering_command_sign_{1.0};
  double max_tire_steer_{0.6};

  StanleyParams forward_stanley_params_;
  StanleyParams reverse_stanley_params_;

  double reverse_curvature_ff_sign_{-1.0};
  double last_steer_command_{0.0};
  double idle_release_initial_steer_{0.0};
  std::atomic<bool> steer_release_after_idle_{false};
  std::atomic<bool> idle_release_active_{false};
  std::atomic<bool> idle_control_released_{false};
  double release_duration_{1.0};
  ros::Time idle_release_start_time_;

  ros::Time last_control_time_;
  ros::Time last_debug_path_time_;
  double debug_path_period_{1.0};

  // Dynamic Reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<carmaker_control::CarmakerControlConfig>> reconfigure_server_;
  void reconfigureCallback(carmaker_control::CarmakerControlConfig& config, uint32_t level);
};

}  // namespace carmaker_control

#endif  // CARMAKER_CONTROL_CONTROL_NODE_H
