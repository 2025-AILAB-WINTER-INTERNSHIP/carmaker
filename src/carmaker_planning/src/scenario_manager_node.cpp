#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <carmaker_msgs/ArrivalError.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <mutex>

namespace carmaker_planning {

class ScenarioManager {
public:
  enum class State {
    kIdle,
    kParking
  };

  ScenarioManager() : nh_(), pnh_("~"), state_(State::kIdle), attempts_(0), has_target_pose_(false) {
    loadParameters();

    // Fetch topics from parameters (RULE §4)
    std::string goal_in_topic = pnh_.param<std::string>("topics/subscribe/goal", "/planning/goal");
    std::string error_in_topic = pnh_.param<std::string>("topics/subscribe/arrival_error", "/planning/local/arrival_error");
    std::string goal_out_topic = pnh_.param<std::string>("topics/publish/goal", "/planning/goal");
    std::string result_out_topic = pnh_.param<std::string>("topics/publish/result", "/parking/result");

    // Subscribers (RULE §1 — delegate processing from callbacks)
    goal_sub_ = nh_.subscribe(goal_in_topic, 1, &ScenarioManager::goalCallback, this);
    error_sub_ = nh_.subscribe(error_in_topic, 1, &ScenarioManager::arrivalErrorCallback, this);

    // Publishers
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_out_topic, 1);
    result_pub_ = nh_.advertise<std_msgs::Bool>(result_out_topic, 1, true);

    ROS_INFO("[ScenarioManager] Initialized.");
  }

  void loadParameters() {
    pnh_.param<int>("parking_scenario/max_re_parking_attempts", max_re_parking_attempts_, 3);
    pnh_.param<double>("parking_scenario/re_parking_xy_threshold", re_parking_xy_threshold_, 0.10);
    pnh_.param<double>("parking_scenario/re_parking_yaw_threshold", re_parking_yaw_threshold_, 0.05);

    ROS_INFO("[ScenarioManager] Parameters loaded: max_attempts=%d, xy_thresh=%.3fm, yaw_thresh=%.3f rad (%.2f deg)",
             max_re_parking_attempts_, re_parking_xy_threshold_, re_parking_yaw_threshold_, re_parking_yaw_threshold_ * 180.0 / M_PI);
  }

private:
  double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  double getYaw(const geometry_msgs::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  // RULE §1 SRP: Callback only enqueues/stores data under lock
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) return; // Null Guard (RULE §2)

    geometry_msgs::PoseStamped goal_msg;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      goal_msg = *msg;
    }
    processGoal(goal_msg);
  }

  // RULE §1 SRP: Callback only enqueues/stores data under lock
  void arrivalErrorCallback(const carmaker_msgs::ArrivalError::ConstPtr& msg) {
    if (!msg) return; // Null Guard (RULE §2)

    carmaker_msgs::ArrivalError error_msg;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      error_msg = *msg;
    }
    processArrivalError(error_msg);
  }

  // Processing logic delegated from callbacks
  void processGoal(const geometry_msgs::PoseStamped& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    double msg_yaw = getYaw(msg.pose.orientation);

    bool is_new_goal = !has_target_pose_ ||
                       std::abs(target_pose_.pose.position.x - msg.pose.position.x) > 1e-3 ||
                       std::abs(target_pose_.pose.position.y - msg.pose.position.y) > 1e-3 ||
                       std::abs(normalizeAngle(getYaw(target_pose_.pose.orientation) - msg_yaw)) > 1e-3;

    if (is_new_goal || state_ == State::kIdle) {
      target_pose_ = msg;
      has_target_pose_ = true;
      attempts_ = 0;
      state_ = State::kParking;
      ROS_INFO("[ScenarioManager] New parking goal received: (%.2f, %.2f, %.2f deg). Starting scenario.",
               msg.pose.position.x, msg.pose.position.y, msg_yaw * 180.0 / M_PI);
    } else {
      ROS_INFO("[ScenarioManager] Goal update matches current scenario target. Keeping state.");
    }
  }

  // Processing logic delegated from callbacks
  void processArrivalError(const carmaker_msgs::ArrivalError& msg) {
    State current_state;
    int current_attempts;
    geometry_msgs::PoseStamped current_target_pose;
    bool has_pose;
    
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_state = state_;
      current_attempts = attempts_;
      current_target_pose = target_pose_;
      has_pose = has_target_pose_;
    }

    if (current_state != State::kParking) {
      ROS_WARN("[ScenarioManager] Received arrival error but state is not kParking. Ignoring.");
      return;
    }

    ROS_INFO("[ScenarioManager] Path completed. Final errors: XY = %.3f m (threshold = %.3f m), Yaw = %.3f deg (threshold = %.3f deg)",
             msg.xy_error, re_parking_xy_threshold_, msg.yaw_error * 180.0 / M_PI, re_parking_yaw_threshold_ * 180.0 / M_PI);

    if (msg.xy_error <= re_parking_xy_threshold_ && msg.yaw_error <= re_parking_yaw_threshold_) {
      ROS_INFO("[ScenarioManager] SUCCESS! Target alignment achieved in %d attempt(s).", current_attempts + 1);

      std_msgs::Bool result_msg;
      result_msg.data = true;
      result_pub_.publish(result_msg);

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_ = State::kIdle;
        has_target_pose_ = false;
      }
    } else {
      int next_attempts = current_attempts + 1;
      if (next_attempts > max_re_parking_attempts_) {
        ROS_WARN("[ScenarioManager] FAILED! Maximum re-parking attempts (%d) reached. Remaining errors: XY = %.3f m, Yaw = %.3f deg",
                 max_re_parking_attempts_, msg.xy_error, msg.yaw_error * 180.0 / M_PI);

        std_msgs::Bool result_msg;
        result_msg.data = false;
        result_pub_.publish(result_msg);

        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          state_ = State::kIdle;
          has_target_pose_ = false;
        }
      } else {
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          attempts_ = next_attempts;
        }
        ROS_INFO("[ScenarioManager] Realignment required. Triggering attempt %d/%d...", next_attempts, max_re_parking_attempts_);

        // Republish target goal to trigger replan
        if (has_pose) {
          // Wait a small moment before publishing to let the planner reset properly
          ros::Duration(0.5).sleep();
          goal_pub_.publish(current_target_pose);
        } else {
          ROS_ERROR("[ScenarioManager] Target pose is not set during realignment trigger!");
          std::lock_guard<std::mutex> lock(state_mutex_);
          state_ = State::kIdle;
        }
      }
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber goal_sub_, error_sub_;
  ros::Publisher goal_pub_, result_pub_;

  // Thread safety (RULE §3)
  std::mutex state_mutex_;
  State state_;
  int attempts_;
  geometry_msgs::PoseStamped target_pose_;
  bool has_target_pose_;

  // Parameters
  int max_re_parking_attempts_;
  double re_parking_xy_threshold_;
  double re_parking_yaw_threshold_;
};

} // namespace carmaker_planning

int main(int argc, char** argv) {
  ros::init(argc, argv, "scenario_manager_node");
  carmaker_planning::ScenarioManager node;
  ros::spin();
  return 0;
}
