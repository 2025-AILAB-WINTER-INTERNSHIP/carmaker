#include "carmaker_planning/ros_utils.h"

#include <cmath>

#include "carmaker_planning/math.h"

namespace carmaker_planning {

Path trajectoryMsgToPath(const carmaker_msgs::TrajectoryPath& msg) {
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
    pt.x = tp.pose.position.x;
    pt.y = tp.pose.position.y;
    pt.theta = quaternionToYaw(tp.pose.orientation);
    pt.kappa = tp.curvature;
    pt.t = tp.time_from_start.toSec();
    pt.direction = active_direction;
    pt.v = std::abs(tp.longitudinal_velocity);
    pt.a = tp.longitudinal_acceleration * pt.direction;
    path.push_back(pt);
  }

  if (!path.empty()) {
    path.front().s = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
      path[i].s = path[i - 1].s + dist(path[i - 1], path[i]);
    }
  }
  return path;
}

carmaker_msgs::TrajectoryPath pathToTrajectoryMsg(const Path& path,
                                                  const std::string& frame_id) {
  carmaker_msgs::TrajectoryPath msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  msg.points.reserve(path.size());

  for (const auto& pt : path) {
    carmaker_msgs::TrajectoryPoint tp;
    tp.pose.position.x = pt.x;
    tp.pose.position.y = pt.y;
    tp.pose.position.z = 0.0;

    const double half_theta = pt.theta * 0.5;
    tp.pose.orientation.x = 0.0;
    tp.pose.orientation.y = 0.0;
    tp.pose.orientation.z = std::sin(half_theta);
    tp.pose.orientation.w = std::cos(half_theta);

    tp.longitudinal_velocity = pt.v * pt.direction;
    tp.longitudinal_acceleration = pt.a * pt.direction;
    tp.curvature = pt.kappa;
    tp.direction = pt.direction < 0 ? -1 : 1;
    tp.time_from_start = ros::Duration(pt.t);
    msg.points.push_back(tp);
  }
  return msg;
}

void logPlannerMessages(const std::string& prefix,
                        const std::vector<std::pair<std::string, std::string>>& logs) {
  for (const auto& log : logs) {
    const std::string message = prefix + " " + log.second;
    if (log.first == "WARN") {
      ROS_WARN("%s", message.c_str());
    } else if (log.first == "ERROR") {
      ROS_ERROR("%s", message.c_str());
    } else if (log.first == "INFO") {
      ROS_INFO("%s", message.c_str());
    } else if (log.first == "DEBUG") {
      ROS_DEBUG("%s", message.c_str());
    }
  }
}

} // namespace carmaker_planning
