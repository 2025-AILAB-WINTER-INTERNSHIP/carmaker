/**
 * @file ros_utils.h
 * @brief ROS message conversion and logging helpers shared by planner nodelets.
 */
#ifndef CARMAKER_PLANNING_ROS_UTILS_H
#define CARMAKER_PLANNING_ROS_UTILS_H

#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <carmaker_msgs/TrajectoryPath.h>

#include "carmaker_planning/types.h"

namespace carmaker_planning {

Path trajectoryMsgToPath(const carmaker_msgs::TrajectoryPath& msg);
carmaker_msgs::TrajectoryPath pathToTrajectoryMsg(const Path& path,
                                                  const std::string& frame_id);
void logPlannerMessages(const std::string& prefix,
                        const std::vector<std::pair<std::string, std::string>>& logs);

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_ROS_UTILS_H
