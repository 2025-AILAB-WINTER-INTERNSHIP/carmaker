/**
 * @file visualizer.h
 * @brief Visualizer class for carmaker_planning matching the style of carmaker_localization.
 */
#ifndef CARMAKER_PLANNING_VISUALIZER_H
#define CARMAKER_PLANNING_VISUALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <string>
#include <utility>
#include "carmaker_planning/types.h"

namespace carmaker_planning {

class Visualizer {
public:
  explicit Visualizer(const ros::NodeHandle& nh);
  ~Visualizer() = default;
  Visualizer(const Visualizer&) = delete;
  Visualizer& operator=(const Visualizer&) = delete;

  void publishPath(const Path& path, const std::string& frame_id, double rear_axle_offset, double max_vel);
  void publishTree(const std::vector<State>& tree,
                   const std::vector<std::pair<State, State>>& branches,
                   const std::string& frame_id);
  void clear();

private:
  ros::NodeHandle nh_;
  ros::Publisher path_pub_;
  ros::Publisher tree_pub_;
  ros::Publisher velocity_pub_;

  std::string path_topic_;
  std::string tree_topic_;
  std::string velocity_topic_;
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_VISUALIZER_H
