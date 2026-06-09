/**
 * @file visualizer.h
 * @brief Visualizer class for carmaker_planning matching the style of carmaker_localization.
 */
#ifndef CARMAKER_PLANNING_VISUALIZER_H
#define CARMAKER_PLANNING_VISUALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <string>
#include <utility>
#include "carmaker_planning/types.h"

namespace carmaker_planning {

class Visualizer {
public:
  explicit Visualizer(const ros::NodeHandle& advertise_nh,
                      const ros::NodeHandle& param_nh = ros::NodeHandle("~"),
                      bool publish_tree = true);
  ~Visualizer() = default;
  Visualizer(const Visualizer&) = delete;
  Visualizer& operator=(const Visualizer&) = delete;

  void visualize(const Path& path,
                 const std::vector<State>& tree,
                 const std::vector<std::pair<State, State>>& branches,
                 const std::string& frame_id,
                 double min_turning_radius);
  void clear();

private:
  nav_msgs::Path createPathMsg(const Path& path, const std::string& frame_id, const ros::Time& stamp);
  visualization_msgs::MarkerArray createVelocityMarkers(const Path& path, const std::string& frame_id, const ros::Time& stamp);
  geometry_msgs::PoseArray createPoseArrayMsg(const Path& path, const std::string& frame_id, const ros::Time& stamp);
  visualization_msgs::Marker createCurvatureMarker(const Path& path, const std::string& frame_id, double min_turning_radius, const ros::Time& stamp);
  visualization_msgs::MarkerArray createTreeMarkers(const std::vector<State>& tree,
                                                    const std::vector<std::pair<State, State>>& branches,
                                                    const std::string& frame_id,
                                                    const ros::Time& stamp);
  std_msgs::ColorRGBA makeCurvatureColor(double kappa, double max_kappa);

  ros::NodeHandle advertise_nh_;
  ros::NodeHandle param_nh_;
  ros::Publisher path_pub_;
  ros::Publisher tree_pub_;
  ros::Publisher velocity_pub_;
  ros::Publisher pose_array_pub_;
  ros::Publisher curvature_pub_;

  std::string path_topic_;
  std::string tree_topic_;
  std::string velocity_topic_;
  std::string pose_array_topic_;
  std::string curvature_topic_;

  bool publish_tree_ = true;
  double arrow_spacing_meters_ = 0.2;
};

} // namespace carmaker_planning

#endif // CARMAKER_PLANNING_VISUALIZER_H
