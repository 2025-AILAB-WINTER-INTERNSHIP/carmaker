/**
 * @file visualizer.cpp
 * @brief Implementation of Visualizer class for carmaker_planning.
 */

#include "carmaker_planning/visualizer.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace carmaker_planning {

Visualizer::Visualizer(const ros::NodeHandle& advertise_nh,
                       const ros::NodeHandle& param_nh,
                       bool publish_tree)
    : advertise_nh_(advertise_nh), param_nh_(param_nh), publish_tree_(publish_tree) {
  path_topic_       = param_nh_.param("topics/publish/debug/path",      std::string("/planning/debug/global/path"));
  velocity_topic_   = param_nh_.param("topics/publish/debug/velocity",  std::string("/planning/debug/global/velocity"));
  pose_array_topic_ = param_nh_.param("topics/publish/debug/pose",      std::string("/planning/debug/global/pose"));
  curvature_topic_  = param_nh_.param("topics/publish/debug/curvature", std::string("/planning/debug/global/curvature"));
  if (publish_tree_) {
    tree_topic_ = param_nh_.param("topics/publish/debug/tree", std::string("/planning/debug/global/tree"));
  }

  path_pub_ = advertise_nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
  velocity_pub_ = advertise_nh_.advertise<visualization_msgs::MarkerArray>(velocity_topic_, 1, true);
  pose_array_pub_ = advertise_nh_.advertise<geometry_msgs::PoseArray>(pose_array_topic_, 1, true);
  curvature_pub_ = advertise_nh_.advertise<visualization_msgs::Marker>(curvature_topic_, 1, true);
  if (publish_tree_) {
    tree_pub_ = advertise_nh_.advertise<visualization_msgs::MarkerArray>(tree_topic_, 1, true);
  }

  param_nh_.param("global_post_processing/visualization/arrow_spacing_meters", arrow_spacing_meters_, 0.2);
  param_nh_.param("local_post_processing/visualization/arrow_spacing_meters", arrow_spacing_meters_, arrow_spacing_meters_);
}

void Visualizer::visualize(const Path& path,
                           const std::vector<State>& tree,
                           const std::vector<std::pair<State, State>>& branches,
                           const std::string& frame_id,
                           double min_turning_radius) {
  if (path.empty()) return;

  ros::Time now = ros::Time::now();

  path_pub_.publish(createPathMsg(path, frame_id, now));
  velocity_pub_.publish(createVelocityMarkers(path, frame_id, now));
  pose_array_pub_.publish(createPoseArrayMsg(path, frame_id, now));
  curvature_pub_.publish(createCurvatureMarker(path, frame_id, min_turning_radius, now));
  if (publish_tree_) {
    tree_pub_.publish(createTreeMarkers(tree, branches, frame_id, now));
  }
}

nav_msgs::Path Visualizer::createPathMsg(const Path& path, const std::string& frame_id, const ros::Time& stamp) {
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = frame_id;
  path_msg.header.stamp = stamp;

  for (const auto& pt : path) {
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;

    pose.pose.position.x = pt.x;
    pose.pose.position.y = pt.y;
    pose.pose.position.z = 0.0;

    // Convert yaw to geometry_msgs::Quaternion
    double half_theta = pt.theta * 0.5;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(half_theta);
    pose.pose.orientation.w = std::cos(half_theta);

    path_msg.poses.push_back(pose);
  }
  return path_msg;
}

visualization_msgs::MarkerArray Visualizer::createVelocityMarkers(const Path& path, const std::string& frame_id, const ros::Time& stamp) {
  visualization_msgs::MarkerArray velocity_markers;

  // Add DELETEALL marker to clear previous speed cylinders
  visualization_msgs::Marker delete_marker;
  delete_marker.header.frame_id = frame_id;
  delete_marker.header.stamp = stamp;
  delete_marker.ns = "velocity";
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  velocity_markers.markers.push_back(delete_marker);

  int marker_id = 0;
  for (const auto& pt : path) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "velocity";
    m.id = marker_id++;
    m.type = visualization_msgs::Marker::CYLINDER;
    m.action = visualization_msgs::Marker::ADD;

    double speed = std::abs(pt.v);
    double height_scale = 1.0; // Scale velocity visually
    double height = speed * height_scale;

    m.pose.position.x = pt.x;
    m.pose.position.y = pt.y;
    m.pose.position.z = height / 2.0; // bottom touches the ground
    m.pose.orientation.w = 1.0;

    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = std::max(height, 0.001); // avoid 0 height warning in RViz

    if (pt.direction == -1) {
      // Solid Red for Reverse
      m.color.r = 0.9; m.color.g = 0.0; m.color.b = 0.0;
    } else {
      // Solid Cyan for Forward
      m.color.r = 0.0; m.color.g = 0.9; m.color.b = 0.9;
    }
    m.color.a = 0.7;

    velocity_markers.markers.push_back(m);
  }
  return velocity_markers;
}

geometry_msgs::PoseArray Visualizer::createPoseArrayMsg(const Path& path, const std::string& frame_id, const ros::Time& stamp) {
  geometry_msgs::PoseArray pose_array_msg;
  pose_array_msg.header.frame_id = frame_id;
  pose_array_msg.header.stamp = stamp;
  pose_array_msg.poses.reserve(path.size());

  double last_s = -999.0;

  for (size_t i = 0; i < path.size(); ++i) {
    const auto& pt = path[i];
    bool is_first = (i == 0);
    bool is_last = (i == path.size() - 1);
    bool is_cusp_start = (i > 0 && path[i].direction != path[i-1].direction);
    bool is_cusp_end = (i < path.size() - 1 && path[i].direction != path[i+1].direction);

    if (is_first || is_last || is_cusp_start || is_cusp_end || (pt.s - last_s >= arrow_spacing_meters_)) {
      geometry_msgs::Pose pose;

      pose.position.x = pt.x;
      pose.position.y = pt.y;
      pose.position.z = 0.03; // Small height offset above the curvature line

      double half_theta = pt.theta * 0.5;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = std::sin(half_theta);
      pose.orientation.w = std::cos(half_theta);

      pose_array_msg.poses.push_back(pose);
      last_s = pt.s;
    }
  }
  return pose_array_msg;
}

visualization_msgs::Marker Visualizer::createCurvatureMarker(const Path& path, const std::string& frame_id, double min_turning_radius, const ros::Time& stamp) {
  visualization_msgs::Marker curv_line;
  curv_line.header.frame_id = frame_id;
  curv_line.header.stamp = stamp;
  curv_line.ns = "curvature";
  curv_line.id = 0;
  curv_line.type = visualization_msgs::Marker::LINE_STRIP;
  curv_line.action = visualization_msgs::Marker::ADD;
  curv_line.scale.x = 0.08; // Line width
  curv_line.pose.orientation.w = 1.0;

  curv_line.points.reserve(path.size());
  curv_line.colors.reserve(path.size());

  const double max_kappa = (min_turning_radius > 0.1) ? (1.0 / min_turning_radius) : 0.2;

  for (const auto& pt : path) {
    geometry_msgs::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = 0.01; // Slightly below arrows/cylinders but above road

    curv_line.points.push_back(p);
    curv_line.colors.push_back(makeCurvatureColor(pt.kappa, max_kappa));
  }
  return curv_line;
}

std_msgs::ColorRGBA Visualizer::makeCurvatureColor(double kappa, double max_kappa) {
  double raw_ratio = std::abs(kappa) / (max_kappa > 1e-6 ? max_kappa : 1.0);
  double ratio = std::min(1.0, std::pow(raw_ratio, 2.0)); // Non-linear scaling (square) for gradual transition
  std_msgs::ColorRGBA color;
  color.r = ratio;
  color.g = 1.0 - ratio;
  color.b = 0.0;
  color.a = 0.8;
  return color;
}

visualization_msgs::MarkerArray Visualizer::createTreeMarkers(const std::vector<State>& tree,
                                                              const std::vector<std::pair<State, State>>& branches,
                                                              const std::string& frame_id,
                                                              const ros::Time& stamp) {
  visualization_msgs::MarkerArray markers;

  // 1. Nodes marker
  visualization_msgs::Marker nodes_marker;
  nodes_marker.header.frame_id = frame_id;
  nodes_marker.header.stamp = stamp;
  nodes_marker.ns = "nodes";
  nodes_marker.id = 0;
  nodes_marker.type = visualization_msgs::Marker::POINTS;
  nodes_marker.action = visualization_msgs::Marker::ADD;
  nodes_marker.pose.orientation.w = 1.0;
  nodes_marker.scale.x = 0.05;
  nodes_marker.scale.y = 0.05;

  // Sleek cyan/green color
  nodes_marker.color.r = 0.0;
  nodes_marker.color.g = 1.0;
  nodes_marker.color.b = 0.5;
  nodes_marker.color.a = 0.4;

  for (const auto& st : tree) {
    geometry_msgs::Point p;
    p.x = st.x;
    p.y = st.y;
    p.z = 0.05;
    nodes_marker.points.push_back(p);
  }
  markers.markers.push_back(nodes_marker);

  // 2. Branches marker
  visualization_msgs::Marker branches_marker;
  branches_marker.header.frame_id = frame_id;
  branches_marker.header.stamp = stamp;
  branches_marker.ns = "branches";
  branches_marker.id = 1;
  branches_marker.type = visualization_msgs::Marker::LINE_LIST;
  branches_marker.action = visualization_msgs::Marker::ADD;
  branches_marker.pose.orientation.w = 1.0;
  branches_marker.scale.x = 0.015;

  // Glowing cyan
  branches_marker.color.r = 0.0;
  branches_marker.color.g = 0.8;
  branches_marker.color.b = 0.8;
  branches_marker.color.a = 0.3;

  for (const auto& branch : branches) {
    geometry_msgs::Point p1, p2;
    p1.x = branch.first.x;
    p1.y = branch.first.y;
    p1.z = 0.03;
    p2.x = branch.second.x;
    p2.y = branch.second.y;
    p2.z = 0.03;
    branches_marker.points.push_back(p1);
    branches_marker.points.push_back(p2);
  }
  markers.markers.push_back(branches_marker);

  return markers;
}

void Visualizer::clear(const std::string& frame_id) {
  const ros::Time now = ros::Time::now();

  nav_msgs::Path clear_path;
  clear_path.header.frame_id = frame_id;
  clear_path.header.stamp = now;
  path_pub_.publish(clear_path);

  visualization_msgs::MarkerArray clear_msg;

  visualization_msgs::Marker clear_nodes;
  clear_nodes.header.frame_id = frame_id;
  clear_nodes.header.stamp = now;
  clear_nodes.action = visualization_msgs::Marker::DELETEALL;
  clear_nodes.ns = "nodes";
  clear_msg.markers.push_back(clear_nodes);

  visualization_msgs::Marker clear_branches;
  clear_branches.header.frame_id = frame_id;
  clear_branches.header.stamp = now;
  clear_branches.action = visualization_msgs::Marker::DELETEALL;
  clear_branches.ns = "branches";
  clear_msg.markers.push_back(clear_branches);

  visualization_msgs::Marker clear_vel;
  clear_vel.header.frame_id = frame_id;
  clear_vel.header.stamp = now;
  clear_vel.action = visualization_msgs::Marker::DELETEALL;
  clear_vel.ns = "velocity";
  clear_msg.markers.push_back(clear_vel);

  if (publish_tree_) {
    tree_pub_.publish(clear_msg);
  }
  velocity_pub_.publish(clear_msg);

  geometry_msgs::PoseArray clear_pose_array;
  clear_pose_array.header.frame_id = frame_id;
  clear_pose_array.header.stamp = now;
  pose_array_pub_.publish(clear_pose_array);

  visualization_msgs::Marker clear_curv;
  clear_curv.header.frame_id = frame_id;
  clear_curv.header.stamp = now;
  clear_curv.ns = "curvature";
  clear_curv.id = 0;
  clear_curv.action = visualization_msgs::Marker::DELETEALL;
  curvature_pub_.publish(clear_curv);
}

} // namespace carmaker_planning
