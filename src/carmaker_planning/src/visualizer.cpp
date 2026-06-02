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

Visualizer::Visualizer(const ros::NodeHandle& nh) : nh_(nh) {
  ros::NodeHandle pnh("~");

  path_topic_ = pnh.param("topics/publish/debug/path", std::string("/planning/debug/global_path"));
  tree_topic_ = pnh.param("topics/publish/debug/tree", std::string("/planning/debug/search_tree"));
  velocity_topic_ = pnh.param("topics/publish/debug/velocity", std::string("/planning/debug/velocity_profile"));
  pose_array_topic_ = pnh.param("topics/publish/debug/pose", std::string("/planning/debug/global_pose"));

  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
  tree_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(tree_topic_, 1, true);
  velocity_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(velocity_topic_, 1, true);
  pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>(pose_array_topic_, 1, true);
}

void Visualizer::publishPath(const Path& path, const std::string& frame_id, double rear_axle_offset) {
  if (path.empty()) return;

  ros::Time now = ros::Time::now();

  // 1. Publish standard nav_msgs::Path
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = frame_id;
  path_msg.header.stamp = now;

  for (const auto& pt : path) {
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;

    // Translate planned path point (rear axle center) backward to rear bumper
    pose.pose.position.x = pt.x - rear_axle_offset * std::cos(pt.theta);
    pose.pose.position.y = pt.y - rear_axle_offset * std::sin(pt.theta);
    pose.pose.position.z = 0.0;

    // Convert yaw to geometry_msgs::Quaternion
    double half_theta = pt.theta * 0.5;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(half_theta);
    pose.pose.orientation.w = std::cos(half_theta);

    path_msg.poses.push_back(pose);
  }
  path_pub_.publish(path_msg);

  // 2. Publish 3D Cyan Cylinder Speed markers
  visualization_msgs::MarkerArray velocity_markers;

  // Add DELETEALL marker to clear previous speed cylinders
  visualization_msgs::Marker delete_marker;
  delete_marker.header.frame_id = frame_id;
  delete_marker.header.stamp = now;
  delete_marker.ns = "velocity";
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  velocity_markers.markers.push_back(delete_marker);

  int marker_id = 0;
  for (const auto& pt : path) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = now;
    m.ns = "velocity";
    m.id = marker_id++;
    m.type = visualization_msgs::Marker::CYLINDER;
    m.action = visualization_msgs::Marker::ADD;

    // Bumper-referred coordinate same as path visualization
    double bx = pt.x - rear_axle_offset * std::cos(pt.theta);
    double by = pt.y - rear_axle_offset * std::sin(pt.theta);

    double speed = std::abs(pt.v);
    double height_scale = 1.0; // Scale velocity visually
    double height = speed * height_scale;

    m.pose.position.x = bx;
    m.pose.position.y = by;
    m.pose.position.z = height / 2.0; // bottom touches the ground
    m.pose.orientation.w = 1.0;

    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = std::max(height, 0.001); // avoid 0 height warning in RViz

    if (pt.direction == -1) {
      // Solid Red for Reverse
      m.color.r = 0.9;
      m.color.g = 0.0;
      m.color.b = 0.0;
    } else {
      // Solid Cyan for Forward
      m.color.r = 0.0;
      m.color.g = 0.9;
      m.color.b = 0.9;
    }
    m.color.a = 0.7;

    velocity_markers.markers.push_back(m);
  }
  velocity_pub_.publish(velocity_markers);

  // 3. Publish PoseArray for Yaw Arrows
  geometry_msgs::PoseArray pose_array_msg;
  pose_array_msg.header.frame_id = frame_id;
  pose_array_msg.header.stamp = now;

  const size_t stride = 5; // Visualize every 5th point to avoid clutter (0.5m spacing at 0.1m resolution)
  pose_array_msg.poses.reserve(path.size() / stride + 1);

  for (size_t i = 0; i < path.size(); i += stride) {
    const auto& pt = path[i];
    geometry_msgs::Pose pose;

    // Translate planned path point (rear axle center) backward to rear bumper to match path visualization
    pose.position.x = pt.x - rear_axle_offset * std::cos(pt.theta);
    pose.position.y = pt.y - rear_axle_offset * std::sin(pt.theta);
    pose.position.z = 0.02; // Small height offset above the path line

    double half_theta = pt.theta * 0.5;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(half_theta);
    pose.orientation.w = std::cos(half_theta);

    pose_array_msg.poses.push_back(pose);
  }
  pose_array_pub_.publish(pose_array_msg);
}

void Visualizer::publishTree(const std::vector<State>& tree,
                             const std::vector<std::pair<State, State>>& branches,
                             const std::string& frame_id) {
  visualization_msgs::MarkerArray markers;
  ros::Time now = ros::Time::now();

  // 1. Nodes marker
  visualization_msgs::Marker nodes_marker;
  nodes_marker.header.frame_id = frame_id;
  nodes_marker.header.stamp = now;
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
  branches_marker.header.stamp = now;
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

  tree_pub_.publish(markers);
}

void Visualizer::clear() {
  visualization_msgs::MarkerArray clear_msg;

  visualization_msgs::Marker clear_nodes;
  clear_nodes.action = visualization_msgs::Marker::DELETEALL;
  clear_nodes.ns = "nodes";
  clear_msg.markers.push_back(clear_nodes);

  visualization_msgs::Marker clear_branches;
  clear_branches.action = visualization_msgs::Marker::DELETEALL;
  clear_branches.ns = "branches";
  clear_msg.markers.push_back(clear_branches);

  visualization_msgs::Marker clear_vel;
  clear_vel.action = visualization_msgs::Marker::DELETEALL;
  clear_vel.ns = "velocity";
  clear_msg.markers.push_back(clear_vel);

  tree_pub_.publish(clear_msg);
  velocity_pub_.publish(clear_msg);

  geometry_msgs::PoseArray clear_pose_array;
  clear_pose_array.header.frame_id = "Fr0";
  clear_pose_array.header.stamp = ros::Time::now();
  pose_array_pub_.publish(clear_pose_array);
}

} // namespace carmaker_planning
