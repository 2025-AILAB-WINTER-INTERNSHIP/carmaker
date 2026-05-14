cmake_minimum_required(VERSION 3.0.2)
project(segmentation_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  carmaker_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  cv_bridge
  carmaker_msgs
)

catkin_install_python(PROGRAMS
  scripts/segmentation_inference_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
