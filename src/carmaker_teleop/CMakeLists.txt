cmake_minimum_required(VERSION 3.0.2)
project(carmaker_teleop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  ackermann_msgs
  carmaker_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/keyboard_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
