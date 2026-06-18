cmake_minimum_required(VERSION 3.0.2)
project(carmaker_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nodelet
  carmaker_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  message_filters
  image_transport
  diagnostic_updater
  visualization_msgs
)

find_package(OpenCV CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES carmaker_localization_nodelet
  CATKIN_DEPENDS
    roscpp nodelet carmaker_msgs sensor_msgs geometry_msgs nav_msgs
    rospy
    cv_bridge tf2_ros tf2_geometry_msgs message_filters image_transport
    diagnostic_updater
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(carmaker_localization_nodelet
  src/localization_nodelet.cpp
  src/feature_extractor.cpp
  src/ekf_core.cpp
  src/visualizer.cpp
  src/osm_feature_loader.cpp
  src/icp_registration.cpp
)
target_link_libraries(carmaker_localization_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(carmaker_localization_nodelet ${catkin_EXPORTED_TARGETS})
endif()

catkin_install_python(PROGRAMS
  tools/evaluate_ekf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS carmaker_localization_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch config data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
