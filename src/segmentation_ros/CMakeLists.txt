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
  scripts/bundle_image_visualizer_node.py
  scripts/segmentation_inference_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(UNIX)
  add_custom_target(${PROJECT_NAME}_chmod_generated_python_nodes ALL
    COMMAND /bin/sh -c "for f in '${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/bundle_image_visualizer_node.py' '${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/segmentation_inference_node.py'; do if [ -f \"$$f\" ]; then chmod +x \"$$f\" || exit 1; fi; done"
    COMMENT "Ensuring generated Python node wrappers are executable"
    VERBATIM
  )
endif()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
