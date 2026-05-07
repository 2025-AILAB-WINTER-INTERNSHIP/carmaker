cmake_minimum_required(VERSION 3.0.2)
project(carmaker_image_synchronizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  carmaker_msgs
  sensor_msgs
  diagnostic_msgs
  message_filters
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES carmaker_image_synchronizer_nodelet
  CATKIN_DEPENDS roscpp nodelet carmaker_msgs sensor_msgs diagnostic_msgs message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(carmaker_image_synchronizer_nodelet src/image_synchronizer_nodelet.cpp)
target_link_libraries(carmaker_image_synchronizer_nodelet ${catkin_LIBRARIES})

if(catkin_EXPORTED_TARGETS)
  add_dependencies(carmaker_image_synchronizer_nodelet ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS carmaker_image_synchronizer_nodelet
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

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
