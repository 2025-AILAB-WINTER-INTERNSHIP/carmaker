cmake_minimum_required(VERSION 3.0.2)
project(carmaker_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  carmaker_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs carmaker_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(carmaker_control_node
  src/control_node.cpp
  src/pid.cpp
  src/stanley.cpp
)

add_dependencies(carmaker_control_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(carmaker_control_node
  ${catkin_LIBRARIES}
)

install(TARGETS carmaker_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
