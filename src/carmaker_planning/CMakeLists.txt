cmake_minimum_required(VERSION 3.0.2)
project(carmaker_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  diagnostic_updater
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  carmaker_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_global carmaker_planning_nodelet
            ${PROJECT_NAME}_local  carmaker_planning_local_nodelet
  CATKIN_DEPENDS roscpp nodelet diagnostic_updater nav_msgs geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs std_msgs carmaker_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ── Global Planning Library ──────────────────────────────────────────
add_library(${PROJECT_NAME}_global
  src/global_map.cpp
  src/global_planner.cpp
  src/hybrid_astar.cpp
  src/reeds_shepp.cpp
  src/post_processing.cpp
  src/visualizer.cpp
)
target_link_libraries(${PROJECT_NAME}_global ${catkin_LIBRARIES})

# ── Global Planner Nodelet ───────────────────────────────────────────
add_library(carmaker_planning_nodelet
  src/global_planner_nodelet.cpp
)
target_link_libraries(carmaker_planning_nodelet
  ${PROJECT_NAME}_global
  ${catkin_LIBRARIES}
)

# ── Local Planning Library ───────────────────────────────────────────
add_library(${PROJECT_NAME}_local
  src/local_planner.cpp
)
target_link_libraries(${PROJECT_NAME}_local
  ${PROJECT_NAME}_global   # provides post_processing, types, math
  ${catkin_LIBRARIES}
)

# ── Local Planner Nodelet ────────────────────────────────────────────
add_library(carmaker_planning_local_nodelet
  src/local_planner_nodelet.cpp
)
target_link_libraries(carmaker_planning_local_nodelet
  ${PROJECT_NAME}_local
  ${catkin_LIBRARIES}
)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(carmaker_planning_nodelet       ${catkin_EXPORTED_TARGETS})
  add_dependencies(carmaker_planning_local_nodelet ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS ${PROJECT_NAME}_global carmaker_planning_nodelet
                ${PROJECT_NAME}_local  carmaker_planning_local_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

