cmake_minimum_required(VERSION 3.0.2)
project(carmaker_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_message_files(
  FILES
  Control_Signal.msg
  DynamicsInfo.msg
  LocalFeature.msg
  LocalFeatures.msg
  Object.msg
  ObjectState.msg
  Objects.msg
  UAQ_Out.msg
  CameraBundle.msg
  IcpRegistrationMetrics.msg
  TrajectoryPoint.msg
  TrajectoryPath.msg
  ArrivalError.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  geometry_msgs
)
