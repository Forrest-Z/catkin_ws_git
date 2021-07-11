cmake_minimum_required(VERSION 2.8.12)
project(veh_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  lanePoint.msg
  laneInfo.msg
  curbPoint.msg
  curbInfo.msg  
  GPS.msg
  Preview.msg
  Search.msg
  Cmd.msg
  Mode.msg
  Sts.msg
  LaserScanARS.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
