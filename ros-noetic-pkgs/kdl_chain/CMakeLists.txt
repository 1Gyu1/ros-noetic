cmake_minimum_required(VERSION 2.8.3)
project(kdl_chain)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  PoseArray.msg
  JointArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
  geometry_msgs
)