cmake_minimum_required(VERSION 2.8.3)
project(coordinate_publisher)

find_package(catkin
  REQUIRED COMPONENTS
  roscpp
  rospy
  openni_tracker
  tf
  visualization_msgs
  geometry_msgs
  message_generation
)
#include_directories(${Eigen_INCLUDE_DIRS} ${pcl_ros_INCLUDE_DIRS})


add_message_files(
  FILES
  PointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
