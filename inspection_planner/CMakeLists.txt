cmake_minimum_required(VERSION 3.16)
project(inspection_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  inspection_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gecko_inspection_planner
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


