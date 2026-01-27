cmake_minimum_required(VERSION 3.0.2)
project(inspection_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  actionlib_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
  jsk_recognition_msgs

)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  InspectionPerformance.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#     MissionStatus.srv
#     AutonomyStatus.srv
#     InitializeInspection.srv
#     PathQuery.srv
#     PlanQuery.srv
#     InterpolatePath.srv
# )

# Generate actions in the 'action' folder
# add_action_files(
  # FILES
  # pathcollisioncheck.action
  # Navigation.action
  # Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs   nav_msgs   std_msgs   visualization_msgs jsk_recognition_msgs actionlib_msgs
)



###################################
## catkin specific configuration ##
###################################

catkin_package(
#  LIBRARIES gecko_msgs
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs visualization_msgs message_runtime jsk_recognition_msgs actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
