cmake_minimum_required(VERSION 3.16)
project(inspection_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters)
find_package(inspection_msgs REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcl_transformer src/pcl_transformer.cpp)
ament_target_dependencies(pcl_transformer
  rclcpp
  tf2_ros
  tf2_sensor_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
)
target_link_libraries(pcl_transformer ${PCL_LIBRARIES})

# Python package
ament_python_install_package(${PROJECT_NAME})

# Install python node as an executable via console_scripts (preferred)
# but also install the file for backwards-compatibility.
install(PROGRAMS
  inspection_planner/planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pcl_transformer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
