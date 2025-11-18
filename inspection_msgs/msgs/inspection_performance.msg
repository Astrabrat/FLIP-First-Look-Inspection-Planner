#InspectionPerformance.msg

# Standard ROS header
std_msgs/Header header

#evaluation
std_msgs/Float64 view_planning_time

# Mission status
std_msgs/String current_mission_status

# Safety / inspection constraints
std_msgs/Float64 maintained_distance
std_msgs/Float64 desired_distance
std_msgs/Float64 view_quality