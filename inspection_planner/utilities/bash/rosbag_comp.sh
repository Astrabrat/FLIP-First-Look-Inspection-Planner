#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

# Set default duration in seconds

# Parse input arguments
BAG_NAME=""

if [[ $# -ge 1 ]]; then
  BAG_NAME="$1"
else
  TIMESTAMP=$(date +'%Y%m%d_%H%M%S')
  BAG_NAME="rosbag_${TIMESTAMP}"
fi


echo "Recording rosbag as: $BAG_NAME"

# Run rosbag record with timeout
rosbag record --lz4 -o "$BAG_NAME" \
  /spot/ouster/points \
  /tf \
  /tf_static \
  /spot/odometry/imu \
  /spot/camera/color/camera_info \
  /spot/camera/depth/camera_info \
  /spot/camera/color/image_raw \
  /spot/camera/depth/image_raw \
  /spot/inspection_planner/results/inspection_performance \
  /spot/inspection_planner/results/cropped_points \
  /spot/inspection_planner/results/maintained_distance \
  /spot/inspection_planner/results/predicted_path \
  /spot/filtered_pointcloud \
  /cpu_monitor/spot/inspection_planner/planner_node/cpu \
  /spot/command/pose \





