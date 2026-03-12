#!/usr/bin/env bash
set -euo pipefail

WS=/inspection_ws

# Source ROS setup safely: temporarily disable 'nounset' because setup.bash
# may reference variables that are intentionally unset.
if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "Warning: /opt/ros/humble/setup.bash not found. Is ROS installed?" >&2
fi

# Go to workspace
cd "$WS"

echo "=== Cleaning build/install/log ==="
rm -rf build install log

echo "=== Running colcon build (Release, symlink install) ==="
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace install if it exists (again allow undefined vars while sourcing)
if [ -f install/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source install/setup.bash
  set -u
  echo "=== Build finished and workspace sourced ==="
else
  echo "Warning: install/setup.bash not found — build may have failed." >&2
fi