#!/usr/bin/env bash
set -e

# Source underlay
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Source overlay if built
if [ -f "${WS:-/workspaces/ws}/install/setup.bash" ]; then
  source "${WS:-/workspaces/ws}/install/setup.bash"
fi

# Nice defaults for many setups (change if you need)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

exec "$@"
