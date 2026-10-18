#!/usr/bin/env bash
# Source from other scripts; keep paths relative to this checkout.
KIT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Requires Ubuntu 22.04 with ROS2 Humble: /opt/ros/humble/setup.bash missing.' >&2
  return 1
fi
# ROS setup scripts may reference unset variables.
source /opt/ros/humble/setup.bash
if [[ -f "$KIT_ROOT/.ros_ws/install/setup.bash" ]]; then
  source "$KIT_ROOT/.ros_ws/install/setup.bash"
fi
export IDL_PACKAGE_FILTER='rm_interfaces;std_msgs;sensor_msgs;geometry_msgs;tf2_msgs;visualization_msgs;nav_msgs'
