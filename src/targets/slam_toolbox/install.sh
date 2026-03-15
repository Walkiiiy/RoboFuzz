#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HELPER="$SCRIPT_DIR/../_shared/install_with_apt.sh"

if [[ ! -f "$HELPER" ]]; then
  echo "[slam_toolbox] helper script not found: $HELPER" >&2
  exit 1
fi

VERIFY_CMD='
  ros2 pkg prefix slam_toolbox >/dev/null &&
  ros2 pkg prefix turtlebot3_gazebo >/dev/null &&
  ros2 interface show sensor_msgs/msg/LaserScan >/dev/null &&
  ros2 interface show nav_msgs/msg/OccupancyGrid >/dev/null
'

TARGET_NAME="slam_toolbox" bash "$HELPER" \
  "$VERIFY_CMD" \
  ros-foxy-slam-toolbox \
  ros-foxy-turtlebot3-gazebo \
  ros-foxy-turtlebot3-description \
  ros-foxy-sensor-msgs \
  ros-foxy-nav-msgs \
  ros-foxy-tf2-msgs
