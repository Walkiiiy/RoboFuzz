#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HELPER="$SCRIPT_DIR/../_shared/install_with_apt.sh"

if [[ ! -f "$HELPER" ]]; then
  echo "[turtlebot3_sitl] helper script not found: $HELPER" >&2
  exit 1
fi

TARGET_NAME="turtlebot3_sitl" bash "$HELPER" 'ros2 pkg prefix turtlebot3_gazebo >/dev/null' ros-foxy-turtlebot3 ros-foxy-turtlebot3-msgs ros-foxy-turtlebot3-gazebo ros-foxy-turtlebot3-bringup
