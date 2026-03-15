#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HELPER="$SCRIPT_DIR/../_shared/install_with_apt.sh"

if [[ ! -f "$HELPER" ]]; then
  echo "[px4_sitl_ros] helper script not found: $HELPER" >&2
  exit 1
fi

TARGET_NAME="px4_sitl_ros" bash "$HELPER" 'ros2 interface show px4_msgs/msg/TrajectorySetpoint >/dev/null' ros-foxy-px4-msgs ros-foxy-mavros-msgs ros-foxy-geometry-msgs
