#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HELPER="$SCRIPT_DIR/../_shared/install_with_apt.sh"

if [[ ! -f "$HELPER" ]]; then
  echo "[robot_state_publisher] helper script not found: $HELPER" >&2
  exit 1
fi

VERIFY_CMD='
  ros2 pkg prefix robot_state_publisher >/dev/null &&
  ros2 interface show sensor_msgs/msg/JointState >/dev/null &&
  ros2 interface show tf2_msgs/msg/TFMessage >/dev/null
'

TARGET_NAME="robot_state_publisher" bash "$HELPER" \
  "$VERIFY_CMD" \
  ros-foxy-robot-state-publisher \
  ros-foxy-sensor-msgs \
  ros-foxy-tf2-msgs
