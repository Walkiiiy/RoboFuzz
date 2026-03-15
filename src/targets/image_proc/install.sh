#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HELPER="$SCRIPT_DIR/../_shared/install_with_apt.sh"

if [[ ! -f "$HELPER" ]]; then
  echo "[image_proc] helper script not found: $HELPER" >&2
  exit 1
fi

VERIFY_CMD='
  ros2 pkg prefix image_proc >/dev/null &&
  ros2 interface show sensor_msgs/msg/Image >/dev/null &&
  ros2 interface show sensor_msgs/msg/CameraInfo >/dev/null
'

TARGET_NAME="image_proc" bash "$HELPER" \
  "$VERIFY_CMD" \
  ros-foxy-image-pipeline \
  ros-foxy-sensor-msgs
