#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HELPER="$SCRIPT_DIR/../_shared/install_with_apt.sh"

if [[ ! -f "$HELPER" ]]; then
  echo "[robot_localization] helper script not found: $HELPER" >&2
  exit 1
fi

VERIFY_CMD='ros2 pkg prefix robot_localization >/dev/null && ros2 pkg executables robot_localization | grep -Eq "(^|[[:space:]])(ekf_node|ukf_node)$"'

TARGET_NAME="robot_localization" bash "$HELPER" \
  "$VERIFY_CMD" \
  ros-foxy-robot-localization \
  ros-foxy-geometry-msgs \
  ros-foxy-nav-msgs \
  ros-foxy-sensor-msgs \
  ros-foxy-tf2 \
  ros-foxy-tf2-ros
