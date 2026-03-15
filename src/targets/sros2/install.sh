#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HELPER="$SCRIPT_DIR/../_shared/install_with_apt.sh"

if [[ ! -f "$HELPER" ]]; then
  echo "[sros2] helper script not found: $HELPER" >&2
  exit 1
fi

TARGET_NAME="sros2" bash "$HELPER" 'python3 -c "import rclpy" && ros2 security --help >/dev/null 2>&1' ros-foxy-sros2 ros-foxy-rclpy ros-foxy-std-msgs
