#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HELPER="$SCRIPT_DIR/../_shared/install_with_apt.sh"

if [[ ! -f "$HELPER" ]]; then
  echo "[rcl_api] helper script not found: $HELPER" >&2
  exit 1
fi

TARGET_NAME="rcl_api" bash "$HELPER" 'python3 -c "import rclpy"' ros-foxy-rclpy ros-foxy-std-msgs
