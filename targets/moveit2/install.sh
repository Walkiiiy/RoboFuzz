#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HELPER="$SCRIPT_DIR/../_shared/install_with_apt.sh"

if [[ ! -f "$HELPER" ]]; then
  echo "[moveit2] helper script not found: $HELPER" >&2
  exit 1
fi

TARGET_NAME="moveit2" bash "$HELPER" 'ros2 pkg prefix moveit2_tutorials >/dev/null' ros-foxy-moveit2-tutorials ros-foxy-moveit ros-foxy-moveit-msgs
