#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HELPER="$SCRIPT_DIR/../_shared/install_with_apt.sh"

if [[ ! -f "$HELPER" ]]; then
  echo "[turtlesim] helper script not found: $HELPER" >&2
  exit 1
fi

TARGET_NAME="turtlesim" bash "$HELPER" 'ros2 pkg prefix turtlesim >/dev/null && python3 -c "import turtlesim.msg"' ros-foxy-turtlesim ros-foxy-geometry-msgs ros-foxy-std-msgs
