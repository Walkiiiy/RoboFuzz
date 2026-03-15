#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PARAMS_FILE="${SCRIPT_DIR}/params.yaml"

exec bash -lc "ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file '${PARAMS_FILE}'"
