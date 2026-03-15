#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HELPER="$SCRIPT_DIR/../_shared/install_with_apt.sh"

if [[ ! -f "$HELPER" ]]; then
  echo "[rosidl] helper script not found: $HELPER" >&2
  exit 1
fi

TARGET_NAME="rosidl" bash "$HELPER" 'python3 -c "import rosidl_runtime_py; import rosidl_generator_py"' ros-foxy-rosidl-runtime-py ros-foxy-rosidl-generator-py ros-foxy-std-msgs
