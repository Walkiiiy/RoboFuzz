#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found; source /opt/ros/foxy/setup.bash first." >&2
  exit 1
fi

needs_install=0
if ! ros2 pkg prefix nav2_amcl >/dev/null 2>&1; then
  needs_install=1
fi
if ! python3 -c "from rosidl_generator_py.import_type_support_impl import import_type_support; import_type_support('nav_msgs')" >/dev/null 2>&1; then
  needs_install=1
fi

if [[ "${needs_install}" -eq 0 ]]; then
  echo "[nav2 install] nav2_amcl and nav_msgs typesupport are healthy"
  exit 0
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Support both layouts:
# - /repo/targets/nav2_amcl/install.sh -> ../../src/nav2_env_prep.sh
# - /repo/src/targets/nav2_amcl/install.sh -> ../../nav2_env_prep.sh
PREP_SCRIPT=""
for cand in \
  "${SCRIPT_DIR}/../../src/nav2_env_prep.sh" \
  "${SCRIPT_DIR}/../../nav2_env_prep.sh"; do
  if [[ -f "$cand" ]]; then
    PREP_SCRIPT="$cand"
    break
  fi
done

if [[ -z "$PREP_SCRIPT" ]]; then
  echo "[nav2 install] nav2_env_prep.sh not found" >&2
  exit 1
fi

bash "$PREP_SCRIPT"

SUDO=""
if [[ "$(id -u)" -ne 0 ]]; then
  SUDO="sudo"
fi

${SUDO} apt-get update
${SUDO} apt-get install -y --reinstall \
  ros-foxy-nav-msgs \
  ros-foxy-nav2-msgs \
  ros-foxy-nav2-amcl \
  ros-foxy-nav2-bringup \
  ros-foxy-builtin-interfaces \
  ros-foxy-std-msgs \
  ros-foxy-geometry-msgs \
  ros-foxy-rosidl-generator-py \
  ros-foxy-rosidl-runtime-c \
  ros-foxy-rosidl-runtime-cpp \
  ros-foxy-rosidl-generator-c \
  ros-foxy-rosidl-typesupport-interface \
  ros-foxy-rosidl-typesupport-c \
  ros-foxy-rosidl-typesupport-cpp \
  ros-foxy-rosidl-typesupport-fastrtps-c \
  ros-foxy-rosidl-typesupport-fastrtps-cpp

${SUDO} ldconfig

if ! ros2 pkg prefix nav2_amcl >/dev/null 2>&1; then
  echo "[nav2 install] nav2_amcl still missing after install" >&2
  exit 1
fi
if ! python3 -c "from rosidl_generator_py.import_type_support_impl import import_type_support; import_type_support('nav_msgs')" >/dev/null 2>&1; then
  echo "[nav2 install] nav_msgs rosidl_typesupport still unavailable after install" >&2
  exit 1
fi

echo "[nav2 install] nav2_amcl installed successfully"
