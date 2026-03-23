#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HELPER="$SCRIPT_DIR/../_shared/install_with_apt.sh"
PREP_SCRIPT=""

if [[ ! -f "$HELPER" ]]; then
  echo "[nav2_controller_server] helper script not found: $HELPER" >&2
  exit 1
fi

for cand in \
  "${SCRIPT_DIR}/../../src/nav2_env_prep.sh" \
  "${SCRIPT_DIR}/../../nav2_env_prep.sh"; do
  if [[ -f "$cand" ]]; then
    PREP_SCRIPT="$cand"
    break
  fi
done

if [[ -n "$PREP_SCRIPT" ]]; then
  bash "$PREP_SCRIPT"
fi

VERIFY_CMD='
  ros2 pkg prefix nav2_controller >/dev/null &&
  ros2 pkg prefix nav2_bringup >/dev/null &&
  ros2 pkg prefix turtlebot3_gazebo >/dev/null &&
  ros2 pkg prefix turtlebot3_navigation2 >/dev/null &&
  ros2 interface show geometry_msgs/msg/Twist >/dev/null &&
  ros2 interface show nav_msgs/msg/Odometry >/dev/null &&
  ros2 interface show nav2_msgs/action/FollowPath >/dev/null &&
  python3 - <<'"'"'PY'"'"' >/dev/null
from rosidl_generator_py.import_type_support_impl import import_type_support
for pkg in ("std_msgs", "geometry_msgs", "nav_msgs", "nav2_msgs"):
    import_type_support(pkg)
PY
'

TARGET_NAME="nav2_controller_server" bash "$HELPER" \
  "$VERIFY_CMD" \
  ros-foxy-navigation2 \
  ros-foxy-nav2-bringup \
  ros-foxy-nav2-controller \
  ros-foxy-nav2-msgs \
  ros-foxy-dwb-msgs \
  ros-foxy-turtlebot3 \
  ros-foxy-turtlebot3-msgs \
  ros-foxy-turtlebot3-gazebo \
  ros-foxy-turtlebot3-navigation2 \
  ros-foxy-std-msgs \
  ros-foxy-builtin-interfaces \
  ros-foxy-geometry-msgs \
  ros-foxy-nav-msgs \
  ros-foxy-sensor-msgs \
  ros-foxy-rosidl-generator-py \
  ros-foxy-rosidl-runtime-c \
  ros-foxy-rosidl-runtime-cpp \
  ros-foxy-rosidl-generator-c \
  ros-foxy-rosidl-typesupport-interface \
  ros-foxy-rosidl-typesupport-c \
  ros-foxy-rosidl-typesupport-cpp \
  ros-foxy-rosidl-typesupport-fastrtps-c \
  ros-foxy-rosidl-typesupport-fastrtps-cpp
