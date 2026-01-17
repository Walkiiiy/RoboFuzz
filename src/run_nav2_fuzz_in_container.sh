#!/usr/bin/env bash
set -euo pipefail

MAXLOOP="${MAXLOOP:-}"
DO_INSTALL="${DO_INSTALL:-0}"
MAP_FILE="${MAP_FILE:-/opt/ros/foxy/share/nav2_bringup/maps/turtlebot3_world.yaml}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found; source /opt/ros/foxy/setup.bash first." >&2
  exit 1
fi

if [[ "${DO_INSTALL}" == "1" ]]; then
  if ! ros2 pkg prefix nav2_amcl >/dev/null 2>&1; then
    ./nav2_env_prep.sh
  fi

  sudo apt-get update
  sudo apt-get install -y --reinstall \
    ros-foxy-builtin-interfaces \
    ros-foxy-std-msgs \
    ros-foxy-geometry-msgs \
    ros-foxy-rosidl-runtime-c \
    ros-foxy-rosidl-runtime-cpp \
    ros-foxy-rosidl-generator-c \
    ros-foxy-rosidl-typesupport-interface \
    ros-foxy-rosidl-typesupport-c \
    ros-foxy-rosidl-typesupport-cpp \
    ros-foxy-rosidl-typesupport-fastrtps-c \
    ros-foxy-rosidl-typesupport-fastrtps-cpp
  sudo ldconfig
fi

# Restart localization-only Nav2 to ensure amcl is active.
pkill -f nav2_bringup || true
pkill -f nav2_amcl_feeder.py || true
ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=False \
  autostart:=True \
  map:="${MAP_FILE}" \
  >/tmp/nav2_loc.log 2>&1 &

sleep 8
ros2 node list

python3 ./utils/nav2_amcl_feeder.py >/tmp/nav2_amcl_feeder.log 2>&1 &

CMD=(./fuzzer.py --no-cov --nav2-amcl --exec-cmd "true" \
  --watchlist watchlist/nav2_amcl.json \
  --method message --schedule single --interval 0.1 \
  --persistent --target-node amcl)

if [[ -n "${MAXLOOP}" ]]; then
  CMD+=(--maxloop "${MAXLOOP}")
fi

"${CMD[@]}"
