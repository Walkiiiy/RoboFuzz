#!/usr/bin/env bash
set -euo pipefail

TARGET_NAME="moveit2_nan_crash"
ROS_DISTRO="${ROS_DISTRO:-foxy}"
PKG1="ros-${ROS_DISTRO}-moveit"
PKG2="ros-${ROS_DISTRO}-moveit-resources-panda-moveit-config"

SUDO=""
if [[ "$(id -u)" -ne 0 ]]; then
  SUDO="sudo"
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "[${TARGET_NAME}] apt-get not found" >&2
  exit 1
fi

${SUDO} apt-get update
${SUDO} env DEBIAN_FRONTEND=noninteractive \
  apt-get install -y --reinstall "${PKG1}" "${PKG2}"

if ! bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && ros2 pkg prefix moveit >/dev/null && ros2 pkg prefix moveit_resources_panda_moveit_config >/dev/null"; then
  echo "[${TARGET_NAME}] dependency verification failed" >&2
  exit 1
fi

echo "[${TARGET_NAME}] dependencies installed and verified"
