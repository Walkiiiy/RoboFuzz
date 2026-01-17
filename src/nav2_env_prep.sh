#!/usr/bin/env bash
set -euo pipefail



SUDO=""
if [ "$(id -u)" -ne 0 ]; then
  SUDO="sudo"
fi

# 1. Ubuntu 源（中科大）
echo "配置中科 Ubuntu 源..."
$SUDO tee /etc/apt/sources.list > /dev/null << 'EOF'
deb https://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse
EOF



SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-/robofuzz/targets/nav2}"
ROS_DISTRO="${ROS_DISTRO:-foxy}"
BUILD_FROM_SOURCE="${BUILD_FROM_SOURCE:-0}"

if [[ "${ROS_DISTRO}" != "foxy" ]]; then
  echo "[nav2] WARNING: this script is tuned for ROS2 Foxy; detected ROS_DISTRO=${ROS_DISTRO}" >&2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[nav2] ros2 not found in PATH. Source your ROS2 setup first." >&2
  echo "       e.g., source /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

if [[ "${BUILD_FROM_SOURCE}" == "1" ]]; then
  echo "[nav2] Building nav2 from source into ${WORKSPACE_ROOT}"
  mkdir -p "${WORKSPACE_ROOT}"
  cd "${WORKSPACE_ROOT}"

  if [[ ! -d src/navigation2 ]]; then
    git clone https://github.com/ros-planning/navigation2.git -b foxy-devel src/navigation2
  fi

  rosdep update
  rosdep install -y --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}"

  colcon build \
    --symlink-install \
    --packages-up-to nav2_amcl nav2_bringup

  echo "[nav2] Build complete. Source the overlay:"
  echo "       source ${WORKSPACE_ROOT}/install/setup.bash"
  exit 0
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "[nav2] apt-get not found; set BUILD_FROM_SOURCE=1 instead." >&2
  exit 1
fi

echo "[nav2] Configuring ROS 2 apt mirror fallback"
${SUDO} apt-get install -y curl gnupg2 lsb-release
${SUDO} curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
${SUDO} rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2-latest.list /etc/apt/sources.list.d/ros2-testing.list

ros2_mirrors=(
  "https://mirrors.aliyun.com/ros2/ubuntu/"
  "https://mirrors.bfsu.edu.cn/ros2/ubuntu/"
  "https://mirrors.cloud.tencent.com/ros2/ubuntu/"
  "https://mirrors.iscas.ac.cn/ros2/ubuntu/"
  "https://mirror.sjtu.edu.cn/ros2/ubuntu/"
  "https://repo.huaweicloud.com/ros2/ubuntu/"
  "https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/"
  "https://mirrors.ustc.edu.cn/ros2/ubuntu/"
  "http://packages.ros.org/ros2/ubuntu/"
)

install_ok=0
for mirror in "${ros2_mirrors[@]}"; do
  echo "[nav2] Using ROS 2 mirror: ${mirror}"
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${mirror} $(lsb_release -cs) main" \
    | ${SUDO} tee /etc/apt/sources.list.d/ros2.list > /dev/null

  ${SUDO} apt-get clean
  ${SUDO} rm -rf /var/lib/apt/lists/*
  if ! ${SUDO} apt-get update; then
    echo "[nav2] apt-get update failed; trying next mirror..."
    continue
  fi

  echo "[nav2] Installing nav2 from deb packages (ROS ${ROS_DISTRO})"
  if ${SUDO} apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-amcl \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-sensor-msgs \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-geometry-msgs; then
    install_ok=1
    break
  fi

  echo "[nav2] Install failed; trying next mirror..."
done

if [[ "${install_ok}" -ne 1 ]]; then
  echo "[nav2] All ROS 2 mirrors failed. Check network or mirror availability." >&2
  exit 1
fi

echo "[nav2] Install complete. Try:"
echo "       ros2 pkg prefix nav2_amcl"
