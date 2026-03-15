#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HELPER="$SCRIPT_DIR/../_shared/install_with_apt.sh"
TARGET_NAME="robot_localization"
RL_WS="${ROBOT_LOCALIZATION_WS:-/robofuzz/robot_localization_ws}"
SMOKE_LOG="/tmp/robot_localization_smoke.log"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[${TARGET_NAME}] ros2 not found; source /opt/ros/foxy/setup.bash first." >&2
  exit 1
fi

smoke_check() {
  set +e
  bash -lc "
    source /opt/ros/foxy/setup.bash
    if [ -f '${RL_WS}/install/setup.bash' ]; then
      source '${RL_WS}/install/setup.bash'
    fi
    timeout 6s ros2 run robot_localization ekf_node --ros-args \
      --params-file /opt/ros/foxy/share/robot_localization/params/ekf.yaml \
      >'${SMOKE_LOG}' 2>&1
  "
  rc=$?
  set -e

  # timeout(124) means process stayed alive, which is success for liveness smoke.
  [[ "$rc" -eq 124 ]]
}

if smoke_check; then
  echo "[${TARGET_NAME}] ekf_node smoke check passed"
  exit 0
fi

if [[ -f "$HELPER" ]]; then
  TARGET_NAME="$TARGET_NAME" bash "$HELPER" \
    'ros2 pkg prefix robot_localization >/dev/null && ros2 pkg executables robot_localization | grep -Eq "(^|[[:space:]])(ekf_node|ukf_node)$"' \
    ros-foxy-robot-localization \
    ros-foxy-geometry-msgs \
    ros-foxy-nav-msgs \
    ros-foxy-sensor-msgs \
    ros-foxy-tf2 \
    ros-foxy-tf2-ros
else
  echo "[${TARGET_NAME}] helper not found, continuing with direct recovery" >&2
fi

if smoke_check; then
  echo "[${TARGET_NAME}] ekf_node recovered after apt reinstall"
  exit 0
fi

echo "[${TARGET_NAME}] apt package still crashes, trying source build fallback"

SUDO=""
if [[ "$(id -u)" -ne 0 ]]; then
  SUDO="sudo"
fi

${SUDO} apt-get update
${SUDO} apt-get install -y \
  git \
  build-essential \
  python3-colcon-common-extensions \
  python3-rosdep

mkdir -p "${RL_WS}/src"
if [[ ! -d "${RL_WS}/src/robot_localization/.git" ]]; then
  git clone -b foxy-devel https://github.com/cra-ros-pkg/robot_localization.git "${RL_WS}/src/robot_localization"
else
  git -C "${RL_WS}/src/robot_localization" fetch --all --tags
  git -C "${RL_WS}/src/robot_localization" checkout foxy-devel
  git -C "${RL_WS}/src/robot_localization" pull --ff-only
fi

set +e
${SUDO} rosdep init >/dev/null 2>&1
set -e
rosdep update || true

bash -lc "
  source /opt/ros/foxy/setup.bash
  rosdep install -y --from-paths '${RL_WS}/src' --ignore-src --rosdistro \
    '${ROS_DISTRO:-foxy}'
  colcon build --merge-install \
    --base-paths '${RL_WS}/src' \
    --packages-select robot_localization \
    --build-base '${RL_WS}/build' \
    --install-base '${RL_WS}/install'
"

if ! smoke_check; then
  echo "[${TARGET_NAME}] smoke check failed after source build" >&2
  echo "[${TARGET_NAME}] last smoke log:" >&2
  tail -n 120 "${SMOKE_LOG}" >&2 || true
  exit 1
fi

echo "[${TARGET_NAME}] source-build fallback succeeded"
