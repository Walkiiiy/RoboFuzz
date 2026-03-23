#!/usr/bin/env bash
set -euo pipefail

TARGET_NAME="nav2_deadlock_3109"
ROS_DISTRO="${ROS_DISTRO:-foxy}"
ROS_APT_KEY_URL="${ROS_APT_KEY_URL:-https://raw.githubusercontent.com/ros/rosdistro/master/ros.key}"
ROS_KEYRING="${ROS_KEYRING:-/usr/share/keyrings/ros-archive-keyring.gpg}"
PACKAGES=(
  "ros-${ROS_DISTRO}-navigation2"
  "ros-${ROS_DISTRO}-nav2-bringup"
  "ros-${ROS_DISTRO}-turtlebot3-gazebo"
)

SUDO=""
if [[ "$(id -u)" -ne 0 ]]; then
  SUDO="sudo"
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "[${TARGET_NAME}] apt-get not found" >&2
  exit 1
fi

resolve_ubuntu_codename() {
  if [[ -n "${UBUNTU_CODENAME:-}" ]]; then
    echo "${UBUNTU_CODENAME}"
    return
  fi

  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    echo "${UBUNTU_CODENAME:-focal}"
    return
  fi

  echo "focal"
}

ensure_ros_apt_source_and_key() {
  if ! command -v curl >/dev/null 2>&1; then
    return 0
  fi
  if ! command -v gpg >/dev/null 2>&1; then
    return 0
  fi

  local codename key_tmp
  codename="$(resolve_ubuntu_codename)"
  key_tmp="$(mktemp)"

  if ! curl -fsSL "${ROS_APT_KEY_URL}" -o "${key_tmp}"; then
    rm -f "${key_tmp}"
    return 0
  fi

  ${SUDO} install -d -m 0755 /usr/share/keyrings
  ${SUDO} gpg --dearmor --yes -o "${ROS_KEYRING}" "${key_tmp}"
  rm -f "${key_tmp}"

  # Remove legacy source entry to avoid duplicate repo warnings.
  ${SUDO} rm -f /etc/apt/sources.list.d/ros2-latest.list || true

  ${SUDO} tee /etc/apt/sources.list.d/ros2.list >/dev/null <<EOF
deb [signed-by=${ROS_KEYRING}] http://packages.ros.org/ros2/ubuntu ${codename} main
EOF
}

verify_install() {
  # 1) Package-level check (works even when ROS env is not sourced)
  for pkg in "${PACKAGES[@]}"; do
    if ! dpkg -s "${pkg}" >/dev/null 2>&1; then
      return 1
    fi
  done

  # 2) ROS-level check (best effort)
  local setup_file="/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ -f "${setup_file}" ]]; then
    bash -lc "
      source '${setup_file}'
      ros2 pkg prefix nav2_bringup >/dev/null &&
      ros2 pkg prefix turtlebot3_gazebo >/dev/null &&
      ros2 interface show nav_msgs/msg/OccupancyGrid >/dev/null
    "
  fi

  return 0
}

refresh_apt_index() {
  ensure_ros_apt_source_and_key || true
  ${SUDO} apt-get clean
  ${SUDO} rm -rf /var/lib/apt/lists/*
  if ${SUDO} apt-get \
    -o Acquire::Retries=5 \
    -o Acquire::http::No-Cache=true \
    -o Acquire::https::No-Cache=true \
    update; then
    return 0
  fi

  # Retry once after explicitly refreshing ROS apt key/source (handles EXPKEYSIG).
  ensure_ros_apt_source_and_key || true
  ${SUDO} apt-get \
    -o Acquire::Retries=5 \
    -o Acquire::http::No-Cache=true \
    -o Acquire::https::No-Cache=true \
    update
}

install_packages() {
  local attempts max_attempts
  max_attempts=5
  attempts=1

  while [[ ${attempts} -le ${max_attempts} ]]; do
    if ${SUDO} env DEBIAN_FRONTEND=noninteractive \
      apt-get -o Acquire::Retries=5 install -y --fix-missing --reinstall "${PACKAGES[@]}"; then
      return 0
    fi

    echo "[${TARGET_NAME}] apt install attempt ${attempts}/${max_attempts} failed; retrying..."
    sleep $((attempts * 3))
    refresh_apt_index || true
    attempts=$((attempts + 1))
  done

  return 1
}

run_nav2_repo_fallback_if_available() {
  local script_dir prep_script
  script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  prep_script=""
  for cand in \
    "${script_dir}/../../nav2_env_prep.sh" \
    "${script_dir}/../../src/nav2_env_prep.sh"; do
    if [[ -f "${cand}" ]]; then
      prep_script="${cand}"
      break
    fi
  done

  if [[ -n "${prep_script}" ]]; then
    echo "[${TARGET_NAME}] fallback: running ${prep_script}"
    bash "${prep_script}"
  fi
}

if verify_install; then
  echo "[${TARGET_NAME}] dependencies already satisfied"
  exit 0
fi

echo "[${TARGET_NAME}] installing Foxy dependencies via apt..."
if ! refresh_apt_index || ! install_packages; then
  echo "[${TARGET_NAME}] first install attempt failed, trying mirror fallback..."
  run_nav2_repo_fallback_if_available || true
  refresh_apt_index
  install_packages
fi

${SUDO} ldconfig || true

if ! verify_install; then
  echo "[${TARGET_NAME}] install completed but verify still failed." >&2
  echo "[${TARGET_NAME}] please check ROS source list and run:" >&2
  echo "  sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/* && sudo apt-get update" >&2
  exit 1
fi

echo "[${TARGET_NAME}] Foxy dependencies installed."
