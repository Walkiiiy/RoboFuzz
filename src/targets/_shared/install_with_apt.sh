#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: install_with_apt.sh <verify_cmd> [apt_packages...]" >&2
  exit 2
fi

VERIFY_CMD="$1"
shift
TARGET_NAME="${TARGET_NAME:-target}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[${TARGET_NAME}] ros2 not found; source /opt/ros/foxy/setup.bash first." >&2
  exit 1
fi

if bash -lc "$VERIFY_CMD" >/dev/null 2>&1; then
  echo "[${TARGET_NAME}] dependency check passed"
  exit 0
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "[${TARGET_NAME}] apt-get not available; cannot auto-install dependencies" >&2
  exit 1
fi

SUDO=""
if [[ "$(id -u)" -ne 0 ]]; then
  SUDO="sudo"
fi

${SUDO} apt-get update
if [[ $# -gt 0 ]]; then
  ${SUDO} apt-get install -y --reinstall "$@"
fi
${SUDO} ldconfig || true

if ! bash -lc "$VERIFY_CMD" >/dev/null 2>&1; then
  echo "[${TARGET_NAME}] dependency check still failing after install" >&2
  echo "[${TARGET_NAME}] verify_cmd: $VERIFY_CMD" >&2
  exit 1
fi

echo "[${TARGET_NAME}] dependency installation completed"
