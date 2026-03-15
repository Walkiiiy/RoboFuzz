#!/usr/bin/env bash
set -euo pipefail

# echo "pulling robofuzz docker origin image..." 
# docker pull ghcr.io/sslab-gatech/robofuzz:latest
# docker tag ghcr.io/sslab-gatech/robofuzz:latest robofuzz

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
IMAGE="${IMAGE:-robofuzz}"
CONTAINER_NAME="${CONTAINER_NAME:-robofuzz}"
FUZZING_FILE="${FUZZING_FILE:-$SCRIPT_DIR/fuzzer.py}"
LOGS_DIR="${LOGS_DIR:-$SCRIPT_DIR/logs}"
TARGETS_DIR="${TARGETS_DIR:-$REPO_ROOT/src/targets}"


if [[ ! -f "$FUZZING_FILE" ]]; then
  echo "fuzzer.py not found: $FUZZING_FILE" >&2
  exit 1
fi

mkdir -p "$LOGS_DIR"

TARGETS_MOUNT_ARGS=()
if [[ -d "$TARGETS_DIR" ]]; then
  TARGETS_MOUNT_ARGS=(
    -v "$TARGETS_DIR:/robofuzz/src/targets"
  )
else
  echo "[run_robofuzz] targets dir not found, skipping mount: $TARGETS_DIR" >&2
fi

docker run --rm -it \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --name "$CONTAINER_NAME" \
  -v "$SCRIPT_DIR:/robofuzz/src" \
  "${TARGETS_MOUNT_ARGS[@]}" \
  "$IMAGE" "$@"
