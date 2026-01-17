#!/usr/bin/env bash
set -euo pipefail

# echo "pulling robofuzz docker origin image..." 
# docker pull ghcr.io/sslab-gatech/robofuzz:latest
# docker tag ghcr.io/sslab-gatech/robofuzz:latest robofuzz

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${IMAGE:-robofuzz}"
CONTAINER_NAME="${CONTAINER_NAME:-robofuzz}"
FUZZING_FILE="${FUZZING_FILE:-$SCRIPT_DIR/fuzzer.py}"
LOGS_DIR="${LOGS_DIR:-$SCRIPT_DIR/logs}"


if [[ ! -f "$FUZZING_FILE" ]]; then
  echo "fuzzer.py not found: $FUZZING_FILE" >&2
  exit 1
fi

mkdir -p "$LOGS_DIR"

docker run --rm -it \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --name "$CONTAINER_NAME" \
  -v "$FUZZING_FILE:/robofuzz/src/fuzzer.py" \
  -v "$LOGS_DIR:/robofuzz/src/logs" \
  "$IMAGE" "$@"
