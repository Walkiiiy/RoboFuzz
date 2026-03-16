#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/log"
PROC_LOG="$LOG_DIR/image_proc.log"

mkdir -p "$LOG_DIR"
: >"$PROC_LOG"

cleanup() {
  jobs -pr | xargs -r kill >/dev/null 2>&1 || true
  wait || true
}

dump_logs_and_fail() {
  echo "[image_proc] startup failed: $1" >&2
  echo "--- image_proc log (tail) ---" >&2
  tail -n 120 "$PROC_LOG" >&2 || true
  exit 1
}

trap cleanup EXIT INT TERM

ros2 launch image_proc image_proc.launch.py >>"$PROC_LOG" 2>&1 &
PROC_PID=$!

node_ready=0
for _ in $(seq 1 20); do
  if ! kill -0 "$PROC_PID" 2>/dev/null; then
    dump_logs_and_fail "image_proc launch exited during startup"
  fi

  if ros2 node list 2>/dev/null | grep -q '^/RectifyNode$'; then
    node_ready=1
    break
  fi

  sleep 1
done

if [[ "$node_ready" -eq 0 ]]; then
  dump_logs_and_fail "/RectifyNode never appeared"
fi

while true; do
  if ! kill -0 "$PROC_PID" 2>/dev/null; then
    dump_logs_and_fail "image_proc launch exited after startup"
  fi
  sleep 1
done
