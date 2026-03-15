#!/usr/bin/env bash
set -euo pipefail

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/log"
GAZEBO_LOG="$LOG_DIR/gazebo.log"
SLAM_LOG="$LOG_DIR/slam.log"

mkdir -p "$LOG_DIR"
: >"$GAZEBO_LOG"
: >"$SLAM_LOG"

cleanup() {
  jobs -pr | xargs -r kill >/dev/null 2>&1 || true
  wait || true
}

dump_logs_and_fail() {
  echo "[slam_toolbox] startup failed: $1" >&2
  echo "--- gazebo log (tail) ---" >&2
  tail -n 80 "$GAZEBO_LOG" >&2 || true
  echo "--- slam log (tail) ---" >&2
  tail -n 120 "$SLAM_LOG" >&2 || true
  exit 1
}

trap cleanup EXIT INT TERM

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py >>"$GAZEBO_LOG" 2>&1 &
GAZEBO_PID=$!
sleep 8

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True >>"$SLAM_LOG" 2>&1 &
SLAM_PID=$!

node_ready=0
map_ready=0
for _ in $(seq 1 60); do
  if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
    dump_logs_and_fail "gazebo exited during startup"
  fi

  if ! kill -0 "$SLAM_PID" 2>/dev/null; then
    dump_logs_and_fail "slam_toolbox exited during startup"
  fi

  if ros2 node list 2>/dev/null | grep -q '^/slam_toolbox$'; then
    node_ready=1
  fi

  if timeout 3 ros2 topic echo /map --once >/dev/null 2>&1; then
    map_ready=1
  fi

  if [[ "$node_ready" -eq 1 && "$map_ready" -eq 1 ]]; then
    break
  fi

  sleep 1
done

if [[ "$node_ready" -eq 0 ]]; then
  dump_logs_and_fail "/slam_toolbox node never appeared"
fi

if [[ "$map_ready" -eq 0 ]]; then
  dump_logs_and_fail "/map never produced a message during startup"
fi

while true; do
  if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
    dump_logs_and_fail "gazebo exited after startup"
  fi

  if ! kill -0 "$SLAM_PID" 2>/dev/null; then
    dump_logs_and_fail "slam_toolbox exited after startup"
  fi

  sleep 1
done
