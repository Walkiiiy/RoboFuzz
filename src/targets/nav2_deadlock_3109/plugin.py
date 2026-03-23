from __future__ import annotations

import copy
import math
import os
import subprocess as sp
import time

from rosidl_runtime_py import message_to_yaml
from target_plugins import BaseTargetPlugin


class TargetPlugin(BaseTargetPlugin):
    def __init__(self, target_config: dict, runtime_config) -> None:
        super().__init__(target_config, runtime_config)
        self._shape_toggle = False
        self._low_activity_rounds = 0

    def _runtime_int(self, key: str, default: int) -> int:
        try:
            return int(getattr(self.runtime_config, key, default))
        except Exception:
            return default

    def _runtime_float(self, key: str, default: float) -> float:
        try:
            return float(getattr(self.runtime_config, key, default))
        except Exception:
            return default

    def pre_exec_hook(self, msg):
        if type(msg).__name__ != "OccupancyGrid":
            return msg

        min_dim = max(1, self._runtime_int("deadlock_min_dimension", 64))
        max_dim = max(min_dim, self._runtime_int("deadlock_max_dimension", 512))
        max_cells = max(min_dim * min_dim, self._runtime_int("deadlock_max_cells", 16384))
        default_resolution = self._runtime_float("deadlock_default_resolution", 0.05)

        try:
            width = int(msg.info.width)
        except Exception:
            width = 0
        try:
            height = int(msg.info.height)
        except Exception:
            height = 0

        if width <= 0:
            width = min_dim
        if height <= 0:
            height = min_dim

        width = min(max(width, min_dim), max_dim)
        height = min(max(height, min_dim), max_dim)

        # Prevent huge YAML payloads when publishing via ROS CLI.
        while width * height > max_cells:
            if width >= height and width > min_dim:
                width = max(min_dim, width // 2)
            elif height > min_dim:
                height = max(min_dim, height // 2)
            else:
                break

        # Keep dimensions changing to better trigger the known lock-order bug.
        if width == height:
            if self._shape_toggle:
                width = min(max_dim, width + 1)
            else:
                height = min(max_dim, height + 1)
            self._shape_toggle = not self._shape_toggle

        msg.info.width = width
        msg.info.height = height

        try:
            resolution = float(msg.info.resolution)
        except Exception:
            resolution = default_resolution
        if (not math.isfinite(resolution)) or resolution <= 0.0:
            resolution = default_resolution
        msg.info.resolution = resolution

        now = time.time()
        msg.header.stamp.sec = int(now)
        msg.header.stamp.nanosec = int((now % 1.0) * 1000000000)
        msg.header.frame_id = "map"

        if (
            msg.info.origin.orientation.x == 0.0
            and msg.info.origin.orientation.y == 0.0
            and msg.info.origin.orientation.z == 0.0
            and msg.info.origin.orientation.w == 0.0
        ):
            msg.info.origin.orientation.w = 1.0

        expected_len = width * height
        if expected_len <= 0:
            expected_len = min_dim * min_dim
            msg.info.width = min_dim
            msg.info.height = min_dim

        if msg.data is None or len(msg.data) != expected_len:
            grid = [0] * expected_len
            step = max(1, width // 16)
            for row in range(0, msg.info.height, max(1, step * 3)):
                base = row * width
                end = min(base + step, expected_len)
                for idx in range(base, end):
                    grid[idx] = 100
            msg.data = grid
        else:
            msg.data = [max(-1, min(100, int(v))) for v in msg.data]

        return msg

    def publish_message(self, msg):
        self._publish_once(msg)

        alt_msg = copy.deepcopy(msg)
        max_dim = max(1, self._runtime_int("deadlock_max_dimension", 256))

        try:
            width = max(1, int(alt_msg.info.width))
            height = max(1, int(alt_msg.info.height))
        except Exception:
            width = 64
            height = 65

        if width >= height:
            height = min(max_dim, height + 1)
        else:
            width = min(max_dim, width + 1)

        alt_msg.info.width = width
        alt_msg.info.height = height
        alt_msg = self.pre_exec_hook(alt_msg)

        # Reproduce #3109 with quick dimension changes under repeated /map updates.
        time.sleep(0.5)
        self._publish_once(alt_msg)

    def _publish_once(self, msg):
        topic = (
            self.target_config.get("fuzzing", {}).get("default_topic")
            or "/map"
        )
        msg_type = (
            self.target_config.get("fuzzing", {}).get("default_msg_type")
            or "nav_msgs/msg/OccupancyGrid"
        )

        try:
            os.remove("out")
        except Exception:
            pass

        try:
            os.remove("err")
        except Exception:
            pass

        cmd = [
            "ros2",
            "topic",
            "pub",
            "--once",
            topic,
            msg_type,
            message_to_yaml(msg),
            "--qos-durability",
            "transient_local",
            "--qos-reliability",
            "reliable",
        ]

        with open("out", "w") as out, open("err", "w") as err:
            rc = sp.call(cmd, stdout=out, stderr=err)

        if rc != 0:
            raise RuntimeError(
                f"failed to publish {msg_type} to {topic}: ros2 topic pub rc={rc}"
            )

    def check_oracle(self, config, msg_list, state_dict, feedback_list):
        topic = "/global_costmap/costmap"
        samples = list(state_dict.get(topic, []))

        if len(samples) == 0:
            return [
                "deadlock suspected: no /global_costmap/costmap messages captured "
                "after /map publish"
            ]

        min_msgs_per_round = max(1, self._runtime_int("deadlock_min_costmap_msgs_per_round", 3))
        if len(samples) < min_msgs_per_round:
            self._low_activity_rounds += 1
        else:
            self._low_activity_rounds = 0

        low_activity_threshold = max(
            1, self._runtime_int("deadlock_stale_round_threshold", 3)
        )
        if self._low_activity_rounds >= low_activity_threshold:
            return [
                "deadlock suspected: /global_costmap/costmap activity dropped "
                f"below {min_msgs_per_round} msgs/round for "
                f"{self._low_activity_rounds} consecutive rounds"
            ]

        return []
