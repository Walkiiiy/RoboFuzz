from __future__ import annotations

import random

from target_plugins import BaseTargetPlugin

import oracles.nav2_amcl


class TargetPlugin(BaseTargetPlugin):
    """Nav2 AMCL target plugin.

    Includes message normalization (pre_exec hook) and target oracle binding.
    """

    def pre_exec_hook(self, msg):
        msg_name = type(msg).__name__

        if msg_name == "LaserScan":
            scan_len = getattr(self.runtime_config, "nav2_scan_len", 360)
            if msg.ranges is None or len(msg.ranges) == 0:
                msg.ranges = [5.0] * scan_len
            if msg.intensities is None or len(msg.intensities) == 0:
                msg.intensities = [0.0] * len(msg.ranges)

            range_min = msg.range_min if msg.range_min > 0 else 0.05
            range_max = msg.range_max if msg.range_max > range_min else 10.0
            for _ in range(random.randint(1, 5)):
                idx = random.randint(0, len(msg.ranges) - 1)
                msg.ranges[idx] = random.uniform(range_min, range_max)
            return msg

        if msg_name == "OccupancyGrid":
            width = msg.info.width or getattr(
                self.runtime_config,
                "nav2_map_default_width",
                384,
            )
            height = msg.info.height or getattr(
                self.runtime_config,
                "nav2_map_default_height",
                384,
            )

            msg.info.width = width
            msg.info.height = height

            expected_len = width * height
            if msg.data is None or len(msg.data) != expected_len:
                msg.data = [0] * expected_len

            for _ in range(random.randint(1, 10)):
                idx = random.randint(0, expected_len - 1)
                msg.data[idx] = random.choice([-1, 0, 100])

            return msg

        return msg

    def post_exec_hook(self) -> None:
        return None

    def check_oracle(self, config, msg_list, state_dict, feedback_list):
        return oracles.nav2_amcl.check(config, msg_list, state_dict, feedback_list)
