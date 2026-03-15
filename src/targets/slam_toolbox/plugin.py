from __future__ import annotations

import math
import time

from target_plugins import BaseTargetPlugin

import oracles.slam_toolbox


class TargetPlugin(BaseTargetPlugin):
    def pre_exec_hook(self, msg):
        runtime = self.target_config.get("runtime", {})
        scan_len = int(runtime.get("slam_scan_len", 360))
        frame_id = runtime.get("slam_frame_id", "base_scan")
        range_min_default = float(runtime.get("slam_range_min_default", 0.12))
        range_max_default = float(runtime.get("slam_range_max_default", 8.0))
        reverse_mod = int(runtime.get("slam_fault_reverse_scan_mod", 41))
        truncate_mod = int(runtime.get("slam_fault_truncate_scan_mod", 59))

        msg.header.frame_id = frame_id
        now = time.time()
        msg.header.stamp.sec = int(now)
        msg.header.stamp.nanosec = int((now % 1.0) * 1000000000)

        if not math.isfinite(msg.angle_min):
            msg.angle_min = -math.pi
        if not math.isfinite(msg.angle_max):
            msg.angle_max = math.pi
        if msg.angle_max <= msg.angle_min:
            msg.angle_min = -math.pi
            msg.angle_max = math.pi

        if not math.isfinite(msg.angle_increment) or msg.angle_increment <= 0.0:
            msg.angle_increment = (msg.angle_max - msg.angle_min) / float(scan_len)

        if not math.isfinite(msg.time_increment) or msg.time_increment < 0.0:
            msg.time_increment = 0.0
        if not math.isfinite(msg.scan_time) or msg.scan_time < 0.0:
            msg.scan_time = 0.1

        if not math.isfinite(msg.range_min) or msg.range_min <= 0.0:
            msg.range_min = range_min_default
        if not math.isfinite(msg.range_max) or msg.range_max <= msg.range_min:
            msg.range_max = range_max_default

        if msg.ranges is None or len(msg.ranges) == 0:
            msg.ranges = [msg.range_max * 0.7] * scan_len
        else:
            msg.ranges = list(msg.ranges[:scan_len])
            if len(msg.ranges) < scan_len:
                msg.ranges.extend([msg.range_max * 0.7] * (scan_len - len(msg.ranges)))

        msg.intensities = [0.0] * len(msg.ranges)

        seed = (
            ((msg.header.stamp.sec & 0xFFFFFFFF) << 32)
            ^ (msg.header.stamp.nanosec & 0xFFFFFFFF)
        ) & 0xFFFFFFFFFFFFFFFF

        if reverse_mod > 0 and seed % reverse_mod == 0:
            msg.ranges = list(reversed(msg.ranges))
        elif truncate_mod > 0 and seed % truncate_mod == 0 and len(msg.ranges) > 8:
            msg.ranges = msg.ranges[:-8]
            msg.intensities = msg.intensities[:-8]

        return msg

    def post_exec_hook(self) -> None:
        return None

    def check_oracle(self, config, msg_list, state_dict, feedback_list):
        return oracles.slam_toolbox.check(
            config, msg_list, state_dict, feedback_list
        )
