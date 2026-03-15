from __future__ import annotations

import time

from target_plugins import BaseTargetPlugin

import oracles.robot_state_publisher


def _next_seed(seed):
    return (seed * 6364136223846793005 + 1442695040888963407) & 0xFFFFFFFFFFFFFFFF


def _signed_unit(seed):
    return ((seed / float(0xFFFFFFFFFFFFFFFF)) * 2.0) - 1.0


class TargetPlugin(BaseTargetPlugin):
    def pre_exec_hook(self, msg):
        runtime = self.target_config.get("runtime", {})
        joint_names = list(runtime.get("rsp_joint_names", ["joint1", "joint2"]))
        default_positions = list(
            runtime.get("rsp_default_positions", [0.0] * len(joint_names))
        )
        position_limits = list(runtime.get("rsp_position_limits", []))
        delta = float(runtime.get("rsp_position_mutation_delta", 0.12))
        vel_limit = float(runtime.get("rsp_velocity_mutation_limit", 0.25))
        normalize_stamp = bool(runtime.get("rsp_normalize_stamp", True))
        duplicate_mod = int(runtime.get("rsp_fault_duplicate_mod", 41))
        unknown_name_mod = int(runtime.get("rsp_fault_unknown_name_mod", 59))
        truncate_mod = int(runtime.get("rsp_fault_truncate_position_mod", 73))

        original_sec = int(msg.header.stamp.sec)
        original_nanosec = int(msg.header.stamp.nanosec)
        seed = (
            ((original_sec & 0xFFFFFFFF) << 32)
            ^ (original_nanosec & 0xFFFFFFFF)
            ^ 0x9E3779B97F4A7C15
        ) & 0xFFFFFFFFFFFFFFFF

        msg.name = list(joint_names)
        msg.position = list(default_positions[: len(joint_names)])
        msg.velocity = [0.0] * len(joint_names)
        msg.effort = [0.0] * len(joint_names)

        for idx in range(len(joint_names)):
            seed = _next_seed(seed)
            pos_delta = _signed_unit(seed) * delta
            seed = _next_seed(seed)
            vel_delta = _signed_unit(seed) * vel_limit

            msg.position[idx] += pos_delta
            msg.velocity[idx] = vel_delta

            if idx < len(position_limits) and len(position_limits[idx]) == 2:
                lo = float(position_limits[idx][0])
                hi = float(position_limits[idx][1])
                msg.position[idx] = min(max(msg.position[idx], lo), hi)

        fault_selector = seed
        if len(msg.name) >= 2 and duplicate_mod > 0 and (fault_selector % duplicate_mod) == 0:
            msg.name[1] = msg.name[0]
        elif unknown_name_mod > 0 and (fault_selector % unknown_name_mod) == 0:
            msg.name[-1] = "joint_unknown"
        elif truncate_mod > 0 and (fault_selector % truncate_mod) == 0 and len(msg.position) > 1:
            msg.position = msg.position[:-1]
            msg.velocity = msg.velocity[:-1]
            msg.effort = msg.effort[:-1]

        if normalize_stamp:
            now = time.time()
            msg.header.stamp.sec = int(now)
            msg.header.stamp.nanosec = int((now % 1.0) * 1000000000)

        msg.header.frame_id = ""
        return msg

    def post_exec_hook(self) -> None:
        return None

    def check_oracle(self, config, msg_list, state_dict, feedback_list):
        return oracles.robot_state_publisher.check(
            config, msg_list, state_dict, feedback_list
        )
