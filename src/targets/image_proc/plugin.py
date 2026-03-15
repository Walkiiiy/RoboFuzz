from __future__ import annotations

import os
import pickle
import subprocess as sp
import tempfile
import time

from target_plugins import BaseTargetPlugin

import oracles.image_proc


def _next_seed(seed):
    return (seed * 6364136223846793005 + 1442695040888963407) & 0xFFFFFFFFFFFFFFFF


class TargetPlugin(BaseTargetPlugin):
    def pre_exec_hook(self, msg):
        runtime = self.target_config.get("runtime", {})
        base_width = int(runtime.get("image_proc_width", 64))
        base_height = int(runtime.get("image_proc_height", 48))
        encoding = str(runtime.get("image_proc_encoding", "mono8"))
        frame_id = str(runtime.get("image_proc_frame_id", "camera_optical_frame"))
        dim_delta = max(1, int(runtime.get("image_proc_dim_delta", 8)))
        fault_dim_mod = int(runtime.get("image_proc_fault_dim_mod", 43))
        fault_flip_mod = int(runtime.get("image_proc_fault_flip_mod", 61))

        original_sec = int(msg.header.stamp.sec)
        original_nanosec = int(msg.header.stamp.nanosec)
        seed = (
            ((original_sec & 0xFFFFFFFF) << 32)
            ^ (original_nanosec & 0xFFFFFFFF)
            ^ int(msg.width)
            ^ (int(msg.height) << 16)
            ^ (int(msg.step) << 8)
            ^ int(msg.is_bigendian)
        ) & 0xFFFFFFFFFFFFFFFF

        width = base_width + int(seed % dim_delta)
        seed = _next_seed(seed)
        height = base_height + int(seed % dim_delta)
        seed = _next_seed(seed)

        if fault_dim_mod > 0 and (seed % fault_dim_mod) == 0:
            width = max(16, width - 4)
        if fault_flip_mod > 0 and (seed % fault_flip_mod) == 0:
            width, height = height, width

        step = width
        total = width * height
        data = bytearray(total)
        local_seed = seed
        for idx in range(total):
            local_seed = _next_seed(local_seed)
            data[idx] = local_seed & 0xFF

        now = time.time()
        msg.header.stamp.sec = int(now)
        msg.header.stamp.nanosec = int((now % 1.0) * 1000000000)
        msg.header.frame_id = frame_id
        msg.height = height
        msg.width = width
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = step
        msg.data = list(data)
        return msg

    def publish_message(self, msg):
        payload = {
            "header_sec": int(msg.header.stamp.sec),
            "header_nanosec": int(msg.header.stamp.nanosec),
            "frame_id": str(msg.header.frame_id),
            "height": int(msg.height),
            "width": int(msg.width),
            "encoding": str(msg.encoding),
            "is_bigendian": int(msg.is_bigendian),
            "step": int(msg.step),
            "data": bytes(msg.data),
        }

        helper = os.path.join(self.target_config["target_dir"], "publish_pair.py")
        with tempfile.NamedTemporaryFile("wb", delete=False) as tmp:
            pickle.dump(payload, tmp)
            payload_path = tmp.name

        try:
            proc = sp.run(
                ["/usr/bin/python3", helper, payload_path],
                cwd=self.target_config["target_dir"],
                capture_output=True,
                text=True,
            )
        finally:
            try:
                os.unlink(payload_path)
            except OSError:
                pass

        if proc.returncode != 0:
            stderr = (proc.stderr or "").strip()
            stdout = (proc.stdout or "").strip()
            detail = stderr or stdout or "unknown error"
            raise RuntimeError(
                f"image_proc publish failed (code={proc.returncode}): {detail}"
            )

    def post_exec_hook(self) -> None:
        return None

    def check_oracle(self, config, msg_list, state_dict, feedback_list):
        return oracles.image_proc.check(config, msg_list, state_dict, feedback_list)
