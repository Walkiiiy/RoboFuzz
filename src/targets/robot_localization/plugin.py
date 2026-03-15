from __future__ import annotations

import math
import random

from target_plugins import BaseTargetPlugin


_MAX_REASONABLE_SPEED = 50.0
_MAX_COV_DIAG = 1.0e6
_MIN_QUAT_NORM = 1.0e-6
_MAX_QUAT_NORM_ERR = 0.2


class TargetPlugin(BaseTargetPlugin):
    """Oracle plugin for robot_localization (ekf/ukf)."""

    def pre_exec_hook(self, msg):
        # Strengthen attacks on EKF update path even when array index mutation
        # is limited by generic whitelist path handling.
        if type(msg).__name__ != "Imu":
            return msg

        # Quaternion singularity poisoning.
        if random.random() < 0.25:
            sel = random.randint(0, 2)
            if sel == 0:
                msg.orientation.x = 0.0
                msg.orientation.y = 0.0
                msg.orientation.z = 0.0
                msg.orientation.w = 0.0
            elif sel == 1:
                msg.orientation.x = random.uniform(-1000.0, 1000.0)
                msg.orientation.y = random.uniform(-1000.0, 1000.0)
                msg.orientation.z = random.uniform(-1000.0, 1000.0)
                msg.orientation.w = random.uniform(-1000.0, 1000.0)
            else:
                msg.orientation.x = float("nan")

        # Covariance poisoning for decomposition stability checks.
        def poison_cov(cov):
            if cov is None or len(cov) != 9:
                return cov
            arr = list(cov)
            mode = random.randint(0, 3)
            if mode == 0:
                # all-zero covariance
                arr = [0.0] * 9
            elif mode == 1:
                # negative variance on diagonal
                diag_idx = random.choice([0, 4, 8])
                arr[diag_idx] = -abs(random.uniform(1.0, 100.0))
            elif mode == 2:
                # asymmetry injection
                arr[1] = random.uniform(-100.0, 100.0)
                arr[3] = random.uniform(-100.0, 100.0)
            else:
                # explosive uncertainty
                for i in (0, 4, 8):
                    arr[i] = random.uniform(_MAX_COV_DIAG, _MAX_COV_DIAG * 100.0)
            return arr

        if random.random() < 0.35:
            msg.orientation_covariance = poison_cov(msg.orientation_covariance)
        if random.random() < 0.35:
            msg.angular_velocity_covariance = poison_cov(msg.angular_velocity_covariance)
        if random.random() < 0.35:
            msg.linear_acceleration_covariance = poison_cov(msg.linear_acceleration_covariance)

        # Time desynchronization poisoning.
        if random.random() < 0.15:
            if random.random() < 0.5:
                self._safe_set_stamp(msg, 0, 1)
            else:
                self._safe_set_stamp(msg, 2_147_483_647, 999_999_999)

        # Regardless of prior mutations, always normalize stamp to legal ROS2 Time range.
        self._normalize_stamp(msg)

        return msg

    def post_exec_hook(self) -> None:
        return None

    def check_oracle(self, config, msg_list, state_dict, feedback_list):
        errs = []

        odom_samples = state_dict.get("/odometry/filtered", [])
        if not odom_samples:
            errs.append("liveness: /odometry/filtered has no messages")
            return errs

        odom_samples = sorted(odom_samples, key=lambda x: x[0])
        prev_ts = None
        prev_msg = None

        for ts, msg in odom_samples:
            self._check_math_poison(errs, msg)
            self._check_covariance(errs, msg)
            self._check_quaternion(errs, msg)

            if prev_ts is not None and prev_msg is not None:
                self._check_kinematics(errs, prev_ts, prev_msg, ts, msg)

            prev_ts = ts
            prev_msg = msg

        # Optional diagnostics channel.
        for _, diag in state_dict.get("/diagnostics", []):
            for status in getattr(diag, "status", []):
                level = self._to_int(getattr(status, "level", 0), default=0)
                if level >= 2:
                    errs.append(
                        f"diagnostics: level={level} name={status.name} msg={status.message}"
                    )

        return list(set(errs))

    def _is_finite(self, x):
        return not (math.isnan(x) or math.isinf(x))

    def _safe_set_stamp(self, msg, sec, nanosec):
        try:
            msg.header.stamp.sec = int(sec)
            msg.header.stamp.nanosec = int(nanosec)
        except Exception:
            # Never let timestamp assignment crash the fuzz loop.
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0

    def _normalize_stamp(self, msg):
        sec_min = -2147483648
        sec_max = 2147483647
        nsec_min = 0
        nsec_max = 999999999
        try:
            sec = int(msg.header.stamp.sec)
        except Exception:
            sec = 0
        try:
            nsec = int(msg.header.stamp.nanosec)
        except Exception:
            nsec = 0

        if sec < sec_min:
            sec = sec_min
        elif sec > sec_max:
            sec = sec_max

        if nsec < nsec_min:
            nsec = nsec_min
        elif nsec > nsec_max:
            nsec = nsec_max

        self._safe_set_stamp(msg, sec, nsec)

    def _to_int(self, value, default=0):
        if isinstance(value, int):
            return value
        if isinstance(value, bytes):
            if len(value) == 0:
                return default
            if len(value) == 1:
                return value[0]
            try:
                value = value.decode("utf-8", errors="ignore").strip()
            except Exception:
                return default
        if isinstance(value, str):
            value = value.strip()
            if value == "":
                return default
            try:
                return int(value, 10)
            except ValueError:
                return default
        try:
            return int(value)
        except Exception:
            return default

    def _check_math_poison(self, errs, msg):
        vals = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ]
        if any(not self._is_finite(v) for v in vals):
            errs.append("math_poison: NaN/INF detected in /odometry/filtered")

    def _check_covariance(self, errs, msg):
        def check_cov(name, cov):
            if cov is None or len(cov) != 36:
                errs.append(f"covariance: {name} length invalid")
                return

            for i, v in enumerate(cov):
                if not self._is_finite(v):
                    errs.append(f"covariance: {name}[{i}] is NaN/INF")

            for i in (0, 7, 14, 21, 28, 35):
                v = cov[i]
                if v < 0:
                    errs.append(f"covariance: {name}[{i}] negative variance {v}")
                if v > _MAX_COV_DIAG:
                    errs.append(f"covariance: {name}[{i}] exploded {v}")

        check_cov("pose", msg.pose.covariance)
        check_cov("twist", msg.twist.covariance)

    def _check_quaternion(self, errs, msg):
        q = msg.pose.pose.orientation
        norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if not self._is_finite(norm):
            errs.append("quaternion: norm is NaN/INF")
            return
        if norm < _MIN_QUAT_NORM:
            errs.append(f"quaternion: near-zero norm {norm}")
            return
        if abs(norm - 1.0) > _MAX_QUAT_NORM_ERR:
            errs.append(f"quaternion: non-normalized norm {norm}")

    def _check_kinematics(self, errs, prev_ts, prev_msg, cur_ts, cur_msg):
        dt = (cur_ts - prev_ts) / 1e9
        if dt <= 0:
            errs.append(f"time_desync: non-positive dt {dt}")
            return

        dx = cur_msg.pose.pose.position.x - prev_msg.pose.pose.position.x
        dy = cur_msg.pose.pose.position.y - prev_msg.pose.pose.position.y
        dz = cur_msg.pose.pose.position.z - prev_msg.pose.pose.position.z

        ds = math.sqrt(dx * dx + dy * dy + dz * dz)
        v = ds / dt
        if self._is_finite(v) and v > _MAX_REASONABLE_SPEED:
            errs.append(f"teleportation: speed {v} m/s exceeds limit {_MAX_REASONABLE_SPEED}")
