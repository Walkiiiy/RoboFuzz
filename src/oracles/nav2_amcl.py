import math


def _is_finite(val):
    return not (math.isnan(val) or math.isinf(val))


def _check_quaternion(errs, prefix, x, y, z, w):
    if not (_is_finite(x) and _is_finite(y) and _is_finite(z) and _is_finite(w)):
        errs.append(f"{prefix}.orientation has NaN/INF")
        return

    norm = x * x + y * y + z * z + w * w
    if not _is_finite(norm) or norm == 0.0:
        errs.append(f"{prefix}.orientation norm is invalid")


def _check_position(errs, prefix, pos):
    if not _is_finite(pos.x):
        errs.append(f"{prefix}.position.x is NaN/INF")
    if not _is_finite(pos.y):
        errs.append(f"{prefix}.position.y is NaN/INF")
    if not _is_finite(pos.z):
        errs.append(f"{prefix}.position.z is NaN/INF")


def _check_pose(errs, prefix, pose):
    _check_position(errs, prefix, pose.position)
    _check_quaternion(
        errs, prefix, pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w
    )


def _check_covariance(errs, prefix, cov):
    for idx, val in enumerate(cov):
        if not _is_finite(val):
            errs.append(f"{prefix}.covariance[{idx}] is NaN/INF")


def check(config, msg_list, state_dict, feedback_list):
    errs = []
    last_pose = None
    last_ts = None
    max_jump = 2.0
    max_cov = 1.0e6

    for (ts, msg) in state_dict.get("/amcl_pose", []):
        _check_pose(errs, "amcl_pose.pose.pose", msg.pose.pose)
        _check_covariance(errs, "amcl_pose.pose", msg.pose.covariance)
        cov = msg.pose.covariance
        for i in (0, 7, 14, 21, 28, 35):
            if i < len(cov) and cov[i] > max_cov:
                errs.append(f"amcl_pose.covariance[{i}] too large: {cov[i]}")

        if last_pose is not None:
            dx = msg.pose.pose.position.x - last_pose.position.x
            dy = msg.pose.pose.position.y - last_pose.position.y
            dist = math.hypot(dx, dy)
            if last_ts is not None and dist > max_jump:
                errs.append(f"amcl_pose jump too large: {dist}")
        last_pose = msg.pose.pose
        last_ts = ts

    for (ts, msg) in state_dict.get("/particlecloud", []):
        if len(msg.poses) == 0:
            errs.append("particlecloud is empty")
        for i, pose in enumerate(msg.poses):
            _check_pose(errs, f"particlecloud.poses[{i}]", pose)

    tf_has_map = False
    for (ts, msg) in state_dict.get("/tf", []):
        for i, transform in enumerate(msg.transforms):
            if (
                transform.header.frame_id == "map"
                and transform.child_frame_id == "odom"
            ):
                tf_has_map = True
            _check_position(errs, f"tf.transforms[{i}].transform", transform.transform.translation)
            _check_quaternion(
                errs,
                f"tf.transforms[{i}].transform",
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )

    if state_dict.get("/tf", []) and not tf_has_map:
        errs.append("tf missing map->odom transform")

    for (ts, msg) in state_dict.get("/diagnostics", []):
        for status in msg.status:
            if status.level >= 2:
                errs.append(f"diagnostics error: {status.name} ({status.message})")

    return errs
