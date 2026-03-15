import math


def _is_finite(val):
    return not (math.isnan(val) or math.isinf(val))


def check(config, msg_list, state_dict, feedback_list):
    errs = []

    map_msgs = list(state_dict.get("/map", []))
    tf_msgs = list(state_dict.get("/tf", []))
    diag_msgs = list(state_dict.get("/diagnostics", []))

    if not map_msgs and not tf_msgs:
        errs.append("pipeline: no slam outputs captured")
        return errs
    if not map_msgs:
        errs.append("pipeline: no /map captured")
        return list(set(errs))

    max_resolution = float(getattr(config, "slam_max_map_resolution", 2.0))
    min_resolution = float(getattr(config, "slam_min_map_resolution", 0.001))
    max_dim = int(getattr(config, "slam_max_map_dim", 4096))

    saw_effective_map = False
    tf_has_map_odom = False
    latest_known_cells = 0

    for _, msg in map_msgs:
        info = msg.info
        res = float(info.resolution)
        width = int(info.width)
        height = int(info.height)

        if not _is_finite(res):
            errs.append("/map resolution is NaN/INF")
            continue
        if res < min_resolution or res > max_resolution:
            errs.append(f"/map resolution out of range: {res}")

        if width <= 0 or height <= 0:
            errs.append(f"/map invalid dimensions: {width}x{height}")
            continue
        if width > max_dim or height > max_dim:
            errs.append(f"/map dimensions too large: {width}x{height}")

        expected = width * height
        actual = len(msg.data)
        if actual != expected:
            errs.append(f"/map data length mismatch: expected {expected}, got {actual}")
            continue

        known_cells = sum(1 for cell in msg.data if cell != -1)
        occupied_cells = sum(1 for cell in msg.data if cell > 50)
        latest_known_cells = known_cells

        if known_cells > 0:
            saw_effective_map = True
        if occupied_cells > expected:
            errs.append("/map occupied cell count invalid")

        origin = info.origin
        vals = [
            origin.position.x,
            origin.position.y,
            origin.position.z,
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        ]
        if not all(_is_finite(v) for v in vals):
            errs.append("/map origin contains NaN/INF")

    for _, msg in tf_msgs:
        for transform in getattr(msg, "transforms", []):
            parent = transform.header.frame_id
            child = transform.child_frame_id
            if parent == "map" and child == "odom":
                tf_has_map_odom = True
            vals = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
            if not all(_is_finite(v) for v in vals):
                errs.append(f"/tf transform {parent}->{child} has NaN/INF")

    if saw_effective_map and tf_msgs and not tf_has_map_odom:
        errs.append("tf missing map->odom transform")

    if map_msgs and not saw_effective_map:
        errs.append("/map never contained any known cells")

    for _, msg in diag_msgs:
        for status in getattr(msg, "status", []):
            if getattr(status, "level", 0) >= 2:
                errs.append(f"diagnostics error: {status.name} ({status.message})")

    for feedback in feedback_list:
        if feedback.name == "slam_map_seen":
            feedback.update_value(1.0 if map_msgs else 0.0)
        elif feedback.name == "slam_known_cells":
            feedback.update_value(float(latest_known_cells))

    return list(set(errs))
