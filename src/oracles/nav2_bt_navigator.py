import math


try:
    from action_msgs.msg import GoalStatus
except Exception:
    GoalStatus = None


def _is_finite(val):
    return not (math.isnan(val) or math.isinf(val))


def _status_name(code):
    names = {
        0: "UNKNOWN",
        1: "ACCEPTED",
        2: "EXECUTING",
        3: "CANCELING",
        4: "SUCCEEDED",
        5: "CANCELED",
        6: "ABORTED",
    }
    return names.get(code, f"INVALID({code})")


def _max_abs_twist(cmd):
    vals = [
        cmd.linear.x,
        cmd.linear.y,
        cmd.linear.z,
        cmd.angular.x,
        cmd.angular.y,
        cmd.angular.z,
    ]
    return max(abs(v) for v in vals)


def _extract_latest_pose(state_dict):
    poses = state_dict.get("/amcl_pose", [])
    if not poses:
        return None
    return poses[-1][1].pose.pose


def check(config, msg_list, state_dict, feedback_list):
    errs = []

    goal_msg = msg_list[0] if msg_list else None
    goal_pose = getattr(goal_msg, "pose", None)

    cmd_vel_msgs = list(state_dict.get("/cmd_vel", []))
    plan_msgs = list(state_dict.get("/plan", []))
    feedback_msgs = list(state_dict.get("/navigate_to_pose/_action/feedback", []))
    status_msgs = list(state_dict.get("/navigate_to_pose/_action/status", []))
    diag_msgs = list(state_dict.get("/diagnostics", []))

    max_linear = float(getattr(config, "nav2_bt_max_linear_speed", 1.5))
    max_angular = float(getattr(config, "nav2_bt_max_angular_speed", 2.5))
    goal_tolerance = float(getattr(config, "nav2_bt_goal_tolerance", 0.75))
    max_recoveries = int(getattr(config, "nav2_bt_max_recoveries", 5))

    if not status_msgs and not feedback_msgs and not cmd_vel_msgs and not plan_msgs:
        errs.append("pipeline: no navigation activity captured")
        return errs

    latest_status_codes = []
    for _, msg in status_msgs:
        for status in getattr(msg, "status_list", []):
            code = int(getattr(status, "status", -1))
            latest_status_codes.append(code)
            if code < 0 or code > 6:
                errs.append(f"navigate_to_pose status invalid: {code}")

    for _, msg in cmd_vel_msgs:
        twist = getattr(msg, "twist", msg)
        vals = [
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        ]
        if not all(_is_finite(v) for v in vals):
            errs.append("/cmd_vel contains NaN/INF")
            continue
        if abs(twist.linear.x) > max_linear:
            errs.append(f"/cmd_vel linear.x too large: {twist.linear.x}")
        if abs(twist.angular.z) > max_angular:
            errs.append(f"/cmd_vel angular.z too large: {twist.angular.z}")

    saw_empty_plan = False
    for _, msg in plan_msgs:
        poses = list(getattr(msg, "poses", []) or [])
        if len(poses) == 0:
            saw_empty_plan = True
        for i, pose_stamped in enumerate(poses[:8]):
            pos = pose_stamped.pose.position
            ori = pose_stamped.pose.orientation
            vals = [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]
            if not all(_is_finite(v) for v in vals):
                errs.append(f"/plan pose[{i}] contains NaN/INF")
                break

    if saw_empty_plan and latest_status_codes:
        if any(code in (1, 2) for code in latest_status_codes):
            errs.append("planner returned empty plan while goal remained active")

    recovery_counts = []
    for _, msg in feedback_msgs:
        feedback = getattr(msg, "feedback", None)
        if feedback is None:
            continue

        current_pose = getattr(feedback, "current_pose", None)
        if current_pose is not None:
            pos = current_pose.pose.position
            ori = current_pose.pose.orientation
            vals = [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]
            if not all(_is_finite(v) for v in vals):
                errs.append("navigate_to_pose feedback current_pose contains NaN/INF")

        dist = getattr(feedback, "distance_remaining", None)
        if isinstance(dist, float) and not _is_finite(dist):
            errs.append("navigate_to_pose feedback distance_remaining is NaN/INF")

        num_recoveries = getattr(feedback, "number_of_recoveries", None)
        if isinstance(num_recoveries, int):
            recovery_counts.append(num_recoveries)

    if recovery_counts and max(recovery_counts) > max_recoveries:
        errs.append(
            f"navigation triggered too many recoveries: {max(recovery_counts)}"
        )

    if goal_pose is not None and latest_status_codes:
        latest_pose = _extract_latest_pose(state_dict)
        if latest_pose is not None:
            dx = latest_pose.position.x - goal_pose.position.x
            dy = latest_pose.position.y - goal_pose.position.y
            dist = math.hypot(dx, dy)
            if 4 in latest_status_codes and dist > goal_tolerance:
                errs.append(
                    f"navigate_to_pose succeeded but final pose error too large: {dist}"
                )

    if latest_status_codes and all(code in (5, 6) for code in latest_status_codes):
        if cmd_vel_msgs:
            max_cmd = max(_max_abs_twist(getattr(msg, "twist", msg)) for _, msg in cmd_vel_msgs)
            if max_cmd > 0.05:
                errs.append(
                    "navigation terminated with canceled/aborted status but kept publishing cmd_vel"
                )

    for _, msg in diag_msgs:
        for status in getattr(msg, "status", []):
            if getattr(status, "level", 0) >= 2:
                errs.append(
                    f"diagnostics error: {status.name} ({status.message})"
                )

    for feedback in feedback_list:
        if feedback.name == "nav_status_seen":
            feedback.update_value(1.0 if latest_status_codes else 0.0)
        elif feedback.name == "nav_cmd_vel_seen":
            feedback.update_value(float(len(cmd_vel_msgs)))

    return list(set(errs))
