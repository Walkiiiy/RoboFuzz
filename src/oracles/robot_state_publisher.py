import math


def _is_finite(x):
    return not (math.isnan(x) or math.isinf(x))


def _quat_norm(rot):
    return (
        rot.x * rot.x
        + rot.y * rot.y
        + rot.z * rot.z
        + rot.w * rot.w
    )


def check(config, msg_list, state_dict, feedback_list):
    errs = []

    tf_samples = list(state_dict.get("/tf", []))
    tf_static_samples = list(state_dict.get("/tf_static", []))

    if not tf_samples and not tf_static_samples:
        errs.append("pipeline: neither /tf nor /tf_static captured")
        return errs

    expected_frames = set(getattr(config, "rsp_expected_frames", []) or [])
    max_translation = float(getattr(config, "rsp_max_translation", 10.0))
    max_frame_jump = float(getattr(config, "rsp_max_frame_jump", 2.5))

    seen_frame_pairs = set()
    dynamic_transforms = []
    prev_by_child = {}
    meaningful_transform_count = 0

    for topic_name, samples in (("/tf", tf_samples), ("/tf_static", tf_static_samples)):
        for _, tf_msg in samples:
            transforms = list(getattr(tf_msg, "transforms", []) or [])
            if not transforms:
                continue

            for transform in transforms:
                meaningful_transform_count += 1
                parent = getattr(transform.header, "frame_id", "")
                child = getattr(transform, "child_frame_id", "")
                trans = transform.transform.translation
                rot = transform.transform.rotation

                if not parent:
                    errs.append(f"{topic_name}: empty frame_id")
                if not child:
                    errs.append(f"{topic_name}: empty child_frame_id")
                if parent == child and parent:
                    errs.append(f"{topic_name}: self transform {parent}->{child}")

                seen_frame_pairs.add((parent, child))

                vals = [
                    trans.x,
                    trans.y,
                    trans.z,
                    rot.x,
                    rot.y,
                    rot.z,
                    rot.w,
                ]
                if not all(_is_finite(v) for v in vals):
                    errs.append(f"{topic_name}: transform {parent}->{child} has NaN/INF")
                    continue

                quat_norm = _quat_norm(rot)
                if quat_norm == 0.0 or not _is_finite(quat_norm):
                    errs.append(f"{topic_name}: transform {parent}->{child} quaternion norm invalid")
                elif abs(quat_norm - 1.0) > 0.2:
                    errs.append(
                        f"{topic_name}: transform {parent}->{child} quaternion not normalized ({quat_norm})"
                    )

                max_abs_translation = max(abs(trans.x), abs(trans.y), abs(trans.z))
                if max_abs_translation > max_translation:
                    errs.append(
                        f"{topic_name}: transform {parent}->{child} translation too large ({max_abs_translation})"
                    )

                if topic_name == "/tf":
                    dynamic_transforms.append((child, (trans.x, trans.y, trans.z)))

    if meaningful_transform_count == 0:
        errs.append("pipeline: no effective transforms captured")
        return errs

    if expected_frames:
        seen_children = {child for _, child in seen_frame_pairs if child}
        missing = sorted(expected_frames - seen_children)
        if missing:
            errs.append(f"tf coverage missing expected child frames: {missing}")

    for child, cur_pos in dynamic_transforms:
        if child in prev_by_child:
            prev_pos = prev_by_child[child]
            dx = cur_pos[0] - prev_pos[0]
            dy = cur_pos[1] - prev_pos[1]
            dz = cur_pos[2] - prev_pos[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist > max_frame_jump:
                errs.append(f"tf jump too large for {child}: {dist}")
        prev_by_child[child] = cur_pos

    for feedback in feedback_list:
        if feedback.name == "tf_liveness":
            feedback.update_value(0.0 if tf_samples else 1.0)
        elif feedback.name == "tf_pair_count":
            feedback.update_value(float(len(seen_frame_pairs)))

    return list(set(errs))
