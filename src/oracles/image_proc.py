import math


def _is_finite(val):
    return not (math.isnan(val) or math.isinf(val))


def _bytes_per_pixel(encoding):
    if encoding in {"mono8", "8UC1"}:
        return 1
    if encoding in {"rgb8", "bgr8", "8UC3"}:
        return 3
    if encoding in {"rgba8", "bgra8", "8UC4"}:
        return 4
    return None


def check(config, msg_list, state_dict, feedback_list):
    errs = []

    rect_samples = list(state_dict.get("/camera/image_rect", []))
    diag_msgs = list(state_dict.get("/diagnostics", []))

    if not rect_samples:
        errs.append("pipeline: no rectified image captured")
        return errs

    input_msg = msg_list[0] if msg_list else None
    expected_width = int(getattr(input_msg, "width", 0) or 0)
    expected_height = int(getattr(input_msg, "height", 0) or 0)
    expected_encoding = str(getattr(input_msg, "encoding", "") or "")
    expected_bpp = _bytes_per_pixel(expected_encoding)

    max_pixels = int(getattr(config, "image_proc_max_pixels", 640 * 480))
    latest_payload = 0

    for _, msg in rect_samples:
        width = int(getattr(msg, "width", 0) or 0)
        height = int(getattr(msg, "height", 0) or 0)
        step = int(getattr(msg, "step", 0) or 0)
        encoding = str(getattr(msg, "encoding", "") or "")
        data = list(getattr(msg, "data", []) or [])
        latest_payload = len(data)

        if width <= 0 or height <= 0:
            errs.append(f"/camera/image_rect invalid dimensions: {width}x{height}")
            continue

        if width * height > max_pixels:
            errs.append(f"/camera/image_rect dimensions too large: {width}x{height}")

        if not encoding:
            errs.append("/camera/image_rect empty encoding")

        bpp = _bytes_per_pixel(encoding)
        if bpp is None:
            errs.append(f"/camera/image_rect unsupported encoding: {encoding}")
            continue

        min_step = width * bpp
        if step < min_step:
            errs.append(
                f"/camera/image_rect step too small: step={step}, min={min_step}"
            )

        expected_len = height * step
        if len(data) != expected_len:
            errs.append(
                f"/camera/image_rect data length mismatch: expected {expected_len}, got {len(data)}"
            )

        if expected_width > 0 and width != expected_width:
            errs.append(
                f"/camera/image_rect width mismatch: expected {expected_width}, got {width}"
            )
        if expected_height > 0 and height != expected_height:
            errs.append(
                f"/camera/image_rect height mismatch: expected {expected_height}, got {height}"
            )
        if expected_encoding and encoding != expected_encoding:
            errs.append(
                f"/camera/image_rect encoding mismatch: expected {expected_encoding}, got {encoding}"
            )
        if expected_bpp is not None and bpp != expected_bpp:
            errs.append(
                f"/camera/image_rect pixel stride mismatch: expected {expected_bpp}, got {bpp}"
            )

        stamp = getattr(msg, "header", None)
        if stamp is not None:
            sec = getattr(stamp.stamp, "sec", 0)
            nanosec = getattr(stamp.stamp, "nanosec", 0)
            if not _is_finite(float(sec)) or not _is_finite(float(nanosec)):
                errs.append("/camera/image_rect timestamp contains NaN/INF")

    for _, msg in diag_msgs:
        for status in getattr(msg, "status", []):
            if getattr(status, "level", 0) >= 2:
                errs.append(f"diagnostics error: {status.name} ({status.message})")

    for feedback in feedback_list:
        if feedback.name == "rect_image_seen":
            feedback.update_value(1.0 if rect_samples else 0.0)
        elif feedback.name == "rect_payload_size":
            feedback.update_value(float(latest_payload))

    return list(set(errs))
