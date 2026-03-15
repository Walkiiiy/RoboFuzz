import re
from typing import List, Tuple

from .schemas import CaseBundle


PATTERN_RULES: List[Tuple[str, str]] = [
    (r"UnsupportedTypeSupport|Could not import 'rosidl_typesupport_c'", "INFRA_TYPE_SUPPORT"),
    (r"publish failed", "INFRA_PUBLISH_FAIL"),
    (r"watch failed: no messages captured", "OBS_EMPTY_CAPTURE"),
    (r"liveness: .* no messages", "LIVENESS_NO_OUTPUT"),
    (r"pipeline: neither .* captured", "OBS_PIPELINE_EMPTY"),
    (r"diagnostics: level=", "DIAGNOSTIC_LEVEL_ERROR"),
    (r"covariance: .* exploded", "NUMERIC_COVARIANCE_EXPLOSION"),
    (r"NaN|INF", "NUMERIC_NAN_INF"),
    (r"quaternion:", "QUATERNION_INVALID"),
    (r"time_desync", "TIME_DESYNC"),
    (r"teleportation:", "STATE_TELEPORTATION"),
    (r"tf missing map->odom transform", "TF_MISSING_EDGE"),
    (r"action doesn't|invalid goal action status", "ACTION_STATUS_INVALID"),
    (r"Motion plan request", "MOTION_PLAN_COUNT_INVALID"),
    (r"Topic .* is lost", "TOPIC_LOST"),
    (r"Sent and replayed .* do not match", "REPLAY_MISMATCH"),
]


def normalize_case(bundle: CaseBundle):
    errors = bundle.error_payload.get("errors", [])
    unique_errors = []
    seen = set()
    for err in errors:
        text = str(err)
        if text in seen:
            continue
        seen.add(text)
        unique_errors.append(text)

    matched = []
    for err in unique_errors:
        for pattern, label in PATTERN_RULES:
            if re.search(pattern, err):
                matched.append(label)

    matched = sorted(set(matched))

    return {
        "unique_errors": unique_errors,
        "matched_patterns": matched,
        "diagnostics_max_level": bundle.diagnostics_summary.get("max_level", 0),
        "publish_succeeded": bundle.execution.get("publish_succeeded"),
        "target_started": bundle.execution.get("target_started"),
        "empty_capture": bundle.observation_summary.get("empty_capture"),
        "contains_nan": bundle.input_summary.get("contains_nan"),
        "contains_inf": bundle.input_summary.get("contains_inf"),
        "contains_extreme_numeric": bundle.input_summary.get(
            "contains_extreme_numeric"
        ),
    }
