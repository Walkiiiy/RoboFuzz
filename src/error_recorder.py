import ast
import glob
import json
import math
import os
from typing import Any, Dict, List

from rosidl_runtime_py import message_to_ordereddict


_MAX_PREVIEW_ITEMS = 8
_MAX_TEXT_LEN = 200
_EXTREME_NUMERIC_THRESHOLD = 1.0e6


def _truncate_text(value: Any, limit: int = _MAX_TEXT_LEN) -> str:
    text = str(value)
    if len(text) <= limit:
        return text
    return text[: limit - 3] + "..."


def _json_safe(value: Any):
    if value is None or isinstance(value, (bool, int, str)):
        return value

    if isinstance(value, float):
        if math.isnan(value):
            return "NaN"
        if math.isinf(value):
            return "INF" if value > 0 else "-INF"
        return value

    if isinstance(value, bytes):
        return _truncate_text(value.decode("utf-8", errors="ignore"))

    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            out[str(key)] = _json_safe(item)
        return out

    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]

    return _truncate_text(repr(value))


def _walk_scalars(value: Any, prefix: str, out: List[tuple]):
    if isinstance(value, dict):
        for key, item in value.items():
            child = f"{prefix}.{key}" if prefix else str(key)
            _walk_scalars(item, child, out)
        return

    if isinstance(value, list):
        for idx, item in enumerate(value[:_MAX_PREVIEW_ITEMS]):
            child = f"{prefix}[{idx}]"
            _walk_scalars(item, child, out)
        return

    out.append((prefix, value))


def _detect_numeric_flags(msg_dict: Dict[str, Any]) -> Dict[str, Any]:
    scalar_items = []
    _walk_scalars(msg_dict, "", scalar_items)

    nan_paths = []
    inf_paths = []
    extreme_paths = []

    for path, value in scalar_items:
        if isinstance(value, float):
            if math.isnan(value):
                nan_paths.append(path)
            elif math.isinf(value):
                inf_paths.append(path)
            elif abs(value) >= _EXTREME_NUMERIC_THRESHOLD:
                extreme_paths.append(path)
        elif isinstance(value, int) and abs(value) >= _EXTREME_NUMERIC_THRESHOLD:
            extreme_paths.append(path)

    return {
        "contains_nan": len(nan_paths) > 0,
        "contains_inf": len(inf_paths) > 0,
        "contains_extreme_numeric": len(extreme_paths) > 0,
        "nan_paths": nan_paths[:_MAX_PREVIEW_ITEMS],
        "inf_paths": inf_paths[:_MAX_PREVIEW_ITEMS],
        "extreme_numeric_paths": extreme_paths[:_MAX_PREVIEW_ITEMS],
    }


def _detect_time_flags(msg_dict: Dict[str, Any]) -> Dict[str, Any]:
    header = msg_dict.get("header")
    if not isinstance(header, dict):
        return {
            "contains_invalid_time": False,
            "time_fields": {},
        }

    stamp = header.get("stamp")
    if not isinstance(stamp, dict):
        return {
            "contains_invalid_time": False,
            "time_fields": {},
        }

    sec = stamp.get("sec")
    nanosec = stamp.get("nanosec")
    invalid = False
    if isinstance(nanosec, int) and (nanosec < 0 or nanosec > 999999999):
        invalid = True

    return {
        "contains_invalid_time": invalid,
        "time_fields": {
            "header.stamp.sec": _json_safe(sec),
            "header.stamp.nanosec": _json_safe(nanosec),
        },
    }


def summarize_message(msg: Any) -> Dict[str, Any]:
    try:
        msg_dict = message_to_ordereddict(msg)
    except Exception:
        msg_dict = {"repr": repr(msg)}

    msg_dict = _json_safe(msg_dict)
    numeric_flags = _detect_numeric_flags(msg_dict if isinstance(msg_dict, dict) else {})
    time_flags = _detect_time_flags(msg_dict if isinstance(msg_dict, dict) else {})

    preview = msg_dict
    if isinstance(msg_dict, dict):
        preview = {}
        for idx, (key, value) in enumerate(msg_dict.items()):
            if idx >= _MAX_PREVIEW_ITEMS:
                preview["__truncated__"] = True
                break
            preview[key] = value

    return {
        "message_type": type(msg).__name__,
        "message_preview": preview,
        **numeric_flags,
        **time_flags,
    }


def summarize_input_messages(msg_list: List[Any], mode: str = "single") -> Dict[str, Any]:
    summaries = [summarize_message(msg) for msg in msg_list]
    return {
        "message_kind": mode,
        "message_count": len(msg_list),
        "message_types": [type(msg).__name__ for msg in msg_list],
        "contains_nan": any(item["contains_nan"] for item in summaries),
        "contains_inf": any(item["contains_inf"] for item in summaries),
        "contains_extreme_numeric": any(
            item["contains_extreme_numeric"] for item in summaries
        ),
        "contains_invalid_time": any(
            item["contains_invalid_time"] for item in summaries
        ),
        "messages": summaries[:_MAX_PREVIEW_ITEMS],
    }


def summarize_state_dict(state_dict: Dict[str, List[Any]]) -> Dict[str, Any]:
    topics = {}
    diagnostics = []
    max_level = 0

    for topic, samples in state_dict.items():
        count = len(samples)
        msg_type = None
        if count > 0:
            try:
                msg_type = type(samples[0][1]).__name__
            except Exception:
                msg_type = None

        topics[topic] = {
            "message_count": count,
            "message_type": msg_type,
        }

        if topic == "/diagnostics":
            for _, diag_msg in samples:
                for status in getattr(diag_msg, "status", []):
                    try:
                        level = int(getattr(status, "level", 0))
                    except Exception:
                        level = 0
                    max_level = max(max_level, level)
                    diagnostics.append(
                        {
                            "level": level,
                            "name": _truncate_text(getattr(status, "name", "")),
                            "message": _truncate_text(
                                getattr(status, "message", "")
                            ),
                        }
                    )

    return {
        "watch_topics": topics,
        "topic_count": len(topics),
        "empty_capture": all(item["message_count"] == 0 for item in topics.values())
        if topics
        else True,
        "diagnostics_summary": {
            "max_level": max_level,
            "status_count": len(diagnostics),
            "errors": diagnostics[:_MAX_PREVIEW_ITEMS],
        },
    }


def build_runtime_context(config, fuzzer=None) -> Dict[str, Any]:
    target_cfg = getattr(config, "target_config", None) or {}
    monitoring = target_cfg.get("monitoring", {})

    plugin_path = None
    if getattr(fuzzer, "target_manager", None) and target_cfg:
        plugin_rel = monitoring.get("plugin")
        if plugin_rel:
            plugin_path = fuzzer.target_manager.resolve_target_path(
                target_cfg, plugin_rel
            )

    return {
        "run_id": os.path.basename(config.log_dir),
        "target": getattr(config, "target_name", None),
        "resolved_target_config_path": target_cfg.get("config_path"),
        "resolved_watchlist_path": getattr(config, "watchlist", None),
        "plugin_path": plugin_path,
        "runtime_flags": {
            "method": getattr(config, "method", None),
            "schedule": str(getattr(config, "schedule", None)),
            "repeat": getattr(config, "repeat", None),
            "interval": getattr(config, "interval", None),
            "persistent": getattr(config, "persistent", None),
            "no_cov": getattr(config, "no_cov", None),
        },
    }


def parse_legacy_error_file(error_file: str):
    if not os.path.isfile(error_file):
        return []

    try:
        with open(error_file, "r") as fp:
            content = fp.read().strip()
        if not content:
            return []
        return ast.literal_eval(content)
    except Exception:
        return [f"unparsed_error_file: {error_file}"]


def find_queue_files(queue_dir: str, frame: str):
    pattern = os.path.join(queue_dir, f"msg-{frame}-*")
    return sorted(glob.glob(pattern))


def write_json(path: str, payload: Dict[str, Any]) -> None:
    with open(path, "w") as fp:
        json.dump(_json_safe(payload), fp, indent=2, sort_keys=True)


def record_error_case(
    config,
    frame: str,
    errors: List[Any],
    fuzzer=None,
    msg_list=None,
    state_dict=None,
    extra=None,
):
    error_json = os.path.join(config.error_dir, f"error-{frame}.json")
    context_json = os.path.join(config.meta_dir, f"context-{frame}.json")
    input_json = os.path.join(config.meta_dir, f"input-summary-{frame}.json")
    obs_json = os.path.join(config.meta_dir, f"observation-summary-{frame}.json")
    diag_json = os.path.join(config.meta_dir, f"diagnostics-summary-{frame}.json")
    exec_json = os.path.join(config.meta_dir, f"execution-summary-{frame}.json")

    execution_summary = getattr(fuzzer, "last_execution_summary", {}) if fuzzer else {}
    runtime_context = build_runtime_context(config, fuzzer=fuzzer)

    existing_errors = []
    if os.path.isfile(error_json):
        try:
            with open(error_json, "r") as fp:
                existing_payload = json.load(fp)
            existing_errors = existing_payload.get("errors", [])
        except Exception:
            existing_errors = []

    merged_errors = existing_errors + _json_safe(errors)

    error_payload = {
        "case_id": frame,
        "timestamp": frame,
        "run_id": os.path.basename(config.log_dir),
        "target": getattr(config, "target_name", None),
        "target_node": execution_summary.get("target_node"),
        "input_topic": execution_summary.get("topic_name"),
        "input_msg_type": execution_summary.get("msg_type"),
        "cycle": execution_summary.get("cycle"),
        "round": execution_summary.get("round"),
        "queue_files": find_queue_files(config.queue_dir, frame),
        "errors": merged_errors,
        "error_count": len(merged_errors),
        "extra": _json_safe(extra or {}),
    }
    write_json(error_json, error_payload)
    write_json(context_json, runtime_context)
    write_json(exec_json, execution_summary)

    if msg_list is not None:
        input_summary = summarize_input_messages(
            msg_list,
            mode=execution_summary.get("mode", "single"),
        )
        write_json(input_json, input_summary)

    if state_dict is not None:
        observation_summary = summarize_state_dict(state_dict)
        write_json(obs_json, observation_summary)
        write_json(diag_json, observation_summary.get("diagnostics_summary", {}))
