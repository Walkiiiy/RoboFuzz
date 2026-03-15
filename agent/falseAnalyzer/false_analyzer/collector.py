import json
import os
import ast
from typing import Dict, List

from .schemas import CaseBundle


def _load_json(path: str) -> Dict:
    if not os.path.isfile(path):
        return {}
    with open(path, "r") as fp:
        return json.load(fp)


def _load_legacy_error(path: str) -> Dict:
    if not os.path.isfile(path):
        return {}
    with open(path, "r") as fp:
        content = fp.read().strip()
    if not content:
        errors = []
    else:
        try:
            errors = ast.literal_eval(content)
        except Exception:
            errors = [content]
    return {
        "case_id": os.path.basename(path).replace("error-", ""),
        "errors": errors,
        "error_count": len(errors),
    }


def collect_run(run_dir: str) -> List[CaseBundle]:
    errors_dir = os.path.join(run_dir, "errors")
    meta_dir = os.path.join(run_dir, "metadata")
    run_id = os.path.basename(run_dir.rstrip("/"))

    bundles = []
    entries = sorted(os.listdir(errors_dir))
    case_names = []
    for name in entries:
        if not name.startswith("error-"):
            continue
        if name.endswith(".json"):
            case_names.append((name[len("error-") : -len(".json")], True))
            continue
        if "-out-" in name or "-trace-" in name:
            continue
        case_names.append((name[len("error-") :], False))

    seen = set()
    for case_id, has_json in case_names:
        if case_id in seen:
            continue
        seen.add(case_id)
        paths = {
            "error_json": os.path.join(errors_dir, f"error-{case_id}.json"),
            "legacy_error": os.path.join(errors_dir, f"error-{case_id}"),
            "context_json": os.path.join(meta_dir, f"context-{case_id}.json"),
            "execution_json": os.path.join(meta_dir, f"execution-summary-{case_id}.json"),
            "input_json": os.path.join(meta_dir, f"input-summary-{case_id}.json"),
            "observation_json": os.path.join(meta_dir, f"observation-summary-{case_id}.json"),
            "diagnostics_json": os.path.join(meta_dir, f"diagnostics-summary-{case_id}.json"),
        }
        error_payload = _load_json(paths["error_json"]) if has_json else {}
        if not error_payload:
            error_payload = _load_legacy_error(paths["legacy_error"])
        bundles.append(
            CaseBundle(
                case_id=case_id,
                run_id=run_id,
                paths=paths,
                error_payload=error_payload,
                context=_load_json(paths["context_json"]),
                execution=_load_json(paths["execution_json"]),
                input_summary=_load_json(paths["input_json"]),
                observation_summary=_load_json(paths["observation_json"]),
                diagnostics_summary=_load_json(paths["diagnostics_json"]),
            )
        )

    return bundles
