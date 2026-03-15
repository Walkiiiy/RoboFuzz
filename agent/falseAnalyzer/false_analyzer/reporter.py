import json
import os
from dataclasses import asdict
from typing import Iterable

from .schemas import TriageResult


def write_case(output_dir: str, result: TriageResult):
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, f"{result.case_id}.json")
    with open(path, "w") as fp:
        json.dump(asdict(result), fp, indent=2, sort_keys=True)


def write_summary(output_dir: str, results: Iterable[TriageResult]):
    os.makedirs(output_dir, exist_ok=True)
    data = [asdict(item) for item in results]
    path = os.path.join(output_dir, "summary.json")
    with open(path, "w") as fp:
        json.dump(data, fp, indent=2, sort_keys=True)
