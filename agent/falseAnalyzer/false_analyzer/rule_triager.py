from .normalizer import normalize_case
from .schemas import CaseBundle, TriageResult


def triage_case(bundle: CaseBundle) -> TriageResult:
    norm = normalize_case(bundle)
    patterns = set(norm["matched_patterns"])
    evidence = []
    counter = []

    verdict = "replay_first"
    vuln_level = "V1"
    score = 50
    confidence = 0.6
    cause = "robustness"
    broad_candidate = False
    next_action = "replay_same_input"
    llm_ready = True

    if "INFRA_TYPE_SUPPORT" in patterns or "INFRA_PUBLISH_FAIL" in patterns:
        verdict = "discard"
        vuln_level = "V0"
        score = 5
        confidence = 0.95
        cause = "infra"
        broad_candidate = False
        next_action = "clean_env_rerun"
        llm_ready = False
        evidence.append("Failure occurred in the publish/type-support path.")

    elif norm["target_started"] is False:
        verdict = "discard"
        vuln_level = "V0"
        score = 10
        confidence = 0.85
        cause = "infra"
        broad_candidate = False
        next_action = "clean_env_rerun"
        llm_ready = False
        evidence.append("Target did not appear to start cleanly.")

    elif "OBS_EMPTY_CAPTURE" in patterns or "OBS_PIPELINE_EMPTY" in patterns:
        verdict = "replay_first"
        vuln_level = "V1"
        score = 30
        confidence = 0.7
        cause = "observation"
        broad_candidate = False
        next_action = "inspect_watchlist"
        evidence.append("Observed data path is incomplete or empty.")

    if "LIVENESS_NO_OUTPUT" in patterns:
        score = max(score, 45)
        verdict = "replay_first"
        vuln_level = "V2"
        cause = "availability"
        evidence.append("Input was observed but expected downstream output was missing.")

    if "NUMERIC_COVARIANCE_EXPLOSION" in patterns:
        score = max(score, 72)
        verdict = "investigate"
        vuln_level = "V2"
        cause = "semantic_integrity"
        broad_candidate = True
        evidence.append("State output contains covariance explosion.")

    if "STATE_TELEPORTATION" in patterns or "TF_MISSING_EDGE" in patterns:
        score = max(score, 78)
        verdict = "investigate"
        vuln_level = "V3"
        cause = "safety"
        broad_candidate = True
        evidence.append("Target state violated a core spatial consistency invariant.")

    if "ACTION_STATUS_INVALID" in patterns or "MOTION_PLAN_COUNT_INVALID" in patterns:
        score = max(score, 76)
        verdict = "investigate"
        vuln_level = "V3"
        cause = "semantic_integrity"
        broad_candidate = True
        evidence.append("Planner/controller action semantics became inconsistent.")

    if norm["diagnostics_max_level"] >= 2 or "DIAGNOSTIC_LEVEL_ERROR" in patterns:
        score = min(100, score + 10)
        verdict = "investigate" if verdict != "discard" else verdict
        evidence.append("Target diagnostics reported an internal error state.")

    if norm["contains_nan"] or norm["contains_inf"]:
        counter.append("Input already contains NaN/INF, so some failures may be direct echoes.")
        score = max(0, score - 8)

    if norm["contains_extreme_numeric"]:
        counter.append("Input contains extreme numeric values, which may reduce exploit significance.")
        score = max(0, score - 4)

    if verdict in {"investigate"} and score >= 85:
        verdict = "high_priority"
        vuln_level = "V3"
        next_action = "manual_review"

    return TriageResult(
        case_id=bundle.case_id,
        run_id=bundle.run_id,
        verdict=verdict,
        vuln_level=vuln_level,
        investigation_score=score,
        confidence=confidence,
        likely_root_cause_class=cause,
        is_broad_vulnerability_candidate=broad_candidate,
        matched_patterns=sorted(patterns),
        primary_evidence=evidence,
        counter_evidence=counter,
        next_action=next_action,
        llm_ready=llm_ready,
        notes=norm["unique_errors"][:6],
    )
