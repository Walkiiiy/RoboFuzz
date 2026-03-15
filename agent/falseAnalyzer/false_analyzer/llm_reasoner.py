import json
import os
import urllib.error
import urllib.request
from dataclasses import replace
from typing import Any, Dict

from .normalizer import normalize_case
from .schemas import CaseBundle, TriageResult


DEFAULT_BASE_URL = "https://api.deepseek.com"
DEFAULT_MODEL = "deepseek-chat"


def _extract_json(text: str) -> Dict[str, Any]:
    text = text.strip()
    if text.startswith("```"):
        parts = text.split("```")
        for part in parts:
            part = part.strip()
            if not part or part == "json":
                continue
            if part.startswith("json"):
                part = part[4:].strip()
            text = part
            break

    start = text.find("{")
    end = text.rfind("}")
    if start == -1 or end == -1 or end < start:
        raise ValueError("no JSON object found in model response")
    return json.loads(text[start : end + 1])


def _build_payload(bundle: CaseBundle, triage: TriageResult) -> Dict[str, Any]:
    norm = normalize_case(bundle)
    return {
        "target": bundle.context.get("target") or bundle.error_payload.get("target"),
        "run_id": bundle.run_id,
        "case_id": bundle.case_id,
        "rule_triage": {
            "verdict": triage.verdict,
            "vuln_level": triage.vuln_level,
            "score": triage.investigation_score,
            "cause": triage.likely_root_cause_class,
            "matched_patterns": triage.matched_patterns,
            "primary_evidence": triage.primary_evidence,
            "counter_evidence": triage.counter_evidence,
            "next_action": triage.next_action,
        },
        "errors": bundle.error_payload.get("errors", [])[:12],
        "execution": bundle.execution,
        "input_summary": bundle.input_summary,
        "observation_summary": bundle.observation_summary,
        "diagnostics_summary": bundle.diagnostics_summary,
        "normalization": norm,
    }


def _build_messages(bundle: CaseBundle, triage: TriageResult):
    system_prompt = (
        "You are a robotics fuzzing triage analyst. "
        "Your job is to judge whether a RoboFuzz finding is worth deeper investigation. "
        "Be conservative: do not misclassify infrastructure, publishing, dependency, "
        "or observation failures as target vulnerabilities. "
        "Prioritize downstream target impact, diagnostics, semantic invariant violations, "
        "availability loss, and safety-relevant consequences. "
        "Return only valid JSON with keys: verdict, vuln_level, investigation_score, "
        "confidence, likely_root_cause_class, is_broad_vulnerability_candidate, "
        "reasoning_summary, primary_evidence, counter_evidence, next_action, missing_evidence."
    )
    user_prompt = json.dumps(_build_payload(bundle, triage), indent=2, sort_keys=True)
    return [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_prompt},
    ]


def _call_deepseek(base_url: str, api_key: str, model: str, messages):
    body = {
        "model": model,
        "messages": messages,
        "temperature": 0.1,
        "response_format": {"type": "json_object"},
    }
    req = urllib.request.Request(
        base_url.rstrip("/") + "/chat/completions",
        data=json.dumps(body).encode("utf-8"),
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}",
        },
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=60) as resp:
        payload = json.loads(resp.read().decode("utf-8"))
    return payload["choices"][0]["message"]["content"]


def enrich_with_llm(
    bundle: CaseBundle,
    triage: TriageResult,
    *,
    use_llm: bool = False,
    api_key: str | None = None,
    base_url: str | None = None,
    model: str | None = None,
) -> TriageResult:
    if not use_llm or not triage.llm_ready:
        return triage

    api_key = api_key or os.environ.get("DEEPSEEK_API_KEY")
    base_url = base_url or os.environ.get("DEEPSEEK_BASE_URL", DEFAULT_BASE_URL)
    model = model or os.environ.get("DEEPSEEK_MODEL", DEFAULT_MODEL)

    if not api_key:
        raise RuntimeError(
            "DeepSeek API key not set. Export DEEPSEEK_API_KEY or pass --api-key."
        )

    try:
        content = _call_deepseek(base_url, api_key, model, _build_messages(bundle, triage))
        parsed = _extract_json(content)
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="ignore")
        raise RuntimeError(f"DeepSeek HTTP error {exc.code}: {detail}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"DeepSeek network error: {exc}") from exc

    return replace(
        triage,
        verdict=parsed.get("verdict", triage.verdict),
        vuln_level=parsed.get("vuln_level", triage.vuln_level),
        investigation_score=int(parsed.get("investigation_score", triage.investigation_score)),
        confidence=float(parsed.get("confidence", triage.confidence)),
        likely_root_cause_class=parsed.get(
            "likely_root_cause_class", triage.likely_root_cause_class
        ),
        is_broad_vulnerability_candidate=bool(
            parsed.get(
                "is_broad_vulnerability_candidate",
                triage.is_broad_vulnerability_candidate,
            )
        ),
        primary_evidence=list(parsed.get("primary_evidence", triage.primary_evidence)),
        counter_evidence=list(parsed.get("counter_evidence", triage.counter_evidence)),
        next_action=parsed.get("next_action", triage.next_action),
        llm_used=True,
        llm_model=model,
        llm_summary=parsed.get("reasoning_summary", ""),
        notes=triage.notes + list(parsed.get("missing_evidence", [])),
    )
