from dataclasses import dataclass, field
from typing import Any, Dict, List


@dataclass
class CaseBundle:
    case_id: str
    run_id: str
    paths: Dict[str, str]
    error_payload: Dict[str, Any] = field(default_factory=dict)
    context: Dict[str, Any] = field(default_factory=dict)
    execution: Dict[str, Any] = field(default_factory=dict)
    input_summary: Dict[str, Any] = field(default_factory=dict)
    observation_summary: Dict[str, Any] = field(default_factory=dict)
    diagnostics_summary: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TriageResult:
    case_id: str
    run_id: str
    verdict: str
    vuln_level: str
    investigation_score: int
    confidence: float
    likely_root_cause_class: str
    is_broad_vulnerability_candidate: bool
    matched_patterns: List[str] = field(default_factory=list)
    primary_evidence: List[str] = field(default_factory=list)
    counter_evidence: List[str] = field(default_factory=list)
    next_action: str = "replay_first"
    llm_ready: bool = False
    llm_used: bool = False
    llm_model: str = ""
    llm_summary: str = ""
    notes: List[str] = field(default_factory=list)
