"""Pure offline regressions for operator-opted historical retention loss exclusion."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.stage_a_retention_candidate_veto import (
    load_retention_veto, excludes_candidate, rejected_evaluation,
)

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "evidence/stage_a1/retirement_20260925/release_failure/report.json"


def request(*, object_id="runtime::part_07", strategy="finger_pinch_basic", index=7):
    return {"observation": {"id": object_id}, "strategy_ref": strategy,
            "candidate": SimpleNamespace(candidate_id=f"{strategy}::{index:03}")}


def test_historical_physical_failure_is_a_veto_not_proof():
    assert REPORT.is_file(), "This test uses the immutable retained real Stage-A trace report"
    veto = load_retention_veto(REPORT, strategy_ref="finger_pinch_basic")
    assert veto["object_id"] == "runtime::part_07"
    assert veto["grasp_index"] == 7
    assert veto["report_sha256"] == hashlib.sha256(REPORT.read_bytes()).hexdigest()
    assert "not a physical stability proof" in veto["interpretation"]
    assert excludes_candidate(request(), veto)
    rejected = rejected_evaluation(request(), veto)
    assert rejected["reason_code"] == "MEASURED_RETENTION_LOSS"
    assert rejected["success"] is False
    assert rejected["retryable"] is False
    assert rejected["stop_search"] is False
    assert rejected["checks"][0]["status"] == "BLOCKED"


@pytest.mark.parametrize("kwargs", [
    {"object_id": "runtime::part_06"},
    {"strategy": "top_2f"},
    {"index": 3},
    {"index": 70},
])
def test_exclusion_is_exact_and_does_not_rewrite_others(kwargs):
    veto = load_retention_veto(REPORT, strategy_ref="finger_pinch_basic")
    sample = request(**kwargs)
    original = copy.deepcopy(sample)
    assert not excludes_candidate(sample, veto)
    assert sample == original


@pytest.mark.parametrize("mutate", [
    lambda r: r.update(classification="PREDICTED_ONLY"),
    lambda r: r.update(failed_stage="EXECUTE_PLACE"),
    lambda r: r.update(open_action_attempted=True),
    lambda r: r.update(cancellation_confirmed=False),
    lambda r: r["reconciliation"].update(acm_restored=False),
    lambda r: r["reconciliation"].update(attached_ids=["runtime::part_07"]),
    lambda r: r["reconciliation"].update(held=True),
    lambda r: r["candidate"].update(grasp=True),
    lambda r: r["candidate"].update(object=""),
    lambda r: r["previous"].update(held_check_passed=False),
    lambda r: r["first_violation"].update(held_check_passed=True),
    lambda r: r["first_violation"].update(fingertips=["unknown_finger"]),
    lambda r: r["first_violation"].update(iteration=r["previous"]["iteration"]),
])
def test_invalid_or_unqualified_history_refused(tmp_path, mutate):
    data = json.loads(REPORT.read_text())
    mutate(data)
    path = tmp_path / "changed.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="RETENTION_EVIDENCE_INVALID"):
        load_retention_veto(path, strategy_ref="finger_pinch_basic")


def test_strategy_is_explicit_and_report_is_not_secretly_enriched():
    with pytest.raises(ValueError, match="RETENTION_STRATEGY_REQUIRED"):
        load_retention_veto(REPORT, strategy_ref="")
    veto = load_retention_veto(REPORT, strategy_ref="finger_pinch_basic")
    assert "source_sha256" not in veto  # Declared hashes of external files are not verified here.
    assert "physical_qualification" not in veto


def test_executor_veto_is_fresh_plan_only_and_stage_runner_scopes_it():
    executor = (ROOT / "scripts/perceived_object_grasp_execute.py").read_text()
    runner = (ROOT / "scripts/run_stage_a1_finish.py").read_text()
    assert "retention_veto=retention_veto" in executor
    assert "RETENTION_VETO_REQUIRES_FRESH_RESOLUTION" in executor
    assert "if retention_veto is not None and excludes_candidate(request, retention_veto)" in executor
    assert "RETENTION_VETO_BYPASSED" in executor
    assert "not args.resolve_task or args.start or args.backend != 'simulator'" in executor
    assert 'retention_veto=args.retention_veto' in runner
    assert 'if not resolve or gate is not None:' in runner
    assert 'cmd += ["--retention-failure-report",str(report),' in runner
