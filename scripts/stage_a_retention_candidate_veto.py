#!/usr/bin/env python3
"""Operator-opted diagnostic exclusion of one historically unstable Stage-A grasp.

A veto only *removes* a candidate from a fresh plan-only search; it cannot
authorize retreat, motion, release, or reusing an old disturbed scene.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


def load_retention_veto(path: Path, *, strategy_ref: str) -> dict[str, Any]:
    path = Path(path).resolve(strict=True)
    content = path.read_bytes()
    try:
        document = json.loads(content)
    except (ValueError, UnicodeDecodeError) as exc:
        raise ValueError("RETENTION_EVIDENCE_INVALID: require a JSON failure report") from exc
    if not isinstance(document, dict):
        raise ValueError("RETENTION_EVIDENCE_INVALID: report must be a mapping")
    if not isinstance(strategy_ref, str) or not strategy_ref.strip() or strategy_ref != strategy_ref.strip():
        raise ValueError("RETENTION_STRATEGY_REQUIRED: explicitly name the historical grasp strategy")
    if not str(document.get("classification") or "").startswith(
            "D: measured opposing-fingertip contact loss"):
        raise ValueError("RETENTION_EVIDENCE_INVALID: not a measured opposing-contact failure")
    if document.get("failed_stage") != "EXECUTE_LIFT" or document.get("open_action_attempted") is not False:
        raise ValueError("RETENTION_EVIDENCE_INVALID: unsupported failure stage or subsequent opening")
    if document.get("cancellation_confirmed") is not True:
        raise ValueError("RETENTION_EVIDENCE_INVALID: owned cancellation was not confirmed")
    reconciled = document.get("reconciliation")
    if not isinstance(reconciled, dict) or reconciled.get("acm_restored") is not True or (
            reconciled.get("attached_ids") != [] or reconciled.get("held") is not False):
        raise ValueError("RETENTION_EVIDENCE_INVALID: measured scene reconciliation is not clean")
    candidate = document.get("candidate")
    if not isinstance(candidate, dict):
        raise ValueError("RETENTION_EVIDENCE_INVALID: selected candidate missing")
    object_id, index = candidate.get("object"), candidate.get("grasp")
    if not isinstance(object_id, str) or not object_id or (
            not isinstance(index, int) or isinstance(index, bool) or index < 0 or index > 10000):
        raise ValueError("RETENTION_EVIDENCE_INVALID: selected object/grasp index invalid")
    previous, failed = document.get("previous"), document.get("first_violation")
    if not isinstance(previous, dict) or not isinstance(failed, dict) or (
            previous.get("held_check_passed") is not True or failed.get("held_check_passed") is not False):
        raise ValueError("RETENTION_EVIDENCE_INVALID: no measured PASS-to-FAIL transition")
    before = previous.get("fingertips")
    after = failed.get("fingertips")
    if not isinstance(before, list) or len(set(before)) != 2 or not isinstance(after, list):
        raise ValueError("RETENTION_EVIDENCE_INVALID: opposing finger contact identities incomplete")
    if len(after) != 1 or after[0] not in before:
        raise ValueError("RETENTION_EVIDENCE_INVALID: missing one specifically measured fingertip")
    for name in ("iteration", "sim_ns"):
        prior, current = previous.get(name), failed.get(name)
        if type(prior) is not int or type(current) is not int or current <= prior:
            raise ValueError("RETENTION_EVIDENCE_INVALID: invalid consecutive physical samples")
    return {
        "object_id": object_id,
        "grasp_index": index,
        "strategy_ref": strategy_ref,
        "report_sha256": hashlib.sha256(content).hexdigest(),
        "source": str(path),
        "interpretation": "operator-reviewed historical rejection, not a physical stability proof",
    }


def excludes_candidate(request: dict[str, Any], veto: dict[str, Any]) -> bool:
    if request.get("strategy_ref") != veto["strategy_ref"]:
        return False
    observation = request.get("observation")
    candidate = request.get("candidate")
    if not isinstance(observation, dict) or observation.get("id") != veto["object_id"]:
        return False
    identity = str(getattr(candidate, "candidate_id", ""))
    # The existing strategy candidate contract ends in ::<integer-index>.
    head, separator, suffix = identity.rpartition("::")
    return bool(head and separator and suffix.isdigit() and int(suffix) == veto["grasp_index"])


def rejected_evaluation(request: dict[str, Any], veto: dict[str, Any]) -> dict[str, Any]:
    return {
        "success": False,
        "checks": [{"status": "BLOCKED", "code": "MEASURED_RETENTION_LOSS",
                    "reason": "Operator-supplied historical physical loss; no planning performed"}],
        "reason_code": "MEASURED_RETENTION_LOSS",
        "reason": ("Fresh commissioning excludes this exact object/strategy/grasp index "
                   "after the recorded opposing-fingertip loss; other candidates still require "
                   "their own physical proof"),
        "stop_search": False,
        "retryable": False,
        "progress_passes": 0,
        "failed_stage": "EXECUTE_LIFT",
    }
