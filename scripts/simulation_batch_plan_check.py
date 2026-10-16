#!/usr/bin/env python3
"""Feature 4B: collision-aware MoveIt plan-only check for every queued object.

Builds on the Feature 4A eligibility queue (simulation_batch_preview.py). For each
queued object the existing owned R1.4 supervisor starts ONE fresh fake-hardware
scene launch, inserts every recorded observation into the PlanningScene, and asks
the existing executor/full-cycle preplanner to plan only that target. All other
observations stay where they were recorded, as obstacles.

Every object is therefore checked against the same known initial PlanningScene.
Individually plannable objects are NOT a validated pick sequence: the batch always
reports SEQUENCE_NOT_VALIDATED. No trajectory is executed and nothing is written
to the scene's authored or generated task files; evidence goes to --output-dir.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from simulation_batch_preview import preview_batch  # noqa: E402

SCHEMA = "workcell_simulation_batch_plan_check/v1"
OUTCOMES = ("PLANNABLE", "UNREACHABLE", "COLLISION", "INVALID_GEOMETRY", "NO_VALID_GRASP",
            "MISSING_DESTINATION", "STALE_INPUT", "BLOCKED")
STAGES = ["PREPLAN_APPROACH", "PREPLAN_GRASP", "PREPLAN_CLOSE_GRIPPER", "PREPLAN_LIFT",
          "PREPLAN_TRANSFER", "PREPLAN_PLACE", "PREPLAN_OPEN_GRIPPER", "PREPLAN_RETREAT",
          "PREPLAN_HOME"]
NO_IK = -31  # moveit_msgs/MoveItErrorCodes.NO_IK_SOLUTION
PLANNER_TIMED_OUT = -6  # moveit_msgs/MoveItErrorCodes.TIMED_OUT
IN_COLLISION = (-10, -12)  # START_STATE_IN_COLLISION, GOAL_IN_COLLISION


def _emit(event: dict[str, Any]) -> None:
    print(json.dumps(event, sort_keys=True), flush=True)


def _complete_plan(audit: dict, object_id: str) -> bool:
    plans = audit.get("plan_metadata") or []
    return (audit.get("result") == "PASS" and audit.get("full_cycle_prevalidated") is True
            and audit.get("execution_attempted") is False and not audit.get("execution_action_goals")
            and audit.get("prevalidation_left_live_scene_unchanged") is True
            and audit.get("shutdown_clean") is True and audit.get("selected_object_id") == object_id
            and [p.get("stage") for p in plans] == STAGES
            and all(p.get("success") and p.get("moveit_code") == 1 and p.get("points", 0) >= 2
                    for p in plans))


def _failed_check(attempt: dict) -> dict:
    return next((c for c in attempt.get("checks", []) if c.get("status") == "FAIL"), {})


def _progress(attempt: dict) -> int:
    return sum(1 for s in attempt.get("stages", []) if s.get("success"))


def classify(audit: dict, object_id: str) -> dict[str, Any]:
    """Map existing executor/resolver evidence to one batch outcome (first failure wins)."""
    if _complete_plan(audit, object_id):
        return dict(outcome="PLANNABLE", first_failed_stage=None, reason="all nine plan-only stages succeeded",
                    moveit_code=1, collision_objects=[], colliding_links=[])
    attempts = [a for a in audit.get("candidate_attempts") or [] if a.get("object_id") == object_id]
    # The executor's own failure is the diagnostic; the supervisor adds only a generic wrapper.
    reason = str(audit.get("failure") or audit.get("orchestration_failure") or "")
    resolution = audit.get("task_intent_resolution") or {}
    primary = (resolution.get("readiness") or {}).get("primary_code") or ""
    text = f"{primary} {reason}"
    stage = audit.get("failed_stage")
    out = dict(outcome="BLOCKED", first_failed_stage=stage, reason=reason or primary or "no verified plan",
               moveit_code=None, collision_objects=[], colliding_links=[])
    if audit.get("execution_attempted") or audit.get("execution_action_goals"):
        return dict(out, reason="SAFETY: execution was observed during a plan-only check")
    if audit.get("orchestration_failure", "").startswith(("CANCELLED", "PLANNER_TIMEOUT")):
        return dict(out, reason=audit["orchestration_failure"])
    if attempts:
        exhausted = [a for a in attempts if "budget exhausted" in (_failed_check(a).get("reason") or "")]
        if exhausted:
            # Running out of planning time is not evidence of unreachability or collision.
            return dict(out, outcome="BLOCKED", first_failed_stage=_failed_check(exhausted[0]).get("code"),
                        candidate_attempts=len(attempts),
                        reason=(f"PLANNER_TIMEOUT: candidate search budget exhausted; {len(exhausted)} of "
                                f"{len(attempts)} grasp candidates were not fully planned. Check again with a "
                                "longer planning time."))
        expired = [a for a in attempts if "expired" in (_failed_check(a).get("reason") or "")]
        evaluated = [a for a in attempts if a not in expired]
        if expired:
            # Candidates that were never planned are not a verdict on reachability or collision.
            first = _failed_check(evaluated[0]) if evaluated else {}
            return dict(out, outcome="STALE_INPUT", first_failed_stage=first.get("code") or "GENERATE_GRASPS",
                        moveit_code=first.get("moveit_code"), candidate_attempts=len(attempts),
                        reason=(f"observation exceeded the task's maximum age after {len(evaluated)} of "
                                f"{len(attempts)} grasp candidates; {len(expired)} were never planned"
                                + (f" (first planned candidate: {first.get('code')}: {first.get('reason')})"
                                   if first else "")
                                + ". Increase Maximum observation age for this recording and check again."))
        # Evidence from the candidate that progressed furthest (first wins ties).
        best = max(attempts, key=_progress)
        check = _failed_check(best)
        code = check.get("moveit_code")
        why = check.get("reason") or check.get("reason_code") or reason
        out.update(first_failed_stage=check.get("code") or stage, reason=why, moveit_code=code,
                   collision_objects=check.get("collision_objects", []),
                   colliding_links=check.get("colliding_links", []),
                   candidate_attempts=len(attempts),
                   attempt_failure_kinds=sorted({_failed_check(a).get("failure_kind", "unknown") for a in attempts}))
        if check.get("failure_kind") == "collision" or code in IN_COLLISION:
            return dict(out, outcome="COLLISION")
        if code == PLANNER_TIMED_OUT and check.get("failure_kind") == "planning":
            # A sampling planner running out of time proves neither reachability nor collision.
            return dict(out, outcome="BLOCKED",
                        reason=(f"PLANNER_INCONCLUSIVE: MoveIt timed out (code -6) at {out['first_failed_stage']} on "
                                "the best candidate; no IK failure or contact was measured. Check plans again."))
        if code == NO_IK or "IK failed" in why or check.get("failure_kind") == "planning":
            # No contacts were measured: MoveIt reported no solution, which is not a collision claim.
            return dict(out, outcome="UNREACHABLE")
        if "DESTINATION" in (check.get("code") or ""):
            return dict(out, outcome="MISSING_DESTINATION",
                        reason="no valid placement in the destination for this object: " + why)
        if "expired" in why or "STALE" in why:
            return dict(out, outcome="STALE_INPUT")
        if any(k in why for k in ("aperture", "geometry", "differs from observation")):
            return dict(out, outcome="INVALID_GEOMETRY")
        return dict(out, outcome="NO_VALID_GRASP")
    if any(k in text for k in ("STALE", "expired", "OBSERVATION_TIME_INVALID")):
        return dict(out, outcome="STALE_INPUT")
    if any(k in text for k in ("PHYSICAL_DESTINATION_INVALID", "PLACE_LOCAL_POSE_OUTSIDE_REGION",
                               "destination bounds")):
        return dict(out, outcome="MISSING_DESTINATION")
    if any(k in text for k in ("TARGET_NOT_ELIGIBLE", "insertion mismatch", "GRASP_CANDIDATE_INVALID",
                               "SIMULATION_INPUT_INVALID")):
        return dict(out, outcome="INVALID_GEOMETRY")
    if "GENERATE_GRASPS" in text or "NO_GRASP" in text:
        return dict(out, outcome="NO_VALID_GRASP")
    return out


class Cancelled(Exception):
    pass


def _stop_group(process: subprocess.Popen, grace_s: float) -> bool:
    """Ask the supervisor to clean up its own launch; escalate only after grace."""
    if process.poll() is not None:
        return True
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return True
    try:
        process.wait(timeout=grace_s)
        return True
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)
        return False


def check_batch(scene: Path, output_dir: Path, *, limit: int, per_object_timeout: float,
                domain_id: int, supervisor: Path, cancel_flag: list[bool]) -> dict[str, Any]:
    preview = preview_batch(scene, limit=limit)
    import yaml
    from task_intent_resolver import context_hash, normalized_intent_hash, read_scene_task
    intent, physical, document = read_scene_task(scene)
    cell = yaml.safe_load((scene / "cell_definition.yaml").read_text())
    handoff_current = (cell.get("normalized_intent_sha256") == normalized_intent_hash(intent) and
                       cell.get("task_intent_resolution", {}).get("context_sha256") ==
                       context_hash(intent, physical, document))
    report: dict[str, Any] = {
        "schema_version": SCHEMA, "scene": str(scene), "output_dir": str(output_dir),
        "preview": {k: preview[k] for k in ("status", "reason", "source_path", "source_sha256",
                                              "task_intent_sha256", "total_observed", "total_eligible",
                                              "queued", "destination_region_ref")},
        "objects": [], "obstacles": [o for o in preview["observed_objects"] if o["role"] != "PICK_CANDIDATE"],
        "initial_scene_policy": "fresh_fake_hardware_launch_per_object_all_observations_inserted",
        "sequence_status": "SEQUENCE_NOT_VALIDATED",
        "sequence_note": "Each object was planned from the same recorded scene with every other object in place. "
                         "Picking one object first is not simulated; a batch order is not validated.",
        "moveit_planning_attempted": False, "motion_commanded": False, "execution_attempted": False,
        "epd_used": False, "simulator_live_connected": False,
    }
    observed = {o["object_id"]: o for o in preview["observed_objects"]}
    if preview["status"] != "ELIGIBILITY_ONLY":
        report.update(status="BLOCKED", reason=preview["reason"])
        return report
    if not handoff_current:
        report.update(status="BLOCKED", reason="TASK_HANDOFF_STALE: Generate the saved task before checking plans")
        return report
    if domain_id + len(preview["queue"]) > 232:
        raise ValueError("DOMAIN_RANGE: choose a lower --domain-id for this queue length")
    report["moveit_planning_attempted"] = True
    _emit(dict(event="batch_start", queued=len(preview["queue"]), observed=preview["total_observed"]))
    for item in preview["queue"]:
        oid = item["object_id"]
        entry = {"order": item["order"], "object_id": oid, "class_id": item["class_id"],
                 "source_zone_ref": preview["pick_zone_ref"] if "pick_zone_ref" in preview else None,
                 "destination_region_ref": item["destination_region_ref"],
                 "grasp_strategy_policy": item["grasp_strategy_policy"],
                 "recorded_geometry": {"dimensions_m": observed[oid]["dimensions_m"],
                                       "world_pose": observed[oid]["world_pose"]}}
        if cancel_flag[0]:
            entry.update(outcome="BLOCKED", reason="CANCELLED: not checked", first_failed_stage=None)
            report["objects"].append(entry)
            continue
        run_dir = output_dir / f"object_{item['order']:02d}"
        _emit(dict(event="object_start", order=item["order"], object_id=oid, class_id=item["class_id"]))
        command = [sys.executable, str(supervisor), "--output-dir", str(run_dir), "--scene-dir", str(scene),
                   "--target-object-id", oid, "--timeout", str(per_object_timeout),
                   "--domain-id", str(domain_id + item["order"] - 1), "--stream-status"]
        started = time.monotonic()
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                   start_new_session=True)
        timed_out = cancelled = False

        def forward(stream=process.stdout, oid=oid):
            for line in stream:
                try:
                    event = json.loads(line)
                except ValueError:
                    continue
                if isinstance(event, dict) and set(event) == {"stage"}:
                    _emit(dict(event="stage", object_id=oid, stage=event["stage"]))
        reader = threading.Thread(target=forward, daemon=True)
        reader.start()
        while process.poll() is None:
            if cancel_flag[0]:
                cancelled = True
                break
            if time.monotonic() - started > per_object_timeout + 45:  # supervisor cleanup budget
                timed_out = True
                break
            time.sleep(0.1)
        clean_stop = True
        if cancelled or timed_out:
            clean_stop = _stop_group(process, 45.0)
        process.wait()
        reader.join(timeout=5)
        acceptance = run_dir / "acceptance.json"
        audit = json.loads(acceptance.read_text()) if acceptance.is_file() else {}
        if cancelled:
            prior = audit.get("orchestration_failure") or audit.get("failure")
            audit["orchestration_failure"] = "CANCELLED by operator" + (f" ({prior})" if prior else "")
            audit["result"] = "FAIL"
        if timed_out:
            audit["orchestration_failure"] = f"PLANNER_TIMEOUT after {per_object_timeout:.0f} s"
            audit["result"] = "FAIL"
        if not audit:
            audit = {"result": "FAIL", "orchestration_failure": f"supervisor exited {process.returncode} without evidence"}
        verdict = classify(audit, oid)
        inserted = set(audit.get("inserted_object_ids") or [])
        entry.update(verdict, evidence_dir=str(run_dir), runtime_seconds=round(time.monotonic() - started, 1),
                     planning_scene_inserted_ids=sorted(inserted),
                     all_observations_in_planning_scene=bool(inserted) and inserted == set(observed),
                     shutdown_clean=bool(audit.get("shutdown_clean")) and clean_stop,
                     execution_attempted=bool(audit.get("execution_attempted")),
                     selected_grasp_strategy=((audit.get("task_intent_resolution") or {})
                                              .get("grasp_resolution") or {}).get("selected_strategy_ref"))
        report["execution_attempted"] |= entry["execution_attempted"]
        report["objects"].append(entry)
        _emit(dict(event="object_result", **{k: entry[k] for k in ("order", "object_id", "outcome",
                                                                   "first_failed_stage", "reason")}))
    counts = {o: sum(1 for e in report["objects"] if e.get("outcome") == o) for o in OUTCOMES}
    report.update(status="CANCELLED" if cancel_flag[0] else "CHECKED", outcome_counts=counts)
    if report["execution_attempted"]:
        report.update(status="BLOCKED", reason="SAFETY: execution observed during plan-only batch")
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=10)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--per-object-timeout", type=float, default=240.0)
    parser.add_argument("--domain-id", type=int, default=183)
    parser.add_argument("--supervisor", type=Path,
                        help="Owned plan-only supervisor (default: the scene repository's scripts/, as Builder uses)")
    args = parser.parse_args(argv)
    if not 0 < args.per_object_timeout <= 900 or not 0 <= args.domain_id <= 232:
        parser.error("per-object-timeout must be in (0,900]; domain-id in [0,232]")
    scene = args.scene.resolve()
    if args.supervisor is None:
        candidates = [scene.parent.parent / "scripts/run_r14_plan_only_acceptance.py",
                      SCRIPT_DIR / "run_r14_plan_only_acceptance.py"]
        args.supervisor = next((c for c in candidates if c.is_file()), candidates[0])
    out = (args.output_dir or scene / "generated/multiobject_plan_check" /
           time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())).resolve()
    out.mkdir(parents=True, exist_ok=False)
    cancel = [False]

    def on_signal(signum, frame):
        cancel[0] = True
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    try:
        report = check_batch(scene, out, limit=args.limit, per_object_timeout=args.per_object_timeout,
                             domain_id=args.domain_id, supervisor=args.supervisor, cancel_flag=cancel)
    except Exception as exc:  # fail closed with a readable reason
        report = {"schema_version": SCHEMA, "status": "BLOCKED", "reason": str(exc), "objects": [],
                  "moveit_planning_attempted": False, "motion_commanded": False,
                  "execution_attempted": False, "sequence_status": "SEQUENCE_NOT_VALIDATED"}
    (out / "batch_plan_check.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    _emit(dict(event="batch_result", report=report))
    return 0 if report["status"] == "CHECKED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
