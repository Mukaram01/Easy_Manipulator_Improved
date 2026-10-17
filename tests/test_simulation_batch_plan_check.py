"""Feature 4B batch plan check: outcome classification and owned-process orchestration.

Runtime MoveIt planning is proven by the workstation acceptance, not here. These tests
pin the mapping from the existing executor/preplanner evidence to batch outcomes and
the fail-closed, cancel and timeout behaviour of the orchestrator with a stub supervisor.
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import yaml

from test_simulation_batch_preview import prepare
from scripts import simulation_batch_plan_check as batch
from scripts.task_intent_resolver import context_hash, normalized_intent_hash, read_scene_task

ROOT = Path(__file__).resolve().parents[1]
STAGES = batch.STAGES
OID = "runtime::bottle-01"


def plan_pass(oid=OID):
    return {"result": "PASS", "full_cycle_prevalidated": True, "execution_attempted": False,
            "execution_action_goals": [], "prevalidation_left_live_scene_unchanged": True,
            "shutdown_clean": True, "selected_object_id": oid,
            "plan_metadata": [dict(stage=s, success=True, moveit_code=1, points=12) for s in STAGES]}


def failed(check, **extra):
    return dict({"result": "FAIL", "failure": check.get("reason", "x"), "shutdown_clean": True,
                 "execution_attempted": False, "candidate_attempts": [
                     {"object_id": OID, "stages": [{"stage": "PREPLAN_APPROACH", "success": True}],
                      "checks": [dict(status="FAIL", **check)]}]}, **extra)


def test_complete_nine_stage_plan_is_plannable():
    assert batch.classify(plan_pass(), OID)["outcome"] == "PLANNABLE"


def test_partial_or_wrong_target_plan_is_never_plannable():
    partial = plan_pass()
    partial["plan_metadata"] = partial["plan_metadata"][:3]
    assert batch.classify(partial, OID)["outcome"] != "PLANNABLE"
    assert batch.classify(plan_pass("runtime::other"), OID)["outcome"] != "PLANNABLE"
    executed = dict(plan_pass(), execution_attempted=True)
    verdict = batch.classify(executed, OID)
    assert verdict["outcome"] == "BLOCKED" and "SAFETY" in verdict["reason"]


def test_collision_evidence_maps_to_collision_with_objects():
    verdict = batch.classify(failed(dict(code="PREPLAN_GRASP", failure_kind="collision", moveit_code=-1,
                                         reason="goal in collision", colliding_links=["left_inner_finger"],
                                         collision_objects=["runtime::cup-02"])), OID)
    assert verdict["outcome"] == "COLLISION"
    assert verdict["first_failed_stage"] == "PREPLAN_GRASP"
    assert verdict["collision_objects"] == ["runtime::cup-02"]


def test_destination_containment_uses_executor_reason_not_supervisor_wrapper():
    # Shape recorded from the real ur5_2f_test run (cup-01, EXACT, 0.05 m clearance).
    audit = failed(dict(code="DESTINATION_CONTAINMENT", failure_kind="constraint",
                        reason="placement/object outside target usable placement geometry",
                        reason_code="DESTINATION_CONTAINMENT_FAILED"),
                   failure="DESTINATION_CONTAINMENT_FAILED: placement/object outside target usable placement geometry",
                   orchestration_failure="executor did not prove an unchanged, complete plan-only cycle")
    verdict = batch.classify(audit, OID)
    assert verdict["outcome"] == "MISSING_DESTINATION"
    assert verdict["first_failed_stage"] == "DESTINATION_CONTAINMENT"
    assert "usable placement geometry" in verdict["reason"]


def test_candidates_expiring_unplanned_are_stale_not_unreachable():
    # Shape recorded from the real cup-03 run: 1 candidate planned (TIMED_OUT), 7 expired unplanned.
    attempts = [{"object_id": OID, "stages": [], "checks": [dict(status="FAIL", code="PREPLAN_APPROACH",
                 failure_kind="planning", moveit_code=-6, reason="MoveIt action failed: status=6, code=-6")]}]
    attempts += [{"object_id": OID, "stages": [], "checks": [dict(status="FAIL", code="GENERATE_GRASPS",
                  failure_kind="constraint", reason="observation expired before candidate planning")]}] * 7
    verdict = batch.classify({"result": "FAIL", "candidate_attempts": attempts}, OID)
    assert verdict["outcome"] == "STALE_INPUT"
    assert "after 1 of 8" in verdict["reason"] and "7 were never planned" in verdict["reason"]
    assert verdict["first_failed_stage"] == "PREPLAN_APPROACH"


def test_exhausted_planning_budget_is_blocked_timeout_not_unreachable():
    # Shape recorded from the real 12 s budget run on ur5_2f_test.
    attempts = [{"object_id": OID, "stages": [], "checks": [dict(status="FAIL", code="PREPLAN_APPROACH",
                 failure_kind="planning", reason="candidate search budget exhausted")]}]
    verdict = batch.classify({"result": "FAIL", "candidate_attempts": attempts}, OID)
    assert verdict["outcome"] == "BLOCKED" and verdict["reason"].startswith("PLANNER_TIMEOUT")


def test_ik_failure_is_unreachable_but_planner_timeout_is_inconclusive():
    ik = failed(dict(code="PREPLAN_APPROACH", failure_kind="planning", moveit_code=-31, reason="IK failed: -31"))
    assert batch.classify(ik, OID)["outcome"] == "UNREACHABLE"
    # Shape recorded from a real ur5_2f_test recheck: OMPL timed out on every top_2f candidate.
    timeout = failed(dict(code="PREPLAN_TRANSFER", failure_kind="planning", moveit_code=-6,
                          reason="MoveIt action failed: status=6, code=-6"))
    verdict = batch.classify(timeout, OID)
    assert verdict["outcome"] == "BLOCKED" and verdict["reason"].startswith("PLANNER_INCONCLUSIVE")


def test_grasp_constraints_geometry_destination_and_stale():
    assert batch.classify(failed(dict(code="GENERATE_GRASPS", failure_kind="constraint",
                                      reason="TASK_CONSTRAINT_UNSATISFIED: candidate yaw excluded")),
                          OID)["outcome"] == "NO_VALID_GRASP"
    assert batch.classify(failed(dict(code="GENERATE_GRASPS", failure_kind="constraint",
                                      reason="target exceeds Robotiq aperture")), OID)["outcome"] == "INVALID_GEOMETRY"
    assert batch.classify({"result": "FAIL", "failure": "PHYSICAL_DESTINATION_INVALID: no region"},
                          OID)["outcome"] == "MISSING_DESTINATION"
    assert batch.classify({"result": "FAIL", "failure": "observation expired before candidate planning"},
                          OID)["outcome"] == "STALE_INPUT"
    assert batch.classify({"result": "FAIL", "failure": "TARGET_NOT_ELIGIBLE: x"}, OID)["outcome"] == "INVALID_GEOMETRY"
    assert batch.classify({"result": "FAIL", "orchestration_failure": "PLANNER_TIMEOUT after 5 s"},
                          OID)["outcome"] == "BLOCKED"


def generated(tmp_path, **kwargs):
    scene, recording = prepare(tmp_path, **kwargs)
    (scene / "cell_definition.yaml").write_text(yaml.safe_dump({}))
    intent, physical, document = read_scene_task(scene)
    cell = {"normalized_intent_sha256": normalized_intent_hash(intent),
            "task_intent_resolution": {"context_sha256": context_hash(intent, physical, document)}}
    (scene / "cell_definition.yaml").write_text(yaml.safe_dump(cell))
    return scene


STUB = textwrap.dedent("""
    import json, signal, sys, time
    from pathlib import Path
    a = sys.argv; out = Path(a[a.index('--output-dir') + 1]); oid = a[a.index('--target-object-id') + 1]
    out.mkdir(parents=True)
    mode = {MODE!r}
    def stop(*_):
        (out / 'acceptance.json').write_text(json.dumps({{'result': 'FAIL', 'failure': 'acceptance interrupted',
            'shutdown_clean': True, 'execution_attempted': False}})); sys.exit(1)
    signal.signal(signal.SIGTERM, stop)
    print(json.dumps({{'stage': 'PREPLAN_APPROACH'}}), flush=True)
    if mode == 'hang':
        time.sleep(600)
    if mode == 'noevidence':
        sys.exit(3)
    stages = {STAGES!r}
    audit = {{'result': 'PASS', 'full_cycle_prevalidated': True, 'execution_attempted': False,
             'execution_action_goals': [], 'prevalidation_left_live_scene_unchanged': True,
             'shutdown_clean': True, 'selected_object_id': oid, 'inserted_object_ids': {IDS!r},
             'plan_metadata': [dict(stage=s, success=True, moveit_code=1, points=5) for s in stages]}}
    (out / 'acceptance.json').write_text(json.dumps(audit))
""")


def stub(tmp_path, mode):
    path = tmp_path / f"stub_{mode}.py"
    path.write_text(STUB.format(MODE=mode, STAGES=STAGES,
                                IDS=["runtime::bottle-01", "runtime::bottle-02", "runtime::cup-01"]))
    return path


def run(scene, tmp_path, mode, timeout=30, extra=()):
    out = tmp_path / f"out_{mode}"
    command = [sys.executable, str(ROOT / "scripts/simulation_batch_plan_check.py"), "--scene", str(scene),
               "--output-dir", str(out), "--supervisor", str(stub(tmp_path, mode)),
               "--per-object-timeout", str(timeout), *extra]
    return subprocess.Popen(command, stdout=subprocess.PIPE, text=True, cwd=ROOT), out


def test_each_queued_object_gets_its_own_owned_check_and_sequence_is_not_validated(tmp_path):
    scene = generated(tmp_path)
    process, out = run(scene, tmp_path, "pass")
    stdout, _ = process.communicate(timeout=60)
    report = json.loads((out / "batch_plan_check.json").read_text())
    assert process.returncode == 0 and report["status"] == "CHECKED"
    assert [o["object_id"] for o in report["objects"]] == ["runtime::bottle-01", "runtime::bottle-02"]
    assert all(o["outcome"] == "PLANNABLE" and o["all_observations_in_planning_scene"] for o in report["objects"])
    assert [o["object_id"] for o in report["obstacles"]] == ["runtime::cup-01"]
    assert report["sequence_status"] == "SEQUENCE_NOT_VALIDATED"
    assert report["motion_commanded"] is False and report["execution_attempted"] is False
    events = [json.loads(line)["event"] for line in stdout.splitlines()]
    assert events.count("object_start") == 2 and events[-1] == "batch_result"


def test_missing_supervisor_evidence_fails_closed(tmp_path):
    scene = generated(tmp_path)
    process, out = run(scene, tmp_path, "noevidence")
    process.communicate(timeout=60)
    report = json.loads((out / "batch_plan_check.json").read_text())
    assert {o["outcome"] for o in report["objects"]} == {"BLOCKED"}
    assert "without evidence" in report["objects"][0]["reason"]


def test_timeout_stops_the_owned_process(tmp_path):
    scene = generated(tmp_path)
    process, out = run(scene, tmp_path, "hang", timeout=1, extra=["--limit", "1"])
    process.communicate(timeout=120)
    report = json.loads((out / "batch_plan_check.json").read_text())
    entry = report["objects"][0]
    assert entry["outcome"] == "BLOCKED" and "PLANNER_TIMEOUT" in entry["reason"]


def test_cancel_stops_current_check_and_marks_rest_not_checked(tmp_path):
    scene = generated(tmp_path)
    process, out = run(scene, tmp_path, "hang", timeout=300)
    deadline = time.time() + 30
    for line in process.stdout:
        if '"object_start"' in line or time.time() > deadline:
            break
    time.sleep(0.5)
    process.send_signal(signal.SIGTERM)
    process.communicate(timeout=90)
    report = json.loads((out / "batch_plan_check.json").read_text())
    assert report["status"] == "CANCELLED" and process.returncode == 1
    assert report["objects"][0]["outcome"] == "BLOCKED"
    assert report["objects"][1]["reason"].startswith("CANCELLED")


def test_stale_handoff_and_missing_input_block_before_any_launch(tmp_path):
    scene = generated(tmp_path)
    task = scene / "config/workcell_builder_task_intent.yaml"
    intent = yaml.safe_load(task.read_text())
    intent["place"]["placement"]["clearance_m"] = 0.02
    task.write_text(yaml.safe_dump(intent))
    process, out = run(scene, tmp_path, "pass")
    process.communicate(timeout=60)
    report = json.loads((out / "batch_plan_check.json").read_text())
    assert report["status"] == "BLOCKED" and report["reason"].startswith("TASK_HANDOFF_STALE")
    assert report["objects"] == [] and not list(out.glob("object_*"))
    (scene / "commissioning.yaml").unlink()
    (tmp_path / "again").mkdir()
    process, out2 = run(scene, tmp_path / "again", "pass")
    process.communicate(timeout=60)
    report = json.loads((out2 / "batch_plan_check.json").read_text())
    assert report["status"] == "BLOCKED" and report["moveit_planning_attempted"] is False


# ---------------------------------------------------------------- Feature 4C sequence
PLANNED = {"result": "PLANNED", "revision_before": 0, "revision_after": 1}


def test_sequence_entry_outcomes_reuse_4b_semantics():
    assert batch.classify_sequence_entry(dict(PLANNED, object_id=OID))["outcome"] == "PLANNABLE"
    timeout = {"object_id": OID, "result": "FAILED", "failure": "PREPLAN_TRANSFER_FAILED: x",
               "candidate_attempts": [{"object_id": OID, "stages": [], "checks": [dict(
                   status="FAIL", code="PREPLAN_TRANSFER", failure_kind="planning", moveit_code=-6,
                   reason="MoveIt action failed: status=6, code=-6")]}]}
    verdict = batch.classify_sequence_entry(timeout)
    assert verdict["outcome"] == "BLOCKED" and verdict["reason"].startswith("PLANNER_INCONCLUSIVE")
    assert batch.classify_sequence_entry({"object_id": OID, "result": "INVALID_INPUT", "reason": "x"})["outcome"] == "INVALID_INPUT"
    assert batch.classify_sequence_entry({"object_id": OID, "result": "SEQUENCE_BLOCKED", "reason": "y"})["outcome"] == "SEQUENCE_BLOCKED"


SEQ_STUB = textwrap.dedent("""
    import json, signal, sys, time
    from pathlib import Path
    a = sys.argv; out = Path(a[a.index('--output-dir') + 1]); ids = a[a.index('--sequence-object-ids') + 1].split(',')
    out.mkdir(parents=True)
    mode = {MODE!r}
    def stop(*_):
        (out / 'acceptance.json').write_text(json.dumps({{'result': 'FAIL', 'failure': 'acceptance interrupted',
            'shutdown_clean': True, 'execution_attempted': False}})); sys.exit(1)
    signal.signal(signal.SIGTERM, stop)
    print(json.dumps({{'stage': 'SEQUENCE_OBJECT_1'}}), flush=True)
    if mode == 'hang':
        time.sleep(600)
    stages = {STAGES!r}
    meta = [dict(stage=s, success=True, moveit_code=1, points=5) for s in stages]
    seq = [dict(position=1, object_id=ids[0], result='PLANNED', revision_before=0, revision_after=1,
                robot_start_matches_previous_end=True, placed_pose=[0.45, 0.22, 0.11, 0, 0, 0, 1],
                grasp_strategy='top_2f', plan_metadata=meta, collision_objects_considered=[])]
    for i, oid in enumerate(ids[1:], 2):
        seq.append(dict(position=i, object_id=oid, result='FAILED', revision_before=1, robot_start_matches_previous_end=True,
                        failure='PREPLAN_PLACE_FAILED: MoveIt action failed: status=6, code=-27',
                        candidate_attempts=[{{'object_id': oid, 'stages': [], 'checks': [dict(status='FAIL',
                            code='PREPLAN_PLACE', failure_kind='collision', moveit_code=-27, reason='MoveIt action failed',
                            collision_objects=[ids[0], oid])]}}]))
    result = 'FAIL' if mode == 'broken' else 'PASS'
    (out / 'acceptance.json').write_text(json.dumps({{'result': result, 'sequence_planning_completed': True,
        'execution_attempted': False, 'execution_action_goals': [], 'shutdown_clean': True, 'sequence': seq}}))
""")


def run_sequence(scene, tmp_path, mode, extra=()):
    path = tmp_path / f"seq_stub_{mode}.py"
    path.write_text(SEQ_STUB.format(MODE=mode, STAGES=STAGES))
    out = tmp_path / f"seq_{mode}"
    command = [sys.executable, str(ROOT / "scripts/simulation_batch_plan_check.py"), "--scene", str(scene),
               "--sequence", "--output-dir", str(out), "--supervisor", str(path), "--per-object-timeout", "60", *extra]
    return subprocess.Popen(command, stdout=subprocess.PIPE, text=True, cwd=ROOT), out


def test_sequence_reports_revisions_and_occupied_destination(tmp_path):
    scene = generated(tmp_path)
    process, out = run_sequence(scene, tmp_path, "ok", ["--order", "runtime::bottle-02,runtime::bottle-01"])
    process.communicate(timeout=60)
    r = json.loads((out / "sequence_plan_check.json").read_text())
    assert r["sequence_order"] == ["runtime::bottle-02", "runtime::bottle-01"]
    first, second = r["objects"]
    assert first["outcome"] == "PLANNABLE" and first["revision_before"] == 0 and first["revision_after"] == 1
    assert second["outcome"] == "COLLISION" and second["revision_before"] == 1
    assert second["blocked_by_virtually_placed"] == ["runtime::bottle-02"]
    assert "single authored placement pose" in second["reason"]
    assert r["sequence_status"] == "SEQUENCE_BLOCKED" and r["first_blocking"]["position"] == 2
    assert r["virtual_scene"][0]["object_id"] == "runtime::bottle-02" and r["final_revision"] == 1
    assert r["planned_not_executed"] is True and r["execution_attempted"] is False
    assert [o["object_id"] for o in r["obstacles"]] == ["runtime::cup-01"]


def test_sequence_never_plannable_when_supervisor_did_not_pass(tmp_path):
    scene = generated(tmp_path)
    process, out = run_sequence(scene, tmp_path, "broken")
    process.communicate(timeout=60)
    r = json.loads((out / "sequence_plan_check.json").read_text())
    assert all(o["outcome"] != "PLANNABLE" for o in r["objects"])
    assert r["sequence_status"] == "SEQUENCE_BLOCKED"


def test_sequence_order_must_be_eligible_and_distinct(tmp_path):
    scene = generated(tmp_path)
    process, out = run_sequence(scene, tmp_path, "ok", ["--order", "runtime::cup-01,runtime::bottle-01"])
    process.communicate(timeout=60)
    r = json.loads((out / "sequence_plan_check.json").read_text())
    assert r["status"] == "BLOCKED" and r["reason"].startswith("SEQUENCE_ORDER_INVALID")
    assert not (out / "sequence").exists()


def test_sequence_cancel_stops_owned_launch(tmp_path):
    scene = generated(tmp_path)
    process, out = run_sequence(scene, tmp_path, "hang")
    for line in process.stdout:
        if "SEQUENCE_OBJECT_1" in line:
            break
    process.send_signal(signal.SIGTERM)
    process.communicate(timeout=90)
    r = json.loads((out / "sequence_plan_check.json").read_text())
    assert r["status"] == "CANCELLED"
    assert all(o["outcome"] == "BLOCKED" for o in r["objects"])
    assert r["sequence_status"] == "SEQUENCE_BLOCKED"
