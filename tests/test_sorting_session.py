"""R2.4 sorting session: lifecycle, ledger, deterministic selection and evidence-based counting."""
from __future__ import annotations

import copy
import json

import pytest

from scripts import sorting_session as ss
from tests.test_class_routing import observation, routed_intent, rule, three_bins

RULES = [rule("cup", "drop_a", priority=10), rule("can", "drop_a"), rule("bottle", "drop_a"),
         rule("box", "drop_b"), rule("tray", "drop_c", enabled=False)]


def session(rules=RULES, policy="leave_in_place", attempts=2, mode="batch_replay"):
    intent = routed_intent(rules)
    intent["routing"].update(unrouted_policy=policy, max_attempts_per_object=attempts)
    intent["place"]["target"] = {"asset_ref": "bin_a", "region_ref": "drop_a"}
    return ss.SortingSession(intent, three_bins(), session_id="s1", mode=mode, clock=lambda: 100.0)


def frame(*objects):
    return {"schema_version": "detected_objects/v1", "source": {"mode": "replayed_snapshot"}}, list(objects)


def batch(s, *objects):
    snapshot, observations = frame(*objects)
    s.ingest_frame(snapshot, observations, source_sha256="f" * 64)


def obj(oid, cls, x, dims=(0.06, 0.05, 0.10)):
    item = observation(f"runtime::{oid}", cls, x=x, dimensions=dims)
    item["object_id"] = oid
    return item


def completed_summary(s, target, placed_xyz):
    """An executor summary as a state-verified fake-hardware cycle reports it."""
    return {
        "result": "PASS", **{flag: True for flag in ss.COMPLETION_FLAGS},
        "selected_object_id": target["id"],
        "execution_results": [{"stage": stage, "code": 1, "action_status": 4} for stage in ss.EXECUTE_STAGES],
        "fake_hardware_guard": {"real_hardware": False, "hardware_classes": ["mock_components/GenericSystem"]},
        "final_planning_scene": {"world_ids": [target["id"]], "attached_ids": []},
        "final_object_state": {"id": target["id"], "pose": list(placed_xyz) + [0.0, 0.0, 0.0, 1.0],
                               "dimensions": list(s.objects[target["id"]]["dimensions"])},
        "task_intent_resolution": {"place_resolution": {"route": {"class_id": target["class_id"],
                                                                    "region_ref": target["destination"]}}},
    }


def test_lifecycle_pause_and_stop_take_effect_at_the_cycle_boundary(tmp_path):
    s = session()
    s.transition("READY"); s.transition("RUNNING")
    control = tmp_path / "control.json"
    control.write_text(json.dumps({"commands": [{"seq": 1, "command": "pause"}]}))
    assert s.poll_control(control, cycle_active=True)[0]["outcome"] == "pausing at the cycle boundary"
    assert s.state == "PAUSING"
    assert s.poll_control(control, cycle_active=True) == []                  # a command applies once
    s.boundary()
    assert s.state == "PAUSED"
    control.write_text(json.dumps({"commands": [{"seq": 1, "command": "pause"}, {"seq": 2, "command": "resume"}]}))
    s.poll_control(control, cycle_active=False)
    assert s.state == "RUNNING"
    assert s.command("stop", cycle_active=True) == "stopping at the cycle boundary"
    assert s.state == "STOPPING" and "not an emergency stop" in s.state_reason
    s.boundary()
    assert s.state == "STOPPED"
    with pytest.raises(ss.SortingFault, match="SORTING_TRANSITION_INVALID"):
        s.transition("RUNNING")
    assert s.command("dance", cycle_active=False).startswith("unknown command")


def test_selection_is_deterministic_and_policies_are_explicit():
    s = session()
    batch(s, obj("cup-2", "cup", 0.40), obj("cup-1", "cup", 0.30), obj("box-1", "box", 0.50),
          obj("tray-1", "tray", 0.60), obj("mug-1", "mug", 0.70),
          obj("bottle-1", "bottle", 0.80, dims=(0.06, 0.06, 0.30)))
    first = s.next_target()
    assert first["class_id"] == "cup" and first["destination"] == "drop_a"     # priority 10 first
    assert s.objects["runtime::mug-1"]["status"] == ss.SKIPPED_UNROUTED
    assert s.objects["runtime::tray-1"]["status"] == ss.SKIPPED_DISABLED
    assert s.objects["runtime::bottle-1"]["status"] == ss.PENDING               # judged only when it is next
    for oid in ("runtime::cup-1", "runtime::cup-2", "runtime::box-1"):
        s.objects[oid]["status"] = ss.REJECTED                                  # (as if failed) to reach the bottle
    assert s.next_target() is None
    assert s.objects["runtime::bottle-1"]["status"] == ss.REJECTED              # cannot fit its own bin
    assert s.objects["runtime::bottle-1"]["rejected_before_planning"] is True
    strict = session(policy="stop_session")
    batch(strict, obj("cup-1", "cup", 0.30), obj("mug-1", "mug", 0.70))
    with pytest.raises(ss.SortingFault, match="UNROUTED_OBJECT_PRESENT"):
        strict.next_target()


def test_only_verified_cycles_count_and_a_completed_object_is_never_picked_again():
    s = session()
    batch(s, obj("cup-1", "cup", 0.30), obj("cup-2", "cup", 0.40), obj("box-1", "box", 0.50))
    s.transition("READY"); s.transition("RUNNING")
    target = s.next_target(); s.begin_cycle(target)
    placed = [1.0, 2.0, 3.06]
    assert s.record_cycle(completed_summary(s, target, placed), 0, [4] * 9) == ss.COMPLETED
    entry = s.objects[target["id"]]
    assert entry["status"] == ss.COMPLETED and entry["pose"][:3] == placed and entry["destination"] == "drop_a"
    second = s.next_target()
    assert second["id"] != target["id"] and second["class_id"] == "cup"        # another instance of the class
    snapshot = s.world_snapshot({"source": {"mode": "replayed_snapshot", "description": "batch"}})
    moved = next(o for o in snapshot["objects"] if o["object_id"] == entry["object_id"])
    assert moved["pose"]["xyz"] == placed                                    # observed where it was placed
    assert snapshot["source"]["sorting_session"]["completed_object_ids"] == [target["id"]]
    # The placed object occupies bin A, so the next cup's place is resolved around it.
    assert any(o["id"] == target["id"] and o["pose"][:3] == placed for o in s.occupied())
    counts = s.counts()
    assert counts["completed"] == 1 and counts["per_class"]["cup"]["completed"] == 1
    assert counts["per_destination"] == {"drop_a": 1}


def test_unverified_motion_faults_and_failures_retry_then_reject():
    s = session(attempts=2)
    batch(s, obj("cup-1", "cup", 0.30), obj("box-1", "box", 0.50))
    target = s.next_target(); s.begin_cycle(target)
    bad = completed_summary(s, target, [1.0, 2.0, 3.06]); bad["detach_verified"] = False
    with pytest.raises(ss.SortingFault, match="CYCLE_EXECUTION_UNVERIFIED"):
        s.record_cycle(bad, 1, [4] * 9)
    assert s.counts()["completed"] == 0                                        # never counted
    s = session(attempts=2)
    batch(s, obj("cup-1", "cup", 0.30))
    for expected in ("RETRY", ss.REJECTED):
        target = s.next_target(); s.begin_cycle(target)
        failure = {"result": "FAIL", "execution_attempted": False, "failure": "NO_FEASIBLE_CYCLE: all candidates failed"}
        assert s.record_cycle(failure, 1, []) == expected
    assert s.next_target() is None and s.counts()["failed"] == 1
    s = session(attempts=5)
    batch(s, obj("cup-1", "cup", 0.30))
    target = s.next_target(); s.begin_cycle(target)
    assert s.record_cycle({"execution_attempted": False, "failure": "TASK_CONSTRAINT_UNSATISFIED: aperture"}, 1, []) == ss.REJECTED


def test_a_full_destination_waits_and_the_batch_ends():
    s = session([rule("cup", "drop_a")])
    batch(s, obj("cup-1", "cup", 0.30), obj("big", "cup", 0.60, dims=(0.06, 0.05, 0.10)))
    s.objects["runtime::big"]["status"] = ss.COMPLETED                         # a placed object filling bin A
    s.objects["runtime::big"].update(pose=[1.05, 2.0, 3.06, 0, 0, 0, 1], dimensions=[0.20, 0.10, 0.10])
    assert s.next_target() is None
    assert s.objects["runtime::cup-1"]["status"] == ss.WAITING_FULL
    assert "DESTINATION_FULL" in s.objects["runtime::cup-1"]["reason"]


def test_frames_are_reconciled_and_stale_frames_never_cause_a_second_pick():
    s = session()
    batch(s, obj("cup-1", "cup", 0.30))
    with pytest.raises(ss.SortingFault, match="BATCH_FRAME_ALREADY_CONSUMED"):
        batch(s, obj("cup-1", "cup", 0.30))
    live = session(mode="continuous")
    snapshot, observations = frame(obj("cup-1", "cup", 0.30), obj("can-1", "can", 0.50))
    live.ingest_frame(snapshot, observations, source_sha256="a" * 64)
    target = live.next_target(); live.begin_cycle(target)
    live.record_cycle(completed_summary(live, target, [1.0, 2.0, 3.06]), 0, [4] * 9)
    stale, stale_obs = frame(obj("cup-1", "cup", 0.30))                        # an old frame: cup back on the table
    with pytest.raises(ss.SortingFault, match="PROCESSED_OBJECT_STILL_OBSERVED"):
        live.ingest_frame(stale, stale_obs, source_sha256="b" * 64)
    fresh, fresh_obs = frame(obj("cup-3", "cup", 0.40))                        # can-1 is gone, a new cup arrived
    live.ingest_frame(fresh, fresh_obs, source_sha256="c" * 64)
    assert live.objects["runtime::can-1"]["status"] == ss.REMOVED
    assert live.next_target()["id"] == "runtime::cup-3"


def test_the_executor_reconciles_a_sorting_world_and_refuses_any_disagreement():
    from scripts.perceived_object_grasp_execute import observations_to_insert
    placed = {"id": "runtime::cup-1", "pose": [1.0, 2.0, 3.06, 0, 0, 0, 1], "dimensions": [0.06, 0.05, 0.10]}
    new = {"id": "runtime::cup-2", "pose": [0.4, -0.1, 0.05, 0, 0, 0, 1], "dimensions": [0.06, 0.05, 0.10]}
    live = {placed["id"]: copy.deepcopy(placed)}
    assert observations_to_insert([placed, new], live, "fake", reconcile=True) == [new]
    with pytest.raises(RuntimeError, match="reset the fake scene"):
        observations_to_insert([placed, new], live, "fake")                    # ordinary runs still refuse
    moved = dict(placed, pose=[1.0, 2.0, 3.20, 0, 0, 0, 1])
    with pytest.raises(RuntimeError, match="geometry changed"):
        observations_to_insert([moved, new], live, "fake", reconcile=True)
    with pytest.raises(RuntimeError, match="unknown prior"):
        observations_to_insert([new], live, "fake", reconcile=True)


def test_physics_stays_blocked_until_a_qualified_scope_covers_the_task():
    from scripts.simulator_world import load_catalog, physics_task_qualification
    from tests.test_class_routing import valid_intent
    catalog = load_catalog()
    assert "task_scope" not in catalog                                         # the Stage-A world declares no more
    routed = routed_intent(RULES)
    verdict = physics_task_qualification(routed, catalog, sorting_session=True)
    assert not verdict["qualified"] and verdict["code"] == "SORTING_PHYSICS_UNQUALIFIED"
    assert len(verdict["problems"]) == 3 and verdict["required"]["destinations"] == ["drop_a", "drop_b"]
    assert physics_task_qualification(valid_intent(), catalog)["qualified"]    # the qualified single cycle
    future = dict(catalog, task_scope={"class_routing": True, "max_destinations": 3, "continuous_sorting": True})
    assert physics_task_qualification(routed, future, sorting_session=True)["qualified"]   # no engine change
