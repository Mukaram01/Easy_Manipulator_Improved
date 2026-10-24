"""No-EPD multi-object eligibility queue: explicit replay and no false planning."""
from __future__ import annotations

import copy
from pathlib import Path

import pytest
import yaml

from test_task_intent_resolver import valid_intent
from test_task_intent_authoring import scene as scene_fixture
from scripts.simulation_batch_preview import preview_batch


def prepare(tmp_path, *, source_mode="off", target="bottle"):
    root = scene_fixture(tmp_path)
    task = valid_intent()
    task["pick"]["selection"]["object_filter"]["class_id"] = target
    task["pick"]["selection"]["object_filter"]["min_confidence"] = None
    task["perception"] = {"mode": source_mode, "replay_path": "commissioning.yaml"}
    if source_mode == "off":
        task["pick"]["selection"].update(source_type="manual_simulated", source_ref="pick_zone_main")
    (root / "config/workcell_builder_task_intent.yaml").write_text(yaml.safe_dump(task))
    recording = {
        "schema_version": "detected_objects/v1",
        "source": {"mode": "replayed_snapshot", "description": "explicit static commissioning fixture"},
        "objects": [
            {"object_id": "cup-01", "class_id": "cup", "confidence": 0.99,
             "age_seconds": 0, "pose": {"frame_id": "world", "xyz": [0.25, -0.10, 0.05],
                                      "rpy": [0, 0, 0]}, "dimensions": [0.06, 0.06, 0.10]},
            {"object_id": "bottle-01", "class_id": "bottle", "confidence": 0.95,
             "age_seconds": 0, "pose": {"frame_id": "world", "xyz": [0.4, -0.08, 0.10],
                                      "rpy": [0, 0, 0]}, "dimensions": [0.06, 0.06, 0.18]},
            {"object_id": "bottle-02", "class_id": "bottle", "confidence": 0.91,
             "age_seconds": 0, "pose": {"frame_id": "world", "xyz": [0.1, 0.1, 0.10],
                                      "rpy": [0, 0, 0]}, "dimensions": [0.06, 0.06, 0.18]},
        ],
    }
    (root / "commissioning.yaml").write_text(yaml.safe_dump(recording))
    return root, recording


def test_two_bottles_queued_cup_retained_as_obstacle(tmp_path):
    scene, _ = prepare(tmp_path)
    task_file = scene / "config/workcell_builder_task_intent.yaml"
    recording_file = scene / "commissioning.yaml"
    original = task_file.read_bytes(), recording_file.read_bytes()
    result = preview_batch(scene, now=100.0)
    assert result["status"] == "ELIGIBILITY_ONLY"
    assert result["total_observed"] == 3
    assert result["total_eligible"] == 2
    assert [x["object_id"] for x in result["queue"]] == [
        "runtime::bottle-01", "runtime::bottle-02"]
    assert result["queue"][0]["destination_region_ref"] == "drop"
    other = next(o for o in result["observed_objects"] if o["object_id"] == "runtime::cup-01")
    assert other["role"] == "NON_TARGET_OBSTACLE"
    assert other["reason"] == "CLASS_OR_COLOUR_MISMATCH"
    assert result["epd_used"] is False
    assert result["simulator_live_connected"] is False
    assert result["moveit_planning_attempted"] is False
    assert result["motion_commanded"] is False
    assert result["physical_grasp_verified"] is False
    assert original == (task_file.read_bytes(), recording_file.read_bytes())


def test_bounded_queue_never_discards_other_objects(tmp_path):
    scene, _ = prepare(tmp_path)
    result = preview_batch(scene, limit=1, now=100.0)
    assert [x["object_id"] for x in result["queue"]] == ["runtime::bottle-01"]
    assert result["not_queued_due_to_limit"] == 1
    assert len(result["observed_objects"]) == 3
    assert all(o["planning_scene_verified"] is False for o in result["observed_objects"])


def test_can_preview_saved_recorded_perception_mode_without_epd(tmp_path):
    scene, _ = prepare(tmp_path, source_mode="replayed_snapshot")
    result = preview_batch(scene, now=100.0)
    assert result["status"] == "ELIGIBILITY_ONLY"
    assert result["source_mode"] == "explicit_commissioning_replay"


def test_no_target_reports_blocker_without_faking_result(tmp_path):
    scene, _ = prepare(tmp_path, target="plate")
    result = preview_batch(scene, now=100.0)
    assert result["status"] == "BLOCKED"
    assert result["queue"] == []
    assert result["reason"].startswith("NO_ELIGIBLE_TARGETS")
    assert len(result["observed_objects"]) == 3


def test_outside_zone_never_selected(tmp_path):
    scene, recording = prepare(tmp_path)
    recording["objects"][1]["pose"]["xyz"] = [6.0, 0, 0]
    (scene / "commissioning.yaml").write_text(yaml.safe_dump(recording))
    result = preview_batch(scene, now=100.0)
    assert result["total_eligible"] == 1
    outside = next(o for o in result["observed_objects"] if o["object_id"] == "runtime::bottle-01")
    assert outside["reason"] == "OUTSIDE_PICK_VOLUME"


@pytest.mark.parametrize("modify,error", [
    (lambda root, t, r: t["perception"].update(replay_path="not-found.yaml"), "not-found"),
    (lambda root, t, r: t["perception"].update(mode="live_epd"), "SIMULATION_SOURCE_REQUIRED"),
    (lambda root, t, r: t["safety"].update(real_hardware_enabled=True), "FAKE_HARDWARE_REQUIRED"),
    (lambda root, t, r: r["source"].update(mode="live_epd"), "REPLAY_INPUT_REQUIRED"),
    (lambda root, t, r: r["objects"][1].update(dimensions=[-1, .02, .03]), "dimensions"),
    (lambda root, t, r: r["objects"][1]["pose"].update(frame_id="camera"), "world transform"),
    (lambda root, t, r: r["objects"][1].update(object_id="cup-01"), "unique"),
])
def test_missing_invalid_or_unsafe_sources_fail_closed(tmp_path, modify, error):
    scene, recording = prepare(tmp_path)
    task_path = scene / "config/workcell_builder_task_intent.yaml"
    task = yaml.safe_load(task_path.read_text())
    modify(scene, task, recording)
    task_path.write_text(yaml.safe_dump(task))
    (scene / "commissioning.yaml").write_text(yaml.safe_dump(recording))
    with pytest.raises((ValueError, FileNotFoundError), match=error):
        preview_batch(scene, now=100.0)


def test_invalid_queue_limit_is_not_silently_clamped(tmp_path):
    scene, _ = prepare(tmp_path)
    for invalid in (0, -1, 101, 1.0, True):
        with pytest.raises(ValueError, match="QUEUE_LIMIT_INVALID"):
            preview_batch(scene, limit=invalid, now=100.0)


def test_future_or_expired_replayed_objects_never_enter_queue(tmp_path):
    scene, recording = prepare(tmp_path)
    recording["objects"][1]["age_seconds"] = 300
    recording["objects"][2]["age_seconds"] = -2
    (scene / "commissioning.yaml").write_text(yaml.safe_dump(recording))
    # Existing normalizer refuses future data rather than inventing a valid stamp.
    with pytest.raises(ValueError, match="future timestamp"):
        preview_batch(scene, now=100.0)
    recording["objects"][2]["age_seconds"] = 0
    (scene / "commissioning.yaml").write_text(yaml.safe_dump(recording))
    result = preview_batch(scene, now=100.0)
    assert [item["object_id"] for item in result["queue"]] == ["runtime::bottle-02"]
    assert any(o["reason"] == "STALE_OR_FUTURE_OBSERVATION" for o in result["observed_objects"])


def test_queue_lists_non_overlapping_places_the_operator_can_choose(tmp_path):
    scene, _ = prepare(tmp_path, target="cup")
    task_file = scene / "config/workcell_builder_task_intent.yaml"
    task = yaml.safe_load(task_file.read_text())
    task["place"]["placement"]["clearance_m"] = 0.0
    task_file.write_text(yaml.safe_dump(task))
    options = preview_batch(scene, now=100.0)["queue"][0]["place_options"]
    assert [o["id"] for o in options] == ["place-1", "place-2", "place-3"]
    # Evenly spread, size + clearance + planner tolerance apart and from the walls.
    assert [o["local_pose"]["xyz_m"] for o in options] == [
        pytest.approx([-0.017, 0.0, 0.10]), pytest.approx([0.05, 0.0, 0.10]), pytest.approx([0.117, 0.0, 0.10])]
    assert [o["label"] for o in options] == [
        "Place 1 of 3 — near end", "Place 2 of 3 — middle", "Place 3 of 3 — far end"]


def test_objects_too_tall_for_the_region_get_no_place_options(tmp_path):
    scene, _ = prepare(tmp_path)  # 0.18 m bottles, 0.10 m tall drop region
    assert all(q["place_options"] == [] for q in preview_batch(scene, now=100.0)["queue"])


# ---------------------------------------------------------------- Feature 4E saved sequence
from scripts.simulation_batch_preview import review_sequence  # noqa: E402
from scripts.task_intent_v2 import validate_intent  # noqa: E402


def sequence_scene(tmp_path, cups=2):
    """Saved task with `cups` eligible 0.10 m cups; zero clearance so they fit the 0.10 m tall drop region."""
    scene, recording = prepare(tmp_path, source_mode="replayed_snapshot", target="cup")
    recording["objects"] = [dict(recording["objects"][0], object_id=f"cup-0{k}",
                                 pose={"frame_id": "world", "xyz": [0.2 + 0.1 * k, -0.1, 0.05], "rpy": [0, 0, 0]})
                            for k in range(1, cups + 1)]
    (scene / "commissioning.yaml").write_text(yaml.safe_dump(recording))
    task_file = scene / "config/workcell_builder_task_intent.yaml"
    task = yaml.safe_load(task_file.read_text())
    task["place"]["placement"]["clearance_m"] = 0.0
    task_file.write_text(yaml.safe_dump(task))
    return scene, task


def save_sequence(scene, task, places):
    """What the Task tab writes: queue bindings plus each object's placement (None = AUTO)."""
    preview = preview_batch(scene, now=100.0)
    observed = {o["object_id"]: o for o in preview["observed_objects"]}
    target = task["place"]["target"]
    items = []
    for oid, local in places:
        obj = observed[oid]
        placement = {"mode": "AUTO"} if local is None else {
            "mode": "MANUAL", "asset_ref": target["asset_ref"], "region_ref": target["region_ref"], "local_pose": local}
        items.append({"object_id": oid, "class_id": obj["class_id"], "observed_xyz_m": obj["world_pose"][:3],
                      "dimensions_m": obj["dimensions_m"], "placement": placement})
    task["sequence"] = {"source_sha256": preview["source_sha256"], "items": items}
    (scene / "config/workcell_builder_task_intent.yaml").write_text(yaml.safe_dump(task))
    return task


NEAR = {"xyz_m": [-0.017, 0.0, 0.10], "rpy_rad": [0.0, 0.0, 0.0]}
MID = {"xyz_m": [0.05, 0.0, 0.10], "rpy_rad": [0.0, 0.0, 0.0]}
FAR = {"xyz_m": [0.117, 0.0, 0.10], "rpy_rad": [0.0, 0.0, 0.0]}
C1, C2, C3, C4 = (f"runtime::cup-0{k}" for k in range(1, 5))


def test_no_saved_sequence_keeps_queue_order_and_automatic_places(tmp_path):
    scene, task = sequence_scene(tmp_path)
    review = preview_batch(scene, now=100.0)["sequence_review"]
    assert review["status"] == "NOT_CONFIGURED" and review["order"] == [] and review["placements"] == {}


def test_saved_sequence_reopens_with_order_bindings_and_reserved_manual_place(tmp_path):
    scene, task = sequence_scene(tmp_path)
    task = save_sequence(scene, task, [(C2, None), (C1, NEAR)])  # Reordered; cup-01 manual.
    assert validate_intent(task) == []
    review = review_sequence(scene, task, now=100.0)
    assert review["status"] == "READY", review["summary"]
    assert review["order"] == [C2, C1] and review["placements"] == {C1: NEAR}
    auto, manual = review["items"]
    assert manual["place_label"] == "Place 1 of 3 — near end"
    # cup-02 comes first, yet keeps clear of the near end reserved for cup-01.
    assert auto["expected_local_pose"]["xyz_m"][0] > NEAR["xyz_m"][0] + 0.06
    assert auto["place_label"] in ("Automatic (expected: middle)", "Automatic (next free place)")


def test_changed_recording_blocks_instead_of_rebinding(tmp_path):
    scene, task = sequence_scene(tmp_path)
    task = save_sequence(scene, task, [(C1, FAR), (C2, NEAR)])
    recording = yaml.safe_load((scene / "commissioning.yaml").read_text())
    recording["objects"][0]["pose"]["xyz"][1] = -0.12  # Same IDs, different detections.
    (scene / "commissioning.yaml").write_text(yaml.safe_dump(recording))
    review = review_sequence(scene, task, now=100.0)
    assert review["status"] == "BLOCKED"
    assert all(i["message"].startswith("SEQUENCE_SOURCE_CHANGED") for i in review["items"])
    assert review["placements"] == {C1: FAR, C2: NEAR}  # Saved choices are reported, never replaced.
    assert [i["place_label"] for i in review["items"]] == ["Place 3 of 3 — far end", "Place 1 of 3 — near end"]


def test_unavailable_object_and_bad_places_need_review(tmp_path):
    scene, task = sequence_scene(tmp_path)
    task = save_sequence(scene, task, [(C1, NEAR), (C2, NEAR)])
    review = review_sequence(scene, task, now=100.0)
    assert review["status"] == "NEEDS_REVIEW" and review["items"][1]["message"].startswith("PLACE_CONFLICT")
    task["sequence"]["items"][1]["placement"]["local_pose"] = {"xyz_m": [0.13, 0.0, 0.10], "rpy_rad": [0, 0, 0]}
    review = review_sequence(scene, task, now=100.0)
    assert review["items"][1]["status"] == "NEEDS_REVIEW"
    assert review["items"][1]["message"].startswith("PLACE_OUTSIDE_REGION")
    task["sequence"]["items"][1]["placement"]["region_ref"] = "other_region"
    assert review_sequence(scene, task, now=100.0)["items"][1]["message"].startswith("DESTINATION_CHANGED")
    task["pick"]["selection"]["object_filter"]["class_id"] = "bottle"  # The saved cups are no longer picked.
    review = review_sequence(scene, task, now=100.0)
    assert review["items"][0]["message"].startswith("OBJECT_NOT_ELIGIBLE")


def test_destination_geometry_change_needs_review(tmp_path):
    scene, task = sequence_scene(tmp_path)
    task = save_sequence(scene, task, [(C1, FAR), (C2, None)])
    environment = yaml.safe_load((scene / "environment.yaml").read_text())
    drop = next(z for z in environment["task_zones"] if z["id"] == "drop")
    drop["placement_local"]["dimensions"][0] = drop["dimensions"][0] = 0.12  # Region shrunk around its centre.
    (scene / "environment.yaml").write_text(yaml.safe_dump(environment))
    review = review_sequence(scene, task, now=100.0)
    assert review["status"] == "NEEDS_REVIEW" and review["items"][0]["message"].startswith("PLACE_OUTSIDE_REGION")


def test_insufficient_capacity_is_reported_and_never_relaxed(tmp_path):
    scene, task = sequence_scene(tmp_path, cups=4)
    task = save_sequence(scene, task, [(C1, None), (C2, None), (C3, None), (C4, None)])
    review = review_sequence(scene, task, now=100.0)
    assert review["status"] == "BLOCKED"
    assert [i["status"] for i in review["items"]] == ["READY", "READY", "READY", "BLOCKED"]
    assert review["items"][3]["message"].startswith("INSUFFICIENT_CAPACITY")
    assert task["place"]["placement"]["clearance_m"] == 0.0  # Unchanged by the review.


@pytest.mark.parametrize("mutate", [
    lambda s: s.update(source_sha256="abc"),
    lambda s: s.update(items=[]),
    lambda s: s["items"].append(copy.deepcopy(s["items"][0])),
    lambda s: s["items"][0]["placement"].update(mode="EXACT"),
    lambda s: s["items"][0]["placement"].update(mode="MANUAL"),
    lambda s: s["items"][0].update(label="Place 1"),
])
def test_malformed_saved_sequence_is_an_authoring_error(tmp_path, mutate):
    scene, task = sequence_scene(tmp_path)
    task = save_sequence(scene, task, [(C1, None)])
    mutate(task["sequence"])
    assert [e["code"] for e in validate_intent(task)] == ["SEQUENCE_INVALID"]


def test_exact_policy_sharing_one_place_needs_review_and_null_sequence_is_none(tmp_path):
    scene, task = sequence_scene(tmp_path)
    task["place"]["placement"]["policy"] = "EXACT"
    task["place"]["placement"]["requested_local_pose"] = MID
    task = save_sequence(scene, task, [(C1, None), (C2, None)])
    review = review_sequence(scene, task, now=100.0)
    assert [i["status"] for i in review["items"]] == ["READY", "NEEDS_REVIEW"]
    assert review["items"][1]["message"].startswith("EXACT_PLACE_SHARED")
    task["sequence"] = None
    assert validate_intent(task) == [] and review_sequence(scene, task)["status"] == "NOT_CONFIGURED"


def test_custom_place_at_the_region_wall_needs_review(tmp_path):
    scene, task = sequence_scene(tmp_path)
    wall = {"xyz_m": [0.12, 0.0, 0.10], "rpy_rad": [0.0, 0.0, 0.0]}  # Fits, but leaves no placement tolerance.
    task = save_sequence(scene, task, [(C1, wall)])
    item = review_sequence(scene, task, now=100.0)["items"][0]
    assert item["status"] == "NEEDS_REVIEW" and "placement tolerance" in item["message"]
    assert item["place_label"] == "Custom place (x +0.120, y +0.000 m)"
