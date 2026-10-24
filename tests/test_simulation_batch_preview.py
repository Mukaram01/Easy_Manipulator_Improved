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
