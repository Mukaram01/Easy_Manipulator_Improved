"""Feature 3B2: safety-critical live EPD input handoff — no ROS required."""
from __future__ import annotations

import copy
import subprocess
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml

from scripts.capture_selected_live_epd import qualify_capture, source_binding, capture_task


NOW = 1791880000.0


def saved_task(max_age=5):
    return {
        "schema": "workcell_builder_task_intent/v2",
        "perception": {"mode": "live_epd", "camera_source": "simulated_rgbd"},
        "pick": {"selection": {"zone_ref": "pick", "object_filter": {"class_id": "cup", "max_age_seconds": max_age}}},
    }


def camera():
    return {"camera_source": "simulated_rgbd", "camera_id": "cam", "frame_id": "cam_optical"}


def captured(now=NOW):
    stamp = int((now - 0.5) * 1e9)
    return {
        "schema_version": "detected_objects/v1",
        "source": {"mode": "live_epd", "type": "epd_localization",
                   "frame_id": "cam_optical", "source_stamp_ns": stamp,
                   "transform": {"status": "PASS", "target_frame": "world"}},
        "objects": [
            {"object_id": "observed_cup", "class_id": "cup", "timestamp": stamp / 1e9,
             "pose": {"frame_id": "world", "xyz": [0.25, -0.1, 0.07], "rpy": [0, 0, 0]},
             "dimensions": {"x": 0.06, "y": 0.06, "z": 0.12}},
        ],
    }


def test_valid_live_candidate_retains_real_stamps_and_measured_geometry():
    original = captured()
    output = qualify_capture(original, saved_task(), camera(), now=NOW)
    assert output["source"]["camera_source"] == "simulated_rgbd"
    assert output["source"]["camera_source_verification"] == "configured_ros_topic_only_not_hardware_attested"
    assert output["objects"][0]["dimensions"] == {"x": 0.06, "y": 0.06, "z": 0.12}
    assert output["objects"][0]["timestamp"] == (NOW - 0.5)


@pytest.mark.parametrize("mutation,error", [
    (lambda p: p["source"].update(mode="replayed_snapshot"), "LIVE_INPUT_REQUIRED"),
    (lambda p: p["source"].update(frame_id="other_camera"), "CAMERA_FRAME_MISMATCH"),
    (lambda p: p["source"]["transform"].update(status="WARN"), "TF_UNVERIFIED"),
    (lambda p: p["source"]["transform"].update(target_frame="camera"), "TF_UNVERIFIED"),
    (lambda p: p["source"].pop("source_stamp_ns"), "EPD_STAMP_MISSING"),
    (lambda p: p["objects"][0].pop("dimensions"), "MEASURED_GEOMETRY_MISSING"),
    (lambda p: p["objects"][0].update(dimensions={"x": 0, "y": .1, "z": .1}), "MEASURED_GEOMETRY_MISSING"),
    (lambda p: p["objects"][0]["pose"].update(frame_id="cam_optical"), "POSE_UNTRANSFORMED"),
    (lambda p: p["objects"][0].update(timestamp=0), "OBJECT_STAMP_INVALID"),
    (lambda p: p.update(objects=[]), "OBJECTS_MISSING"),
])
def test_rejects_unqualified_capture(mutation, error):
    sample = captured()
    mutation(sample)
    with pytest.raises(ValueError, match=error):
        qualify_capture(sample, saved_task(), camera(), now=NOW)


def test_rejects_stale_and_future_sensor_timestamps():
    sample = captured(now=NOW - 30)
    with pytest.raises(ValueError, match="EPD_STALE"):
        qualify_capture(sample, saved_task(), camera(), now=NOW)
    sample = captured(now=NOW + 3)
    with pytest.raises(ValueError, match="EPD_STALE"):
        qualify_capture(sample, saved_task(), camera(), now=NOW)


def scene(tmp_path):
    (tmp_path / "config").mkdir()
    (tmp_path / "config/workcell_builder_task_intent.yaml").write_text(yaml.safe_dump(saved_task()))
    (tmp_path / "environment.yaml").write_text(yaml.safe_dump({
        "camera": {"enabled": True, "camera_id": "cam", "frame_id": "cam_optical",
                   "rgb_topic": "/sim/rgb", "depth_topic": "/sim/aligned_depth"},
        "perception": {"epd_input": {"topic": "/epd/localization",
                                      "message_type": "epd_msgs/msg/EPDObjectLocalization"}},
    }))
    return tmp_path


def test_source_binding_reuses_saved_task_scene_camera_and_epd_topic(tmp_path):
    root = scene(tmp_path)
    task, cam, epd, metadata = source_binding(root)
    assert task["perception"]["camera_source"] == "simulated_rgbd"
    assert cam["rgb_topic"] == "/sim/rgb"
    assert epd == {"topic": "/epd/localization", "mode": "localization"}
    assert metadata["epd_input"]["message_type"] == "epd_msgs/msg/EPDObjectLocalization"
    task["perception"]["mode"] = "replayed_snapshot"
    (root / "config/workcell_builder_task_intent.yaml").write_text(yaml.safe_dump(task))
    with pytest.raises(ValueError, match="LIVE_TASK_REQUIRED"):
        source_binding(root)


def test_source_binding_rejects_missing_epd_runtime_config(tmp_path):
    root = scene(tmp_path)
    document = yaml.safe_load((root / "environment.yaml").read_text())
    document.pop("perception")
    (root / "environment.yaml").write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="EPD_CONFIG_MISSING"):
        source_binding(root)


def test_candidate_is_new_and_never_overwrites_saved_task(tmp_path):
    root = scene(tmp_path)
    task_file = root / "config/workcell_builder_task_intent.yaml"
    before = task_file.read_bytes()
    def fake_runner(command, *, capture_output, text, timeout, check):
        assert command[command.index("--message-type") + 1] == "localization"
        path = Path(command[command.index("--output") + 1])
        assert root / "captures" / "epd" in path.parents
        path.write_text(yaml.safe_dump(captured()))
        return subprocess.CompletedProcess(command, 0, "", "")
    # Unit qualification uses the real capture parser but not a ROS graph.
    with patch("scripts.capture_selected_live_epd.source_binding",
               return_value=(saved_task(), camera(),
                             {"topic": "/epd/localization", "mode": "localization"}, {})), \
         patch("task_intent_authoring.planning_input", return_value=(Path("/unused"), False)):
        report = capture_task(root, runner=fake_runner, clock=lambda: NOW)
    assert report["status"] == "CAPTURE_QUALIFIED_FOR_TASK_INPUT"
    assert report["planning_ready"] is False
    assert report["robot_motion_commanded"] is False
    assert report["source_identity_attested"] is False
    assert Path(report["capture_path"]).is_file()
    assert task_file.read_bytes() == before


def test_failed_capture_never_uses_previous_file(tmp_path):
    root = scene(tmp_path)
    before = (root / "config/workcell_builder_task_intent.yaml").read_bytes()
    def failed_runner(command, **kwargs):
        return subprocess.CompletedProcess(command, 1, "", "missing TF")
    with patch("scripts.capture_selected_live_epd.source_binding",
               return_value=(saved_task(), camera(),
                             {"topic": "/epd/localization", "mode": "localization"}, {})):
        with pytest.raises(ValueError, match="EPD_CAPTURE_FAILED"):
            capture_task(root, runner=failed_runner, clock=lambda: NOW)
    assert (root / "config/workcell_builder_task_intent.yaml").read_bytes() == before
