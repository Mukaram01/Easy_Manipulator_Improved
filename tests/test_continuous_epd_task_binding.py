"""Strict TaskIntent v2 -> existing continuous EPD PlanningScene bridge preflight."""
from __future__ import annotations

import copy
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from continuous_epd_task_binding import load_bound_live_task, bound_epd_message_error


def _task():
    return {
        "schema": "workcell_builder_task_intent/v2",
        "perception": {"mode": "live_epd", "camera_source": "simulated_rgbd"},
        "pick": {"selection": {
            "source_ref": "detected_objects/v1", "source_type": "perception",
            "zone_ref": "pick", "object_filter": {
                "class_id": "cup", "min_confidence": None, "max_age_seconds": 2.0}}},
        "place": {"target": {"asset_ref": "bin", "region_ref": "drop"}},
        "safety": {"require_fake_hardware": True, "real_hardware_enabled": False},
    }


def _environment():
    return {
        "camera": {
            "enabled": True, "camera_id": "cam", "frame_id": "camera_optical",
            "rgb_topic": "/sim/color/image_raw",
            "depth_topic": "/sim/aligned_depth/image_raw",
        },
        "perception": {"epd_input": {
            "topic": "/easy_perception_deployment/epd_localize_output",
            "message_type": "epd_msgs/msg/EPDObjectLocalization"}},
    }


def _generated(name: str):
    return {
        "cell": {"id": name, "planning_frame": "world"},
        "camera": {"id": "cam", "frame_id": "camera_optical"},
        "perception": {
            "camera_id": "cam", "frame_id": "camera_optical",
            "epd_input": {
                "topic": "/easy_perception_deployment/epd_localize_output",
                "message_type": "epd_msgs/msg/EPDObjectLocalization"}},
        "task": {"object_source": "perception", "pick": {"source_ref": "pick"},
                 "place": {"region_ref": "drop"},
                 "object_filter": {
                     "class_id": "cup", "max_age_seconds": 2.0, "min_confidence": None}},
        "environment": {
            "task_zones": [
                {"id": "pick", "frame": "world", "pose_xyz": [0, 0, 0],
                 "dimensions": [1, 1, 1]},
                {"id": "drop", "frame": "world", "pose_xyz": [0, 0, 0],
                 "dimensions": [1, 1, 1]},
            ]},
    }


def _write(scene, rel, obj):
    path = scene / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(obj), encoding="utf-8")


def make_scene(tmp_path):
    scene = tmp_path / "new_cell"
    scene.mkdir()
    _write(scene, "config/workcell_builder_task_intent.yaml", _task())
    _write(scene, "environment.yaml", _environment())
    _write(scene, "cell_definition.yaml", _generated(scene.name))
    return scene


def test_preflight_binds_saved_sim_camera_and_same_generated_task(tmp_path):
    result = load_bound_live_task(make_scene(tmp_path))
    assert result["scene_id"] == "new_cell"
    assert result["camera_source"] == "simulated_rgbd"
    assert result["camera_id"] == "cam"
    assert result["frame_id"] == "camera_optical"
    assert result["topic"] == "/easy_perception_deployment/epd_localize_output"
    assert result["message_type"] == "localization"
    assert result["target_class"] == "cup"
    assert result["source_identity_attested"] is False
    assert result["planning_ready"] is False
    assert result["robot_motion_commanded"] is False


@pytest.mark.parametrize("rel,modify,error", [
    ("config/workcell_builder_task_intent.yaml",
     lambda t: t["perception"].update(mode="replayed_snapshot"), "LIVE_TASK_REQUIRED"),
    ("config/workcell_builder_task_intent.yaml",
     lambda t: t["safety"].update(real_hardware_enabled=True), "FAKE_HARDWARE_REQUIRED"),
    ("config/workcell_builder_task_intent.yaml",
     lambda t: t["pick"]["selection"]["object_filter"].update(class_id="bottle"), "TASK_GENERATION_STALE"),
    ("config/workcell_builder_task_intent.yaml",
     lambda t: t["pick"]["selection"]["object_filter"].update(max_age_seconds=15), "TASK_GENERATION_STALE"),
    ("config/workcell_builder_task_intent.yaml",
     lambda t: t["place"]["target"].update(region_ref="different"), "TASK_GENERATION_STALE"),
    ("cell_definition.yaml",
     lambda c: c["perception"].update(camera_id="different"), "CAMERA_GENERATION_STALE"),
    ("cell_definition.yaml",
     lambda c: c["perception"]["epd_input"].update(topic="/wrong"), "CAMERA_GENERATION_STALE"),
    ("cell_definition.yaml",
     lambda c: c["task"]["object_filter"].update(min_confidence=0.01), "TASK_GENERATION_STALE"),
    ("cell_definition.yaml",
     lambda c: c["cell"].update(planning_frame="base_link"), "PLANNING_FRAME_UNSUPPORTED"),
    ("environment.yaml",
     lambda e: e["camera"].update(enabled=False), "CAMERA_NOT_CONFIGURED"),
])
def test_mismatched_saved_or_generated_contract_blocks(tmp_path, rel, modify, error):
    scene = make_scene(tmp_path)
    path = scene / rel
    payload = yaml.safe_load(path.read_text())
    modify(payload)
    path.write_text(yaml.safe_dump(payload))
    with pytest.raises(ValueError, match=error):
        load_bound_live_task(scene)


def _msg(frame="camera_optical", sec=50, nanosec=500_000_000):
    return SimpleNamespace(header=SimpleNamespace(
        frame_id=frame, stamp=SimpleNamespace(sec=sec, nanosec=nanosec)))


def test_frame_timestamp_and_monotonicity_gate():
    bound = {"frame_id": "camera_optical", "max_age_seconds": 2.0}
    now = 51_000_000_000
    assert bound_epd_message_error(_msg(), bound, now_ns=now) == ""
    assert "CAMERA_FRAME_MISMATCH" in bound_epd_message_error(_msg("other"), bound, now)
    assert "EPD_OUT_OF_ORDER" in bound_epd_message_error(
        _msg(), bound, now, last_stamp_ns=50_500_000_000)
    assert "EPD_STALE_OR_CLOCK_MISMATCH" in bound_epd_message_error(
        _msg(sec=1), bound, now)
    assert "EPD_STALE_OR_CLOCK_MISMATCH" in bound_epd_message_error(
        _msg(sec=70), bound, now)
    assert "EPD_STAMP_MISSING" in bound_epd_message_error(
        _msg(sec=0, nanosec=0), bound, now)
    assert "EPD_STAMP_INVALID" in bound_epd_message_error(
        _msg(sec=50, nanosec=1_000_000_000), bound, now)
    assert "EPD_STAMP_MISSING" in bound_epd_message_error(
        SimpleNamespace(header=SimpleNamespace(frame_id="camera_optical", stamp=None)), bound, now)


def test_existing_continuous_node_is_reused_not_forked():
    node = (ROOT / "scripts/epd_dynamic_planning_scene_node.py").read_text()
    assert "--task-intent-scene" in node
    assert "--preflight-only" in node
    assert "load_bound_live_task" in node
    assert "bound_epd_message_error" in node
    assert "build_collision_object" in node
    assert "apply_and_verify" in node
    assert "build_remove_collision_object" in node
    assert "scene_observed_ids" in node
    assert "geometry_blockers" in node
    assert "camera_source" in node
    assert "robot_motion_commanded" not in node or "motion_command_sent" in node
