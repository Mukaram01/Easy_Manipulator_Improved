from __future__ import annotations
import json, time
from pathlib import Path

from scripts.live_epd_feed_bridge import parse_snapshot_json, map_live_snapshot, write_live_preview_artifacts, build_scene_status


def _snapshot(ts=None, conf=0.92):
    return {
      "schema_version":1,"source":"epd_live","runtime_mode":"live_adapter_metadata_only","camera":"realsense_d435i_1","camera_frame":"camera_color_optical_frame","timestamp_sec": ts if ts is not None else time.time(),
      "detections":[{"id":"det_001","class_label":"box","confidence":conf,"zone_hint":"detection_zone_1"}]
    }

def _env():
    return {"camera_placements":[{"name":"realsense_d435i_1"}],"work_zones":[{"name":"detection_zone_1"}],"conveyor_flows":[{"name":"flow_1","detection_zone":"detection_zone_1","estimated_distance_m":0.5,"speed_mps":0.25}]}


def test_snapshot_topic_parser_and_malformed_json():
    parsed = parse_snapshot_json(json.dumps(_snapshot()))
    assert parsed["schema_version"] == 1
    try:
        parse_snapshot_json("not-json")
        assert False
    except ValueError as exc:
        assert "malformed detection JSON" in str(exc)


def test_live_bridge_mapping_and_time_to_pick_and_no_motion():
    mapping = map_live_snapshot(_snapshot(), _env(), {"max_time_to_pick_s": 5.0})
    assert mapping["detections_mapped"]
    assert "time_to_pick_s" in mapping["detections_mapped"][0]
    assert mapping["robot_motion_commanded"] is False


def test_stale_snapshot_warn_and_confidence_threshold_and_artifacts_and_status_and_ui_strings(tmp_path: Path):
    old = _snapshot(ts=time.time() - 60.0, conf=0.2)
    mapping = map_live_snapshot(old, _env(), {"max_time_to_pick_s": 5.0}, min_confidence=0.5)
    status = build_scene_status(mapping, old["timestamp_sec"], now=time.time())
    assert status["Last EPD snapshot age"] == "WARN"
    assert any("confidence" in w for w in mapping["warnings"])

    out = write_live_preview_artifacts(tmp_path, old, mapping)
    for p in out.values():
        assert p.exists()

    scene_ui = Path("workcell_builder/workcell_builder/gui/scene_select.ui").read_text(encoding="utf-8")
    for s in ["Enable Live EPD Feed Preview", "Disable Live EPD Feed Preview", "Refresh Live EPD Status", "live_adapter_metadata_only"]:
        assert s in scene_ui
