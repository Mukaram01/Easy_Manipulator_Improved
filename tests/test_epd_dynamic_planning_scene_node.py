import importlib.util
import sys
from pathlib import Path


SCRIPT = Path("scripts/epd_dynamic_planning_scene_node.py")
sys.path.insert(0, str(SCRIPT.parent.resolve()))
SPEC = importlib.util.spec_from_file_location("epd_dynamic_planning_scene_node", SCRIPT)
NODE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NODE)


def test_summary_tracks_update_without_duplicate_id():
    summary = NODE.initial_summary()
    NODE.record_verified(summary, "1")
    NODE.record_verified(summary, "1")
    NODE.record_verified(summary, "2")
    assert summary["objects_applied"] == 3
    assert summary["objects_updated"] == 1
    assert summary["planning_scene_verified_ids"] == ["1", "2"]
    assert summary["duplicate_ids"] == []


def test_runtime_uses_production_tracking_and_existing_seams_only():
    source = SCRIPT.read_text(encoding="utf-8")
    assert "EPDObjectTracking" in source
    assert "convert_epd_message_to_detected_objects" in source
    assert "normalize_detected_objects_snapshot" in source
    assert "build_collision_object" in source
    assert "apply_and_verify" in source
    assert "if replay" not in source
    assert "rs_launch" not in source
