import importlib.util
import sys
from pathlib import Path

import yaml

SCRIPT = Path("scripts/dynamic_object_planning_scene_bridge.py")
SPEC = importlib.util.spec_from_file_location("dynamic_object_planning_scene_bridge", SCRIPT)
BRIDGE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BRIDGE
SPEC.loader.exec_module(BRIDGE)


def snapshot(object_id="obj_001", frame="camera_frame", dimensions=None, position=None):
    return {"schema_version":"workcell_perception_snapshot/v1", "scene_id":"pick_place",
            "camera_id":"camera_01", "timestamp":"2026-08-28T00:00:00Z", "frame_id":frame,
            "objects":[{"object_id":object_id, "label":"part", "confidence":0.9,
                        "pose":{"frame_id":frame, "position":position or [0.1,0.2,0.3],
                                "orientation_xyzw":[0.0,0.0,0.0,1.0]},
                        "dimensions_xyz":[0.08,0.04,0.03] if dimensions is None else dimensions,
                        "shape":"box"}]}


def test_valid_dynamic_box_preserves_id_and_dimensions():
    result = BRIDGE.build_collision_object(snapshot(), "obj_001", "camera_frame")
    assert result.status == "PASS"
    assert result.collision_object.id == "obj_001"
    assert list(result.collision_object.primitives[0].dimensions) == [0.08,0.04,0.03]


def test_pose_is_transformed_into_planning_frame():
    def deterministic_tf(pose, target):
        assert pose.header.frame_id == "camera_frame"
        pose.header.frame_id = target
        pose.pose.position.x += 1.0
        pose.pose.position.y -= 2.0
        return pose
    result = BRIDGE.build_collision_object(snapshot(), "obj_001", "base_link", deterministic_tf)
    assert result.status == "PASS"
    assert result.collision_object.header.frame_id == "base_link"
    pose = result.collision_object.primitive_poses[0]
    assert [pose.position.x,pose.position.y,pose.position.z] == [1.1,-1.8,0.3]


def test_stable_identity_uses_add_for_update_without_new_id():
    first = BRIDGE.build_collision_object(snapshot(position=[0.1,0.2,0.3]), "obj_001", "camera_frame")
    second = BRIDGE.build_collision_object(snapshot(position=[0.4,0.5,0.6]), "obj_001", "camera_frame")
    assert first.collision_object.id == second.collision_object.id == "obj_001"
    assert first.collision_object.operation == second.collision_object.operation == first.collision_object.ADD
    assert second.collision_object.primitive_poses[0].position.x == 0.4


def test_different_object_ids_remain_distinct():
    first = BRIDGE.build_collision_object(snapshot("obj_001"), "obj_001", "camera_frame")
    second = BRIDGE.build_collision_object(snapshot("obj_002"), "obj_002", "camera_frame")
    assert {first.collision_object.id,second.collision_object.id} == {"obj_001","obj_002"}


def test_missing_dimensions_is_explicit_geometry_blocker():
    data = snapshot(); del data["objects"][0]["dimensions_xyz"]
    result = BRIDGE.build_collision_object(data, "obj_001", "camera_frame")
    assert result.status == "BLOCKED" and "geometry unavailable" in result.reason
    assert result.collision_object is None


def test_unavailable_tf_is_explicit_blocker():
    result = BRIDGE.build_collision_object(snapshot(), "obj_001", "base_link")
    assert result.status == "BLOCKED" and "TF unavailable" in result.reason
    assert result.collision_object is None


def test_non_finite_and_invalid_geometry_are_rejected():
    for dimensions in ([float("nan"),0.04,0.03], [0.08,0.0,0.03]):
        result = BRIDGE.build_collision_object(snapshot(dimensions=dimensions), "obj_001", "camera_frame")
        assert result.status == "FAIL" and result.collision_object is None


def test_existing_epd_fixture_reaches_collision_object_without_authored_dimensions():
    from epd_snapshot_adapter import normalize_detected_objects_snapshot
    detected = yaml.safe_load(Path("tests/fixtures/detected_objects/valid_epd_single_box.yaml").read_text())
    detected.update({"scene_id":"pick_place", "camera_id":"camera_01",
                     "timestamp":detected["source"]["captured_at"], "frame_id":detected["source"]["frame_id"]})
    profile = {"scene_id":"pick_place", "perception":{"camera":{"camera_id":"camera_01", "frame_id":detected["frame_id"]}}}
    normalized = normalize_detected_objects_snapshot(detected, profile)
    result = BRIDGE.build_collision_object(normalized, "obj_001", detected["frame_id"])
    assert result.status == "PASS"
    assert list(result.collision_object.primitives[0].dimensions) == [0.08,0.04,0.03]


def test_bridge_contains_no_perception_or_grasp_algorithm():
    source = SCRIPT.read_text(encoding="utf-8")
    assert not any(token in source for token in ["segment_point_cloud","detect_objects","classify_object","generate_grasp"])
