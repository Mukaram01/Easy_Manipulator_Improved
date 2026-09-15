from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from scripts.task_intent_v2 import (
    canonical_hash,
    migrate_v1,
    normalize_v2,
    validate_intent,
    write_without_rewrite,
)


def valid_v2(policy: str = "AUTO") -> dict:
    return {
        "schema": "workcell_builder_task_intent/v2",
        "scene_package": "scenes/ur5_2f_test",
        "task": {"id": "t", "type": "pick_place", "template": "pick_place"},
        "pick": {
            "selection": {
                "source_ref": "detected_objects/v1", "source_type": "perception", "zone_ref": "pick_zone_main",
                "object_filter": {"class_id": "bottle", "color": None, "min_confidence": None, "max_age_seconds": 2.0},
            },
            "grasp": {"policy": policy, "required_capability": "two_finger_parallel", "strategy_ref": "top_2f" if policy != "AUTO" else None,
                      "approach": {"axis": "z_down", "distance_m": 0.12}, "orientation": {"mode": "vertical", "allowed_roll_deg": [0], "allowed_yaw_deg": [0]},
                      "tcp_offset_xyz_m": [0, 0, 0], "tcp_offset_rpy_rad": [0, 0, 0], "contact": {"required": True}, "aperture": {"min_m": 0, "max_m": .085}, "lift": {"axis": "z_up", "distance_m": .15}},
        },
        "place": {"target": {"asset_ref": "target_bin_default", "region_ref": "default_drop_zone"},
                  "placement": {"policy": policy, "requested_local_pose": {"xyz_m": [0, 0, .05], "rpy_rad": [0, 0, 0]} if policy != "AUTO" else None,
                                 "orientation": {"mode": "target_default", "rpy_rad": [0, 0, 0]}, "approach": {"axis": "z_down", "distance_m": .1}, "clearance_m": .05, "retreat": {"axis": "z_up", "distance_m": .1}},
                  "release": {"strategy": "tool_release"}},
        "safety": {"execution_mode": "simulation_preview", "require_fake_hardware": True, "real_hardware_enabled": False, "preview_policy": "diagnostic_if_unresolved"},
    }


def test_valid_policies_and_semantic_release():
    for policy in ("AUTO", "PREFERRED", "EXACT"):
        model = normalize_v2(valid_v2(policy))
        assert validate_intent(model) == []
        assert model["place"]["release"]["strategy"] == "tool_release"


def test_exact_missing_values_blocked_and_pick_is_sole_authority():
    model = valid_v2("EXACT")
    model["pick"]["grasp"].pop("orientation")
    model["task"]["target_policy"] = {"class_id": "cube", "max_age_seconds": 9}
    codes = {x["code"] for x in validate_intent(model)}
    assert "EXACT_REQUIRED_FIELD_MISSING" in codes
    assert "DUPLICATE_PICK_SELECTION_AUTHORITY" in codes


def test_unsupported_capability_and_invented_alias_rejected():
    model = valid_v2("AUTO")
    model["pick"]["grasp"]["required_capability"] = "vacuum_pick"
    model["pick"]["grasp"]["strategy_ref"] = "finger_side"
    codes = {x["code"] for x in validate_intent(normalize_v2(model))}
    assert {"UNSUPPORTED_CAPABILITY", "UNKNOWN_STRATEGY_ID"} <= codes


def test_v2_dynamic_runtime_state_rejected_and_not_serialized():
    model = valid_v2()
    model["safety"]["motion_started"] = False
    assert any(x["code"] == "DYNAMIC_SAFETY_IN_INTENT" for x in validate_intent(model))
    assert "motion_started" not in normalize_v2(model)["safety"]


def test_canonical_hash_ignores_order_whitespace_and_comments():
    a = normalize_v2(valid_v2("AUTO"))
    b = json.loads(json.dumps(a, sort_keys=True))
    assert canonical_hash(a) == canonical_hash(b)
    assert canonical_hash(a) == hashlib.sha256(canonical_bytes(a)).hexdigest()
    golden = json.loads((Path(__file__).parent / "fixtures/task_intent_v2_hash_golden.json").read_text())
    assert canonical_hash({"a": [0, 1.0], "b": 2.0}) == golden["sha256"]
    assert canonical_bytes({"a": [0, 1.0], "b": 2.0}).decode() == golden["canonical_json"]


def canonical_bytes(value):
    from scripts.task_intent_v2 import canonical_bytes as cb
    return cb(value)


def test_v1_migration_preserves_top2f_and_records_catalog_materialization():
    old = {"schema": "workcell_builder_task_intent/v1", "task": {"type": "pick_place"},
           "pick": {"source": {"type": "perception", "id": "detected_objects/v1"}, "zone": {"id": "pick_zone_main"}, "object_filter": {"class_id": "bottle"}},
           "grasp": {"strategy_ref": "top_2f"}, "place": {"target": {"id": "default_drop_zone", "asset_ref": "target_bin_default"}, "place_offset_xyz": [0, 0, .05], "release_strategy": "tool_release"}}
    migrated = migrate_v1(old)
    assert migrated["pick"]["grasp"]["policy"] == "EXACT"
    assert migrated["pick"]["grasp"]["strategy_ref"] == "top_2f"
    assert migrated["provenance"]["migration"]["materialized_from_catalog"]
    assert migrated["place"]["target"]["region_ref"] == "default_drop_zone"


def test_v1_open_does_not_write(tmp_path: Path):
    p = tmp_path / "intent.yaml"
    original = "schema: workcell_builder_task_intent/v1\ntask: {type: pick_place}\n"
    p.write_text(original)
    write_without_rewrite(p)
    assert p.read_text() == original


def test_v1_place_migration_materializes_r19_target_local_destination():
    from scripts.physical_destination import resolve_destination
    env = {"assets": [{"id": "bin", "frame": "world", "pose_xyz": [1., 2., 3.], "pose_rpy": [0., 0., 0.], "collision": {"enabled": True},
                       "usable_placement": {"pose_xyz": [0., 0., .1], "pose_rpy": [0., 0., 0.], "dimensions": [.4, .3, .2]}}],
           "task_zones": [{"id": "drop", "target_ref": "bin", "frame": "world", "placement_local": {"pose_xyz": [.05, 0., .1], "pose_rpy": [0., 0., 0.], "dimensions": [.2, .1, .1]},
                           "pose_xyz": [1.05, 2., 3.1], "pose_rpy": [0., 0., 0.], "dimensions": [.2, .1, .1]}]}
    old = {"schema": "workcell_builder_task_intent/v1", "task": {"type": "pick_place"}, "pick": {"source": {"id": "objects"}, "zone": {"id": "pick"}, "object_filter": {"class_id": "bottle"}},
           "grasp": {"strategy_ref": "top_2f"}, "place": {"target": {"id": "drop"}, "release_strategy": "tool_release"}}
    before = resolve_destination(env, "drop")
    migrated = migrate_v1(old, env)
    assert migrated["place"]["target"] == {"asset_ref": "bin", "region_ref": "drop"}
    assert migrated["place"]["placement"]["requested_local_pose"]["xyz_m"] == before["placement_local"]["pose_xyz"]
    assert migrated["provenance"]["migration"]["materialized_place_from_r19"] is True
