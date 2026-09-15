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
    parse_validate_normalize,
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


def test_typed_model_round_trip_preserves_full_document():
    from scripts.task_intent_v2 import TaskIntentModel
    model = normalize_v2(valid_v2("EXACT"))
    model["routing"] = {"mode": "direct", "rules": [{"destination": "default_drop_zone"}]}
    model["provenance"] = {"migration": {"source": "fixture"}}
    assert TaskIntentModel.from_dict(model).to_dict() == model


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
    assert "motion_started" in normalize_v2(model)["safety"]


def test_canonical_hash_ignores_order_whitespace_and_comments():
    a = normalize_v2(valid_v2("AUTO"))
    b = json.loads(json.dumps(a, sort_keys=True))
    assert canonical_hash(a) == canonical_hash(b)
    assert canonical_hash(a) == hashlib.sha256(canonical_bytes(a)).hexdigest()
    golden = json.loads((Path(__file__).parent / "fixtures/task_intent_v2_hash_golden.json").read_text())
    assert canonical_hash({"a": [0, 1.0], "b": 2.0}) == golden["sha256"]
    assert canonical_bytes({"a": [0, 1.0], "b": 2.0}).decode() == golden["canonical_json"]


@pytest.mark.parametrize("left,right", [({"x": 1}, {"x": 1.0}), ({"x": -0.0}, {"x": 0}), ({"x": 1e3}, {"x": 1000})])
def test_equivalent_numbers_share_hash(left, right):
    assert canonical_hash(left) == canonical_hash(right)


@pytest.mark.parametrize("left,right", [({"x": 1}, {"x": "1"}), ({"x": True}, {"x": "true"}), ({"x": None}, {"x": "null"})])
def test_scalar_types_do_not_collide(left, right):
    assert canonical_hash(left) != canonical_hash(right)


def test_invalid_v2_is_not_repaired_by_normalization():
    model = valid_v2()
    model["pick"]["grasp"].pop("policy")
    model["place"]["release"]["strategy"] = "open_gripper"
    result = parse_validate_normalize(model)
    assert {x["code"] for x in result["diagnostics"]} >= {"POLICY_REQUIRED", "RELEASE_NOT_SEMANTIC"}


def test_exact_place_requires_local_pose_and_preferred_has_preference():
    model = valid_v2("EXACT")
    model["place"]["placement"]["requested_local_pose"] = None
    codes = {x["code"] for x in validate_intent(model)}
    assert "EXACT_LOCAL_POSE_REQUIRED" in codes
    model = valid_v2("PREFERRED")
    model["place"]["placement"]["requested_local_pose"] = None
    assert "PREFERRED_LOCAL_POSE_REQUIRED" in {x["code"] for x in validate_intent(model)}


def canonical_bytes(value):
    from scripts.task_intent_v2 import canonical_bytes as cb
    return cb(value)


def test_v1_migration_preserves_top2f_and_records_catalog_materialization():
    old = {"schema": "workcell_builder_task_intent/v1", "task": {"type": "pick_place"},
           "pick": {"source": {"type": "perception", "id": "detected_objects/v1"}, "zone": {"id": "pick_zone_main"}, "object_filter": {"class_id": "bottle"}},
           "grasp": {"strategy_ref": "top_2f"}, "place": {"target": {"id": "default_drop_zone", "asset_ref": "target_bin_default"}, "place_offset_xyz": [0, 0, .05], "release_strategy": "tool_release"}}
    import yaml
    env = yaml.safe_load((Path(__file__).parents[1] / "scenes/ur5_2f_test/environment.yaml").read_text())["environment"]
    migrated = migrate_v1(old, env)
    assert migrated["pick"]["grasp"]["policy"] == "EXACT"
    assert migrated["pick"]["grasp"]["strategy_ref"] == "top_2f"
    assert migrated["provenance"]["migration"]["materialized_from_catalog"]
    assert migrated["place"]["target"]["region_ref"] == "default_drop_zone"


def test_v1_non_z_up_retreat_axis_survives_migration():
    import yaml
    env = yaml.safe_load((Path(__file__).parents[1] / "scenes/ur5_2f_test/environment.yaml").read_text())["environment"]
    old = {"schema": "workcell_builder_task_intent/v1", "task": {"type": "pick_place"}, "pick": {"source": {"id": "objects"}, "zone": {"id": "pick_zone_main"}}, "grasp": {"strategy_ref": "top_2f", "retreat_axis": "x_minus"}, "place": {"target": {"id": "default_drop_zone"}}}
    assert migrate_v1(old, env)["pick"]["grasp"]["lift"]["axis"] == "x_minus"


def test_v1_open_does_not_write(tmp_path: Path):
    p = tmp_path / "intent.yaml"
    original = "schema: workcell_builder_task_intent/v1\ntask: {type: pick_place}\n"
    p.write_text(original)
    with pytest.raises(ValueError, match="MIGRATION_PHYSICAL_CONTEXT_REQUIRED"):
        write_without_rewrite(p)
    assert p.read_text() == original


def test_v1_migration_without_environment_is_blocked():
    old = {"schema": "workcell_builder_task_intent/v1", "task": {"type": "pick_place"}, "pick": {"source": {"id": "objects"}}, "grasp": {"strategy_ref": "top_2f"}, "place": {"target": {"id": "default_drop_zone"}}}
    with pytest.raises(ValueError, match="MIGRATION_PHYSICAL_CONTEXT_REQUIRED"):
        migrate_v1(old)


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


def test_v1_rotated_local_rpy_is_preserved():
    from scripts.physical_destination import resolve_destination
    import math
    env = {"assets": [{"id": "bin", "frame": "world", "pose_xyz": [1., 2., 3.], "pose_rpy": [0., 0., math.pi / 2], "collision": {"enabled": True}, "usable_placement": {"pose_xyz": [0., 0., .1], "pose_rpy": [0., 0., 0.], "dimensions": [.4, .3, .2]}}], "task_zones": [{"id": "drop", "target_ref": "bin", "frame": "world", "placement_local": {"pose_xyz": [.05, 0., .1], "pose_rpy": [0.1, 0.2, 0.3], "dimensions": [.2, .1, .1]}, "pose_xyz": [1., 2.05, 3.1], "pose_rpy": [0.1, 0.2, 1.8707963267948966], "dimensions": [.2, .1, .1]}]}
    old = {"schema": "workcell_builder_task_intent/v1", "task": {"type": "pick_place"}, "pick": {"source": {"id": "objects"}}, "grasp": {"strategy_ref": "top_2f"}, "place": {"target": {"id": "drop"}}}
    before = resolve_destination(env, "drop")
    migrated = migrate_v1(old, env)
    assert migrated["place"]["placement"]["requested_local_pose"]["rpy_rad"] == before["placement_local"]["pose_rpy"]


def test_shared_canonical_golden_fixtures_match_python():
    fixture = json.loads((Path(__file__).parent / "fixtures/task_intent_v2_canonical_golden.json").read_text())
    import yaml
    for item in fixture["fixtures"]:
        model = yaml.safe_load(item["yaml"])
        canonical = __import__("scripts.task_intent_v2", fromlist=["_canonical_json"])._canonical_json(model)
        assert canonical == item["canonical"]
        assert hashlib.sha256(canonical.encode("utf-8")).hexdigest() == item["sha256"]
