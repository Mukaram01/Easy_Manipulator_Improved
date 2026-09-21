from __future__ import annotations

import copy
import importlib
from pathlib import Path

import yaml

from scripts.task_intent_v2 import normalized_intent_hash

ROOT = Path(__file__).resolve().parents[1]


def valid_intent(grasp_policy="AUTO", place_policy="AUTO"):
    return {
        "schema": "workcell_builder_task_intent/v2",
        "scene_package": "scenes/ur5_2f_test",
        "task": {"id": "fixture", "type": "pick_place", "template": "pick_place"},
        "pick": {
            "selection": {
                "source_ref": "detected_objects/v1",
                "source_type": "perception",
                "zone_ref": "pick_zone_main",
                "object_filter": {
                    "class_id": "bottle",
                    "color": None,
                    "min_confidence": 0.5,
                    "max_age_seconds": 2.0,
                },
            },
            "grasp": {
                "policy": grasp_policy,
                "required_capability": "two_finger_parallel",
                "strategy_ref": "top_2f" if grasp_policy != "AUTO" else None,
                "approach": {"axis": "z_down", "distance_m": 0.12},
                "orientation": {
                    "mode": "vertical",
                    "allowed_roll_deg": [0.0],
                    "allowed_yaw_deg": [0.0, 90.0, 180.0, 270.0],
                    "tolerance_rad": [0.0, 0.0, 0.0],
                },
                "tcp_offset_xyz_m": [0.0, 0.0, 0.0],
                "tcp_offset_rpy_rad": [0.0, 0.0, 0.0],
                "contact": {"required": True, "min_quality": 0.0},
                "aperture": {"min_m": 0.0, "max_m": 0.085},
                "lift": {"axis": "z_up", "distance_m": 0.15},
            },
        },
        "place": {
            "target": {"asset_ref": "bin", "region_ref": "drop"},
            "placement": {
                "policy": place_policy,
                "requested_local_pose": (
                    {"xyz_m": [0.05, 0.0, 0.10], "rpy_rad": [0.0, 0.0, 0.0]}
                    if place_policy != "AUTO" else None
                ),
                "orientation": {
                    "mode": "target_default",
                    "rpy_rad": [0.0, 0.0, 0.0],
                    "tolerance_rad": [0.0, 0.0, 0.0],
                },
                "approach": {"axis": "z_down", "distance_m": 0.10},
                "clearance_m": 0.01,
                "retreat": {"axis": "z_up", "distance_m": 0.10},
            },
            "release": {"strategy": "tool_release"},
        },
        "routing": {
            "mode": "direct",
            "rules": [{"id": "default_place", "when": {"always": True}, "destination": "drop"}],
        },
        "safety": {
            "execution_mode": "simulation_preview",
            "require_fake_hardware": True,
            "real_hardware_enabled": False,
            "preview_policy": "diagnostic_if_unresolved",
        },
    }


def environment():
    return {
        "assets": [
            {
                "id": "bin",
                "frame": "world",
                "pose_xyz": [1.0, 2.0, 3.0],
                "pose_rpy": [0.0, 0.0, 0.0],
                "collision": {"enabled": True},
                "usable_placement": {
                    "pose_xyz": [0.0, 0.0, 0.10],
                    "pose_rpy": [0.0, 0.0, 0.0],
                    "dimensions": [0.40, 0.30, 0.20],
                },
            }
        ],
        "task_zones": [
            {"id": "pick_zone_main", "frame": "world", "pose_xyz": [0., 0., 0.],
             "pose_rpy": [0., 0., 0.], "dimensions": [2., 2., 2.]},
            {
                "id": "drop",
                "target_ref": "bin",
                "frame": "world",
                "placement_local": {
                    "pose_xyz": [0.05, 0.0, 0.10],
                    "pose_rpy": [0.0, 0.0, 0.0],
                    "dimensions": [0.20, 0.10, 0.10],
                },
                "pose_xyz": [1.05, 2.0, 3.10],
                "pose_rpy": [0.0, 0.0, 0.0],
                "dimensions": [0.20, 0.10, 0.10],
            }
        ],
    }


def cell():
    return {"end_effector": {"id": "robotiq_85_gripper", "type": "finger"}}


def observations():
    return [
        {
            "id": "epd::bottle-1",
            "class_id": "bottle",
            "confidence": 0.91,
            "timestamp": 99.0,
            "frame_id": "world",
            "shape": "BOX",
            "pose": [0.4, -0.1, 0.1, 0.0, 0.0, 0.0, 1.0],
            "dimensions": [0.06, 0.05, 0.18],
        }
    ]


def resolver_module():
    path = ROOT / "scripts" / "task_intent_resolver.py"
    assert path.is_file(), "R2.0b resolver module has not been implemented yet"
    return importlib.import_module("scripts.task_intent_resolver")


def pass_cycle(request):
    return {
        "success": True,
        "candidate_id": f"{request['strategy_ref']}::0",
        "checks": [
            {"code": "reachable", "status": "PASS"},
            {"code": "collision_free", "status": "PASS"},
            {"code": "retreat_feasible", "status": "PASS"},
        ],
    }


def test_auto_resolution_is_deterministic_and_uses_r19_world_pose():
    resolver = resolver_module()
    first = resolver.resolve_task_intent(
        valid_intent(), environment(), cell(), observations(), pass_cycle, now=100.0
    )
    second = resolver.resolve_task_intent(
        valid_intent(), environment(), cell(), observations(), pass_cycle, now=100.0
    )
    assert first == second
    assert first["schema"] == "workcell_task_intent_resolution/v1"
    assert first["readiness_status"] == "READY"
    assert first["normalized_intent_sha256"] == normalized_intent_hash(valid_intent())
    assert first["capability_profile"] == {
        "installed_tool_id": "robotiq_85_gripper",
        "capabilities": ["two_finger_parallel"],
        "release_mapping": {"tool_release": "fingers_open"},
    }
    assert first["place_resolution"]["selected_local_pose"] == {
        "xyz_m": [0.05, 0.0, 0.10],
        "rpy_rad": [0.0, 0.0, 0.0],
    }
    assert first["place_resolution"]["world_pose"]["xyz_m"] == [1.05, 2.0, 3.10]
    assert first["place_resolution"]["physical_destination_contract"] == "target_local/v1"


def test_preferred_grasp_fallback_is_explicit_warning():
    resolver = resolver_module()
    intent = valid_intent("PREFERRED", "AUTO")

    def evaluator(request):
        if request["strategy_ref"] == "top_2f":
            return {
                "success": False,
                "reason_code": "GRASP_COLLISION",
                "reason": "preferred candidate collides",
                "checks": [{"code": "collision_free", "status": "FAIL"}],
            }
        return pass_cycle(request)

    result = resolver.resolve_task_intent(intent, environment(), cell(), observations(), evaluator, now=100.0)
    grasp = result["grasp_resolution"]
    assert result["readiness_status"] == "WARNING"
    assert grasp["requested_strategy_ref"] == "top_2f"
    assert grasp["selected_strategy_ref"] != "top_2f"
    assert grasp["fallback"]["used"] is True
    assert grasp["fallback"]["reason_code"] == "GRASP_COLLISION"


def test_exact_grasp_failure_blocks_without_substitution():
    resolver = resolver_module()
    intent = valid_intent("EXACT", "EXACT")
    calls = []

    def evaluator(request):
        calls.append(request["strategy_ref"])
        return {
            "success": False,
            "reason_code": "GRASP_UNREACHABLE",
            "reason": "exact grasp cannot be reached",
            "checks": [{"code": "reachable", "status": "FAIL"}],
        }

    result = resolver.resolve_task_intent(intent, environment(), cell(), observations(), evaluator, now=100.0)
    assert result["readiness_status"] == "BLOCKED"
    assert calls == ["top_2f"]
    assert result["grasp_resolution"]["selected_strategy_ref"] is None
    assert result["grasp_resolution"]["fallback"]["used"] is False


def test_exact_place_outside_region_blocks_without_clamp_or_fallback():
    resolver = resolver_module()
    intent = valid_intent("EXACT", "EXACT")
    requested = {"xyz_m": [0.30, 0.0, 0.10], "rpy_rad": [0.0, 0.0, 0.0]}
    intent["place"]["placement"]["requested_local_pose"] = copy.deepcopy(requested)
    result = resolver.resolve_task_intent(intent, environment(), cell(), observations(), pass_cycle, now=100.0)
    assert result["readiness_status"] == "BLOCKED"
    place = result["place_resolution"]
    assert place["requested_local_pose"] == requested
    assert place["selected_local_pose"] is None
    assert place["fallback"]["used"] is False
    assert result["readiness"]["primary_code"] == "PLACE_LOCAL_POSE_OUTSIDE_REGION"


def test_preferred_place_invalid_request_falls_back_to_region_default():
    resolver = resolver_module()
    intent = valid_intent("AUTO", "PREFERRED")
    intent["place"]["placement"]["requested_local_pose"] = {
        "xyz_m": [0.30, 0.0, 0.10],
        "rpy_rad": [0.0, 0.0, 0.0],
    }
    result = resolver.resolve_task_intent(intent, environment(), cell(), observations(), pass_cycle, now=100.0)
    assert result["readiness_status"] == "WARNING"
    place = result["place_resolution"]
    assert place["fallback"]["used"] is True
    assert place["fallback"]["reason_code"] == "PLACE_LOCAL_POSE_OUTSIDE_REGION"
    assert place["selected_local_pose"] == {
        "xyz_m": [0.05, 0.0, 0.10],
        "rpy_rad": [0.0, 0.0, 0.0],
    }


def test_unsupported_installed_tool_capability_is_blocked():
    resolver = resolver_module()
    bad_cell = {"end_effector": {"id": "vacuum_tool", "type": "suction"}}
    result = resolver.resolve_task_intent(
        valid_intent(), environment(), bad_cell, observations(), pass_cycle, now=100.0
    )
    assert result["readiness_status"] == "BLOCKED"
    assert result["readiness"]["primary_code"] == "REQUIRED_CAPABILITY_UNAVAILABLE"


def test_observation_filtering_is_deterministic():
    resolver = resolver_module()
    objects = observations() + [
        dict(observations()[0], id="epd::bottle-2", confidence=0.95),
        dict(observations()[0], id="epd::cup", class_id="cup", confidence=0.99),
    ]
    seen = []

    def evaluator(request):
        seen.append(request["observation"]["id"])
        return pass_cycle(request)

    result = resolver.resolve_task_intent(valid_intent(), environment(), cell(), objects, evaluator, now=100.0)
    assert result["grasp_resolution"]["selected_object_id"] == "epd::bottle-2"
    assert seen[0] == "epd::bottle-2"


def test_yaml_json_resolution_artifacts_are_semantically_equal(tmp_path):
    resolver = resolver_module()
    result = resolver.resolve_task_intent(
        valid_intent(), environment(), cell(), observations(), pass_cycle, now=100.0
    )
    paths = resolver.write_resolution_artifacts(result, tmp_path)
    yaml_doc = yaml.safe_load(Path(paths["yaml"]).read_text())
    import json
    json_doc = json.loads(Path(paths["json"]).read_text())
    assert yaml_doc == json_doc == result


def test_resolution_binds_successful_ik_branch_after_rejected_candidate():
    import pytest
    resolver = resolver_module()
    branch = {'schema': 'workcell_approach_ik/v1', 'joint_positions': {'joint': -2.43}}
    seen = []
    def evaluate(request):
        seen.append(copy.deepcopy(request))
        if len(seen) == 1:
            return {'success': False, 'reason_code': 'PREPLAN_GRASP_FAILED', 'checks': []}
        return dict(pass_cycle(request), approach_ik=copy.deepcopy(branch))
    result = resolver.resolve_task_intent(valid_intent(), environment(), cell(), observations(), evaluate, now=100.)
    assert result['grasp_resolution']['approach_ik'] == branch
    def revalidate(request):
        assert request['approach_ik'] == branch
        assert request['candidate'].candidate_id == result['grasp_resolution']['selected_candidate_id']
        return dict(pass_cycle(request), approach_ik=copy.deepcopy(branch))
    checked = resolver.resolve_task_intent(valid_intent(), environment(), cell(), observations(), revalidate, now=100., resolved=result)
    assert checked['resolution_sha256'] == result['resolution_sha256']
    damaged = copy.deepcopy(result); damaged['grasp_resolution']['approach_ik']['joint_positions']['joint'] += 1
    with pytest.raises(ValueError, match='CORRUPT'):
        resolver.consume_resolution(damaged, valid_intent(), environment(), cell())


def test_preferred_fallback_uses_selected_strategy_catalog_geometry():
    resolver = resolver_module()
    intent = valid_intent("PREFERRED", "AUTO")

    def evaluator(request):
        grasp = request["intent"]["pick"]["grasp"]
        if request["strategy_ref"] == "top_2f":
            return {
                "success": False,
                "reason_code": "GRASP_COLLISION",
                "reason": "preferred top grasp collides",
                "checks": [{"code": "collision_free", "status": "FAIL"}],
            }
        assert request["strategy_ref"] == "side_grip_basic"
        assert grasp["strategy_ref"] == "side_grip_basic"
        assert grasp["approach"] == {"axis": "x_plus", "distance_m": 0.08}
        assert grasp["orientation"]["mode"] == "horizontal"
        assert request["candidate"].effective["approach_distance_m"] == 0.08
        return pass_cycle(request)

    result = resolver.resolve_task_intent(
        intent, environment(), cell(), observations(), evaluator, now=100.0)
    assert result["readiness_status"] == "WARNING"
    assert result["grasp_resolution"]["selected_strategy_ref"] == "side_grip_basic"
    effective = result["grasp_resolution"]["effective_grasp"]
    assert effective["approach"] == {"axis": "x_plus", "distance_m": 0.08}
    assert effective["orientation"]["mode"] == "horizontal"
    assert effective["lift"] == {"axis": "z_up", "distance_m": 0.08}


def test_resolved_preferred_fallback_replays_same_effective_grasp():
    resolver = resolver_module()
    intent = valid_intent("PREFERRED", "AUTO")

    def first(request):
        if request["strategy_ref"] == "top_2f":
            return {"success": False, "reason_code": "BLOCKED",
                    "reason": "preferred blocked", "checks": []}
        return pass_cycle(request)

    resolved = resolver.resolve_task_intent(
        intent, environment(), cell(), observations(), first, now=100.0)
    assert resolved["grasp_resolution"]["selected_strategy_ref"] == "side_grip_basic"

    def replay(request):
        grasp = request["intent"]["pick"]["grasp"]
        assert request["strategy_ref"] == "side_grip_basic"
        assert grasp == resolved["grasp_resolution"]["effective_grasp"]
        assert grasp["approach"]["axis"] == "x_plus"
        assert grasp["orientation"]["mode"] == "horizontal"
        return pass_cycle(request)

    checked = resolver.resolve_task_intent(
        intent, environment(), cell(), observations(), replay, now=100.0, resolved=resolved)
    assert checked["resolution_sha256"] == resolved["resolution_sha256"]
