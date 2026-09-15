"""R2.0a task-intent v2 normalization, migration, validation, and hashing.

The canonical hash is SHA-256 over UTF-8 canonical JSON. Objects are sorted by
UTF-8 key, arrays retain authored order, booleans/null keep JSON spelling, and
numbers are normalized to an unquoted decimal token with up to 12 fractional digits
(trailing zeroes removed; ``-0`` becomes ``0``). This representation is shared
by the future C++ model and Python validator.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    from scripts.capability_registry import load_structured_data
except ImportError:  # CLI execution with scripts/ on sys.path
    from capability_registry import load_structured_data

ROOT = Path(__file__).resolve().parents[1]
STRATEGY_DIR = ROOT / "catalog" / "grasp_strategies"
STRATEGIES = {"top_2f", "side_grip_basic", "finger_pinch_basic"}
SUPPORTED_CAPABILITIES = {"two_finger_parallel"}
TOOL_CAPABILITY_MAP = {
    "robotiq_2f_85": {"two_finger_parallel"},
    "robotiq_85_gripper": {"two_finger_parallel"},
    "finger_gripper": {"two_finger_parallel"},
}
POLICIES = {"AUTO", "PREFERRED", "EXACT"}


@dataclass
class TaskIntentModel:
    """Typed semantic envelope matching the C++ R2.0a model fields."""
    task: dict[str, Any]
    pick_selection: dict[str, Any]
    grasp: dict[str, Any]
    place: dict[str, Any]
    safety: dict[str, Any]
    scene_package: str = ""
    routing: dict[str, Any] | None = None
    migration_provenance: dict[str, Any] | None = None
    extra: dict[str, Any] | None = None

    @classmethod
    def from_dict(cls, model: dict[str, Any]) -> "TaskIntentModel":
        known = {"schema", "scene_package", "task", "pick", "place", "safety", "routing", "provenance"}
        return cls(model.get("task", {}), model.get("pick", {}).get("selection", {}), model.get("pick", {}).get("grasp", {}), model.get("place", {}), model.get("safety", {}), str(model.get("scene_package", "")), model.get("routing"), copy.deepcopy(model.get("provenance")) if model.get("provenance") is not None else None, {k: copy.deepcopy(v) for k, v in model.items() if k not in known})

    def to_dict(self) -> dict[str, Any]:
        out = {"schema": "workcell_builder_task_intent/v2", "scene_package": self.scene_package, "task": self.task, "pick": {"selection": self.pick_selection, "grasp": self.grasp}, "place": self.place, "safety": self.safety}
        if self.routing is not None:
            out["routing"] = self.routing
        if self.migration_provenance is not None:
            out["provenance"] = copy.deepcopy(self.migration_provenance)
        out.update(self.extra or {})
        return out


def _strategy_payload(strategy_id: str) -> dict[str, Any] | None:
    path = STRATEGY_DIR / f"{strategy_id}.yaml"
    if not path.is_file():
        return None
    doc, _ = load_structured_data(path)
    payload = doc.get("grasp_strategy")
    return payload if isinstance(payload, dict) else None


def _decimal_number(value: Any) -> str:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("not a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("non-finite number")
    if number == 0:
        return "0"
    text = f"{number:.12f}".rstrip("0").rstrip(".")
    return text


def _canonical_json(value: Any) -> str:
    if isinstance(value, dict):
        keys = sorted((str(k) for k in value), key=lambda x: x.encode("utf-8"))
        return "{" + ",".join(json.dumps(k, ensure_ascii=False) + ":" + _canonical_json(value[k]) for k in keys) + "}"
    if isinstance(value, list):
        return "[" + ",".join(_canonical_json(v) for v in value) + "]"
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return _decimal_number(value)
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def canonical_bytes(model: dict[str, Any]) -> bytes:
    """Return the exact cross-language canonical semantic representation."""
    return _canonical_json(model).encode("utf-8")


def canonical_hash(model: dict[str, Any]) -> str:
    return hashlib.sha256(canonical_bytes(model)).hexdigest()


def normalize_v2(payload: dict[str, Any]) -> dict[str, Any]:
    """Normalize v2 without injecting physical geometry or runtime state."""
    model = copy.deepcopy(payload)
    model["schema"] = "workcell_builder_task_intent/v2"
    # Validation owns rejection; normalization only copies authored semantics
    # and canonicalizes policy spelling when the field exists. It never deletes
    # duplicate authorities, fills missing policy, rewrites release, or removes
    # dynamic state from invalid authored input.
    grasp = model.get("pick", {}).get("grasp", {})
    if "policy" in grasp:
        grasp["policy"] = str(grasp["policy"]).upper()
    placement = model.get("place", {}).get("placement", {})
    if "policy" in placement:
        placement["policy"] = str(placement["policy"]).upper()
    return model


def migrate_v1(payload: dict[str, Any], environment: dict[str, Any] | None = None) -> dict[str, Any]:
    """Migrate v1 in memory, preserving explicit strategy/place behavior."""
    if payload.get("schema") == "workcell_builder_task_intent/v2":
        return normalize_v2(payload)
    if environment is None:
        raise ValueError("MIGRATION_PHYSICAL_CONTEXT_REQUIRED: v1 place migration requires environment.yaml and R1.9 destination context")
    task = payload.get("task") if isinstance(payload.get("task"), dict) else {}
    old_pick = payload.get("pick") if isinstance(payload.get("pick"), dict) else {}
    old_source = old_pick.get("source") if isinstance(old_pick.get("source"), dict) else {}
    old_zone = old_pick.get("zone") if isinstance(old_pick.get("zone"), dict) else {}
    old_place = payload.get("place") if isinstance(payload.get("place"), dict) else {}
    old_target = old_place.get("target") if isinstance(old_place.get("target"), dict) else {}
    old_grasp = payload.get("grasp") if isinstance(payload.get("grasp"), dict) else {}
    strategy_id = old_grasp.get("strategy_ref")
    strategy = _strategy_payload(str(strategy_id)) if strategy_id else None
    if strategy_id and strategy is None:
        raise ValueError("MIGRATION_CATALOG_MISSING: referenced strategy cannot be recovered")
    selection = {"source_ref": old_source.get("id", ""), "source_type": old_source.get("type", "perception"),
                 "zone_ref": old_zone.get("id", ""), "object_filter": copy.deepcopy(old_pick.get("object_filter", {}))}
    grasp = {"policy": "EXACT" if strategy_id else "AUTO", "required_capability": "two_finger_parallel" if strategy_id else None,
             "strategy_ref": strategy_id}
    if strategy:
        for old, new in (("approach_axis", "approach"), ("approach_distance_m", "approach_distance_m"), ("orientation_mode", "orientation_mode"),
                         ("allowed_roll_angles_deg", "allowed_roll_deg"), ("allowed_yaw_angles_deg", "allowed_yaw_deg"),
                         ("tool_frame_offset_xyz", "tcp_offset_xyz_m"), ("tool_frame_offset_rpy", "tcp_offset_rpy_rad"), ("retreat_distance_m", "lift_distance_m")):
            value = old_grasp.get(old, strategy.get(old))
            if value is not None:
                if new == "approach": grasp[new] = {"axis": value, "distance_m": old_grasp.get("approach_distance_m", strategy.get("approach_distance_m"))}
                elif new == "approach_distance_m": continue
                elif new == "orientation_mode": grasp["orientation"] = {"mode": value}
                elif new == "allowed_roll_deg": grasp.setdefault("orientation", {})[new] = value
                elif new == "allowed_yaw_deg": grasp.setdefault("orientation", {})[new] = value
                elif new.startswith("tcp_"): grasp[new] = value
                elif new == "lift_distance_m": grasp["lift"] = {"axis": old_grasp.get("retreat_axis") or strategy.get("retreat_axis") or "z_up", "distance_m": value}
    old_offset = old_place.get("place_offset_xyz") or old_place.get("offset_xyz")
    # R1.9 physical destination truth is consulted before choosing a migrated
    # placement policy. This materializes the existing target-local point when
    # the v1 file relied on the authored environment projection.
    try:
        from scripts.physical_destination import resolve_destination
    except ImportError:
        from physical_destination import resolve_destination
    zone_id = old_target.get("region_ref", old_target.get("id", ""))
    destination = resolve_destination(environment, zone_id)
    if old_offset is None:
        local = destination["placement_local"]
        old_offset = local.get("pose_xyz") or local.get("pose", {}).get("xyz")
    effective_local = destination["placement_local"]
    default_rpy = effective_local.get("pose_rpy") or effective_local.get("pose", {}).get("rpy") or [0.0, 0.0, 0.0]
    local_pose = {"xyz_m": old_offset, "rpy_rad": old_place.get("place_offset_rpy", default_rpy)} if old_offset is not None else None
    model = {"schema": "workcell_builder_task_intent/v2", "scene_package": payload.get("scene_package", ""),
             "task": {"id": task.get("id", "migrated_task"), "type": task.get("type", "pick_place"), "template": task.get("template", task.get("type", "pick_place"))},
             "pick": {"selection": selection, "grasp": grasp},
             "place": {"target": {"asset_ref": destination["target_id"] if destination is not None else old_target.get("asset_ref", old_target.get("id", "")), "region_ref": old_target.get("region_ref", old_target.get("id", ""))},
                       "placement": {"policy": "EXACT" if local_pose is not None else "AUTO", "requested_local_pose": local_pose, "orientation": {"mode": "target_default", "rpy_rad": [0.0, 0.0, 0.0]}, "approach": {"axis": "z_down", "distance_m": 0.1}, "clearance_m": float(old_place.get("place_clearance_m", 0.05)), "retreat": {"axis": old_place.get("retreat_axis", "z_up"), "distance_m": float(old_place.get("retreat_distance_m", 0.1))}},
                       "release": {"strategy": "tool_release"}},
             "routing": copy.deepcopy(payload.get("routing", {})), "safety": {"execution_mode": "simulation_preview", "require_fake_hardware": True, "real_hardware_enabled": False, "preview_policy": "diagnostic_if_unresolved"},
             "provenance": {"migration": {"from_schema": "workcell_builder_task_intent/v1", "materialized_from_catalog": bool(strategy), "preserved_explicit_strategy": bool(strategy_id), "preserved_place_behavior": True, "materialized_place_from_r19": destination is not None}}}
    return normalize_v2(model)


def validate_intent(model: dict[str, Any]) -> list[dict[str, str]]:
    errors: list[dict[str, str]] = []
    if model.get("schema") != "workcell_builder_task_intent/v2":
        errors.append({"code": "SCHEMA_UNSUPPORTED", "message": "expected workcell_builder_task_intent/v2"})
    if "target_policy" in model.get("task", {}) or "object_filter" in model.get("pick", {}):
        errors.append({"code": "DUPLICATE_PICK_SELECTION_AUTHORITY", "message": "object eligibility belongs only to pick.selection"})
    selection = model.get("pick", {}).get("selection", {})
    if not isinstance(selection, dict) or not isinstance(selection.get("object_filter"), dict):
        errors.append({"code": "PICK_SELECTION_REQUIRED", "message": "pick.selection.object_filter is required"})
    grasp = model.get("pick", {}).get("grasp", {})
    placement = model.get("place", {}).get("placement", {})
    for block, label in ((grasp, "grasp"), (placement, "place")):
        policy = block.get("policy")
        if policy is None:
            errors.append({"code": "POLICY_REQUIRED", "message": f"{label}.policy is required"})
            continue
        if policy not in POLICIES:
            errors.append({"code": "POLICY_INVALID", "message": f"{label}.policy is invalid"})
        if policy == "PREFERRED" and label == "place" and not _valid_pose(block.get("requested_local_pose")):
            errors.append({"code": "PREFERRED_LOCAL_POSE_REQUIRED", "message": "place.placement.requested_local_pose is required for PREFERRED"})
        if policy == "EXACT":
            required = ["orientation", "approach", "retreat"] if label == "place" else ["orientation", "approach", "lift"]
            for field in required:
                if field not in block:
                    errors.append({"code": "EXACT_REQUIRED_FIELD_MISSING", "message": f"{label}.{field} is required for EXACT"})
            if label == "place" and not _valid_pose(block.get("requested_local_pose")):
                errors.append({"code": "EXACT_LOCAL_POSE_REQUIRED", "message": "EXACT placement requires finite xyz_m/rpy_rad in target-asset-local frame"})
        if policy in {"PREFERRED", "EXACT"} and not block.get("strategy_ref") and label == "grasp":
            errors.append({"code": "STRATEGY_REQUIRED", "message": "grasp.strategy_ref is required for this policy"})
    cap = grasp.get("required_capability")
    if cap not in SUPPORTED_CAPABILITIES:
        errors.append({"code": "UNSUPPORTED_CAPABILITY", "message": f"unsupported capability: {cap}"})
    strategy_id = grasp.get("strategy_ref")
    if strategy_id and strategy_id not in STRATEGIES:
        errors.append({"code": "UNKNOWN_STRATEGY_ID", "message": f"unknown strategy ID: {strategy_id}"})
    if model.get("place", {}).get("release", {}).get("strategy") != "tool_release":
        errors.append({"code": "RELEASE_NOT_SEMANTIC", "message": "release strategy must be tool_release"})
    if "tool" in model:
        errors.append({"code": "INSTALLED_TOOL_IN_INTENT", "message": "installed tool identity belongs to environment"})
    if any(key in model.get("safety", {}) for key in ("runtime_io_applied", "motion_started", "ros_launch_started")):
        errors.append({"code": "DYNAMIC_SAFETY_IN_INTENT", "message": "dynamic runtime state is not authored intent"})
    return errors


def write_without_rewrite(path: Path) -> dict[str, Any]:
    payload, _ = load_structured_data(path)
    if payload.get("schema") == "workcell_builder_task_intent/v2":
        return normalize_v2(payload)
    env_path = path.parent.parent / "environment.yaml"
    if not env_path.is_file():
        raise ValueError("MIGRATION_PHYSICAL_CONTEXT_REQUIRED: environment.yaml not found beside v1 scene")
    environment, _ = load_structured_data(env_path)
    return migrate_v1(payload, environment.get("environment", environment))


def _valid_pose(value: Any) -> bool:
    if not isinstance(value, dict):
        return False
    for key in ("xyz_m", "rpy_rad"):
        values = value.get(key)
        if not isinstance(values, list) or len(values) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            return False
    return True


def parse_validate_normalize(payload: dict[str, Any]) -> dict[str, Any]:
    """Single safe API: validate authored semantics, then normalize only if valid."""
    if payload.get("schema") == "workcell_builder_task_intent/v2":
        candidate = normalize_v2(payload)
        diagnostics = validate_intent(candidate)
        return {"diagnostics": diagnostics, "normalized": candidate if not diagnostics else None}
    raise ValueError("Use migrate_v1(payload, environment) for v1 input")


def normalized_intent_hash(payload: dict[str, Any]) -> str:
    result = parse_validate_normalize(payload)
    if result["diagnostics"]:
        raise ValueError("INTENT_INVALID: authoritative hash is unavailable")
    return canonical_hash(result["normalized"])
