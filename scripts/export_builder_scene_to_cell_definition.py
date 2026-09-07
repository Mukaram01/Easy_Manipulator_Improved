#!/usr/bin/env python3
"""Export builder-generated scene metadata into Workcell Studio source files."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from capability_registry import load_structured_data
from workcell_builder_grasp_strategy import normalize_grasp_strategy

try:
    import yaml as _pyyaml
except Exception:
    _pyyaml = None


def _load_optional(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    loaded, _ = load_structured_data(path)
    return loaded if isinstance(loaded, dict) else {}


def _load_environment_layout(scene_path: Path) -> tuple[dict[str, Any], str | None, list[str]]:
    warnings: list[str] = []
    preferred = scene_path / "layout" / "workcell_studio_layout.yaml"
    legacy = scene_path / "environment_layout.yaml"
    generated_legacy = scene_path / "generated" / "environment_layout.yaml"
    selected: Path | None = None
    if preferred.is_file() and legacy.is_file():
        warnings.append("Both layout/workcell_studio_layout.yaml and environment_layout.yaml exist; preferring layout/workcell_studio_layout.yaml.")
        selected = preferred
    elif preferred.is_file():
        selected = preferred
    elif legacy.is_file():
        warnings.append("Using legacy environment_layout.yaml; prefer layout/workcell_studio_layout.yaml.")
        selected = legacy
    elif generated_legacy.is_file():
        warnings.append("Using generated/environment_layout.yaml fallback; prefer layout/workcell_studio_layout.yaml.")
        selected = generated_legacy
    if selected is None:
        return {}, None, warnings
    loaded = _load_optional(selected)
    return (loaded if loaded else {}), str(selected.resolve()), warnings


def _to_yaml(value: Any, indent: int = 0) -> str:
    sp = " " * indent
    if isinstance(value, dict):
        lines = []
        for k, v in value.items():
            if isinstance(v, (dict, list)):
                lines.append(f"{sp}{k}:")
                lines.append(_to_yaml(v, indent + 2))
            else:
                lines.append(f"{sp}{k}: {_scalar(v)}")
        return "\n".join(lines)
    if isinstance(value, list):
        lines = []
        for item in value:
            if isinstance(item, dict) and item:
                keys=list(item.keys())
                first=keys[0]
                first_val=item[first]
                if isinstance(first_val,(dict,list)):
                    lines.append(f"{sp}- {first}:")
                    lines.append(_to_yaml(first_val, indent+4))
                else:
                    lines.append(f"{sp}- {first}: {_scalar(first_val)}")
                for k in keys[1:]:
                    v=item[k]
                    if isinstance(v,(dict,list)):
                        lines.append(f"{sp}  {k}:")
                        lines.append(_to_yaml(v, indent+4))
                    else:
                        lines.append(f"{sp}  {k}: {_scalar(v)}")
            elif isinstance(item, list):
                lines.append(f"{sp}-")
                lines.append(_to_yaml(item, indent + 2))
            else:
                lines.append(f"{sp}- {_scalar(item)}")
        return "\n".join(lines)
    return f"{sp}{_scalar(value)}"


def _scalar(v: Any) -> str:
    if v is True:
        return "true"
    if v is False:
        return "false"
    if v is None:
        return "null"
    if isinstance(v, (int, float)):
        return str(v)
    return json.dumps(str(v))


def _write_structured(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if _pyyaml is not None:
        path.write_text(_pyyaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    else:
        path.write_text(_to_yaml(payload) + "\n", encoding="utf-8")


def _find_task_intent(scene_path: Path) -> Path | None:
    # Authored inputs always win.  The generated copy is only a compatibility
    # fallback and must never shadow the canonical config source.
    for rel in [
        "config/workcell_builder_task_intent.yaml",
        "workcell_builder_task_intent.yaml",
        "generated/workcell_builder_task_intent.yaml",
    ]:
        cand = scene_path / rel
        if cand.is_file():
            return cand
    return None


def _resolve_authored_task(
    task_intent: dict[str, Any], task_intent_path: Path, environment: dict[str, Any],
    layout: dict[str, Any], layout_path: Path,
) -> dict[str, Any]:
    """Resolve task IDs through environment bindings into authored layout items."""
    def fail(identifier: Any, reason: str) -> None:
        raise ValueError(
            f"Task intent '{task_intent_path}' and layout '{layout_path}' cannot resolve "
            f"identifier '{identifier}': {reason}"
        )

    if task_intent.get("schema") != "workcell_builder_task_intent/v1":
        fail(task_intent.get("schema", "<missing schema>"), "invalid task intent schema")

    items = layout.get("items")
    if not isinstance(items, list):
        fail("items", "authored layout does not contain an items list")
    item_by_id = {str(item.get("id")): item for item in items
                  if isinstance(item, dict) and item.get("id")}
    task_zones = environment.get("task_zones")
    zone_by_id = {str(zone.get("id")): zone for zone in task_zones
                  if isinstance(zone, dict) and zone.get("id")} if isinstance(task_zones, list) else {}

    def intent_ref(block: str, child: str) -> tuple[str, dict[str, Any]]:
        section = task_intent.get(block)
        ref = section.get(child) if isinstance(section, dict) else None
        identifier = ref.get("id") if isinstance(ref, dict) else None
        if not identifier:
            fail(f"{block}.{child}.id", "required identifier is missing")
        zone = zone_by_id.get(str(identifier))
        if zone is None:
            fail(identifier, f"not found in task_zones from authored environment for {block}.{child}.id")
        layout_id = zone.get("layout_item_ref")
        if not layout_id or str(layout_id) not in item_by_id:
            fail(layout_id or identifier, f"layout_item_ref for task zone '{identifier}' is missing or unresolved")
        return str(identifier), item_by_id[str(layout_id)]

    # A perception-backed pick has two distinct bindings: source is the
    # normalized perception stream, while zone is the authored eligibility
    # region.  Older exports conflated those IDs, which made regeneration try
    # to find ``detected_objects/v1`` in the physical layout.
    pick_section = task_intent.get("pick") if isinstance(task_intent.get("pick"), dict) else {}
    pick_child = "zone" if isinstance(pick_section.get("zone"), dict) else "source"
    pick_id, pick_item = intent_ref("pick", pick_child)
    place_id, place_zone = intent_ref("place", "target")
    target_ref = place_zone.get("target_ref")
    if not target_ref or str(target_ref) not in item_by_id:
        fail(target_ref or place_zone.get("id"), f"target_ref on place layout item '{place_zone.get('id')}' is missing or unresolved")
    target = item_by_id[str(target_ref)]
    # The physical bin and the semantic free place region intentionally have
    # different transforms and are separate authoring entities.  Preserve the
    # old transform-group check only when a scene explicitly supplies groups;
    # requiring identical poses here would make a valid bin interior
    # impossible to author.
    zone_group = place_zone.get("transform_group")
    target_group = target.get("transform_group")
    if zone_group is not None and target_group is not None and zone_group != target_group:
        fail(target_ref, f"inconsistent transform_group between '{place_zone.get('id')}' and '{target_ref}'")
    for item_id, item in ((pick_item.get("id"), pick_item), (target_ref, target)):
        pose = item.get("pose")
        if (not isinstance(pose, dict) or not isinstance(pose.get("xyz"), list)
                or len(pose["xyz"]) != 3 or not isinstance(pose.get("rpy"), list)
                or len(pose["rpy"]) != 3):
            fail(item_id, "layout pose must contain three-element xyz and rpy lists")
    return {"pick_id": pick_id, "pick_item": pick_item, "place_id": place_id,
            "place_zone": place_zone, "target_ref": str(target_ref), "target": target}


def _validate_task_intent(task_intent_path: Path, scene_path: Path) -> dict[str, Any]:
    cmd = ["python3", str(SCRIPT_DIR / "validate_builder_task_intent.py"), str(task_intent_path), "--scene-package", str(scene_path), "--grasp-strategies-dir", str(scene_path.parent / "catalog" / "grasp_strategies"), "--json"]
    run = subprocess.run(cmd, capture_output=True, text=True, check=False)
    try:
        return json.loads(run.stdout) if run.stdout.strip() else {"status": "FAIL", "errors": [run.stderr.strip()]}
    except Exception:
        return {"status": "FAIL", "errors": [run.stdout.strip() or run.stderr.strip()]}

def _generate_task_recipe(task_intent_path: Path, output_path: Path, scene_path: Path) -> dict[str, Any]:
    cmd = ["python3", str(SCRIPT_DIR / "convert_builder_task_intent_to_task_recipe.py"), "--task-intent", str(task_intent_path), "--output", str(output_path), "--scene-package", str(scene_path), "--validate", "--json"]
    run = subprocess.run(cmd, capture_output=True, text=True, check=False)
    try:
        return json.loads(run.stdout) if run.stdout.strip() else {"status": "FAIL", "errors": [run.stderr.strip()]}
    except Exception:
        return {"status": "FAIL", "errors": [run.stdout.strip() or run.stderr.strip()]}

def _generate_plan_preview(task_recipe_path: Path, output_path: Path, cell_path: Path, layout_path: Path) -> dict[str, Any]:
    cmd = ["python3", str(SCRIPT_DIR / "generate_offline_plan_preview_request.py"), "--task-recipe", str(task_recipe_path), "--output", str(output_path), "--cell-definition", str(cell_path), "--environment-layout", str(layout_path), "--validate", "--json"]
    run = subprocess.run(cmd, capture_output=True, text=True, check=False)
    try:
        return json.loads(run.stdout) if run.stdout.strip() else {"status": "FAIL", "errors": [run.stderr.strip()]}
    except Exception:
        return {"status": "FAIL", "errors": [run.stdout.strip() or run.stderr.strip()]}




def _build_task_intent_from_scene(scene_path: Path, env_layout: dict[str, Any], meta: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    missing=[]
    task_type = "pick_place"
    grasp_meta = meta.get("grasp_strategy") if isinstance(meta.get("grasp_strategy"), dict) else {}
    ee_meta = meta.get("end_effector") if isinstance(meta.get("end_effector"), dict) else {}
    normalized_grasp, grasp_warnings = normalize_grasp_strategy(grasp_meta, ee_meta)
    strategy = grasp_meta.get("strategy_id")
    pick_id = None
    place_id = None
    zones = env_layout.get("zones") if isinstance(env_layout.get("zones"), list) else []
    targets = env_layout.get("targets") if isinstance(env_layout.get("targets"), list) else []
    for z in zones + targets:
        if not isinstance(z, dict):
            continue
        zid = z.get("id")
        ztype = str(z.get("type", "")).lower()
        if not pick_id and ("pick" in ztype or "pick" in str(zid).lower()):
            pick_id = zid
        if not place_id and ("place" in ztype or "bin" in ztype or any(k in str(zid).lower() for k in ["place","bin","drop"])):
            place_id = zid
    if not pick_id:
        missing.append("pick source missing")
    if not strategy:
        missing.append("grasp strategy missing")
    if not place_id:
        missing.append("place target missing")
    release_strategy = "tool_release"
    routing_rules = [{"id": "route_any_to_selected_place", "when": {"always": True}, "place_target": place_id}]
    if not release_strategy:
        missing.append("release strategy missing")
    if not routing_rules:
        missing.append("routing rule missing")
    intent = {
        "schema": "workcell_builder_task_intent/v1",
        "scene_package": scene_path.as_posix(),
        "task": {"id": "default_builder_task", "type": task_type, "mode": "offline_preview"},
        "pick": {"source": {"type": "zone", "id": pick_id}},
        "grasp": {"strategy_ref": strategy, "approach_axis": normalized_grasp.get("approach_axis", "z_down"), "orientation_mode": normalized_grasp.get("orientation_mode", "auto_align"), "approach_distance_m": normalized_grasp.get("approach_distance_m", 0.1), "retreat_axis": "z_up", "retreat_distance_m": normalized_grasp.get("retreat_distance_m", 0.1), "allowed_roll_angles_deg": normalized_grasp.get("allowed_roll_angles_deg", [0.0]), "allowed_yaw_angles_deg": normalized_grasp.get("allowed_yaw_angles_deg", [0.0, 180.0]), "gripper_tcp_offset": normalized_grasp.get("gripper_tcp_offset", {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}), "suction_cups": normalized_grasp.get("suction_cups")},
        "place": {"target": {"type": "bin", "id": place_id}, "release_strategy": release_strategy, "retreat_axis": "z_up", "retreat_distance_m": 0.1},
        "routing": {"rules": routing_rules},
        "safety": {"metadata_only": True, "runtime_io_applied": False, "motion_started": False, "ros_launch_started": False},
    }
    return intent, missing


def _extract_task_zones(environment: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, int], list[str]]:
    zones = environment.get("task_zones") if isinstance(environment.get("task_zones"), list) else []
    normalized=[]
    counts={"total":0,"pick":0,"place":0}
    ids=[]
    for i,z in enumerate(zones):
        if not isinstance(z,dict):
            continue
        zid=str(z.get("id") or f"task_zone_{i+1:02d}")
        ztype=str(z.get("type") or "").lower()
        role="other"
        if "pick" in ztype or "pick" in zid.lower():
            role="pick"; counts["pick"] += 1
        elif "place" in ztype or "target" in ztype or "bin" in ztype or "place" in zid.lower():
            role="place"; counts["place"] += 1
        ids.append(zid)
        pose = z.get("pose") if isinstance(z.get("pose"), dict) else {}
        normalized.append({
            "id": zid,
            "type": z.get("type", ""),
            "frame": z.get("frame", "world"),
            "pose_xyz": z.get("pose_xyz") or pose.get("xyz") or [0.0, 0.0, 0.0],
            "pose_rpy": z.get("pose_rpy") or pose.get("rpy") or [0.0, 0.0, 0.0],
            "dimensions": z.get("dimensions") or z.get("size") or [0.3, 0.3, 0.1],
            "role": role,
            "layout_item_ref": z.get("layout_item_ref"),
            "target_ref": z.get("target_ref"),
        })
    counts["total"]=len(normalized)
    return normalized, counts, ids

def _task_type_from_meta(meta: dict[str, Any]) -> str:
    task = meta.get("task_template") if isinstance(meta.get("task_template"), dict) else {}
    selected = str(task.get("selected") or task.get("id") or "pick_place").strip().lower()
    mapping={"pick_place":"pick_place","sorting":"sorting","inspection":"inspection","machine_tending":"machine_tending","conveyor_picking":"conveyor_picking","palletising":"palletising","bin_picking":"bin_picking"}
    return mapping.get(selected, "pick_place")

def export_scene(scene_path: Path, output_dir: Path, validate: bool) -> dict[str, Any]:
    env = _load_optional(scene_path / "environment.yaml")
    meta = _load_optional(scene_path / "workcell_builder_metadata.yaml")
    task_zones, task_zone_counts, _ = _extract_task_zones(env)
    warnings: list[str] = []
    task_intent_path = _find_task_intent(scene_path)
    builder_task_intent: dict[str, Any] = {}
    task_intent_validation: dict[str, Any] = {}
    task_recipe_generation: dict[str, Any] = {}
    plan_preview_generation: dict[str, Any] = {}

    robot_env = env.get("robot") if isinstance(env.get("robot"), dict) else {}
    ee_env = env.get("end_effector") if isinstance(env.get("end_effector"), dict) else {}
    env_metadata = env.get("metadata") if isinstance(env.get("metadata"), dict) else {}
    robot_mount = env_metadata.get("robot_mount") if isinstance(env_metadata.get("robot_mount"), dict) else {}
    tool_attachment = env_metadata.get("tool_attachment") if isinstance(env_metadata.get("tool_attachment"), dict) else {}
    objects_env = env.get("objects") if isinstance(env.get("objects"), dict) else {}

    robot_meta = meta.get("robot") if isinstance(meta.get("robot"), dict) else {}
    ee_meta = meta.get("end_effector") if isinstance(meta.get("end_effector"), dict) else {}
    grasp_meta = meta.get("grasp_strategy") if isinstance(meta.get("grasp_strategy"), dict) else {}
    ee_meta = meta.get("end_effector") if isinstance(meta.get("end_effector"), dict) else {}
    normalized_grasp, grasp_warnings = normalize_grasp_strategy(grasp_meta, ee_meta)
    sensors_meta = meta.get("sensors") if isinstance(meta.get("sensors"), list) else []

    robot_name = robot_env.get("name") or robot_env.get("model") or robot_meta.get("selected_name") or "unknown_robot"
    ee_name = ee_env.get("name") or ee_env.get("model") or ee_meta.get("selected_name") or "unknown_end_effector"
    task_type = _task_type_from_meta(meta)
    robot_id = robot_mount.get("id") or robot_env.get("id") or robot_name
    robot_base_link = robot_mount.get("base_link") or robot_env.get("base_link") or "base_link"
    robot_parent_frame = robot_mount.get("parent_frame") or "world"
    # ``environment.yaml`` is the authored physical source.  Older builder
    # exports stored this in metadata.robot_mount; accept both shapes while
    # preserving the authored values in the generated handoff.
    robot_pose = robot_mount.get("pose") if isinstance(robot_mount.get("pose"), dict) else {
        "xyz": robot_env.get("pose_xyz", [0.0, 0.0, 0.0]),
        "rpy": robot_env.get("pose_rpy", [0.0, 0.0, 0.0]),
    }
    ee_id = tool_attachment.get("id") or ee_env.get("id") or ee_name
    ee_parent_link = tool_attachment.get("parent_link") or ee_env.get("parent_link") or "tool0"
    ee_child_link = tool_attachment.get("child_link") or ee_env.get("base_link") or "tool0"
    ee_attach_pose = tool_attachment.get("attach_pose") if isinstance(tool_attachment.get("attach_pose"), dict) else {
        "xyz": ee_env.get("mount_pose_xyz", [0.0, 0.0, 0.0]),
        "rpy": ee_env.get("mount_pose_rpy", [0.0, 0.0, 0.0]),
    }

    authored_camera = env.get("camera") if isinstance(env.get("camera"), dict) else {}
    authored_task = env.get("task") if isinstance(env.get("task"), dict) else {}
    authored_perception = env.get("perception") if isinstance(env.get("perception"), dict) else {}

    assets: list[dict[str, Any]] = []
    object_entries: list[dict[str, Any]] = []
    for idx, (obj_name, obj) in enumerate(objects_env.items()):
        if not isinstance(obj, dict):
            continue
        filepath = obj.get("filepath")
        dims = obj.get("dimensions") if isinstance(obj.get("dimensions"), list) else [0.1, 0.1, 0.1]
        pose_xyz = obj.get("origin_xyz") if isinstance(obj.get("origin_xyz"), list) and len(obj.get("origin_xyz")) == 3 else [0.0, 0.0, 0.0]
        pose_rpy = obj.get("origin_rpy") if isinstance(obj.get("origin_rpy"), list) and len(obj.get("origin_rpy")) == 3 else [0.0, 0.0, 0.0]
        if not isinstance(filepath, str):
            warnings.append(f"Object '{obj_name}' is missing filepath; using primitive fallback.")
        assets.append({
            "id": str(obj_name),
            "label": str(obj_name),
            "kind": "object",
            "pose": {"xyz": pose_xyz, "rpy": pose_rpy},
            "dimensions": dims,
            "source": {"path": filepath} if isinstance(filepath, str) else {"kind": "primitive"},
            "static": True,
            "collision": True,
        })
        object_entries.append({"id": str(obj_name), "name": str(obj_name), "mesh": filepath, "dimensions": dims, "index": idx})

    authored_layout, authored_layout_ref, layout_warnings = _load_environment_layout(scene_path)
    warnings.extend(layout_warnings)
    if not task_intent_path:
        generated_intent, missing_msgs = _build_task_intent_from_scene(scene_path, authored_layout, meta)
        if missing_msgs:
            warnings.append("Builder-authored task intent is incomplete: " + ", ".join(missing_msgs))
        else:
            generated_task_intent_path = output_dir / "workcell_builder_task_intent.yaml"
            _write_structured(generated_task_intent_path, generated_intent)
            task_intent_path = generated_task_intent_path
    if not task_intent_path:
        raise ValueError(
            f"Task intent '{scene_path / 'config/workcell_builder_task_intent.yaml'}' is missing; "
            f"layout '{authored_layout_ref or scene_path / 'layout/workcell_studio_layout.yaml'}' cannot resolve task identifiers"
        )
    resolved_layout_path = (Path(authored_layout_ref) if authored_layout_ref else
                            scene_path / "layout" / "workcell_studio_layout.yaml")
    try:
        task_intent_payload = _load_optional(task_intent_path)
    except Exception as exc:
        raise ValueError(
            f"Task intent '{task_intent_path}' and layout '{resolved_layout_path}' cannot resolve "
            f"identifier '<invalid task intent>': {exc}"
        ) from exc
    resolved_task = _resolve_authored_task(
        task_intent_payload, task_intent_path, env, authored_layout,
        resolved_layout_path,
    )
    authored_items = authored_layout.get("items") if isinstance(authored_layout.get("items"), list) else []
    layout_assets = []
    for item in authored_items:
        if not isinstance(item, dict):
            continue
        exported_item = dict(item)
        item_type = str(item.get("type") or "")
        exported_item["type"] = {
            "pick_zone": "object_spawn_area",
            "realsense": "camera_mount",
        }.get(item_type, item_type)
        exported_item.setdefault("pose", {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]})
        layout_assets.append(exported_item)

    environment_layout = {
        "schema_version": "environment_layout/v1",
        "layout_id": authored_layout.get("layout_id", f"{scene_path.name}_layout"),
        "name": authored_layout.get("name", f"{scene_path.name} Builder Layout"),
        "frame": authored_layout.get("frame", "world"),
        "assets": assets + layout_assets,
        "source_metadata": {
            "generated_by": "workcell_builder",
            "raw_environment_keys": sorted(env.keys()),
        },
    }
    if isinstance(authored_layout.get("zones"), list):
        environment_layout["zones"] = authored_layout["zones"]
    if isinstance(authored_layout.get("targets"), list):
        environment_layout["targets"] = authored_layout["targets"]
    if isinstance(authored_layout.get("objects"), list):
        environment_layout["objects"] = authored_layout["objects"]
    if isinstance(authored_layout.get("camera"), dict):
        environment_layout["camera"] = authored_layout["camera"]
    # Preserve the Workcell Studio authored item graph verbatim in the derived
    # handoff.  In particular, destination ``target_ref``/``transform_group``
    # relationships and edited poses must survive regeneration.
    if isinstance(authored_layout.get("items"), list):
        environment_layout["items"] = authored_layout["items"]
    if isinstance(authored_layout.get("task_bindings"), dict):
        environment_layout["task_bindings"] = authored_layout["task_bindings"]

    authored_grasp = dict(authored_task.get("grasp")) if isinstance(authored_task.get("grasp"), dict) else {}
    intent_grasp = task_intent_payload.get("grasp") if isinstance(task_intent_payload.get("grasp"), dict) else {}
    authored_strategy_ref = str(
        authored_grasp.get("strategy_ref") or intent_grasp.get("strategy_ref") or ""
    ).strip()
    # Cell Definition uses one canonical grasp selector.  Keep detailed intent
    # fields, but do not emit both the legacy ``strategy`` and ``strategy_ref``
    # selectors because the validator correctly treats that as ambiguous.
    authored_grasp.pop("strategy", None)
    authored_grasp.pop("strategy_ref", None)

    cell_def = {
        "schema_version": "cell_definition/v1",
        "cell": {"id": scene_path.name, "name": scene_path.name, "planning_frame": "world"},
        "robot": {
            "id": robot_id,
            "name": robot_name,
            "model": robot_name,
            "capability": robot_meta.get("capability_id"),
            "planning_group": robot_env.get("planning_group", "manipulator"),
            "base_link": robot_base_link,
            "parent_frame": robot_parent_frame,
            "pose": robot_pose,
            "base_frame": robot_env.get("base_link", "base_link"),
            "tool_link": ee_env.get("parent_link", "tool0"),
            "home_named_target": robot_env.get("home_named_target", "home"),
            "safe_joint_state": robot_env.get("safe_joint_state", []),
            "joint_names": robot_env.get("joint_names", []),
            "pose_xyz": robot_env.get("pose_xyz", robot_pose.get("xyz", [0.0, 0.0, 0.0])),
            "pose_rpy": robot_env.get("pose_rpy", robot_pose.get("rpy", [0.0, 0.0, 0.0])),
        },
        "end_effector": {
            "id": ee_id,
            "name": ee_name,
            "capability": ee_meta.get("capability_id"),
            "type": ee_env.get("type") or ee_meta.get("family") or "custom",
            "grasp_frame": ee_env.get("grasp_frame", ee_env.get("base_link", "tool0")),
            "parent_link": ee_parent_link,
            "child_link": ee_child_link,
            "attach_pose": ee_attach_pose,
            "mount_pose_xyz": ee_env.get("mount_pose_xyz", [0.0, 0.0, 0.0]),
            "mount_pose_rpy": ee_env.get("mount_pose_rpy", [0.0, 0.0, 0.0]),
            "tcp_pose_xyz": ee_env.get("tcp_pose_xyz", [0.0, 0.0, 0.0]),
            "tcp_pose_rpy": ee_env.get("tcp_pose_rpy", [0.0, 0.0, 0.0]),
            "allowed_touch_links": ee_env.get("allowed_touch_links", []),
        },
        "camera": {"id": authored_camera.get("camera_id", "realsense_overhead"),
                    "type": "realsense_d435i",
                    "parent_frame": authored_camera.get("parent_frame", "world"),
                    "frame_id": authored_camera.get("frame_id", "camera_color_optical_frame"),
                    "pose": {"xyz": authored_camera.get("pose", [0.0, 0.0, 1.0])[:3],
                             "rpy": authored_camera.get("pose", [0.0, 0.0, 1.0, 0.0, 0.0, 0.0])[3:6]},
                    "frames": {"optical_frame": authored_camera.get("frame_id", "camera_color_optical_frame")},
                    "topics": {"color": authored_camera.get("rgb_topic", "/camera/camera/color/image_raw"),
                               "depth": authored_camera.get("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"),
                               "camera_info": "/camera/camera/color/camera_info"}},
        "environment": {
            "frame": "world",
            "layout": "layout/workcell_studio_layout.yaml",
            "task_zones": task_zones,
            "task_zones_summary": task_zone_counts,
            "support_surfaces": [{"id": a["id"], "type": "table", "frame": "world", "pose_xyz": a["pose"]["xyz"], "pose_rpy": a["pose"]["rpy"], "dimensions": a["dimensions"]} for a in assets],
        },
        "objects": [{"id": o["id"], "class": "part", "shape": "mesh", "color": "unknown", "material": "unknown", "frame": "world", "dimensions": o["dimensions"], "pose_xyz": [0.0,0.0,0.0], "pose_rpy": [0.0,0.0,0.0]} for o in object_entries],
        "task": {
            **authored_task,
            "id": authored_task.get("id", "default_task"),
            "type": authored_task.get("type", task_type),
            "object_source": "perception",
            # Compatibility scalar retained for older readiness consumers;
            # the structured pick.source_ref remains authoritative.
            "pick_zone": resolved_task["pick_id"],
            "pick": {**(authored_task.get("pick") if isinstance(authored_task.get("pick"), dict) else {}),
                     "source_ref": resolved_task["pick_id"],
                     "object_source": "perception"},
            "place": {**(authored_task.get("place") if isinstance(authored_task.get("place"), dict) else {}),
                      "target_ref": resolved_task["place_id"],
                      "region_ref": resolved_task["place_id"]},
            "destinations": authored_task.get("destinations") or [{
                "id": resolved_task["place_id"], "target_ref": resolved_task["target_ref"],
                "frame": authored_layout.get("frame", "world"),
                "pose_xyz": resolved_task["target"]["pose"]["xyz"],
                "pose_rpy": resolved_task["target"]["pose"]["rpy"],
            }],
            "object_filter": authored_task.get("object_filter", {"class_id": "bottle", "min_confidence": None, "max_age_seconds": 2.0}),
            "grasp": authored_grasp or {"intent": "top_2f", "approach_distance_m": 0.12, "retreat_distance_m": 0.15},
            "home_pose": authored_task.get("home_pose", robot_env.get("home_named_target", "home")),
        },
        "grasp": {"strategy_ref": authored_strategy_ref or normalized_grasp.get("strategy_id") or "top_2f",
                   **authored_grasp},
        "perception": {**authored_perception, "enabled": True, "mode": authored_perception.get("mode", "live_epd"),
                        "frame_id": authored_camera.get("frame_id", "camera_color_optical_frame"),
                        "normalized_output_contract": "detected_objects/v1"},
        "commissioning": {"self_test_enabled": False, "export_bundle": False, "generated_by": "workcell_builder", "review_required": True, "fake_hardware_first": True, "runtime_send_disabled_by_default": True},
    }

    warnings.extend(grasp_warnings)

    if not robot_meta.get("capability_id"):
        warnings.append("Robot capability is unknown; review required before runtime commissioning.")
    if robot_meta.get("preview_only"):
        warnings.append("Selected robot is preview_only; runtime execution remains blocked.")

    if task_intent_path:
        task_intent_validation = _validate_task_intent(task_intent_path, scene_path)
        if task_intent_validation.get("status") == "FAIL":
            details = "; ".join(str(e) for e in task_intent_validation.get("errors", []))
            raise ValueError(
                f"Task intent '{task_intent_path}' and layout '{resolved_layout_path}' cannot resolve "
                f"identifier '<invalid task intent>': {details or 'validation failed'}"
            )
        ti = task_intent_validation.get("task_intent", {})
        builder_task_intent = {
            "schema": ti.get("schema"),
            "source_file": str(task_intent_path),
            "pick": ti.get("pick"),
            "grasp": ti.get("grasp"),
            "place": ti.get("place"),
            "routing": ti.get("routing"),
            "safety": ti.get("safety"),
        }
        cell_def["builder_task_intent"] = builder_task_intent
        task_recipe_generation = _generate_task_recipe(task_intent_path, output_dir / "task_recipe_from_builder_intent.yaml", scene_path)
        if task_recipe_generation.get("status") != "PASS":
            warnings.append("Task recipe generation from builder intent is partial or failed.")
    else:
        warnings.append("No builder task intent file found; exported scene has physical layout metadata but no pick/place/grasp task intent.")

    cell_path = output_dir / "cell_definition.yaml"
    layout_path = output_dir / "environment_layout.yaml"
    selected_assets_path = output_dir / "selected_assets.json"
    compatibility_path = output_dir / "compatibility_report.json"
    _write_structured(cell_path, cell_def)
    _write_structured(layout_path, environment_layout)
    selected_assets = {"robot": robot_name, "end_effector": ee_name, "sensor": sensors_meta[0] if sensors_meta else None, "environment_assets": [a.get("id") for a in assets], "custom_stl_assets": [a.get("source",{}).get("path") for a in assets if isinstance(a.get("source"), dict) and a.get("source",{}).get("path")], "task_template": task_type, "grasp_strategy": normalized_grasp.get("strategy_id"), "fake_hardware_default": True, "project_name": scene_path.name}
    selected_assets_path.write_text(json.dumps(selected_assets, indent=2)+"\n", encoding="utf-8")
    compatibility_result = "WARN" if robot_meta.get("preview_only") else "OK"
    compatibility_report = {"status": compatibility_result, "runtime_supported": not bool(robot_meta.get("preview_only")), "preview_only": bool(robot_meta.get("preview_only")), "fake_hardware_default": True, "real_hardware_default": False, "warnings": [w for w in warnings if "preview_only" in w or "unknown" in w.lower()]}
    compatibility_path.write_text(json.dumps(compatibility_report, indent=2)+"\n", encoding="utf-8")
    recipe_path = output_dir / "task_recipe_from_builder_intent.yaml"
    if recipe_path.is_file():
        plan_preview_generation = _generate_plan_preview(recipe_path, output_dir / "offline_plan_preview_request.yaml", cell_path, layout_path)

    summary: dict[str, Any] = {
        "generated_by": "workcell_builder",
        "scene_path": str(scene_path),
        "output_dir": str(output_dir),
        "warnings": warnings,
        "exported_files": [str(cell_path), str(layout_path), str(selected_assets_path), str(compatibility_path)] + ([str(task_intent_path)] if task_intent_path else []),
        "validation": {},
        "builder_task_intent": builder_task_intent,
        "task_intent_validation": task_intent_validation,
        "task_recipe_generation": task_recipe_generation,
        "offline_plan_preview_request": plan_preview_generation,
    }

    if validate:
        commands = {
            "cell_definition": ["python3", str(SCRIPT_DIR / "validate_cell_definition.py"), str(cell_path), "--json"],
            "environment_layout": ["python3", str(SCRIPT_DIR / "validate_environment_layout.py"), str(layout_path), "--json"],
        }
        for key, cmd in commands.items():
            run = subprocess.run(cmd, check=False, capture_output=True, text=True)
            parsed: Any
            try:
                parsed = json.loads(run.stdout) if run.stdout.strip() else {"result": "FAIL", "errors": [run.stderr.strip()]}
            except Exception:
                parsed = {"result": "FAIL", "errors": [run.stdout.strip() or run.stderr.strip()]}
            summary["validation"][key] = parsed

    (output_dir / "builder_export_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("scene_path", type=Path)
    ap.add_argument("--output-dir", type=Path)
    ap.add_argument("--validate", action="store_true")
    ap.add_argument("--json", action="store_true", help="Print the export summary JSON to stdout")
    args = ap.parse_args()
    output_dir = args.output_dir or (args.scene_path / "generated")
    try:
        authored_yaml = [
            args.scene_path / "environment.yaml",
            args.scene_path / "layout" / "workcell_studio_layout.yaml",
            args.scene_path / "config" / "workcell_builder_task_intent.yaml",
            args.scene_path / "workcell_builder_task_intent.yaml",
            args.scene_path / "workcell_builder_metadata.yaml",
        ]
        if not authored_yaml[0].is_file():
            raise ValueError(f"required authored file is missing: {authored_yaml[0]}")
        if _pyyaml is not None:
            for source_path in authored_yaml:
                if not source_path.is_file():
                    continue
                try:
                    _pyyaml.safe_load(source_path.read_text(encoding="utf-8"))
                except (_pyyaml.YAMLError, OSError, UnicodeError) as exc:
                    raise ValueError(f"cannot parse authored YAML '{source_path}': {exc}") from exc
        summary = export_scene(args.scene_path, output_dir, validate=args.validate)
    except Exception as exc:  # noqa: BLE001 - command boundary reports authored source.
        print(f"Export failed for authored scene '{args.scene_path}': {exc}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))

    if args.validate:
        validation = summary.get("validation") if isinstance(summary, dict) else None
        if not isinstance(validation, dict) or not validation:
            print(f"Export failed for authored scene '{args.scene_path}': canonical validation results are absent", file=sys.stderr)
            return 1
        failed = [name for name, result in validation.items()
                  if not isinstance(result, dict) or result.get("result") == "FAIL"]
        if failed:
            print(
                f"Export failed for authored scene '{args.scene_path}': canonical validation failed: "
                + ", ".join(sorted(failed)),
                file=sys.stderr,
            )
            return 1

    required_outputs = [
        output_dir / "cell_definition.yaml",
        output_dir / "environment_layout.yaml",
        output_dir / "selected_assets.json",
        output_dir / "compatibility_report.json",
        output_dir / "builder_export_summary.json",
    ]
    missing = [path for path in required_outputs if not path.is_file()]
    if missing:
        print(
            f"Export failed for authored scene '{args.scene_path}': required canonical output is absent: {missing[0]}",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
