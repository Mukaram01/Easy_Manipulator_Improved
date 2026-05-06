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

try:
    import yaml as _pyyaml
except Exception:
    _pyyaml = None


def _load_optional(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    loaded, _ = load_structured_data(path)
    return loaded if isinstance(loaded, dict) else {}




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
    for rel in ["generated/workcell_builder_task_intent.yaml", "workcell_builder_task_intent.yaml"]:
        cand = scene_path / rel
        if cand.is_file():
            return cand
    return None


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


def export_scene(scene_path: Path, output_dir: Path, validate: bool) -> dict[str, Any]:
    env = _load_optional(scene_path / "environment.yaml")
    meta = _load_optional(scene_path / "workcell_builder_metadata.yaml")
    warnings: list[str] = []
    task_intent_path = _find_task_intent(scene_path)
    builder_task_intent: dict[str, Any] = {}
    task_intent_validation: dict[str, Any] = {}
    task_recipe_generation: dict[str, Any] = {}
    plan_preview_generation: dict[str, Any] = {}

    robot_env = env.get("robot") if isinstance(env.get("robot"), dict) else {}
    ee_env = env.get("end_effector") if isinstance(env.get("end_effector"), dict) else {}
    objects_env = env.get("objects") if isinstance(env.get("objects"), dict) else {}

    robot_meta = meta.get("robot") if isinstance(meta.get("robot"), dict) else {}
    ee_meta = meta.get("end_effector") if isinstance(meta.get("end_effector"), dict) else {}
    grasp_meta = meta.get("grasp_strategy") if isinstance(meta.get("grasp_strategy"), dict) else {}
    sensors_meta = meta.get("sensors") if isinstance(meta.get("sensors"), list) else []

    robot_name = robot_env.get("name") or robot_meta.get("selected_name") or "unknown_robot"
    ee_name = ee_env.get("name") or ee_meta.get("selected_name") or "unknown_end_effector"

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

    environment_layout = {
        "schema_version": "environment_layout/v1",
        "layout_id": f"{scene_path.name}_layout",
        "name": f"{scene_path.name} Builder Layout",
        "frame": "world",
        "assets": assets,
        "source_metadata": {
            "generated_by": "workcell_builder",
            "raw_environment_keys": sorted(env.keys()),
        },
    }

    cell_def = {
        "schema_version": "cell_definition/v1",
        "cell": {"id": scene_path.name, "name": scene_path.name, "planning_frame": "world"},
        "robot": {
            "name": robot_name,
            "model": robot_name,
            "capability": robot_meta.get("capability_id"),
            "planning_group": robot_env.get("planning_group", "manipulator"),
            "base_frame": robot_env.get("base_link", "base_link"),
            "tool_link": ee_env.get("parent_link", "tool0"),
            "home_named_target": "home",
        },
        "end_effector": {
            "id": ee_name,
            "name": ee_name,
            "capability": ee_meta.get("capability_id"),
            "type": ee_meta.get("family"),
            "grasp_frame": ee_env.get("base_link", "tool0"),
            "allowed_touch_links": [],
        },
        "camera": ({"id": (sensors_meta[0].get("capability_id") or "realsense_d435i"), "type": (sensors_meta[0].get("family") or "depth_camera"), "frame": "camera_depth_optical_frame"} if sensors_meta and isinstance(sensors_meta[0], dict) else {"id": "realsense_d435i", "type": "depth_camera", "frame": "camera_depth_optical_frame"}),
        "environment": {
            "frame": "world",
            "layout": "generated/environment_layout.yaml",
            "support_surfaces": [{"id": a["id"], "type": "table", "frame": "world", "pose_xyz": a["pose"]["xyz"], "pose_rpy": a["pose"]["rpy"], "dimensions": a["dimensions"]} for a in assets],
        },
        "objects": [{"id": o["id"], "class": "part", "shape": "mesh", "color": "unknown", "material": "unknown", "frame": "world", "dimensions": o["dimensions"], "pose_xyz": [0.0,0.0,0.0], "pose_rpy": [0.0,0.0,0.0]} for o in object_entries],
        "task": {
            "id": "default_task",
            "type": "pick_place",
            "source_object": object_entries[0]["id"] if object_entries else "unknown_object",
            "destinations": [{"id": "default_drop", "frame": "world", "pose_xyz": [0.4, 0.0, 0.2], "pose_rpy": [0.0, 0.0, 0.0]}],
            "rules": [{"id": "default_rule", "when": {"always": True}, "destination": "default_drop"}],
        },
        "grasp": {"strategy_ref": grasp_meta.get("strategy_id")},
        "commissioning": {"self_test_enabled": True, "export_bundle": False, "generated_by": "workcell_builder", "review_required": True, "fake_hardware_first": True, "runtime_send_disabled_by_default": True},
    }

    if not robot_meta.get("capability_id"):
        warnings.append("Robot capability is unknown; review required before runtime commissioning.")
    if robot_meta.get("preview_only"):
        warnings.append("Selected robot is preview_only; runtime execution remains blocked.")

    if task_intent_path:
        task_intent_validation = _validate_task_intent(task_intent_path, scene_path)
        if task_intent_validation.get("status") == "FAIL":
            warnings.append("Builder task intent validation failed; preserving metadata for review.")
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
    _write_structured(cell_path, cell_def)
    _write_structured(layout_path, environment_layout)
    recipe_path = output_dir / "task_recipe_from_builder_intent.yaml"
    if recipe_path.is_file():
        plan_preview_generation = _generate_plan_preview(recipe_path, output_dir / "offline_plan_preview_request.yaml", cell_path, layout_path)

    summary: dict[str, Any] = {
        "generated_by": "workcell_builder",
        "scene_path": str(scene_path),
        "output_dir": str(output_dir),
        "warnings": warnings,
        "exported_files": [str(cell_path), str(layout_path)],
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
    args = ap.parse_args()
    output_dir = args.output_dir or (args.scene_path / "generated")
    export_scene(args.scene_path, output_dir, validate=args.validate)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
