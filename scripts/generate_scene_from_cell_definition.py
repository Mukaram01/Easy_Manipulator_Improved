#!/usr/bin/env python3
"""Generate scene/task/commissioning preview artifacts from Cell Definition v1."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import validate_cell_definition as cell_validator
from capability_registry import load_capability_registry


def _task_recipe_type(task_type: str) -> str:
    mapping = {
        "sort_by_colour": "sort",
        "sort_by_shape": "sort",
        "sort_by_class": "sort",
        "garbage_sorting": "sort",
        "inspection_then_place": "inspection_then_place",
        "pick_place": "pick_place",
        "custom": "custom",
    }
    return mapping.get(task_type, task_type)


def _to_yaml_text(data: Any) -> str:
    if cell_validator._pyyaml is not None:
        return str(cell_validator._pyyaml.safe_dump(data, sort_keys=False))

    def _scalar_text(value: Any) -> str:
        if isinstance(value, bool):
            return "true" if value else "false"
        if value is None:
            return "null"
        if isinstance(value, (int, float)):
            return str(value)
        if isinstance(value, list) and all(not isinstance(item, (dict, list)) for item in value):
            return "[" + ", ".join(_scalar_text(item) for item in value) + "]"
        return str(value)

    def dump(value: Any, indent: int = 0) -> list[str]:
        prefix = " " * indent
        if isinstance(value, dict):
            lines: list[str] = []
            for key, child in value.items():
                if isinstance(child, dict):
                    lines.append(f"{prefix}{key}:")
                    lines.extend(dump(child, indent + 2))
                elif isinstance(child, list) and child and all(isinstance(item, dict) for item in child):
                    lines.append(f"{prefix}{key}:")
                    for item in child:
                        first = True
                        for item_key, item_value in item.items():
                            if first and not isinstance(item_value, (dict, list)):
                                lines.append(f"{' ' * (indent + 2)}- {item_key}: {_scalar_text(item_value)}")
                                first = False
                            elif first:
                                lines.append(f"{' ' * (indent + 2)}- {item_key}:")
                                lines.extend(dump(item_value, indent + 6))
                                first = False
                            elif isinstance(item_value, (dict, list)):
                                lines.append(f"{' ' * (indent + 4)}{item_key}:")
                                lines.extend(dump(item_value, indent + 6))
                            else:
                                lines.append(f"{' ' * (indent + 4)}{item_key}: {_scalar_text(item_value)}")
                elif isinstance(child, list):
                    lines.append(f"{prefix}{key}: {_scalar_text(child)}")
                else:
                    lines.append(f"{prefix}{key}: {_scalar_text(child)}")
            return lines
        if isinstance(value, list):
            return [f"{prefix}- {_scalar_text(item)}" for item in value]
        return [f"{prefix}{_scalar_text(value)}"]

    return "\n".join(dump(data)) + "\n"


def _extract_capability_refs(cell_def: dict[str, Any]) -> dict[str, Any]:
    robot = cell_def.get("robot", {}) if isinstance(cell_def.get("robot"), dict) else {}
    end_effector = cell_def.get("end_effector", {}) if isinstance(cell_def.get("end_effector"), dict) else {}
    sensors = cell_def.get("sensors") if isinstance(cell_def.get("sensors"), list) else []
    task = cell_def.get("task", {}) if isinstance(cell_def.get("task"), dict) else {}
    environment = cell_def.get("environment", {}) if isinstance(cell_def.get("environment"), dict) else {}
    assets = environment.get("assets") if isinstance(environment.get("assets"), list) else []
    refs = {
        "robot": robot.get("capability"),
        "end_effector": end_effector.get("capability"),
        "sensors": [item.get("capability") for item in sensors if isinstance(item, dict) and item.get("capability")],
        "task": task.get("capability"),
        "environment_assets": [item.get("capability") for item in assets if isinstance(item, dict) and item.get("capability")],
    }
    return {k: v for k, v in refs.items() if v}


def extract_grasp_strategy_metadata(cell_def: dict[str, Any]) -> dict[str, Any] | None:
    grasp = cell_def.get("grasp") if isinstance(cell_def.get("grasp"), dict) else None
    if not grasp:
        return None
    strategy = grasp.get("strategy") if isinstance(grasp.get("strategy"), dict) else {}
    metadata: dict[str, Any] = {"metadata_only": True, "runtime_applied": False}

    for key in ("strategy_ref",):
        value = grasp.get(key)
        if isinstance(value, str) and value.strip():
            metadata[key] = value

    scalar_fields = (
        "id",
        "label",
        "strategy",
        "approach_axis",
        "orientation_mode",
        "approach_distance_m",
        "retreat_distance_m",
    )
    for field in scalar_fields:
        value = strategy.get(field)
        if value is not None:
            metadata[field] = value

    for list_field in ("tool_families", "allowed_roll_angles_deg", "allowed_yaw_angles_deg", "limitations_warnings"):
        value = strategy.get(list_field)
        if isinstance(value, list) and value:
            metadata[list_field] = value

    for object_field in ("contact", "release"):
        value = strategy.get(object_field)
        if isinstance(value, dict) and value:
            metadata[object_field] = value

    return metadata if len(metadata) > 2 else None


def build_scene_manifest(cell_def: dict[str, Any], capability_summary: dict[str, Any] | None = None) -> dict[str, Any]:
    cell = cell_def.get("cell", {})
    robot = cell_def.get("robot", {})
    end_effector = cell_def.get("end_effector", {})
    environment = cell_def.get("environment", {})
    objects = cell_def.get("objects", [])
    task = cell_def.get("task", {})
    commissioning = cell_def.get("commissioning", {})
    perception = cell_def.get("perception", {}) if isinstance(cell_def.get("perception"), dict) else {}

    self_test = cell_def.get("self_test", {}) if isinstance(cell_def.get("self_test"), dict) else {}
    self_test_object = self_test.get("object", {}) if isinstance(self_test.get("object"), dict) else {}
    self_test_enabled = bool(self_test.get("enabled", commissioning.get("self_test_enabled", True)))
    # Backwards compatibility for definitions created before self-test fixtures
    # had their own source layer.
    if not self_test_object:
        self_test_object = objects[0] if objects else {}

    ee_type = str(end_effector.get("type", "unknown")).strip().lower()
    is_suction = ee_type in {"suction", "vacuum", "vacuum_array"}
    robot_capability_id = robot.get("capability")
    robot_family = None
    if isinstance(robot_capability_id, str) and robot_capability_id.strip():
        record = load_capability_registry().get(robot_capability_id.strip())
        if record and isinstance(record.family, str):
            robot_family = record.family
    if not robot_family:
        model_lower = str(robot.get("model", "")).lower()
        if "delta" in model_lower:
            robot_family = "delta"
        elif "gantry" in model_lower or "cartesian" in model_lower:
            robot_family = "gantry"
        else:
            robot_family = "articulated"
    is_placeholder_family = robot_family in {"delta", "gantry", "cartesian"}
    runtime_blockers = [
        "Placeholder robot family has no approved MoveIt/ros2_control runtime stack yet.",
        "Real hardware execution is disabled for this generated template.",
        "Motion execution requires robot-specific MoveIt/ros2_control integration.",
    ] if is_placeholder_family else []
    manifest = {
        "schema_version": "1.0",
        "scene": {"name": cell.get("id", "generated_cell")},
        "robot": {
            "capability": robot_capability_id,
            "family": robot_family,
            "model": robot.get("model", "unknown"),
            "planning_group": robot.get("planning_group", "manipulator"),
            "base_frame": robot.get("base_frame", "world"),
            "ee_link": robot.get("tool_link", "tool0"),
            "home_named_target": robot.get("home_named_target", "home"),
            "runtime_supported": not is_placeholder_family,
            "preview_only": is_placeholder_family,
            "runtime_blockers": runtime_blockers,
        },
        "planning": {"pipeline": "ompl", "planner_id": "RRTConnectkConfigDefault"},
        "end_effector": {
            "type": "vacuum_array" if ee_type == "vacuum_array" else ("suction" if is_suction else end_effector.get("type", "unknown")),
            "brand": end_effector.get("brand", "unknown"),
            "grasp_frame": end_effector.get("grasp_frame", "tool0"),
            "allowed_touch_links": end_effector.get("allowed_touch_links", []),
            "contact_links": end_effector.get("contact_links", end_effector.get("allowed_touch_links", [])),
        },
        "frames": {
            "world": environment.get("frame", "world"),
            "robot_base": robot.get("base_frame", "world"),
            "grasp_frame": end_effector.get("grasp_frame", "tool0"),
            "support_surface_frame": environment.get("frame", "world"),
        },
        "environment": {
            "support_surface_link": (
                environment.get("support_surfaces", [{}])[0].get("id", "table")
                if isinstance(environment.get("support_surfaces"), list) and environment.get("support_surfaces")
                else "table"
            ),
            "support_surfaces": environment.get("support_surfaces", []),
            "objects": objects,
        },
        "perception": {
            "enabled": bool(perception.get("enabled", False)),
            "mode": perception.get("mode", "disabled"),
            "camera_id": perception.get("camera_id"),
            "frame_id": perception.get("frame_id"),
            "normalized_output_contract": perception.get("normalized_output_contract", "detected_objects/v1"),
        },
        "task_recipe": build_task_recipe(cell_def),
        "home_return": {
            "enabled": True,
            "strategy": "named_target_or_safe_joint_state",
            "named_target": robot.get("home_named_target", "home"),
            "safe_joint_state": robot.get("safe_joint_state", []),
        },
    }
    if self_test_enabled and self_test_object:
        manifest["self_test"] = {
            "enabled": True,
            "object": {
                "id": self_test_object.get("id", "commissioning_box"),
                "shape": self_test_object.get("shape", "box"),
                "frame_id": self_test_object.get("frame", "world"),
                "dimensions": self_test_object.get("dimensions", [0.05, 0.05, 0.05]),
                "pose_xyz": self_test_object.get("pose_xyz", [0.45, 0.0, 0.08]),
                "pose_rpy": self_test_object.get("pose_rpy", [0.0, 0.0, 0.0]),
                "attributes": {
                    "color": self_test_object.get("color", "unknown"),
                    "shape": self_test_object.get("shape", "unknown"),
                    "material": self_test_object.get("material", "unknown"),
                    "class": self_test_object.get("class", "unknown"),
                },
            },
            "expected": {"min_grasp_candidates": 1, "allow_simulated_execution": True},
        }
    grasp_strategy = extract_grasp_strategy_metadata(cell_def)
    if grasp_strategy:
        manifest["grasp_strategy"] = grasp_strategy

    refs = _extract_capability_refs(cell_def)
    if refs:
        manifest["capabilities"] = refs
    if capability_summary and isinstance(capability_summary, dict):
        checks = capability_summary.get("checks", {})
        status = checks.get("status")
        if status:
            manifest.setdefault("generated_defaults", {})
            if isinstance(manifest["generated_defaults"], dict):
                manifest["generated_defaults"]["capability_checks"] = status
    environment = cell_def.get("environment", {}) if isinstance(cell_def.get("environment"), dict) else {}
    if isinstance(environment.get("layout"), str) and environment.get("layout").strip():
        manifest["environment_layout"] = {
            "path": environment.get("layout"),
            "metadata_only": True,
        }
    if isinstance(task, dict) and task.get("recipe") is not None:
        recipe = task.get("recipe")
        if isinstance(recipe, str):
            manifest["task_recipe_metadata"] = {"mode": "external", "path": recipe, "type": task.get("type")}
        elif isinstance(recipe, dict):
            recipe_task = recipe.get("task") if isinstance(recipe.get("task"), dict) else recipe
            rules = recipe_task.get("decision_rules") if isinstance(recipe_task, dict) else []
            manifest["task_recipe_metadata"] = {
                "mode": "embedded",
                "schema_version": recipe.get("schema_version"),
                "type": recipe_task.get("type") if isinstance(recipe_task, dict) else task.get("type"),
                "destinations_count": len(recipe_task.get("destinations", []))
                if isinstance(recipe_task, dict) and isinstance(recipe_task.get("destinations"), list)
                else 0,
                "rules_count": len(rules) if isinstance(rules, list) else 0,
            }
    if is_suction:
        manifest["end_effector"]["supported_grasp_strategies"] = end_effector.get("supported_grasp_strategies", [])
        manifest["end_effector"]["required_io_signals"] = end_effector.get("required_io_signals", [])
        manifest["end_effector"]["release_behavior"] = end_effector.get("release_behavior", {"mode": "vacuum_off"})
        manifest["end_effector"]["metadata_only"] = True
        manifest["end_effector"]["runtime_io_applied"] = False
    if is_placeholder_family:
        manifest["runtime_status"] = "BLOCKED_PREVIEW_ONLY"

    return manifest


def _map_rule(rule: dict[str, Any]) -> dict[str, Any]:
    mapped = {"id": rule.get("id", "rule"), "destination": rule.get("destination")}
    when = rule.get("when") if isinstance(rule.get("when"), dict) else {}
    if when.get("always") is True:
        mapped["when"] = {"default": True}
    else:
        attr_key = None
        attr_value = None
        for key, value in when.items():
            if key == "always":
                continue
            attr_key = key
            attr_value = value
            break
        if attr_key is not None:
            mapped["when"] = {"attribute": attr_key, "equals": attr_value}
        else:
            mapped["when"] = {"default": True}
    mapped["action"] = "place"
    return mapped


def build_task_recipe(cell_def: dict[str, Any]) -> dict[str, Any]:
    task = cell_def.get("task", {})
    perception = cell_def.get("perception", {}) if isinstance(cell_def.get("perception"), dict) else {}
    commissioning = cell_def.get("commissioning", {}) if isinstance(cell_def.get("commissioning"), dict) else {}
    self_test = cell_def.get("self_test", {}) if isinstance(cell_def.get("self_test"), dict) else {}
    perception_backed = str(task.get("object_source", "")).lower() == "perception" or bool(perception.get("enabled", False))
    self_test_enabled = bool(self_test.get("enabled", commissioning.get("self_test_enabled", True)))
    object_source = task.get("object_source") or ("perception" if perception_backed else ("self_test" if self_test_enabled else "fixed_object"))
    source = task.get("perception_source") if object_source == "perception" else task.get("source_object")
    source = source or ("detected_objects/v1" if object_source == "perception" else "detected_object")
    task_type = str(task.get("type", "custom"))
    grasp_strategy = extract_grasp_strategy_metadata(cell_def)
    strategy_type = str((grasp_strategy or {}).get("strategy", "")).lower()
    strategy_ref = str((grasp_strategy or {}).get("strategy_ref", "")).lower()
    default_methods = ["finger", "suction"]
    if strategy_type or strategy_ref:
        if any(token in strategy_type for token in ("suction", "vacuum")):
            default_methods = ["suction"]
        elif any(token in strategy_ref for token in ("suction", "vacuum")):
            default_methods = ["suction"]
        elif any(token in strategy_type for token in ("pinch", "side_grip", "finger")):
            default_methods = ["finger"]
        elif "magnetic" in strategy_type:
            default_methods = ["magnetic"]
    return {
        "enabled": True,
        "recipe_id": task.get("id", "generated_task"),
        "id": task.get("id", "generated_task"),
        "task_type": task_type,
        "type": _task_recipe_type(task_type),
        "name": task.get("id", "Generated Task"),
        "description": f"Preview generated from cell definition task type '{task_type}'.",
        "pick": {
            "source": source,
            "object_source": object_source,
            **({"pick_zone": task.get("pick_zone")} if task.get("pick_zone") else {}),
            "allowed_grasp_methods": default_methods,
            "runtime_note": "Suction/vacuum grasp is metadata only; runtime IO application is disabled.",
            "runtime_io_applied": False,
            **({"grasp_strategy": grasp_strategy} if grasp_strategy else {}),
        },
        "decision_rules": [
            _map_rule(rule) for rule in task.get("rules", []) if isinstance(rule, dict)
        ],
        "destinations": [
            {
                "id": item.get("id"),
                "frame_id": item.get("frame", "world"),
                "pose_xyz": item.get("pose_xyz", [0.0, 0.0, 0.0]),
                "pose_rpy": item.get("pose_rpy", [0.0, 0.0, 0.0]),
                "action": "place",
            }
            for item in task.get("destinations", [])
            if isinstance(item, dict)
        ],
        "expected": {"allow_fallback_rule": True, "require_destination_pose": True},
    }


def build_commissioning_summary(cell_def: dict[str, Any], warnings: list[str], capability_summary: dict[str, Any] | None = None) -> str:
    cell = cell_def.get("cell", {})
    robot = cell_def.get("robot", {})
    end_effector = cell_def.get("end_effector", {})
    camera = cell_def.get("camera", {})
    task = cell_def.get("task", {})
    objects = cell_def.get("objects", [])
    destinations = task.get("destinations", [])


    refs = _extract_capability_refs(cell_def)
    capability_status = None
    if capability_summary and isinstance(capability_summary, dict):
        capability_status = capability_summary.get("checks", {}).get("status")
    lines = [
        "# Commissioning Summary (Preview)",
        "",
        f"- Cell: **{cell.get('name', '(unknown)')}** (`{cell.get('id', '(unknown)')}`)",
        f"- Robot: `{robot.get('model', '(unknown)')}` / planning group `{robot.get('planning_group', '(unknown)')}`",
        f"- End effector: `{end_effector.get('id', '(unknown)')}` ({end_effector.get('type', '(unknown)' )})",
        f"- Camera: `{camera.get('id', '(unknown)')}` ({camera.get('type', '(unknown)')})",
        f"- Task type: `{task.get('type', '(unknown)')}`",
        f"- Objects in definition: `{len(objects)}`",
        f"- Destinations in task: `{len(destinations)}`",
        "",
        "## Warnings",
    ]
    if warnings:
        lines.extend([f"- {warning}" for warning in warnings])
    else:
        lines.append("- None")

    lines.extend(["", "## Capability references"])
    if refs:
        lines.append(f"- Selected capability ids: `{refs}`")
    else:
        lines.append("- None (direct cell_definition fields only)")
    if capability_status:
        lines.append(f"- Capability compatibility status: **{capability_status}**")
    grasp_strategy = extract_grasp_strategy_metadata(cell_def)
    lines.extend(["", "## Grasp strategy"])
    if grasp_strategy:
        lines.append(
            f"- Selected strategy: ref=`{grasp_strategy.get('strategy_ref', '(none)')}`, "
            f"id=`{grasp_strategy.get('id', '(none)')}`, label=`{grasp_strategy.get('label', '(none)')}`"
        )
        lines.append(
            f"- Type/axis: `{grasp_strategy.get('strategy', '(unknown)')}` on `{grasp_strategy.get('approach_axis', '(unknown)')}`"
        )
        lines.append(
            f"- Distances: approach=`{grasp_strategy.get('approach_distance_m', '(n/a)')}` m, retreat=`{grasp_strategy.get('retreat_distance_m', '(n/a)')}` m"
        )
        lines.append("- Note: metadata only; runtime execution still requires engineering review.")
    else:
        lines.append("- No explicit grasp strategy was selected in this cell definition.")

    lines.extend(
        [
            "",
            "## Next commissioning steps",
            "1. Review this preview with controls/robotics engineer.",
            "2. Generate/update workcell_builder scene package using approved metadata.",
            "3. Run scene validation, task checks, and simulation smoke tests.",
            "4. Validate real robot interlocks and operator safety workflow on hardware.",
            "",
            "> Limitation: This preview is not proof of physical reachability, collision-free motion, or safety compliance.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("yaml_path", type=Path, help="Path to cell definition YAML")
    parser.add_argument("--output-dir", type=Path, required=True, help="Output directory for preview artifacts")
    args = parser.parse_args()

    try:
        loaded, parser_name, parser_notes = cell_validator.load_yaml(args.yaml_path)
    except Exception as exc:
        print(f"FAIL: Unable to load cell definition: {exc}")
        return 1

    summary = cell_validator.validate_cell_definition(loaded, args.yaml_path, parser_name, parser_notes)
    status = "PASS" if summary.ok and not summary.warnings else "WARN" if summary.ok else "FAIL"
    if not summary.ok:
        print("FAIL: Cell definition failed validation; preview generation aborted.")
        for error in summary.errors:
            print(f" - {error}")
        return 1

    args.output_dir.mkdir(parents=True, exist_ok=True)
    scene_manifest_path = args.output_dir / "scene_manifest.preview.yaml"
    task_recipe_path = args.output_dir / "task_recipe.preview.yaml"
    commissioning_path = args.output_dir / "commissioning_summary.md"

    scene_manifest = build_scene_manifest(loaded, capability_summary=summary.capability_summary)
    task_recipe = build_task_recipe(loaded)

    scene_manifest_path.write_text(_to_yaml_text(scene_manifest), encoding="utf-8")
    task_recipe_path.write_text(_to_yaml_text(task_recipe), encoding="utf-8")
    commissioning_path.write_text(build_commissioning_summary(loaded, summary.warnings, summary.capability_summary), encoding="utf-8")

    print(f"RESULT: {status}")
    print(f"Wrote: {scene_manifest_path}")
    print(f"Wrote: {task_recipe_path}")
    print(f"Wrote: {commissioning_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
