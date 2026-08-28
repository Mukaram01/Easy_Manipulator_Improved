#!/usr/bin/env python3
"""Conservative task_recipe/v1 runtime adapter (offline-first)."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import validate_cell_definition as cell_yaml
import validate_task_recipe as task_validator

SUPPORTED_TYPES = {
    "pick_place",
    "sort_by_colour",
    "sort_by_shape",
    "sort_by_class",
    "garbage_sorting",
    "inspection_then_place",
}

ROS_BOUNDARY = {
    "run_grasp_execution": {
        "topic": "grasp_tasks",
        "service": "grasp_requests",
        "source": "easy_manipulation_deployment/emd_demo_nodes/run_grasp_execution/src/demo_node.cpp",
    },
    "run_grasp_planner": {
        "note": "Planner runs as scene/topic subscriber; no stable task submission service is inferred by this adapter.",
        "source": "easy_manipulation_deployment/emd_demo_nodes/run_grasp_planner/src/demo_node.cpp",
    },
}


@dataclass
class AdapterResult:
    payload: dict[str, Any]
    warnings: list[str]
    errors: list[str]


def _load_yaml_or_json(path: Path) -> tuple[dict[str, Any], str, list[str]]:
    if path.suffix.lower() == ".json":
        loaded = json.loads(path.read_text(encoding="utf-8"))
        if loaded is None:
            loaded = {}
        if not isinstance(loaded, dict):
            raise ValueError("JSON root must be an object/mapping.")
        return loaded, "json", []
    return cell_yaml.load_yaml(path)


def _resolve_task_recipe_path(task_recipe: Path | None, project_dir: Path | None) -> tuple[Path, list[str]]:
    notes: list[str] = []
    if task_recipe is not None:
        return task_recipe, notes
    if project_dir is None:
        raise ValueError("Either --task-recipe or --project-dir must be provided.")

    manifest_path = project_dir / "project_manifest.json"
    if manifest_path.is_file():
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        artifacts = payload.get("artifacts") if isinstance(payload.get("artifacts"), dict) else {}
        for key in ("task_preview", "task_recipe"):
            rel = artifacts.get(key)
            if isinstance(rel, str) and rel.strip():
                candidate = (project_dir / rel).resolve()
                if candidate.is_file():
                    notes.append(f"Resolved task recipe from project manifest artifact '{key}'.")
                    return candidate, notes

    for pattern in (
        "generated_workcell/*/generated/task_recipe.preview.yaml",
        "generated_workcell/*/config/task_recipe.yaml",
    ):
        matches = sorted(project_dir.glob(pattern))
        if matches:
            notes.append(f"Resolved task recipe by pattern '{pattern}'.")
            return matches[0], notes

    raise FileNotFoundError("Unable to auto-discover task recipe from --project-dir.")


def _as_dimensions_list(raw: Any) -> list[float]:
    if isinstance(raw, list) and raw:
        return [float(x) for x in raw]
    if isinstance(raw, dict) and all(k in raw for k in ("x", "y", "z")):
        return [float(raw["x"]), float(raw["y"]), float(raw["z"])]
    return []


def _normalize_objects(doc: dict[str, Any]) -> list[dict[str, Any]]:
    schema_version = str(doc.get("schema_version", ""))
    objects = doc.get("objects") if isinstance(doc.get("objects"), list) else None
    if objects is None:
        if isinstance(doc, list):
            objects = doc
        else:
            raise ValueError("Objects input must provide an 'objects' list or be a top-level list.")
    out: list[dict[str, Any]] = []
    for idx, obj in enumerate(objects):
        if not isinstance(obj, dict):
            raise ValueError(f"objects[{idx}] must be a mapping.")
        is_detected_v1 = schema_version == "detected_objects/v1" or "object_id" in obj
        oid = str(obj.get("id") or obj.get("object_id") or f"object_{idx+1:03d}")
        pose = obj.get("pose") if isinstance(obj.get("pose"), dict) else {}
        pose_frame = str(
            pose.get("frame_id")
            or obj.get("frame_id")
            or doc.get("source", {}).get("frame_id", "unknown")
        )
        colour = obj.get("colour") or obj.get("color")
        shape = obj.get("shape")
        if isinstance(shape, dict):
            shape = shape.get("type")
        attributes = obj.get("attributes") if isinstance(obj.get("attributes"), dict) else {}
        colour = colour or attributes.get("colour")
        shape = shape or attributes.get("shape")
        class_id = obj.get("class_id") or obj.get("class") or obj.get("name")

        attrs = {
            "class": class_id,
            "colour": colour,
            "shape": shape,
            "material": obj.get("material") or attributes.get("material"),
            "inspection_result": obj.get("inspection_result"),
        }
        attrs = {k: v for k, v in attrs.items() if isinstance(v, str) and v.strip()}
        out.append(
            {
                "id": oid,
                "name": obj.get("name") or class_id,
                "class_id": class_id,
                "colour": colour,
                "shape": shape,
                "material": obj.get("material") or attributes.get("material"),
                "inspection_result": obj.get("inspection_result"),
                "confidence": float(obj.get("confidence", 1.0)),
                "frame_id": pose_frame,
                "pose": pose,
                "centroid": obj.get("centroid") if isinstance(obj.get("centroid"), dict) else {},
                "dimensions": _as_dimensions_list(obj.get("dimensions") or obj.get("dimensions_xyz")),
                "preferred_end_effector": obj.get("preferred_end_effector") or obj.get("ee_id"),
                "attributes": attrs,
                "source_type": "detected_objects/v1" if is_detected_v1 else "runtime_objects",
            }
        )
    return out


def _match_rule(rules: list[dict[str, Any]], attrs: dict[str, Any], confidence: float) -> dict[str, Any] | None:
    default_rule: dict[str, Any] | None = None
    for rule in rules:
        when = rule.get("when") if isinstance(rule.get("when"), dict) else {}
        if when.get("default") is True or when.get("always") is True or rule.get("fallback") is True:
            if default_rule is None:
                default_rule = rule
            continue

        matched = True
        if isinstance(when.get("attribute"), str):
            if attrs.get(when["attribute"]) != when.get("equals"):
                matched = False
        if "confidence_below" in when:
            threshold = float(when["confidence_below"])
            if not (confidence < threshold):
                matched = False
        if matched:
            return rule
    return default_rule


def _destination_index(recipe_task: dict[str, Any]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for destination in recipe_task.get("destinations", []):
        if isinstance(destination, dict) and isinstance(destination.get("id"), str):
            out[destination["id"]] = destination
    return out


def _rpy_to_quaternion(rpy: list[float]) -> list[float]:
    roll, pitch, yaw = rpy
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return [qx, qy, qz, qw]


def _as_numeric3(raw: Any) -> list[float] | None:
    if not isinstance(raw, list) or len(raw) != 3:
        return None
    try:
        return [float(raw[0]), float(raw[1]), float(raw[2])]
    except (TypeError, ValueError):
        return None


def _resolve_destination_pose(
    destination: dict[str, Any],
    destination_id: str,
    planning_frame: str,
) -> dict[str, Any]:
    safety_warnings: list[str] = []
    frame_id_raw = destination.get("frame")
    if isinstance(frame_id_raw, str) and frame_id_raw.strip():
        frame_id = frame_id_raw.strip()
    else:
        frame_id = planning_frame
        safety_warnings.append(
            f"Destination '{destination_id}' has no frame; defaulting to planning frame '{planning_frame}'."
        )

    pose_xyz = _as_numeric3(destination.get("pose_xyz"))
    pose_rpy = _as_numeric3(destination.get("pose_rpy"))
    if pose_xyz is None:
        safety_warnings.append(f"Destination '{destination_id}' has no valid pose_xyz; release fallback will be used.")
    if pose_rpy is None:
        safety_warnings.append(f"Destination '{destination_id}' has no valid pose_rpy; release fallback will be used.")

    if frame_id.startswith("camera"):
        safety_warnings.append(f"Destination '{destination_id}' uses suspicious frame '{frame_id}'.")

    resolved_pose: dict[str, Any] | None = None
    if pose_xyz is not None and pose_rpy is not None:
        resolved_pose = {
            "frame_id": frame_id,
            "xyz": pose_xyz,
            "quaternion_xyzw": _rpy_to_quaternion(pose_rpy),
            "rpy": pose_rpy,
        }

    return {
        "destination_id": destination_id,
        "destination_name": destination.get("name") or destination.get("label") or destination_id,
        "destination_label": destination.get("label"),
        "frame_id": frame_id,
        "pose": resolved_pose,
        "approach": destination.get("approach") if isinstance(destination.get("approach"), dict) else None,
        "retreat": destination.get("retreat") if isinstance(destination.get("retreat"), dict) else None,
        "safety_warnings": safety_warnings,
    }


def build_plan(task_recipe: dict[str, Any], objects: list[dict[str, Any]], mode: str, dry_run: bool) -> AdapterResult:
    warnings: list[str] = []
    errors: list[str] = []

    task = task_recipe.get("task") if isinstance(task_recipe.get("task"), dict) else {}
    task_type = str(task.get("type", ""))
    planning_frame = str(task.get("planning_frame") or "world")
    if task_type not in SUPPORTED_TYPES:
        warnings.append(f"task.type '{task_type}' is not in adapter guaranteed set; using conservative generic routing.")

    destination_by_id = _destination_index(task)
    rules = [r for r in task.get("decision_rules", []) if isinstance(r, dict)]

    steps: list[dict[str, Any]] = []
    for idx, obj in enumerate(objects, start=1):
        matched_rule = _match_rule(rules, obj["attributes"], float(obj["confidence"]))
        if not matched_rule:
            errors.append(f"No decision rule matched object '{obj['id']}'.")
            continue
        destination_id = matched_rule.get("destination")
        destination = destination_by_id.get(destination_id) if isinstance(destination_id, str) else None
        if destination is None:
            errors.append(f"Matched destination '{destination_id}' for object '{obj['id']}' is undefined.")
            continue
        resolved_destination = _resolve_destination_pose(destination, str(destination_id), planning_frame)
        for warning in resolved_destination.get("safety_warnings", []):
            warnings.append(f"[{obj['id']}] {warning}")

        step = {
            "step_id": f"route_{idx:03d}_{obj['id']}",
            "task": "pick_route_place",
            "object": {
                "id": obj["id"],
                "name": obj.get("name"),
                "class_id": obj.get("class_id"),
                "colour": obj.get("colour"),
                "shape": obj.get("shape"),
                "material": obj.get("material"),
                "inspection_result": obj.get("inspection_result"),
                "confidence": obj.get("confidence"),
                "frame_id": obj.get("frame_id"),
                "pose": obj.get("pose"),
                "centroid": obj.get("centroid"),
                "dimensions": obj.get("dimensions"),
                "preferred_end_effector": obj.get("preferred_end_effector"),
                "attributes": obj.get("attributes"),
                "source_type": obj.get("source_type"),
            },
            "routing": {
                "matched_rule_id": str(matched_rule.get("id", f"rule_{idx}")),
                "destination_id": destination_id,
                "destination": {
                    "frame": destination.get("frame"),
                    "pose_xyz": destination.get("pose_xyz"),
                    "pose_rpy": destination.get("pose_rpy"),
                    "label": destination.get("label"),
                    "action": destination.get("action", "place"),
                },
                "destination_resolved": resolved_destination,
            },
            "execution": {
                "mode": mode,
                "dry_run": dry_run,
                "planner_adapter": "run_grasp_planner",
                "execution_adapter": "run_grasp_execution",
                "runtime_status": "offline_planned" if (mode == "offline" or dry_run) else "runtime_candidate",
            },
        }
        steps.append(step)

    if mode == "ros":
        warnings.append("ROS mode is a guarded placeholder; no live ROS topics/services are called by this adapter.")

    payload = {
        "schema_version": "runtime_execution_plan/v1",
        "metadata": {
            "generated_at_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "adapter": "scripts/run_task_recipe_adapter.py",
            "mode": mode,
            "dry_run": dry_run,
            "planning_frame": planning_frame,
        },
        "task_recipe": {
            "id": task.get("id"),
            "type": task_type,
            "source": task.get("source") or task.get("source_object") or task.get("perception_source"),
        },
        "ros_bridge_boundary": ROS_BOUNDARY,
        "summary": {
            "object_count": len(objects),
            "routed_count": len(steps),
            "unrouted_count": max(0, len(objects) - len(steps)),
        },
        "steps": steps,
        "warnings": warnings,
        "errors": errors,
    }
    return AdapterResult(payload=payload, warnings=warnings, errors=errors)


def _emit(payload: dict[str, Any], output: Path | None, as_json: bool) -> None:
    rendered = json.dumps(payload, indent=2, sort_keys=True)
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")
    if as_json or output is None:
        print(rendered)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-recipe", type=Path, help="Path to task_recipe/v1 YAML or JSON file.")
    parser.add_argument("--project-dir", type=Path, help="Optional workcell project directory for recipe auto-discovery.")
    parser.add_argument("--objects", type=Path, required=True, help="Detected/mock objects input YAML or JSON.")
    parser.add_argument("--output", type=Path, help="Optional output plan path.")
    parser.add_argument("--json", action="store_true", help="Print JSON output to stdout.")
    parser.add_argument("--strict", action="store_true", help="Treat warnings as failures.")
    parser.add_argument("--dry-run", action=argparse.BooleanOptionalAction, default=True, help="Enable/disable dry-run mode.")
    parser.add_argument("--mode", choices=["offline", "ros"], default="offline", help="Execution mode; defaults to offline.")
    args = parser.parse_args(argv)

    notes: list[str] = []
    errors: list[str] = []
    warnings: list[str] = []

    try:
        recipe_path, resolve_notes = _resolve_task_recipe_path(args.task_recipe, args.project_dir)
        notes.extend(resolve_notes)
        recipe_loaded, parser_name, parser_notes = _load_yaml_or_json(recipe_path)
        notes.append(f"Task recipe parser: {parser_name}.")
        notes.extend(parser_notes)

        summary = task_validator.validate_task_recipe_doc(
            recipe_loaded,
            recipe_path,
            parser_name,
            parser_notes,
            strict=args.strict,
        )
        errors.extend(summary.errors)
        warnings.extend(summary.warnings)

        obj_loaded, obj_parser, obj_notes = _load_yaml_or_json(args.objects)
        notes.append(f"Objects parser: {obj_parser}.")
        notes.extend(obj_notes)
        objects = _normalize_objects(obj_loaded)

        result = build_plan(recipe_loaded, objects, mode=args.mode, dry_run=bool(args.dry_run))
        warnings.extend(result.warnings)
        errors.extend(result.errors)

        result.payload["metadata"]["task_recipe_path"] = str(recipe_path)
        result.payload["metadata"]["objects_path"] = str(args.objects)
        if args.project_dir:
            result.payload["metadata"]["project_dir"] = str(args.project_dir)
        result.payload["notes"] = notes

        _emit(result.payload, args.output, args.json)

    except Exception as exc:
        failure = {
            "schema_version": "runtime_execution_plan/v1",
            "result": "FAIL",
            "error": str(exc),
        }
        print(json.dumps(failure, indent=2, sort_keys=True))
        return 2

    blocking = bool(errors) or (args.strict and bool(warnings))
    return 1 if blocking else 0


if __name__ == "__main__":
    raise SystemExit(main())
