#!/usr/bin/env python3
"""Generate an offline-reviewable ROS 2 scene package from Cell Definition v1 YAML."""

from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import math
import os
import shutil
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from xml.sax.saxutils import escape

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
VALIDATOR_PATH = SCRIPTS_DIR / "validate_cell_definition.py"
SCENE_GENERATOR_PATH = SCRIPTS_DIR / "generate_scene_from_cell_definition.py"
SCENE_CONTRACT_PATH = SCRIPTS_DIR / "validate_scene_contract.py"
DRY_RUN_PATH = SCRIPTS_DIR / "dry_run_task_recipe.py"
PLAN_PATH = SCRIPTS_DIR / "generate_task_execution_plan.py"
BUNDLE_EXPORT_PATH = SCRIPTS_DIR / "export_workcell_bundle.py"
MESH_INDEX_EXTRACTOR_PATH = SCRIPTS_DIR / "extract_scene_urdf_visual_mesh_index.py"
TEMPLATE_DIR = REPO_ROOT / "workcell_builder" / "workcell_builder" / "templates" / "ros2" / "humble"

SUPPORTED_TASK_TYPES = {
    "pick_place",
    "sort_by_colour",
    "sort_by_shape",
    "sort_by_class",
    "garbage_sorting",
    "inspection_then_place",
    "custom",
}
SUPPORTED_LAYOUT_ASSET_TYPES = {"table", "support_surface", "robot_base", "sensor", "camera", "conveyor", "bin", "fixture"}


def _load_module(module_name: str, module_path: Path):
    if not module_path.is_file():
        raise FileNotFoundError(f"Required helper script not found: {module_path}")
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if not spec or not spec.loader:
        raise RuntimeError(f"Unable to load module from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module




def _repo_relative_path(path_value: Any) -> str:
    if path_value is None:
        return ""
    text = str(path_value)
    if not text:
        return ""
    if text.startswith(("package://", "http://", "https://", "model://")):
        return text
    path = Path(text)
    if not path.is_absolute():
        return text
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except Exception:
        return text


def _portable_source_metadata(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _portable_source_metadata(child) for key, child in value.items()}
    if isinstance(value, list):
        return [_portable_source_metadata(child) for child in value]
    if isinstance(value, str):
        return _repo_relative_path(value)
    return value

def _yaml_text_from(scene_generator: Any, data: Any) -> str:
    return str(scene_generator._to_yaml_text(data))


def _header_yaml(cell_definition_path: Path) -> str:
    return (
        "# GENERATED FILE - DO NOT EDIT DIRECTLY\n"
        f"# Generated from cell_definition YAML: {cell_definition_path}\n"
    )


def _header_markdown(cell_definition_path: Path) -> str:
    return (
        "<!-- GENERATED FILE - DO NOT EDIT DIRECTLY -->\n"
        f"<!-- Generated from cell_definition YAML: {cell_definition_path} -->\n\n"
    )


def _normalize_task_recipe(task_recipe: dict[str, Any], warnings: list[str]) -> dict[str, Any]:
    normalized = copy.deepcopy(task_recipe)
    task_type = str(normalized.get("task_type", normalized.get("type", "custom")))
    if task_type not in SUPPORTED_TASK_TYPES:
        warnings.append(
            f"Unknown task type '{task_type}' in cell definition; using conservative custom task metadata."
        )
        task_type = "custom"
    normalized["task_type"] = task_type

    rule_has_default = False
    for rule in normalized.get("decision_rules", []):
        if not isinstance(rule, dict):
            continue
        when = rule.get("when") if isinstance(rule.get("when"), dict) else {}
        if when.get("default") is True or when.get("always") is True:
            rule_has_default = True

    destination_ids = [item.get("id") for item in normalized.get("destinations", []) if isinstance(item, dict)]
    has_reject_destination = any(
        isinstance(dest_id, str) and ("reject" in dest_id.lower() or "fallback" in dest_id.lower())
        for dest_id in destination_ids
    )

    if not rule_has_default:
        warnings.append("task_recipe has no explicit default/fallback decision rule.")
    if not has_reject_destination:
        warnings.append("task_recipe has no reject/fallback destination id; add one for robust routing.")

    return normalized


def _augment_scene_manifest(cell_def: dict[str, Any], scene_manifest: dict[str, Any], warnings: list[str]) -> dict[str, Any]:
    out = copy.deepcopy(scene_manifest)
    robot = cell_def.get("robot", {}) if isinstance(cell_def.get("robot"), dict) else {}
    end_effector = cell_def.get("end_effector", {}) if isinstance(cell_def.get("end_effector"), dict) else {}
    camera = cell_def.get("camera", {}) if isinstance(cell_def.get("camera"), dict) else {}

    out.setdefault("schema_version", "1.0")
    out.setdefault("generated_defaults", {})
    if isinstance(out["generated_defaults"], dict):
        out["generated_defaults"]["review_required"] = True
        out["generated_defaults"]["notes"] = [
            "Generated from Cell Definition v1 for offline review.",
            "Review robot/EE/camera geometry and safety before runtime use.",
        ]

    if str(robot.get("model", "unknown")).strip().lower() in {"", "unknown"}:
        warnings.append("Robot model is unknown; generated manifest uses conservative placeholder values.")
    if str(end_effector.get("type", "unknown")).strip().lower() in {"", "unknown"}:
        warnings.append("End-effector type is unknown; generated manifest uses conservative placeholder values.")

    out["perception"] = {
        "camera_id": camera.get("id", "unknown_camera"),
        "camera_type": camera.get("type", "unknown"),
        "camera_frame": camera.get("frame", "world"),
        "input_frame_options": [
            out.get("frames", {}).get("world", "world"),
            out.get("robot", {}).get("base_frame", "world"),
        ],
    }

    home_return = out.get("home_return")
    if not isinstance(home_return, dict):
        home_return = {}
        out["home_return"] = home_return
    home_return.setdefault("enabled", True)
    home_return.setdefault("strategy", "named_target_or_safe_joint_state")
    home_return.setdefault("named_target", robot.get("home_named_target", "home"))
    if "safe_joint_state" not in home_return:
        home_return["safe_joint_state"] = robot.get("safe_joint_state", [])

    if not isinstance(home_return.get("safe_joint_state"), list):
        warnings.append("home_return.safe_joint_state was not a list; replacing with empty list.")
        home_return["safe_joint_state"] = []

    if not home_return.get("safe_joint_state") and not str(home_return.get("named_target", "")).strip():
        warnings.append(
            "home_return.safe_joint_state is empty and no home_named_target was provided; update before runtime use."
        )

    self_test = out.get("self_test")
    if isinstance(self_test, dict) and isinstance(self_test.get("object"), dict):
        test_object = self_test["object"]
        shape = str(test_object.get("shape", "box")).strip().lower()
        if shape != "box":
            warnings.append(
                f"self_test.object.shape '{shape}' is not currently supported by validator; using conservative 'box'."
            )
            test_object["shape"] = "box"

    return out


def _render_package_xml(package_name: str) -> str:
    template_path = TEMPLATE_DIR / "package_example.xml"
    if template_path.is_file():
        text = template_path.read_text(encoding="utf-8")
        return text.replace("workcellexample", package_name)
    return f"""<?xml version=\"1.0\"?>
<package format=\"3\">
  <name>{package_name}</name>
  <version>0.1.0</version>
  <description>Generated workcell package from Cell Definition.</description>
  <maintainer email=\"noreply@example.com\">generated</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>python3-yaml</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
"""


def _snapshot_input_file(path: Path | None) -> tuple[Path, str] | None:
    if path is None or not path.is_file():
        return None
    return path, path.read_text(encoding="utf-8")


def _write_snapshot(snapshot: tuple[Path, str] | None, destination: Path) -> bool:
    if snapshot is None:
        return False
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(snapshot[1], encoding="utf-8")
    return True


def _source_environment_yaml_path(cell_definition_path: Path) -> Path | None:
    candidate = cell_definition_path.parent / "environment.yaml"
    return candidate if candidate.is_file() else None


def _source_workcell_studio_layout_path(cell_definition_path: Path) -> Path | None:
    candidate = cell_definition_path.parent / "layout" / "workcell_studio_layout.yaml"
    return candidate if candidate.is_file() else None


def _render_environment_yaml(cell_def: dict[str, Any], scene_generator: Any, cell_definition_path: Path) -> str:
    environment = cell_def.get("environment", {}) if isinstance(cell_def.get("environment"), dict) else {}
    cell = cell_def.get("cell", {}) if isinstance(cell_def.get("cell"), dict) else {}
    payload = {
        "schema_version": "workcell_environment/v1",
        "generated_from": str(cell_definition_path),
        "frame": environment.get("frame", cell.get("planning_frame", "world")),
        "robot": cell_def.get("robot", {}),
        "end_effector": cell_def.get("end_effector", {}),
        "camera": cell_def.get("camera", {}),
        "environment": environment,
        "objects": cell_def.get("objects", []),
        "support_surfaces": environment.get("support_surfaces", []),
        "assets": environment.get("assets", []),
        "fake_hardware_first": True,
        "runtime_execution_enabled": False,
    }
    return _header_yaml(cell_definition_path) + _yaml_text_from(scene_generator, payload)



def _build_perception_adapter_config(cell_def: dict[str, Any], task_recipe: dict[str, Any], warnings: list[str]) -> dict[str, Any]:
    cell = cell_def.get("cell", {}) if isinstance(cell_def.get("cell"), dict) else {}
    camera = cell_def.get("camera", {}) if isinstance(cell_def.get("camera"), dict) else {}
    perception = cell_def.get("perception", {}) if isinstance(cell_def.get("perception"), dict) else {}
    task = cell_def.get("task", {}) if isinstance(cell_def.get("task"), dict) else {}
    scene_id = str(cell.get("id") or cell_def.get("scene_id") or "").strip()
    enabled = bool(perception.get("enabled", camera.get("enabled", False)))
    mode = str(perception.get("mode", perception.get("source_mode", "disabled" if not enabled else "live"))).strip().lower()
    aliases = {"off": "disabled", "none": "disabled", "epd_snapshot": "replay", "replayed_snapshot": "replay", "live_epd": "live"}
    mode = aliases.get(mode, mode)
    if not enabled or mode == "disabled":
        return {"schema_version": "workcell_perception_adapter_config/v1", "status": "NOT_APPLICABLE", "mode": "disabled", "state": "DISABLED", "reason": "scene has no perception requirement", "scene_id": scene_id, "normalized_output_contract": "workcell_perception_snapshot/v1"}
    if mode not in {"replay", "live"}:
        raise ValueError(f"Invalid perception source mode: {mode}")
    camera_id = str(perception.get("camera_id") or camera.get("camera_id") or camera.get("id") or "").strip()
    frame_id = str(perception.get("frame_id") or camera.get("frame_id") or camera.get("frame") or "").strip()
    expected_frames = perception.get("expected_frames") or [f for f in [frame_id, camera.get("optical_frame_id"), (cell_def.get("environment", {}) or {}).get("frame")] if f]
    epd = perception.get("epd_input", {}) if isinstance(perception.get("epd_input"), dict) else {}
    required = perception.get("required_object_classes") or task.get("required_object_classes") or []
    threshold = perception.get("confidence_threshold", 0.5)
    blockers=[]
    if not scene_id: blockers.append("cell.id is required for perception adapter config")
    if not camera_id or camera_id == "UNKNOWN_CAMERA": blockers.append("camera_id is required for perception-backed scenes")
    if not frame_id or frame_id == "UNKNOWN_FRAME": blockers.append("frame_id is required for perception-backed scenes")
    if not epd.get("topic"): blockers.append("perception.epd_input.topic is required")
    if not epd.get("message_type"): blockers.append("perception.epd_input.message_type is required")
    if not isinstance(required, list) or not required: blockers.append("required_object_classes must list at least one class")
    try:
        threshold = float(threshold)
        if threshold < 0.0 or threshold > 1.0: blockers.append("confidence_threshold must be in [0, 1]")
    except Exception:
        blockers.append("confidence_threshold must be numeric")
        threshold = 0.5
    if blockers:
        raise ValueError("Invalid perception-backed scene metadata: " + "; ".join(blockers))
    return {
        "schema_version": "workcell_perception_adapter_config/v1",
        "status": "READY",
        "mode": mode,
        "state": "WAITING",
        "scene_id": scene_id,
        "camera": {"camera_id": camera_id, "frame_id": frame_id},
        "expected_frames": list(dict.fromkeys(str(f) for f in expected_frames if f)),
        "epd_input": {"topic": epd.get("topic"), "message_type": epd.get("message_type")},
        "freshness_timeout_s": float(perception.get("freshness_timeout_s", 2.0)),
        "replay": {"path": perception.get("replay_path", ""), "rate_hz": float(perception.get("replay_rate_hz", 1.0)), "single_step": bool(perception.get("single_step", False)), "loop": bool(perception.get("loop", False))},
        "required_object_classes": required,
        "confidence_threshold": threshold,
        "task_binding": {"task_id": task.get("id", task_recipe.get("id")), "pick_source": (task.get("pick") or {}).get("source_ref") or task.get("source_object"), "object_source": "epd_snapshot"},
        "normalized_output_contract": {"schema_version": "workcell_perception_snapshot/v1", "required_fields": ["scene_id", "camera_id", "timestamp", "frame_id", "objects[].object_id_or_track_id", "objects[].label", "objects[].pose_or_centroid", "objects[].confidence"]},
        "ownership": {"epd": "camera processing, detection, localization, tracking", "workcell_studio": "contract validation, scene/task binding, adapter configuration"},
    }

def _render_cmakelists(package_name: str) -> str:
    template_path = TEMPLATE_DIR / "CMakeLists_example.txt"
    contract_installs = """
install(DIRECTORY config generated layout launch urdf
  DESTINATION share/${PROJECT_NAME}
)
install(FILES environment.yaml cell_definition.yaml scene_manifest.yaml
  DESTINATION share/${PROJECT_NAME}
)
"""
    if template_path.is_file():
        text = template_path.read_text(encoding="utf-8")
        text = text.replace("project(workcellexample)", f"project({package_name})")
        if "install(DIRECTORY config generated layout" not in text:
            text = text.replace("ament_package()", contract_installs + "ament_package()")
        return text
    return f"""cmake_minimum_required(VERSION 3.5)
project({package_name})
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config layout urdf generated DESTINATION share/${{PROJECT_NAME}})
install(FILES environment.yaml cell_definition.yaml scene_manifest.yaml DESTINATION share/${{PROJECT_NAME}})
ament_package()
"""




def _xml_attr(value: Any) -> str:
    return escape(str(value), {'"': '&quot;', "'": '&apos;'})


def _safe_xml_comment(text: str) -> str:
    return str(text).replace("--", "-").replace("<", "(").replace(">", ")")


def _safe_link_name(value: Any, fallback: str) -> str:
    raw = str(value or fallback).strip() or fallback
    safe = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in raw)
    if not safe or safe[0].isdigit():
        safe = f"link_{safe}"
    return safe


def _coerce_float_list(value: Any, default: list[float], expected_len: int) -> list[float]:
    if isinstance(value, list):
        out: list[float] = []
        for item in value[:expected_len]:
            try:
                out.append(float(item))
            except (TypeError, ValueError):
                break
        if len(out) == expected_len:
            return out
    return list(default)


def _pose_xyz_rpy(entry: dict[str, Any]) -> tuple[list[float], list[float]]:
    pose = entry.get("pose") if isinstance(entry.get("pose"), dict) else {}
    xyz = entry.get("pose_xyz", pose.get("xyz"))
    rpy = entry.get("pose_rpy", pose.get("rpy"))
    return (
        _coerce_float_list(xyz, [0.0, 0.0, 0.0], 3),
        _coerce_float_list(rpy, [0.0, 0.0, 0.0], 3),
    )


def _dimensions_xyz(entry: dict[str, Any], default: list[float]) -> list[float]:
    dimensions = entry.get("dimensions")
    primitive = entry.get("primitive") if isinstance(entry.get("primitive"), dict) else {}
    if not isinstance(dimensions, list):
        dimensions = primitive.get("dimensions") or primitive.get("size")
    coerced = _coerce_float_list(dimensions, default, 3)
    return [max(0.001, value) for value in coerced]


def _iter_scene_urdf_placeholders(cell_def: dict[str, Any]) -> list[dict[str, Any]]:
    placeholders: list[dict[str, Any]] = []
    seen: set[str] = set()

    def append(entry: Any, role: str, default_size: list[float]) -> None:
        if not isinstance(entry, dict):
            return
        item_id = str(entry.get("id", entry.get("name", f"{role}_{len(placeholders)+1}"))).strip()
        if not item_id:
            item_id = f"{role}_{len(placeholders)+1}"
        link_name = _safe_link_name(item_id, f"{role}_{len(placeholders)+1}")
        if link_name in seen:
            suffix = 2
            base = link_name
            while f"{base}_{suffix}" in seen:
                suffix += 1
            link_name = f"{base}_{suffix}"
        seen.add(link_name)
        xyz, rpy = _pose_xyz_rpy(entry)
        placeholders.append(
            {
                "id": item_id,
                "link_name": link_name,
                "role": role,
                "type": str(entry.get("type", entry.get("shape", role))),
                "xyz": xyz,
                "rpy": rpy,
                "dimensions": _dimensions_xyz(entry, default_size),
            }
        )

    top_surfaces = cell_def.get("support_surfaces", []) if isinstance(cell_def.get("support_surfaces"), list) else []
    for surface in top_surfaces:
        append(surface, "support_surface", [1.0, 1.0, 0.05])

    environment = cell_def.get("environment", {}) if isinstance(cell_def.get("environment"), dict) else {}
    env_surfaces = environment.get("support_surfaces", []) if isinstance(environment.get("support_surfaces"), list) else []
    for surface in env_surfaces:
        append(surface, "support_surface", [1.0, 1.0, 0.05])

    assets = cell_def.get("assets", []) if isinstance(cell_def.get("assets"), list) else []
    for asset in assets:
        if not isinstance(asset, dict):
            continue
        asset_type = str(asset.get("type", asset.get("role", "asset"))).strip().lower()
        role = "support_surface" if asset_type in SUPPORTED_LAYOUT_ASSET_TYPES else "asset"
        append(asset, role, [0.25, 0.25, 0.25])

    objects = cell_def.get("objects", []) if isinstance(cell_def.get("objects"), list) else []
    for obj in objects:
        append(obj, "object", [0.05, 0.05, 0.05])

    return placeholders


def _render_demo_launch(package_name: str, source_path: Path) -> str:
    return (
        "#!/usr/bin/env python3\n"
        f"\"\"\"Offline-safe generated launch entrypoint for {package_name}.\"\"\"\n\n"
        "from launch import LaunchDescription\n"
        "from launch.actions import DeclareLaunchArgument, LogInfo\n"
        "from launch.substitutions import LaunchConfiguration\n\n\n"
        "def generate_launch_description() -> LaunchDescription:\n"
        "    \"\"\"Generated offline review launch.\n\n"
        "    This launch intentionally avoids runtime robot drivers and physical motion execution.\n"
        "    It is a review-safe placeholder to satisfy generated package audit checks.\n"
        "    \"\"\"\n"
        "    use_fake_hardware = LaunchConfiguration(\"use_fake_hardware\")\n"
        "    launch_rviz = LaunchConfiguration(\"launch_rviz\")\n\n"
        "    return LaunchDescription([\n"
        "        DeclareLaunchArgument(\n"
        "            \"use_fake_hardware\",\n"
        "            default_value=\"true\",\n"
        "            description=\"Keep true for offline-safe simulation/review launch flow.\",\n"
        "        ),\n"
        "        DeclareLaunchArgument(\n"
        "            \"launch_rviz\",\n"
        "            default_value=\"true\",\n"
        "            description=\"Enable RViz for visual review of the generated scene package.\",\n"
        "        ),\n"
        f"        LogInfo(msg=[\"[generated scene] package={package_name} source={source_path}\"]),\n"
        "        LogInfo(msg=[\"[generated scene] offline review launch active (no real hardware drivers).\"]),\n"
        "        LogInfo(msg=[\"[generated scene] placeholder scene xacro is visual-review metadata only; no execution readiness is implied.\"]),\n"
        "        LogInfo(msg=[\"[generated scene] use_fake_hardware:=\", use_fake_hardware]),\n"
        "        LogInfo(msg=[\"[generated scene] launch_rviz:=\", launch_rviz]),\n"
        "    ])\n"
    )

def _build_readme(
    cell_def: dict[str, Any], package_name: str, source_path: Path, package_dir: Path, warnings: list[str], scene_generator: Any, capability_summary: dict[str, Any] | None = None
) -> str:
    cell = cell_def.get("cell", {}) if isinstance(cell_def.get("cell"), dict) else {}
    robot = cell_def.get("robot", {}) if isinstance(cell_def.get("robot"), dict) else {}
    end_effector = cell_def.get("end_effector", {}) if isinstance(cell_def.get("end_effector"), dict) else {}
    camera = cell_def.get("camera", {}) if isinstance(cell_def.get("camera"), dict) else {}
    task = cell_def.get("task", {}) if isinstance(cell_def.get("task"), dict) else {}
    grasp_strategy = scene_generator.extract_grasp_strategy_metadata(cell_def)

    capabilities = (capability_summary or {}).get("capability_refs", {}) if isinstance(capability_summary, dict) else {}
    cap_status = (capability_summary or {}).get("checks", {}).get("status") if isinstance(capability_summary, dict) else None
    lines = [
        f"# Generated Workcell Package: {package_name}",
        "",
        "This package was generated offline from a high-level cell definition YAML.",
        "",
        "## Metadata",
        f"- Cell name/id: `{cell.get('name', '(unknown)')}` / `{cell.get('id', '(unknown)')}`",
        f"- Generated package: `{package_name}`",
        f"- Source cell definition: `{source_path}`",
        f"- Robot: `{robot.get('model', '(unknown)')}`",
        f"- End-effector: `{end_effector.get('id', '(unknown)')} ({end_effector.get('type', '(unknown)')})`",
        f"- Camera: `{camera.get('id', '(unknown)')} ({camera.get('type', '(unknown)')})`",
        f"- Task type: `{task.get('type', '(unknown)')}`",
        "",
        "## Offline checks",
        f"- Validate scene contract: `python3 scripts/validate_scene_contract.py {package_dir / 'scene_manifest.yaml'}`",
        "- Dry-run task recipe (practical): `python3 scripts/dry_run_task_recipe.py --check`",
        "- Generate execution plan: `python3 scripts/generate_task_execution_plan.py --check`",
        "- Export commissioning bundle: `python3 scripts/export_workcell_bundle.py --force`",
        "",
        "## Generation warnings",
    ]
    if warnings:
        lines.extend(f"- {warning}" for warning in warnings)
    else:
        lines.append("- None")

    lines.extend(["", "## Capability references"])
    if capabilities:
        lines.append(f"- Capability ids: `{capabilities}`")
    else:
        lines.append("- None")
    if cap_status:
        lines.append(f"- Capability compatibility status: **{cap_status}**")
    lines.extend(["", "## Grasp strategy"])
    if grasp_strategy:
        lines.append(
            f"- Selected: ref=`{grasp_strategy.get('strategy_ref', '(none)')}`, id=`{grasp_strategy.get('id', '(none)')}`, type=`{grasp_strategy.get('strategy', '(unknown)')}`"
        )
        lines.append("- This is offline metadata only and is not proof of runtime grasp success.")
    else:
        lines.append("- No explicit grasp strategy selected in source cell definition.")

    lines.extend(
        [
            "",
            "## Safety note",
            "Generated package content is not proof of physical reachability, collision-free motion,",
            "or machine safety compliance. Review and commissioning sign-off are required.",
            "",
        ]
    )
    return "\n".join(lines)


def _write_validation_report(
    report_path: Path,
    manifest: dict[str, Any],
    scene_contract: Any,
    dry_result: Any,
    generation_warnings: list[str] | None = None,
) -> None:
    task_status, task_notes = scene_contract.validate_task_recipe_block(manifest)
    lines = [
        "# Generated Scene Validation Report",
        "",
        f"- Task recipe contract status: **{task_status}**",
        f"- Dry-run status: **{dry_result.status}**",
        f"- Dry-run matched rule: `{dry_result.matched_rule_id}`",
        f"- Dry-run selected destination: `{dry_result.selected_destination_id}`",
        "",
        "## Notes",
    ]
    notes = list(task_notes) + list(dry_result.notes)
    if generation_warnings:
        notes.extend(generation_warnings)
    for note in notes:
        lines.append(f"- {note}")
    if not notes:
        lines.append("- None")
    lines.append("")
    report_path.write_text("\n".join(lines), encoding="utf-8")


def _build_detected_objects_example(cell_def: dict[str, Any], task_recipe: dict[str, Any]) -> dict[str, Any]:
    planning_frame = str((cell_def.get("cell", {}) or {}).get("planning_frame", "world"))
    rules = task_recipe.get("decision_rules", []) if isinstance(task_recipe.get("decision_rules"), list) else []
    matched = {}
    for rule in rules:
        when = rule.get("when", {}) if isinstance(rule, dict) and isinstance(rule.get("when"), dict) else {}
        if any(when.get(k) for k in ("class", "label", "colour", "color", "material")):
            matched = when
            break
    support_surfaces = cell_def.get("support_surfaces", []) if isinstance(cell_def.get("support_surfaces"), list) else []
    first_surface = support_surfaces[0] if support_surfaces and isinstance(support_surfaces[0], dict) else {}
    surface_pose = first_surface.get("pose", {}) if isinstance(first_surface.get("pose"), dict) else {}
    surface_dims = first_surface.get("dimensions", [1.0, 1.0, 0.2]) if isinstance(first_surface.get("dimensions"), list) else [1.0, 1.0, 0.2]
    sx, sy, sz = (surface_dims + [1.0, 1.0, 0.2])[:3]
    pose_xyz = surface_pose.get("xyz", [0.5, 0.0, 0.75]) if isinstance(surface_pose.get("xyz"), list) else [0.5, 0.0, 0.75]
    object_pose = [float(pose_xyz[0]), float(pose_xyz[1]), float(pose_xyz[2]) + max(float(sz) * 0.5, 0.05)]
    return {
        "schema_version": "detected_objects/v1",
        "scene_id": str((cell_def.get("cell", {}) or {}).get("id", "generated_scene")),
        "source": {"type": "generated_example", "note": "offline-safe example for gated dry-run"},
        "objects": [
            {
                "name": "generated_example_object_1",
                "class": matched.get("class", "box"),
                "label": matched.get("label", matched.get("class", "demo_item")),
                "colour": matched.get("colour", matched.get("color", "blue")),
                "material": matched.get("material", "plastic"),
                "confidence": 0.9,
                "dimensions": [max(0.03, float(sx) * 0.1), max(0.03, float(sy) * 0.1), 0.05],
                "pose": {"frame_id": planning_frame, "position": object_pose, "orientation_rpy": [0.0, 0.0, 0.0]},
            }
        ],
    }


def _build_destinations_export(task_recipe: dict[str, Any]) -> dict[str, Any]:
    out = {"schema_version": "generated_destinations/v1", "destinations": []}
    for dst in task_recipe.get("destinations", []):
        if not isinstance(dst, dict):
            continue
        pose = dst.get("pose", {}) if isinstance(dst.get("pose"), dict) else {}
        out["destinations"].append(
            {
                "id": dst.get("id"),
                "frame": pose.get("frame", "world"),
                "pose_xyz": pose.get("xyz", [0.0, 0.0, 0.0]),
                "pose_rpy": pose.get("rpy", [0.0, 0.0, 0.0]),
                "role": dst.get("role"),
                "linked_environment_object": dst.get("environment_object"),
            }
        )
    return out


def _build_environment_objects(cell_def: dict[str, Any]) -> dict[str, Any]:
    objects = []

    def append_entry(entry: dict[str, Any], key: str, role: str) -> None:
        pose = entry.get("pose") if isinstance(entry.get("pose"), dict) else {}
        if not pose and (entry.get("pose_xyz") is not None or entry.get("pose_rpy") is not None):
            pose = {
                "xyz": entry.get("pose_xyz", [0.0, 0.0, 0.0]),
                "rpy": entry.get("pose_rpy", [0.0, 0.0, 0.0]),
                "frame": entry.get("frame", "world"),
            }
        objects.append(
            {
                "id": entry.get("id", entry.get("name")),
                "type": entry.get("type", entry.get("shape", key.rstrip("s"))),
                "role": entry.get("role", role),
                "dimensions": entry.get("dimensions"),
                "pose": pose,
                "primitive_fallback": entry.get("primitive", {"shape": entry.get("shape", "box")}),
                "mesh": entry.get("mesh") or entry.get("mesh_path"),
            }
        )

    for key, role in [("support_surfaces", "support_surface"), ("assets", "asset"), ("objects", "object")]:
        entries = cell_def.get(key, []) if isinstance(cell_def.get(key), list) else []
        for entry in entries:
            if isinstance(entry, dict):
                append_entry(entry, key, role)

    environment = cell_def.get("environment", {}) if isinstance(cell_def.get("environment"), dict) else {}
    for key, role in [("support_surfaces", "support_surface"), ("assets", "asset"), ("objects", "object")]:
        entries = environment.get(key, []) if isinstance(environment.get(key), list) else []
        for entry in entries:
            if isinstance(entry, dict):
                append_entry(entry, f"environment.{key}", role)

    return {"schema_version": "environment_objects/v1", "objects": objects}


def _float_list(values: Any, size: int, default: float = 0.0) -> list[float]:
    raw = values if isinstance(values, list) else []
    out: list[float] = []
    for idx in range(size):
        try:
            value = float(raw[idx]) if idx < len(raw) else default
            out.append(value if math.isfinite(value) else default)
        except Exception:
            out.append(default)
    return out


def _xml_attr(value: Any) -> str:
    return escape(str(value), {'"': '&quot;'})


def _render_scene_urdf_xacro(
    package_name: str,
    env_objects: dict[str, Any],
    cell_definition_path: Path | dict[str, Any],
    warnings: list[str] | None = None,
) -> str:
    """Render a deterministic, preview-only scene URDF from generated environment metadata.

    The fourth argument is retained for older unit tests/callers that passed
    ``asset_tracking`` and ``warnings`` into the former renderer overload.
    """
    if isinstance(cell_definition_path, dict):
        # Compatibility path for older callers that pass a full cell definition,
        # asset tracking, and a warnings list. Keep this conservative placeholder
        # contract separate from the generated environment-object renderer below.
        cell_def = env_objects
        asset_tracking = cell_definition_path
        if warnings is not None:
            warnings.append("scene.urdf.xacro uses a placeholder robot comment because robot geometry is not available in cell_definition.yaml.")
            warnings.append("scene.urdf.xacro uses a placeholder tool comment because end-effector geometry is not available in cell_definition.yaml.")
        placeholders = _iter_scene_urdf_placeholders(cell_def)
        tracked_count = len(asset_tracking.get("tracked", [])) if isinstance(asset_tracking, dict) else 0
        unsupported_count = len(asset_tracking.get("unsupported", [])) if isinstance(asset_tracking, dict) else 0
        lines = [
            "<?xml version=\"1.0\"?>",
            f"<!-- GENERATED FILE - conservative offline placeholder for {_safe_xml_comment(package_name)}. -->",
            "<!-- No hardware interfaces, ros2_control blocks, transmissions, or real robot drivers are declared here. -->",
            f"<!-- Asset tracking summary: tracked={tracked_count}, unsupported={unsupported_count}. See urdf/generated_asset_metadata.yaml. -->",
            f"<robot name=\"{_xml_attr(package_name)}\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">",
            "  <link name=\"world\" />",
        ]
        for item in placeholders:
            xyz = " ".join(f"{value:.6g}" for value in item["xyz"])
            rpy = " ".join(f"{value:.6g}" for value in item["rpy"])
            size = " ".join(f"{value:.6g}" for value in item["dimensions"])
            link_name = _xml_attr(item["link_name"])
            lines.extend(
                [
                    f"  <link name=\"{link_name}\">",
                    "    <visual>",
                    "      <geometry>",
                    f"        <box size=\"{_xml_attr(size)}\" />",
                    "      </geometry>",
                    "    </visual>",
                    "  </link>",
                    f"  <joint name=\"world_to_{link_name}\" type=\"fixed\">",
                    "    <parent link=\"world\" />",
                    f"    <child link=\"{link_name}\" />",
                    f"    <origin xyz=\"{_xml_attr(xyz)}\" rpy=\"{_xml_attr(rpy)}\" />",
                    "  </joint>",
                ]
            )
        lines.append("</robot>")
        return "\n".join(lines) + "\n"
    lines = [
        "<?xml version=\"1.0\"?>",
        f"<!-- GENERATED FILE - DO NOT EDIT DIRECTLY. Source cell_definition YAML: {_xml_attr(cell_definition_path)} -->",
        f"<robot name=\"{_xml_attr(package_name)}_scene\">",
        "  <link name=\"world\"/>",
    ]
    objects = env_objects.get("objects") if isinstance(env_objects.get("objects"), list) else []
    emitted_ids: set[str] = set()
    for idx, obj in enumerate(objects):
        if not isinstance(obj, dict):
            continue
        raw_id = str(obj.get("id") or f"generated_object_{idx + 1}")
        safe_id = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in raw_id).strip("_") or f"generated_object_{idx + 1}"
        if safe_id in emitted_ids:
            safe_id = f"{safe_id}_{idx + 1}"
        emitted_ids.add(safe_id)
        pose = obj.get("pose") if isinstance(obj.get("pose"), dict) else {}
        xyz = _float_list(pose.get("xyz"), 3, 0.0)
        rpy = _float_list(pose.get("rpy"), 3, 0.0)
        dims = _float_list(obj.get("dimensions"), 3, 0.1)
        dims = [max(0.001, abs(v)) for v in dims]
        mesh_ref = str(obj.get("mesh") or "").strip()
        link_name = f"{safe_id}_link"
        joint_name = f"{safe_id}_joint"
        visual_name = f"{safe_id}_visual"
        lines.extend(
            [
                f"  <link name=\"{_xml_attr(link_name)}\">",
                f"    <visual name=\"{_xml_attr(visual_name)}\">",
                "      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>",
                "      <geometry>",
            ]
        )
        if mesh_ref:
            lines.append(f"        <mesh filename=\"{_xml_attr(mesh_ref)}\" scale=\"1 1 1\"/>")
        else:
            lines.append(f"        <box size=\"{dims[0]:.6g} {dims[1]:.6g} {dims[2]:.6g}\"/>")
        lines.extend(
            [
                "      </geometry>",
                "    </visual>",
                "  </link>",
                f"  <joint name=\"{_xml_attr(joint_name)}\" type=\"fixed\">",
                "    <parent link=\"world\"/>",
                f"    <child link=\"{_xml_attr(link_name)}\"/>",
                f"    <origin xyz=\"{xyz[0]:.6g} {xyz[1]:.6g} {xyz[2]:.6g}\" rpy=\"{rpy[0]:.6g} {rpy[1]:.6g} {rpy[2]:.6g}\"/>",
                "  </joint>",
            ]
        )
    lines.append("</robot>")
    return "\n".join(lines) + "\n"


def _workspace_root_from_path(path: Path) -> Path | None:
    """Infer a colcon workspace root from a generated package path when possible."""
    resolved = path.expanduser().resolve()
    for candidate in (resolved, *resolved.parents):
        if candidate.name == "src":
            return candidate.parent
        if candidate.name in {"install", "build", "log"}:
            return candidate.parent
    return None


def _determine_workspace_root(
    package_dir: Path, workspace_root: Path | str | None = None
) -> Path | None:
    """Resolve the workspace root used for xacro/package URI discovery.

    Caller-provided roots win, then explicit environment variables, then a
    best-effort inference from common colcon workspace layouts such as
    <workspace>/src/.../<scene_package> or <workspace>/install/share/....
    """
    if workspace_root:
        return Path(workspace_root).expanduser().resolve()

    for env_name in (
        "WORKSPACE_ROOT",
        "COLCON_WORKSPACE_ROOT",
        "ROS_WORKSPACE",
        "WORKCELL_WORKSPACE_ROOT",
    ):
        raw = os.environ.get(env_name, "").strip()
        if raw:
            return Path(raw).expanduser().resolve()

    return _workspace_root_from_path(package_dir)




def _fallback_reason_skipped_ur_robot_macro(reason: str, mode: str = '') -> bool:
    text = str(reason or '').lower()
    return 'ur_robot' in text and ('skipped unresolved macros' in text or str(mode or '') in {'xacro_lite_expanded', 'xacro_lite_fallback'})


def _preview_degraded_fallback_warning(reason: str, mode: str = '') -> str:
    if _fallback_reason_skipped_ur_robot_macro(reason, mode):
        return 'xacro-lite skipped robot macro ur_robot; preview uses degraded fallback geometry'
    return ''

def _fallback_scene_visual_mesh_index(
    package_name: str,
    package_dir: Path,
    urdf_path: Path,
    reason: str,
    warning_text: str,
) -> dict[str, Any]:
    return {
        "schema_version": "scene_visual_mesh_index/v1",
        "scene_name": package_name,
        "visual_count": 0,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "resolved": 0,
        "unresolved": 0,
        "extractor_version": "fallback",
        "path_reference_root": "repository",
        "extraction_mode": "fallback_empty_safe_preview",
        "source_urdf_xacro_path": "urdf/scene.urdf.xacro",
        "source_mtime": urdf_path.stat().st_mtime if urdf_path.exists() else None,
        "safe_for_preview": False,
        "candidate_mesh_count": 0,
        "emitted_visual_count": 0,
        "renderable_mesh_count": 0,
        "renderable_item_count": 0,
        "visual_items": [],
        "items": [],
        "blockers": [warning_text],
        "warnings": [warning_text],
        "fallback_reason": reason,
        "stale_index": False,
        "stale_reasons": [],
        "package_resolution_diagnostics": {"resolved_packages": [], "shadowed_packages": [], "resolution_paths": []},
        "generated_package_dir": _repo_relative_path(package_dir),
    }


def _write_scene_visual_mesh_index(
    package_name: str,
    package_dir: Path,
    warnings: list[str],
    workspace_root: Path | str | None = None,
) -> str | None:
    """Best-effort visual mesh index generation with a preview-safe fallback contract."""
    generated_dir = package_dir / "generated"
    index_path = generated_dir / "scene_visual_mesh_index.json"
    urdf_path = package_dir / "urdf" / "scene.urdf.xacro"
    resolved_workspace_root = _determine_workspace_root(package_dir, workspace_root=workspace_root)

    def write_fallback(reason: str) -> str:
        warning_text = f"Visual mesh extraction fallback for {index_path}: {reason}"
        payload = _fallback_scene_visual_mesh_index(package_name, package_dir, urdf_path, reason, warning_text)
        generated_dir.mkdir(parents=True, exist_ok=True)
        index_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        warnings.append(warning_text)
        return warning_text

    if not urdf_path.is_file():
        return write_fallback(f"source URDF/Xacro does not exist: {urdf_path}")
    if not MESH_INDEX_EXTRACTOR_PATH.is_file():
        return write_fallback(f"mesh index extractor is unavailable: {MESH_INDEX_EXTRACTOR_PATH}")

    try:
        extractor = _load_module("generated_scene_visual_mesh_index_extractor", MESH_INDEX_EXTRACTOR_PATH)
        xargs = {"use_fake_hardware": "true", "robot_prefix": "", "tool_prefix": ""}
        xml_text = ""
        mode = "best_effort_recursive"
        fallback_reason = ""
        xacro_cmd: list[str] = []
        try:
            expanded_result = extractor.expand_xacro(
                urdf_path,
                package_dir,
                xargs,
                workspace_root=resolved_workspace_root,
            )
            if isinstance(expanded_result, tuple) and len(expanded_result) >= 4:
                xml_text, _, fallback_reason, xacro_cmd = expanded_result[:4]
            elif isinstance(expanded_result, tuple) and len(expanded_result) == 3:
                xml_text, mode_hint, fallback_reason = expanded_result
                if mode_hint and not xml_text:
                    mode = str(mode_hint)
            if xml_text:
                mode = "xacro_lite_expanded" if xacro_cmd and xacro_cmd[0] == "xacro-lite" else "xacro_expanded"
        except Exception as exc:
            fallback_reason = f"xacro expansion skipped: {exc}"
        if not xml_text:
            xml_text = urdf_path.read_text(encoding="utf-8", errors="ignore")
        package_map, package_diagnostics = extractor.discover_package_map(
            package_dir,
            workspace_root=resolved_workspace_root,
            package_names=extractor.extract_referenced_package_names(xml_text)
            if hasattr(extractor, "extract_referenced_package_names")
            else None,
        )
        items = extractor.extract_from_urdf(xml_text, package_map)
        mesh_items = [item for item in items if item.get("geometry_type") == "mesh"]
        if not mesh_items:
            return write_fallback(f"no meshes were found in {urdf_path}")
        unresolved = [
            item
            for item in items
            if any(extractor.contains_placeholder(item.get(key, "")) for key in ("id", "link", "parent_link"))
        ]
        renderable_mesh_count = sum(
            1 for item in mesh_items if item.get("render_expected", True)
        )
        renderable_item_count = sum(1 for item in items if item.get("render_expected", True))
        degraded_preview_warning = _preview_degraded_fallback_warning(fallback_reason, mode)
        safe_for_preview = (
            len(unresolved) == 0
            and mode == "xacro_expanded"
            and renderable_mesh_count > 0
            and not degraded_preview_warning
        )
        visual_readiness_reasons: list[str] = []
        if degraded_preview_warning:
            visual_readiness_reasons.append(degraded_preview_warning)
        if not safe_for_preview:
            visual_readiness_reasons.append(
                "visual mesh index was not produced from a fully expanded xacro"
                if mode != "xacro_expanded"
                else "visual mesh index contains unresolved xacro placeholders"
                if unresolved
                else "visual mesh index does not have renderable mesh-backed visuals"
            )
        if renderable_mesh_count <= 0:
            visual_readiness_reasons.append(
                "no renderable mesh-backed visual was resolved; primitive fallback alone is not a PASS condition"
            )
        if fallback_reason:
            visual_readiness_reasons.append(str(fallback_reason))
        visual_readiness_status = "PASS" if not visual_readiness_reasons else "WARN"
        payload = {
            "schema_version": "scene_visual_mesh_index/v1",
            "scene_name": package_name,
            "visual_count": len(items),
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "resolved": sum(1 for item in items if item.get("resolved")),
            "unresolved": sum(1 for item in items if not item.get("resolved")),
            "extractor_version": getattr(extractor, "EXTRACTOR_VERSION", "unknown"),
            "path_reference_root": "repository",
            "extraction_mode": mode,
            "xacro_available": extractor.discover_xacro_command()[1],
            "source_urdf_xacro_path": "urdf/scene.urdf.xacro",
            "source_mtime": urdf_path.stat().st_mtime if urdf_path.exists() else None,
            "source_expanded_urdf_path": "generated/expanded_scene_preview.urdf" if mode == "xacro_expanded" else "",
            "fallback_reason": fallback_reason or "",
            "safe_for_preview": safe_for_preview,
            "unresolved_placeholder_count": len(unresolved),
            "candidate_mesh_count": len(mesh_items),
            "emitted_visual_count": len(items),
            "renderable_mesh_count": renderable_mesh_count,
            "renderable_item_count": renderable_item_count,
            "visual_items": _portable_source_metadata(items),
            "items": _portable_source_metadata(items),
            "blockers": [degraded_preview_warning] if degraded_preview_warning else [],
            "warnings": [reason for reason in (degraded_preview_warning, fallback_reason) if reason],
            "stale_index": False,
            "stale_reasons": [],
            "xacro_command": _portable_source_metadata(xacro_cmd),
            "workspace_root": _repo_relative_path(resolved_workspace_root) if resolved_workspace_root else "",
            "visual_readiness": {
                "status": visual_readiness_status,
                "reasons": visual_readiness_reasons,
            },
            "status": visual_readiness_status,
            "package_resolution_diagnostics": _portable_source_metadata(package_diagnostics),
        }
        generated_dir.mkdir(parents=True, exist_ok=True)
        index_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return None
    except Exception as exc:  # pragma: no cover - defensive optional tooling guard
        return write_fallback(f"mesh index extraction failed but package generation continues: {exc}")

def _resolve_layout_path(layout_path: str, cell_definition_path: Path) -> Path | None:
    raw = Path(layout_path).expanduser()
    candidates = [raw, Path.cwd() / raw, cell_definition_path.parent / raw, REPO_ROOT / raw]
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved and resolved.is_file():
            return resolved
    return None


def _normalize_workcell_studio_layout(layout_data: Any) -> dict[str, Any]:
    normalized: dict[str, Any] = layout_data if isinstance(layout_data, dict) else {}
    normalized["schema_version"] = "workcell_studio_layout/v1"
    raw_items = normalized.get("items")
    items = raw_items if isinstance(raw_items, list) else []
    fixed: list[dict[str, Any]] = []
    for idx, item in enumerate(items):
        if not isinstance(item, dict):
            continue
        item_id = str(item.get("id", "")).strip() or f"item_{idx+1}"
        pose = item.get("pose") if isinstance(item.get("pose"), dict) else {}
        xyz = pose.get("xyz") if isinstance(pose.get("xyz"), list) else []
        rpy = pose.get("rpy") if isinstance(pose.get("rpy"), list) else []
        pose["xyz"] = list(xyz[:3]) + [0.0] * max(0, 3 - len(xyz))
        pose["rpy"] = list(rpy[:3]) + [0.0] * max(0, 3 - len(rpy))
        item["id"] = item_id
        item["pose"] = pose
        fixed.append(item)
    has_camera_metadata = any(
        isinstance(item, dict)
        and (
            "camera_asset_id" in item
            or "pointcloud_topic" in item
            or "perception" in item
            or "camera" in str(item.get("type", "")).lower()
        )
        for item in fixed
    )
    if not has_camera_metadata:
        fixed = [
            item
            for item in fixed
            if "camera" not in str(item.get("type", "")).lower() and "fov" not in str(item.get("type", "")).lower()
        ]
    normalized["items"] = fixed
    return normalized


def _extract_asset_tracking(cell_def: dict[str, Any], warnings: list[str], cell_definition_path: Path) -> dict[str, Any]:
    tracked: list[dict[str, Any]] = []
    supported: list[dict[str, Any]] = []
    unsupported: list[dict[str, Any]] = []
    seen_unsupported: set[str] = set()

    def _append(entry: dict[str, Any], source: str) -> None:
        asset_id = str(entry.get("id", entry.get("name", "unknown_asset")))
        mesh_ref = entry.get("mesh") or entry.get("mesh_path")
        pose = entry.get("pose") if isinstance(entry.get("pose"), dict) else {}
        pose_xyz = pose.get("xyz", entry.get("pose_xyz"))
        pose_rpy = pose.get("rpy", entry.get("pose_rpy"))
        asset_type = str(entry.get("type", "unknown")).strip().lower()
        item = {
            "asset_id": asset_id,
            "asset_type": asset_type or "unknown",
            "source": source,
            "mesh_ref": mesh_ref,
            "transform": {"frame": pose.get("frame", entry.get("frame", "world")), "xyz": pose_xyz, "rpy": pose_rpy},
        }
        tracked.append(item)
        if source == "environment.layout":
            if asset_type in SUPPORTED_LAYOUT_ASSET_TYPES:
                supported.append(item)
            else:
                unsupported.append(item)
                if asset_id not in seen_unsupported:
                    warnings.append(f"Generated preview metadata only: {asset_id}")
                    seen_unsupported.add(asset_id)
        else:
            supported.append(item)

    for key in ("support_surfaces", "environment", "assets", "objects"):
        entries = cell_def.get(key, []) if isinstance(cell_def.get(key), list) else []
        for entry in entries:
            if isinstance(entry, dict):
                _append(entry, f"cell_def.{key}")

    environment = cell_def.get("environment", {}) if isinstance(cell_def.get("environment"), dict) else {}
    layout_path = environment.get("layout")
    if isinstance(layout_path, str) and layout_path.strip():
        resolved = _resolve_layout_path(layout_path, cell_definition_path)
        if resolved and resolved.is_file():
            try:
                validator = _load_module("generated_environment_layout_validator", SCRIPTS_DIR / "validate_environment_layout.py")
                layout_data, _, _ = validator.load_layout(resolved)
                for asset in layout_data.get("assets", []) if isinstance(layout_data.get("assets"), list) else []:
                    if isinstance(asset, dict):
                        _append(asset, "environment.layout")
            except Exception as exc:
                warnings.append(f"Layout asset tracking skipped for '{layout_path}': {exc}")
        else:
            warnings.append(f"Layout asset tracking skipped; file not found: {layout_path}")
    return {"tracked": tracked, "supported": supported, "unsupported": unsupported}


def _run_optional_bundle_export(
    bundle_exporter: Any,
    package_name: str,
    manifest_path: Path,
    generated_dir: Path,
    warnings: list[str],
) -> None:
    try:
        bundle_root = generated_dir / "bundle"
        bundle_exporter.export_scene(package_name, manifest_path, bundle_root, zip_output=False, force=True)
    except Exception as exc:  # pragma: no cover - environment dependent
        warnings.append(f"Optional commissioning bundle export skipped: {exc}")



def _safe_load_yaml_file(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _snapshot_scene_package_inputs(
    cell_definition_path: Path,
    package_dir: Path,
    loaded_cell_definition: dict[str, Any],
    warnings: list[str],
) -> dict[str, Any]:
    """Read all source-side inputs before any package directory overwrite."""
    scene_dir = cell_definition_path.parent
    snapshot: dict[str, Any] = {
        "source_scene_dir": scene_dir,
        "source_cell_definition_path": cell_definition_path,
        "cell_definition_text": cell_definition_path.read_text(encoding="utf-8"),
        "environment": None,
        "environment_text": None,
        "layout_ref_path": None,
        "layout_ref_data": None,
        "canonical_layout_path": None,
        "canonical_layout_data": None,
        "canonical_layout_text": None,
        "existing_scene_manifest": None,
    }

    environment_path = scene_dir / "environment.yaml"
    if environment_path.is_file():
        snapshot["environment_text"] = environment_path.read_text(encoding="utf-8")
        try:
            snapshot["environment"] = yaml.safe_load(snapshot["environment_text"]) or {}
        except Exception as exc:
            warnings.append(f"Existing environment.yaml could not be parsed; synthesizing environment.yaml: {exc}")

    environment = loaded_cell_definition.get("environment", {}) if isinstance(loaded_cell_definition.get("environment"), dict) else {}
    layout_ref = environment.get("layout")
    if isinstance(layout_ref, str) and layout_ref.strip():
        resolved_layout = _resolve_layout_path(layout_ref, cell_definition_path)
        if resolved_layout and resolved_layout.is_file():
            snapshot["layout_ref_path"] = resolved_layout
            try:
                snapshot["layout_ref_data"] = _safe_load_yaml_file(resolved_layout)
            except Exception as exc:
                warnings.append(f"Referenced environment.layout could not be parsed ({resolved_layout}): {exc}")
        else:
            warnings.append(f"Referenced environment.layout source not found before generation: {layout_ref}")

    canonical_layout = scene_dir / "layout" / "workcell_studio_layout.yaml"
    if canonical_layout.is_file():
        snapshot["canonical_layout_path"] = canonical_layout
        snapshot["canonical_layout_text"] = canonical_layout.read_text(encoding="utf-8")
        try:
            snapshot["canonical_layout_data"] = _safe_load_yaml_file(canonical_layout)
        except Exception as exc:
            warnings.append(f"Canonical layout/workcell_studio_layout.yaml could not be parsed: {exc}")

    manifest_path = scene_dir / "scene_manifest.yaml"
    if manifest_path.is_file():
        try:
            snapshot["existing_scene_manifest"] = _safe_load_yaml_file(manifest_path) or {}
        except Exception as exc:
            warnings.append(f"Existing scene_manifest.yaml metadata was not reusable: {exc}")

    if package_dir == scene_dir:
        warnings.append("Output package directory matches source scene directory; source inputs were snapshotted before overwrite.")
    return snapshot


def _synthesize_environment(loaded: dict[str, Any], package_name: str, source_snapshot: dict[str, Any]) -> dict[str, Any]:
    environment = copy.deepcopy(source_snapshot.get("environment")) if isinstance(source_snapshot.get("environment"), dict) else {}
    if not environment:
        cell = loaded.get("cell", {}) if isinstance(loaded.get("cell"), dict) else {}
        robot = loaded.get("robot", {}) if isinstance(loaded.get("robot"), dict) else {}
        end_effector = loaded.get("end_effector", {}) if isinstance(loaded.get("end_effector"), dict) else {}
        camera = loaded.get("camera", {}) if isinstance(loaded.get("camera"), dict) else {}
        task = loaded.get("task", {}) if isinstance(loaded.get("task"), dict) else {}
        environment = {
            "schema_version": "workcell_scene/v1",
            "scene": {"id": cell.get("id", package_name), "name": cell.get("name", package_name)},
            "robot": {"id": robot.get("model", "unknown"), "profile": robot.get("model", "unknown")},
            "tool": {"id": end_effector.get("id", "unknown_tool"), "profile": end_effector.get("type", "unknown")},
            "camera": copy.deepcopy(camera),
            "task": {"template": task.get("type", "custom")},
            "workspace": {"bounds": {"x_min": -1.0, "x_max": 1.0, "y_min": -1.0, "y_max": 1.0, "z_min": 0.0, "z_max": 1.8}},
            "metadata": {"generated_by": "generate_workcell_from_cell_definition.py", "source": "cell_definition.yaml"},
        }
    safety = environment.get("safety") if isinstance(environment.get("safety"), dict) else {}
    safety["fake_hardware_first"] = True
    safety["real_hardware_enabled"] = False
    safety["runtime_execution_enabled"] = False
    safety["motion_command_sent"] = False
    environment["safety"] = safety
    metadata = environment.get("metadata") if isinstance(environment.get("metadata"), dict) else {}
    metadata.setdefault("generated_by", "generate_workcell_from_cell_definition.py")
    metadata["source_cell_definition"] = str(source_snapshot.get("source_cell_definition_path", "cell_definition.yaml"))
    metadata["generated_at"] = datetime.now(timezone.utc).isoformat()
    environment["metadata"] = metadata
    return environment


def _build_contract_layout(package_name: str, loaded: dict[str, Any], source_snapshot: dict[str, Any]) -> dict[str, Any]:
    if source_snapshot.get("canonical_layout_data") is not None:
        layout = source_snapshot["canonical_layout_data"]
    elif source_snapshot.get("layout_ref_data") is not None:
        layout = source_snapshot["layout_ref_data"]
    else:
        cell = loaded.get("cell", {}) if isinstance(loaded.get("cell"), dict) else {}
        layout = {
            "schema_version": "workcell_studio_layout/v1",
            "scene": {"id": cell.get("id", package_name), "name": cell.get("name", package_name)},
            "items": [],
            "metadata": {"generated_by": "generate_workcell_from_cell_definition.py", "source": "empty_fallback"},
        }
    normalized = _normalize_workcell_studio_layout(copy.deepcopy(layout))
    metadata = normalized.get("metadata") if isinstance(normalized.get("metadata"), dict) else {}
    metadata.setdefault("generated_by", "generate_workcell_from_cell_definition.py")
    metadata.setdefault("scene_package", package_name)
    metadata.setdefault("generated_at", datetime.now(timezone.utc).isoformat())
    normalized["metadata"] = metadata
    return normalized


def _render_placeholder_scene_xacro(package_name: str, warnings: list[str]) -> str:
    warning = "Generated conservative placeholder urdf/scene.urdf.xacro; approved robot/environment geometry must be connected before runtime use."
    if warning not in warnings:
        warnings.append(warning)
    return (
        '<?xml version="1.0"?>\n'
        f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{package_name}_scene">\n'
        '  <xacro:arg name="use_fake_hardware" default="true"/>\n'
        '  <!-- Offline-safe placeholder: no real robot drivers or hardware interfaces are declared here. -->\n'
        '  <link name="world"/>\n'
        '  <link name="generated_scene_anchor"/>\n'
        '  <joint name="world_to_generated_scene_anchor" type="fixed">\n'
        '    <parent link="world"/>\n'
        '    <child link="generated_scene_anchor"/>\n'
        '    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        '  </joint>\n'
        '</robot>\n'
    )


def _build_scene_package_readiness(
    package_name: str,
    package_dir: Path,
    required_files: list[Path],
    warnings: list[str],
    blockers: list[str] | None = None,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    contract_files = []
    missing = []
    for path in required_files:
        rel = path.relative_to(package_dir).as_posix()
        exists = path.is_file()
        contract_files.append({"path": rel, "exists": exists})
        if not exists:
            missing.append(f"missing required contract file: {rel}")
    readiness_blockers = list(blockers or []) + missing
    payload = {
        "schema_version": "scene_package_readiness/v1",
        "package_name": package_name,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": "BLOCKED" if readiness_blockers else ("WARN" if warnings else "PASS"),
        "safety": {
            "fake_hardware_first": True,
            "runtime_execution_enabled": False,
            "real_hardware_driver_launch_default": False,
        },
        "contract_files": contract_files,
        "warnings": warnings,
        "blockers": readiness_blockers,
    }
    if extra:
        payload.update(extra)
    return payload


def write_scene_package_contract(
    *,
    package_dir: Path,
    package_name: str,
    cell_definition_path: Path,
    loaded: dict[str, Any],
    scene_manifest: dict[str, Any],
    task_recipe: dict[str, Any],
    asset_tracking: dict[str, Any],
    source_snapshot: dict[str, Any],
    warnings: list[str],
    scene_generator: Any,
    scene_contract: Any | None = None,
    dry_result: Any | None = None,
    readiness_extra: dict[str, Any] | None = None,
    workspace_root: Path | str | None = None,
) -> list[Path]:
    """Write the generated scene package contract from one authoritative helper."""
    (package_dir / "config").mkdir(parents=True, exist_ok=True)
    (package_dir / "launch").mkdir(parents=True, exist_ok=True)
    (package_dir / "layout").mkdir(parents=True, exist_ok=True)
    (package_dir / "urdf").mkdir(parents=True, exist_ok=True)
    (package_dir / "generated").mkdir(parents=True, exist_ok=True)

    scene_manifest = copy.deepcopy(scene_manifest)
    scene_manifest.setdefault("files", {})
    if isinstance(scene_manifest["files"], dict):
        scene_manifest["files"].update(
            {
                "package_xml": "package.xml",
                "cmakelists": "CMakeLists.txt",
                "environment": "environment.yaml",
                "cell_definition": "cell_definition.yaml",
                "scene_manifest": "scene_manifest.yaml",
                "workcell_studio_layout": "layout/workcell_studio_layout.yaml",
                "demo_launch": "launch/demo.launch.py",
                "urdf_xacro": "urdf/scene.urdf.xacro",
                "visual_mesh_index": "generated/scene_visual_mesh_index.json",
                "validation_report": "generated/validation_report.md",
                "scene_package_readiness": "generated/scene_package_readiness.json",
                "perception_adapter_config": "generated/perception_adapter_config.yaml",
            }
        )
    scene_manifest["safety"] = {
        "fake_hardware_first": True,
        "runtime_execution_enabled": False,
        "real_hardware_driver_launch_default": False,
    }

    scene_manifest_path = package_dir / "scene_manifest.yaml"
    task_recipe_path = package_dir / "config" / "task_recipe.yaml"
    scene_manifest_text = _header_yaml(cell_definition_path) + _yaml_text_from(scene_generator, scene_manifest)
    task_recipe_text = _header_yaml(cell_definition_path) + _yaml_text_from(scene_generator, task_recipe)

    (package_dir / "package.xml").write_text(_render_package_xml(package_name), encoding="utf-8")
    (package_dir / "CMakeLists.txt").write_text(_render_cmakelists(package_name), encoding="utf-8")
    environment_text = source_snapshot.get("environment_text")
    if isinstance(environment_text, str):
        (package_dir / "environment.yaml").write_text(environment_text, encoding="utf-8")
    else:
        (package_dir / "environment.yaml").write_text(
            _header_yaml(cell_definition_path) + _yaml_text_from(scene_generator, _synthesize_environment(loaded, package_name, source_snapshot)),
            encoding="utf-8",
        )
    (package_dir / "cell_definition.yaml").write_text(str(source_snapshot["cell_definition_text"]), encoding="utf-8")
    scene_manifest_path.write_text(scene_manifest_text, encoding="utf-8")
    (package_dir / "workcell.yaml").write_text(scene_manifest_text, encoding="utf-8")
    task_recipe_path.write_text(task_recipe_text, encoding="utf-8")
    (package_dir / "generated" / "task_recipe.preview.yaml").write_text(task_recipe_text, encoding="utf-8")
    (package_dir / "generated" / "scene_manifest.preview.yaml").write_text(scene_manifest_text, encoding="utf-8")
    perception_adapter_config = _build_perception_adapter_config(loaded, task_recipe, warnings)
    (package_dir / "generated" / "perception_adapter_config.yaml").write_text(_yaml_text_from(scene_generator, perception_adapter_config), encoding="utf-8")
    canonical_layout_text = source_snapshot.get("canonical_layout_text")
    if isinstance(canonical_layout_text, str):
        (package_dir / "layout" / "workcell_studio_layout.yaml").write_text(canonical_layout_text, encoding="utf-8")
    else:
        (package_dir / "layout" / "workcell_studio_layout.yaml").write_text(
            yaml.safe_dump(_build_contract_layout(package_name, loaded, source_snapshot), sort_keys=False),
            encoding="utf-8",
        )
    (package_dir / "launch" / "demo.launch.py").write_text(_render_demo_launch(package_name, cell_definition_path), encoding="utf-8")
    env_objects = _build_environment_objects(loaded)
    env_objects["tracked_assets"] = asset_tracking["tracked"]
    env_objects["unsupported_assets"] = asset_tracking["unsupported"]
    (package_dir / "urdf" / "scene.urdf.xacro").write_text(
        _render_scene_urdf_xacro(package_name, env_objects, cell_definition_path),
        encoding="utf-8",
    )
    _write_scene_visual_mesh_index(package_name, package_dir, warnings, workspace_root=workspace_root)
    if dry_result is not None and scene_contract is not None:
        _write_validation_report(
            package_dir / "generated" / "validation_report.md",
            scene_manifest,
            scene_contract,
            dry_result,
            generation_warnings=warnings,
        )
    else:
        (package_dir / "generated" / "validation_report.md").write_text(
            _header_markdown(cell_definition_path)
            + "# Generated Scene Validation Report\n\n- Status: **PENDING**\n- Dry-run status: not evaluated yet.\n",
            encoding="utf-8",
        )

    (package_dir / "launch" / "README.md").write_text(
        _header_markdown(cell_definition_path)
        + "# Launch placeholders\n\nGenerated package includes offline-safe demo.launch.py for review. Reuse validated scene launch assets after review.\n",
        encoding="utf-8",
    )
    (package_dir / "urdf" / "README.md").write_text(
        _header_markdown(cell_definition_path)
        + "# URDF placeholders\n\nReview and connect approved robot/environment geometry assets manually.\n",
        encoding="utf-8",
    )
    (package_dir / "urdf" / "generated_asset_metadata.yaml").write_text(
        _header_yaml(cell_definition_path)
        + _yaml_text_from(
            scene_generator,
            {
                "schema_version": "generated_asset_metadata/v1",
                "supported_assets": asset_tracking["supported"],
                "unsupported_assets": asset_tracking["unsupported"],
            },
        ),
        encoding="utf-8",
    )

    required_files = [
        package_dir / "package.xml",
        package_dir / "CMakeLists.txt",
        package_dir / "environment.yaml",
        package_dir / "cell_definition.yaml",
        package_dir / "scene_manifest.yaml",
        package_dir / "layout" / "workcell_studio_layout.yaml",
        package_dir / "launch" / "demo.launch.py",
        package_dir / "urdf" / "scene.urdf.xacro",
        package_dir / "generated" / "scene_visual_mesh_index.json",
        package_dir / "generated" / "validation_report.md",
        package_dir / "generated" / "scene_package_readiness.json",
    ]
    readiness = _build_scene_package_readiness(package_name, package_dir, required_files, warnings, extra=readiness_extra)
    (package_dir / "generated" / "scene_package_readiness.json").write_text(
        json.dumps(readiness, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return required_files


def _summarize_plan_failure(plan_result: Any, dry_result: Any, manifest_path: Path) -> str:
    dry_status = getattr(dry_result, "status", "UNKNOWN")
    matched_rule = getattr(plan_result, "matched_rule_id", "(n/a)") or "(n/a)"
    destination_id = getattr(plan_result, "destination_id", "(n/a)") or "(n/a)"
    notes = [str(n).strip() for n in getattr(plan_result, "notes", []) if str(n).strip()]
    note_text = "; ".join(notes[:3]) if notes else "no additional notes"
    if len(notes) > 3:
        note_text += f"; (+{len(notes)-3} more)"
    return (
        f"Execution plan generation status: {getattr(plan_result, 'status', 'UNKNOWN')} "
        f"(dry-run={dry_status}, matched_rule={matched_rule}, destination={destination_id}, "
        f"manifest={manifest_path}, notes={note_text})"
    )


def generate_package(
    cell_definition_path: Path,
    output_dir: Path,
    package_name: str,
    force: bool,
    dry_run: bool,
    workspace_root: Path | str | None = None,
    existing_package_dir: Path | None = None,
) -> int:
    if not VALIDATOR_PATH.is_file():
        print(f"FAIL: Missing required validation tool: {VALIDATOR_PATH}")
        return 2

    try:
        cell_validator = _load_module("generated_cell_definition_validator", VALIDATOR_PATH)
        scene_generator = _load_module("generated_scene_preview_builder", SCENE_GENERATOR_PATH)
        scene_contract = _load_module("generated_scene_contract_validator", SCENE_CONTRACT_PATH)
        dry_runner = _load_module("generated_task_recipe_dry_run", DRY_RUN_PATH)
        plan_generator = _load_module("generated_task_plan_generator", PLAN_PATH)
        bundle_exporter = _load_module("generated_bundle_exporter", BUNDLE_EXPORT_PATH)
    except Exception as exc:
        print(f"FAIL: Unable to load helper tooling: {exc}")
        return 2

    try:
        loaded, parser_name, parser_notes = cell_validator.load_yaml(cell_definition_path)
    except Exception as exc:
        print(f"FAIL: Unable to load cell definition: {exc}")
        return 1

    summary = cell_validator.validate_cell_definition(loaded, cell_definition_path, parser_name, parser_notes)
    warnings = list(summary.warnings)
    if not summary.ok:
        print("FAIL: Cell definition failed validation.")
        for error in summary.errors:
            print(f" - {error}")
        return 1

    scene_manifest = _augment_scene_manifest(loaded, scene_generator.build_scene_manifest(loaded), warnings)
    asset_tracking = _extract_asset_tracking(loaded, warnings, cell_definition_path)
    task_recipe = _normalize_task_recipe(scene_generator.build_task_recipe(loaded), warnings)
    scene_manifest["task_recipe"] = task_recipe
    scene_manifest["generated_assets"] = {
        "tracked_count": len(asset_tracking["tracked"]),
        "supported_count": len(asset_tracking["supported"]),
        "unsupported_count": len(asset_tracking["unsupported"]),
        "supported": asset_tracking["supported"],
        "unsupported": asset_tracking["unsupported"],
    }

    final_package_dir = output_dir / package_name
    if existing_package_dir is not None:
        existing_package_dir = existing_package_dir.resolve()
        source_scene_dir = cell_definition_path.parent.resolve()
        if existing_package_dir != source_scene_dir:
            print("FAIL: --existing-package-dir must exactly match the cell definition's scene directory")
            return 2
        if existing_package_dir.name != package_name:
            print("FAIL: --package-name must match the existing package directory name")
            return 2
        if not (existing_package_dir / "package.xml").is_file() or not (
            existing_package_dir / "CMakeLists.txt"
        ).is_file():
            print("FAIL: --existing-package-dir requires an existing ROS package (package.xml and CMakeLists.txt)")
            return 2
        final_package_dir = existing_package_dir
    source_snapshot = _snapshot_scene_package_inputs(cell_definition_path, final_package_dir, loaded, warnings)
    if final_package_dir.exists() and not force and existing_package_dir is None:
        print(f"FAIL: Output package already exists: {final_package_dir} (use --force to overwrite)")
        return 1

    source_environment_snapshot = _snapshot_input_file(_source_environment_yaml_path(cell_definition_path))
    resolved_layout_path = None
    environment = loaded.get("environment", {}) if isinstance(loaded.get("environment"), dict) else {}
    layout_ref = environment.get("layout")
    if isinstance(layout_ref, str) and layout_ref.strip():
        resolved_layout_path = _resolve_layout_path(layout_ref, cell_definition_path)
    source_environment_layout_snapshot = _snapshot_input_file(resolved_layout_path)
    source_workcell_layout_snapshot = _snapshot_input_file(_source_workcell_studio_layout_path(cell_definition_path))
    source_cell_definition_snapshot = _snapshot_input_file(cell_definition_path)

    if dry_run:
        print(f"PASS: dry-run successful for package '{package_name}'")
        print(f"WARN count: {len(warnings)}")
        for warning in warnings:
            print(f"WARN: {warning}")
        print(f"Would write package to: {final_package_dir}")
        return 0

    output_dir.mkdir(parents=True, exist_ok=True)
    staging_dir = output_dir / f"{package_name}.tmp-generation"
    if staging_dir.exists():
        shutil.rmtree(staging_dir)
    package_dir = staging_dir
    if package_dir.exists():
        shutil.rmtree(package_dir)

    (package_dir / "config").mkdir(parents=True, exist_ok=True)
    (package_dir / "launch").mkdir(parents=True, exist_ok=True)
    (package_dir / "layout").mkdir(parents=True, exist_ok=True)
    (package_dir / "urdf").mkdir(parents=True, exist_ok=True)
    (package_dir / "generated").mkdir(parents=True, exist_ok=True)

    if source_environment_layout_snapshot is not None:
        _write_snapshot(source_environment_layout_snapshot, package_dir / "environment_layout.yaml")
        _write_snapshot(source_environment_layout_snapshot, package_dir / "generated" / "environment_layout.yaml")

    scene_manifest_path = package_dir / "scene_manifest.yaml"
    dry_scene_manifest_path = package_dir / "generated" / "scene_manifest.dry_run.yaml"
    dry_scene_manifest_text = _header_yaml(cell_definition_path) + _yaml_text_from(scene_generator, scene_manifest)
    dry_scene_manifest_path.write_text(dry_scene_manifest_text, encoding="utf-8")

    env_objects = _build_environment_objects(loaded)
    env_objects["tracked_assets"] = asset_tracking["tracked"]
    env_objects["unsupported_assets"] = asset_tracking["unsupported"]

    dry_result = dry_runner.evaluate_scene(package_name, dry_scene_manifest_path)

    with tempfile.TemporaryDirectory(prefix="generated_workcell_plan_") as tmp_plan_dir:
        original_plan_dir = plan_generator.OUTPUT_DIR
        plan_generator.OUTPUT_DIR = Path(tmp_plan_dir)
        try:
            plan_result = plan_generator.evaluate_scene(package_name, dry_scene_manifest_path)
            if plan_result.status == "PASS" and plan_result.markdown_path and plan_result.json_path:
                shutil.copy2(plan_result.markdown_path, package_dir / "generated" / "execution_plan.md")
                shutil.copy2(plan_result.json_path, package_dir / "generated" / "execution_plan.json")
            else:
                warnings.append(_summarize_plan_failure(plan_result, dry_result, dry_scene_manifest_path))
        finally:
            plan_generator.OUTPUT_DIR = original_plan_dir

    required_contract_files = write_scene_package_contract(
        package_dir=package_dir,
        package_name=package_name,
        cell_definition_path=cell_definition_path,
        loaded=loaded,
        scene_manifest=scene_manifest,
        task_recipe=task_recipe,
        asset_tracking=asset_tracking,
        source_snapshot=source_snapshot,
        warnings=warnings,
        scene_generator=scene_generator,
        scene_contract=scene_contract,
        dry_result=dry_result,
        workspace_root=workspace_root,
        readiness_extra={
            "dry_run_status": getattr(dry_result, "status", "UNKNOWN"),
            "source_inputs": {
                "environment_yaml": str(source_snapshot["source_scene_dir"] / "environment.yaml"),
                "cell_definition_yaml": str(source_snapshot["source_cell_definition_path"]),
                "environment_layout": str(source_snapshot.get("layout_ref_path") or ""),
                "workcell_studio_layout": str(source_snapshot.get("canonical_layout_path") or ""),
                "existing_scene_manifest_used_as_metadata": bool(source_snapshot.get("existing_scene_manifest")),
            },
        },
    )

    (package_dir / "README.md").write_text(
        _header_markdown(cell_definition_path)
        + _build_readme(
            loaded,
            package_name,
            cell_definition_path,
            final_package_dir,
            warnings,
            scene_generator,
            summary.capability_summary,
        ),
        encoding="utf-8",
    )

    commissioning_summary = scene_generator.build_commissioning_summary(loaded, warnings, summary.capability_summary)
    (package_dir / "generated" / "commissioning_summary.md").write_text(
        _header_markdown(cell_definition_path) + commissioning_summary,
        encoding="utf-8",
    )

    generated_dir = package_dir / "generated"
    detected_example_path = generated_dir / "generated_detected_objects_example.yaml"
    env_objects_path = generated_dir / "generated_environment_objects.yaml"
    destinations_path = generated_dir / "generated_destinations.yaml"
    summary_path = generated_dir / "generated_workcell_summary.json"
    command_script_path = generated_dir / "generated_gated_dry_run_command.sh"
    detected_example = _build_detected_objects_example(loaded, task_recipe)
    destinations = _build_destinations_export(task_recipe)
    detected_example_path.write_text(_yaml_text_from(scene_generator, detected_example), encoding="utf-8")
    env_objects_path.write_text(_yaml_text_from(scene_generator, env_objects), encoding="utf-8")
    destinations_path.write_text(_yaml_text_from(scene_generator, destinations), encoding="utf-8")
    final_task_recipe_path = final_package_dir / "config" / "task_recipe.yaml"
    final_detected_example_path = final_package_dir / "generated" / "generated_detected_objects_example.yaml"
    preflight_cmd = (
        f"python3 scripts/run_cell_readiness_check.py --scene-package {package_name} "
        f"--task-recipe {final_task_recipe_path} --detected-objects {final_detected_example_path} --json"
    )
    gated_cmd = (
        f"python3 scripts/run_generated_cell_cycle.py --scene-package {package_name} "
        f"--task-recipe {final_task_recipe_path} --detected-objects {final_detected_example_path} "
        f"--output-dir /tmp/{package_name}_gated_dry_run --min-objects 1 --once --dry-run --no-replay --require-preflight --json"
    )
    runtime = loaded.get("runtime", {}) if isinstance(loaded.get("runtime"), dict) else {}
    runtime_scene_package = runtime.get("scene_package")
    summary_payload = {
        "schema_version": "generated_workcell_bundle/v1",
        "package_name": package_name,
        "source_cell_definition": str(cell_definition_path),
        "scene_package": package_name,
        "runtime_scene_package": runtime_scene_package,
        "planning_frame": str((loaded.get("cell", {}) or {}).get("planning_frame", "world")),
        "robot": loaded.get("robot", {}),
        "end_effector": loaded.get("end_effector", {}),
        "camera": loaded.get("camera", {}),
        "task_recipe_path": str(final_task_recipe_path),
        "detected_objects_example_path": str(final_detected_example_path),
        "environment_objects_path": str(final_package_dir / "generated" / "generated_environment_objects.yaml"),
        "destinations_path": str(final_package_dir / "generated" / "generated_destinations.yaml"),
        "warnings": warnings,
        "tracked_assets": asset_tracking["tracked"],
        "unsupported_assets": asset_tracking["unsupported"],
        "blockers": [],
        "recommended_commands": {"preflight": preflight_cmd, "gated_dry_run": gated_cmd},
        "approval": {"status": "unapproved", "approved_by": None, "approved_at": None, "notes": ""},
        "grasp_strategy": scene_generator.extract_grasp_strategy_metadata(loaded),
    }
    summary_path.write_text(json.dumps(summary_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    command_script_path.write_text(
        "#!/usr/bin/env bash\nset -euo pipefail\n\n"
        f"cd {REPO_ROOT}\n{gated_cmd}\n",
        encoding="utf-8",
    )
    command_script_path.chmod(0o755)

    commissioning = loaded.get("commissioning", {}) if isinstance(loaded.get("commissioning"), dict) else {}
    if bool(commissioning.get("export_bundle", False)):
        _run_optional_bundle_export(bundle_exporter, package_name, scene_manifest_path, package_dir / "generated", warnings)

    if existing_package_dir is not None:
        # Authored inputs stay in the selected package.  Only derived/runtime artifacts
        # from staging are merged back; never replace the package directory itself.
        authored_paths = {
            Path("layout/workcell_studio_layout.yaml"),
            Path("config/workcell_builder_task_intent.yaml"),
            Path("environment.yaml"),
            Path("cell_definition.yaml"),
            Path("scene_manifest.yaml"),
        }
        for staged_path in sorted(package_dir.rglob("*")):
            relative = staged_path.relative_to(package_dir)
            if relative in authored_paths or staged_path.is_dir():
                continue
            destination = final_package_dir / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(staged_path, destination)
        shutil.rmtree(package_dir)
    else:
        backup_dir = output_dir / f"{package_name}.previous-generation"
        if backup_dir.exists():
            shutil.rmtree(backup_dir)
        try:
            if final_package_dir.exists():
                final_package_dir.rename(backup_dir)
            package_dir.rename(final_package_dir)
        except Exception:
            if final_package_dir.exists():
                shutil.rmtree(final_package_dir)
            if backup_dir.exists():
                backup_dir.rename(final_package_dir)
            raise
        else:
            if backup_dir.exists():
                shutil.rmtree(backup_dir)

    final_contract_files = [final_package_dir / path.relative_to(staging_dir) for path in required_contract_files]
    missing_contract_files = [path for path in final_contract_files if not path.is_file()]
    status = "PASS" if not warnings and not missing_contract_files else "WARN"
    print(f"{status}: generated package at {final_package_dir}")
    for contract_path in final_contract_files:
        rel = contract_path.relative_to(final_package_dir).as_posix()
        prefix = "PASS" if contract_path.is_file() else "FAIL"
        print(f"{prefix}: {rel} -> {contract_path}")
    print(f"PASS: README.md -> {final_package_dir / 'README.md'}")
    print(f"PASS: commissioning_summary -> {final_package_dir / 'generated' / 'commissioning_summary.md'}")
    for warning in warnings:
        print(f"WARN: {warning}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cell_definition", type=Path, help="Path to Cell Definition v1 YAML")
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory where package folder is generated")
    parser.add_argument("--package-name", type=str, required=True, help="Output ROS package name")
    parser.add_argument("--force", action="store_true", help="Overwrite existing generated package directory")
    parser.add_argument(
        "--existing-package-dir",
        type=Path,
        default=None,
        help="Refresh derived artifacts inside this existing source package without replacing authored inputs",
    )
    parser.add_argument("--dry-run", action="store_true", help="Validate and preview actions without writing files")
    parser.add_argument(
        "--workspace-root",
        type=Path,
        default=None,
        help="Optional colcon workspace root used to resolve package:// meshes during preview index generation",
    )
    args = parser.parse_args()

    return generate_package(
        cell_definition_path=args.cell_definition.resolve(),
        output_dir=args.output_dir.resolve(),
        package_name=args.package_name.strip(),
        force=args.force,
        dry_run=args.dry_run,
        workspace_root=args.workspace_root.resolve() if args.workspace_root else None,
        existing_package_dir=args.existing_package_dir.resolve() if args.existing_package_dir else None,
    )


if __name__ == "__main__":
    raise SystemExit(main())
