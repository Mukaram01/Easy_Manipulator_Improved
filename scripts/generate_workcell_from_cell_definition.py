#!/usr/bin/env python3
"""Generate an offline-reviewable ROS 2 scene package from Cell Definition v1 YAML."""

from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import math
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


def _render_cmakelists(package_name: str) -> str:
    template_path = TEMPLATE_DIR / "CMakeLists_example.txt"
    if template_path.is_file():
        text = template_path.read_text(encoding="utf-8")
        return text.replace("project(workcellexample)", f"project({package_name})")
    return f"""cmake_minimum_required(VERSION 3.5)
project({package_name})
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config urdf generated DESTINATION share/${{PROJECT_NAME}})
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


def _render_scene_urdf_xacro(
    package_name: str,
    cell_def: dict[str, Any],
    asset_tracking: dict[str, Any],
    warnings: list[str],
) -> str:
    """Render a conservative scene-only xacro for offline review.

    The generated xacro intentionally contains no hardware interfaces, ros2_control blocks,
    transmissions, driver plugin configuration, or motion-execution wiring. It is a stable
    placeholder scene contract for RViz/MoveIt review flows until approved robot/tool geometry
    is connected separately.
    """
    robot = cell_def.get("robot", {}) if isinstance(cell_def.get("robot"), dict) else {}
    end_effector = cell_def.get("end_effector", {}) if isinstance(cell_def.get("end_effector"), dict) else {}
    robot_geometry_refs = [robot.get(key) for key in ("urdf", "xacro", "mesh", "mesh_path", "description_package")]
    tool_geometry_refs = [end_effector.get(key) for key in ("urdf", "xacro", "mesh", "mesh_path", "description_package")]
    if not any(str(ref).strip() for ref in robot_geometry_refs if ref is not None):
        warnings.append("scene.urdf.xacro uses a placeholder robot comment because robot geometry is not available in cell_definition.yaml.")
    if not any(str(ref).strip() for ref in tool_geometry_refs if ref is not None):
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
        "",
        "  <!-- Robot geometry placeholder: connect an approved robot description during reviewed simulation setup. -->",
        f"  <!-- Requested robot model: {_safe_xml_comment(robot.get('model', 'unknown'))}; planning group: {_safe_xml_comment(robot.get('planning_group', 'unknown'))}. -->",
        "  <!-- Tool geometry placeholder: connect approved end-effector geometry separately; no execution readiness is implied. -->",
        f"  <!-- Requested end effector: {_safe_xml_comment(end_effector.get('id', 'unknown'))} ({_safe_xml_comment(end_effector.get('type', 'unknown'))}). -->",
    ]
    if not placeholders:
        lines.extend(
            [
                "",
                "  <!-- No support surfaces or objects were available in cell_definition.yaml for placeholder visuals. -->",
            ]
        )
    for item in placeholders:
        xyz = " ".join(f"{value:.6g}" for value in item["xyz"])
        rpy = " ".join(f"{value:.6g}" for value in item["rpy"])
        size = " ".join(f"{value:.6g}" for value in item["dimensions"])
        link_name = _xml_attr(item["link_name"])
        material = "workcell_support_placeholder" if item["role"] == "support_surface" else "workcell_object_placeholder"
        rgba = "0.45 0.45 0.45 0.55" if item["role"] == "support_surface" else "0.1 0.45 0.9 0.75"
        lines.extend(
            [
                "",
                f"  <!-- Placeholder {item['role']}: {_safe_xml_comment(item['id'])} ({_safe_xml_comment(item['type'])}). -->",
                f"  <link name=\"{link_name}\">",
                "    <visual>",
                "      <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />",
                "      <geometry>",
                f"        <box size=\"{_xml_attr(size)}\" />",
                "      </geometry>",
                f"      <material name=\"{material}\"><color rgba=\"{rgba}\" /></material>",
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


def _render_scene_urdf_xacro(package_name: str, env_objects: dict[str, Any], cell_definition_path: Path) -> str:
    """Render a deterministic, preview-only scene URDF from generated environment metadata."""
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
        "extraction_mode": "fallback_empty_safe_preview",
        "source_urdf_xacro_path": str(urdf_path),
        "source_mtime": urdf_path.stat().st_mtime if urdf_path.exists() else None,
        "safe_for_preview": True,
        "candidate_mesh_count": 0,
        "emitted_visual_count": 0,
        "renderable_mesh_count": 0,
        "renderable_item_count": 0,
        "visual_items": [],
        "items": [],
        "blockers": [],
        "warnings": [warning_text],
        "fallback_reason": reason,
        "stale_index": False,
        "stale_reasons": [],
        "package_resolution_diagnostics": {"resolved_packages": [], "shadowed_packages": [], "resolution_paths": []},
        "generated_package_dir": str(package_dir),
    }


def _write_scene_visual_mesh_index(package_name: str, package_dir: Path, warnings: list[str]) -> str | None:
    """Best-effort visual mesh index generation with a preview-safe fallback contract."""
    generated_dir = package_dir / "generated"
    index_path = generated_dir / "scene_visual_mesh_index.json"
    urdf_path = package_dir / "urdf" / "scene.urdf.xacro"

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
            expanded_result = extractor.expand_xacro(urdf_path, package_dir, xargs)
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
        package_map, package_diagnostics = extractor.discover_package_map(package_dir)
        items = extractor.extract_from_urdf(xml_text, package_map)
        mesh_items = [item for item in items if item.get("geometry_type") == "mesh"]
        if not mesh_items:
            return write_fallback(f"no meshes were found in {urdf_path}")
        unresolved = [
            item
            for item in items
            if any(extractor.contains_placeholder(item.get(key, "")) for key in ("id", "link", "parent_link"))
        ]
        safe_for_preview = len(unresolved) == 0 and mode in {"xacro_expanded", "xacro_lite_expanded", "best_effort_recursive"}
        payload = {
            "schema_version": "scene_visual_mesh_index/v1",
            "scene_name": package_name,
            "visual_count": len(items),
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "resolved": sum(1 for item in items if item.get("resolved")),
            "unresolved": sum(1 for item in items if not item.get("resolved")),
            "extractor_version": getattr(extractor, "EXTRACTOR_VERSION", "unknown"),
            "extraction_mode": mode,
            "xacro_available": extractor.discover_xacro_command()[1],
            "source_urdf_xacro_path": str(urdf_path),
            "source_mtime": urdf_path.stat().st_mtime if urdf_path.exists() else None,
            "source_expanded_urdf_path": "generated/expanded_scene_preview.urdf" if mode == "xacro_expanded" else "",
            "fallback_reason": fallback_reason or "",
            "safe_for_preview": safe_for_preview,
            "unresolved_placeholder_count": len(unresolved),
            "candidate_mesh_count": len(mesh_items),
            "emitted_visual_count": len(items),
            "renderable_mesh_count": sum(
                1 for item in mesh_items if item.get("render_expected", True)
            ),
            "renderable_item_count": sum(1 for item in items if item.get("render_expected", True)),
            "visual_items": items,
            "items": items,
            "blockers": [],
            "warnings": [fallback_reason] if fallback_reason else [],
            "stale_index": False,
            "stale_reasons": [],
            "xacro_command": xacro_cmd,
            "package_resolution_diagnostics": package_diagnostics,
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

    package_dir = output_dir / package_name
    if package_dir.exists() and not force:
        print(f"FAIL: Output package already exists: {package_dir} (use --force to overwrite)")
        return 1

    if dry_run:
        print(f"PASS: dry-run successful for package '{package_name}'")
        print(f"WARN count: {len(warnings)}")
        for warning in warnings:
            print(f"WARN: {warning}")
        print(f"Would write package to: {package_dir}")
        return 0

    if package_dir.exists():
        shutil.rmtree(package_dir)

    (package_dir / "config").mkdir(parents=True, exist_ok=True)
    (package_dir / "launch").mkdir(parents=True, exist_ok=True)
    (package_dir / "layout").mkdir(parents=True, exist_ok=True)
    (package_dir / "urdf").mkdir(parents=True, exist_ok=True)
    (package_dir / "generated").mkdir(parents=True, exist_ok=True)

    environment = loaded.get("environment", {}) if isinstance(loaded.get("environment"), dict) else {}
    layout_ref = environment.get("layout")
    if isinstance(layout_ref, str) and layout_ref.strip():
        resolved_layout = _resolve_layout_path(layout_ref, cell_definition_path)
        if resolved_layout and resolved_layout.is_file():
            with resolved_layout.open("r", encoding="utf-8") as handle:
                loaded_layout = yaml.safe_load(handle)
            normalized_layout = _normalize_workcell_studio_layout(loaded_layout)
            (package_dir / "layout" / "workcell_studio_layout.yaml").write_text(
                yaml.safe_dump(normalized_layout, sort_keys=False),
                encoding="utf-8",
            )

    scene_manifest_path = package_dir / "scene_manifest.yaml"
    workcell_yaml_path = package_dir / "workcell.yaml"
    task_recipe_path = package_dir / "config" / "task_recipe.yaml"

    scene_manifest_text = _header_yaml(cell_definition_path) + _yaml_text_from(scene_generator, scene_manifest)
    task_recipe_text = _header_yaml(cell_definition_path) + _yaml_text_from(scene_generator, task_recipe)

    (package_dir / "package.xml").write_text(_render_package_xml(package_name), encoding="utf-8")
    (package_dir / "CMakeLists.txt").write_text(_render_cmakelists(package_name), encoding="utf-8")
    scene_manifest_path.write_text(scene_manifest_text, encoding="utf-8")
    workcell_yaml_path.write_text(scene_manifest_text, encoding="utf-8")
    task_recipe_path.write_text(task_recipe_text, encoding="utf-8")
    (package_dir / "generated" / "task_recipe.preview.yaml").write_text(task_recipe_text, encoding="utf-8")
    (package_dir / "generated" / "scene_manifest.preview.yaml").write_text(scene_manifest_text, encoding="utf-8")
    (package_dir / "urdf" / "scene.urdf.xacro").write_text(
        _render_scene_urdf_xacro(package_name, loaded, asset_tracking, warnings),
        encoding="utf-8",
    )

    (package_dir / "README.md").write_text(
        _header_markdown(cell_definition_path)
        + _build_readme(loaded, package_name, cell_definition_path, package_dir, warnings, scene_generator, summary.capability_summary),
        encoding="utf-8",
    )

    commissioning_summary = scene_generator.build_commissioning_summary(loaded, warnings, summary.capability_summary)
    (package_dir / "generated" / "commissioning_summary.md").write_text(
        _header_markdown(cell_definition_path) + commissioning_summary,
        encoding="utf-8",
    )

    shutil.copy2(cell_definition_path, package_dir / "cell_definition.yaml")

    (package_dir / "launch" / "demo.launch.py").write_text(
        _render_demo_launch(package_name, cell_definition_path),
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

    env_objects = _build_environment_objects(loaded)
    env_objects["tracked_assets"] = asset_tracking["tracked"]
    env_objects["unsupported_assets"] = asset_tracking["unsupported"]
    scene_urdf_path = package_dir / "urdf" / "scene.urdf.xacro"
    scene_urdf_path.write_text(
        _render_scene_urdf_xacro(package_name, env_objects, cell_definition_path),
        encoding="utf-8",
    )

    dry_result = dry_runner.evaluate_scene(package_name, scene_manifest_path)

    with tempfile.TemporaryDirectory(prefix="generated_workcell_plan_") as tmp_plan_dir:
        original_plan_dir = plan_generator.OUTPUT_DIR
        plan_generator.OUTPUT_DIR = Path(tmp_plan_dir)
        try:
            plan_result = plan_generator.evaluate_scene(package_name, scene_manifest_path)
            if plan_result.status == "PASS" and plan_result.markdown_path and plan_result.json_path:
                shutil.copy2(plan_result.markdown_path, package_dir / "generated" / "execution_plan.md")
                shutil.copy2(plan_result.json_path, package_dir / "generated" / "execution_plan.json")
            else:
                warnings.append(_summarize_plan_failure(plan_result, dry_result, scene_manifest_path))
        finally:
            plan_generator.OUTPUT_DIR = original_plan_dir

    _write_scene_visual_mesh_index(package_name, package_dir, warnings)
    _write_validation_report(
        package_dir / "generated" / "validation_report.md",
        scene_manifest,
        scene_contract,
        dry_result,
        generation_warnings=warnings,
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
    preflight_cmd = (
        f"python3 scripts/run_cell_readiness_check.py --scene-package {package_name} "
        f"--task-recipe {task_recipe_path} --detected-objects {detected_example_path} --json"
    )
    gated_cmd = (
        f"python3 scripts/run_generated_cell_cycle.py --scene-package {package_name} "
        f"--task-recipe {task_recipe_path} --detected-objects {detected_example_path} "
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
        "task_recipe_path": str(task_recipe_path),
        "detected_objects_example_path": str(detected_example_path),
        "environment_objects_path": str(env_objects_path),
        "destinations_path": str(destinations_path),
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

    status = "PASS" if not warnings else "WARN"
    print(f"{status}: generated package at {package_dir}")
    print(f"PASS: package.xml -> {package_dir / 'package.xml'}")
    print(f"PASS: scene_manifest.yaml -> {scene_manifest_path}")
    print(f"PASS: scene.urdf.xacro -> {package_dir / 'urdf' / 'scene.urdf.xacro'}")
    print(f"PASS: README.md -> {package_dir / 'README.md'}")
    print(f"PASS: commissioning_summary -> {package_dir / 'generated' / 'commissioning_summary.md'}")
    print(f"PASS: validation_report -> {package_dir / 'generated' / 'validation_report.md'}")
    for warning in warnings:
        print(f"WARN: {warning}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cell_definition", type=Path, help="Path to Cell Definition v1 YAML")
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory where package folder is generated")
    parser.add_argument("--package-name", type=str, required=True, help="Output ROS package name")
    parser.add_argument("--force", action="store_true", help="Overwrite existing generated package directory")
    parser.add_argument("--dry-run", action="store_true", help="Validate and preview actions without writing files")
    args = parser.parse_args()

    return generate_package(
        cell_definition_path=args.cell_definition.resolve(),
        output_dir=args.output_dir.resolve(),
        package_name=args.package_name.strip(),
        force=args.force,
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    raise SystemExit(main())
