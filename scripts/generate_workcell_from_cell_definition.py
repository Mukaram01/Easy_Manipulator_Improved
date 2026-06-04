#!/usr/bin/env python3
"""Generate an offline-reviewable ROS 2 scene package from Cell Definition v1 YAML."""

from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import shutil
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
VALIDATOR_PATH = SCRIPTS_DIR / "validate_cell_definition.py"
SCENE_GENERATOR_PATH = SCRIPTS_DIR / "generate_scene_from_cell_definition.py"
SCENE_CONTRACT_PATH = SCRIPTS_DIR / "validate_scene_contract.py"
DRY_RUN_PATH = SCRIPTS_DIR / "dry_run_task_recipe.py"
PLAN_PATH = SCRIPTS_DIR / "generate_task_execution_plan.py"
BUNDLE_EXPORT_PATH = SCRIPTS_DIR / "export_workcell_bundle.py"
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


def _write_validation_report(report_path: Path, manifest: dict[str, Any], scene_contract: Any, dry_result: Any) -> None:
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
    for note in list(task_notes) + list(dry_result.notes):
        lines.append(f"- {note}")
    if len(lines) == 8:
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
    for key, role in [("support_surfaces", "support_surface"), ("environment", "environment"), ("assets", "asset"), ("objects", "object")]:
        entries = cell_def.get(key, []) if isinstance(cell_def.get(key), list) else []
        for entry in entries:
            if not isinstance(entry, dict):
                continue
            objects.append(
                {
                    "id": entry.get("id", entry.get("name")),
                    "type": entry.get("type", key.rstrip("s")),
                    "role": entry.get("role", role),
                    "dimensions": entry.get("dimensions"),
                    "pose": entry.get("pose"),
                    "primitive_fallback": entry.get("primitive", {"shape": "box"}),
                    "mesh": entry.get("mesh") or entry.get("mesh_path"),
                }
            )
    return {"schema_version": "environment_objects/v1", "objects": objects}


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


def _fallback_visual_mesh_index(package_name: str, package_dir: Path, blockers: list[str]) -> dict[str, Any]:
    return {
        "schema_version": "scene_visual_mesh_index/v1",
        "scene_name": package_name,
        "status": "FALLBACK_EMPTY",
        "safe_for_preview": True,
        "source_urdf_xacro_path": str(package_dir / "urdf" / "scene.urdf.xacro"),
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "visual_count": 0,
        "candidate_mesh_count": 0,
        "emitted_visual_count": 0,
        "resolved": 0,
        "unresolved": 0,
        "visual_items": [],
        "blockers": blockers,
    }


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
    (package_dir / "layout" / "workcell_studio_layout.yaml").write_text(
        yaml.safe_dump(_build_contract_layout(package_name, loaded, source_snapshot), sort_keys=False),
        encoding="utf-8",
    )
    (package_dir / "launch" / "demo.launch.py").write_text(_render_demo_launch(package_name, cell_definition_path), encoding="utf-8")
    (package_dir / "urdf" / "scene.urdf.xacro").write_text(_render_placeholder_scene_xacro(package_name, warnings), encoding="utf-8")
    (package_dir / "generated" / "scene_visual_mesh_index.json").write_text(
        json.dumps(
            _fallback_visual_mesh_index(
                package_name,
                package_dir,
                ["Full visual mesh extraction is not run by generate_workcell_from_cell_definition.py; fallback index emitted."],
            ),
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    if dry_result is not None and scene_contract is not None:
        _write_validation_report(package_dir / "generated" / "validation_report.md", scene_manifest, scene_contract, dry_result)
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
    source_snapshot = _snapshot_scene_package_inputs(cell_definition_path, final_package_dir, loaded, warnings)
    if final_package_dir.exists() and not force:
        print(f"FAIL: Output package already exists: {final_package_dir} (use --force to overwrite)")
        return 1

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

    scene_manifest_path = package_dir / "scene_manifest.yaml"
    task_recipe_path = package_dir / "config" / "task_recipe.yaml"

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
    )

    (package_dir / "README.md").write_text(
        _header_markdown(cell_definition_path)
        + _build_readme(loaded, package_name, cell_definition_path, final_package_dir, warnings, scene_generator, summary.capability_summary),
        encoding="utf-8",
    )

    commissioning_summary = scene_generator.build_commissioning_summary(loaded, warnings, summary.capability_summary)
    (package_dir / "generated" / "commissioning_summary.md").write_text(
        _header_markdown(cell_definition_path) + commissioning_summary,
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

    generated_dir = package_dir / "generated"
    detected_example_path = generated_dir / "generated_detected_objects_example.yaml"
    env_objects_path = generated_dir / "generated_environment_objects.yaml"
    destinations_path = generated_dir / "generated_destinations.yaml"
    summary_path = generated_dir / "generated_workcell_summary.json"
    command_script_path = generated_dir / "generated_gated_dry_run_command.sh"
    detected_example = _build_detected_objects_example(loaded, task_recipe)
    env_objects = _build_environment_objects(loaded)
    env_objects["tracked_assets"] = asset_tracking["tracked"]
    env_objects["unsupported_assets"] = asset_tracking["unsupported"]
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
