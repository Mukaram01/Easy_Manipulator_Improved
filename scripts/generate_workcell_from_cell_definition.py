#!/usr/bin/env python3
"""Generate an offline-reviewable ROS 2 scene package from Cell Definition v1 YAML."""

from __future__ import annotations

import argparse
import copy
import importlib.util
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any

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


def _build_readme(cell_def: dict[str, Any], package_name: str, source_path: Path, warnings: list[str]) -> str:
    cell = cell_def.get("cell", {}) if isinstance(cell_def.get("cell"), dict) else {}
    robot = cell_def.get("robot", {}) if isinstance(cell_def.get("robot"), dict) else {}
    end_effector = cell_def.get("end_effector", {}) if isinstance(cell_def.get("end_effector"), dict) else {}
    camera = cell_def.get("camera", {}) if isinstance(cell_def.get("camera"), dict) else {}
    task = cell_def.get("task", {}) if isinstance(cell_def.get("task"), dict) else {}

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
        f"- Validate scene contract: `python3 scripts/validate_scene_contract.py {package_name}`",
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
    task_recipe = _normalize_task_recipe(scene_generator.build_task_recipe(loaded), warnings)
    scene_manifest["task_recipe"] = task_recipe

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
    (package_dir / "urdf").mkdir(parents=True, exist_ok=True)
    (package_dir / "generated").mkdir(parents=True, exist_ok=True)

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

    (package_dir / "README.md").write_text(
        _header_markdown(cell_definition_path)
        + _build_readme(loaded, package_name, cell_definition_path, warnings),
        encoding="utf-8",
    )

    commissioning_summary = scene_generator.build_commissioning_summary(loaded, warnings)
    (package_dir / "generated" / "commissioning_summary.md").write_text(
        _header_markdown(cell_definition_path) + commissioning_summary,
        encoding="utf-8",
    )

    (package_dir / "launch" / "README.md").write_text(
        _header_markdown(cell_definition_path)
        + "# Launch placeholders\n\nGenerated package placeholder. Reuse validated scene launch assets after review.\n",
        encoding="utf-8",
    )
    (package_dir / "urdf" / "README.md").write_text(
        _header_markdown(cell_definition_path)
        + "# URDF placeholders\n\nReview and connect approved robot/environment geometry assets manually.\n",
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
                warnings.append(f"Execution plan generation status: {plan_result.status}")
        finally:
            plan_generator.OUTPUT_DIR = original_plan_dir

    _write_validation_report(package_dir / "generated" / "validation_report.md", scene_manifest, scene_contract, dry_result)

    commissioning = loaded.get("commissioning", {}) if isinstance(loaded.get("commissioning"), dict) else {}
    if bool(commissioning.get("export_bundle", False)):
        _run_optional_bundle_export(bundle_exporter, package_name, scene_manifest_path, package_dir / "generated", warnings)

    status = "PASS" if not warnings else "WARN"
    print(f"{status}: generated package at {package_dir}")
    print(f"PASS: package.xml -> {package_dir / 'package.xml'}")
    print(f"PASS: scene_manifest.yaml -> {scene_manifest_path}")
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
