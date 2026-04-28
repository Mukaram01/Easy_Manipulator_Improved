#!/usr/bin/env python3
"""Create a complete offline workcell project from a cell definition YAML or template."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import shutil
import sys
import tempfile
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "dist" / "workcell_projects"

VALIDATE_CELL_DEF_PATH = SCRIPTS_DIR / "validate_cell_definition.py"
WORKCELL_GEN_PATH = SCRIPTS_DIR / "generate_workcell_from_cell_definition.py"
VALIDATE_SCENE_PATH = SCRIPTS_DIR / "validate_scene_contract.py"
DRY_RUN_PATH = SCRIPTS_DIR / "dry_run_task_recipe.py"
PLAN_PATH = SCRIPTS_DIR / "generate_task_execution_plan.py"
BUNDLE_PATH = SCRIPTS_DIR / "export_workcell_bundle.py"
WIZARD_PATH = SCRIPTS_DIR / "create_cell_definition_wizard.py"
TEMPLATE_TOOL_PATH = SCRIPTS_DIR / "create_cell_from_template.py"
DASHBOARD_PATH = SCRIPTS_DIR / "generate_workcell_dashboard.py"

REQUIRED_GENERATED_FILES = (
    "package.xml",
    "CMakeLists.txt",
    "scene_manifest.yaml",
    "workcell.yaml",
    "generated/scene_manifest.preview.yaml",
    "generated/task_recipe.preview.yaml",
    "generated/commissioning_summary.md",
)


@dataclass
class StepStatus:
    status: str
    notes: list[str]


def _load_module(module_name: str, module_path: Path):
    if not module_path.is_file():
        raise FileNotFoundError(f"Missing required tool: {module_path}")
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if not spec or not spec.loader:
        raise RuntimeError(f"Unable to load module from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _sanitize_id(value: str) -> str:
    lowered = value.strip().lower().replace(" ", "_")
    cleaned = "".join(ch if (ch.isalnum() or ch == "_") else "_" for ch in lowered)
    while "__" in cleaned:
        cleaned = cleaned.replace("__", "_")
    return cleaned.strip("_") or "generated_cell"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            chunk = handle.read(65536)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _status_from_validator_result(result: Any, exit_code: int) -> StepStatus:
    if exit_code != 0 or not result.ok:
        return StepStatus("FAIL", list(result.errors) + list(result.warnings) + list(result.notes))
    if result.has_warnings:
        return StepStatus("WARN", list(result.warnings) + list(result.notes))
    return StepStatus("PASS", list(result.notes))


def _write_text(path: Path, text: str, dry_run: bool, planned: list[str]) -> None:
    planned.append(str(path))
    if dry_run:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _render_validation_summary(statuses: dict[str, StepStatus], warnings: list[str], errors: list[str]) -> str:
    lines = [
        "# Workcell Project Validation Summary",
        "",
        "## Step status",
        f"- Cell definition validation: **{statuses['cell_definition_validation'].status}**",
        f"- Workcell generation: **{statuses['workcell_generation'].status}**",
        f"- Scene contract validation (direct file): **{statuses['scene_manifest_validation'].status}**",
        f"- Task recipe dry-run: **{statuses['task_recipe_dry_run'].status}**",
        f"- Task execution plan: **{statuses['task_execution_plan'].status}**",
        f"- Commissioning bundle: **{statuses['commissioning_bundle'].status}**",
        "",
        "## Warnings",
    ]
    lines.extend([f"- {item}" for item in warnings] or ["- (none)"])
    lines.extend(["", "## Errors"])
    lines.extend([f"- {item}" for item in errors] or ["- (none)"])
    return "\n".join(lines) + "\n"


def _render_project_readme(
    cell_def: dict[str, Any],
    project_dir: Path,
    package_name: str,
    source_type: str,
    source_ref: str,
) -> str:
    cell = cell_def.get("cell", {}) if isinstance(cell_def.get("cell"), dict) else {}
    robot = cell_def.get("robot", {}) if isinstance(cell_def.get("robot"), dict) else {}
    end_effector = cell_def.get("end_effector", {}) if isinstance(cell_def.get("end_effector"), dict) else {}
    camera = cell_def.get("camera", {}) if isinstance(cell_def.get("camera"), dict) else {}
    task = cell_def.get("task", {}) if isinstance(cell_def.get("task"), dict) else {}

    capability_refs = _extract_capability_refs(cell_def)

    return (
        f"# Workcell Project: {cell.get('name', '(unknown)')}\n\n"
        "This folder is a one-command, offline-generated review package for a robotic workcell.\n\n"
        "## Project identity\n"
        f"- Cell id: `{cell.get('id', '(unknown)')}`\n"
        f"- Cell name: `{cell.get('name', '(unknown)')}`\n"
        f"- Source type: `{source_type}` (`{source_ref}`)\n"
        f"- Generated ROS package: `{package_name}`\n"
        f"- Task type: `{task.get('type', '(unknown)')}`\n"
        f"- Robot: `{robot.get('model', '(unknown)')}`\n"
        f"- End effector: `{end_effector.get('id', '(unknown)')}`\n"
        f"- Camera: `{camera.get('id', '(unknown)')}`\n"
        f"- Capabilities: `{capability_refs if capability_refs else 'none'}`\n\n"
        "## Safety boundary\n"
        "Offline review only. This project is not a certified production runtime or safety approval.\n\n"
        "## Inspect generated files\n"
        f"- Project manifest JSON: `{project_dir / 'project_manifest.json'}`\n"
        f"- Generated package: `{project_dir / 'generated_workcell' / package_name}`\n"
        f"- Validation summary: `{project_dir / 'reports' / 'validation_summary.md'}`\n"
        f"- Dry-run summary: `{project_dir / 'reports' / 'task_recipe_dry_run.md'}`\n"
        f"- Execution plan: `{project_dir / 'reports' / 'task_execution_plan.md'}`\n"
        f"- Commissioning bundle: `{project_dir / 'commissioning_bundle'}`\n\n"
        "## Validate again (offline)\n"
        f"python3 scripts/validate_cell_definition.py {project_dir / 'cell_definition.yaml'}\n"
        f"python3 scripts/validate_scene_contract.py {project_dir / 'generated_workcell' / package_name / 'scene_manifest.yaml'}\n"
        "python3 scripts/dry_run_task_recipe.py --check\n"
        "python3 scripts/generate_task_execution_plan.py --check\n\n"
        "## Copy into a ROS 2 workspace\n"
        f"cp -r {project_dir / 'generated_workcell' / package_name} ~/workcell_ws/src/\n\n"
        "## Build and preflight\n"
        "cd ~/workcell_ws\n"
        f"colcon build --packages-select {package_name}\n"
        "source install/setup.bash\n"
        "./src/easy_manipulation_deployment/scripts/preflight_workcell.sh\n\n"
        "## Launch example after build\n"
        f"ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:={package_name}\n\n"
        "## Commissioning review still required\n"
        "- Frame consistency and calibration\n"
        "- Collision/safety boundaries and speed limits\n"
        "- End-effector payload/tooling validation\n"
        "- Operator sign-off and production acceptance\n"
    )


def _render_next_commands(project_dir: Path, package_name: str) -> str:
    pkg_path = project_dir / "generated_workcell" / package_name
    lines = [
        "# Next Commands",
        "",
        "```bash",
        f"python3 scripts/validate_cell_definition.py {project_dir / 'cell_definition.yaml'}",
        f"python3 scripts/validate_scene_contract.py {pkg_path / 'scene_manifest.yaml'}",
        f"python3 scripts/export_workcell_bundle.py {package_name} --force",
        "python3 scripts/dry_run_task_recipe.py --check",
        "python3 scripts/generate_task_execution_plan.py --check",
        f"cp -r {pkg_path} ~/workcell_ws/src/",
        "cd ~/workcell_ws",
        f"colcon build --packages-select {package_name}",
        "source install/setup.bash",
        "./src/easy_manipulation_deployment/scripts/preflight_workcell.sh",
        f"ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:={package_name}",
        "```",
        "",
    ]
    return "\n".join(lines)




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


def _extract_environment_layout(cell_def: dict[str, Any]) -> dict[str, Any]:
    environment = cell_def.get("environment", {}) if isinstance(cell_def.get("environment"), dict) else {}
    layout = environment.get("layout")
    if isinstance(layout, str) and layout.strip():
        return {"path": layout, "metadata_only": True}
    return {}
def _create_from_template(args: argparse.Namespace, cell_yaml_path: Path) -> tuple[int, list[str]]:
    notes: list[str] = []
    command: list[str]
    if TEMPLATE_TOOL_PATH.is_file():
        command = [
            sys.executable,
            str(TEMPLATE_TOOL_PATH),
            "--template",
            args.template,
            "--cell-name",
            args.cell_name,
            "--cell-id",
            args.cell_id,
            "--robot",
            args.robot,
            "--end-effector",
            args.end_effector,
            "--camera",
            args.camera,
            "--output",
            str(cell_yaml_path),
            "--force",
        ]
    elif WIZARD_PATH.is_file():
        notes.append("scripts/create_cell_from_template.py not found; using create_cell_definition_wizard.py template mode.")
        command = [
            sys.executable,
            str(WIZARD_PATH),
            "--template",
            args.template,
            "--cell-name",
            args.cell_name,
            "--cell-id",
            args.cell_id,
            "--robot",
            args.robot,
            "--end-effector",
            args.end_effector,
            "--camera",
            args.camera,
            "--output",
            str(cell_yaml_path),
            "--force",
        ]
    else:
        return 1, ["Template generation tool not found (expected scripts/create_cell_from_template.py or wizard)."]

    import subprocess

    proc = subprocess.run(command, capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        notes.append(proc.stdout.strip())
        notes.append(proc.stderr.strip())
    return proc.returncode, [item for item in notes if item]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cell-definition", type=Path, help="Path to an existing cell_definition/v1 YAML")
    parser.add_argument("--template", type=str, help="Template id to generate a cell definition")
    parser.add_argument("--cell-name", type=str, help="Template mode: human-readable cell name")
    parser.add_argument("--cell-id", type=str, help="Template mode: cell id")
    parser.add_argument("--robot", type=str, help="Template mode: robot preset")
    parser.add_argument("--end-effector", dest="end_effector", type=str, help="Template mode: end-effector preset")
    parser.add_argument("--camera", type=str, help="Template mode: camera preset")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_ROOT, help="Root output directory")
    parser.add_argument("--package-name", type=str, help="Override generated ROS package name")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--skip-bundle", action="store_true")
    parser.add_argument("--skip-execution-plan", action="store_true")
    parser.add_argument("--skip-dashboard", action="store_true", help="Skip generation of static dashboard")
    parser.add_argument("--dashboard-output", type=Path, help="Optional output HTML path for generated dashboard")
    parser.add_argument("--json", action="store_true", help="Print final summary as JSON")
    parser.add_argument("--print-next-commands", action="store_true")
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--keep-temp", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()

    if bool(args.cell_definition) == bool(args.template):
        print("FAIL: provide exactly one of --cell-definition or --template")
        return 1

    if args.template:
        required = ["cell_name", "cell_id", "robot", "end_effector", "camera"]
        missing = [name for name in required if not getattr(args, name)]
        if missing:
            print(f"FAIL: template mode missing required options: {', '.join(missing)}")
            return 1

    try:
        cell_validator = _load_module("project_validate_cell", VALIDATE_CELL_DEF_PATH)
        workcell_generator = _load_module("project_generate_workcell", WORKCELL_GEN_PATH)
        scene_validator = _load_module("project_validate_scene", VALIDATE_SCENE_PATH)
        dry_runner = _load_module("project_dry_run", DRY_RUN_PATH)
        plan_generator = _load_module("project_plan", PLAN_PATH)
        bundle_exporter = _load_module("project_bundle", BUNDLE_PATH)
        dashboard_generator = _load_module("project_dashboard", DASHBOARD_PATH)
    except Exception as exc:
        print(f"FAIL: unable to load offline tooling: {exc}")
        return 2

    source_type = "template" if args.template else "cell_definition"
    source_ref = args.template if args.template else str(args.cell_definition)

    temp_dir: Path | None = None
    created_temp = False
    temp_notes: list[str] = []

    if args.template:
        if args.keep_temp:
            temp_dir = (args.output_dir.resolve() / "_tmp").resolve()
            if not args.dry_run:
                temp_dir.mkdir(parents=True, exist_ok=True)
        else:
            temp_dir = Path(tempfile.mkdtemp(prefix="workcell_project_cell_"))
            created_temp = True
        cell_definition_path = (temp_dir / f"{_sanitize_id(args.cell_id)}.cell_definition.yaml").resolve()
        if args.dry_run:
            temp_notes.append(f"Would generate cell definition from template '{args.template}' to {cell_definition_path}")
        else:
            rc_template, template_notes = _create_from_template(args, cell_definition_path)
            temp_notes.extend(template_notes)
            if rc_template != 0 or not cell_definition_path.is_file():
                print("FAIL: template generation failed")
                for note in temp_notes:
                    if note:
                        print(f" - {note}")
                return 1
    else:
        cell_definition_path = args.cell_definition.resolve()
        if not cell_definition_path.is_file():
            print(f"FAIL: missing cell definition file: {cell_definition_path}")
            return 1

    loaded: dict[str, Any] = {}
    parser_name = "(n/a)"
    parser_notes: list[str] = []
    cell_validation = StepStatus("FAIL", ["Cell definition not validated"])

    if not args.dry_run:
        try:
            loaded, parser_name, parser_notes = cell_validator.load_yaml(cell_definition_path)
            summary = cell_validator.validate_cell_definition(loaded, cell_definition_path, parser_name, parser_notes)
            if not summary.ok:
                print("FAIL: invalid cell definition")
                for error in summary.errors:
                    print(f" - {error}")
                return 1
            cell_validation = StepStatus("WARN" if summary.warnings else "PASS", list(summary.warnings) + list(summary.notes))
        except Exception as exc:
            print(f"FAIL: invalid cell definition: {exc}")
            return 1
    else:
        cell_validation = StepStatus("PASS", ["Dry-run mode: validation and generation steps not executed."])

    cell_id = _sanitize_id(args.cell_id if args.template else str(loaded.get("cell", {}).get("id", "")) or cell_definition_path.stem)
    package_name = args.package_name.strip() if args.package_name else f"generated_{cell_id}"

    project_dir = args.output_dir.resolve() / cell_id
    generated_root = project_dir / "generated_workcell"
    generated_pkg_dir = generated_root / package_name
    reports_dir = project_dir / "reports"
    bundle_dir = project_dir / "commissioning_bundle"

    if project_dir.exists() and not args.force:
        print(f"FAIL: project already exists: {project_dir} (use --force to overwrite)")
        return 1

    planned_paths: list[str] = [
        str(project_dir / "README.md"),
        str(project_dir / "project_manifest.json"),
        str(project_dir / "cell_definition.yaml"),
        str(project_dir / "next_commands.md"),
        str(generated_pkg_dir),
        str(reports_dir / "validation_summary.md"),
        str(reports_dir / "task_recipe_dry_run.md"),
        str(reports_dir / "task_execution_plan.md"),
        str(reports_dir / "task_execution_plan.json"),
        str(bundle_dir),
        str(project_dir / "dashboard" / "index.html"),
    ]

    if args.dry_run:
        print("PASS: dry-run; no files written.")
        print("Would create:")
        for item in planned_paths:
            print(f" - {item}")
        return 0

    if project_dir.exists():
        shutil.rmtree(project_dir)
    reports_dir.mkdir(parents=True, exist_ok=True)
    generated_root.mkdir(parents=True, exist_ok=True)

    shutil.copy2(cell_definition_path, project_dir / "cell_definition.yaml")

    workcell_rc = workcell_generator.generate_package(
        cell_definition_path=cell_definition_path,
        output_dir=generated_root,
        package_name=package_name,
        force=args.force,
        dry_run=False,
    )
    workcell_generation = StepStatus("PASS", [])
    if workcell_rc != 0:
        workcell_generation = StepStatus("FAIL", ["generate_workcell_from_cell_definition returned non-zero"]) 
    else:
        missing_required = [name for name in REQUIRED_GENERATED_FILES if not (generated_pkg_dir / name).is_file()]
        if missing_required:
            workcell_generation = StepStatus("FAIL", [f"Missing generated package files: {', '.join(missing_required)}"])

    scene_manifest_path = generated_pkg_dir / "scene_manifest.yaml"
    scene_result, scene_exit = scene_validator.validate_scene_manifest_path(str(scene_manifest_path), package_label=package_name)
    scene_status = _status_from_validator_result(scene_result, scene_exit)

    dry_row = dry_runner.evaluate_scene(package_name, scene_manifest_path)
    dry_status = StepStatus(dry_row.status, list(dry_row.notes))

    task_recipe_report = "\n".join(
        [
            "# Task Recipe Dry-Run",
            "",
            f"- Scene: `{dry_row.scene}`",
            f"- Status: **{dry_row.status}**",
            f"- Recipe id: `{dry_row.recipe_id}`",
            f"- Task type: `{dry_row.task_type}`",
            f"- Object id: `{dry_row.object_id}`",
            f"- Matched rule: `{dry_row.matched_rule_id}`",
            f"- Destination: `{dry_row.selected_destination_id}`",
            f"- Action: `{dry_row.selected_action}`",
            "",
            "## Notes",
            *([f"- {note}" for note in dry_row.notes] or ["- (none)"]),
            "",
        ]
    )
    _write_text(reports_dir / "task_recipe_dry_run.md", task_recipe_report, False, planned_paths)

    if args.skip_execution_plan:
        plan_status = StepStatus("SKIP", ["Skipped by --skip-execution-plan"])
    else:
        original_output = plan_generator.OUTPUT_DIR
        plan_generator.OUTPUT_DIR = reports_dir
        try:
            plan_row = plan_generator.evaluate_scene(package_name, scene_manifest_path)
        finally:
            plan_generator.OUTPUT_DIR = original_output
        plan_status = StepStatus(plan_row.status, list(plan_row.notes))
        if plan_row.status == "PASS" and plan_row.markdown_path and plan_row.json_path:
            shutil.move(str(plan_row.markdown_path), str(reports_dir / "task_execution_plan.md"))
            shutil.move(str(plan_row.json_path), str(reports_dir / "task_execution_plan.json"))
        else:
            _write_text(
                reports_dir / "task_execution_plan.md",
                "# Task Execution Plan\n\nNo execution plan generated.\n",
                False,
                planned_paths,
            )
            _write_text(
                reports_dir / "task_execution_plan.json",
                json.dumps({"status": plan_row.status, "notes": list(plan_row.notes)}, indent=2) + "\n",
                False,
                planned_paths,
            )

    if args.skip_bundle:
        bundle_status = StepStatus("SKIP", ["Skipped by --skip-bundle"])
    else:
        try:
            exported_bundle_dir, _ = bundle_exporter.export_scene(
                package_name,
                scene_manifest_path,
                output_dir=project_dir,
                zip_output=False,
                force=True,
            )
            if bundle_dir.exists():
                shutil.rmtree(bundle_dir)
            shutil.move(str(exported_bundle_dir), str(bundle_dir))
            checksum_path = bundle_dir / "checksums.sha256"
            checksum_lines = []
            for file_path in sorted(path for path in bundle_dir.rglob("*") if path.is_file() and path.name != "checksums.sha256"):
                checksum_lines.append(f"{_sha256(file_path)}  {file_path.relative_to(bundle_dir).as_posix()}")
            checksum_path.write_text("\n".join(checksum_lines) + "\n", encoding="utf-8")
            bundle_status = StepStatus("PASS", [])
        except Exception as exc:
            bundle_status = StepStatus("WARN", [f"Bundle export warning: {exc}"])

    statuses = {
        "cell_definition_validation": cell_validation,
        "workcell_generation": workcell_generation,
        "scene_manifest_validation": scene_status,
        "task_recipe_dry_run": dry_status,
        "task_execution_plan": plan_status,
        "commissioning_bundle": bundle_status,
    }

    next_commands_text = _render_next_commands(project_dir, package_name)
    _write_text(project_dir / "next_commands.md", next_commands_text, False, planned_paths)

    readme_text = _render_project_readme(loaded, project_dir, package_name, source_type, source_ref)
    _write_text(project_dir / "README.md", readme_text, False, planned_paths)

    important_paths = {
        "cell_definition": "cell_definition.yaml",
        "generated_package": f"generated_workcell/{package_name}",
        "scene_manifest": f"generated_workcell/{package_name}/scene_manifest.yaml",
        "workcell_yaml": f"generated_workcell/{package_name}/workcell.yaml",
        "scene_preview": f"generated_workcell/{package_name}/generated/scene_manifest.preview.yaml",
        "task_preview": f"generated_workcell/{package_name}/generated/task_recipe.preview.yaml",
        "commissioning_summary": f"generated_workcell/{package_name}/generated/commissioning_summary.md",
        "validation_summary": "reports/validation_summary.md",
        "dry_run_report": "reports/task_recipe_dry_run.md",
        "execution_plan_md": "reports/task_execution_plan.md",
        "execution_plan_json": "reports/task_execution_plan.json",
        "commissioning_bundle": "commissioning_bundle",
        "next_commands": "next_commands.md",
    }

    dashboard_output = args.dashboard_output.resolve() if args.dashboard_output else (project_dir / "dashboard" / "index.html")
    if args.skip_dashboard:
        dashboard_status = StepStatus("SKIP", ["Skipped by --skip-dashboard"])
    else:
        try:
            rc = dashboard_generator.main(
                [
                    "--project-dir",
                    str(project_dir),
                    "--manifest",
                    str(project_dir / "project_manifest.json"),
                    "--output",
                    str(dashboard_output),
                    "--quiet",
                ]
            )
            if rc == 0 and dashboard_output.is_file():
                dashboard_status = StepStatus("PASS", [])
                try:
                    important_paths["dashboard"] = dashboard_output.relative_to(project_dir).as_posix()
                except ValueError:
                    important_paths["dashboard"] = dashboard_output.as_posix()
            else:
                dashboard_status = StepStatus("WARN", [f"Dashboard generation returned non-zero: {rc}"])
        except Exception as exc:
            dashboard_status = StepStatus("WARN", [f"Dashboard generation warning: {exc}"])

    statuses["dashboard_generation"] = dashboard_status

    warnings: list[str] = []
    errors: list[str] = []
    for step_name, step in statuses.items():
        for note in step.notes:
            if step.status in {"FAIL"}:
                errors.append(f"{step_name}: {note}")
            elif step.status in {"WARN", "SKIP"}:
                warnings.append(f"{step_name}: {note}")

    checksums: dict[str, str] = {}
    for key, rel_path in important_paths.items():
        abs_path = project_dir / rel_path
        if abs_path.is_file():
            checksums[key] = _sha256(abs_path)

    manifest_payload = {
        "schema_version": "workcell_project/v1",
        "cell_id": cell_id,
        "cell_name": loaded.get("cell", {}).get("name", cell_id),
        "source_type": source_type,
        ("source_path" if source_type == "cell_definition" else "template_id"): source_ref,
        "generated_package_name": package_name,
        "generated_at_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "tool_versions": [
            "validate_cell_definition.py",
            "generate_workcell_from_cell_definition.py",
            "validate_scene_contract.py",
            "dry_run_task_recipe.py",
            "generate_task_execution_plan.py",
            "export_workcell_bundle.py",
            "create_cell_definition_wizard.py",
            "generate_workcell_dashboard.py",
        ],
        "statuses": {key: value.status for key, value in statuses.items()},
        "artifacts": important_paths,
        "warnings": warnings + temp_notes,
        "errors": errors,
        "checksums": checksums,
        "parser": parser_name,
        "parser_notes": parser_notes,
        "capabilities": _extract_capability_refs(loaded),
        "capability_validation": getattr(summary, "capability_summary", {}),
        "environment_layout": getattr(summary, "environment_layout_summary", {}) or _extract_environment_layout(loaded),
    }
    _write_text(project_dir / "project_manifest.json", json.dumps(manifest_payload, indent=2, sort_keys=True) + "\n", False, planned_paths)

    if not args.skip_dashboard and dashboard_status.status != "PASS":
        try:
            rc = dashboard_generator.main(
                [
                    "--project-dir",
                    str(project_dir),
                    "--manifest",
                    str(project_dir / "project_manifest.json"),
                    "--output",
                    str(dashboard_output),
                    "--quiet",
                ]
            )
            if rc == 0 and dashboard_output.is_file():
                statuses["dashboard_generation"] = StepStatus("PASS", [])
                warnings = [w for w in warnings if not w.startswith("dashboard_generation:")]
                try:
                    important_paths["dashboard"] = dashboard_output.relative_to(project_dir).as_posix()
                except ValueError:
                    important_paths["dashboard"] = dashboard_output.as_posix()
                checksums["dashboard"] = _sha256(dashboard_output)
                manifest_payload["statuses"] = {key: value.status for key, value in statuses.items()}
                manifest_payload["artifacts"] = important_paths
                manifest_payload["warnings"] = warnings + temp_notes
                manifest_payload["checksums"] = checksums
                _write_text(project_dir / "project_manifest.json", json.dumps(manifest_payload, indent=2, sort_keys=True) + "\n", False, planned_paths)
        except Exception:
            pass

    _write_text(reports_dir / "validation_summary.md", _render_validation_summary(statuses, warnings, errors), False, planned_paths)

    blocking_failed = any(
        statuses[key].status == "FAIL"
        for key in ("cell_definition_validation", "workcell_generation", "scene_manifest_validation")
    )
    if not args.skip_execution_plan and statuses["task_execution_plan"].status == "FAIL":
        blocking_failed = True
    if not args.skip_bundle and statuses["commissioning_bundle"].status == "FAIL":
        blocking_failed = True
    if not args.skip_dashboard and statuses["dashboard_generation"].status == "FAIL":
        blocking_failed = True

    strict_failed = args.strict and any(step.status in {"WARN", "SKIP"} for step in statuses.values())

    summary = {
        "project_path": str(project_dir),
        "generated_package_path": str(generated_pkg_dir),
        "validation_status": statuses["scene_manifest_validation"].status,
        "dry_run_status": statuses["task_recipe_dry_run"].status,
        "execution_plan_status": statuses["task_execution_plan"].status,
        "bundle_status": statuses["commissioning_bundle"].status,
        "dashboard_status": statuses["dashboard_generation"].status,
        "next_command": f"cat {project_dir / 'next_commands.md'}",
        "warnings": warnings + temp_notes,
        "errors": errors,
    }

    if not args.quiet:
        print("=== Workcell Project Summary ===")
        for key in (
            "project_path",
            "generated_package_path",
            "validation_status",
            "dry_run_status",
            "execution_plan_status",
            "bundle_status",
            "dashboard_status",
            "next_command",
        ):
            print(f"{key}: {summary[key]}")
        overall = "FAIL" if (blocking_failed or strict_failed) else ("WARN" if (warnings or temp_notes) else "PASS")
        print(f"overall: {overall}")

    if args.print_next_commands:
        print(next_commands_text)

    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))

    if created_temp and not args.keep_temp and temp_dir and temp_dir.exists():
        shutil.rmtree(temp_dir, ignore_errors=True)

    return 1 if (blocking_failed or strict_failed) else 0


if __name__ == "__main__":
    raise SystemExit(main())
