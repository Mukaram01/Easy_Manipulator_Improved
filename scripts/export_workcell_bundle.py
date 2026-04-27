#!/usr/bin/env python3
"""Generate per-scene offline commissioning bundles for handover and archival."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import shutil
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from zipfile import ZIP_DEFLATED, ZipFile

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = REPO_ROOT / "dist" / "workcell_bundles"
REPORTS_DIR = REPO_ROOT / "docs" / "manuals"
PLAN_OUTPUT_DIR = REPORTS_DIR / "generated_execution_plans"
DRY_RUN_PATH = REPO_ROOT / "scripts" / "dry_run_task_recipe.py"
VALIDATOR_PATH = REPO_ROOT / "scripts" / "validate_scene_contract.py"
SELF_TEST_REPORTER_PATH = REPO_ROOT / "scripts" / "generate_scene_self_test_report.py"
TASK_RECIPE_REPORTER_PATH = REPO_ROOT / "scripts" / "generate_task_recipe_report.py"
PLAN_GENERATOR_PATH = REPO_ROOT / "scripts" / "generate_task_execution_plan.py"

OPTIONAL_REPORTS = (
    "latest_scene_validation_report.md",
    "latest_scene_self_test_report.md",
    "latest_task_recipe_report.md",
    "latest_task_recipe_dry_run_report.md",
    "latest_task_execution_plan_report.md",
)

REQUIRED_FILES = (
    "README.md",
    "bundle_manifest.json",
    "operator_checklist.md",
    "scene_manifest.yaml",
    "execution_plan.md",
    "execution_plan.json",
    "validation_summary.md",
)


@dataclass
class SceneStatus:
    scene: str
    scene_validation_status: str
    self_test_status: str
    task_recipe_status: str
    dry_run_status: str
    execution_plan_status: str
    warnings: list[str]


def _load_module(module_name: str, module_path: Path):
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if not spec or not spec.loader:
        raise RuntimeError(f"Unable to load module from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


dry_run = _load_module("bundle_dry_run_task_recipe", DRY_RUN_PATH)
validator = _load_module("bundle_validate_scene_contract", VALIDATOR_PATH)
self_test_reporter = _load_module("bundle_generate_scene_self_test_report", SELF_TEST_REPORTER_PATH)
task_recipe_reporter = _load_module("bundle_generate_task_recipe_report", TASK_RECIPE_REPORTER_PATH)
plan_generator = _load_module("bundle_generate_task_execution_plan", PLAN_GENERATOR_PATH)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            chunk = handle.read(65536)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _read_manifest(path: Path) -> tuple[dict[str, Any], list[str]]:
    manifest, _, parser_notes = validator._read_manifest(str(path))
    return manifest, list(parser_notes)


def _normalize_ee_type(end_effector: dict[str, Any]) -> str:
    ee_type = str(end_effector.get("type", "unknown")).strip().lower()
    if ee_type in {"finger", "suction"}:
        return ee_type
    return "unknown"


def _status_from_scene(scene: str, manifest_path: Path) -> SceneStatus:
    warnings: list[str] = []
    manifest, parser_notes = _read_manifest(manifest_path)
    warnings.extend(parser_notes)

    scene_validation_status = "PASS"
    _, task_recipe_notes = validator.validate_task_recipe_block(manifest)
    task_row = task_recipe_reporter.evaluate_scene(scene, manifest_path)
    self_test_row = self_test_reporter.evaluate_scene(scene, manifest_path)
    dry_row = dry_run.evaluate_scene(scene, manifest_path)

    if task_row.status == "FAIL" or self_test_row.status == "FAIL" or dry_row.status == "FAIL":
        scene_validation_status = "FAIL"
    elif task_row.status == "WARN" or self_test_row.status == "WARN" or dry_row.status in {"WARN", "SKIP"}:
        scene_validation_status = "WARN"

    plan_row = plan_generator.evaluate_scene(scene, manifest_path)
    if plan_row.status == "PASS":
        execution_plan_status = "PASS"
    elif dry_row.status == "PASS":
        execution_plan_status = "FAIL"
        warnings.append("Dry-run is PASS but execution plan generation did not produce PASS.")
    else:
        execution_plan_status = plan_row.status

    for note in task_recipe_notes:
        if note not in warnings:
            warnings.append(note)
    for note in dry_row.notes + plan_row.notes:
        if note not in warnings:
            warnings.append(note)

    return SceneStatus(
        scene=scene,
        scene_validation_status=scene_validation_status,
        self_test_status=self_test_row.status,
        task_recipe_status=task_row.status,
        dry_run_status=dry_row.status,
        execution_plan_status=execution_plan_status,
        warnings=warnings,
    )


def _write_readme(bundle_dir: Path, scene: str) -> None:
    text = f"""# Workcell Commissioning Bundle: {scene}

This package is a per-scene commissioning export bundle for offline review, handover, and archival.

## What this bundle is
- A dependency-light handover artifact for scene `{scene}`.
- A collection of manifest metadata, offline validation summaries, task recipe dry-run output, execution plan artifacts, and a practical operator checklist.

## What this bundle proves
- Required commissioning artifacts were generated for this scene.
- Offline contract checks and metadata validation outputs are bundled in one place.
- A deterministic task execution plan artifact is present for review.

## What this bundle does not prove
- It does **not** prove runtime collision-free behavior.
- It does **not** prove real robot reachability in your physical cell.
- It does **not** certify machine safety compliance.

**This bundle is a commissioning aid, not a safety certificate.**

## Quick operator review order
1. Open `validation_summary.md`
2. Open `task_recipe_dry_run_summary.md`
3. Open `execution_plan.md`
4. Open `operator_checklist.md`

## Offline-only note
This bundle is generated from offline scene metadata and reports. No runtime robot command is executed by this export action.
"""
    (bundle_dir / "README.md").write_text(text, encoding="utf-8")


def _write_checklist(bundle_dir: Path) -> None:
    text = """# Operator / Commissioning Checklist

## Offline checks
- [ ] Scene manifest reviewed
- [ ] Robot base frame checked
- [ ] End-effector link checked
- [ ] Grasp frame checked
- [ ] Self-test object pose reviewed
- [ ] Task recipe decision reviewed
- [ ] Destination pose reviewed
- [ ] Execution plan reviewed

## Simulation checks
- [ ] Headless launch smoke test passed
- [ ] RViz scene visually inspected
- [ ] Object spawn pose visually inspected
- [ ] Grasp approach visually inspected
- [ ] Release destination visually inspected
- [ ] Return-home path visually inspected

## Physical cell checks
- [ ] Emergency stop tested
- [ ] Guarding/interlocks checked
- [ ] Robot speed limits set
- [ ] Payload/tool settings checked
- [ ] TCP checked
- [ ] Work object/base frame checked
- [ ] Dry cycle completed without payload
- [ ] Dry cycle completed with payload
- [ ] Operator sign-off completed

This checklist is a commissioning aid and does not grant any safety approval.
"""
    (bundle_dir / "operator_checklist.md").write_text(text, encoding="utf-8")


def _write_dry_run_summary(bundle_dir: Path, dry_row: Any) -> None:
    attrs = (", ".join(f"{k}={dry_row.object_attributes[k]}" for k in sorted(dry_row.object_attributes))
             if dry_row.object_attributes else "(n/a)")
    lines = [
        "# Task Recipe Dry-Run Summary",
        "",
        f"- Scene: `{dry_row.scene}`",
        f"- Object id: `{dry_row.object_id}`",
        f"- Object attributes: `{attrs}`",
        f"- Recipe id: `{dry_row.recipe_id}`",
        f"- Task type: `{dry_row.task_type}`",
        f"- Matched rule id: `{dry_row.matched_rule_id}`",
        f"- Destination id: `{dry_row.selected_destination_id}`",
        f"- Destination action: `{dry_row.selected_action}`",
        f"- Status: **{dry_row.status}**",
        "",
        "## Notes",
        "",
    ]
    for note in dry_row.notes:
        lines.append(f"- {note}")
    if dry_row.status != "PASS":
        lines.extend(["", "No runtime action is implied by a non-PASS dry-run result."])
    (bundle_dir / "task_recipe_dry_run_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_validation_summary(bundle_dir: Path, status: SceneStatus, generated_files: list[str]) -> None:
    lines = [
        "# Validation Summary",
        "",
        f"- Scene contract validation status: **{status.scene_validation_status}**",
        f"- Self-test status: **{status.self_test_status}**",
        f"- Task recipe validation status: **{status.task_recipe_status}**",
        f"- Dry-run status: **{status.dry_run_status}**",
        f"- Execution plan generation status: **{status.execution_plan_status}**",
        "",
        "## Generated bundle files",
        "",
    ]
    for item in sorted(generated_files):
        lines.append(f"- `{item}`")
    lines.extend(["", "## Warnings", ""])
    if status.warnings:
        for warning in status.warnings:
            lines.append(f"- {warning}")
    else:
        lines.append("- (none)")

    (bundle_dir / "validation_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_placeholder_plan(scene: str, bundle_dir: Path, plan_row: Any) -> None:
    md_text = (
        f"# Offline Task Execution Plan: {scene}\n\n"
        f"No executable plan was generated for this scene because status is `{plan_row.status}`.\n\n"
        "This file is intentionally provided for commissioning package consistency.\n"
    )
    (bundle_dir / "execution_plan.md").write_text(md_text, encoding="utf-8")
    payload = {
        "scene": scene,
        "status": plan_row.status,
        "notes": list(plan_row.notes),
        "offline_only": True,
    }
    (bundle_dir / "execution_plan.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _copy_or_generate_execution_plans(scene: str, manifest_path: Path, bundle_dir: Path, warnings: list[str]) -> str:
    md_src = PLAN_OUTPUT_DIR / f"{scene}_execution_plan.md"
    json_src = PLAN_OUTPUT_DIR / f"{scene}_execution_plan.json"

    plan_row = plan_generator.evaluate_scene(scene, manifest_path)

    if md_src.is_file() and json_src.is_file():
        shutil.copy2(md_src, bundle_dir / "execution_plan.md")
        shutil.copy2(json_src, bundle_dir / "execution_plan.json")
        return plan_row.status

    if plan_row.markdown_path and plan_row.json_path and plan_row.markdown_path.is_file() and plan_row.json_path.is_file():
        shutil.copy2(plan_row.markdown_path, bundle_dir / "execution_plan.md")
        shutil.copy2(plan_row.json_path, bundle_dir / "execution_plan.json")
        return plan_row.status

    if plan_row.status == "PASS":
        warnings.append("Execution plan expected for PASS dry-run but plan files were not generated.")
        return "FAIL"

    warnings.append(f"Execution plan not generated; writing placeholder files (status={plan_row.status}).")
    _write_placeholder_plan(scene, bundle_dir, plan_row)
    return plan_row.status


def _copy_reports(bundle_dir: Path, warnings: list[str]) -> None:
    reports_dir = bundle_dir / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    for report_name in OPTIONAL_REPORTS:
        src = REPORTS_DIR / report_name
        if src.is_file():
            shutil.copy2(src, reports_dir / report_name)
        else:
            warnings.append(f"Optional report missing: {report_name}")


def _build_manifest(
    bundle_dir: Path,
    scene: str,
    manifest: dict[str, Any],
    dry_row: Any,
    status: SceneStatus,
    warnings: list[str],
) -> dict[str, Any]:
    robot = manifest.get("robot") if isinstance(manifest.get("robot"), dict) else {}
    end_effector = manifest.get("end_effector") if isinstance(manifest.get("end_effector"), dict) else {}
    task_recipe = manifest.get("task_recipe") if isinstance(manifest.get("task_recipe"), dict) else {}

    files: list[dict[str, str]] = []
    for file_path in sorted(p for p in bundle_dir.rglob("*") if p.is_file()):
        rel = file_path.relative_to(bundle_dir).as_posix()
        if rel == "bundle_manifest.json":
            continue
        files.append({"path": rel, "sha256": _sha256(file_path)})

    final_warnings = list(dict.fromkeys(warnings + status.warnings))
    return {
        "bundle_schema_version": "1.0",
        "scene": scene,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "repository": "Easy_Manipulator_Improved",
        "offline_only": True,
        "contains_runtime_code": False,
        "scene_manifest_path": "scene_manifest.yaml",
        "execution_plan_markdown": "execution_plan.md",
        "execution_plan_json": "execution_plan.json",
        "operator_checklist": "operator_checklist.md",
        "task_recipe_status": status.task_recipe_status,
        "execution_plan_status": status.execution_plan_status,
        "robot": {
            "planning_group": str(robot.get("planning_group", "unknown")),
            "base_frame": str(robot.get("base_frame", "unknown")),
            "ee_link": str(robot.get("ee_link", "unknown")),
        },
        "end_effector": {
            "brand": str(end_effector.get("brand", "unknown")),
            "grasp_frame": str(end_effector.get("grasp_frame", "unknown")),
            "type": _normalize_ee_type(end_effector),
        },
        "task": {
            "recipe_id": str(dry_row.recipe_id),
            "task_type": str(dry_row.task_type),
            "matched_rule_id": str(dry_row.matched_rule_id),
            "destination_id": str(dry_row.selected_destination_id),
        },
        "files": files,
        "warnings": final_warnings,
    }


def _zip_bundle(scene: str, bundle_dir: Path, output_dir: Path, force: bool) -> Path:
    zip_path = output_dir / f"{scene}.zip"
    if zip_path.exists() and not force:
        raise RuntimeError(f"ZIP already exists: {zip_path}. Use --force to overwrite.")
    if zip_path.exists():
        zip_path.unlink()

    with ZipFile(zip_path, "w", compression=ZIP_DEFLATED) as archive:
        for file_path in sorted(path for path in bundle_dir.rglob("*") if path.is_file()):
            archive.write(file_path, file_path.relative_to(bundle_dir).as_posix())
    return zip_path


def export_scene(scene: str, manifest_path: Path, output_dir: Path, zip_output: bool, force: bool) -> tuple[Path, str]:
    bundle_dir = output_dir / scene
    if bundle_dir.exists() and not force:
        raise RuntimeError(f"Bundle directory already exists: {bundle_dir}. Use --force to overwrite.")
    if bundle_dir.exists():
        shutil.rmtree(bundle_dir)
    bundle_dir.mkdir(parents=True, exist_ok=True)

    warnings: list[str] = []
    manifest, _ = _read_manifest(manifest_path)
    status = _status_from_scene(scene, manifest_path)
    dry_row = dry_run.evaluate_scene(scene, manifest_path)

    shutil.copy2(manifest_path, bundle_dir / "scene_manifest.yaml")
    _write_readme(bundle_dir, scene)
    _write_checklist(bundle_dir)
    _write_dry_run_summary(bundle_dir, dry_row)
    status.execution_plan_status = _copy_or_generate_execution_plans(scene, manifest_path, bundle_dir, warnings)
    _copy_reports(bundle_dir, warnings)

    generated_files = [p.relative_to(bundle_dir).as_posix() for p in bundle_dir.rglob("*") if p.is_file()]
    _write_validation_summary(bundle_dir, status, generated_files)

    manifest_payload = _build_manifest(bundle_dir, scene, manifest, dry_row, status, warnings)
    (bundle_dir / "bundle_manifest.json").write_text(
        json.dumps(manifest_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    missing_required = [name for name in REQUIRED_FILES if not (bundle_dir / name).is_file()]
    if missing_required:
        raise RuntimeError(f"Bundle missing required files for scene {scene}: {', '.join(missing_required)}")

    zip_path_text = ""
    if zip_output:
        zip_path = _zip_bundle(scene, bundle_dir, output_dir, force)
        zip_path_text = f" zip={zip_path}"

    return bundle_dir, zip_path_text


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenes", nargs="*", help="Optional scene names to export.")
    parser.add_argument("--zip", action="store_true", help="Also export each bundle as <scene>.zip.")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR), help="Bundle output directory.")
    parser.add_argument("--force", action="store_true", help="Overwrite existing bundle directory and zip.")
    args = parser.parse_args()

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    discovered = dry_run.discover_scene_manifests()
    if args.scenes:
        selected = set(args.scenes)
        discovered = [entry for entry in discovered if entry[0] in selected]

    if not discovered:
        print("No scene manifests discovered for bundle export.")
        return 2

    for scene, manifest_path in discovered:
        bundle_dir, zip_text = export_scene(scene, manifest_path, output_dir, args.zip, args.force)
        print(f"PASS {scene:24} bundle={bundle_dir}{zip_text}")

    print(f"Bundles written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
