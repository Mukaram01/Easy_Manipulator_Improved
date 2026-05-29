#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from supported_scene_catalog import DEFAULT_SUPPORTED_SCENES_CATALOG, load_supported_scene_catalog

DEFAULT_REGISTRY = DEFAULT_SUPPORTED_SCENES_CATALOG
VALIDATOR = Path("scripts/validate_guided_generated_scene_build_readiness.py")


def _parse_package_xml_name(scene_path: Path) -> tuple[str | None, list[str]]:
    """Return the ROS package name declared by scene_path/package.xml."""
    package_xml = scene_path / "package.xml"
    if not package_xml.exists():
        return None, [f"package_xml_missing: {package_xml}"]
    if not package_xml.is_file():
        return None, [f"package_xml_not_file: {package_xml}"]
    try:
        root = ET.parse(package_xml).getroot()
    except ET.ParseError as exc:
        return None, [f"package_xml_malformed: {exc}"]
    name_node = root.find("name")
    package_name = (name_node.text or "").strip() if name_node is not None else ""
    if not package_name:
        return None, ["package_xml_missing_name"]
    return package_name, []


def _build_command_package_selection(build_command: str) -> list[str]:
    try:
        tokens = shlex.split(build_command)
    except ValueError:
        return []

    selected: list[str] = []
    for idx, token in enumerate(tokens):
        if token == "--packages-select":
            for value in tokens[idx + 1 :]:
                if value.startswith("-"):
                    break
                selected.append(value)
            break
        if token.startswith("--packages-select="):
            selected.append(token.split("=", 1)[1])
            break
    return selected


def _fake_hardware_launch_package(fake_hardware_launch_command: str) -> str | None:
    try:
        tokens = shlex.split(fake_hardware_launch_command)
    except ValueError:
        return None

    for idx in range(len(tokens) - 2):
        if tokens[idx] == "ros2" and tokens[idx + 1] == "launch":
            return tokens[idx + 2]
    return None


def _catalog_contract_mismatches(catalog_entry, scene_dir: Path) -> tuple[str | None, list[str]]:
    package_xml_name, package_xml_errors = _parse_package_xml_name(scene_dir)
    mismatches = list(package_xml_errors)

    if package_xml_name:
        if catalog_entry.package_name != package_xml_name:
            mismatches.append(
                "catalog_package_name_mismatch: "
                f"catalog package_name '{catalog_entry.package_name}' != package.xml <name> '{package_xml_name}'"
            )
        if catalog_entry.build_package_name != package_xml_name:
            mismatches.append(
                "build_package_name_mismatch: "
                f"catalog build_package_name '{catalog_entry.build_package_name}' != package.xml <name> '{package_xml_name}'"
            )

    build_selection = _build_command_package_selection(catalog_entry.build_command)
    if build_selection != [catalog_entry.build_package_name]:
        mismatches.append(
            "build_command_package_selection_mismatch: "
            f"build_command --packages-select {build_selection or '<missing>'} != build_package_name '{catalog_entry.build_package_name}'"
        )

    launch_package = _fake_hardware_launch_package(catalog_entry.fake_hardware_launch_command)
    if launch_package != catalog_entry.package_name:
        mismatches.append(
            "fake_hardware_launch_package_mismatch: "
            f"fake_hardware_launch_command package '{launch_package or '<missing>'}' != package_name '{catalog_entry.package_name}'"
        )

    return package_xml_name, mismatches


def _run_validator(repo_root: Path, workspace_root: Path, scene: str, skip_build: bool, skip_launch_smoke: bool, timeout_sec: int):
    validator_path = repo_root / VALIDATOR
    if not validator_path.exists():
        validator_path = Path(__file__).resolve().parent / 'validate_guided_generated_scene_build_readiness.py'
    cmd = [
        sys.executable,
        str(validator_path),
        "--scene",
        scene,
        "--repo-root",
        str(repo_root),
        "--workspace-root",
        str(workspace_root),
        "--json",
        "--timeout-sec",
        str(timeout_sec),
    ]
    if skip_build:
        cmd.append("--skip-build")
    if skip_launch_smoke:
        cmd.append("--skip-launch-smoke")

    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    payload = {}
    if proc.stdout.strip():
        try:
            payload = json.loads(proc.stdout)
        except json.JSONDecodeError:
            payload = {"status": "BLOCKED", "blockers": ["validator_output_not_json"], "warnings": [proc.stdout.strip()[:500]]}
    if not isinstance(payload, dict):
        payload = {"status": "BLOCKED", "blockers": ["validator_output_not_object"], "warnings": []}
    return cmd, proc.returncode, payload


def _build_markdown(report: dict) -> str:
    lines = [
        "# Workcell Studio All Scenes Readiness Report",
        "",
        "Scene | Support Level | Static | Build | Fake Launch | Status | Blocker",
        "--- | --- | --- | --- | --- | --- | ---",
    ]
    for row in report["per_scene"]:
        blockers = "; ".join(row.get("blockers", [])) or "-"
        lines.append(
            f"{row['scene_name']} | {row['support_level']} | {row['static_validation']['status']} | {row['build']['status']} | {row['launch_smoke']['status']} | {row['status']} | {blockers}"
        )
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="Validate supported Workcell Studio scenes from a registry.")
    ap.add_argument("--repo-root", type=Path, required=True)
    ap.add_argument("--workspace-root", type=Path, required=True)
    ap.add_argument("--registry", type=Path, default=None)
    ap.add_argument("--json", action="store_true", dest="json_out")
    ap.add_argument("--skip-build", action="store_true")
    ap.add_argument("--skip-launch-smoke", action="store_true")
    ap.add_argument("--scene", action="append", default=[])
    ap.add_argument("--include-experimental", action="store_true")
    ap.add_argument("--timeout-sec", type=int, default=30)
    ap.add_argument("--output", type=Path, default=None)
    args = ap.parse_args()

    repo_root = args.repo_root.resolve()
    workspace_root = args.workspace_root.resolve()
    registry_path = (args.registry or (repo_root / DEFAULT_REGISTRY)).resolve()

    report = {
        "registry_path": str(registry_path),
        "workspace_root": str(workspace_root),
        "repo_root": str(repo_root),
        "scenes_checked": [],
        "scenes_skipped": [],
        "per_scene": [],
        "summary": {"total": 0, "pass": 0, "fail": 0, "blocked": 0, "warnings": 0, "skipped": 0},
    }

    if not registry_path.exists():
        report["summary"]["blocked"] = 1
        report["error"] = f"Registry does not exist: {registry_path}"
        if args.json_out:
            print(json.dumps(report, indent=2))
        return 2

    registry, entries, catalog_errors = load_supported_scene_catalog(registry_path)
    if catalog_errors:
        report["catalog_errors"] = catalog_errors
        report["summary"]["blocked"] = 1
        if args.json_out:
            print(json.dumps(report, indent=2))
        return 2

    filter_set = set(args.scene or [])

    for catalog_entry in entries:
        entry = catalog_entry.raw
        scene_name = catalog_entry.scene_name
        if not scene_name:
            continue
        if filter_set and scene_name not in filter_set:
            continue

        support_level = catalog_entry.support_level
        enabled = catalog_entry.enabled
        scene_dir = (repo_root / catalog_entry.scene_path).resolve()
        required = list(catalog_entry.required_files)
        skip_build = args.skip_build or bool(entry.get("allow_skip_build", False))
        skip_launch = args.skip_launch_smoke or bool(entry.get("allow_skip_launch_smoke", False))

        row = {
            "scene_name": scene_name,
            "support_level": support_level,
            "catalog_status": catalog_entry.status,
            "known_blocker": catalog_entry.known_blocker,
            "status": "SKIPPED",
            "package_name": catalog_entry.package_name,
            "build_package_name": catalog_entry.build_package_name,
            "package_xml_name": None,
            "authoring_files": list(catalog_entry.authoring_files),
            "generated_files": list(catalog_entry.generated_files),
            "required_files": required,
            "validation_command": catalog_entry.validation_command,
            "build_command": catalog_entry.build_command,
            "fake_hardware_launch_command": catalog_entry.fake_hardware_launch_command,
            "static_validation": {"status": "SKIPPED", "missing_files": []},
            "guided_build_launch_readiness": {"status": "SKIPPED"},
            "build": {"status": "SKIPPED"},
            "launch_smoke": {"status": "SKIPPED"},
            "blockers": [],
            "warnings": [],
            "commands_run": [],
            "artifact_paths": {},
            "catalog_contract": {"status": "SKIPPED", "mismatches": []},
        }

        if not enabled:
            report["scenes_skipped"].append(scene_name)
            row["status"] = "SKIPPED"
            row["blockers"].append("scene_disabled_in_catalog")
            report["per_scene"].append(row)
            continue
        if support_level == "experimental" and not args.include_experimental:
            report["scenes_skipped"].append(scene_name)
            row["status"] = "SKIPPED"
            row["warnings"].append("experimental_scene_skipped_without_include_experimental")
            report["per_scene"].append(row)
            continue

        report["scenes_checked"].append(scene_name)
        if not scene_dir.exists():
            row["status"] = "BLOCKED"
            row["static_validation"] = {"status": "BLOCKED", "missing_files": required}
            if catalog_entry.status == "blocked":
                row["blockers"].append(catalog_entry.known_blocker or "scene_marked_blocked_in_catalog")
            else:
                row["blockers"].append(f"scene_path_missing: {scene_dir}")
            report["per_scene"].append(row)
            continue

        package_xml_name, catalog_mismatches = _catalog_contract_mismatches(catalog_entry, scene_dir)
        row["package_xml_name"] = package_xml_name
        row["catalog_contract"] = {
            "status": "FAIL" if catalog_mismatches else "PASS",
            "mismatches": catalog_mismatches,
        }

        missing = [rel for rel in required if not (scene_dir / rel).exists()]
        row["static_validation"] = {"status": "PASS" if not missing else "FAIL", "missing_files": missing}
        if missing:
            if catalog_entry.status == "blocked":
                row["status"] = "BLOCKED"
                row["blockers"].append(catalog_entry.known_blocker or "scene_marked_blocked_in_catalog")
            else:
                row["status"] = "FAIL"
                row["blockers"].extend([f"missing_required_file: {m}" for m in missing])
                if catalog_entry.status == "supported":
                    row["blockers"].extend(catalog_mismatches)
            report["per_scene"].append(row)
            continue

        if catalog_entry.status == "blocked":
            row["status"] = "BLOCKED"
            row["blockers"].append(catalog_entry.known_blocker or "scene_marked_blocked_in_catalog")
            report["per_scene"].append(row)
            continue

        if catalog_entry.status == "supported" and catalog_mismatches:
            row["status"] = "BLOCKED"
            row["blockers"].extend(catalog_mismatches)
            report["per_scene"].append(row)
            continue

        cmd, _, payload = _run_validator(repo_root, workspace_root, str(scene_dir), skip_build, skip_launch, args.timeout_sec)
        row["commands_run"].append(" ".join(cmd))
        guided_status = payload.get("status", "BLOCKED")
        row["guided_build_launch_readiness"] = payload
        row["build"] = payload.get("build", {"status": "SKIPPED"})
        row["launch_smoke"] = payload.get("launch_smoke", {"status": "SKIPPED"})
        row["artifact_paths"] = payload.get("artifact_paths", {})
        row["warnings"].extend(payload.get("warnings", []))
        row["blockers"].extend(payload.get("blockers", []))

        if guided_status in {"PASS", "PASS_WITH_WARNINGS", "FAIL", "BLOCKED"}:
            row["status"] = guided_status
        else:
            row["status"] = "BLOCKED"
            row["blockers"].append(f"unknown_guided_status: {guided_status}")

        report["per_scene"].append(row)

    for row in report["per_scene"]:
        report["summary"]["total"] += 1
        status = row["status"]
        if status == "PASS":
            report["summary"]["pass"] += 1
        elif status == "PASS_WITH_WARNINGS":
            report["summary"]["pass"] += 1
            report["summary"]["warnings"] += 1
        elif status == "FAIL":
            report["summary"]["fail"] += 1
        elif status == "BLOCKED":
            report["summary"]["blocked"] += 1
        elif status == "SKIPPED":
            report["summary"]["skipped"] += 1

    md_path = args.output.resolve() if args.output else (repo_root / "build/workcell_studio/all_scenes_readiness_report.md")
    md_path.parent.mkdir(parents=True, exist_ok=True)
    md_path.write_text(_build_markdown(report), encoding="utf-8")
    report["markdown_report_path"] = str(md_path)

    if args.json_out:
        print(json.dumps(report, indent=2))

    json_path = None
    if args.output and args.output.suffix.lower() == ".json":
        json_path = args.output.resolve()
    elif args.output:
        json_path = args.output.resolve().with_suffix(".json")
    if json_path is not None:
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    return 0 if report["summary"]["fail"] == 0 and report["summary"]["blocked"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
