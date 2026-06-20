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

MESH_INDEX_REL_PATH = "generated/scene_visual_mesh_index.json"
RVIZ_TRUTH_REQUIRED_FILES = ("package.xml", "CMakeLists.txt", "launch/demo.launch.py", "urdf/scene.urdf.xacro")



def _parse_package_xml_name(scene_dir: Path) -> tuple[str | None, str | None]:
    """Return the ROS package name declared by <scene_dir>/package.xml."""
    package_xml = scene_dir / "package.xml"
    if not package_xml.exists():
        return None, "package_xml_missing: package.xml"
    try:
        root = ET.parse(package_xml).getroot()
    except ET.ParseError as exc:
        return None, f"package_xml_invalid_xml: package.xml ({exc})"
    name_element = root.find("name")
    if name_element is None or not (name_element.text or "").strip():
        return None, "package_xml_missing_name: package.xml <name> must be non-empty"
    return name_element.text.strip(), None


def _extract_colcon_packages_select(command: str) -> list[str]:
    """Extract package names passed to colcon build --packages-select."""
    try:
        tokens = shlex.split(command)
    except ValueError:
        return []
    try:
        option_index = tokens.index("--packages-select")
    except ValueError:
        return []
    selected: list[str] = []
    for token in tokens[option_index + 1:]:
        if token.startswith("-"):
            break
        selected.extend(part for part in token.split(",") if part)
    return selected


def _extract_ros2_launch_package(command: str) -> str | None:
    """Extract the package argument from a ros2 launch command."""
    try:
        tokens = shlex.split(command)
    except ValueError:
        return None
    for index in range(len(tokens) - 2):
        if tokens[index] == "ros2" and tokens[index + 1] == "launch":
            return tokens[index + 2]
    try:
        launch_index = tokens.index("launch")
    except ValueError:
        return None
    if launch_index + 1 < len(tokens):
        return tokens[launch_index + 1]
    return None


def _check_rviz_truth_readiness(catalog_entry, scene_dir: Path) -> dict:
    """Validate RViz/MoveIt truth-preview package files and safe fake-hardware command."""
    missing = [rel for rel in RVIZ_TRUTH_REQUIRED_FILES if not (scene_dir / rel).exists()]
    package_contract = _check_package_contract(catalog_entry, scene_dir)
    command = catalog_entry.fake_hardware_launch_command
    launch_rviz_supported = "launch_rviz:=true" in command
    blockers = [f"rviz_truth_missing_file: {rel}" for rel in missing]
    blockers.extend(package_contract.get("blockers", []))
    if "use_fake_hardware:=true" not in command:
        blockers.append("rviz_truth_unsafe_launch_command: use_fake_hardware:=true is required")
    return {
        "status": "PASS" if not blockers else "BLOCKED",
        "required_files": list(RVIZ_TRUTH_REQUIRED_FILES),
        "missing_files": missing,
        "safe_fake_hardware_launch_command": command,
        "use_fake_hardware_required": "use_fake_hardware:=true" in command,
        "launch_rviz_supported": launch_rviz_supported,
        "package_contract": package_contract,
        "blockers": blockers,
    }


def _check_package_contract(catalog_entry, scene_dir: Path) -> dict:
    """Validate catalog package names and commands against package.xml."""
    package_xml_name, package_xml_error = _parse_package_xml_name(scene_dir)
    build_command_packages = _extract_colcon_packages_select(catalog_entry.build_command)
    fake_launch_package = _extract_ros2_launch_package(catalog_entry.fake_hardware_launch_command)

    result = {
        "status": "PASS",
        "package_xml_name": package_xml_name,
        "build_command_packages_select": build_command_packages,
        "fake_hardware_launch_package": fake_launch_package,
        "blockers": [],
    }

    if package_xml_error:
        result["blockers"].append(package_xml_error)
    else:
        if catalog_entry.package_name != package_xml_name:
            result["blockers"].append(
                f"catalog_package_name_mismatch: package_name={catalog_entry.package_name!r} package.xml name={package_xml_name!r}"
            )
        if catalog_entry.build_package_name != package_xml_name:
            result["blockers"].append(
                f"catalog_build_package_name_mismatch: build_package_name={catalog_entry.build_package_name!r} package.xml name={package_xml_name!r}"
            )

    if build_command_packages != [catalog_entry.build_package_name]:
        result["blockers"].append(
            "build_command_package_selection_mismatch: "
            f"--packages-select={build_command_packages!r} build_package_name={catalog_entry.build_package_name!r}"
        )
    if fake_launch_package != catalog_entry.package_name:
        result["blockers"].append(
            f"fake_hardware_launch_package_mismatch: launch_package={fake_launch_package!r} package_name={catalog_entry.package_name!r}"
        )

    if result["blockers"]:
        result["status"] = "BLOCKED"
    return result


def _mesh_index_result(status: str, blocker: str | None = None) -> dict:
    result = {
        "path": MESH_INDEX_REL_PATH,
        "status": status,
        "renderable_items": 0,
        "total_items": 0,
        "malformed_items": [],
        "items_key": "",
        "blockers": [],
    }
    if blocker:
        result["blockers"].append(blocker)
    return result


def _check_mesh_index_contract(scene_dir: Path) -> dict:
    """Validate optional native Scene3D editable-preview mesh diagnostics, when present."""
    index_path = scene_dir / MESH_INDEX_REL_PATH
    if not index_path.exists():
        return _mesh_index_result(
            "OPTIONAL_MISSING",
            f"native_scene3d_optional_mesh_index_missing: {MESH_INDEX_REL_PATH}",
        )

    try:
        payload = json.loads(index_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        return _mesh_index_result(
            "INVALID_JSON",
            f"mesh_index_validation_invalid_json: {MESH_INDEX_REL_PATH} ({exc.msg} at line {exc.lineno}, column {exc.colno})",
        )

    if not isinstance(payload, dict):
        return _mesh_index_result(
            "ROOT_NOT_OBJECT",
            f"mesh_index_validation_root_not_object: {MESH_INDEX_REL_PATH} root must be a JSON object",
        )

    has_visual_items = "visual_items" in payload
    has_items = "items" in payload
    if not has_visual_items and not has_items:
        return _mesh_index_result(
            "MISSING_ITEMS",
            f"mesh_index_validation_missing_items: {MESH_INDEX_REL_PATH} must define items or visual_items",
        )

    items_key = "visual_items" if isinstance(payload.get("visual_items"), list) else "items"
    items = payload.get(items_key)
    if not isinstance(items, list):
        return _mesh_index_result(
            "MALFORMED_ITEMS",
            f"mesh_index_validation_malformed_items: {MESH_INDEX_REL_PATH} items/visual_items must be a list",
        )

    result = _mesh_index_result("PASS")
    result["items_key"] = items_key
    result["total_items"] = len(items)

    malformed_items: list[dict] = []
    renderable_items = 0
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            malformed_items.append({"index": index, "reason": "item_not_object"})
            continue
        render_expected = item.get("render_expected", True)
        if not isinstance(render_expected, bool):
            malformed_items.append({"index": index, "reason": "render_expected_not_bool"})
            continue
        if render_expected:
            renderable_items += 1

    result["renderable_items"] = renderable_items
    result["malformed_items"] = malformed_items

    if malformed_items:
        result["status"] = "MALFORMED_ITEMS"
        result["blockers"].append(
            f"mesh_index_validation_malformed_items: {MESH_INDEX_REL_PATH} contains {len(malformed_items)} malformed item entries"
        )
        return result

    if renderable_items <= 0:
        result["status"] = "NO_RENDERABLE_ITEMS"
        result["blockers"].append(
            f"mesh_index_validation_no_renderable_items: {MESH_INDEX_REL_PATH} contains 0 renderable items"
        )

    return result


def _join_blockers(blockers: list[str]) -> str:
    return "; ".join(str(blocker) for blocker in blockers if str(blocker).strip())

def _build_markdown(report: dict) -> str:
    lines = [
        "# Workcell Studio All Scenes Readiness Report",
        "",
        "Scene | Support Level | Static | Build | Fake Launch | Status | Blocker",
        "--- | --- | --- | --- | --- | --- | ---",
    ]
    for row in report["per_scene"]:
        blocker = row.get("blocker") or _join_blockers(row.get("blockers", [])) or "-"
        lines.append(
            f"{row['scene_name']} | {row['support_level']} | {row['static_validation']['status']} | {row['build']['status']} | {row['launch_smoke']['status']} | {row['status']} | {blocker}"
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

        missing_authoring_files = [rel for rel in catalog_entry.authoring_files if not (scene_dir / rel).exists()]
        missing_generated_files = [rel for rel in catalog_entry.generated_files if not (scene_dir / rel).exists()]
        rviz_truth_missing_generated_files = [rel for rel in missing_generated_files if rel != MESH_INDEX_REL_PATH]
        missing_required_files = [*missing_authoring_files, *rviz_truth_missing_generated_files]

        row = {
            "scene_name": scene_name,
            "support_level": support_level,
            "catalog_status": catalog_entry.status,
            "known_blocker": catalog_entry.known_blocker,
            "status": "SKIPPED",
            "blocker": "",
            "missing_authoring_files": missing_authoring_files,
            "missing_generated_files": missing_generated_files,
            "validation_result": "SKIPPED",
            "package_name": catalog_entry.package_name,
            "build_package_name": catalog_entry.build_package_name,
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
            "package_contract": {"status": "SKIPPED"},
            "rviz_truth_readiness": {"status": "SKIPPED"},
            "native_scene3d_editable_preview_diagnostics": {"status": "SKIPPED"},
            "mesh_index_validation": {"status": "SKIPPED"},
            "mesh_index": {"status": "SKIPPED"},
            "blockers": [],
            "warnings": [],
            "commands_run": [],
            "artifact_paths": {},
        }

        if catalog_entry.status == "blocked":
            report["scenes_checked"].append(scene_name)
            catalog_blocker = catalog_entry.known_blocker
            row["status"] = "BLOCKED"
            row["validation_result"] = "BLOCKED"
            row["known_blocker"] = catalog_blocker
            row["blocker"] = catalog_blocker
            row["blockers"] = [catalog_blocker]
            row["static_validation"] = {
                "status": "PASS" if not missing_required_files else "FAIL",
                "missing_files": missing_required_files,
            }
            if scene_dir.exists():
                rviz_truth = _check_rviz_truth_readiness(catalog_entry, scene_dir)
                row["rviz_truth_readiness"] = rviz_truth
                package_contract = rviz_truth["package_contract"]
                row["package_contract"] = package_contract
                row["blockers"].extend(rviz_truth.get("blockers", []))
                mesh_index = _check_mesh_index_contract(scene_dir)
                row["native_scene3d_editable_preview_diagnostics"] = mesh_index
                row["mesh_index_validation"] = mesh_index
                row["mesh_index"] = mesh_index
                row["warnings"].extend(mesh_index.get("blockers", []))
            report["per_scene"].append(row)
            continue

        if not enabled or catalog_entry.status == "disabled":
            report["scenes_skipped"].append(scene_name)
            row["status"] = "SKIPPED"
            row["validation_result"] = "SKIPPED"
            row["blockers"].append("scene_disabled_in_catalog")
            row["blocker"] = _join_blockers(row["blockers"])
            report["per_scene"].append(row)
            continue
        if support_level == "experimental" and not args.include_experimental:
            report["scenes_skipped"].append(scene_name)
            row["status"] = "SKIPPED"
            row["validation_result"] = "SKIPPED"
            row["warnings"].append("experimental_scene_skipped_without_include_experimental")
            report["per_scene"].append(row)
            continue
        report["scenes_checked"].append(scene_name)
        if not scene_dir.exists():
            row["status"] = "BLOCKED"
            row["validation_result"] = "BLOCKED"
            row["static_validation"] = {"status": "BLOCKED", "missing_files": required}
            row["blockers"].append(f"scene_path_missing: {scene_dir}")
            row["blocker"] = _join_blockers(row["blockers"])
            report["per_scene"].append(row)
            continue

        row["static_validation"] = {"status": "PASS" if not missing_required_files else "FAIL", "missing_files": missing_required_files}

        rviz_truth = _check_rviz_truth_readiness(catalog_entry, scene_dir)
        row["rviz_truth_readiness"] = rviz_truth
        package_contract = rviz_truth["package_contract"]
        row["package_contract"] = package_contract
        if rviz_truth["status"] != "PASS":
            row["blockers"].extend(rviz_truth.get("blockers", []))

        mesh_index = _check_mesh_index_contract(scene_dir)
        row["native_scene3d_editable_preview_diagnostics"] = mesh_index
        row["mesh_index_validation"] = mesh_index
        row["mesh_index"] = mesh_index
        if mesh_index["status"] != "PASS":
            row["warnings"].extend(mesh_index.get("blockers", []))

        if rviz_truth["status"] != "PASS":
            row["status"] = "BLOCKED"
            row["validation_result"] = "BLOCKED"
            if missing_required_files:
                row["blockers"] = [f"missing_required_file: {m}" for m in missing_required_files] + row["blockers"]
            row["blocker"] = _join_blockers(row["blockers"])
            report["per_scene"].append(row)
            continue

        if missing_required_files:
            row["status"] = "FAIL"
            row["validation_result"] = "FAIL"
            row["blockers"] = [f"missing_required_file: {m}" for m in missing_required_files] + row["blockers"]
            row["blocker"] = _join_blockers(row["blockers"])
            report["per_scene"].append(row)
            continue

        cmd, _, payload = _run_validator(repo_root, workspace_root, str(scene_dir), skip_build, skip_launch, args.timeout_sec)
        row["commands_run"].append(" ".join(cmd))
        guided_status = payload.get("status", "BLOCKED")
        row["validation_result"] = guided_status
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

        row["blocker"] = _join_blockers(row["blockers"])
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
