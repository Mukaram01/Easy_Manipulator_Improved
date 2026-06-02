#!/usr/bin/env python3
"""Catalog-backed Workcell Studio scene readiness matrix.

This script performs offline, fake-hardware-first readiness checks for every
scene that the supported-scene catalog marks as an enabled supported entry.  It
writes a JSON and Markdown summary under build/workcell_studio_scene_readiness
by default.  ROS launch commands are only derived and recorded; they are not
executed by this script.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT_DEFAULT = SCRIPT_DIR.parents[0]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_scene_readiness
import validate_cell_definition
import validate_scene3d_visual_quality_matrix as scene3d_quality
from workcell_studio_path_resolver import (
    resolve_workcell_builder_executable,
    resolve_workspace_root,
)

SCHEMA_VERSION = "workcell_studio_scene_readiness_matrix/v1"
SAFE_LAUNCH_RE = re.compile(
    r"^ros2\s+launch\s+(?P<package>[A-Za-z0-9_]+)\s+demo\.launch\.py"
    r"(?=.*(?:^|\s)use_fake_hardware:=true(?:\s|$))"
    r"(?=.*(?:^|\s)launch_rviz:=true(?:\s|$))[A-Za-z0-9_:=./\-\s]*$"
)
REQUIRED_FILES = (
    ("package_xml", "package.xml", False),
    ("cmakelists", "CMakeLists.txt", False),
    ("environment_yaml", "environment.yaml", False),
    ("cell_definition_yaml", "cell_definition.yaml", False),
    ("scene_manifest_yaml", "scene_manifest.yaml", False),
    ("workcell_studio_layout_yaml", "layout/workcell_studio_layout.yaml", False),
    ("demo_launch_py", "launch/demo.launch.py", False),
    ("scene_urdf_xacro", "urdf/scene.urdf.xacro", False),
    ("scene_visual_mesh_index_json", "generated/scene_visual_mesh_index.json", False),
    ("scene_package_readiness_json", "generated/scene_package_readiness.json", True),
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _load_yaml(path: Path) -> dict[str, Any]:
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError(f"YAML root must be a mapping/object: {path}")
    return loaded


def _string(value: Any) -> str:
    return str(value or "").strip()


def _lower(value: Any) -> str:
    return _string(value).lower()


def _rel(path: Path, repo_root: Path) -> str:
    try:
        return str(path.resolve().relative_to(repo_root.resolve()))
    except Exception:
        return str(path)


def _category(status: str, summary: str, *, details: dict[str, Any] | None = None) -> dict[str, Any]:
    return {"status": status, "summary": summary, "details": details or {}}


def _file_category(scene_path: Path, rel_path: str, *, optional: bool = False) -> dict[str, Any]:
    path = scene_path / rel_path
    if path.is_file():
        details: dict[str, Any] = {"path": str(path), "present": True, "optional": optional}
        if rel_path.endswith(('.json', '.yaml', '.yml')):
            try:
                if rel_path.endswith('.json'):
                    json.loads(path.read_text(encoding="utf-8"))
                else:
                    yaml.safe_load(path.read_text(encoding="utf-8"))
                details["parseable"] = True
            except Exception as exc:
                details["parseable"] = False
                details["error"] = str(exc)
                return _category("FAIL", f"{rel_path} exists but is not parseable: {exc}", details=details)
        return _category("PASS", f"{rel_path} present", details=details)
    if optional:
        return _category(
            "PASS",
            f"{rel_path} is optional/generated-if-present and is not present",
            details={"path": str(path), "present": False, "optional": True},
        )
    return _category("FAIL", f"Missing required file: {rel_path}", details={"path": str(path), "present": False})


def _scene_entries(catalog_path: Path) -> list[dict[str, Any]]:
    catalog = _load_yaml(catalog_path)
    scenes = catalog.get("scenes")
    if not isinstance(scenes, list):
        return []
    entries: list[dict[str, Any]] = []
    for entry in scenes:
        if not isinstance(entry, dict):
            continue
        enabled = bool(entry.get("enabled", True))
        support_level = _lower(entry.get("support_level"))
        status = _lower(entry.get("status"))
        if enabled and support_level == "supported" and status not in {"disabled", "ignored", "unsupported"}:
            entries.append(entry)
    return entries


def _resolve_catalog_value(entry: dict[str, Any], key: str, fallback: str) -> str:
    value = _string(entry.get(key))
    return value or fallback


def _requires_cmakelists(scene_path: Path) -> bool:
    package_xml = scene_path / "package.xml"
    if not package_xml.is_file():
        return True
    try:
        text = package_xml.read_text(encoding="utf-8", errors="ignore")
    except Exception:
        return True
    return "ament_cmake" in text or "<build_type>ament_cmake</build_type>" in text


def _validate_cell_definition(scene_path: Path) -> dict[str, Any]:
    path = scene_path / "cell_definition.yaml"
    if not path.is_file():
        return _category("BLOCKED", "cell_definition.yaml is missing; validation cannot run", details={"path": str(path)})
    try:
        loaded, parser_name, notes = validate_cell_definition.load_yaml(path)
        summary = validate_cell_definition.validate_cell_definition(
            loaded,
            path,
            parser_name,
            notes,
            strict=False,
            capabilities_dir=validate_cell_definition.DEFAULT_CAPABILITIES_DIR,
        )
    except Exception as exc:
        return _category("FAIL", f"cell_definition.yaml validation failed to execute: {exc}", details={"path": str(path), "error": str(exc)})
    details = {
        "path": str(path),
        "parser": summary.parser,
        "errors": summary.errors,
        "warnings": summary.warnings,
        "notes": summary.notes,
        "capabilities": summary.capability_summary,
        "environment_layout": summary.environment_layout_summary,
        "grasp_strategy": summary.grasp_strategy_summary,
    }
    if summary.errors:
        return _category("FAIL", f"cell definition has {len(summary.errors)} error(s)", details=details)
    return _category("PASS", "cell definition validates offline" + (f" with {len(summary.warnings)} warning(s)" if summary.warnings else ""), details=details)


def _manifest_local_file_references(scene_path: Path) -> dict[str, Any]:
    path = scene_path / "scene_manifest.yaml"
    if not path.is_file():
        return _category("BLOCKED", "scene_manifest.yaml is missing; local-file references cannot be checked", details={"path": str(path)})
    try:
        manifest = _load_yaml(path)
    except Exception as exc:
        return _category("FAIL", f"scene_manifest.yaml is not parseable: {exc}", details={"path": str(path), "error": str(exc)})
    files = manifest.get("files")
    if not isinstance(files, dict) or not files:
        return _category("FAIL", "scene_manifest.yaml has no files mapping to validate", details={"path": str(path), "files": files})
    checked: list[dict[str, Any]] = []
    missing: list[str] = []
    external: list[str] = []
    for key, raw in sorted(files.items()):
        if isinstance(raw, str) and raw.strip():
            value = raw.strip()
            if "://" in value or Path(value).is_absolute():
                external.append(value)
                checked.append({"key": key, "value": value, "local": False, "present": False})
                continue
            exists = (scene_path / value).is_file()
            checked.append({"key": key, "value": value, "local": True, "present": exists})
            if not exists:
                missing.append(value)
    details = {"path": str(path), "checked": checked, "missing": missing, "external_or_absolute": external}
    if external:
        return _category("FAIL", "manifest files mapping contains external or absolute file references", details=details)
    if missing:
        return _category("FAIL", f"manifest references {len(missing)} missing local file(s)", details=details)
    return _category("PASS", f"manifest local files resolve ({len(checked)} checked)", details=details)


def _scene3d_visual_quality(scene_name: str, scene_path: Path) -> dict[str, Any]:
    mesh_index_path = scene_path / "generated" / "scene_visual_mesh_index.json"
    smoke_json_path = scene_path / "generated" / "scene3d_gui_smoke.json"
    result = scene3d_quality.evaluate_scene(
        scene_name=scene_name,
        scene_dir=scene_path,
        mesh_index_path=mesh_index_path,
        smoke_json_path=smoke_json_path,
    )
    blockers = list(result.get("blockers") or [])
    details = dict(result)
    if blockers:
        # Missing GUI smoke evidence is expected in this offline matrix and blocks
        # visual-quality evaluation rather than proving a scene is visually bad.
        if all(str(b).startswith("smoke_json_missing") for b in blockers):
            return _category("BLOCKED", "Scene3D smoke evidence is missing; visual-quality summary cannot be completed", details=details)
        return _category("FAIL", f"Scene3D visual-quality check has {len(blockers)} blocker(s)", details=details)
    return _category("PASS", "Scene3D visual-quality evidence passes", details=details)


def _credible_physical_visual_evidence(scene_path: Path) -> dict[str, Any]:
    mesh_index_path = scene_path / "generated" / "scene_visual_mesh_index.json"
    if not mesh_index_path.is_file():
        return _category("BLOCKED", "scene_visual_mesh_index.json is missing; physical visual evidence cannot be evaluated", details={"path": str(mesh_index_path)})
    mesh_index = scene3d_quality._load_json(mesh_index_path)
    if mesh_index.get("_load_error"):
        return _category("FAIL", f"scene_visual_mesh_index.json is not readable: {mesh_index['_load_error']}", details={"path": str(mesh_index_path), "error": mesh_index["_load_error"]})
    total, mesh_sources, primitive_sources, missing = scene3d_quality.classify_source_geometry(mesh_index)
    failure_summary = scene3d_quality.mesh_failure_summary(mesh_index)
    details = {
        "path": str(mesh_index_path),
        "total_payload_count": total,
        "mesh_source_count": mesh_sources,
        "primitive_source_count": primitive_sources,
        "missing_geometry_count": missing,
        "mesh_failure_summary_by_reason_code": failure_summary,
    }
    if failure_summary:
        return _category("FAIL", "mesh-backed source items have unresolved failure reasons", details=details)
    if mesh_sources + primitive_sources <= 0:
        return _category("FAIL", "no mesh-backed or primitive physical visual source items were found", details=details)
    return _category("PASS", f"credible physical visual source evidence found ({mesh_sources} mesh, {primitive_sources} primitive)", details=details)


def _safe_fake_hardware_launch(entry: dict[str, Any], package_name: str, build_package_name: str) -> dict[str, Any]:
    raw = _string(entry.get("fake_hardware_launch_command"))
    derived = f"ros2 launch {build_package_name or package_name} demo.launch.py use_fake_hardware:=true launch_rviz:=true"
    command = raw or derived
    match = SAFE_LAUNCH_RE.match(command)
    details = {
        "catalog_command": raw,
        "derived_command": derived,
        "recorded_command": command,
        "safe_pattern": SAFE_LAUNCH_RE.pattern,
        "invoked": False,
    }
    if not match:
        return _category("FAIL", "fake-hardware launch command is missing required safe use_fake_hardware/launch_rviz flags or has an unsafe shape", details=details)
    launch_package = match.group("package")
    expected_packages = {p for p in (package_name, build_package_name) if p}
    if expected_packages and launch_package not in expected_packages:
        details["expected_packages"] = sorted(expected_packages)
        details["launch_package"] = launch_package
        return _category("FAIL", "fake-hardware launch command package does not match catalog package/build package", details=details)
    return _category("PASS", "safe fake-hardware launch command recorded but not executed", details=details)


def _ros_launch_smoke_state(ros_humble_available: bool, launch_category: dict[str, Any]) -> dict[str, Any]:
    details = {
        "ros_humble_available": ros_humble_available,
        "safe_command_available": launch_category.get("status") == "PASS",
        "command": (launch_category.get("details") or {}).get("recorded_command"),
        "invoked": False,
        "reason": "offline readiness matrix records safe launch commands but does not invoke ROS launch or any robot motion",
    }
    if launch_category.get("status") != "PASS":
        return _category("BLOCKED", "ROS launch smoke not evaluated because no safe fake-hardware launch command is available", details=details)
    if not ros_humble_available:
        return _category("BLOCKED", "ROS Humble setup not found; fake-hardware launch smoke remains skipped", details=details)
    return _category("BLOCKED", "ROS Humble is available, but launch smoke is intentionally skipped by this offline matrix", details=details)


def _scene_package_exists(scene_path: Path) -> dict[str, Any]:
    if scene_path.is_dir():
        return _category("PASS", "scene package directory exists", details={"path": str(scene_path)})
    return _category("FAIL", "scene package directory is missing", details={"path": str(scene_path)})


def _check_scene_readiness_summary(scene_path: Path) -> dict[str, Any]:
    try:
        payload = check_scene_readiness.check_readiness(None, scene_path, strict=False)
    except Exception as exc:
        return {"result": "FAIL", "errors": [str(exc)], "warnings": [], "notes": [], "summary": {}}
    return {
        "result": payload.get("result"),
        "summary": payload.get("summary", {}),
        "errors": payload.get("errors", []),
        "warnings": payload.get("warnings", []),
        "notes": payload.get("notes", []),
    }


def _evaluate_scene(entry: dict[str, Any], repo_root: Path, ros_humble_available: bool) -> dict[str, Any]:
    scene_name = _resolve_catalog_value(entry, "scene_name", _string(entry.get("package_name")))
    package_name = _resolve_catalog_value(entry, "package_name", scene_name)
    scene_path_rel = _resolve_catalog_value(entry, "scene_path", f"scenes/{scene_name}")
    scene_path = (repo_root / scene_path_rel).resolve()
    build_package_name = _resolve_catalog_value(entry, "build_package_name", package_name)

    categories: dict[str, dict[str, Any]] = {
        "scene_package_exists": _scene_package_exists(scene_path),
    }
    for key, rel_path, optional in REQUIRED_FILES:
        if key == "cmakelists" and not _requires_cmakelists(scene_path):
            categories[key] = _category("PASS", "CMakeLists.txt is not required by this package.xml build type", details={"required": False})
            continue
        categories[key] = _file_category(scene_path, rel_path, optional=optional)

    categories["cell_definition_validation"] = _validate_cell_definition(scene_path)
    categories["manifest_local_file_references"] = _manifest_local_file_references(scene_path)
    categories["scene3d_visual_quality_summary"] = _scene3d_visual_quality(scene_name, scene_path)
    categories["credible_physical_visual_evidence"] = _credible_physical_visual_evidence(scene_path)
    categories["fake_hardware_launch_command_derivation"] = _safe_fake_hardware_launch(entry, package_name, build_package_name)
    categories["ros_launch_smoke_skip_evaluation_state"] = _ros_launch_smoke_state(
        ros_humble_available, categories["fake_hardware_launch_command_derivation"]
    )

    status_counts = Counter(cat["status"] for cat in categories.values())
    if status_counts.get("FAIL", 0):
        overall = "FAIL"
    elif status_counts.get("BLOCKED", 0):
        overall = "BLOCKED"
    else:
        overall = "PASS"

    return {
        "scene_name": scene_name,
        "scene_path": str(scene_path),
        "scene_path_relative": _rel(scene_path, repo_root),
        "package_name": package_name,
        "build_package_name": build_package_name,
        "catalog_status": entry.get("status"),
        "support_level": entry.get("support_level"),
        "known_blocker": entry.get("known_blocker", ""),
        "validation_command": entry.get("validation_command"),
        "build_command": entry.get("build_command"),
        "fake_hardware_launch_command": (categories["fake_hardware_launch_command_derivation"].get("details") or {}).get("recorded_command"),
        "overall_status": overall,
        "totals": dict(status_counts),
        "categories": categories,
        "offline_scene_readiness": _check_scene_readiness_summary(scene_path),
    }


def _build_totals(scenes: list[dict[str, Any]]) -> dict[str, Any]:
    overall = Counter(scene["overall_status"] for scene in scenes)
    category_totals: dict[str, Counter[str]] = {}
    for scene in scenes:
        for name, category in scene.get("categories", {}).items():
            category_totals.setdefault(name, Counter())[category.get("status", "UNKNOWN")] += 1
    return {
        "overall": dict(overall),
        "categories": {name: dict(counts) for name, counts in sorted(category_totals.items())},
    }


def _commands(repo_root: Path, catalog_path: Path, output_dir: Path, scenes: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "matrix": f"python3 scripts/run_workcell_studio_scene_readiness_matrix.py --catalog {_rel(catalog_path, repo_root)} --output-dir {_rel(output_dir, repo_root)}",
        "per_scene_validation_commands": [scene.get("validation_command") for scene in scenes if scene.get("validation_command")],
        "safe_fake_hardware_launch_commands_recorded_not_run": [scene.get("fake_hardware_launch_command") for scene in scenes if scene.get("fake_hardware_launch_command")],
    }


def build_matrix(repo_root: Path, catalog_path: Path, output_dir: Path, workspace_root: Path | None = None) -> dict[str, Any]:
    entries = _scene_entries(catalog_path)
    resolved_workspace = workspace_root or resolve_workspace_root(repo_root)
    ros_humble_available = (Path("/opt/ros/humble/setup.bash").is_file())
    workcell_builder_exe = resolve_workcell_builder_executable(resolved_workspace)
    scenes = [_evaluate_scene(entry, repo_root, ros_humble_available) for entry in entries]
    payload = {
        "schema_version": SCHEMA_VERSION,
        "generated_at": _utc_now(),
        "repo_root": str(repo_root),
        "workspace_root": str(resolved_workspace) if resolved_workspace else None,
        "catalog_path": str(catalog_path),
        "output_dir": str(output_dir),
        "scene_count": len(scenes),
        "totals": _build_totals(scenes),
        "scenes": scenes,
        "commands": _commands(repo_root, catalog_path, output_dir, scenes),
        "ros_humble_available": ros_humble_available,
        "workcell_builder_executable_found": bool(workcell_builder_exe),
        "workcell_builder_executable": str(workcell_builder_exe) if workcell_builder_exe else None,
        "safety": {
            "fake_hardware_only": True,
            "real_robot_motion_invoked": False,
            "ros_launch_invoked": False,
            "note": "Launch commands are derived/recorded only when they include use_fake_hardware:=true and launch_rviz:=true.",
        },
    }
    return payload


def _write_markdown(payload: dict[str, Any], path: Path) -> None:
    lines = [
        "# Workcell Studio Scene Readiness Matrix",
        "",
        f"Schema: `{payload['schema_version']}`",
        f"Generated: `{payload.get('generated_at')}`",
        f"Repo root: `{payload['repo_root']}`",
        f"Workspace root: `{payload.get('workspace_root')}`",
        f"Scene count: **{payload['scene_count']}**",
        f"ROS Humble available: **{payload['ros_humble_available']}**",
        f"workcell_builder executable found: **{payload['workcell_builder_executable_found']}**",
        "",
        "## Safety",
        "",
        "This matrix records safe fake-hardware launch commands only. It does not invoke `ros2 launch`, publish topics/services, or enable real robot motion.",
        "",
        "## Overall scene status",
        "",
        "| Scene | Overall | PASS | FAIL | BLOCKED | Known blocker |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for scene in payload.get("scenes", []):
        totals = scene.get("totals", {})
        known = _string(scene.get("known_blocker")) or "-"
        lines.append(
            f"| `{scene.get('scene_name')}` | **{scene.get('overall_status')}** | {totals.get('PASS', 0)} | {totals.get('FAIL', 0)} | {totals.get('BLOCKED', 0)} | {known} |"
        )
    lines.extend(["", "## Category matrix", ""])
    for scene in payload.get("scenes", []):
        lines.extend([f"### {scene.get('scene_name')}", "", "| Category | Status | Summary |", "|---|---:|---|"])
        for name, category in scene.get("categories", {}).items():
            lines.append(f"| `{name}` | **{category.get('status')}** | {category.get('summary', '').replace('|', '/')} |")
        lines.append("")
    lines.extend(["## Commands", ""])
    lines.append(f"Matrix command: `{payload.get('commands', {}).get('matrix')}`")
    lines.append("")
    lines.append("Safe fake-hardware launch commands recorded but not run:")
    for command in payload.get("commands", {}).get("safe_fake_hardware_launch_commands_recorded_not_run", []):
        lines.append(f"- `{command}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT_DEFAULT)
    parser.add_argument("--workspace-root", type=Path, default=None)
    parser.add_argument("--catalog", type=Path, default=None, help="Defaults to <repo>/scenes/supported_scenes.yaml")
    parser.add_argument("--output-dir", type=Path, default=None, help="Defaults to <repo>/build/workcell_studio_scene_readiness")
    parser.add_argument("--quiet", action="store_true", help="Do not print the JSON payload to stdout")
    parser.add_argument("--fail-on-failures", action="store_true", help="Return non-zero when any scene has FAIL categories; default is report-generation success")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    repo_root = args.repo_root.resolve()
    catalog_path = (args.catalog or (repo_root / "scenes" / "supported_scenes.yaml")).resolve()
    output_dir = (args.output_dir or (repo_root / "build" / "workcell_studio_scene_readiness")).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    payload = build_matrix(
        repo_root=repo_root,
        catalog_path=catalog_path,
        output_dir=output_dir,
        workspace_root=args.workspace_root.resolve() if args.workspace_root else None,
    )

    json_path = output_dir / "scene_readiness_summary.json"
    md_path = output_dir / "scene_readiness_summary.md"
    json_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _write_markdown(payload, md_path)

    if not args.quiet:
        print(json.dumps(payload, indent=2, sort_keys=True))
    overall = payload.get("totals", {}).get("overall", {})
    if args.fail_on_failures and overall.get("FAIL"):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
