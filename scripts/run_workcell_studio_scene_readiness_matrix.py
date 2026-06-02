#!/usr/bin/env python3
"""Build a Workcell Studio supported-scene readiness matrix.

The matrix intentionally reports evidence and blockers without launching real robot
motion.  Launch smoke checks are represented as command records and are safely
skipped when ROS 2 Humble is not available in the current environment.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml  # type: ignore

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from supported_scene_catalog import default_catalog_path, load_supported_scene_catalog

SCHEMA_VERSION = "workcell_studio_scene_readiness_matrix/v1"
PASS = "PASS"
WARN = "WARN"
FAIL = "FAIL"
BLOCKED = "BLOCKED"
SKIP = "SKIP"

CORE_REQUIRED_FILES = (
    "package.xml",
    "CMakeLists.txt",
    "environment.yaml",
    "scene_manifest.yaml",
    "cell_definition.yaml",
    "launch/demo.launch.py",
)
VISUAL_MESH_INDEX = "generated/scene_visual_mesh_index.json"


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _load_yaml(path: Path) -> tuple[Any | None, str | None]:
    if not path.exists():
        return None, f"missing_file: {path.name}"
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")), None
    except Exception as exc:  # noqa: BLE001 - reports are diagnostic by design.
        return None, f"yaml_parse_error: {path.name}: {exc.__class__.__name__}: {exc}"


def _load_json(path: Path) -> tuple[Any | None, str | None]:
    if not path.exists():
        return None, f"missing_file: {path.name}"
    try:
        return json.loads(path.read_text(encoding="utf-8")), None
    except Exception as exc:  # noqa: BLE001 - reports are diagnostic by design.
        return None, f"json_parse_error: {path.name}: {exc.__class__.__name__}: {exc}"


def _status_from_blockers(blockers: list[str]) -> str:
    return PASS if not blockers else BLOCKED


def _dedupe(items: list[str]) -> list[str]:
    seen: set[str] = set()
    out: list[str] = []
    for item in items:
        if item not in seen:
            seen.add(item)
            out.append(item)
    return out


def _required_files_for_entry(entry: Any) -> list[str]:
    required = [*CORE_REQUIRED_FILES, *getattr(entry, "required_files", [])]
    return _dedupe([rel for rel in required if rel])


def _relative_reference_values(value: Any, parent_key: str = "") -> list[tuple[str, str]]:
    """Return likely local file references from a manifest-like structure."""
    refs: list[tuple[str, str]] = []
    if isinstance(value, dict):
        for key, child in value.items():
            child_key = f"{parent_key}.{key}" if parent_key else str(key)
            refs.extend(_relative_reference_values(child, child_key))
        return refs
    if isinstance(value, list):
        for idx, child in enumerate(value):
            refs.extend(_relative_reference_values(child, f"{parent_key}[{idx}]"))
        return refs
    if not isinstance(value, str):
        return refs

    normalized = value.strip()
    if not normalized or "://" in normalized or normalized.startswith(("package://", "$(", "${", "/")):
        return refs
    key = parent_key.lower()
    suffixes = ("path", "file", "files", "reference", "references", "urdf", "xacro", "launch", "layout", "environment")
    looks_like_file = "/" in normalized or "." in Path(normalized).name
    if looks_like_file and (key.endswith(suffixes) or any(part in key for part in ("files", "assets", "generated_assets"))):
        refs.append((parent_key, normalized))
    return refs


def _manifest_reference_blockers(scene_dir: Path) -> list[str]:
    manifest, error = _load_yaml(scene_dir / "scene_manifest.yaml")
    if error:
        return [f"manifest_reference_failure: {error}"]
    refs = _relative_reference_values(manifest)
    blockers: list[str] = []
    for key, rel in refs:
        if rel.startswith("../") or "/../" in rel:
            blockers.append(f"manifest_reference_failure: {key} -> {rel} escapes scene directory")
            continue
        if not (scene_dir / rel).exists():
            blockers.append(f"manifest_reference_failure: {key} -> {rel} missing")
    return blockers


def _cell_definition_blockers(scene_dir: Path) -> list[str]:
    cell_path = scene_dir / "cell_definition.yaml"
    data, error = _load_yaml(cell_path)
    if error:
        return [f"schema_validation_blocker: cell_definition.yaml: {error}"]
    if not isinstance(data, dict):
        return ["schema_validation_blocker: cell_definition.yaml root must be a YAML map"]
    # Synthetic fixtures and legacy generated scenes use a few different shapes;
    # require enough identity to prove this is not an empty placeholder.
    identity_keys = {"robot", "robot_model", "workcell", "cell", "metadata", "scene"}
    if not any(key in data for key in identity_keys):
        return ["schema_validation_blocker: cell_definition.yaml missing robot/workcell identity"]
    return []


def _visual_status(scene_dir: Path) -> tuple[str, list[str], dict[str, Any]]:
    index_path = scene_dir / VISUAL_MESH_INDEX
    payload, error = _load_json(index_path)
    if error:
        return BLOCKED, [f"visual_evidence_blocked: missing_file: {VISUAL_MESH_INDEX}"], {"path": str(index_path)}
    if not isinstance(payload, dict):
        return BLOCKED, [f"visual_evidence_blocked: invalid_json_shape: {VISUAL_MESH_INDEX} root must be an object"], {"path": str(index_path)}

    visual_quality_status = str(
        payload.get("visual_quality_status")
        or payload.get("visual_status")
        or payload.get("status")
        or PASS
    ).upper()
    if visual_quality_status in {BLOCKED, FAIL}:
        reasons = payload.get("blocker_reasons") or payload.get("blockers") or ["visual quality report blocked scene"]
        if not isinstance(reasons, list):
            reasons = [str(reasons)]
        return BLOCKED, [f"visual_quality_blocked: {reason}" for reason in reasons], {"path": str(index_path), "payload_status": visual_quality_status}

    items = payload.get("visual_items") if isinstance(payload.get("visual_items"), list) else payload.get("items")
    if not isinstance(items, list) or not items:
        return BLOCKED, [f"visual_evidence_blocked: {VISUAL_MESH_INDEX} has no visual_items/items evidence"], {"path": str(index_path)}
    return PASS, [], {"path": str(index_path), "visual_item_count": len(items), "payload_status": visual_quality_status}


def ros_humble_available() -> bool:
    return Path("/opt/ros/humble/setup.bash").exists() and shutil.which("ros2") is not None


def derive_fake_hardware_launch_command(entry: Any) -> str:
    command = str(getattr(entry, "fake_hardware_launch_command", "") or "").strip()
    package_name = str(getattr(entry, "package_name", "") or getattr(entry, "scene_name", "")).strip()
    if not command:
        command = f"ros2 launch {package_name} demo.launch.py use_fake_hardware:=true launch_rviz:=true"
    if "use_fake_hardware:=" not in command:
        command = f"{command} use_fake_hardware:=true"
    elif "use_fake_hardware:=true" not in command:
        command = command.replace("use_fake_hardware:=false", "use_fake_hardware:=true")
    return command


def _launch_command_record(entry: Any, *, ros_available: bool | None = None) -> tuple[str, dict[str, Any], list[str]]:
    command = derive_fake_hardware_launch_command(entry)
    safety_blockers: list[str] = []
    if "use_fake_hardware:=true" not in command:
        safety_blockers.append("unsafe_launch_command: missing use_fake_hardware:=true")
    if "use_fake_hardware:=false" in command or "real_hardware:=true" in command:
        safety_blockers.append("unsafe_launch_command: command appears to enable real hardware")

    available = ros_humble_available() if ros_available is None else ros_available
    if not available:
        return SKIP, {
            "name": "fake_hardware_launch_smoke",
            "command": command,
            "status": SKIP,
            "executed": False,
            "safely_skipped": True,
            "reason": "ROS Humble unavailable in this environment; launch smoke was not executed and is not treated as a scene failure.",
        }, safety_blockers

    return PASS if not safety_blockers else BLOCKED, {
        "name": "fake_hardware_launch_smoke",
        "command": command,
        "status": "READY_TO_RUN" if not safety_blockers else BLOCKED,
        "executed": False,
        "safely_skipped": True,
        "reason": "Launch command derived only; no robot motion commanded by readiness matrix.",
    }, safety_blockers


def recommended_next_action(category_statuses: dict[str, str], blockers: list[str]) -> str:
    if any("missing_file:" in blocker for blocker in blockers):
        return "Regenerate the scene package or restore the missing required file, then rerun the readiness matrix."
    if category_statuses.get("manifest_references") == BLOCKED:
        return "Fix scene_manifest.yaml local file references so every referenced scene-local artifact exists."
    if category_statuses.get("visual_evidence") == BLOCKED or category_statuses.get("visual_quality") == BLOCKED:
        return "Regenerate generated/scene_visual_mesh_index.json and rerun visual-quality validation."
    if category_statuses.get("launch_smoke") == SKIP:
        return "Install/source ROS 2 Humble in a ROS workspace to run the fake-hardware launch smoke check."
    return "Validate / Plan & Simulate using fake hardware."


def evaluate_scene(repo_root: Path, entry: Any, *, ros_available: bool | None = None) -> dict[str, Any]:
    scene_dir = repo_root / getattr(entry, "scene_path", f"scenes/{getattr(entry, 'scene_name', '')}")
    file_blockers = [f"missing_file: {rel}" for rel in _required_files_for_entry(entry) if not (scene_dir / rel).exists()]
    schema_blockers = _cell_definition_blockers(scene_dir)
    manifest_blockers = _manifest_reference_blockers(scene_dir)
    visual_overall_status, visual_blockers, visual_evidence = _visual_status(scene_dir)
    launch_status, launch_record, safety_blockers = _launch_command_record(entry, ros_available=ros_available)

    category_statuses = {
        "file_presence": _status_from_blockers(file_blockers),
        "schema_validation": _status_from_blockers(schema_blockers),
        "manifest_references": _status_from_blockers(manifest_blockers),
        "visual_evidence": PASS if visual_overall_status == PASS else BLOCKED,
        "visual_quality": visual_overall_status,
        "launch_smoke": launch_status,
        "safety": _status_from_blockers(safety_blockers),
    }
    blocker_reasons = [*file_blockers, *schema_blockers, *manifest_blockers, *visual_blockers, *safety_blockers]
    overall_status = PASS if not blocker_reasons else BLOCKED
    if overall_status == PASS and launch_status == SKIP:
        # Environment-only launch skips should not turn an otherwise healthy scene into a failure.
        overall_status = PASS

    return {
        "scene_name": getattr(entry, "scene_name", scene_dir.name),
        "package_name": getattr(entry, "package_name", scene_dir.name),
        "scene_path": str(scene_dir),
        "support_level": getattr(entry, "support_level", "supported"),
        "catalog_status": getattr(entry, "status", "supported"),
        "known_blocker": getattr(entry, "known_blocker", ""),
        "overall_status": overall_status,
        "category_statuses": category_statuses,
        "blocker_reasons": blocker_reasons,
        "recommended_next_action": recommended_next_action(category_statuses, blocker_reasons),
        "command_records": [launch_record],
        "visual_evidence": visual_evidence,
    }


def build_readiness_matrix(repo_root: Path, *, catalog_path: Path | None = None, ros_available: bool | None = None) -> dict[str, Any]:
    repo_root = repo_root.resolve()
    catalog_path = catalog_path or default_catalog_path(repo_root)
    _catalog, entries, catalog_errors = load_supported_scene_catalog(catalog_path)
    enabled_entries = [entry for entry in entries if entry.enabled]
    scenes = [evaluate_scene(repo_root, entry, ros_available=ros_available) for entry in enabled_entries]
    totals = {PASS: 0, WARN: 0, FAIL: 0, BLOCKED: 0, SKIP: 0}
    for scene in scenes:
        totals[scene["overall_status"]] = totals.get(scene["overall_status"], 0) + 1
    command_records = [record for scene in scenes for record in scene.get("command_records", [])]
    overall_status = PASS if totals.get(BLOCKED, 0) == 0 and not catalog_errors else BLOCKED
    return {
        "schema_version": SCHEMA_VERSION,
        "generated_at_utc": _utc_now(),
        "repo_root": str(repo_root),
        "catalog_path": str(catalog_path),
        "overall_status": overall_status,
        "scene_count": len(scenes),
        "totals": totals,
        "catalog_errors": catalog_errors,
        "scenes": scenes,
        "command_records": command_records,
    }


def write_readiness_matrix(report: dict[str, Any], output: Path) -> Path:
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build Workcell Studio supported-scene readiness matrix JSON.")
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--catalog", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=Path("build/workcell_studio/scene_readiness_matrix.json"))
    parser.add_argument("--json", action="store_true", help="Print the JSON report to stdout")
    args = parser.parse_args(argv)

    output = args.output
    if not output.is_absolute():
        output = args.repo_root / output
    report = build_readiness_matrix(args.repo_root, catalog_path=args.catalog)
    write_readiness_matrix(report, output)
    if args.json:
        print(json.dumps(report, indent=2))
    return 0 if report["overall_status"] == PASS else 1


if __name__ == "__main__":
    raise SystemExit(main())
