#!/usr/bin/env python3
"""Build an offline Workcell Studio scene readiness matrix from the supported-scene catalog.

The matrix is intentionally fake-hardware-first: it derives or records only safe
`ros2 launch ... use_fake_hardware:=true ...` commands and does not execute ROS
launches or publish to any robot/control topics.
"""

from __future__ import annotations

import argparse
import inspect
import json
import os
import shlex
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml  # type: ignore

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT_DEFAULT = SCRIPT_DIR.parents[0]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(REPO_ROOT_DEFAULT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT_DEFAULT))

from check_scene_readiness import check_readiness  # noqa: E402
from supported_scene_catalog import default_catalog_path, load_supported_scene_catalog  # noqa: E402
from validate_builder_generated_scene import validate_scene as validate_builder_scene  # noqa: E402
from validate_cell_definition import load_yaml as load_cell_yaml  # noqa: E402
from validate_cell_definition import validate_cell_definition  # noqa: E402
from validate_scene3d_visual_quality_matrix import evaluate_scene as evaluate_scene3d_visual_quality  # noqa: E402
from workcell_studio_path_resolver import describe_resolution, resolve_workspace_root, resolve_workcell_builder_executable  # noqa: E402

SCHEMA_VERSION = "workcell_studio_scene_readiness_matrix/v1"
PASS = "PASS"
FAIL = "FAIL"
BLOCKED = "BLOCKED"
VALID_STATES = {PASS, FAIL, BLOCKED}

REQUIRED_CATEGORY_FILES: tuple[tuple[str, str, bool], ...] = (
    ("package_xml", "package.xml", True),
    ("cmakelists_txt", "CMakeLists.txt", True),
    ("environment_yaml", "environment.yaml", True),
    ("cell_definition_yaml", "cell_definition.yaml", True),
    ("scene_manifest_yaml", "scene_manifest.yaml", True),
    ("layout_workcell_studio_layout_yaml", "layout/workcell_studio_layout.yaml", True),
    ("launch_demo_launch_py", "launch/demo.launch.py", True),
    ("urdf_scene_urdf_xacro", "urdf/scene.urdf.xacro", True),
    ("generated_scene_visual_mesh_index_json", "generated/scene_visual_mesh_index.json", True),
    ("generated_scene_package_readiness_json", "generated/scene_package_readiness.json", False),
)
LOCAL_REF_KEY_HINTS = ("path", "file", "uri", "xacro", "urdf", "mesh", "launch", "config", "layout", "manifest")
LOCAL_REF_EXTENSIONS = (
    ".yaml",
    ".yml",
    ".json",
    ".xml",
    ".xacro",
    ".urdf",
    ".srdf",
    ".rviz",
    ".py",
    ".stl",
    ".dae",
    ".obj",
)


def _result(status: str, message: str, **extra: Any) -> dict[str, Any]:
    if status not in VALID_STATES:
        raise ValueError(f"invalid readiness status: {status}")
    payload = {"status": status, "message": message}
    payload.update(extra)
    return payload


def _load_yaml_file(path: Path) -> tuple[Any | None, str | None]:
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")), None
    except Exception as exc:  # noqa: BLE001
        return None, f"{exc.__class__.__name__}: {exc}"


def _load_json_file(path: Path) -> tuple[Any | None, str | None]:
    try:
        return json.loads(path.read_text(encoding="utf-8")), None
    except Exception as exc:  # noqa: BLE001
        return None, f"{exc.__class__.__name__}: {exc}"


def _check_file(scene_dir: Path, rel_path: str, *, required: bool) -> dict[str, Any]:
    path = scene_dir / rel_path
    if path.is_file():
        return _result(PASS, f"{rel_path} exists", path=str(path))
    if required:
        return _result(FAIL, f"missing required file: {rel_path}", path=str(path))
    return _result(PASS, f"optional/generated-if-present file is not present: {rel_path}", path=str(path), optional=True)


def _safe_launch_command_from_catalog(command: str, package_name: str) -> tuple[str, list[str]]:
    """Return a safe fake-hardware launch command and warnings.

    Catalog commands are accepted only when they are a ROS launch invocation for
    demo.launch.py and explicitly set use_fake_hardware:=true. Unsafe or missing
    commands are replaced with a derived fake-hardware command from package_name.
    """
    derived = f"ros2 launch {package_name} demo.launch.py use_fake_hardware:=true launch_rviz:=true"
    warnings: list[str] = []
    raw = command.strip()
    if not raw:
        return derived, ["catalog fake_hardware_launch_command missing; derived safe fake-hardware command"]

    try:
        tokens = shlex.split(raw)
    except ValueError as exc:
        return derived, [f"catalog fake_hardware_launch_command could not be parsed ({exc}); derived safe fake-hardware command"]

    unsafe_reasons: list[str] = []
    if len(tokens) < 4 or tokens[0] != "ros2" or tokens[1] != "launch":
        unsafe_reasons.append("command is not a ros2 launch invocation")
    if "demo.launch.py" not in tokens:
        unsafe_reasons.append("command does not launch demo.launch.py")
    if "use_fake_hardware:=true" not in tokens:
        unsafe_reasons.append("command does not explicitly set use_fake_hardware:=true")
    if any(tok == "use_fake_hardware:=false" for tok in tokens):
        unsafe_reasons.append("command explicitly disables fake hardware")
    if any(tok.startswith(("real_hardware:=true", "use_real_hardware:=true", "send_to_robot:=true")) for tok in tokens):
        unsafe_reasons.append("command includes real-hardware/send-to-robot flag")
    if any(tok in {";", "&&", "||", "|", ">", "<"} for tok in tokens):
        unsafe_reasons.append("command includes shell control/redirection token")

    if unsafe_reasons:
        return derived, ["catalog fake_hardware_launch_command rejected as unsafe: " + "; ".join(unsafe_reasons)]

    if "launch_rviz:=true" not in tokens:
        warnings.append("catalog command is safe but does not include launch_rviz:=true")
    return raw, warnings


def _check_fake_hardware_launch(entry: Any, launch_exists: bool) -> tuple[dict[str, Any], str, list[str]]:
    package_name = entry.package_name or entry.build_package_name or entry.scene_name
    command, warnings = _safe_launch_command_from_catalog(entry.fake_hardware_launch_command, package_name)
    status = PASS if launch_exists else FAIL
    message = "safe fake-hardware launch command recorded" if launch_exists else "cannot derive runnable launch command because launch/demo.launch.py is missing"
    return (
        _result(
            status,
            message,
            command=command,
            package_name=package_name,
            build_package_name=entry.build_package_name,
            warnings=warnings,
            safety="not executed; fake hardware explicitly required",
        ),
        command,
        warnings,
    )


def _iter_manifest_refs(value: Any, *, key_path: str = "") -> list[tuple[str, str]]:
    refs: list[tuple[str, str]] = []
    if isinstance(value, dict):
        for key, child in value.items():
            child_key = str(key)
            child_path = f"{key_path}.{child_key}" if key_path else child_key
            refs.extend(_iter_manifest_refs(child, key_path=child_path))
    elif isinstance(value, list):
        for idx, child in enumerate(value):
            refs.extend(_iter_manifest_refs(child, key_path=f"{key_path}[{idx}]"))
    elif isinstance(value, str):
        stripped = value.strip()
        lowered_key = key_path.lower()
        lowered_value = stripped.lower()
        if not stripped:
            return refs
        if stripped.startswith(("package://", "http://", "https://", "model://")):
            return refs
        looks_fileish = lowered_value.endswith(LOCAL_REF_EXTENSIONS) or any(hint in lowered_key for hint in LOCAL_REF_KEY_HINTS)
        if looks_fileish and not any(ch in stripped for ch in "\n\r"):
            refs.append((key_path, stripped))
    return refs


def _check_manifest_refs(scene_dir: Path) -> dict[str, Any]:
    manifest = scene_dir / "scene_manifest.yaml"
    if not manifest.is_file():
        return _result(BLOCKED, "scene_manifest.yaml is missing; local-file references cannot be checked")
    payload, error = _load_yaml_file(manifest)
    if error:
        return _result(FAIL, f"scene_manifest.yaml is not parseable: {error}")
    refs = _iter_manifest_refs(payload)
    missing: list[dict[str, str]] = []
    checked: list[dict[str, str]] = []
    for key_path, ref in refs:
        # Ignore package names and launch arguments that do not look like relative files.
        if ref.startswith("$") or ":=" in ref or ref in {"true", "false"}:
            continue
        ref_path = Path(ref)
        if ref_path.is_absolute():
            missing.append({"field": key_path, "reference": ref, "reason": "absolute path is not a local scene-relative file"})
            continue
        candidate = (scene_dir / ref_path).resolve()
        checked.append({"field": key_path, "reference": ref, "path": str(candidate)})
        if not candidate.exists():
            missing.append({"field": key_path, "reference": ref, "reason": "referenced file does not exist"})
    if missing:
        return _result(FAIL, f"{len(missing)} manifest local-file reference(s) are missing or unsafe", checked=checked, missing=missing)
    return _result(PASS, f"manifest local-file references resolved ({len(checked)} checked)", checked=checked)


def _check_tool_metadata_consistency(scene_dir: Path) -> dict[str, Any]:
    """Check that scene-local tool declarations do not contradict each other.

    This is intentionally metadata-only and does not resolve ROS packages.  It
    preserves known blocked/failing states such as a scene folder declaring one
    end effector in cell_definition.yaml while environment.yaml or
    scene_manifest.yaml advertises another/no tool.
    """
    observed: dict[str, Any] = {}
    errors: list[str] = []

    cell_path = scene_dir / "cell_definition.yaml"
    env_path = scene_dir / "environment.yaml"
    manifest_path = scene_dir / "scene_manifest.yaml"

    cell, cell_error = _load_yaml_file(cell_path) if cell_path.is_file() else ({}, None)
    env, env_error = _load_yaml_file(env_path) if env_path.is_file() else ({}, None)
    manifest, manifest_error = _load_yaml_file(manifest_path) if manifest_path.is_file() else ({}, None)

    for label, error in (("cell_definition.yaml", cell_error), ("environment.yaml", env_error), ("scene_manifest.yaml", manifest_error)):
        if error:
            errors.append(f"{label} is not parseable for tool consistency check: {error}")

    cell_ee = cell.get("end_effector") if isinstance(cell, dict) and isinstance(cell.get("end_effector"), dict) else {}
    env_tool = env.get("tool") if isinstance(env, dict) and isinstance(env.get("tool"), dict) else {}
    manifest_ee = manifest.get("end_effector") if isinstance(manifest, dict) and isinstance(manifest.get("end_effector"), dict) else {}

    cell_ee_id = str(cell_ee.get("id") or cell_ee.get("capability") or "").strip()
    env_tool_id = str(env_tool.get("id") or env_tool.get("profile") or "").strip()
    manifest_ee_type = str(manifest_ee.get("type") or "").strip()
    manifest_ee_brand = str(manifest_ee.get("brand") or "").strip()

    def family(value: str) -> str | None:
        lowered = value.lower()
        if "3f" in lowered or "three" in lowered:
            return "three_finger_gripper"
        if "2f" in lowered or "two" in lowered:
            return "two_finger_gripper"
        if "suction" in lowered or "vacuum" in lowered or "airpick" in lowered:
            return "suction"
        if "finger" in lowered or "gripper" in lowered:
            return "finger_gripper"
        if lowered == "none":
            return "none"
        return None

    cell_family = family(cell_ee_id)
    env_family = family(env_tool_id)
    manifest_family = family(manifest_ee_type)
    warnings: list[str] = []
    observed = {
        "cell_definition_end_effector_id": cell_ee_id or None,
        "environment_tool_id": env_tool_id or None,
        "manifest_end_effector_type": manifest_ee_type or None,
        "manifest_end_effector_brand": manifest_ee_brand or None,
        "cell_definition_family": cell_family,
        "environment_tool_family": env_family,
        "manifest_end_effector_family": manifest_family,
    }

    if cell_ee_id and env_tool_id and cell_ee_id != env_tool_id:
        if cell_family and env_family and cell_family != env_family:
            errors.append(
                "Tool metadata mismatch: cell_definition.yaml end_effector.id "
                f"'{cell_ee_id}' resolves to '{cell_family}' but environment.yaml tool.id/profile "
                f"'{env_tool_id}' resolves to '{env_family}'."
            )
        else:
            warnings.append(
                "Tool metadata names differ but resolve to the same broad family: "
                f"cell_definition.yaml end_effector.id '{cell_ee_id}', environment.yaml tool.id/profile '{env_tool_id}'."
            )
    if manifest_family == "none" and (cell_ee_id or env_tool_id):
        errors.append(
            "Tool metadata mismatch: scene_manifest.yaml end_effector.type is 'none' "
            "while scene-local tool/end_effector metadata declares a tool."
        )

    if errors:
        return _result(FAIL, "tool/end-effector metadata is inconsistent", observed=observed, errors=errors, warnings=warnings)
    return _result(PASS, "tool/end-effector metadata has no explicit contradictions", observed=observed, warnings=warnings)


def _check_cell_definition(scene_dir: Path) -> dict[str, Any]:
    path = scene_dir / "cell_definition.yaml"
    if not path.is_file():
        return _result(BLOCKED, "cell_definition.yaml is missing; validation cannot run")
    try:
        loaded, parser_name, notes = load_cell_yaml(path)
        summary = validate_cell_definition(loaded, path, parser_name, notes)
    except Exception as exc:  # noqa: BLE001
        return _result(FAIL, f"cell_definition.yaml validation failed to run: {exc.__class__.__name__}: {exc}")
    status = PASS if summary.ok else FAIL
    return _result(
        status,
        "cell_definition.yaml validates" if status == PASS else "cell_definition.yaml has validation errors",
        parser=summary.parser,
        errors=summary.errors,
        warnings=summary.warnings,
        notes=summary.notes,
        capabilities=summary.capability_summary,
        environment_layout=summary.environment_layout_summary,
        grasp_strategy=summary.grasp_strategy_summary,
    )


def _extract_scene3d_smoke_evidence(smoke_json: Path) -> dict[str, Any]:
    payload: dict[str, Any] = {}
    smoke_error: str | None = None
    if smoke_json.is_file():
        loaded, smoke_error = _load_json_file(smoke_json)
        if isinstance(loaded, dict):
            payload = loaded

    def nested_value(*keys: str) -> Any:
        sources = [payload]
        for nested_key in ("result", "render_debug_counters", "static_scene3d_visual_evidence"):
            nested = payload.get(nested_key)
            if isinstance(nested, dict):
                sources.append(nested)
        for source in sources:
            for key in keys:
                if key in source and source.get(key) is not None:
                    return source.get(key)
        return None

    def optional_bool(value: Any) -> bool | None:
        if value is None:
            return None
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"true", "1", "yes", "y"}:
                return True
            if normalized in {"false", "0", "no", "n"}:
                return False
        return bool(value)

    runtime_available = optional_bool(nested_value("runtime_available"))
    screenshot_available = optional_bool(nested_value("screenshot_available"))
    default_status = "INVALID" if smoke_error else "MISSING" if not smoke_json.is_file() else "UNKNOWN"
    smoke_status = str(nested_value("status") or default_status).upper()
    resolved_executable = nested_value("resolved_executable", "executable")
    searched_paths_raw = nested_value("searched_paths")
    searched_paths = [str(item) for item in searched_paths_raw] if isinstance(searched_paths_raw, list) else []
    if runtime_available is None and smoke_json.is_file():
        blockers = nested_value("blockers")
        blocker_text = " ".join(str(item) for item in blockers) if isinstance(blockers, list) else str(blockers or "")
        if resolved_executable:
            runtime_available = True
        elif searched_paths or "unable_to_resolve_workcell_builder_executable" in blocker_text:
            runtime_available = False
    return {
        "smoke_status": smoke_status,
        "runtime_available": runtime_available,
        "screenshot_available": screenshot_available,
        "resolved_executable": str(resolved_executable) if resolved_executable else None,
        "searched_paths": searched_paths,
        "smoke_load_error": smoke_error,
    }
def _smoke_indicates_runtime_screenshot_evidence(smoke_json: Path) -> bool:
    if not smoke_json.is_file():
        return False
    payload, error = _load_json_file(smoke_json)
    if error or not isinstance(payload, dict):
        return False
    if bool(payload.get("screenshot_available")):
        return True
    if str(payload.get("screenshot_path") or "").strip():
        return True
    for key in ("render_debug_counters", "counters", "static_scene3d_visual_evidence"):
        nested = payload.get(key)
        if isinstance(nested, dict) and bool(nested.get("runtime_available")):
            return True
    return bool(payload.get("runtime_available"))


def _check_scene3d(scene_name: str, scene_dir: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    mesh_index = scene_dir / "generated" / "scene_visual_mesh_index.json"
    smoke_json = scene_dir / "generated" / "scene3d_gui_smoke.json"
    smoke_evidence = _extract_scene3d_smoke_evidence(smoke_json)
    screenshot_path = (
        scene_dir / "generated" / "scene3d_gui_smoke.png"
        if smoke_evidence["smoke_status"] != BLOCKED and _smoke_indicates_runtime_screenshot_evidence(smoke_json)
        else None
    )
    visual = evaluate_scene3d_visual_quality(
        scene_name=scene_name,
        scene_dir=scene_dir,
        mesh_index_path=mesh_index,
        smoke_json_path=smoke_json,
        screenshot_path=screenshot_path,
    )
    blockers = [str(item) for item in visual.get("blockers", [])]
    visual_blocker_reasons = [str(item) for item in visual.get("blocker_reasons", [])]
    physical_blockers = [blocker for blocker in blockers if not blocker.startswith("screenshot_missing:") and blocker != "screenshot_missing"]
    warnings = [str(item) for item in visual.get("warnings", [])]
    visual_status = str(visual.get("visual_quality_status") or "").upper()
    runtime_blocked = smoke_evidence["smoke_status"] == BLOCKED or smoke_evidence["runtime_available"] is False
    runtime_available = bool(visual.get("runtime_available"))
    summary_state = PASS if visual_status == PASS else FAIL
    if visual_status == BLOCKED or not mesh_index.is_file() or not smoke_json.is_file() or runtime_blocked or not runtime_available:
        summary_state = BLOCKED
    if summary_state == PASS:
        summary_message = "Scene3D visual-quality evidence passes"
    elif runtime_blocked:
        summary_message = "Scene3D runtime GUI evidence is blocked or unavailable; visual evidence cannot be evaluated as a failure"
    else:
        summary_message = "Scene3D visual-quality evidence is blocked or failing"
    smoke_context = dict(smoke_evidence)
    smoke_context.pop("runtime_available", None)
    visual_result = _result(
        summary_state,
        summary_message,
        visual_quality_status=visual.get("visual_quality_status"),
        mesh_index_path=str(mesh_index),
        smoke_json=str(smoke_json),
        screenshot_path=str(screenshot_path) if screenshot_path is not None else None,
        blockers=blockers,
        blocker_reasons=visual_blocker_reasons,
        warnings=warnings,
        runtime_available=runtime_available,
        counters={
            "total_payload_count": visual.get("total_payload_count", 0),
            "mesh_source_count": visual.get("mesh_source_count", 0),
            "primitive_source_count": visual.get("primitive_source_count", 0),
            "mesh_rendered_count": visual.get("mesh_rendered_count", 0),
            "primitive_rendered_count": visual.get("primitive_rendered_count", 0),
            "physical_rendered_count": visual.get("physical_rendered_count", 0),
            "runtime_available": runtime_available,
            "credible_physical_rendered_count": visual.get("credible_physical_rendered_count", 0),
            "helper_overlay_count": visual.get("helper_overlay_count", 0),
            "diagnostic_fallback_count": visual.get("diagnostic_fallback_count", 0),
        },
        **smoke_context,
    )

    source_count = int(visual.get("mesh_source_count") or 0) + int(visual.get("primitive_source_count") or 0)
    physical_rendered = int(visual.get("physical_rendered_count") or 0)
    if not mesh_index.is_file():
        physical_state = BLOCKED
        physical_message = "mesh-index evidence is missing; physical visual evidence cannot be evaluated"
    elif not smoke_json.is_file():
        physical_state = BLOCKED
        physical_message = "Scene3D GUI smoke evidence is missing; physical rendered evidence cannot be evaluated"
    elif runtime_blocked:
        physical_state = BLOCKED
        physical_message = "Scene3D runtime GUI evidence is blocked or unavailable; physical rendered evidence cannot be evaluated as a failure"
    elif not runtime_available:
        physical_state = BLOCKED
        physical_message = "Scene3D runtime is unavailable; physical rendered evidence is not evaluated from static renderability counts"
    elif source_count <= 0:
        physical_state = FAIL
        physical_message = "mesh index does not contain credible mesh or primitive source geometry"
    elif physical_rendered <= 0:
        physical_state = FAIL
        physical_message = "no credible physical mesh/primitive render evidence was recorded"
    elif physical_blockers:
        physical_state = FAIL
        physical_message = "physical render evidence exists but physical visual-quality blockers remain"
    else:
        physical_state = PASS
        physical_message = "credible physical mesh/primitive visual evidence is present"
    physical_result = _result(
        physical_state,
        physical_message,
        source_geometry_count=source_count,
        physical_rendered_count=physical_rendered,
        runtime_available=runtime_available,
        mesh_failure_summary_by_reason_code=visual.get("mesh_failure_summary_by_reason_code", {}),
        **smoke_context,
        blockers=physical_blockers,
    )
    return visual_result, physical_result


def _check_readiness_json(scene_dir: Path) -> dict[str, Any]:
    path = scene_dir / "generated" / "scene_package_readiness.json"
    if not path.is_file():
        return _result(PASS, "optional/generated-if-present readiness JSON is not present", optional=True, path=str(path))
    payload, error = _load_json_file(path)
    if error:
        return _result(FAIL, f"generated/scene_package_readiness.json is not valid JSON: {error}", path=str(path))
    return _result(PASS, "generated/scene_package_readiness.json is present and parseable", path=str(path), summary=payload)


def _ros_humble_available() -> bool:
    return Path("/opt/ros/humble/setup.bash").is_file() or os.environ.get("ROS_DISTRO") == "humble"


def _resolve_workcell_builder_executable_evidence(workspace_root: Path | None) -> tuple[bool, dict[str, Any]]:
    executable = resolve_workcell_builder_executable(workspace_root)
    evidence = describe_resolution()
    return executable is not None, evidence


def _ros_launch_smoke_state(ros_available: bool, launch_check: dict[str, Any]) -> dict[str, Any]:
    if launch_check.get("status") != PASS:
        return _result(BLOCKED, "ROS launch smoke not evaluated because no safe launch command is available")
    if not ros_available:
        return _result(BLOCKED, "ROS Humble is not available in this environment; safe fake-hardware launch command recorded but not executed", command=launch_check.get("command"))
    return _result(BLOCKED, "offline readiness matrix does not execute ros2 launch; run the recorded safe fake-hardware command in a sourced ROS workspace", command=launch_check.get("command"))


def _overall_status(categories: dict[str, dict[str, Any]], catalog_status: str, known_blocker: str) -> str:
    if catalog_status == "blocked" and known_blocker:
        return BLOCKED
    states = [str(item.get("status")) for item in categories.values()]
    if FAIL in states:
        return FAIL
    if BLOCKED in states:
        return BLOCKED
    return PASS


def _evaluate_scene(repo_root: Path, entry: Any, ros_available: bool) -> dict[str, Any]:
    scene_dir = (repo_root / entry.scene_path).resolve()
    scene_exists = scene_dir.is_dir()
    categories: dict[str, dict[str, Any]] = {
        "scene_package_exists": _result(PASS if scene_exists else FAIL, "scene package directory exists" if scene_exists else "scene package directory is missing", path=str(scene_dir)),
    }

    for key, rel_path, required in REQUIRED_CATEGORY_FILES:
        categories[key] = _check_readiness_json(scene_dir) if key == "generated_scene_package_readiness_json" else _check_file(scene_dir, rel_path, required=required)

    categories["cell_definition_validation"] = _check_cell_definition(scene_dir)
    categories["tool_metadata_consistency"] = _check_tool_metadata_consistency(scene_dir)
    categories["manifest_local_file_references"] = _check_manifest_refs(scene_dir)

    scene3d_summary, physical_visual = _check_scene3d(entry.scene_name, scene_dir)
    categories["scene3d_visual_quality_summary"] = scene3d_summary
    categories["credible_physical_visual_evidence"] = physical_visual

    launch_check, launch_command, launch_warnings = _check_fake_hardware_launch(entry, (scene_dir / "launch" / "demo.launch.py").is_file())
    categories["fake_hardware_launch_command_derivation"] = launch_check
    categories["ros_launch_smoke_skip_evaluation_state"] = _ros_launch_smoke_state(ros_available, launch_check)

    builder_report: dict[str, Any] | None = None
    readiness_report: dict[str, Any] | None = None
    if scene_exists:
        try:
            builder_report = validate_builder_scene(scene_dir)
        except Exception as exc:  # noqa: BLE001
            builder_report = {"ok": False, "errors": [f"validate_builder_generated_scene failed: {exc.__class__.__name__}: {exc}"]}
        try:
            readiness_report = check_readiness(None, scene_dir, strict=False)
        except Exception as exc:  # noqa: BLE001
            readiness_report = {"result": "FAIL", "errors": [f"check_scene_readiness failed: {exc.__class__.__name__}: {exc}"]}

    overall = _overall_status(categories, entry.status, entry.known_blocker)
    return {
        "scene_name": entry.scene_name,
        "scene_path": str(scene_dir),
        "package_name": entry.package_name,
        "build_package_name": entry.build_package_name,
        "support_level": entry.support_level,
        "catalog_status": entry.status,
        "known_blocker": entry.known_blocker,
        "overall_status": overall,
        "categories": categories,
        "commands": {
            "validation_command": entry.validation_command,
            "build_command": entry.build_command,
            "fake_hardware_launch_command": launch_command,
        },
        "safe_command_warnings": launch_warnings,
        "builder_generated_scene_validation": builder_report,
        "offline_scene_readiness": readiness_report,
    }


def _write_markdown(payload: dict[str, Any], path: Path) -> None:
    lines = [
        "# Workcell Studio Scene Readiness Matrix",
        "",
        f"- schema_version: `{payload['schema_version']}`",
        f"- generated_at: `{payload['generated_at']}`",
        f"- repo_root: `{payload['repo_root']}`",
        f"- workspace_root: `{payload['workspace_root']}`",
        f"- scene_count: `{payload['scene_count']}`",
        f"- ros_humble_available: `{payload['ros_humble_available']}`",
        f"- workcell_builder_executable_found: `{payload['workcell_builder_executable_found']}`",
        "",
        "## Totals",
        "",
        "| Status | Count |",
        "|---|---:|",
    ]
    for status in (PASS, FAIL, BLOCKED):
        lines.append(f"| {status} | {payload['totals'].get(status, 0)} |")
    lines.extend(["", "## Scenes", "", "| Scene | Catalog | Overall | Package | Fake-hardware command | First blocker/failure |", "|---|---|---|---|---|---|"])
    for scene in payload["scenes"]:
        first_problem = ""
        for category in scene["categories"].values():
            if category.get("status") in {FAIL, BLOCKED}:
                first_problem = str(category.get("message", ""))
                break
        if not first_problem:
            first_problem = "-"
        command = scene["commands"].get("fake_hardware_launch_command", "")
        lines.append(
            f"| {scene['scene_name']} | {scene['catalog_status']} | {scene['overall_status']} | {scene['package_name']} | `{command}` | {first_problem} |"
        )
    lines.extend(["", "## Category Matrix", ""])
    for scene in payload["scenes"]:
        lines.extend([f"### {scene['scene_name']}", "", "| Category | Status | Message |", "|---|---|---|"])
        for category_name, result in scene["categories"].items():
            lines.append(f"| {category_name} | {result.get('status')} | {str(result.get('message', '')).replace('|', '/')} |")
        lines.append("")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_matrix(
    repo_root: Path,
    workspace_root: Path | None = None,
    catalog_path: Path | None = None,
    output_dir: Path | None = None,
) -> dict[str, Any]:
    if output_dir is None and catalog_path is not None:
        output_dir = catalog_path
        catalog_path = workspace_root
        workspace_root = None
    if catalog_path is None or output_dir is None:
        raise TypeError("build_matrix requires catalog_path and output_dir")

    catalog, entries, catalog_errors = load_supported_scene_catalog(catalog_path)
    ros_available = _ros_humble_available()
    builder_found, builder_evidence = _resolve_workcell_builder_executable_evidence(workspace_root)

    enabled_supported_entries = [
        entry
        for entry in entries
        if entry.enabled and entry.support_level == "supported" and entry.status != "disabled"
    ]
    scenes = [_evaluate_scene(repo_root, entry, ros_available) for entry in enabled_supported_entries]
    totals = Counter(scene["overall_status"] for scene in scenes)
    for status in (PASS, FAIL, BLOCKED):
        totals.setdefault(status, 0)

    commands = {
        "matrix_command": "python3 scripts/run_workcell_studio_scene_readiness_matrix.py",
        "catalog": str(catalog_path),
        "outputs": {
            "json": str(output_dir / "scene_readiness_summary.json"),
            "markdown": str(output_dir / "scene_readiness_summary.md"),
        },
        "safe_launch_policy": "Commands are recorded only when they explicitly use use_fake_hardware:=true; ros2 launch is not executed by this matrix.",
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "repo_root": str(repo_root),
        "workspace_root": str(workspace_root) if workspace_root else None,
        "catalog_path": str(catalog_path),
        "catalog_schema_version": catalog.get("schema_version"),
        "catalog_errors": catalog_errors,
        "scene_count": len(scenes),
        "totals": dict(totals),
        "scenes": scenes,
        "commands": commands,
        "ros_humble_available": ros_available,
        "workcell_builder_executable_found": builder_found,
        "workcell_builder_executable_resolution": builder_evidence,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT_DEFAULT)
    parser.add_argument(
        "--workspace-root",
        type=Path,
        default=None,
        help="ROS workspace root; inferred from --repo-root when omitted",
    )
    parser.add_argument(
        "--catalog",
        "--supported-scenes",
        dest="catalog",
        type=Path,
        default=None,
        help="supported-scene catalog path; defaults to <repo>/scenes/supported_scenes.yaml",
    )
    parser.add_argument("--output-dir", type=Path, default=None, help="defaults to <repo>/build/workcell_studio_scene_readiness")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    repo_root = args.repo_root.resolve()
    workspace_root = resolve_workspace_root(repo_root, args.workspace_root) or repo_root
    catalog_path = (args.catalog.resolve() if args.catalog else default_catalog_path(repo_root).resolve())
    output_dir = (args.output_dir.resolve() if args.output_dir else (repo_root / "build" / "workcell_studio_scene_readiness").resolve())
    output_dir.mkdir(parents=True, exist_ok=True)

    if "workspace_root" in inspect.signature(build_matrix).parameters:
        payload = build_matrix(repo_root, workspace_root, catalog_path, output_dir)
    else:
        payload = build_matrix(repo_root, catalog_path, output_dir)
    json_path = output_dir / "scene_readiness_summary.json"
    md_path = output_dir / "scene_readiness_summary.md"
    json_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _write_markdown(payload, md_path)

    print(f"Wrote JSON: {json_path}")
    print(f"Wrote Markdown: {md_path}")
    print("Summary " + " ".join(f"{status}={payload['totals'].get(status, 0)}" for status in (PASS, FAIL, BLOCKED)))
    # This is a reporting matrix: non-zero only for invalid catalog plumbing, not
    # for blocked/failing scenes that the report is meant to surface.
    return 2 if payload.get("catalog_errors") else 0


if __name__ == "__main__":
    raise SystemExit(main())
