#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import yaml

SCHEMA = "workcell_studio_scene3d_visual_quality_matrix/v1"
SMOKE_SCHEMA = "workcell_studio_scene3d_gui_smoke/v1"
PHYSICAL_ITEM_DOMINANCE_RATIO = 0.5
DIAGNOSTIC_FALLBACK_DOMINANCE_RATIO = 0.5
VALID_PHYSICAL_FALLBACK_DOMINANCE_RATIO = 0.5

HELPER_OVERLAY_COUNTERS = (
    "overlay_helper_count",
    "overlay_count",
    "label_count",
    "labels_drawn",
    "warning_anchor_count",
    "fov_helper_count",
    "reach_helper_count",
    "safety_zone_count",
)
PHYSICAL_FIT_BOUNDS_COUNTERS = (
    "physical_fit_bounds_count",
    "physical_fit_bound_count",
    "physical_bounds_count",
    "fit_bounds_physical_count",
)
HELPER_OVERLAY_FIT_BOUNDS_COUNTERS = (
    "overlay_fit_bounds_count",
    "overlay_fit_bound_count",
    "helper_fit_bounds_count",
    "helper_overlay_fit_bounds_count",
    "fit_bounds_overlay_count",
)
VALID_PHYSICAL_FALLBACK_COUNTERS = (
    "valid_physical_fallback_count",
    "physical_fallback_count",
    "physical_fallback_rendered_count",
    "collision_primitive_rendered_count",
)

MESH_KEYS = {"mesh", "mesh_path", "mesh_uri", "mesh_filename", "filename", "resource", "uri"}
MESH_FAILURE_REASON_KEYS = (
    "reason_code",
    "failure_reason_code",
    "mesh_failure_reason_code",
    "mesh_failure_reason",
    "failure_reason",
    "unresolved_reason",
    "reason",
    "warning",
)
PRIMITIVE_TYPES = {"box", "cube", "cylinder", "sphere", "capsule", "cone", "primitive"}


def _load_json(path: Path) -> dict[str, Any]:
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"_load_error": str(exc)}
    return loaded if isinstance(loaded, dict) else {"_load_error": "JSON root is not an object"}


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"_load_error": str(exc)}
    return loaded if isinstance(loaded, dict) else {"_load_error": "YAML root is not an object"}


def _as_int(value: Any, default: int = 0) -> int:
    try:
        if value is None or isinstance(value, bool):
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _counter(payload: dict[str, Any], *keys: str) -> int:
    sources = [payload]
    for nested in ("counters", "render_debug_counters"):
        nested_payload = payload.get(nested)
        if isinstance(nested_payload, dict):
            sources.append(nested_payload)
    for source in sources:
        for key in keys:
            if key in source:
                return _as_int(source.get(key))
    return 0


def _counter_values(payload: dict[str, Any], keys: tuple[str, ...]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for key in keys:
        value = _counter(payload, key)
        if value > 0:
            counts[key] = value
    return counts


def _runtime_available_from_smoke(payload: dict[str, Any], smoke_exists: bool) -> bool:
    if not smoke_exists:
        return False
    for source in (payload, payload.get("counters"), payload.get("render_debug_counters")):
        if isinstance(source, dict) and isinstance(source.get("runtime_available"), bool):
            return bool(source.get("runtime_available"))
    status = _stringify(payload.get("status"))
    blockers = [str(item).lower() for item in payload.get("blockers", [])] if isinstance(payload.get("blockers"), list) else []
    if status == "blocked" and any(
        token in blocker
        for blocker in blockers
        for token in ("unable_to_resolve_workcell_builder_executable", "ros_humble_unavailable", "runtime_unavailable")
    ):
        return False
    # Older smoke files did not carry runtime_available.  If a smoke payload exists
    # and is not an explicit runtime-unavailable BLOCKED report, treat it as an
    # app-launched runtime payload so legacy/synthetic fixtures remain evaluable.
    return True


def _helper_overlay_counts(payload: dict[str, Any]) -> tuple[dict[str, int], int]:
    counts = _counter_values(payload, HELPER_OVERLAY_COUNTERS)
    overlay_family_count = max(counts.get("overlay_helper_count", 0), counts.get("overlay_count", 0))
    helper_overlay_count = overlay_family_count
    for key, value in counts.items():
        if key not in {"overlay_helper_count", "overlay_count"}:
            helper_overlay_count += value
    return counts, helper_overlay_count


def _max_counter(payload: dict[str, Any], keys: tuple[str, ...]) -> int:
    return max((_counter(payload, key) for key in keys), default=0)


def _list_field(payload: dict[str, Any], *keys: str) -> list[Any]:
    for key in keys:
        value = payload.get(key)
        if isinstance(value, list):
            return value
    return []


def _stringify(value: Any) -> str:
    return str(value or "").strip().lower()


def _looks_like_mesh(item: dict[str, Any]) -> bool:
    for key in MESH_KEYS:
        value = item.get(key)
        if isinstance(value, str) and value.strip():
            if key == "mesh" and value.lower() in PRIMITIVE_TYPES:
                continue
            return True
        if isinstance(value, dict):
            filename = value.get("filename") or value.get("uri") or value.get("path")
            if isinstance(filename, str) and filename.strip():
                return True
    geometry = item.get("geometry")
    if isinstance(geometry, dict):
        if _looks_like_mesh(geometry):
            return True
        nested_mesh = geometry.get("mesh")
        if isinstance(nested_mesh, dict):
            return _looks_like_mesh(nested_mesh)
        if isinstance(nested_mesh, str) and nested_mesh.strip() and nested_mesh.lower() not in PRIMITIVE_TYPES:
            return True
    geometry_type = _stringify(item.get("geometry_type") or item.get("visual_type") or item.get("type"))
    return geometry_type == "mesh"


def _looks_like_primitive(item: dict[str, Any]) -> bool:
    for key in ("primitive_type", "shape", "geometry_type", "visual_type", "type"):
        if _stringify(item.get(key)) in PRIMITIVE_TYPES:
            return True
    geometry = item.get("geometry")
    if isinstance(geometry, str):
        return _stringify(geometry) in PRIMITIVE_TYPES
    if isinstance(geometry, dict):
        for primitive in PRIMITIVE_TYPES:
            if primitive in geometry and primitive != "mesh":
                return True
        return _looks_like_primitive(geometry)
    for key in ("box", "cylinder", "sphere", "capsule", "cone"):
        if key in item:
            return True
    return False


def _visual_items(mesh_index: dict[str, Any]) -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    for raw in [*_list_field(mesh_index, "visual_items"), *_list_field(mesh_index, "items")]:
        if isinstance(raw, dict):
            items.append(raw)
    return items


def classify_source_geometry(mesh_index: dict[str, Any]) -> tuple[int, int, int, int]:
    """Return total payload, mesh source, primitive source, missing geometry counts."""
    items = _visual_items(mesh_index)
    mesh_source_count = 0
    primitive_source_count = 0
    missing_geometry_count = 0
    for item in items:
        has_mesh = _looks_like_mesh(item)
        has_primitive = _looks_like_primitive(item)
        if has_mesh:
            mesh_source_count += 1
        elif has_primitive:
            primitive_source_count += 1
        else:
            missing_geometry_count += 1

    if not items:
        mesh_source_count = max(
            _as_int(mesh_index.get("mesh_source_count")),
            _as_int(mesh_index.get("candidate_mesh_count")),
            _as_int(mesh_index.get("renderable_mesh_count")),
        )
        primitive_source_count = max(
            _as_int(mesh_index.get("primitive_source_count")),
            _as_int(mesh_index.get("primitive_visual_count")),
        )
        missing_geometry_count = _as_int(mesh_index.get("unresolved_placeholder_count"))

    total_payload_count = max(
        len(items),
        _as_int(mesh_index.get("total_payload_count")),
        _as_int(mesh_index.get("visual_count")),
        _as_int(mesh_index.get("renderable_item_count")),
        mesh_source_count + primitive_source_count + missing_geometry_count,
    )
    if missing_geometry_count == 0 and total_payload_count > mesh_source_count + primitive_source_count:
        missing_geometry_count = total_payload_count - mesh_source_count - primitive_source_count
    return total_payload_count, mesh_source_count, primitive_source_count, missing_geometry_count


def _mesh_failure_reason_for_item(item: dict[str, Any]) -> str | None:
    for key in MESH_FAILURE_REASON_KEYS:
        value = item.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip().lower().replace(" ", "_")
    if item.get("resolved") is False and _looks_like_mesh(item):
        return "unresolved_mesh_without_reason_code"
    return None


def mesh_failure_summary(mesh_index: dict[str, Any]) -> dict[str, int]:
    reasons: dict[str, int] = {}
    for item in _visual_items(mesh_index):
        if item.get("resolved") is False and _looks_like_mesh(item):
            reason = _mesh_failure_reason_for_item(item) or "unresolved_mesh_without_reason_code"
            reasons[reason] = reasons.get(reason, 0) + 1
    unresolved = mesh_index.get("unresolved")
    if isinstance(unresolved, list):
        for raw in unresolved:
            if isinstance(raw, dict):
                reason = _mesh_failure_reason_for_item(raw) or "unresolved_mesh_without_reason_code"
            elif isinstance(raw, str) and raw.strip():
                reason = raw.strip().lower().replace(" ", "_")
            else:
                continue
            reasons[reason] = reasons.get(reason, 0) + 1
    return dict(sorted(reasons.items()))


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
        status = _stringify(entry.get("status"))
        support_level = _stringify(entry.get("support_level"))
        if enabled and status not in {"disabled", "ignored"} and support_level not in {"disabled", "ignored"}:
            entries.append(entry)
    return entries


def _resolve_mesh_index(scene_dir: Path) -> Path:
    if scene_dir.is_file():
        return scene_dir
    return scene_dir / "generated" / "scene_visual_mesh_index.json"


def _resolve_smoke_json(smoke_dir: Path | None, scene_name: str, scene_dir: Path) -> Path | None:
    candidates: list[Path] = []
    if smoke_dir is not None:
        candidates.extend([
            smoke_dir / f"scene3d_gui_smoke_{scene_name}.json",
            smoke_dir / f"{scene_name}.json",
        ])
    candidates.append(scene_dir / "generated" / "scene3d_gui_smoke.json")
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0] if candidates else None


def _resolve_screenshot(screenshot_dir: Path | None, scene_name: str) -> Path | None:
    if screenshot_dir is None:
        return None
    for suffix in ("png", "jpg", "jpeg"):
        candidate = screenshot_dir / f"scene3d_gui_smoke_{scene_name}.{suffix}"
        if candidate.exists():
            return candidate
        candidate = screenshot_dir / f"{scene_name}.{suffix}"
        if candidate.exists():
            return candidate
    return screenshot_dir / f"scene3d_gui_smoke_{scene_name}.png"


def evaluate_scene(
    *,
    scene_name: str,
    scene_dir: Path,
    mesh_index_path: Path,
    smoke_json_path: Path | None,
    screenshot_path: Path | None = None,
    synthetic_fixture: bool = False,
    require_composition_evidence: bool = False,
) -> dict[str, Any]:
    warnings: list[str] = []
    blockers: list[str] = []
    blocker_reasons: list[str] = []

    def add_blocker(message: str, reason: str | None = None) -> None:
        blockers.append(message)
        if reason and reason not in blocker_reasons:
            blocker_reasons.append(reason)

    mesh_index: dict[str, Any] = {}
    if not mesh_index_path.exists():
        blockers.append(f"missing mesh/source payload: {mesh_index_path}")
    else:
        mesh_index = _load_json(mesh_index_path)
        if mesh_index.get("_load_error"):
            blockers.append(f"could not read mesh/source payload: {mesh_index_path}: {mesh_index['_load_error']}")

    total_payload_count, mesh_source_count, primitive_source_count, missing_geometry_count = classify_source_geometry(mesh_index)
    mesh_failure_reasons = mesh_failure_summary(mesh_index)

    smoke_payload: dict[str, Any] = {}
    smoke_exists = bool(smoke_json_path and smoke_json_path.exists())
    if smoke_exists and smoke_json_path is not None:
        smoke_payload = _load_json(smoke_json_path)
        if smoke_payload.get("_load_error"):
            add_blocker(
                f"smoke_json_unreadable: could not read smoke JSON: {smoke_json_path}: {smoke_payload['_load_error']}",
                "smoke_json_unreadable",
            )
        elif smoke_payload.get("schema") not in {SMOKE_SCHEMA, None}:
            warnings.append(f"unexpected smoke schema: {smoke_payload.get('schema')!r}")
    elif smoke_json_path is not None:
        add_blocker(f"smoke_json_missing: smoke JSON not found: {smoke_json_path}", "smoke_json_missing")
    else:
        warnings.append("smoke JSON not provided")

    runtime_available = _runtime_available_from_smoke(smoke_payload, smoke_exists)
    if not runtime_available:
        warnings.append("Scene3D runtime unavailable; static renderability counts are diagnostic context, not GUI render proof")

    mesh_rendered_count = _counter(smoke_payload, "mesh_rendered_count", "mesh_backed_count")
    primitive_rendered_count = _counter(smoke_payload, "primitive_rendered_count", "urdf_primitive_rendered_count")
    # Diagnostic fallback evidence is useful for debugging viewport plumbing, but it is not
    # proof that a scene rendered its physical mesh/URDF-primitive geometry truthfully.
    placeholder_count = _counter(smoke_payload, "placeholder_count", "placeholder_box_count")
    wireframe_fallback_count = _counter(smoke_payload, "wireframe_fallback_count", "wireframe_box_count")
    raw_generated_bounds_count = _counter(
        smoke_payload,
        "raw_generated_bounds_count",
        "generated_bounds_count",
        "generated_fallback_count",
        "raw_generated_fallback_count",
        "bounds_fallback_count",
    )
    missing_geometry_box_count = _counter(
        smoke_payload,
        "missing_geometry_box_count",
        "missing_geometry_rendered_count",
        "missing_geometry_fallback_count",
    )
    rendered_count = _counter(smoke_payload, "rendered_count")
    helper_overlay_counts, helper_overlay_count = _helper_overlay_counts(smoke_payload)
    physical_fit_bounds_count = _max_counter(smoke_payload, PHYSICAL_FIT_BOUNDS_COUNTERS)
    helper_overlay_fit_bounds_count = _max_counter(smoke_payload, HELPER_OVERLAY_FIT_BOUNDS_COUNTERS)
    valid_physical_fallback_count = _max_counter(smoke_payload, VALID_PHYSICAL_FALLBACK_COUNTERS)
    physical_anchor_count = _counter(smoke_payload, "physical_anchor_count", "initial_fit_physical_anchor_count")
    generated_robot_mesh_count = _counter(smoke_payload, "generated_robot_mesh_count", "robot_mesh_count")
    tool_gripper_visual_count = _counter(smoke_payload, "tool_gripper_visual_count", "gripper_tool_visual_count", "tool_visual_count", "gripper_visual_count")
    table_workbench_visual_count = _counter(smoke_payload, "table_workbench_visual_count", "table_visual_count", "workbench_visual_count")
    camera_body_visual_count = _counter(smoke_payload, "camera_body_visual_count", "camera_visual_count")

    credible_source_count = mesh_source_count + primitive_source_count
    credible_physical_rendered_count = mesh_rendered_count + primitive_rendered_count
    physical_rendered_count = credible_physical_rendered_count
    valid_physical_fallback_dominates = False
    if valid_physical_fallback_count > 0:
        valid_fallback_limit = credible_physical_rendered_count * VALID_PHYSICAL_FALLBACK_DOMINANCE_RATIO
        if credible_physical_rendered_count > 0 and valid_physical_fallback_count <= valid_fallback_limit:
            physical_rendered_count += valid_physical_fallback_count
        else:
            valid_physical_fallback_dominates = True
    diagnostic_fallback_count = (
        raw_generated_bounds_count
        + placeholder_count
        + wireframe_fallback_count
        + max(0, missing_geometry_box_count - placeholder_count)
    )

    if runtime_available:
        if credible_source_count > 0 and physical_rendered_count <= 0:
            add_blocker(
                "scene_rendered_no_physical_items: Scene3D runtime launched but physical mesh/primitive render counters are zero",
                "scene_rendered_no_physical_items",
            )
        if mesh_source_count > 0 and mesh_rendered_count <= 0:
            blockers.append("mesh_source_count > 0 requires mesh_rendered_count > 0")
        if primitive_source_count > 0 and primitive_rendered_count <= 0:
            blockers.append("primitive_source_count > 0 requires primitive_rendered_count > 0")
        if primitive_source_count > 0 and placeholder_count > missing_geometry_count:
            blockers.append("valid URDF primitives must not increment placeholder_count")
        if total_payload_count > 0 and credible_source_count <= 0:
            blockers.append("source geometry classification missing; rendered_count alone cannot prove visual quality")
    elif smoke_payload.get("status") == "BLOCKED" or not smoke_exists:
        if "scene3d_runtime_unavailable" not in blocker_reasons:
            blocker_reasons.append("scene3d_runtime_unavailable")
    if rendered_count == total_payload_count and total_payload_count > 0:
        warnings.append("rendered_count equals total_payload_count; pass still requires mesh/primitive-specific rendered counts")

    if diagnostic_fallback_count > 0:
        warnings.append(
            "diagnostic fallback evidence present; physical_rendered_count excludes raw bounds, placeholders, and wireframes"
        )
    if helper_overlay_count > 0:
        warnings.append("helper/overlay render evidence present; physical_rendered_count excludes helper overlays")
    if runtime_available and valid_physical_fallback_dominates:
        add_blocker(
            "valid physical fallback counters dominate credible mesh/primitive render evidence; physical_rendered_count excludes them",
            "physical_fallback_dominates",
        )
    if missing_geometry_count > 0:
        warnings.append(f"{missing_geometry_count} source payload item(s) lack mesh or primitive geometry")
    for reason, count in mesh_failure_reasons.items():
        add_blocker(f"{reason}: {count} mesh-backed source item(s) could not be resolved", reason)

    if runtime_available and helper_overlay_count > 0 and physical_rendered_count <= 0 and rendered_count > 0:
        add_blocker(
            "no_physical_scene_items_rendered: helper/overlay count is the only render evidence; rendered_count cannot prove Scene3D visual quality",
            "no_physical_scene_items_rendered",
        )
    if runtime_available and helper_overlay_count > 0 and helper_overlay_count >= physical_rendered_count:
        add_blocker(
            "overlay_helper_dominates: helper/overlay count is greater than or equal to physical_rendered_count; render physical scene items instead of relying on overlays",
            "overlay_helper_dominates",
        )
    if (
        runtime_available
        and physical_fit_bounds_count > 0
        and helper_overlay_fit_bounds_count >= physical_fit_bounds_count
        and helper_overlay_fit_bounds_count > 0
    ):
        add_blocker(
            "overlay_helper_dominates: helper/overlay fit-bounds counters dominate physical fit-bounds counters; fit the camera to physical scene bounds",
            "overlay_helper_dominates",
        )

    if runtime_available and credible_source_count > 0 and physical_rendered_count <= 0 and raw_generated_bounds_count > 0:
        blockers.append("raw/generated fallback bounds are the only visible evidence despite mesh or URDF primitive sources")
    if runtime_available and credible_source_count > 0 and physical_rendered_count > 0 and raw_generated_bounds_count > 0:
        if raw_generated_bounds_count > int(physical_rendered_count * DIAGNOSTIC_FALLBACK_DOMINANCE_RATIO):
            blockers.append("raw/generated fallback bounds dominate physical render evidence despite mesh or URDF primitive sources")
    if runtime_available and credible_source_count > 0 and physical_rendered_count > 0 and diagnostic_fallback_count > 0:
        if diagnostic_fallback_count > int(physical_rendered_count * DIAGNOSTIC_FALLBACK_DOMINANCE_RATIO):
            blockers.append("diagnostic fallback evidence dominates physical render evidence despite mesh or URDF primitive sources")

    if runtime_available:
        composition_expected = generated_robot_mesh_count > 0 or tool_gripper_visual_count > 0 or table_workbench_visual_count > 0 or camera_body_visual_count > 0
        if require_composition_evidence and physical_anchor_count <= 0 and physical_rendered_count > 0:
            add_blocker("physical_composition_missing: physical_anchor_count is zero despite rendered physical evidence", "physical_composition_missing")
        if require_composition_evidence or composition_expected:
            if generated_robot_mesh_count <= 0:
                add_blocker("robot_composition_missing: generated_robot_mesh_count is zero; supported scenes need credible robot composition evidence", "robot_composition_missing")
            if table_workbench_visual_count <= 0:
                add_blocker("table_composition_missing: table_workbench_visual_count is zero; supported scenes need table/workbench composition evidence", "table_composition_missing")
            if camera_body_visual_count <= 0:
                add_blocker("camera_composition_missing: camera_body_visual_count is zero; supported scenes need camera body composition evidence", "camera_composition_missing")
            if tool_gripper_visual_count <= 0:
                add_blocker("tool_composition_missing: tool_gripper_visual_count is zero; supported scenes need tool/gripper composition evidence", "tool_composition_missing")

    visible_physical_count = physical_rendered_count
    if runtime_available and wireframe_fallback_count > 0 and visible_physical_count > 0:
        if wireframe_fallback_count > int(visible_physical_count * PHYSICAL_ITEM_DOMINANCE_RATIO):
            blockers.append("wireframe_fallback_count dominates visible physical items")

    if screenshot_path is not None and not screenshot_path.exists():
        add_blocker(f"screenshot_missing: screenshot not found: {screenshot_path}", "screenshot_missing")

    if synthetic_fixture:
        if mesh_source_count < 1 or primitive_source_count < 1:
            blockers.append("synthetic fixture must contain at least one mesh and one primitive source")
        if mesh_rendered_count < 1 or primitive_rendered_count < 1:
            blockers.append("synthetic fixture must render both one mesh and one primitive")
        if placeholder_count != 0:
            blockers.append("synthetic fixture must render mesh and primitive without placeholder boxes")
        if diagnostic_fallback_count != 0:
            blockers.append("synthetic fixture must render mesh and primitive without diagnostic fallback bounds or boxes")

    visual_quality_status = "PASS" if runtime_available and not blockers else "FAIL"
    return {
        "scene_name": scene_name,
        "scene_path": str(scene_dir),
        "mesh_index_path": str(mesh_index_path),
        "smoke_json": str(smoke_json_path) if smoke_json_path is not None else "",
        "screenshot": str(screenshot_path) if screenshot_path is not None else "",
        "total_payload_count": total_payload_count,
        "mesh_source_count": mesh_source_count,
        "mesh_rendered_count": mesh_rendered_count,
        "mesh_failure_summary_by_reason_code": mesh_failure_reasons,
        "primitive_source_count": primitive_source_count,
        "primitive_rendered_count": primitive_rendered_count,
        "physical_rendered_count": physical_rendered_count,
        "credible_physical_rendered_count": credible_physical_rendered_count,
        "valid_physical_fallback_count": valid_physical_fallback_count,
        "physical_anchor_count": physical_anchor_count,
        "generated_robot_mesh_count": generated_robot_mesh_count,
        "tool_gripper_visual_count": tool_gripper_visual_count,
        "table_workbench_visual_count": table_workbench_visual_count,
        "camera_body_visual_count": camera_body_visual_count,
        "helper_overlay_count": helper_overlay_count,
        "helper_overlay_counts": helper_overlay_counts,
        "physical_fit_bounds_count": physical_fit_bounds_count,
        "helper_overlay_fit_bounds_count": helper_overlay_fit_bounds_count,
        "placeholder_count": placeholder_count,
        "raw_generated_bounds_count": raw_generated_bounds_count,
        "missing_geometry_box_count": missing_geometry_box_count,
        "diagnostic_fallback_count": diagnostic_fallback_count,
        "missing_geometry_count": missing_geometry_count,
        "wireframe_fallback_count": wireframe_fallback_count,
        "rendered_count": rendered_count,
        "runtime_available": runtime_available,
        "visual_quality_status": visual_quality_status,
        "warnings": warnings,
        "blockers": blockers,
        "blocker_reasons": blocker_reasons,
    }


def build_matrix(
    *,
    repo_root: Path,
    supported_scenes: Path,
    synthetic_fixture: Path,
    smoke_dir: Path | None = None,
    screenshot_dir: Path | None = None,
) -> dict[str, Any]:
    scenes: list[dict[str, Any]] = []
    for entry in _scene_entries(supported_scenes):
        scene_name = str(entry.get("scene_name") or entry.get("package_name") or "").strip()
        if not scene_name:
            continue
        scene_dir = repo_root / str(entry.get("scene_path") or f"scenes/{scene_name}")
        mesh_index_path = _resolve_mesh_index(scene_dir)
        scenes.append(
            evaluate_scene(
                scene_name=scene_name,
                scene_dir=scene_dir,
                mesh_index_path=mesh_index_path,
                smoke_json_path=_resolve_smoke_json(smoke_dir, scene_name, scene_dir),
                screenshot_path=_resolve_screenshot(screenshot_dir, scene_name),
                require_composition_evidence=True,
            )
        )

    synthetic_path = synthetic_fixture
    synthetic_scene_dir = synthetic_path if synthetic_path.is_dir() else synthetic_path.parent
    synthetic_name = synthetic_scene_dir.name or "synthetic_visual_quality_fixture"
    synthetic_smoke = _resolve_smoke_json(smoke_dir, synthetic_name, synthetic_scene_dir)
    if synthetic_path.is_file() and synthetic_path.name.endswith(".json"):
        mesh_index_path = synthetic_path
        if synthetic_path.name.startswith("scene3d_gui_smoke") or "smoke" in synthetic_path.name:
            synthetic_smoke = synthetic_path
            mesh_index_path = synthetic_scene_dir / "scene_visual_mesh_index.json"
    else:
        mesh_index_path = _resolve_mesh_index(synthetic_scene_dir)
    scenes.append(
        evaluate_scene(
            scene_name=synthetic_name,
            scene_dir=synthetic_scene_dir,
            mesh_index_path=mesh_index_path,
            smoke_json_path=synthetic_smoke,
            screenshot_path=_resolve_screenshot(screenshot_dir, synthetic_name),
            synthetic_fixture=True,
        )
    )

    blockers = [f"{scene['scene_name']}: {blocker}" for scene in scenes for blocker in scene.get("blockers", [])]
    return {
        "schema": SCHEMA,
        "repo_root": str(repo_root),
        "supported_scenes": str(supported_scenes),
        "synthetic_fixture": str(synthetic_fixture),
        "scene_count": len(scenes),
        "scenes": scenes,
        "blockers": blockers,
        "pass": not blockers,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate Scene3D per-scene mesh/primitive visual quality evidence.")
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--supported-scenes", type=Path, default=None, help="Catalog YAML, defaults to <repo>/scenes/supported_scenes.yaml")
    parser.add_argument("--synthetic-fixture", type=Path, required=True, help="Synthetic generated scene directory or mesh-index JSON with one mesh and one primitive")
    parser.add_argument("--smoke-dir", type=Path, default=None, help="Optional directory containing scene3d_gui_smoke_<scene>.json files")
    parser.add_argument("--screenshot-dir", type=Path, default=None, help="Optional directory containing scene3d_gui_smoke_<scene>.png files")
    parser.add_argument("--json", type=Path, default=None, help="Output JSON path; defaults to stdout only")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    repo_root = args.repo_root.resolve()
    supported_scenes = (args.supported_scenes or (repo_root / "scenes" / "supported_scenes.yaml")).resolve()
    payload = build_matrix(
        repo_root=repo_root,
        supported_scenes=supported_scenes,
        synthetic_fixture=args.synthetic_fixture.resolve(),
        smoke_dir=args.smoke_dir.resolve() if args.smoke_dir else None,
        screenshot_dir=args.screenshot_dir.resolve() if args.screenshot_dir else None,
    )
    rendered = json.dumps(payload, indent=2)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if payload["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
