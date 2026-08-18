#!/usr/bin/env python3
"""Run Web 3D visual acceptance for any Workcell Studio scene.

The acceptance flow keeps all generated browser artifacts under
``build/workcell_studio_web_scene/`` and validates the same staged JSON that the
static viewer opens.  Browser collection is best-effort unless
``--require-browser`` is supplied.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import time
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from threading import Thread
from typing import Any, Mapping, Sequence
from urllib.parse import quote

REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_ROOT = REPO_ROOT / "build" / "workcell_studio_web_scene"
PNG_1X1 = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+/p9sAAAAASUVORK5CYII="
)

EXPECTED_MESH_LOADED_COUNT = 18
EXPECTED_REQUIRED_MESH_FAILED_COUNT = 0
REQUIRED_VIEWER_RESOLVED_DISTANCE_PAIRS = {
    "wrist_3_link -> tool0": 0.20,
    "tool0 -> gripper_base_link": 0.35,
    "wrist_3_link -> gripper_base_link": 0.45,
}
REQUIRED_RENDERED_MESH_ADJACENT_PAIRS = {
    "base_link_inertia -> shoulder_link": 0.75,
    "shoulder_link -> upper_arm_link": 0.75,
    "upper_arm_link -> forearm_link": 0.75,
    "forearm_link -> wrist_1_link": 0.75,
    "wrist_1_link -> wrist_2_link": 0.75,
    "wrist_2_link -> wrist_3_link": 0.75,
}
REQUIRED_BROWSER_MATRIX_LINKS = (
    "base_link", "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link",
    "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0", "gripper_base_link",
    "gripper_finger1_knuckle_link", "gripper_finger2_knuckle_link",
    "gripper_finger1_finger_link", "gripper_finger2_finger_link",
    "gripper_finger1_inner_knuckle_link", "gripper_finger2_inner_knuckle_link",
    "gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link",
)

REQUIRED_PRODUCT_FALLBACK_TOKENS = (
    "required_mesh_failed_debug_fallback",
    "primitive_fallback",
    "box_fallback",
    "debug_fallback",
    "fallback_geometry",
)

LIFECYCLE_STATES = {"idle", "loading_urdf", "loading_meshes", "ready", "failed"}
REQUIRED_UR5_ROBOTIQ_MESH_TOKENS = (
    "ur5", "base", "shoulder", "upper_arm", "forearm", "wrist", "robotiq", "gripper",
)
REQUIRED_ROBOTIQ_MESH_BASENAMES = (
    "robotiq_85_base_link.dae",
    "robotiq_85_knuckle_link.dae",
    "robotiq_85_finger_link.dae",
    "robotiq_85_inner_knuckle_link.dae",
    "robotiq_85_finger_tip_link.dae",
)

SUPPORTED_SCENE_ACCEPTANCE_ORDER = (
    "ur5_2f_test",
    "ur5_3f_test",
    "suction_test",
    "ur10_2f_test",
    "ur3_suction_test",
    "ur5_airpick4_test",
    "ur5_2f_sorting_test",
    "ur5_2f_builder_pick_place_demo",
)
SEQUENTIAL_SCENE_SWITCH_ORDER = (
    "ur5_2f_test",
    "suction_test",
    "ur5_3f_test",
    "ur5_airpick4_test",
    "ur5_2f_test",
)



def _status_value(status: Mapping[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in status and status.get(key) is not None:
            return status.get(key)
    return None


def _status_bool(status: Mapping[str, Any], *keys: str) -> bool | None:
    for key in keys:
        if key in status:
            value = status.get(key)
            if isinstance(value, bool):
                return value
            if isinstance(value, (int, float)) and value in (0, 1):
                return bool(value)
    return None


def _bounds_from_status(status: Mapping[str, Any], *keys: str) -> dict[str, tuple[float, float, float]] | None:
    value = _status_value(status, *keys)
    if not isinstance(value, Mapping):
        return None
    mins = _diagnostic_vector(value.get("min") or value.get("mins") or value.get("minimum"))
    maxs = _diagnostic_vector(value.get("max") or value.get("maxs") or value.get("maximum"))
    if mins is None or maxs is None:
        return None
    return {"min": mins, "max": maxs}


def _bounds_error(field: str, raw: Any) -> str | None:
    if not isinstance(raw, Mapping):
        return f"browser viewer {field} must be a bounds object with finite min/max vectors, got {raw!r}"
    bounds = _bounds_from_status({field: raw}, field)
    if bounds is None:
        return f"browser viewer {field} must contain finite min/max vectors, got {raw!r}"
    mins, maxs = bounds["min"], bounds["max"]
    extents = tuple(maxs[i] - mins[i] for i in range(3))
    if not all(math.isfinite(v) for v in (*mins, *maxs, *extents)):
        return f"browser viewer {field} must be finite, got {raw!r}"
    if any(v <= 1e-9 for v in extents):
        return f"browser viewer {field} must be non-degenerate with positive volume, got {raw!r}"
    return None


def _bounds_contains(outer: Mapping[str, tuple[float, float, float]], inner: Mapping[str, tuple[float, float, float]], tolerance: float = 1e-4) -> bool:
    return all(outer["min"][i] <= inner["min"][i] + tolerance and outer["max"][i] + tolerance >= inner["max"][i] for i in range(3))


def _physical_fit_errors(status: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    root_count = _status_int_any(status, "physical_assembly_root_count", "physicalAssemblyRootCount")
    if root_count != 1:
        errors.append(f"browser viewer physical_assembly_root_count required 1, got {root_count!r}")
    included = _status_bool(status, "physical_fit_included_robot_preview", "physicalFitIncludedRobotPreview")
    if included is not True:
        errors.append(f"browser viewer physical_fit_included_robot_preview required True, got {included!r}")
    raw_assembly = _status_value(status, "physical_assembly_bounds", "physicalAssemblyBounds")
    raw_final = _status_value(status, "final_physical_fit_bounds", "finalPhysicalFitBounds")
    for field, raw in (("physical_assembly_bounds", raw_assembly), ("final_physical_fit_bounds", raw_final)):
        error = _bounds_error(field, raw)
        if error:
            errors.append(error)
    assembly = _bounds_from_status(status, "physical_assembly_bounds", "physicalAssemblyBounds")
    final = _bounds_from_status(status, "final_physical_fit_bounds", "finalPhysicalFitBounds")
    # The viewer owns fit bounds truth; acceptance reads these values without rewriting them.
    return errors




def png_dimensions(path: Path) -> tuple[int, int] | None:
    try:
        data = path.read_bytes()[:24]
    except OSError:
        return None
    if len(data) < 24 or data[:8] != b"\x89PNG\r\n\x1a\n" or data[12:16] != b"IHDR":
        return None
    return (int.from_bytes(data[16:20], "big"), int.from_bytes(data[20:24], "big"))


def require_browser_artifact_errors(browser: Mapping[str, Any], server_status: str, screenshot_path: Path, browser_status_path: Path) -> list[str]:
    errors: list[str] = []
    if browser.get("available") is not True:
        errors.append(f"browser.available required True when --require-browser is active, got {browser.get('available')!r}")
    method = str(browser.get("method") or "")
    if method != "playwright":
        errors.append(f"browser.method required real Playwright/Chromium path 'playwright', got {method!r}")
    if server_status not in {"started", "reused"}:
        errors.append(f"server_status required 'started' or 'reused', got {server_status!r}")
    dims = png_dimensions(screenshot_path)
    size = screenshot_path.stat().st_size if screenshot_path.exists() else 0
    if dims is None:
        errors.append(f"screenshot file must be a real PNG, got path={screenshot_path} size={size}")
    elif dims[0] <= 1 or dims[1] <= 1:
        errors.append(f"screenshot dimensions must be larger than 1x1, got {dims!r}")
    if size <= len(PNG_1X1) + 32:
        errors.append(f"screenshot file size must be non-placeholder, got {size} bytes")
    if not browser_status_path.is_file() or browser_status_path.stat().st_size <= 2:
        errors.append(f"browser status JSON must be populated, got {browser_status_path}")
    return errors


def _status_list(status: Mapping[str, Any], *keys: str) -> list[Any]:
    value = _status_value(status, *keys)
    if isinstance(value, list):
        return value
    if isinstance(value, tuple):
        return list(value)
    return []


def _final_lifecycle_state(status: Mapping[str, Any]) -> str:
    return str(_status_value(status, "robot_preview_lifecycle_state", "robotPreviewLifecycleState", "final_state", "finalState") or "").strip()


def robot_preview_lifecycle_diagnostics(status: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "expected_visual_count": _status_int_any(status, "robot_expected_visual_count", "robotExpectedVisualCount"),
        "completed_visual_count": _status_int_any(status, "robot_completed_visual_count", "robotCompletedVisualCount", "robot_loaded_visual_count", "robotLoadedVisualCount"),
        "failed_visual_count": _status_int_any(status, "robot_failed_visual_count", "robotFailedVisualCount"),
        "loaded_link_count": _status_int_any(status, "robot_loaded_link_count", "robotLoadedLinkCount"),
        "root_link_count": _status_int_any(status, "robot_root_link_count", "robotRootLinkCount"),
        "missing_links": _status_list(status, "robot_hierarchy_missing_links", "robotHierarchyMissingLinks"),
        "disconnected_links": _status_list(status, "robot_disconnected_links", "robotDisconnectedLinks"),
        "duplicate_links": _status_list(status, "robot_duplicate_links", "robotDuplicateLinks"),
        "missing_meshes": _status_list(status, "robot_missing_meshes", "robotMissingMeshes"),
        "final_state": _final_lifecycle_state(status),
    }


def _required_robot_mesh_failure_errors(status: Mapping[str, Any]) -> list[str]:
    missing_meshes = [str(item) for item in _status_list(status, "robot_missing_meshes", "robotMissingMeshes")]
    failures = []
    for item in missing_meshes:
        text = item.lower().replace("_", "-")
        if any(token.replace("_", "-") in text for token in REQUIRED_UR5_ROBOTIQ_MESH_TOKENS):
            failures.append(item)
    if failures:
        return ["browser viewer required UR5/Robotiq meshes failed or are missing: " + "; ".join(failures[:20])]
    return []


def _robot_lifecycle_errors(status: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    final_state = _final_lifecycle_state(status)
    if final_state not in LIFECYCLE_STATES:
        errors.append(f"browser viewer robot-preview lifecycle final state must be one of {sorted(LIFECYCLE_STATES)}, got {final_state!r}")
    if final_state != "ready":
        errors.append(f"browser viewer robot-preview lifecycle final state must be ready, got {final_state!r}")
    root_count = _status_int_any(status, "robot_root_link_count", "robotRootLinkCount")
    if root_count > 1:
        roots = _status_list(status, "robot_root_links", "robotRootLinks")
        errors.append(f"browser viewer robot/tool must have at most one root link, got {root_count}: {roots!r}")
    disconnected = _status_list(status, "robot_disconnected_links", "robotDisconnectedLinks")
    if disconnected:
        errors.append(f"browser viewer robot disconnected links must be empty, got {disconnected!r}")
    duplicates = _status_list(status, "robot_duplicate_links", "robotDuplicateLinks")
    if duplicates:
        errors.append(f"browser viewer duplicate links must be empty, got {duplicates!r}")
    failed_count = _status_int_any(status, "robot_failed_visual_count", "robotFailedVisualCount")
    if failed_count > 0:
        errors.append(f"browser viewer robot failed visual count must be 0, got {failed_count}")
    if _status_value(status, "robot_preview_canonical_fallback_used", "robotPreviewCanonicalFallbackUsed") is True:
        errors.append("browser viewer canonical robot fallback path must not be used")
    errors.extend(_required_robot_mesh_failure_errors(status))
    return errors

def _status_int(status: Mapping[str, Any], snake: str, camel: str) -> int:
    return int(status.get(snake) if status.get(snake) is not None else status.get(camel) or 0)


def _status_int_any(status: Mapping[str, Any], *keys: str) -> int:
    for key in keys:
        if status.get(key) is not None:
            return int(status.get(key) or 0)
    return 0


def _distance_map_from_status(status: Mapping[str, Any]) -> Mapping[str, Any]:
    for key in ("viewer_resolved_distances_m", "resolved_distances_m", "resolvedFrameDistancesM", "resolved_frame_distances_m"):
        value = status.get(key)
        if isinstance(value, Mapping):
            return value
    return {}


def _resolved_frame_positions_from_status(status: Mapping[str, Any]) -> Mapping[str, Any]:
    for key in ("resolved_frame_positions", "resolvedFramePositions"):
        value = status.get(key)
        if isinstance(value, Mapping):
            return value
    return {}


def _frame_diagnostics_from_status(status: Mapping[str, Any]) -> Sequence[Any] | Mapping[str, Any]:
    for key in ("frame_diagnostics", "frameDiagnostics"):
        value = status.get(key)
        if isinstance(value, Mapping):
            return value
        if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
            return value
    return []


def _rendered_mesh_diagnostics_from_status(status: Mapping[str, Any]) -> Sequence[Any]:
    for key in ("rendered_mesh_diagnostics", "renderedMeshDiagnostics"):
        value = status.get(key)
        if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
            return value
    return []



def _diagnostic_text(diagnostic: Mapping[str, Any]) -> str:
    values = []
    for key in ("id", "object_id", "objectId", "object_name", "objectName", "display_name", "displayName", "name", "label", "category", "role", "type"):
        value = diagnostic.get(key)
        if isinstance(value, str):
            values.append(value)
    return " ".join(values).lower().replace("_", "-")


def _diagnostic_bbox_size(diagnostic: Mapping[str, Any]) -> tuple[float, float, float] | None:
    for key in ("bounding_box_size", "boundingBoxSize", "loaded_mesh_bounding_box_size", "loadedMeshBoundingBoxSize"):
        vector = _diagnostic_vector(diagnostic.get(key))
        if vector is not None:
            return tuple(abs(component) for component in vector)
    return None


def _diagnostic_status(diagnostic: Mapping[str, Any]) -> str:
    return str(diagnostic.get("render_status") or diagnostic.get("renderStatus") or diagnostic.get("mesh_status") or "").lower()


def _diagnostic_bool(diagnostic: Mapping[str, Any], *keys: str) -> bool:
    for key in keys:
        value = diagnostic.get(key)
        if isinstance(value, bool):
            return value
    return False


def _diagnostic_expected_dimensions(diagnostic: Mapping[str, Any]) -> tuple[float, float, float] | None:
    for key in ("expected_dimensions_m", "expectedDimensionsM", "expected_dimensions", "expectedDimensions"):
        vector = _diagnostic_vector(diagnostic.get(key))
        if vector is not None:
            return tuple(abs(component) for component in vector)
    return None


def _diagnostic_scale(diagnostic: Mapping[str, Any]) -> tuple[float, float, float] | None:
    for key in ("mesh_local_scale", "meshLocalScale", "scale", "mesh_scale", "meshScale"):
        vector = _diagnostic_vector(diagnostic.get(key))
        if vector is not None:
            return tuple(abs(component) for component in vector)
    correction = diagnostic.get("mesh_unit_correction") or diagnostic.get("meshUnitCorrection")
    if isinstance(correction, Mapping):
        raw = correction.get("scale")
        try:
            scale = abs(float(raw))
        except (TypeError, ValueError):
            return None
        return (scale, scale, scale)
    return None


def _is_non_uniform(vector: tuple[float, float, float], tolerance: float = 0.05) -> bool:
    finite = [component for component in vector if component > 0 and math.isfinite(component)]
    if len(finite) != 3:
        return False
    return max(finite) / min(finite) > (1.0 + tolerance)


def _is_required_product_diagnostic(diagnostic: Mapping[str, Any]) -> bool:
    text = _diagnostic_text(diagnostic)
    category = str(diagnostic.get("category") or "").lower()
    return (
        category in {"robot", "tool", "table"}
        or any(token in text for token in ("ur5", "robot", "base-link", "shoulder-link", "upper-arm", "forearm", "wrist", "tool0", "gripper", "robotiq", "table", "workbench", "bench"))
    )


def _required_product_fallback_errors(status: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    for diagnostic in _rendered_mesh_diagnostics_from_status(status):
        if not isinstance(diagnostic, Mapping) or not _is_required_product_diagnostic(diagnostic):
            continue
        render_status = _diagnostic_status(diagnostic)
        fallback_visible = _diagnostic_bool(diagnostic, "fallback_visible", "fallbackVisible")
        if fallback_visible and any(token in render_status for token in REQUIRED_PRODUCT_FALLBACK_TOKENS):
            name = _diagnostic_link_name(diagnostic) or str(diagnostic.get("id") or diagnostic.get("object_id") or "required product")
            errors.append(f"browser viewer required product {name} has visible fallback/debug geometry render_status={render_status!r}")
    return errors


def _table_mesh_contract_errors(status: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    for diagnostic in _rendered_mesh_diagnostics_from_status(status):
        if not isinstance(diagnostic, Mapping):
            continue
        if not any(token in _diagnostic_text(diagnostic) for token in ("table", "workbench", "work-bench", "bench")):
            continue
        name = _diagnostic_link_name(diagnostic) or str(diagnostic.get("object_id") or diagnostic.get("id") or "table/workbench")
        render_status = _diagnostic_status(diagnostic)
        expected = _diagnostic_expected_dimensions(diagnostic)
        mesh_uri = str(diagnostic.get("mesh_uri") or diagnostic.get("meshUri") or "")
        if expected and mesh_uri and render_status != "mesh_loaded":
            errors.append(f"browser viewer table/workbench {name} expected a loaded mesh but got render_status={render_status!r}")
        if expected:
            scale = _diagnostic_scale(diagnostic)
            if scale and _is_non_uniform(scale):
                errors.append(f"browser viewer table/workbench {name} has non-uniform mesh scale derived from expected_dimensions_m: x={scale[0]:.3f}, y={scale[1]:.3f}, z={scale[2]:.3f}")
    return errors


def _camera_bounds_errors(status: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    for diagnostic in _rendered_mesh_diagnostics_from_status(status):
        if not isinstance(diagnostic, Mapping):
            continue
        if not any(token in _diagnostic_text(diagnostic) for token in ("camera", "realsense", "sensor")):
            continue
        if _diagnostic_status(diagnostic) != "mesh_loaded":
            continue
        size = _diagnostic_bbox_size(diagnostic)
        expected = _diagnostic_expected_dimensions(diagnostic)
        excluded = _diagnostic_bool(diagnostic, "exclude_from_fit_bounds", "excludeFromFitBounds", "debug_overlay", "debugOverlay")
        if not size or not expected:
            continue
        ratios = [size[index] / expected[index] for index in range(3) if expected[index] > 0]
        if ratios and max(ratios) > 3.0 and not excluded:
            name = _diagnostic_link_name(diagnostic) or str(diagnostic.get("object_id") or diagnostic.get("id") or "camera")
            errors.append(f"browser viewer camera/Realsense {name} loaded mesh bounds are oversized and still included in normal product/fit bounds")
    return errors




def _diagnostic_support_surface_kind(diagnostic: Mapping[str, Any]) -> str:
    for key in (
        "support_surface_kind",
        "supportSurfaceKind",
        "semantic_type",
        "semanticType",
        "support_kind",
        "supportKind",
        "surface_kind",
        "surfaceKind",
        "support_surface_type",
        "supportSurfaceType",
    ):
        value = diagnostic.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip().lower().replace("-", "_").replace(" ", "_")
    return ""


def _diagnostic_float(diagnostic: Mapping[str, Any], *keys: str) -> float | None:
    for key in keys:
        value = diagnostic.get(key)
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            return number
    return None

def _diagnostic_up_axis(diagnostic: Mapping[str, Any]) -> tuple[float, float, float] | None:
    for key in ("inferred_up_axis", "inferredUpAxis", "inferred_normal", "inferredNormal", "normal", "up_axis", "upAxis"):
        vector = _diagnostic_vector(diagnostic.get(key))
        if vector is not None:
            length = _euclidean_distance(vector, (0.0, 0.0, 0.0))
            if length > 0.0:
                return tuple(component / length for component in vector)
    return None


def _table_horizontal_errors(status: Mapping[str, Any]) -> list[str]:
    diagnostics = _rendered_mesh_diagnostics_from_status(status)
    table_diagnostics = [
        raw for raw in diagnostics
        if isinstance(raw, Mapping) and any(token in _diagnostic_text(raw) for token in ("table", "workbench", "work-bench", "bench"))
    ]
    if not table_diagnostics:
        return ["browser viewer canonical table/workbench rendered bounding-box diagnostic is required"]

    errors: list[str] = []
    tabletop_kinds = {"table_surface", "tabletop"}
    body_kinds = {"workbench_body", "cabinet", "support_surface"}
    for diagnostic in table_diagnostics:
        name = _diagnostic_link_name(diagnostic) or str(diagnostic.get("object_id") or diagnostic.get("id") or "table/workbench")
        kind = _diagnostic_support_surface_kind(diagnostic)
        if not kind:
            errors.append(
                f"browser viewer table/workbench {name} missing explicit support-surface kind metadata; "
                "exporter must provide support_surface_kind/supportSurfaceKind or semantic_type/support_kind"
            )
            continue
        if kind not in tabletop_kinds | body_kinds:
            errors.append(f"browser viewer table/workbench {name} has unsupported support-surface kind {kind!r}")
            continue

        size = _diagnostic_bbox_size(diagnostic)
        if size is None:
            errors.append(f"browser viewer table/workbench {name} missing rendered bounding-box size")
            continue
        x, y, z = size

        if kind in tabletop_kinds:
            smallest_axis = min(((x, "X"), (y, "Y"), (z, "Z")), key=lambda item: item[0])[1]
            if smallest_axis != "Z":
                errors.append(f"browser viewer table/workbench {name} expected thickness/smallest dimension along Z, got size x={x:.3f}, y={y:.3f}, z={z:.3f}")
            if z >= min(x, y) or z > 0.20:
                errors.append(f"browser viewer table/workbench {name} expected thin tabletop Z and largest dimensions in XY, got size x={x:.3f}, y={y:.3f}, z={z:.3f}")
            if min(x, y) < 0.30 or max(x, y) < 0.50:
                errors.append(f"browser viewer table/workbench {name} expected wide tabletop XY footprint, got size x={x:.3f}, y={y:.3f}, z={z:.3f}")
            up = _diagnostic_up_axis(diagnostic)
            if up is None:
                errors.append(f"browser viewer table/workbench {name} missing inferred up axis/normal")
                continue
            dot_world_z = up[2]
            max_tilt_rad = math.radians(15.0)
            if dot_world_z < math.cos(max_tilt_rad):
                errors.append(f"browser viewer table/workbench {name} expected normal/up close to world +Z, got ({up[0]:.3f}, {up[1]:.3f}, {up[2]:.3f})")
            continue

        if min(x, y) < 0.30 or max(x, y) < 0.50:
            errors.append(f"browser viewer table/workbench {name} expected sensible support body XY footprint, got size x={x:.3f}, y={y:.3f}, z={z:.3f}")
        if not (0.20 <= z <= 1.50):
            errors.append(f"browser viewer table/workbench {name} expected plausible support body height Z, got size x={x:.3f}, y={y:.3f}, z={z:.3f}")
        support_height = _diagnostic_float(diagnostic, "top_surface_z_m", "topSurfaceZM", "support_surface_height_m", "supportSurfaceHeightM")
        if support_height is None:
            errors.append(
                f"browser viewer table/workbench {name} kind {kind!r} missing finite top/support height metadata; "
                "exporter must provide top_surface_z_m or support_surface_height_m"
            )
            continue
        center = _diagnostic_position(diagnostic, "loaded_mesh_bounding_box_center", "loadedMeshBoundingBoxCenter", "bounding_box_center", "boundingBoxCenter")
        if center is None:
            errors.append(f"browser viewer table/workbench {name} kind {kind!r} missing loaded bounding-box center needed to validate top/support height")
            continue
        loaded_top_z = center[2] + (z / 2.0)
        if abs(support_height - loaded_top_z) > 0.15:
            errors.append(
                f"browser viewer table/workbench {name} kind {kind!r} top/support height {support_height:.3f} m "
                f"is not plausibly near loaded bbox top {loaded_top_z:.3f} m"
            )
    return errors

def _diagnostic_link_name(diagnostic: Mapping[str, Any]) -> str:
    for key in ("link_name", "linkName", "link", "frame", "id", "display_name", "displayName"):
        value = diagnostic.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return ""


def _frame_diagnostic_name(diagnostic: Mapping[str, Any]) -> str:
    for key in ("frame_name", "frameName", "frame", "link_name", "linkName", "link", "id", "name"):
        value = diagnostic.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return ""


def _diagnostic_vector(value: Any) -> tuple[float, float, float] | None:
    if isinstance(value, Mapping):
        raw = (value.get("x"), value.get("y"), value.get("z"))
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)) and len(value) >= 3:
        raw = (value[0], value[1], value[2])
    else:
        return None
    try:
        vector = (float(raw[0]), float(raw[1]), float(raw[2]))
    except (TypeError, ValueError):
        return None
    if not all(component == component for component in vector):
        return None
    return vector


def _diagnostic_position(diagnostic: Mapping[str, Any], *keys: str) -> tuple[float, float, float] | None:
    for key in keys:
        vector = _diagnostic_vector(diagnostic.get(key))
        if vector is not None:
            return vector
    return None


def _pose_xyz(value: Any) -> tuple[float, float, float] | None:
    if isinstance(value, Mapping):
        for key in ("xyz", "position", "translation"):
            vector = _diagnostic_vector(value.get(key))
            if vector is not None:
                return vector
    return _diagnostic_vector(value)


def _expected_rendered_wrapper_center(diagnostic: Mapping[str, Any]) -> tuple[float, float, float] | None:
    mode = str(diagnostic.get("workcell_web_render_pose_mode") or diagnostic.get("workcellWebRenderPoseMode") or "").strip()
    if mode != "baked_visible_world_pose":
        return None
    for key in ("baked_world_visual_pose", "bakedWorldVisualPose", "expected_visual_pose", "expectedVisualPose", "final_transform", "finalTransform", "world_from_visual", "worldFromVisual"):
        vector = _pose_xyz(diagnostic.get(key))
        if vector is not None:
            return vector
    return None



def _has_baked_pose(diagnostic: Mapping[str, Any]) -> bool:
    for key in ("baked_world_visual_pose", "bakedWorldVisualPose", "expected_visual_pose", "expectedVisualPose", "final_transform", "finalTransform", "world_from_visual", "worldFromVisual"):
        if _pose_xyz(diagnostic.get(key)) is not None:
            return True
    return False


def _is_generated_urdf_mesh_diagnostic(diagnostic: Mapping[str, Any]) -> bool:
    text = _diagnostic_text(diagnostic)
    mesh_uri = str(diagnostic.get("mesh_uri") or diagnostic.get("meshUri") or "").strip()
    render_status = _diagnostic_status(diagnostic)
    has_mesh = bool(mesh_uri) or render_status == "mesh_loaded" or _diagnostic_bool(diagnostic, "mesh_loaded", "meshLoaded")
    generated = "urdf" in text or "generated" in text or any(token in text for token in ("robot", "gripper", "robotiq", "realsense", "workbench"))
    visual = any(token in text for token in ("visual", "mesh", "link", "robot", "tool", "gripper", "camera", "table", "workbench"))
    return has_mesh and generated and visual


def baked_pose_render_mode_summary(status: Mapping[str, Any]) -> dict[str, Any]:
    rows = [d for d in _rendered_mesh_diagnostics_from_status(status) if isinstance(d, Mapping) and _is_generated_urdf_mesh_diagnostic(d) and _has_baked_pose(d)]
    rendered = []
    mismatches = []
    for diagnostic in rows:
        mode = str(diagnostic.get("workcell_web_render_pose_mode") or diagnostic.get("workcellWebRenderPoseMode") or "").strip()
        exported_mode = str(diagnostic.get("exported_workcell_web_render_pose_mode") or diagnostic.get("exportedWorkcellWebRenderPoseMode") or "").strip()
        name = str(diagnostic.get("link") or diagnostic.get("object_name") or diagnostic.get("id") or diagnostic.get("display_name") or "unnamed")
        if mode == "baked_visible_world_pose":
            rendered.append(name)
        else:
            mismatches.append({"name": name, "mode": mode, "exported_mode": exported_mode})
    return {
        "generated_urdf_mesh_rows_with_baked_pose": len(rows),
        "rows_rendered_in_baked_visible_pose_mode": len(rendered),
        "rows_with_baked_pose_but_empty_or_legacy_render_mode": mismatches,
    }


def _assembled_hierarchy_mode_active(status: Mapping[str, Any]) -> bool:
    mode = str(status.get("robot_render_mode") or status.get("robotRenderMode") or "").strip()
    return mode in {"assembled_urdf_hierarchy", "verified_urdf_fk_visual_world_pose", "expanded_urdf_loader"}

def _expanded_urdf_fk_mode_active(status: Mapping[str, Any]) -> bool:
    mode = str(status.get("robot_render_mode") or status.get("robotRenderMode") or "").strip()
    return mode == "expanded_urdf_loader"


def _baked_pose_render_mode_errors(status: Mapping[str, Any]) -> list[str]:
    if _assembled_hierarchy_mode_active(status):
        return []
    summary = baked_pose_render_mode_summary(status)
    expected = int(summary["generated_urdf_mesh_rows_with_baked_pose"])
    actual = int(summary["rows_rendered_in_baked_visible_pose_mode"])
    if expected != actual:
        missing = ", ".join(row["name"] for row in summary["rows_with_baked_pose_but_empty_or_legacy_render_mode"][:20])
        return [f"browser viewer baked pose render mode mismatch: generated URDF mesh rows with baked pose={expected}, rendered in baked_visible_world_pose={actual}; missing/legacy rows: {missing or 'unknown'}"]
    return []


def _assembled_hierarchy_errors(status: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    if not _expanded_urdf_fk_mode_active(status):
        errors.append("browser viewer ur5_2f_test must use robot_render_mode=expanded_urdf_loader")
    links = status.get("robot_hierarchy_links") or status.get("robotHierarchyLinks") or []
    missing_links = status.get("robot_hierarchy_missing_links") or []
    missing_parents = status.get("robot_hierarchy_missing_parents") or []
    mesh_count = _status_int_any(status, "robot_loaded_visual_count", "robotLoadedVisualCount", "robot_hierarchy_mesh_count", "robotHierarchyMeshCount")
    required = ["base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0", "gripper_base_link"]
    absent = [link for link in required if link not in links]
    if absent:
        errors.append(f"browser viewer assembled hierarchy missing required links: {', '.join(absent)}")
    if missing_links:
        errors.append(f"browser viewer robot_hierarchy_missing_links must be empty, got {missing_links!r}")
    if missing_parents:
        errors.append(f"browser viewer robot_hierarchy_missing_parents must be empty, got {missing_parents!r}")
    if mesh_count < 16:
        errors.append(f"browser viewer robot_loaded_visual_count expected >= 16, got {mesh_count}")
    duplicate_count = _status_int(status, "visible_duplicate_generated_urdf_count", "visibleDuplicateGeneratedUrdfCount")
    if duplicate_count > 0:
        errors.append(f"browser viewer visible_duplicate_generated_urdf_count must be 0, got {duplicate_count}")
    if status.get("robot_preview_loaded") is not True:
        errors.append("browser viewer robot_preview_loaded must be true")
    if status.get("robot_missing_meshes"):
        errors.append(f"browser viewer robot_missing_meshes must be empty, got {status.get('robot_missing_meshes')!r}")
    tool0_fallback_count = _status_int(status, "visible_tool0_fallback_count", "visibleTool0FallbackCount")
    if tool0_fallback_count > 0:
        errors.append(f"browser viewer visible_tool0_fallback_count must be 0, got {tool0_fallback_count}")
    detached_clusters = _status_int_any(status, "detached_robot_mesh_clusters_count", "detachedRobotMeshClusters", "detached_robot_mesh_clusters")
    if detached_clusters > 0:
        errors.append(f"browser viewer detached_robot_mesh_clusters count must be 0, got {detached_clusters}")
    return errors


def _visual_acceptance_debug_dump(status: Mapping[str, Any]) -> dict[str, Any]:
    distances = status.get("viewer_resolved_distances_m") or status.get("resolvedFrameDistancesM") or {}
    max_distance = status.get("max_wrist_tool0_to_gripper_base_distance_m") or status.get("maxWristTool0ToGripperBaseDistanceM")
    if max_distance is None and isinstance(distances, Mapping):
        candidates = [
            distances.get("tool0 -> gripper_base_link"),
            distances.get("wrist_3_link -> gripper_base_link"),
        ]
        finite = []
        for value in candidates:
            try:
                finite.append(float(value))
            except (TypeError, ValueError):
                pass
        max_distance = max(finite) if finite else None
    return {
        "robot_transform_source": status.get("robot_transform_source") or status.get("robotTransformSource") or "",
        "robot_render_mode": status.get("robot_render_mode") or status.get("robotRenderMode") or "",
        "assembled_hierarchy_rendered_mesh_count": _status_int(status, "assembled_hierarchy_rendered_mesh_count", "assembledHierarchyRenderedMeshCount"),
        "skipped_flattened_urdf_visual_count": _status_int(status, "skipped_flattened_urdf_visual_count", "skippedFlattenedUrdfVisualCount"),
        "visible_duplicate_generated_urdf_count": _status_int(status, "visible_duplicate_generated_urdf_count", "visibleDuplicateGeneratedUrdfCount"),
        "visible_tool0_fallback_count": _status_int(status, "visible_tool0_fallback_count", "visibleTool0FallbackCount"),
        "detached_robot_mesh_clusters": _status_int_any(status, "detached_robot_mesh_clusters_count", "detachedRobotMeshClusters", "detached_robot_mesh_clusters"),
        "physical_assembly_root_count": _status_int_any(status, "physical_assembly_root_count", "physicalAssemblyRootCount"),
        "physical_fit_included_robot_preview": _status_bool(status, "physical_fit_included_robot_preview", "physicalFitIncludedRobotPreview"),
        "physical_assembly_bounds": _status_value(status, "physical_assembly_bounds", "physicalAssemblyBounds"),
        "final_physical_fit_bounds": _status_value(status, "final_physical_fit_bounds", "finalPhysicalFitBounds"),
        "max_distance_from_wrist_3_link_or_tool0_to_gripper_base_link_m": max_distance,
    }

def _euclidean_distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sum((left - right) ** 2 for left, right in zip(a, b)) ** 0.5


def _mesh_backing_errors(status: Mapping[str, Any]) -> list[str]:
    diagnostics = _rendered_mesh_diagnostics_from_status(status)
    table_seen = False
    table_loaded = False
    table_name = "table/workbench"
    camera_seen = False
    camera_loaded = False
    camera_name = "camera"
    errors: list[str] = []
    for diagnostic in diagnostics:
        if not isinstance(diagnostic, Mapping):
            continue
        text = _diagnostic_text(diagnostic)
        category = str(diagnostic.get("category") or "").lower()
        if category == "table" or any(token in text for token in ("table", "workbench", "work-bench", "bench")):
            if not table_seen:
                table_name = _diagnostic_link_name(diagnostic) or str(diagnostic.get("object_id") or diagnostic.get("id") or table_name)
            table_seen = True
            table_loaded = table_loaded or (
                _diagnostic_status(diagnostic) == "mesh_loaded"
                and _diagnostic_bool(diagnostic, "mesh_loaded", "meshLoaded") is True
            )
        if category == "camera" or any(token in text for token in ("camera", "realsense", "sensor")):
            if not camera_seen:
                camera_name = _diagnostic_link_name(diagnostic) or str(diagnostic.get("object_id") or diagnostic.get("id") or camera_name)
            camera_seen = True
            camera_loaded = camera_loaded or (
                _diagnostic_status(diagnostic) == "mesh_loaded"
                and _diagnostic_bool(diagnostic, "mesh_loaded", "meshLoaded") is True
            )
    if not table_seen:
        errors.append("browser viewer table/workbench mesh-backed diagnostic is required")
    elif not table_loaded:
        errors.append(f"browser viewer table/workbench {table_name} must remain mesh-backed; got no loaded mesh visual")
    if not camera_seen:
        errors.append("browser viewer camera/Realsense mesh-backed diagnostic is required")
    elif not camera_loaded:
        errors.append(f"browser viewer camera/Realsense {camera_name} must remain mesh-backed; got no loaded mesh visual")
    return errors


def _rendered_mesh_adjacency_errors(status: Mapping[str, Any]) -> list[str]:
    diagnostics = _rendered_mesh_diagnostics_from_status(status)
    if not diagnostics:
        return ["browser viewer renderedMeshDiagnostics/rendered_mesh_diagnostics is required for rendered UR5 mesh adjacency checks"]

    by_link: dict[str, Mapping[str, Any]] = {}
    for raw in diagnostics:
        if not isinstance(raw, Mapping):
            continue
        link_name = _diagnostic_link_name(raw)
        if link_name and link_name not in by_link:
            by_link[link_name] = raw

    errors: list[str] = []
    for pair, max_distance_m in REQUIRED_RENDERED_MESH_ADJACENT_PAIRS.items():
        left, right = [part.strip() for part in pair.split("->", 1)]
        left_diag = by_link.get(left)
        right_diag = by_link.get(right)
        if left_diag is None or right_diag is None:
            missing = [link for link, diag in ((left, left_diag), (right, right_diag)) if diag is None]
            errors.append(f"browser viewer rendered mesh adjacency {pair} missing required link diagnostics: {', '.join(missing)}")
            continue
        for label, keys in (
            ("visual wrapper", ("visual_wrapper_world_position", "visualWrapperWorldPosition")),
            ("loaded mesh bounds center", ("loaded_mesh_bounding_box_center", "loadedMeshBoundingBoxCenter", "bounding_box_center", "boundingBoxCenter")),
        ):
            left_position = _diagnostic_position(left_diag, *keys)
            right_position = _diagnostic_position(right_diag, *keys)
            if left_position is None or right_position is None:
                errors.append(f"browser viewer rendered mesh adjacency {pair} missing {label} diagnostics")
                continue
            distance = _euclidean_distance(left_position, right_position)
            if distance > max_distance_m:
                errors.append(f"browser viewer rendered mesh adjacency {pair} {label} expected <= {max_distance_m:.3f} m, got {distance:.3f} m")

        for link_name, diagnostic in ((left, left_diag), (right, right_diag)):
            expected = _expected_rendered_wrapper_center(diagnostic)
            actual = _diagnostic_position(diagnostic, "visual_wrapper_world_position", "visualWrapperWorldPosition")
            if expected is None or actual is None:
                continue
            error = _euclidean_distance(expected, actual)
            if error > 0.005:
                errors.append(
                    f"browser viewer rendered mesh {link_name} baked_visible_world_pose wrapper center expected "
                    f"({expected[0]:.3f}, {expected[1]:.3f}, {expected[2]:.3f}), got "
                    f"({actual[0]:.3f}, {actual[1]:.3f}, {actual[2]:.3f}); visual origin may have been applied twice"
                )
    return errors


def _tool0_frame_contract_errors(status: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []

    resolved_positions = _resolved_frame_positions_from_status(status)
    if "tool0" not in resolved_positions:
        errors.append("browser viewer resolvedFramePositions/resolved_frame_positions must include tool0")

    frame_diagnostics = _frame_diagnostics_from_status(status)
    tool0_diagnostic_seen = False
    if isinstance(frame_diagnostics, Mapping):
        tool0_diagnostic_seen = "tool0" in frame_diagnostics
    else:
        for raw in frame_diagnostics:
            if isinstance(raw, Mapping) and _frame_diagnostic_name(raw) == "tool0":
                tool0_diagnostic_seen = True
                break
    if not tool0_diagnostic_seen:
        errors.append("browser viewer frameDiagnostics/frame_diagnostics must include tool0")

    return errors



def _browser_matrix_contract_errors(status: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    matrices = status.get("robot_link_world_matrices") or status.get("robotLinkWorldMatrices") or {}
    if not isinstance(matrices, Mapping):
        return ["browser viewer must expose robot_link_world_matrices from actual Object3D matrixWorld values"]
    absent = [link for link in REQUIRED_BROWSER_MATRIX_LINKS if link not in matrices]
    # flange is optional, but if loaded it must have a matrix.
    hierarchy = status.get("robot_hierarchy_links") or status.get("robotHierarchyLinks") or []
    if isinstance(hierarchy, list) and "flange" in hierarchy and "flange" not in matrices:
        absent.append("flange")
    if absent:
        errors.append(f"browser viewer actual Object3D matrix diagnostics missing required links: {', '.join(absent)}")
    for link, diagnostic in matrices.items():
        matrix = diagnostic.get("matrix_world") if isinstance(diagnostic, Mapping) else None
        if not (isinstance(matrix, list) and len(matrix) == 16 and all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in matrix)):
            errors.append(f"browser viewer Object3D matrixWorld for {link} must be a finite 16-number matrix")
    visual_matrices = status.get("robot_visual_wrapper_world_matrices") or status.get("robotVisualWrapperWorldMatrices") or []
    collada_diagnostics = status.get("robot_collada_mesh_diagnostics") or status.get("robotColladaMeshDiagnostics") or []
    if (not isinstance(visual_matrices, list) or not visual_matrices) and not (str(_status_value(status, "robot_render_mode", "robotRenderMode") or "") == "expanded_urdf_loader" and isinstance(collada_diagnostics, list) and collada_diagnostics):
        errors.append("browser viewer must expose robot_visual_wrapper_world_matrices or robot_collada_mesh_diagnostics for T_world_link × T_link_visual wrappers")
    elif isinstance(visual_matrices, list) and visual_matrices and str(_status_value(status, "robot_render_mode", "robotRenderMode") or "") != "expanded_urdf_loader":
        visual_links = {str(row.get("link_name") or row.get("linkName") or "") for row in visual_matrices if isinstance(row, Mapping)}
        mesh_links = [link for link in REQUIRED_BROWSER_MATRIX_LINKS if link != "tool0"]
        missing_visuals = [link for link in mesh_links if link not in visual_links]
        if missing_visuals:
            errors.append(f"browser viewer visual-wrapper matrix diagnostics missing required mesh links: {', '.join(missing_visuals)}")
    return errors


def _matrix_translation(value: Any) -> tuple[float, float, float] | None:
    matrix = value.get("matrix_world") if isinstance(value, Mapping) else None
    if not (isinstance(matrix, list) and len(matrix) == 16):
        return None
    try:
        xyz = (float(matrix[12]), float(matrix[13]), float(matrix[14]))
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in xyz):
        return None
    return xyz

def validate_browser_status(status: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    mesh_loaded_count = _status_int(status, "mesh_loaded_count", "meshLoadedCount")
    required_mesh_failed_count = _status_int(status, "required_mesh_failed_count", "requiredMeshFailedCount")
    robot_loaded_visual_count = _status_int_any(status, "robot_loaded_visual_count", "robotLoadedVisualCount")
    robot_expected_visual_count = _status_int_any(status, "robot_expected_visual_count", "robotExpectedVisualCount")
    robot_completed_visual_count = _status_int_any(status, "robot_completed_visual_count", "robotCompletedVisualCount")
    robot_failed_visual_count = _status_int_any(status, "robot_failed_visual_count", "robotFailedVisualCount")
    robot_callbacks_complete = _status_bool(status, "robot_mesh_callbacks_complete", "robotMeshCallbacksComplete")
    render_mode = str(_status_value(status, "robot_render_mode", "robotRenderMode") or "")
    expected_scene_mesh_raw = _status_value(
        status, "expected_scene_mesh_loaded_count", "expectedSceneMeshLoadedCount"
    )
    expected_scene_mesh_loaded_count = _status_int_any(
        status, "expected_scene_mesh_loaded_count", "expectedSceneMeshLoadedCount"
    )
    if render_mode == "expanded_urdf_loader" and expected_scene_mesh_raw is not None:
        if robot_expected_visual_count != 16:
            errors.append(f"browser viewer robot_expected_visual_count expected 16, got {robot_expected_visual_count}")
        if robot_completed_visual_count != 16:
            errors.append(f"browser viewer robot_completed_visual_count expected 16, got {robot_completed_visual_count}")
        if robot_loaded_visual_count != 16:
            errors.append(f"browser viewer robot_loaded_visual_count expected 16, got {robot_loaded_visual_count}")
        if robot_failed_visual_count != 0:
            errors.append(f"browser viewer robot_failed_visual_count expected 0, got {robot_failed_visual_count}")
        if robot_callbacks_complete is not True:
            errors.append(f"browser viewer robot_mesh_callbacks_complete expected true, got {robot_callbacks_complete!r}")
        if mesh_loaded_count != expected_scene_mesh_loaded_count:
            errors.append(
                "browser viewer scene mesh_loaded_count expected "
                f"{expected_scene_mesh_loaded_count}, got {mesh_loaded_count}"
            )
        expected_total = expected_scene_mesh_loaded_count + robot_expected_visual_count
        actual_total = mesh_loaded_count + robot_loaded_visual_count
        if actual_total != expected_total:
            errors.append(
                "browser viewer scene plus robot loaded visual count expected "
                f"{expected_total}, got meshLoadedCount={mesh_loaded_count}, "
                f"robot_loaded_visual_count={robot_loaded_visual_count}"
            )
    else:
        # Preserve the legacy 18-mesh artifact contract for older viewer
        # statuses that do not yet export a scene-specific expectation.
        effective_mesh_loaded_count = mesh_loaded_count if mesh_loaded_count >= EXPECTED_MESH_LOADED_COUNT else mesh_loaded_count + robot_loaded_visual_count
        if render_mode == "expanded_urdf_loader" and mesh_loaded_count < EXPECTED_MESH_LOADED_COUNT:
            if robot_expected_visual_count != 16:
                errors.append(f"browser viewer robot_expected_visual_count expected 16, got {robot_expected_visual_count}")
            if robot_completed_visual_count != 16:
                errors.append(f"browser viewer robot_completed_visual_count expected 16, got {robot_completed_visual_count}")
            if robot_loaded_visual_count != 16:
                errors.append(f"browser viewer robot_loaded_visual_count expected 16, got {robot_loaded_visual_count}")
            if robot_failed_visual_count != 0:
                errors.append(f"browser viewer robot_failed_visual_count expected 0, got {robot_failed_visual_count}")
            if robot_callbacks_complete is not True:
                errors.append(f"browser viewer robot_mesh_callbacks_complete expected true, got {robot_callbacks_complete!r}")
            if mesh_loaded_count != 2:
                errors.append(f"browser viewer scene mesh_loaded_count expected 2, got {mesh_loaded_count}")
        if effective_mesh_loaded_count != EXPECTED_MESH_LOADED_COUNT:
            errors.append(f"browser viewer meshLoadedCount plus robot_loaded_visual_count expected {EXPECTED_MESH_LOADED_COUNT}, got meshLoadedCount={mesh_loaded_count}, robot_loaded_visual_count={robot_loaded_visual_count}")
    collada_diagnostics = status.get("robot_collada_mesh_diagnostics") or status.get("robotColladaMeshDiagnostics") or []
    collada_text = json.dumps(collada_diagnostics).lower() if isinstance(collada_diagnostics, list) else ""
    missing_robotiq = [name for name in REQUIRED_ROBOTIQ_MESH_BASENAMES if name.lower() not in collada_text]
    if render_mode == "expanded_urdf_loader" and (
        expected_scene_mesh_raw is not None or mesh_loaded_count < EXPECTED_MESH_LOADED_COUNT
    ) and missing_robotiq:
        errors.append("browser viewer robot_collada_mesh_diagnostics missing Robotiq mesh basenames: " + ", ".join(missing_robotiq))
    if required_mesh_failed_count != EXPECTED_REQUIRED_MESH_FAILED_COUNT:
        errors.append(f"browser viewer requiredMeshFailedCount expected {EXPECTED_REQUIRED_MESH_FAILED_COUNT}, got {required_mesh_failed_count}")

    distances = dict(_distance_map_from_status(status))
    matrices = status.get("robot_link_world_matrices") or status.get("robotLinkWorldMatrices") or {}
    if isinstance(matrices, Mapping):
        for pair in REQUIRED_VIEWER_RESOLVED_DISTANCE_PAIRS:
            if pair in distances:
                continue
            left, right = [part.strip() for part in pair.split("->", 1)]
            left_pos = _matrix_translation(matrices.get(left))
            right_pos = _matrix_translation(matrices.get(right))
            if left_pos is not None and right_pos is not None:
                distances[pair] = _euclidean_distance(left_pos, right_pos)
    for pair, max_distance_m in REQUIRED_VIEWER_RESOLVED_DISTANCE_PAIRS.items():
        raw = distances.get(pair)
        try:
            distance = float(raw) if raw is not None else float("nan")
        except (TypeError, ValueError):
            distance = float("nan")
        if not (0.0 <= distance <= max_distance_m):
            errors.append(f"browser viewer resolved distance {pair} expected <= {max_distance_m:.3f} m, got {raw!r}")
    errors.extend(_robot_lifecycle_errors(status))
    errors.extend(_browser_matrix_contract_errors(status))
    errors.extend(_assembled_hierarchy_errors(status))
    errors.extend(_tool0_frame_contract_errors(status))
    rendered_diags = _rendered_mesh_diagnostics_from_status(status)
    has_rendered_robot_diags = any(isinstance(d, Mapping) and _diagnostic_link_name(d) in {p.split("->", 1)[0].strip() for p in REQUIRED_RENDERED_MESH_ADJACENT_PAIRS} for d in rendered_diags)
    if (not _expanded_urdf_fk_mode_active(status)) or has_rendered_robot_diags:
        errors.extend(_rendered_mesh_adjacency_errors(status))
    errors.extend(_mesh_backing_errors(status))
    errors.extend(_table_horizontal_errors(status))
    errors.extend(_required_product_fallback_errors(status))
    errors.extend(_table_mesh_contract_errors(status))
    errors.extend(_camera_bounds_errors(status))
    errors.extend(_baked_pose_render_mode_errors(status))
    errors.extend(_physical_fit_errors(status))
    return errors


def _load_yaml(path: Path) -> Mapping[str, Any]:
    if not path.is_file():
        return {}
    try:
        import yaml  # type: ignore
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return data if isinstance(data, Mapping) else {}


def _first_text(*values: Any) -> str | None:
    for value in values:
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def _safe_scene_id(raw: str) -> str:
    cleaned = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in raw.strip())
    return cleaned.strip("._-") or "scene"


def derive_scene_id(scene_dir: Path, override: str | None = None) -> str:
    if override:
        return _safe_scene_id(override)
    manifest = _load_yaml(scene_dir / "scene_manifest.yaml")
    environment = _load_yaml(scene_dir / "environment.yaml")
    manifest_scene = manifest.get("scene") if isinstance(manifest.get("scene"), Mapping) else {}
    env_scene = environment.get("scene") if isinstance(environment.get("scene"), Mapping) else {}
    env_root = environment.get("environment") if isinstance(environment.get("environment"), Mapping) else {}
    return _safe_scene_id(
        _first_text(
            manifest_scene.get("id"), manifest_scene.get("name"),
            env_scene.get("id"), env_scene.get("name"),
            env_root.get("scene_id"), env_root.get("id"), env_root.get("name"),
            scene_dir.name,
        ) or scene_dir.name
    )


def repo_relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def run_step(command: list[str]) -> dict[str, Any]:
    result = subprocess.run(command, cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    return {"command": command, "returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr}


def is_port_open(port: int) -> bool:
    with socket.socket() as sock:
        sock.settimeout(0.25)
        try:
            sock.connect(("127.0.0.1", port))
            return True
        except OSError:
            return False


def start_or_reuse_server(port: int) -> tuple[str, ThreadingHTTPServer | None]:
    if is_port_open(port):
        return "reused", None

    class Handler(SimpleHTTPRequestHandler):
        def log_message(self, fmt: str, *args: Any) -> None:  # keep acceptance output concise
            return

    server = ThreadingHTTPServer(("127.0.0.1", port), lambda *a, **kw: Handler(*a, directory=str(REPO_ROOT), **kw))
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    deadline = time.time() + 2.0
    while time.time() < deadline:
        if is_port_open(port):
            return "started", server
        time.sleep(0.05)
    return "failed", server


def browser_command(url: str, screenshot: Path) -> dict[str, Any] | None:
    for binary in ("chromium", "chromium-browser", "google-chrome", "google-chrome-stable"):
        exe = shutil.which(binary)
        if exe:
            return {
                "command": [exe, "--headless=new", "--disable-gpu", "--no-sandbox", "--ignore-certificate-errors", f"--screenshot={screenshot}", "--window-size=1280,900", url],
                "kind": binary,
            }
    return None


def _request_failure_error_text(failure: Any) -> str:
    if failure is None:
        return ""
    if isinstance(failure, str):
        return failure
    if isinstance(failure, Mapping):
        for key in ("error_text", "errorText", "message", "error"):
            value = failure.get(key)
            if value is not None:
                return str(value)
        return str(dict(failure))
    try:
        value = getattr(failure, "error_text")
    except Exception as exc:
        return f"<unreadable failure.error_text: {exc}>"
    return "" if value is None else str(value)


def _safe_request_field(request: Any, field: str) -> str:
    try:
        value = getattr(request, field)
    except Exception as exc:
        return f"<unreadable {field}: {exc}>"
    return "" if value is None else str(value)


def _record_failed_request(request: Any, failed_requests: list[str]) -> None:
    try:
        method = _safe_request_field(request, "method")
        url = _safe_request_field(request, "url")
        try:
            failure = getattr(request, "failure", None)
        except Exception as exc:
            failure = f"<unreadable failure: {exc}>"
        failed_requests.append(f"{method} {url} {_request_failure_error_text(failure)}")
    except Exception as exc:
        try:
            failed_requests.append(f"<requestfailed callback error: {exc}>")
        except Exception:
            pass


def _record_successful_response(response: Any, successful_responses: set[tuple[str, str]]) -> None:
    """Remember successful responses so Playwright event pairs can be reconciled."""
    try:
        status = int(getattr(response, "status", 0))
        request = getattr(response, "request", None)
        method = _safe_request_field(request, "method").upper()
        url = _safe_request_field(request, "url")
        if 200 <= status < 400 and method and url:
            successful_responses.add((method, url))
    except Exception:
        pass


def _reconcile_successful_head_aborts(
    failed_requests: list[str], successful_responses: set[tuple[str, str]]
) -> list[str]:
    """Drop Chromium's false requestfailed event for a successful HEAD response.

    Python's SimpleHTTPRequestHandler correctly returns the GET Content-Length
    and no body for HEAD. Chromium resolves fetch() with the successful response
    but Playwright also emits net::ERR_ABORTED for the absent body. Only that
    paired, successful HEAD event is non-failing; every other network failure is
    retained.
    """
    successful_head_urls = {
        url for method, url in successful_responses if method == "HEAD"
    }
    return [
        failure
        for failure in failed_requests
        if not (
            "net::ERR_ABORTED" in failure
            and any(failure.startswith(f"HEAD {url} ") for url in successful_head_urls)
        )
    ]


def run_browser(url: str, status_path: Path, screenshot_path: Path, require: bool) -> dict[str, Any]:
    js_errors: list[str] = []
    failed_requests: list[str] = []
    successful_responses: set[tuple[str, str]] = set()
    console_messages: list[str] = []
    try:
        from playwright.sync_api import sync_playwright  # type: ignore
        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True, args=["--ignore-certificate-errors"])
            page = browser.new_page(viewport={"width": 1280, "height": 900}, ignore_https_errors=True)
            page.on("pageerror", lambda exc: js_errors.append(str(exc)))
            page.on("console", lambda msg: console_messages.append(f"{msg.type}: {msg.text}"))
            page.on("requestfailed", lambda req: _record_failed_request(req, failed_requests))
            page.on("response", lambda response: _record_successful_response(response, successful_responses))
            page.goto(url, wait_until="networkidle", timeout=45000)
            page.wait_for_function("window.__WORKCELL_VIEWER_STATUS__ && typeof window.__WORKCELL_VIEWER_STATUS__ === 'object'", timeout=45000)
            page.wait_for_function("() => { const s = window.__WORKCELL_VIEWER_STATUS__ || {}; const state = s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || ''; return state === 'scene_ready' || state === 'scene_failed'; }", timeout=60000)
            page.wait_for_function("window.__WORKCELL_ROBOT_PREVIEW_READY__ && typeof window.__WORKCELL_ROBOT_PREVIEW_READY__.then === 'function'", timeout=45000)
            page.evaluate("() => window.__WORKCELL_ROBOT_PREVIEW_READY__.then(() => true)")
            page.wait_for_function("() => { const s = window.__WORKCELL_VIEWER_STATUS__ || {}; const state = s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || ''; return state === 'ready' || state === 'failed' || (s.web3d_readiness_state || s.web3dReadinessState) === 'scene_failed'; }", timeout=45000)
            status = page.evaluate("window.__WORKCELL_VIEWER_STATUS__ || null")
            final_state = page.evaluate("() => (window.__WORKCELL_VIEWER_STATUS__ || {}).robot_preview_lifecycle_state || (window.__WORKCELL_VIEWER_STATUS__ || {}).robotPreviewLifecycleState || ''")
            screenshot_before_ready = final_state not in {'ready', 'failed'}
            page.screenshot(path=str(screenshot_path), full_page=True)
            browser.close()
        failed_requests = _reconcile_successful_head_aborts(failed_requests, successful_responses)
        return {"available": True, "method": "playwright", "status": status, "screenshot_before_ready": screenshot_before_ready, "javascript_errors": js_errors, "failed_network_requests": failed_requests, "console_messages": console_messages[-100:]}
    except Exception as exc:
        playwright_error = str(exc)

    command_spec = browser_command(url, screenshot_path)
    if command_spec:
        result = subprocess.run(command_spec["command"], cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        return {"available": result.returncode == 0, "method": command_spec["kind"], "status": None, "returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr, "playwright_error": playwright_error, "javascript_errors": js_errors, "failed_network_requests": failed_requests}
    if require:
        raise RuntimeError(f"No usable browser found. Playwright error: {playwright_error}")
    screenshot_path.write_bytes(PNG_1X1)
    return {"available": False, "method": "skipped", "status": None, "reason": f"No Playwright/chromium/google-chrome browser available. Playwright error: {playwright_error}", "javascript_errors": js_errors, "failed_network_requests": failed_requests}


SCENE_REPRODUCIBILITY_SCHEMA = "workcell_studio_supported_scene_reproducibility/v1"


def _load_supported_entries(catalog_path: Path | None = None) -> tuple[list[Any], list[str]]:
    if str(REPO_ROOT / "scripts") not in sys.path:
        sys.path.insert(0, str(REPO_ROOT / "scripts"))
    from supported_scene_catalog import default_catalog_path, load_supported_scene_catalog
    path = catalog_path or default_catalog_path(REPO_ROOT)
    _catalog, entries, errors = load_supported_scene_catalog(path)
    return entries, errors


def _step_status(ok: bool, reason: str = "") -> dict[str, Any]:
    return {"status": "PASS" if ok else "FAIL", "reason": reason}


def _check_render_ownership(web_scene_path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(web_scene_path.read_text(encoding="utf-8"))
    except Exception as exc:
        return _step_status(False, f"could not read staged web scene JSON {repo_relative(web_scene_path)}: {exc}")
    summary = payload.get("render_ownership_summary") if isinstance(payload, Mapping) else None
    if not isinstance(summary, Mapping):
        return _step_status(False, "staged web scene missing render_ownership_summary")
    unknown = int(summary.get("unknown_physical_owners") or 0)
    duplicate = int(summary.get("duplicate_primary_identities") or 0)
    if unknown or duplicate:
        return _step_status(False, f"render ownership has unknown={unknown} duplicate_primary={duplicate}")
    return {"status": "PASS", "summary": dict(summary)}


def _check_readiness_metadata(scene_dir: Path, entry: Any) -> dict[str, Any]:
    paths = [scene_dir / "generated/generated_workcell_summary.json", scene_dir / "generated/scene_package_readiness.json"]
    missing = [repo_relative(p) for p in paths if not p.is_file()]
    if missing:
        return _step_status(False, "missing robot/tool readiness metadata: " + ", ".join(missing))
    try:
        summary = json.loads(paths[0].read_text(encoding="utf-8"))
    except Exception as exc:
        return _step_status(False, f"could not parse {repo_relative(paths[0])}: {exc}")
    missing_fields = [key for key in ("robot", "end_effector", "grasp_strategy") if not summary.get(key)]
    missing_caps = [cap for cap in getattr(entry, "required_capabilities", ()) if cap not in json.dumps(summary).lower() and cap not in json.dumps(getattr(entry, "raw", {})).lower()]
    if missing_fields:
        return _step_status(False, "generated readiness metadata missing fields: " + ", ".join(missing_fields))
    return {"status": "PASS", "robot": getattr(entry, "robot", ""), "tool": getattr(entry, "tool", ""), "required_capabilities": list(getattr(entry, "required_capabilities", ())), "unmatched_registry_capabilities": missing_caps}


def evaluate_supported_scene_reproducibility(entry: Any, *, output_root: Path, require_browser: bool = False, port: int = 8765) -> dict[str, Any]:
    scene_dir = (REPO_ROOT / entry.scene_path).resolve()
    row: dict[str, Any] = {"scene_id": entry.scene_name, "robot": entry.robot, "tool": entry.tool, "required_capabilities": list(entry.required_capabilities), "status": "PASS", "blocker_reason": "", "checks": {}}
    if entry.status == "blocked" or not entry.enabled:
        reason = entry.known_blocker or "catalog marks scene blocked/disabled without an explicit blocker reason"
        row.update(status="BLOCKED", blocker_reason=reason)
        row["checks"]["catalog_status"] = {"status": "BLOCKED", "reason": reason}
        return row
    missing_source = [rel for rel in entry.authoring_files if not (scene_dir / rel).is_file()]
    row["checks"]["required_source_files"] = _step_status(not missing_source, "missing: " + ", ".join(missing_source) if missing_source else "")
    gen_root = (output_root / entry.scene_name).resolve()
    cmd = [sys.executable, "scripts/generate_workcell_from_cell_definition.py", repo_relative(scene_dir / "cell_definition.yaml"), "--output-dir", repo_relative(gen_root.parent), "--package-name", entry.package_name, "--force"]
    row["checks"]["scene_generation"] = run_step(cmd)
    generated_scene = gen_root.parent / entry.package_name
    if row["checks"]["scene_generation"]["returncode"] == 0:
        val_cmd = [sys.executable, "scripts/validate_builder_generated_scene.py", repo_relative(generated_scene), "--json"]
        row["checks"]["schema_validation"] = run_step(val_cmd)
        web_json = BUILD_ROOT / f"{entry.scene_name}.web_scene.json"
        row["checks"]["web3d_acceptance_tooling"] = run_step([sys.executable, "scripts/ensure_workcell_studio_web_scene_fresh.py", "--scene", repo_relative(generated_scene), "--output", repo_relative(web_json), "--stage-assets"])
        if row["checks"]["web3d_acceptance_tooling"]["returncode"] == 0:
            row["checks"]["staged_urdf_meshes"] = run_step([sys.executable, "scripts/check_workcell_web_scene_mesh_contract.py", repo_relative(web_json)])
            row["checks"]["visual_bounds"] = run_step([sys.executable, "scripts/check_workcell_web_scene_visual_bounds.py", repo_relative(web_json), "--json"])
            row["checks"]["render_ownership"] = _check_render_ownership(web_json)
        row["checks"]["robot_tool_readiness_metadata"] = _check_readiness_metadata(generated_scene, entry)
        missing_launch = [rel for rel in ("launch/demo.launch.py", "urdf/scene.urdf.xacro") if not (generated_scene / rel).is_file()]
        row["checks"]["fake_hardware_launch_files"] = _step_status(not missing_launch and "use_fake_hardware:=true" in entry.fake_hardware_launch_command, "missing/unguarded launch contract: " + ", ".join(missing_launch) if missing_launch else "")
    failed = []
    for name, check in row["checks"].items():
        if check.get("status") == "FAIL" or check.get("returncode", 0) != 0:
            failed.append(name)
    if failed:
        row["status"] = "FAIL"
        row["failure_reason"] = "failed checks: " + ", ".join(failed)
    return row


def run_supported_scene_reproducibility_gate(*, output: Path | None = None, catalog: Path | None = None, scene_ids: Sequence[str] | None = None, require_browser: bool = False, port: int = 8765) -> dict[str, Any]:
    entries, catalog_errors = _load_supported_entries(catalog)
    if scene_ids:
        wanted = set(scene_ids)
        entries = [e for e in entries if e.scene_name in wanted]
    out_root = REPO_ROOT / "build" / "workcell_studio_supported_scene_reproducibility"
    out_root.mkdir(parents=True, exist_ok=True)
    rows = [evaluate_supported_scene_reproducibility(e, output_root=out_root, require_browser=require_browser, port=port) for e in entries]
    counts = {"PASS": 0, "FAIL": 0, "BLOCKED": 0}
    for row in rows:
        counts[row["status"]] += 1
    report = {"schema": SCENE_REPRODUCIBILITY_SCHEMA, "status": "FAIL" if counts["FAIL"] or catalog_errors else "PASS", "counts": counts, "catalog_errors": catalog_errors, "scenes": rows}
    if output:
        out = output if output.is_absolute() else (REPO_ROOT / output)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print("Supported scene reproducibility gate")
    for row in rows:
        reason = row.get("blocker_reason") or row.get("failure_reason") or ""
        print(f"{row['status']}: {row['scene_id']} robot={row['robot']} tool={row['tool']} {reason}")
    print(json.dumps({"status": report["status"], "counts": counts}, sort_keys=True))
    return report

def _entry_by_scene_name(entries: Sequence[Any]) -> dict[str, Any]:
    return {str(getattr(entry, "scene_name", "")): entry for entry in entries}


def _scene_expectations(entry: Any | None, scene_dir: Path) -> dict[str, str]:
    robot = str(getattr(entry, "robot", "") or "") if entry is not None else ""
    tool = str(getattr(entry, "tool", "") or "") if entry is not None else ""
    if robot and tool:
        return {"robot": robot, "tool": tool}
    cell = _load_yaml(scene_dir / "cell_definition.yaml")
    for key in ("robot", "robot_model", "robot_type"):
        value = cell.get(key)
        if isinstance(value, str) and value.strip() and not robot:
            robot = value.strip()
        elif isinstance(value, Mapping) and not robot:
            robot = _first_text(value.get("model"), value.get("type"), value.get("name")) or robot
    for key in ("end_effector", "tool", "gripper"):
        value = cell.get(key)
        if isinstance(value, str) and value.strip() and not tool:
            tool = value.strip()
        elif isinstance(value, Mapping) and not tool:
            tool = _first_text(value.get("model"), value.get("type"), value.get("name")) or tool
    return {"robot": robot or "unknown", "tool": tool or "unknown"}


def _count_status_links(status: Mapping[str, Any], expected_tool: str) -> tuple[int, int]:
    links = [str(x) for x in _status_list(status, "robot_hierarchy_links", "robotHierarchyLinks")]
    robot_links = [x for x in links if x and not any(tok in x.lower() for tok in ("gripper", "finger", "suction", "vacuum", "airpick"))]
    tool_links = [x for x in links if x and x not in robot_links]
    return len(robot_links), len(tool_links)


def _missing_required_link_lists(status: Mapping[str, Any]) -> tuple[list[Any], list[Any]]:
    robot_missing = _status_list(status, "robot_missing_required_robot_visual_links", "robotMissingRequiredRobotVisualLinks", "missing_required_robot_visuals")
    if not robot_missing:
        robot_missing = _status_list(status, "robot_hierarchy_missing_links", "robotHierarchyMissingLinks")
    tool_missing = _status_list(status, "robot_missing_required_tool_visual_links", "robotMissingRequiredToolVisualLinks", "missing_required_tool_visuals")
    return robot_missing, tool_missing


def _matrix_row_from_report(report: Mapping[str, Any], expected: Mapping[str, str], require_browser: bool) -> dict[str, Any]:
    browser = report.get("browser") if isinstance(report.get("browser"), Mapping) else {}
    status = browser.get("status") if isinstance(browser.get("status"), Mapping) else {}
    robot_links, tool_links = _count_status_links(status, expected.get("tool", "")) if isinstance(status, Mapping) else (0, 0)
    missing_robot, missing_tool = _missing_required_link_lists(status) if isinstance(status, Mapping) else ([], [])
    missing_meshes = _status_list(status, "robot_missing_meshes", "robotMissingMeshes") if isinstance(status, Mapping) else []
    unresolved = _status_list(status, "unresolved_package_uris", "unresolvedPackageUris") if isinstance(status, Mapping) else []
    failed_mesh_urls = _status_list(status, "failed_mesh_urls", "failedMeshUrls", "robot_failed_mesh_urls", "robotFailedMeshUrls") if isinstance(status, Mapping) else []
    terminal_stage = str(_status_value(status, "web3d_readiness_state", "web3dReadinessState", "viewer_boot_state", "viewerBootState") or "not_run") if isinstance(status, Mapping) else "not_run"
    errors: list[str] = []
    if require_browser and browser.get("available") is not True:
        errors.append("browser unavailable; acceptance is BLOCKED")
    if browser.get("method") != "playwright":
        errors.append(f"real Playwright browser required, got {browser.get('method')!r}")
    if terminal_stage != "scene_ready":
        errors.append(f"terminal stage must be scene_ready, got {terminal_stage!r}")
    if _status_bool(status, "robot_preview_loaded", "robotPreviewLoaded") is False:
        errors.append("robot_preview_loaded is false")
    if _status_int_any(status, "robot_failed_visual_count", "robotFailedVisualCount") != 0:
        errors.append("robot_failed_visual_count is non-zero")
    if missing_robot:
        errors.append("missing required robot links: " + ", ".join(map(str, missing_robot)))
    if missing_tool:
        errors.append("missing required tool links: " + ", ".join(map(str, missing_tool)))
    if missing_meshes:
        errors.append("missing meshes: " + ", ".join(map(str, missing_meshes[:10])))
    if unresolved:
        errors.append("unresolved assets: " + ", ".join(map(str, unresolved[:10])))
    if browser.get("javascript_errors"):
        errors.append("JavaScript errors: " + "; ".join(map(str, browser.get("javascript_errors")[:5])))
    if browser.get("failed_network_requests"):
        errors.append("failed network requests: " + "; ".join(map(str, browser.get("failed_network_requests")[:5])))
    if any(step.get("returncode", 0) != 0 for step in report.get("steps", []) if isinstance(step, Mapping)):
        errors.append("one or more staging/static checks failed")
    errors.extend(str(error) for error in report.get("strict_status_errors", []) if error)
    return {
        "scene_id": report.get("scene_id", ""),
        "status": "BLOCKED" if errors and browser.get("available") is not True else ("FAIL" if errors else "PASS"),
        "terminal_stage": terminal_stage,
        "expected_robot": expected.get("robot", "unknown"),
        "expected_tool": expected.get("tool", "unknown"),
        "rendered_robot_link_count": robot_links,
        "rendered_tool_link_count": tool_links,
        "failed_mesh_urls": failed_mesh_urls,
        "missing_required_robot_links": missing_robot,
        "missing_required_tool_links": missing_tool,
        "missing_mesh_count": len(missing_meshes),
        "missing_meshes": missing_meshes,
        "unresolved_asset_count": len(unresolved),
        "unresolved_assets": unresolved,
        "javascript_errors": browser.get("javascript_errors", []),
        "failed_network_requests": browser.get("failed_network_requests", []),
        "screenshot": report.get("screenshot", "") if browser.get("available") is True else "",
        "errors": errors,
    }


def run_one_scene(scene_dir: Path, *, output_path: Path | None, scene_id: str | None, require_browser: bool, port: int, expected: Mapping[str, str] | None = None) -> tuple[int, dict[str, Any]]:
    scene_id = derive_scene_id(scene_dir, scene_id)
    output_path = output_path or BUILD_ROOT / f"{scene_id}.web_scene.json"
    if not output_path.is_absolute():
        output_path = (REPO_ROOT / output_path).resolve()
    report_path = BUILD_ROOT / f"{scene_id}.visual_acceptance.json"
    screenshot_path = BUILD_ROOT / (f"{scene_id}.rviz_parity.png" if scene_id == "ur5_2f_test" else f"{scene_id}.visual_acceptance.png")
    BUILD_ROOT.mkdir(parents=True, exist_ok=True)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    steps = [run_step([sys.executable, "scripts/ensure_workcell_studio_web_scene_fresh.py", "--scene", repo_relative(scene_dir), "--output", repo_relative(output_path), "--stage-assets"])]
    if steps[-1]["returncode"] == 0:
        steps.append(run_step([sys.executable, "scripts/check_workcell_web_scene_mesh_contract.py", repo_relative(output_path)]))
    if steps[-1]["returncode"] == 0:
        steps.append(run_step([sys.executable, "scripts/check_workcell_web_scene_visual_bounds.py", repo_relative(output_path), "--json"]))
    server_status = "not_started"; server = None
    browser = {"available": False, "method": "not_run", "status": None}
    viewer_url = f"http://localhost:{port}/workcell_studio_web/viewer/index.html?scene={quote(repo_relative(output_path))}&force_refresh={int(time.time() * 1000)}"
    if all(step["returncode"] == 0 for step in steps):
        server_status, server = start_or_reuse_server(port)
        if server_status != "failed":
            try:
                browser = run_browser(viewer_url, report_path, screenshot_path, require_browser)
            except Exception as exc:
                browser = {"available": False, "method": "failed", "status": None, "error": str(exc), "javascript_errors": [], "failed_network_requests": []}
        elif require_browser:
            browser = {"available": False, "method": "server_failed", "status": None, "error": f"could not start or reuse HTTP server on port {port}", "javascript_errors": [], "failed_network_requests": []}
    if not screenshot_path.exists() and not require_browser:
        screenshot_path.write_bytes(PNG_1X1)
    browser_status_path = BUILD_ROOT / f"{scene_id}.browser_status.json"
    if isinstance(browser.get("status"), Mapping):
        browser_status_path.write_text(json.dumps({"status": browser.get("status")}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    status = browser.get("status") if isinstance(browser, Mapping) else None
    legacy_strict_scene = not expected or (expected.get("robot") == "ur5" and expected.get("tool") == "robotiq_2f")
    strict_status_errors = validate_browser_status(status) if legacy_strict_scene and isinstance(status, Mapping) else []
    report = {"scene_id": scene_id, "scene_dir": repo_relative(scene_dir), "expected": dict(expected or {}), "web_scene_json": repo_relative(output_path), "viewer_url": viewer_url, "server_status": server_status, "browser": browser, "robot_preview_lifecycle_diagnostics": robot_preview_lifecycle_diagnostics(browser.get("status")) if isinstance(browser.get("status"), Mapping) else {}, "strict_status_errors": strict_status_errors, "steps": steps, "browser_status_json": repo_relative(browser_status_path) if browser_status_path.exists() else "", "report": repo_relative(report_path), "screenshot": repo_relative(screenshot_path), "screenshot_dimensions": png_dimensions(screenshot_path), "screenshot_size_bytes": screenshot_path.stat().st_size if screenshot_path.exists() else 0}
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if server is not None:
        server.shutdown()
    rc = 0
    if any(step["returncode"] != 0 for step in steps): rc = 1
    if require_browser and require_browser_artifact_errors(browser, server_status, screenshot_path, browser_status_path): rc = 1
    if require_browser and isinstance(status, Mapping) and str(_status_value(status, "web3d_readiness_state", "web3dReadinessState", "viewer_boot_state", "viewerBootState") or "") != "scene_ready": rc = 1
    if isinstance(status, Mapping) and legacy_strict_scene and strict_status_errors: rc = 1
    elif require_browser and not isinstance(status, Mapping): rc = 1
    return rc, report


def run_supported_scene_browser_matrix(*, output: Path, catalog: Path | None, scene_ids: Sequence[str] | None, require_browser: bool, port: int, sequential: bool) -> dict[str, Any]:
    entries, catalog_errors = _load_supported_entries(catalog)
    by_name = _entry_by_scene_name(entries)
    order = list(scene_ids or SUPPORTED_SCENE_ACCEPTANCE_ORDER)
    rows = []
    for scene_name in order:
        entry = by_name.get(scene_name)
        scene_dir = (REPO_ROOT / (getattr(entry, "scene_path", f"scenes/{scene_name}"))).resolve() if entry else (REPO_ROOT / "scenes" / scene_name).resolve()
        expected = _scene_expectations(entry, scene_dir)
        if not scene_dir.is_dir():
            rows.append({"scene_id": scene_name, "status": "BLOCKED", "terminal_stage": "not_run", "expected_robot": expected["robot"], "expected_tool": expected["tool"], "errors": [f"scene directory missing: {repo_relative(scene_dir)}"]})
            continue
        rc, report = run_one_scene(scene_dir, output_path=BUILD_ROOT / f"{scene_name}.web_scene.json", scene_id=scene_name, require_browser=require_browser, port=port, expected=expected)
        row = _matrix_row_from_report(report, expected, require_browser)
        if rc != 0 and row["status"] == "PASS": row["status"] = "FAIL"
        rows.append(row)
    counts = {"PASS": 0, "FAIL": 0, "BLOCKED": 0}
    for row in rows: counts[row.get("status", "FAIL")] += 1
    report = {"schema": "workcell_studio_web3d_supported_scenes_acceptance_matrix/v1", "sequential": bool(sequential), "status": "FAIL" if counts["FAIL"] or catalog_errors else ("BLOCKED" if counts["BLOCKED"] else "PASS"), "counts": counts, "catalog_errors": catalog_errors, "scenes": rows, "scene_switch_sequence": list(SEQUENTIAL_SCENE_SWITCH_ORDER)}
    out = output if output.is_absolute() else REPO_ROOT / output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return report

def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run browser visual acceptance for a Workcell Studio Web 3D scene.")
    parser.add_argument("--scene", help="Scene directory to export and validate.")
    parser.add_argument("--output", help="Optional staged web_scene.json output path.")
    parser.add_argument("--scene-id", help="Override derived scene id used for default output/report names.")
    parser.add_argument("--require-browser", action="store_true", help="Fail if Playwright/chromium/google-chrome is unavailable or cannot run.")
    parser.add_argument("--port", type=int, default=8765, help="Repo-root HTTP server port (default: 8765).")
    parser.add_argument("--all-supported-scenes", action="store_true", help="Run the supported-scenes browser Product View acceptance matrix.")
    parser.add_argument("--sequential", action="store_true", help="Run matrix scenes sequentially and include the scene-switch contract sequence.")
    parser.add_argument("--catalog", type=Path, help="Supported-scene catalog path for --all-supported-scenes.")
    parser.add_argument("--only-scene", action="append", default=[], help="Limit --all-supported-scenes to one scene id; repeatable.")
    args = parser.parse_args(argv)

    if args.all_supported_scenes:
        report = run_supported_scene_browser_matrix(output=Path(args.output) if args.output else BUILD_ROOT / "supported_scenes_visual_acceptance_matrix.json", catalog=args.catalog, scene_ids=args.only_scene, require_browser=args.require_browser, port=args.port, sequential=args.sequential)
        return 1 if report["status"] in {"FAIL", "BLOCKED"} and args.require_browser else (1 if report["status"] == "FAIL" else 0)

    if not args.scene:
        print("error: --scene is required unless --all-supported-scenes is used", file=sys.stderr)
        return 2
    scene_dir = Path(args.scene).expanduser()
    if not scene_dir.is_absolute():
        scene_dir = (REPO_ROOT / scene_dir).resolve()
    if not scene_dir.is_dir():
        print(f"error: --scene must be an existing scene directory: {scene_dir}", file=sys.stderr)
        return 2

    scene_id = derive_scene_id(scene_dir, args.scene_id)
    rc, report = run_one_scene(scene_dir, output_path=Path(args.output).expanduser() if args.output else None, scene_id=scene_id, require_browser=args.require_browser, port=args.port, expected=_scene_expectations(None, scene_dir))
    print(json.dumps(report, indent=2, sort_keys=True))
    print(f"viewer_url: {report['viewer_url']}")
    print(f"screenshot_path: {report.get('screenshot', '')}")
    print(f"report_path: {report.get('report', '')}")
    return rc



if __name__ == "__main__":
    raise SystemExit(main())
