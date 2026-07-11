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



def _status_value(status: Mapping[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in status and status.get(key) is not None:
            return status.get(key)
    return None


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
        category in {"robot", "tool", "table", "environment"}
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
        "max_distance_from_wrist_3_link_or_tool0_to_gripper_base_link_m": max_distance,
    }

def _euclidean_distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sum((left - right) ** 2 for left, right in zip(a, b)) ** 0.5


def _mesh_backing_errors(status: Mapping[str, Any]) -> list[str]:
    diagnostics = _rendered_mesh_diagnostics_from_status(status)
    table_seen = False
    camera_seen = False
    errors: list[str] = []
    for diagnostic in diagnostics:
        if not isinstance(diagnostic, Mapping):
            continue
        text = _diagnostic_text(diagnostic)
        category = str(diagnostic.get("category") or "").lower()
        if category == "table" or any(token in text for token in ("table", "workbench", "work-bench", "bench")):
            table_seen = True
            if _diagnostic_status(diagnostic) != "mesh_loaded" or not _diagnostic_bool(diagnostic, "mesh_loaded", "meshLoaded"):
                name = _diagnostic_link_name(diagnostic) or str(diagnostic.get("object_id") or diagnostic.get("id") or "table/workbench")
                errors.append(f"browser viewer table/workbench {name} must remain mesh-backed; got render_status={_diagnostic_status(diagnostic)!r}")
        if category == "camera" or any(token in text for token in ("camera", "realsense", "sensor")):
            camera_seen = True
            if _diagnostic_status(diagnostic) != "mesh_loaded" or not _diagnostic_bool(diagnostic, "mesh_loaded", "meshLoaded"):
                name = _diagnostic_link_name(diagnostic) or str(diagnostic.get("object_id") or diagnostic.get("id") or "camera")
                errors.append(f"browser viewer camera/Realsense {name} must remain mesh-backed; got render_status={_diagnostic_status(diagnostic)!r}")
    if not table_seen:
        errors.append("browser viewer table/workbench mesh-backed diagnostic is required")
    if not camera_seen:
        errors.append("browser viewer camera/Realsense mesh-backed diagnostic is required")
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
    if not isinstance(visual_matrices, list) or not visual_matrices:
        errors.append("browser viewer must expose robot_visual_wrapper_world_matrices for T_world_link × T_link_visual wrappers")
    else:
        visual_links = {str(row.get("link_name") or row.get("linkName") or "") for row in visual_matrices if isinstance(row, Mapping)}
        mesh_links = [link for link in REQUIRED_BROWSER_MATRIX_LINKS if link != "tool0"]
        missing_visuals = [link for link in mesh_links if link not in visual_links]
        if missing_visuals:
            errors.append(f"browser viewer visual-wrapper matrix diagnostics missing required mesh links: {', '.join(missing_visuals)}")
    return errors

def validate_browser_status(status: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    mesh_loaded_count = _status_int(status, "mesh_loaded_count", "meshLoadedCount")
    required_mesh_failed_count = _status_int(status, "required_mesh_failed_count", "requiredMeshFailedCount")
    if mesh_loaded_count != EXPECTED_MESH_LOADED_COUNT:
        errors.append(f"browser viewer meshLoadedCount expected {EXPECTED_MESH_LOADED_COUNT}, got {mesh_loaded_count}")
    if required_mesh_failed_count != EXPECTED_REQUIRED_MESH_FAILED_COUNT:
        errors.append(f"browser viewer requiredMeshFailedCount expected {EXPECTED_REQUIRED_MESH_FAILED_COUNT}, got {required_mesh_failed_count}")

    distances = _distance_map_from_status(status)
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
    errors.extend(_rendered_mesh_adjacency_errors(status))
    errors.extend(_mesh_backing_errors(status))
    errors.extend(_table_horizontal_errors(status))
    errors.extend(_required_product_fallback_errors(status))
    errors.extend(_table_mesh_contract_errors(status))
    errors.extend(_camera_bounds_errors(status))
    errors.extend(_baked_pose_render_mode_errors(status))
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


def run_browser(url: str, status_path: Path, screenshot_path: Path, require: bool) -> dict[str, Any]:
    try:
        from playwright.sync_api import sync_playwright  # type: ignore
        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True, args=["--ignore-certificate-errors"])
            page = browser.new_page(viewport={"width": 1280, "height": 900}, ignore_https_errors=True)
            page.goto(url, wait_until="networkidle", timeout=45000)
            page.wait_for_function("window.__WORKCELL_VIEWER_STATUS__ && typeof window.__WORKCELL_VIEWER_STATUS__ === 'object'", timeout=45000)
            page.wait_for_function("window.__WORKCELL_ROBOT_PREVIEW_READY__ && typeof window.__WORKCELL_ROBOT_PREVIEW_READY__.then === 'function'", timeout=45000)
            page.evaluate("() => window.__WORKCELL_ROBOT_PREVIEW_READY__.then(() => true)")
            page.wait_for_function("() => { const s = window.__WORKCELL_VIEWER_STATUS__ || {}; const state = s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || ''; return state === 'ready' || state === 'failed'; }", timeout=45000)
            status = page.evaluate("window.__WORKCELL_VIEWER_STATUS__ || null")
            final_state = page.evaluate("() => (window.__WORKCELL_VIEWER_STATUS__ || {}).robot_preview_lifecycle_state || (window.__WORKCELL_VIEWER_STATUS__ || {}).robotPreviewLifecycleState || ''")
            screenshot_before_ready = final_state != 'ready'
            page.screenshot(path=str(screenshot_path), full_page=True)
            browser.close()
        return {"available": True, "method": "playwright", "status": status, "screenshot_before_ready": screenshot_before_ready}
    except Exception as exc:
        playwright_error = str(exc)

    command_spec = browser_command(url, screenshot_path)
    if command_spec:
        result = subprocess.run(command_spec["command"], cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        return {"available": result.returncode == 0, "method": command_spec["kind"], "status": None, "returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr, "playwright_error": playwright_error}
    if require:
        raise RuntimeError(f"No usable browser found. Playwright error: {playwright_error}")
    screenshot_path.write_bytes(PNG_1X1)
    return {"available": False, "method": "skipped", "status": None, "reason": f"No Playwright/chromium/google-chrome browser available. Playwright error: {playwright_error}"}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run browser visual acceptance for a Workcell Studio Web 3D scene.")
    parser.add_argument("--scene", required=True, help="Scene directory to export and validate.")
    parser.add_argument("--output", help="Optional staged web_scene.json output path.")
    parser.add_argument("--scene-id", help="Override derived scene id used for default output/report names.")
    parser.add_argument("--require-browser", action="store_true", help="Fail if Playwright/chromium/google-chrome is unavailable or cannot run.")
    parser.add_argument("--port", type=int, default=8765, help="Repo-root HTTP server port (default: 8765).")
    args = parser.parse_args(argv)

    scene_dir = Path(args.scene).expanduser()
    if not scene_dir.is_absolute():
        scene_dir = (REPO_ROOT / scene_dir).resolve()
    if not scene_dir.is_dir():
        print(f"error: --scene must be an existing scene directory: {scene_dir}", file=sys.stderr)
        return 2

    scene_id = derive_scene_id(scene_dir, args.scene_id)
    output_path = Path(args.output).expanduser() if args.output else BUILD_ROOT / f"{scene_id}.web_scene.json"
    if not output_path.is_absolute():
        output_path = (REPO_ROOT / output_path).resolve()
    report_path = BUILD_ROOT / f"{scene_id}.visual_acceptance.json"
    screenshot_path = BUILD_ROOT / (f"{scene_id}.rviz_parity.png" if scene_id == "ur5_2f_test" else f"{scene_id}.visual_acceptance.png")
    BUILD_ROOT.mkdir(parents=True, exist_ok=True)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    steps = []
    steps.append(run_step([sys.executable, "scripts/ensure_workcell_studio_web_scene_fresh.py", "--scene", repo_relative(scene_dir), "--output", repo_relative(output_path), "--stage-assets"]))
    if steps[-1]["returncode"] == 0:
        steps.append(run_step([sys.executable, "scripts/check_workcell_web_scene_mesh_contract.py", repo_relative(output_path)]))
    if steps[-1]["returncode"] == 0:
        steps.append(run_step([sys.executable, "scripts/check_workcell_web_scene_visual_bounds.py", repo_relative(output_path), "--json"]))

    server_status = "not_started"
    server = None
    browser = {"available": False, "method": "not_run", "status": None}
    viewer_url = f"http://localhost:{args.port}/workcell_studio_web/viewer/index.html?scene={quote(repo_relative(output_path))}"
    if all(step["returncode"] == 0 for step in steps):
        server_status, server = start_or_reuse_server(args.port)
        if server_status != "failed":
            try:
                browser = run_browser(viewer_url, report_path, screenshot_path, args.require_browser)
            except Exception as exc:
                browser = {"available": False, "method": "failed", "status": None, "error": str(exc)}
        elif args.require_browser:
            browser = {"available": False, "method": "server_failed", "status": None, "error": f"could not start or reuse HTTP server on port {args.port}"}

    if not screenshot_path.exists():
        screenshot_path.write_bytes(PNG_1X1)

    browser_status_path = BUILD_ROOT / f"{scene_id}.browser_status.json"
    if isinstance(browser.get("status"), Mapping):
        browser_status_path.write_text(json.dumps({"status": browser.get("status")}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        compare_report_path = BUILD_ROOT / f"{scene_id}.browser_fk_vs_ros_tf.json"
        steps.append(run_step([sys.executable, "scripts/compare_web_scene_fk_to_ros_tf.py", "--scene", repo_relative(scene_dir), "--web-scene", repo_relative(output_path), "--output", repo_relative(compare_report_path), "--browser-status", repo_relative(browser_status_path), "--tolerance", "0.0001"]))

    report = {
        "scene_id": scene_id,
        "scene_dir": repo_relative(scene_dir),
        "web_scene_json": repo_relative(output_path),
        "viewer_url": viewer_url,
        "server_status": server_status,
        "browser": browser,
        "robot_preview_lifecycle_diagnostics": robot_preview_lifecycle_diagnostics(browser.get("status")) if isinstance(browser.get("status"), Mapping) else {},
        "steps": steps,
        "browser_status_json": repo_relative(browser_status_path) if browser_status_path.exists() else "",
        "report": repo_relative(report_path),
        "screenshot": repo_relative(screenshot_path),
    }
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if server is not None:
        server.shutdown()
    print(json.dumps(report, indent=2, sort_keys=True))
    print(f"viewer_url: {viewer_url}")
    print(f"screenshot_path: {repo_relative(screenshot_path)}")
    print(f"report_path: {repo_relative(report_path)}")
    if any(step["returncode"] != 0 for step in steps):
        return 1
    if args.require_browser and not browser.get("available"):
        return 1
    status = browser.get("status") if isinstance(browser, Mapping) else None
    if isinstance(status, Mapping):
        lifecycle_diagnostics = robot_preview_lifecycle_diagnostics(status)
        debug_summary = baked_pose_render_mode_summary(status)
        print("robot_preview_lifecycle_diagnostics: " + json.dumps(lifecycle_diagnostics, sort_keys=True))
        print("visual_debug_baked_pose_summary: " + json.dumps(debug_summary, sort_keys=True))
        print("visual_acceptance_debug_dump: " + json.dumps(_visual_acceptance_debug_dump(status), sort_keys=True))
        if browser.get("screenshot_before_ready"):
            print("error: browser screenshot was captured before robot-preview readiness completed", file=sys.stderr)
            return 1
        status_errors = validate_browser_status(status)
        if status_errors:
            for error in status_errors:
                print(f"error: {error}", file=sys.stderr)
            return 1
    elif args.require_browser:
        print("error: browser viewer did not expose window.__WORKCELL_VIEWER_STATUS__", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
