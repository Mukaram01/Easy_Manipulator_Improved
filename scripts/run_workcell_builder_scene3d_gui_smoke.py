#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, math, os, shlex, shutil, subprocess, sys, time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
from scripts.workcell_studio_script_bootstrap import ensure_repo_root_on_sys_path
ensure_repo_root_on_sys_path(__file__)
from scripts.workcell_studio_path_resolver import describe_resolution, resolve_repo_root, resolve_workspace_root, resolve_workcell_builder_executable, workcell_builder_executable_candidates
from scripts.scene_root_resolver import resolve_scene_root
from scripts.scene3d_scene_discovery import discover_scene3d_scenes
from scripts.validate_ur5_scene3d_transform_plausibility import validate_index as validate_ur5_transform_plausibility_index

EXPECTED_SCHEMA = "workcell_studio_scene3d_gui_smoke/v1"

ROS_HUMBLE_SETUP_PATH = Path("/opt/ros/humble/setup.bash")
ROS_HUMBLE_MISSING_MESSAGE = (
    "ROS Humble is not available: /opt/ros/humble/setup.bash was not found "
    "and ROS_DISTRO is not humble"
)


def _ros_humble_available() -> bool:
    return ROS_HUMBLE_SETUP_PATH.is_file() or os.environ.get("ROS_DISTRO") == "humble"


def _ros_humble_environment() -> dict[str, Any]:
    return {
        "ros_humble_available": _ros_humble_available(),
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "ros_humble_setup_path": str(ROS_HUMBLE_SETUP_PATH),
    }


def _add_ros_humble_context(payload: dict[str, Any]) -> dict[str, Any]:
    payload.update(_ros_humble_environment())
    return payload


def _append_unique(values: list[str], value: str) -> None:
    if value not in values:
        values.append(value)


def _apply_generated_urdf_visual_first_drop_smoke_stage(payload: dict[str, Any]) -> None:
    diagnostics = payload.get("visual_ingestion_diagnostics")
    if not isinstance(diagnostics, dict):
        diagnostics = payload.get("scene3d_visual_ingestion_diagnostics")
    if not isinstance(diagnostics, dict):
        return
    rows = diagnostics.get("generated_urdf_visual_row_diagnostics")
    if not isinstance(rows, list):
        return
    final_draw_rows = payload.get("final_draw_visual_items")
    if not isinstance(final_draw_rows, list):
        final_draw_rows = payload.get("final_draw_diagnostics")
    if not isinstance(final_draw_rows, list):
        return
    final_draw_ids = {
        str(row.get("item_id") or row.get("id") or "").strip()
        for row in final_draw_rows
        if isinstance(row, dict)
    }
    if not final_draw_ids:
        return
    updated_rows: list[Any] = []
    changed = False
    for row in rows:
        if not isinstance(row, dict):
            updated_rows.append(row)
            continue
        updated = dict(row)
        row_id = str(updated.get("id") or "").strip()
        if row_id in final_draw_ids and not str(updated.get("first_drop_stage") or "").strip():
            updated["first_drop_stage"] = "smoke_output_or_audit_only"
            changed = True
        updated_rows.append(updated)
    if changed:
        diagnostics["generated_urdf_visual_row_diagnostics"] = updated_rows
        payload["visual_ingestion_diagnostics"] = diagnostics
        if isinstance(payload.get("scene3d_visual_ingestion_diagnostics"), dict):
            payload["scene3d_visual_ingestion_diagnostics"] = diagnostics


def _record_ros_humble_missing(payload: dict[str, Any], *, as_blocker: bool) -> None:
    _add_ros_humble_context(payload)
    if payload.get("ros_humble_available") is True:
        return
    messages = payload.get("blocker_messages")
    if not isinstance(messages, dict):
        messages = {}
    messages["ros_humble_missing"] = ROS_HUMBLE_MISSING_MESSAGE
    payload["blocker_messages"] = messages
    target_key = "blockers" if as_blocker else "warnings"
    values = payload.get(target_key)
    if not isinstance(values, list):
        values = []
    _append_unique(values, "ros_humble_missing")
    payload[target_key] = values
    warning_messages = payload.get("warning_messages")
    if not isinstance(warning_messages, dict):
        warning_messages = {}
    warning_messages["ros_humble_missing"] = ROS_HUMBLE_MISSING_MESSAGE
    payload["warning_messages"] = warning_messages


def _executable_can_run(exe: Path | str) -> bool:
    exe_str = str(exe)
    if os.sep not in exe_str and shutil.which(exe_str):
        return True
    path = Path(exe_str)
    return path.is_file() and os.access(path, os.X_OK)

def _tail(text: str, lines: int = 40) -> str:
    parts = (text or "").splitlines()
    return "\n".join(parts[-lines:])

def _resolve_executable_candidates(workspace_root: Path | None) -> list[Path]:
    return workcell_builder_executable_candidates(workspace_root)

def build_cmd(exe: Path | str, args: argparse.Namespace) -> list[str]:
    cmd = [str(exe), "--scene3d-smoke"]
    if args.new_cell_recommended_layout_smoke:
        cmd.append("--new-cell-recommended-layout-smoke")
    elif args.scene:
        cmd += ["--scene", args.scene]
    if args.scene_path:
        cmd += ["--scene-path", str(args.scene_path)]
    cmd += ["--smoke-output", str(args.output)]
    if args.screenshot:
        cmd += ["--smoke-screenshot", str(args.screenshot)]
    cmd.append("--exit-after-smoke")
    return cmd

def with_xvfb(cmd: list[str], use_xvfb: bool) -> tuple[list[str], list[str], dict[str, str]]:
    if not use_xvfb:
        return cmd, [], {}
    xvfb_run = shutil.which("xvfb-run")
    if xvfb_run:
        return [xvfb_run, "-a"] + cmd, [], {}
    return cmd, ["xvfb_requested_but_unavailable_using_qt_offscreen"], {
        "QT_QPA_PLATFORM": "offscreen",
        "QT_OPENGL": "software",
        "LIBGL_ALWAYS_SOFTWARE": "1",
    }

def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _path_str(path: Path | None) -> str | None:
    return str(path) if path is not None else None


def _workspace_resolution_warnings(workspace_root: Path | None, explicit_workspace_root: Path | None) -> list[str]:
    warnings: list[str] = []
    if workspace_root is None:
        warnings.append("workspace_root_inference_failed_path_only_executable_search")
    elif explicit_workspace_root is not None and not workspace_root.exists():
        warnings.append("explicit_workspace_root_does_not_exist")
    return warnings


def _merge_unique(base: list[str], extra: list[str]) -> list[str]:
    merged = list(base)
    for item in extra:
        _append_unique(merged, item)
    return merged


def _ros_humble_available() -> bool:
    return Path("/opt/ros/humble/setup.bash").is_file() or os.environ.get("ROS_DISTRO") == "humble"


def _as_int(value: Any) -> int:
    try:
        if value is None or isinstance(value, bool):
            return 0
        return int(value)
    except (TypeError, ValueError):
        return 0


def _counter(payload: dict[str, Any], *keys: str) -> int:
    sources = [payload]
    for nested_key in ("counters", "render_debug_counters"):
        nested = payload.get(nested_key)
        if isinstance(nested, dict):
            sources.append(nested)
    for source in sources:
        for key in keys:
            if key in source:
                return _as_int(source.get(key))
    return 0


def _physical_rendered_count(payload: dict[str, Any]) -> int:
    mesh_count = _counter(payload, "physical_mesh_items_rendered", "mesh_rendered_count", "mesh_backed_count")
    primitive_count = _counter(
        payload,
        "primitive_fallback_items_rendered",
        "primitive_rendered_count",
        "urdf_primitive_rendered_count",
        "valid_physical_fallback_count",
        "physical_fallback_count",
        "physical_fallback_rendered_count",
        "collision_primitive_rendered_count",
    )
    physical_total = _counter(payload, "physical_rendered_count", "credible_physical_rendered_count")
    return max(physical_total, mesh_count + primitive_count)


def _mark_runtime_available(payload: dict[str, Any], runtime_available: bool) -> None:
    payload["runtime_available"] = runtime_available


def _enforce_physical_render_evidence(payload: dict[str, Any]) -> dict[str, Any]:
    _mark_runtime_available(payload, True)
    if _physical_rendered_count(payload) > 0:
        return payload
    if "scene_rendered_no_physical_items" not in blockers:
        blockers.append("scene_rendered_no_physical_items")
    payload["blockers"] = blockers
    payload["status"] = "FAIL"
    payload["physical_rendered_count"] = 0
    return payload

def _first_present_mapping(payload: dict[str, Any], *keys: str) -> dict[str, Any]:
    for key in keys:
        value = payload.get(key)
        if isinstance(value, dict):
            return value
    return {}


def _first_present_list(payload: dict[str, Any], *keys: str) -> list[Any]:
    for key in keys:
        value = payload.get(key)
        if isinstance(value, list):
            return value
    return []


def _counter_from_sources(payload: dict[str, Any], keys: tuple[str, ...], *extra_sources: dict[str, Any]) -> int:
    sources: list[dict[str, Any]] = [payload]
    for nested_key in ("runtime_render_class_counts", "render_class_counts", "class_render_counts", "counters", "render_debug_counters"):
        nested = payload.get(nested_key)
        if isinstance(nested, dict):
            sources.append(nested)
    sources.extend(source for source in extra_sources if isinstance(source, dict))
    for source in sources:
        for key in keys:
            if key in source:
                return _as_int(source.get(key))
    return 0


def _viewport_size_from_payload(payload: dict[str, Any]) -> dict[str, int] | None:
    for source in (
        payload,
        _first_present_mapping(payload, "counters"),
        _first_present_mapping(payload, "render_debug_counters"),
    ):
        value = source.get("viewport_size") if isinstance(source, dict) else None
        if isinstance(value, dict):
            width = _as_int(value.get("width") or value.get("w"))
            height = _as_int(value.get("height") or value.get("h"))
            if width > 0 or height > 0:
                return {"width": width, "height": height}
        if isinstance(value, list) and len(value) >= 2:
            width = _as_int(value[0])
            height = _as_int(value[1])
            if width > 0 or height > 0:
                return {"width": width, "height": height}
        width = _counter_from_sources(source if isinstance(source, dict) else {}, ("viewport_width", "width"))
        height = _counter_from_sources(source if isinstance(source, dict) else {}, ("viewport_height", "height"))
        if width > 0 or height > 0:
            return {"width": width, "height": height}
    return None


def _camera_fit_target_from_payload(payload: dict[str, Any]) -> str | None:
    for source in (
        payload,
        _first_present_mapping(payload, "counters"),
        _first_present_mapping(payload, "filter_diagnostics"),
        _first_present_mapping(payload, "runtime_scene3d_diagnostics"),
    ):
        value = source.get("camera_fit_target") if isinstance(source, dict) else None
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def _visible_item_labels_from_payload(payload: dict[str, Any]) -> list[str]:
    labels: list[str] = []

    def append_label(value: Any) -> None:
        if isinstance(value, str):
            label = value.strip()
        elif isinstance(value, dict):
            label = str(value.get("label") or value.get("display_name") or value.get("name") or value.get("id") or "").strip()
        else:
            label = ""
        if label and label not in labels:
            labels.append(label)

    for item in _first_present_list(payload, "visible_item_labels", "visible_labels"):
        append_label(item)
    for item in _first_present_list(payload, "visible_items", "rendered_items", "viewport_visible_items"):
        append_label(item)
    return labels


def _runtime_render_class_counts(payload: dict[str, Any]) -> dict[str, int]:
    counters = _first_present_mapping(payload, "counters")
    render_debug = _first_present_mapping(payload, "render_debug_counters")
    return {
        "robot_mesh_rendered": _counter_from_sources(payload, ("robot_mesh_rendered", "robot_mesh_rendered_count", "robot_rendered_count"), counters, render_debug),
        "tool_mesh_rendered": _counter_from_sources(payload, ("tool_mesh_rendered", "tool_mesh_rendered_count", "end_effector_mesh_rendered", "end_effector_mesh_rendered_count"), counters, render_debug),
        "environment_rendered": _counter_from_sources(payload, ("environment_rendered", "environment_rendered_count", "environment_mesh_rendered", "environment_mesh_rendered_count"), counters, render_debug),
        "zones_rendered": _counter_from_sources(payload, ("zones_rendered", "zones_rendered_count", "zones_overlays_rendered", "overlay_rendered_count"), counters, render_debug),
        "camera_rendered": _counter_from_sources(payload, ("camera_rendered", "camera_rendered_count", "camera_mesh_rendered", "camera_mesh_rendered_count"), counters, render_debug),
        "semantic_fallback_rendered": _counter_from_sources(payload, ("semantic_fallback_rendered", "semantic_fallback_rendered_count", "generated_fallback_count"), counters, render_debug),
    }


def _runtime_gui_diagnostics_available(payload: dict[str, Any]) -> bool:
    if not bool(payload.get("runtime_available")):
        return False
    if isinstance(payload.get("counters"), dict) or isinstance(payload.get("render_debug_counters"), dict):
        return True
    return _counter(payload, "viewport_received_count", "rendered_count", "visible_count") > 0


def _apply_runtime_transform_counter_mapping(payload: dict[str, Any]) -> None:
    if not _runtime_gui_diagnostics_available(payload):
        return
    counters = _first_present_mapping(payload, "counters")
    render_debug = _first_present_mapping(payload, "render_debug_counters")

    runtime_diagnostics = _first_present_mapping(payload, "runtime_scene3d_diagnostics", "runtime_diagnostics", "diagnostics")

    def runtime_counter(*keys: str) -> int:
        for source in (render_debug, counters, runtime_diagnostics, payload):
            if not isinstance(source, dict):
                continue
            for key in keys:
                if key in source:
                    return _as_int(source.get(key))
        return _counter_from_sources(payload, keys, runtime_diagnostics)

    transform_chain_applied_count = runtime_counter("transform_chain_applied_count")
    visual_origin_applied_count = runtime_counter("visual_origin_applied_count")
    generated_visual_count = runtime_counter(
        "locked_generated_urdf_visual_count",
        "generated_visual_count",
        "generated_urdf_visual_count",
    )
    baked_world_visual_pose_applied_count = runtime_counter(
        "runtime_baked_world_visual_pose_applied_count",
        "baked_world_visual_pose_count",
        "baked_world_visual_transform_count",
    )
    payload["transform_chain_applied_count"] = transform_chain_applied_count
    payload["visual_origin_applied_count"] = visual_origin_applied_count
    payload["locked_generated_urdf_visual_count"] = generated_visual_count
    payload["runtime_baked_world_visual_pose_applied_count"] = baked_world_visual_pose_applied_count

    warnings = payload.get("warnings")
    if not isinstance(warnings, list):
        warnings = []
    if generated_visual_count > 0 and transform_chain_applied_count <= 0 and baked_world_visual_pose_applied_count <= 0:
        _append_unique(warnings, "runtime_transform_chain_applied_count_zero_with_generated_visuals")
    if generated_visual_count > 0 and visual_origin_applied_count <= 0 and baked_world_visual_pose_applied_count <= 0:
        _append_unique(warnings, "runtime_visual_origin_applied_count_zero_with_generated_visuals")
    payload["warnings"] = warnings


def _apply_generated_urdf_visual_number_stages(payload: dict[str, Any]) -> None:
    counters = _first_present_mapping(payload, "counters")
    filter_diagnostics = _first_present_mapping(payload, "filter_diagnostics")
    counter_ingestion = _first_present_mapping(counters, "visual_ingestion_diagnostics")
    filter_ingestion = _first_present_mapping(filter_diagnostics, "visual_ingestion_diagnostics")
    sources = (filter_diagnostics, counters, counter_ingestion, filter_ingestion, payload)
    stage_keys = {
        "after_ingest": "generated_urdf_visual_numbers_after_ingest",
        "after_suppression": "generated_urdf_visual_numbers_after_suppression",
        "after_filter": "generated_urdf_visual_numbers_after_filter",
    }
    for short_key, full_key in stage_keys.items():
        value = None
        for source in sources:
            if isinstance(source, dict) and isinstance(source.get(full_key), list):
                value = source[full_key]
                break
        if value is not None:
            payload[short_key] = value
            payload[full_key] = value

def _add_smoke_report_supplemental_evidence(payload: dict[str, Any], *, screenshot_path: str | None, screenshot_available: bool | None) -> dict[str, Any]:
    runtime_available = bool(payload.get("runtime_available"))
    if runtime_available:
        render_class_counts = _runtime_render_class_counts(payload)
        payload["runtime_render_class_counts"] = render_class_counts
        for key, value in render_class_counts.items():
            payload.setdefault(key, value)
        _apply_runtime_transform_counter_mapping(payload)
    payload.setdefault("visible_item_labels", _visible_item_labels_from_payload(payload) if runtime_available else [])
    payload.setdefault("viewport_size", _viewport_size_from_payload(payload))
    payload.setdefault("camera_fit_target", _camera_fit_target_from_payload(payload))
    _apply_generated_urdf_visual_number_stages(payload)
    if screenshot_path is not None:
        payload.setdefault("screenshot_path", screenshot_path)
    if screenshot_available is not None:
        payload["screenshot_available"] = bool(screenshot_available or payload.get("screenshot_available") or payload.get("screenshot_saved"))
    return payload



def _scene_name_from_payload_or_args(payload: dict[str, Any], args: argparse.Namespace) -> str | None:
    scene = payload.get("scene") or getattr(args, "scene", None)
    if isinstance(scene, str) and scene.strip():
        return scene.strip()
    scene_path = payload.get("scene_path") or getattr(args, "scene_path", None)
    if scene_path:
        return Path(str(scene_path)).name
    return None


def _mesh_index_path_for_scene(repo_root: Path, payload: dict[str, Any], args: argparse.Namespace) -> Path | None:
    scene_path = payload.get("scene_path") or getattr(args, "scene_path", None)
    if scene_path:
        return (Path(str(scene_path)) / "generated" / "scene_visual_mesh_index.json").resolve()
    scene = _scene_name_from_payload_or_args(payload, args)
    if scene:
        return (repo_root / "scenes" / scene / "generated" / "scene_visual_mesh_index.json").resolve()
    return None


def _baked_world_visual_transform_count(index_payload: dict[str, Any]) -> int:
    items = index_payload.get("visual_items") or index_payload.get("items") or []
    if not isinstance(items, list):
        return 0
    count = 0
    for item in items:
        if not isinstance(item, dict):
            continue
        pose = item.get("pose")
        world_pose = item.get("world_pose")
        link_world_pose = item.get("link_world_pose")
        if isinstance(pose, dict) and (isinstance(world_pose, dict) or isinstance(link_world_pose, dict)):
            count += 1
        elif str(item.get("transform_status") or "").strip().lower() in {"ok", "resolved", "static_fallback", "static_fallback_parent"}:
            count += 1
    return count



UR5_RENDERED_MESH_LINK_ALIASES: dict[str, tuple[str, ...]] = {
    "base_link": ("base_link", "base_link_inertia", "base"),
    "shoulder_link": ("shoulder_link",),
    "upper_arm_link": ("upper_arm_link",),
    "forearm_link": ("forearm_link",),
    "wrist_1_link": ("wrist_1_link",),
    "wrist_2_link": ("wrist_2_link",),
    "wrist_3_link": ("wrist_3_link",),
    "tool0": ("tool0",),
    "robotiq_base": ("robotiq_base", "gripper_base_link", "robotiq_85_base_link", "robotiq base visual item"),
}
UR5_RENDERED_MESH_ADJACENT_PAIRS: tuple[tuple[str, str], ...] = (
    ("base_link", "shoulder_link"),
    ("shoulder_link", "upper_arm_link"),
    ("upper_arm_link", "forearm_link"),
    ("forearm_link", "wrist_1_link"),
    ("wrist_1_link", "wrist_2_link"),
    ("wrist_2_link", "wrist_3_link"),
)
RENDERED_MESH_ADJACENCY_MAX_SEPARATION_M = 0.20
RENDERED_MESH_ADJACENCY_PAIR_LIMITS_M: dict[tuple[str, str], float] = {}
REQUIRED_UR5_FINAL_VIEWPORT_LINKS: tuple[str, ...] = (
    "base_link_inertia",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
)
RETAINED_VISUAL_ROWS_MISSING_WARNING = "retained visual rows missing after loader filtering"
UR5_ACCEPTED_FINAL_RENDER_STATUSES: frozenset[str] = frozenset(
    {
        "ok",
        "ur5_emergency_visible_fallback",
        "ur5_emergency_fallback",
        "ur5_emergency_fallback_ok",
        "ur5_emergency_fallback_drawn",
    }
)
UR5_FINAL_RENDER_SOURCE_LAYERS: frozenset[str] = frozenset(
    {"locked_generated_urdf_visual", "generated_urdf_visual"}
)


def _rendered_mesh_adjacency_limit(parent: str, child: str) -> float:
    return RENDERED_MESH_ADJACENCY_PAIR_LIMITS_M.get((parent, child), RENDERED_MESH_ADJACENCY_MAX_SEPARATION_M)


def _payload_scene_name(payload: dict[str, Any]) -> str | None:
    scene = payload.get("scene")
    if isinstance(scene, str) and scene.strip():
        return scene.strip()
    scene_path = payload.get("scene_path")
    if isinstance(scene_path, str) and scene_path.strip():
        return Path(scene_path).name
    return None


def _payload_message_values(payload: dict[str, Any]) -> list[str]:
    values: list[str] = []
    for key in ("warnings", "blockers", "messages", "log_lines", "stdout_tail", "stderr_tail"):
        raw = payload.get(key)
        if isinstance(raw, list):
            values.extend(str(item) for item in raw)
        elif isinstance(raw, str):
            values.append(raw)
    for key in ("warning_messages", "blocker_messages"):
        raw = payload.get(key)
        if isinstance(raw, dict):
            values.extend(str(item) for item in raw.values())
    return values


def _final_visible_viewport_rows(payload: dict[str, Any]) -> list[dict[str, Any]]:
    rows = _first_present_list(payload, "final_draw_visual_items", "final_draw_diagnostics", "viewport_visible_items", "rendered_items", "visible_items")
    return [row for row in rows if isinstance(row, dict)]


def _row_canonical_link_candidates(row: dict[str, Any]) -> set[str]:
    candidates: set[str] = set()
    for key in ("link", "link_name", "canonical_link_name", "frame_id", "visual_name", "item_id", "id", "display_name"):
        canonical = _canonical_ur5_rendered_mesh_link_name(row.get(key))
        if canonical:
            candidates.add(canonical)
        value = row.get(key)
        if isinstance(value, str) and value.strip():
            candidates.add(value.strip())
    for canonical in _normalized_link_chain(row):
        candidates.add(canonical)
    return candidates


def _accepted_final_render_status(row: dict[str, Any]) -> bool:
    return str(row.get("final_draw_status") or "").strip().lower() in UR5_ACCEPTED_FINAL_RENDER_STATUSES


def _canonical_final_render_identity(value: Any) -> str | None:
    if not isinstance(value, str) or not value.strip():
        return None
    raw = value.strip().lower()
    canonical = _canonical_ur5_rendered_mesh_link_name(value)
    if canonical == "base_link" and raw == "base_link_inertia":
        return "base_link_inertia"
    return canonical or raw


def _row_final_rendered_ur5_link_identities(row: dict[str, Any]) -> set[str]:
    """Return only actual final draw identity fields, not chain/index metadata."""
    links: set[str] = set()
    for key in ("canonical_link_name", "link", "link_name"):
        canonical = _canonical_final_render_identity(row.get(key))
        if canonical:
            links.add(canonical)
    return links


def _row_is_final_visible_renderable(row: dict[str, Any]) -> bool:
    if row.get("visible") is False or row.get("rendered") is False:
        return False
    status = str(row.get("final_draw_status") or row.get("draw_status") or "").strip().lower()
    if status in {"missing_mesh_cache", "missing_bounds", "invalid_mesh", "non_finite_bounds", "skipped", "filtered"}:
        return False
    if status == "ok":
        return True
    if row.get("final_draw_bbox") or row.get("final_draw_bbox_min"):
        return True
    if row.get("active_visual_source") in {"primitive_fallback", "urdf_primitive"}:
        return True
    return bool(row.get("has_mesh_metadata") or row.get("mesh_path") or row.get("mesh_source") or row.get("primitive_geometry_type"))


def _table_camera_visible(payload: dict[str, Any], rows: list[dict[str, Any]]) -> tuple[bool, bool]:
    audit = _first_present_mapping(_first_present_mapping(payload, "filter_diagnostics"), "ur5_2f_test_final_viewport_audit")
    table_visible = _as_int(audit.get("rendered_table_count")) > 0
    camera_visible = _as_int(audit.get("rendered_camera_count")) > 0
    labels = [label.lower() for label in _visible_item_labels_from_payload(payload)]
    if any("table" in label or "workbench" in label or "support_surface" in label for label in labels):
        table_visible = True
    if any("camera" in label or "sensor" in label for label in labels):
        camera_visible = True
    for row in rows:
        text = " ".join(str(row.get(key) or "") for key in ("id", "item_id", "display_name", "link", "link_name", "category", "role")).lower()
        if "table" in text or "workbench" in text or "support_surface" in text:
            table_visible = True
        if "camera" in text or "sensor" in text:
            camera_visible = True
    return table_visible, camera_visible


def _apply_ur5_final_viewport_payload_contract(payload: dict[str, Any]) -> None:
    """Validate UR5 evidence from final visible/rendered viewport rows only."""
    if _payload_scene_name(payload) != "ur5_2f_test":
        return

    rows = _final_visible_viewport_rows(payload)
    visible_rows = [row for row in rows if _row_is_final_visible_renderable(row)]

    visible_robot_render_rows = [
        row for row in visible_rows
        if str(row.get("source_layer") or "").strip().lower() in UR5_FINAL_RENDER_SOURCE_LAYERS
        and _accepted_final_render_status(row)
    ]
    required_ur5_links = set(REQUIRED_UR5_FINAL_VIEWPORT_LINKS)
    visible_links: set[str] = set()
    for row in visible_robot_render_rows:
        visible_links.update(_row_final_rendered_ur5_link_identities(row) & required_ur5_links)

    robot_mesh_rows = [row for row in visible_robot_render_rows if _bbox_from_final_draw(row) is not None]
    ur5_mesh_rows = [row for row in visible_robot_render_rows if _row_final_rendered_ur5_link_identities(row) & required_ur5_links]
    robotiq_mesh_rows = [row for row in visible_robot_render_rows if _is_robotiq_base_record(row) or "robotiq" in " ".join(
        str(row.get(key) or "")
        for key in ("canonical_link_name", "link", "link_name", "visual_name", "item_id", "id", "mesh_uri", "package_uri", "mesh_path")
    ).lower()]
    payload["rviz_parity_robot_layer"] = bool(robot_mesh_rows)
    payload["robot_mesh_renderables_count"] = len(visible_robot_render_rows)
    payload["ur5_mesh_renderables_count"] = len(ur5_mesh_rows)
    payload["robotiq_mesh_renderables_count"] = len(robotiq_mesh_rows)

    robot_aabb_min = [math.inf, math.inf, math.inf]
    robot_aabb_max = [-math.inf, -math.inf, -math.inf]
    for row in visible_robot_render_rows:
        bbox = _bbox_from_final_draw(row)
        if not bbox:
            continue
        for axis in range(3):
            robot_aabb_min[axis] = min(robot_aabb_min[axis], bbox["min"][axis])
            robot_aabb_max[axis] = max(robot_aabb_max[axis], bbox["max"][axis])
    robot_aabb_valid = all(math.isfinite(v) for v in robot_aabb_min + robot_aabb_max) and any(
        robot_aabb_max[i] > robot_aabb_min[i] for i in range(3)
    )
    payload["robot_aabb_min"] = robot_aabb_min if robot_aabb_valid else None
    payload["robot_aabb_max"] = robot_aabb_max if robot_aabb_valid else None

    missing = [link for link in REQUIRED_UR5_FINAL_VIEWPORT_LINKS if link not in visible_links]
    rendered_ur5_link_count = len([link for link in REQUIRED_UR5_FINAL_VIEWPORT_LINKS if link in visible_links])
    payload["rendered_ur5_link_count"] = rendered_ur5_link_count
    payload["required_ur5_final_viewport_links"] = list(REQUIRED_UR5_FINAL_VIEWPORT_LINKS)
    payload["missing_required_visible_ur5_links"] = missing

    blockers = payload.get("blockers") if isinstance(payload.get("blockers"), list) else []
    if not payload["rviz_parity_robot_layer"]:
        _append_unique(blockers, "rviz_parity_robot_layer_missing")
    if payload["robot_mesh_renderables_count"] < 7:
        _append_unique(blockers, "robot_mesh_renderables_count_below_7")
    if payload["ur5_mesh_renderables_count"] < len(REQUIRED_UR5_FINAL_VIEWPORT_LINKS):
        _append_unique(blockers, "ur5_mesh_renderables_count_below_required_links")
    if payload["robotiq_mesh_renderables_count"] <= 0:
        _append_unique(blockers, "robotiq_mesh_renderables_missing")
    if not robot_aabb_valid:
        _append_unique(blockers, "robot_aabb_empty_or_invalid")
    if missing:
        _append_unique(blockers, "ur5_final_viewport_links_missing")
    if payload["rendered_ur5_link_count"] < len(REQUIRED_UR5_FINAL_VIEWPORT_LINKS):
        _append_unique(blockers, "rendered_ur5_link_count_below_7")

    camera_fit_target = str(payload.get("camera_fit_target") or "").strip().lower()
    camera_fit_includes_robot = robot_aabb_valid and (
        "robot" in camera_fit_target or "ur5_included" in camera_fit_target or "physical_initial_fit" in camera_fit_target
    )
    payload["camera_fit_includes_robot"] = camera_fit_includes_robot
    if not camera_fit_includes_robot:
        _append_unique(blockers, "camera_fit_does_not_include_robot")

    table_visible, camera_visible = _table_camera_visible(payload, rows)
    payload["table_visible_in_final_viewport"] = table_visible
    payload["camera_visible_in_final_viewport"] = camera_visible
    if not table_visible:
        _append_unique(blockers, "table_not_visible_in_final_viewport")
    if not camera_visible:
        _append_unique(blockers, "camera_not_visible_in_final_viewport")

    retained_warning_present = any(RETAINED_VISUAL_ROWS_MISSING_WARNING in value for value in _payload_message_values(payload))
    if retained_warning_present and not missing:
        _append_unique(blockers, "stale_retained_visual_rows_missing_warning")

    generated_mesh_rows = [
        row for row in rows
        if str(row.get("source_layer") or "").strip().lower() in {"locked_generated_urdf_visual", "generated_urdf_visual"}
        and (row.get("has_mesh_metadata") or row.get("mesh_source") or row.get("mesh_path"))
    ]
    generated_mesh_missing_renderables = bool(generated_mesh_rows) and not any(
        _row_is_final_visible_renderable(row) and str(row.get("final_draw_status") or "").strip().lower() == "ok"
        for row in generated_mesh_rows
    )
    fallback_count = _counter(
        payload,
        "generated_fallback_count",
        "primitive_fallback_rendered_count",
        "urdf_primitive_rendered_count",
        "valid_physical_fallback_count",
    )
    payload["generated_robot_fallback_required"] = generated_mesh_missing_renderables
    if generated_mesh_missing_renderables and fallback_count <= 0:
        _append_unique(blockers, "generated_robot_fallback_not_activated")

    if blockers:
        payload["blockers"] = blockers
        payload["status"] = "FAIL"


def _finite_float_list(value: Any, length: int) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < length:
        return None
    out: list[float] = []
    for raw in value[:length]:
        try:
            f = float(raw)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(f):
            return None
        out.append(f)
    return out


def _pose_xyz_rpy(record: dict[str, Any]) -> tuple[list[float], list[float]] | None:
    for key in ("baked_world_visual_pose", "pose", "world_pose"):
        pose = record.get(key)
        if not isinstance(pose, dict):
            continue
        xyz = _finite_float_list(pose.get("xyz"), 3)
        rpy = _finite_float_list(pose.get("rpy"), 3) or [0.0, 0.0, 0.0]
        if xyz is not None:
            return xyz, rpy
    return None


def _mesh_scale(record: dict[str, Any]) -> list[float]:
    return _finite_float_list(record.get("mesh_scale"), 3) or [1.0, 1.0, 1.0]


def _matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _rpy_matrix(rpy: list[float]) -> list[list[float]]:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rz = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]
    ry = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]
    rx = [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]
    return _matmul(_matmul(rz, ry), rx)


def _transform_point(point: list[float], xyz: list[float], rpy: list[float], scale: list[float]) -> list[float]:
    scaled = [point[i] * scale[i] for i in range(3)]
    rot = _rpy_matrix(rpy)
    return [xyz[i] + sum(rot[i][j] * scaled[j] for j in range(3)) for i in range(3)]


def _dae_local_bounds(path: Path) -> tuple[list[float], list[float]] | None:
    try:
        root = ET.parse(path).getroot()
    except Exception:
        return None
    mins = [math.inf, math.inf, math.inf]
    maxs = [-math.inf, -math.inf, -math.inf]
    found = False
    for elem in root.iter():
        if not elem.tag.endswith("float_array"):
            continue
        elem_id = str(elem.attrib.get("id", "")).lower()
        if "position" not in elem_id and "positions" not in elem_id:
            continue
        values = (elem.text or "").split()
        for idx in range(0, len(values) - 2, 3):
            try:
                point = [float(values[idx]), float(values[idx + 1]), float(values[idx + 2])]
            except ValueError:
                continue
            if not all(math.isfinite(v) for v in point):
                continue
            found = True
            for axis in range(3):
                mins[axis] = min(mins[axis], point[axis])
                maxs[axis] = max(maxs[axis], point[axis])
    return (mins, maxs) if found else None


def _world_bounds_for_visual(repo_root: Path, record: dict[str, Any]) -> dict[str, list[float]] | None:
    existing = record.get("world_bounds") or record.get("rendered_world_bounds") or record.get("bounds")
    if isinstance(existing, dict):
        mn = _finite_float_list(existing.get("min"), 3)
        mx = _finite_float_list(existing.get("max"), 3)
        if mn is not None and mx is not None:
            return {"min": mn, "max": mx}
    resolved = str(record.get("resolved_source_path") or record.get("resolved_path") or "").strip()
    if not resolved:
        return None
    mesh_path = Path(resolved)
    if not mesh_path.is_absolute():
        mesh_path = repo_root / mesh_path
    if mesh_path.suffix.lower() != ".dae" or not mesh_path.is_file():
        return None
    local = _dae_local_bounds(mesh_path)
    pose = _pose_xyz_rpy(record)
    if local is None or pose is None:
        return None
    lmin, lmax = local
    xyz, rpy = pose
    scale = _mesh_scale(record)
    wmins = [math.inf, math.inf, math.inf]
    wmaxs = [-math.inf, -math.inf, -math.inf]
    for xi in (lmin[0], lmax[0]):
        for yi in (lmin[1], lmax[1]):
            for zi in (lmin[2], lmax[2]):
                wp = _transform_point([xi, yi, zi], xyz, rpy, scale)
                for axis in range(3):
                    wmins[axis] = min(wmins[axis], wp[axis])
                    wmaxs[axis] = max(wmaxs[axis], wp[axis])
    return {"min": wmins, "max": wmaxs}


def _aabb_separation(a: dict[str, list[float]], b: dict[str, list[float]]) -> float:
    sq = 0.0
    for axis in range(3):
        if a["max"][axis] < b["min"][axis]:
            gap = b["min"][axis] - a["max"][axis]
        elif b["max"][axis] < a["min"][axis]:
            gap = a["min"][axis] - b["max"][axis]
        else:
            gap = 0.0
        sq += gap * gap
    return math.sqrt(sq)


def _record_rendered_mesh_adjacency_failure(payload: dict[str, Any], errors: list[str]) -> None:
    if not errors:
        return
    warnings = payload.get("warnings")
    if not isinstance(warnings, list):
        warnings = []
    _append_unique(warnings, "scene3d_rendered_mesh_adjacency_failed")
    payload["warnings"] = warnings
    if str(payload.get("status", "")).upper() in {"PASS", "OK"}:
        payload["status"] = "WARN"
    messages = payload.get("warning_messages")
    if not isinstance(messages, dict):
        messages = {}
    messages["scene3d_rendered_mesh_adjacency_failed"] = "; ".join(errors)
    payload["warning_messages"] = messages


def _record_rendered_mesh_adjacency_fallback_warning(payload: dict[str, Any]) -> None:
    warnings = payload.get("warnings")
    if not isinstance(warnings, list):
        warnings = []
    _append_unique(warnings, "rendered_mesh_adjacency_used_index_fallback")
    payload["warnings"] = warnings
    messages = payload.get("warning_messages")
    if not isinstance(messages, dict):
        messages = {}
    messages["rendered_mesh_adjacency_used_index_fallback"] = (
        "Final draw diagnostics were unavailable; UR5 rendered mesh adjacency was checked with visual-index/world-bounds fallback data."
    )
    payload["warning_messages"] = messages


def _bbox_from_final_draw(record: dict[str, Any]) -> dict[str, list[float]] | None:
    bbox = record.get("final_draw_bbox")
    if not isinstance(bbox, dict):
        return None
    mn = _finite_float_list(bbox.get("min"), 3)
    mx = _finite_float_list(bbox.get("max"), 3)
    if mn is None or mx is None:
        return None
    return {"min": mn, "max": mx}


def _item_id(record: dict[str, Any]) -> str | None:
    for key in ("item_id", "id", "name", "label"):
        value = record.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def _mesh_path(record: dict[str, Any]) -> str | None:
    for key in ("mesh_path", "resolved_source_path", "resolved_path", "source_path", "path"):
        value = record.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def _is_robotiq_base_record(record: dict[str, Any]) -> bool:
    canonical = str(record.get("canonical_link_name") or record.get("link_name") or record.get("link") or "").strip().lower()
    if canonical in {"gripper_base_link", "robotiq_85_base_link", "robotiq_base"}:
        return True
    haystack = " ".join(
        str(record.get(key) or "")
        for key in ("canonical_link_name", "link", "link_name", "visual_name", "name", "id", "item_id", "label", "mesh_uri", "package_uri", "mesh_path", "resolved_source_path", "resolved_path", "source_path")
    ).lower()
    return "robotiq" in haystack and ("base" in haystack or "2f" in haystack or "arg2f" in haystack)


def _canonical_ur5_rendered_mesh_link_name(value: Any) -> str | None:
    if not isinstance(value, str) or not value.strip():
        return None
    text = value.strip()
    lowered = text.lower()
    alias_to_canonical: dict[str, str] = {}
    for canonical, aliases in UR5_RENDERED_MESH_LINK_ALIASES.items():
        alias_to_canonical[canonical] = canonical
        for alias in aliases:
            alias_to_canonical[alias] = canonical
    if lowered in alias_to_canonical:
        return alias_to_canonical[lowered]
    for alias, canonical in alias_to_canonical.items():
        if alias and alias in lowered:
            return canonical
    return None


def _normalized_link_chain(record: dict[str, Any]) -> list[str]:
    raw_chain = record.get("link_chain")
    if not isinstance(raw_chain, list):
        return []
    chain: list[str] = []
    for raw in raw_chain:
        canonical = _canonical_ur5_rendered_mesh_link_name(raw)
        if canonical and canonical not in chain:
            chain.append(canonical)
    return chain


def _record_stable_link(record: dict[str, Any]) -> str | None:
    for key in ("link", "link_name", "canonical_link_name"):
        canonical = _canonical_ur5_rendered_mesh_link_name(record.get(key))
        if canonical:
            return canonical
    return None


def _record_parent_links(record: dict[str, Any]) -> set[str]:
    parents: set[str] = set()
    for key in ("parent_link", "immediate_parent_link"):
        canonical = _canonical_ur5_rendered_mesh_link_name(record.get(key))
        if canonical:
            parents.add(canonical)
    chain = _normalized_link_chain(record)
    child = _record_stable_link(record)
    if child in chain:
        child_index = chain.index(child)
        if child_index > 0:
            parents.add(chain[child_index - 1])
    return parents


def _stable_metadata_proves_adjacency(parent: str, child: str, child_item: dict[str, Any] | None) -> bool:
    if child_item is None:
        return False
    if parent in _record_parent_links(child_item):
        return True
    chain = _normalized_link_chain(child_item)
    if parent in chain and child in chain:
        return chain.index(parent) < chain.index(child)
    return False


def _stable_metadata_candidates(record: dict[str, Any]) -> list[str]:
    candidates: list[str] = []
    for key in ("link", "link_name", "canonical_link_name", "parent_link", "immediate_parent_link"):
        canonical = _canonical_ur5_rendered_mesh_link_name(record.get(key))
        if canonical and canonical not in candidates:
            candidates.append(canonical)
    for canonical in _normalized_link_chain(record):
        if canonical and canonical not in candidates:
            candidates.append(canonical)
    return candidates


def _bbox_gap(parent_item: dict[str, Any] | None, child_item: dict[str, Any] | None) -> list[float] | None:
    pb = (parent_item or {}).get("bounds")
    cb = (child_item or {}).get("bounds")
    if not isinstance(pb, dict) or not isinstance(cb, dict):
        return None
    pmin, pmax, cmin, cmax = pb.get("min"), pb.get("max"), cb.get("min"), cb.get("max")
    if not all(isinstance(v, list) and len(v) == 3 for v in (pmin, pmax, cmin, cmax)):
        return None
    return [max(0.0, max(float(cmin[i]) - float(pmax[i]), float(pmin[i]) - float(cmax[i]))) for i in range(3)]

def _checked_pair_record(
    parent: str,
    child: str,
    parent_item: dict[str, Any] | None,
    child_item: dict[str, Any] | None,
    sep: float | None,
    limit_m: float,
    ok: bool,
) -> dict[str, Any]:
    parent_link = (
        (parent_item or {}).get("canonical_link_name")
        or (parent_item or {}).get("link")
        or (parent_item or {}).get("link_name")
        or parent
    )
    child_link = (
        (child_item or {}).get("canonical_link_name")
        or (child_item or {}).get("link")
        or (child_item or {}).get("link_name")
        or child
    )
    return {
        "parent": parent,
        "child": child,
        "parent_link": parent_link,
        "child_link": child_link,
        "parent_item_id": _item_id(parent_item or {}),
        "child_item_id": _item_id(child_item or {}),
        "parent_link_name": (parent_item or {}).get("link_name"),
        "child_link_name": (child_item or {}).get("link_name"),
        "parent_canonical_link_name": (parent_item or {}).get("canonical_link_name") or (parent_item or {}).get("link"),
        "child_canonical_link_name": (child_item or {}).get("canonical_link_name") or (child_item or {}).get("link"),
        "parent_mesh_path": _mesh_path(parent_item or {}),
        "child_mesh_path": _mesh_path(child_item or {}),
        "parent_bbox_min": (parent_item or {}).get("bounds", {}).get("min") if isinstance((parent_item or {}).get("bounds"), dict) else None,
        "parent_bbox_max": (parent_item or {}).get("bounds", {}).get("max") if isinstance((parent_item or {}).get("bounds"), dict) else None,
        "child_bbox_min": (child_item or {}).get("bounds", {}).get("min") if isinstance((child_item or {}).get("bounds"), dict) else None,
        "child_bbox_max": (child_item or {}).get("bounds", {}).get("max") if isinstance((child_item or {}).get("bounds"), dict) else None,
        "separation_m": sep,
        "bbox_gap_m": _bbox_gap(parent_item, child_item),
        "limit_m": limit_m,
        "threshold_m": limit_m,
        "passed": ok,
        "ok": ok,
    }


def _final_draw_bbox_from_row(raw: dict[str, Any]) -> dict[str, list[float]] | None:
    bbox = raw.get("final_draw_bbox")
    if not isinstance(bbox, dict):
        return None
    bmin = _finite_float_list(bbox.get("min"), 3)
    bmax = _finite_float_list(bbox.get("max"), 3)
    if bmin is None or bmax is None:
        return None
    return {"min": bmin, "max": bmax}


def _apply_ur5_rendered_mesh_adjacency(payload: dict[str, Any], *, repo_root: Path, scene_name: str | None, index_data: dict[str, Any]) -> None:
    if scene_name != "ur5_2f_test":
        payload.setdefault("rendered_mesh_adjacency_status", "SKIPPED")
        payload.setdefault("rendered_mesh_adjacency_errors", [])
        payload.setdefault("rendered_mesh_adjacency_checked_pairs", [])
        return

    final_draw_items = payload.get("final_draw_visual_items")
    if not isinstance(final_draw_items, list) or not final_draw_items:
        final_draw_items = payload.get("final_draw_diagnostics")
    errors: list[str] = []
    source = "final_draw_visual_items"
    if not isinstance(final_draw_items, list) or not final_draw_items:
        index_items = index_data.get("visual_items") or index_data.get("items") or []
        if isinstance(index_items, list):
            payload["rendered_mesh_adjacency_visual_index_supplemental_count"] = len(index_items)
        errors.append(
            "Final Scene3D viewport/renderable diagnostics are missing; "
            "visual-index metadata cannot prove UR5 arm visibility"
        )
        payload["rendered_mesh_adjacency_source"] = "missing_final_draw_diagnostics"
        payload["rendered_mesh_adjacency_status"] = "FAIL"
        payload["rendered_mesh_adjacency_errors"] = errors
        payload["rendered_mesh_adjacency_checked_pairs"] = []
        _record_rendered_mesh_adjacency_failure(payload, errors)
        return

    payload["rendered_mesh_adjacency_source"] = source
    alias_to_canonical: dict[str, str] = {}
    if source == "final_draw_visual_items":
        final_rows = [row for row in final_draw_items if isinstance(row, dict)]
        by_source_row = {row.get("source_row_index"): row for row in final_rows if row.get("source_row_index") is not None}
        expected_source_rows = {
            0: "base_link_inertia",
            1: "shoulder_link",
            2: "upper_arm_link",
            3: "forearm_link",
            4: "wrist_1_link",
            5: "wrist_2_link",
            6: "wrist_3_link",
        }
        final_links = {str(row.get("link") or row.get("link_name") or "").strip() for row in final_rows}
        final_canonical_links = {
            canonical
            for row in final_rows
            for canonical in _stable_metadata_candidates(row)
        }
        missing_ur5 = [
            link
            for link in expected_source_rows.values()
            if link not in final_links and _canonical_ur5_rendered_mesh_link_name(link) not in final_canonical_links
        ]
        if missing_ur5:
            errors.append("Final Scene3D payload is missing visible/rendered UR5 arm links: " + ",".join(missing_ur5))
        if by_source_row:
            for row_index, expected_link in expected_source_rows.items():
                row = by_source_row.get(row_index)
                actual_link = str((row or {}).get("link") or (row or {}).get("link_name") or "").strip()
                if actual_link != expected_link:
                    errors.append(
                        f"Final Scene3D payload source_row_index={row_index} expected {expected_link} but got {actual_link or '<missing>'}"
                    )
            replacement_links = {"gripper_base_link", "table", "camera", "camera_link"}
            for row_index in expected_source_rows:
                row = by_source_row.get(row_index)
                actual_link = str((row or {}).get("link") or (row or {}).get("link_name") or "").strip()
                if actual_link in replacement_links:
                    errors.append(
                        f"Final Scene3D payload source_row_index={row_index} was replaced by non-UR5 logical row {actual_link}"
                    )

    for canonical, aliases in UR5_RENDERED_MESH_LINK_ALIASES.items():
        for alias in aliases:
            alias_to_canonical[alias] = canonical
    alias_to_canonical["tool0"] = "tool0"

    by_link: dict[str, dict[str, Any]] = {}
    robotiq_base: dict[str, Any] | None = None
    for raw in final_draw_items:
        if not isinstance(raw, dict):
            continue
        bounds = _bbox_from_final_draw(raw) or _final_draw_bbox_from_row(raw)
        normalized = dict(raw)
        if bounds is not None:
            normalized["bounds"] = bounds
        elif str(raw.get("final_draw_status") or "") == "ok":
            normalized["bbox_error"] = "final_draw_bbox_missing_or_non_finite"
        if _is_robotiq_base_record(raw) and robotiq_base is None:
            robotiq_base = normalized
        # Prefer stable generated-URDF metadata over runtime/display ids such as
        # generated_urdf::...; ids are only used as a last-resort diagnostic
        # fallback when metadata is absent.
        for canonical in _stable_metadata_candidates(raw):
            if canonical not in by_link:
                by_link[canonical] = normalized
        for key in ("frame_id", "visual_name", "id", "item_id", "mesh_uri", "package_uri", "mesh_path", "source_path"):
            link = str(raw.get(key) or "").strip()
            candidates = {link, link.lower()}
            for alias, canonical_value in alias_to_canonical.items():
                if alias and alias in link:
                    candidates.add(alias)
            for candidate in candidates:
                canonical = alias_to_canonical.get(candidate)
                if canonical and canonical not in by_link:
                    by_link[canonical] = normalized

    checked: list[dict[str, Any]] = []
    for parent, child in UR5_RENDERED_MESH_ADJACENT_PAIRS:
        parent_item = by_link.get(parent)
        child_item = by_link.get(child)
        if parent_item is None or child_item is None:
            errors.append(f"Missing final draw rendered mesh adjacency link for UR5 pair {parent}->{child}")
            checked.append(
                _checked_pair_record(
                    parent, child, parent_item, child_item, None, _rendered_mesh_adjacency_limit(parent, child), False
                )
            )
            continue
        if _stable_metadata_proves_adjacency(parent, child, child_item):
            checked.append(
                _checked_pair_record(
                    parent, child, parent_item, child_item, None, _rendered_mesh_adjacency_limit(parent, child), True
                )
            )
            checked[-1]["evidence"] = "stable_metadata"
            continue
        if not isinstance(parent_item.get("bounds"), dict) or not isinstance(child_item.get("bounds"), dict):
            detail = parent_item.get("bbox_error") or child_item.get("bbox_error") or "bbox_missing"
            errors.append(f"Missing or non-finite final_draw_bbox diagnostics for UR5 adjacency pair {parent}->{child}: {detail}")
            checked.append(
                _checked_pair_record(
                    parent, child, parent_item, child_item, None, _rendered_mesh_adjacency_limit(parent, child), False
                )
            )
            continue
        sep = _aabb_separation(parent_item["bounds"], child_item["bounds"])
        limit_m = _rendered_mesh_adjacency_limit(parent, child)
        ok = math.isfinite(sep) and sep <= limit_m
        checked.append(_checked_pair_record(parent, child, parent_item, child_item, sep, limit_m, ok))
        if not ok:
            errors.append(
                f"UR5 final draw bbox adjacency {parent}->{child} separated by {sep:.3f} m; expected <= {limit_m:.3f} m"
            )

    tool_parent = by_link.get("tool0") or by_link.get("wrist_3_link")
    tool_parent_name = "tool0" if by_link.get("tool0") is not None else "wrist_3_link"
    if robotiq_base is None or tool_parent is None:
        errors.append("Robotiq base could not be associated with wrist_3_link or tool0 final draw diagnostics")
        checked.append(
            _checked_pair_record(
                tool_parent_name,
                "robotiq_base",
                tool_parent,
                robotiq_base,
                None,
                _rendered_mesh_adjacency_limit(tool_parent_name, "robotiq_base"),
                False,
            )
        )
    elif _stable_metadata_proves_adjacency(tool_parent_name, "robotiq_base", robotiq_base) or _stable_metadata_proves_adjacency("wrist_3_link", "robotiq_base", robotiq_base):
        checked.append(
            _checked_pair_record(
                tool_parent_name,
                "robotiq_base",
                tool_parent,
                robotiq_base,
                None,
                _rendered_mesh_adjacency_limit(tool_parent_name, "robotiq_base"),
                True,
            )
        )
        checked[-1]["evidence"] = "stable_metadata"
    elif not isinstance(tool_parent.get("bounds"), dict) or not isinstance(robotiq_base.get("bounds"), dict):
        detail = tool_parent.get("bbox_error") or robotiq_base.get("bbox_error") or "bbox_missing"
        errors.append(f"Missing or non-finite final_draw_bbox diagnostics for Robotiq base association with wrist_3_link/tool0: {detail}")
        checked.append(
            _checked_pair_record(
                tool_parent_name,
                "robotiq_base",
                tool_parent,
                robotiq_base,
                None,
                _rendered_mesh_adjacency_limit(tool_parent_name, "robotiq_base"),
                False,
            )
        )
    else:
        sep = _aabb_separation(tool_parent["bounds"], robotiq_base["bounds"])
        limit_m = _rendered_mesh_adjacency_limit(tool_parent_name, "robotiq_base")
        ok = math.isfinite(sep) and sep <= limit_m
        checked.append(_checked_pair_record(tool_parent_name, "robotiq_base", tool_parent, robotiq_base, sep, limit_m, ok))
        if not ok:
            errors.append(
                f"Robotiq base final draw bbox separated from {tool_parent_name} by {sep:.3f} m; expected <= {limit_m:.3f} m"
            )

    payload["rendered_mesh_adjacency_status"] = "PASS" if not errors else "FAIL"
    payload["rendered_mesh_adjacency_errors"] = errors
    payload["rendered_mesh_adjacency_checked_pairs"] = checked
    _record_rendered_mesh_adjacency_failure(payload, errors)


def _xyz_from_array(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) != 3:
        return None
    out: list[float] = []
    for component in value:
        if not isinstance(component, (int, float)):
            return None
        f = float(component)
        if not math.isfinite(f):
            return None
        out.append(f)
    return out


def _bbox_center(row: dict[str, Any]) -> list[float] | None:
    lo = _xyz_from_array(row.get("final_draw_bbox_min"))
    hi = _xyz_from_array(row.get("final_draw_bbox_max"))
    if lo is None or hi is None:
        return None
    return [(a + b) * 0.5 for a, b in zip(lo, hi)]


def _dist(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _apply_ur5_final_draw_bbox_regression(payload: dict[str, Any]) -> None:
    """Fail the smoke on exploded UR5 final draw bounds, not mesh-index poses."""
    if str(payload.get("scene") or "") != "ur5_2f_test":
        return
    rows = _first_present_list(payload, "final_draw_visual_items")
    if not rows:
        payload.setdefault("ur5_final_draw_bbox_status", "SKIPPED")
        return
    centers: dict[str, list[float]] = {}
    for row_any in rows:
        if not isinstance(row_any, dict):
            continue
        token = "|".join(str(row_any.get(k, "")).lower() for k in ("item_id", "link", "frame_id", "mesh_source"))
        center = _bbox_center(row_any)
        if center is None:
            continue
        for link in ("base", "shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3"):
            if link in token and link not in centers:
                centers[link] = center
    chain = ("base", "shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3")
    limits = {
        ("base", "shoulder"): 0.35,
        ("shoulder", "upper_arm"): 0.55,
        ("upper_arm", "forearm"): 0.80,
        ("forearm", "wrist_1"): 0.80,
        ("wrist_1", "wrist_2"): 0.40,
        ("wrist_2", "wrist_3"): 0.40,
    }
    errors: list[str] = []
    distances: dict[str, float] = {}
    for parent, child in zip(chain, chain[1:]):
        if parent not in centers or child not in centers:
            continue
        d = _dist(centers[parent], centers[child])
        distances[f"{parent}_to_{child}"] = d
        limit = limits[(parent, child)]
        if d > limit:
            errors.append(f"final draw bbox center distance {parent}->{child} is {d:.3f} m; expected <= {limit:.3f} m")
    payload["ur5_final_draw_bbox_distances_m"] = distances
    payload["ur5_final_draw_bbox_status"] = "FAIL" if errors else "PASS"
    payload["ur5_final_draw_bbox_errors"] = errors
    if errors:
        blockers = payload.get("blockers") if isinstance(payload.get("blockers"), list) else []
        _append_unique(blockers, "ur5_final_draw_bbox_regression_failed")
        payload["blockers"] = blockers
        payload["status"] = "FAIL"

def _downgrade_preview_ready_language(payload: dict[str, Any], warning_message: str) -> None:
    replacements = {
        "3D Preview Ready": "3D Preview Warning",
        "fully ready": "warning",
    }
    for key in ("message", "readiness_message", "preview_status", "scene3d_status", "status_text"):
        value = payload.get(key)
        if not isinstance(value, str):
            continue
        updated = value
        for before, after in replacements.items():
            updated = updated.replace(before, after)
        if updated != value:
            payload[key] = updated
    messages = payload.get("warning_messages")
    if not isinstance(messages, dict):
        messages = {}
    messages["urdf_transform_parity_failed"] = warning_message
    payload["warning_messages"] = messages


def _apply_ur5_transform_parity(payload: dict[str, Any], *, repo_root: Path, args: argparse.Namespace) -> None:
    scene_name = _scene_name_from_payload_or_args(payload, args)
    if scene_name != "ur5_2f_test":
        payload.setdefault("urdf_transform_parity_status", "SKIPPED")
        payload.setdefault("urdf_transform_parity_errors", [])
        payload.setdefault("baked_world_visual_transform_count", 0)
        _apply_ur5_rendered_mesh_adjacency(payload, repo_root=repo_root, scene_name=scene_name, index_data={})
        return
    index_path = _mesh_index_path_for_scene(repo_root, payload, args)
    payload["urdf_transform_parity_index_path"] = str(index_path) if index_path else None
    payload["urdf_transform_parity_scene"] = scene_name
    if index_path is None or not index_path.is_file():
        report = {"ok": False, "errors": [f"Missing mesh index for scene {scene_name}: {index_path}"]}
        index_data: dict[str, Any] = {}
    else:
        try:
            index_data = json.loads(index_path.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001 - validate_index will provide the detailed read/parse error.
            index_data = {}
        report = validate_ur5_transform_plausibility_index(index_path)
    errors = list(report.get("errors") or []) if isinstance(report, dict) else ["UR5 transform parity validator did not return a report"]
    parity_ok = bool(isinstance(report, dict) and report.get("ok"))
    payload["urdf_transform_parity_status"] = "PASS" if parity_ok else "FAIL"
    payload["urdf_transform_parity_errors"] = errors
    payload["urdf_transform_parity_report"] = report
    payload["baked_world_visual_transform_count"] = _baked_world_visual_transform_count(index_data)
    _apply_ur5_rendered_mesh_adjacency(payload, repo_root=repo_root, scene_name=scene_name, index_data=index_data)
    if parity_ok:
        return
    warning = (
        f"URDF transform parity failed for scene {scene_name}; inspect or regenerate mesh index {index_path}. "
        + ("; ".join(errors) if errors else "No detailed validator errors were reported.")
    )
    warnings = payload.get("warnings")
    if not isinstance(warnings, list):
        warnings = []
    _append_unique(warnings, "urdf_transform_parity_failed")
    payload["warnings"] = warnings
    if str(payload.get("status", "")).upper() in {"PASS", "OK"}:
        payload["status"] = "WARN"
    payload["smoke_result_warning"] = warning
    _downgrade_preview_ready_language(payload, warning)

def _subprocess_exception_to_text(exc: FileNotFoundError | PermissionError) -> str:
    return f"{type(exc).__name__}: {exc}"


WORKCELL_BUILDER_EXECUTABLE_MISSING_MESSAGE = (
    "workcell_builder executable was not found; build workcell_builder in a ROS Humble "
    "workspace and source install/setup.bash, or set WORKCELL_BUILDER_EXECUTABLE"
)


def _executable_guidance() -> list[str]:
    return [
        "cd /home/user/workcell_ws",
        "source /opt/ros/humble/setup.bash",
        "colcon build --symlink-install --packages-select workcell_builder",
        "source install/setup.bash",
        "export WORKCELL_BUILDER_EXECUTABLE=/home/user/workcell_ws/install/workcell_builder/bin/workcell_builder",
    ]


def _non_runtime_static_headless_renderability_counts(static_evidence: dict[str, Any]) -> dict[str, Any]:
    return {
        "runtime_available": False,
        "physical_mesh_items_renderable": static_evidence.get("physical_mesh_items_renderable", 0),
        "primitive_fallback_items_renderable": static_evidence.get("primitive_fallback_items_renderable", 0),
        "zones_overlays_renderable": static_evidence.get("zones_overlays_renderable", 0),
        "missing_mesh_items": static_evidence.get("missing_mesh_items", 0),
        "unresolved_transform_items": static_evidence.get("unresolved_transform_items", 0),
        "skipped_helper_static_fallback_items": static_evidence.get("skipped_helper_static_fallback_items", 0),
        "note": "non-runtime evidence only; static/headless renderability is not GUI runtime PASS evidence",
    }


def _write_blocked_executable_payload(
    output: Path,
    *,
    repo_root: Path,
    workspace_root: Path | None,
    args: argparse.Namespace,
    searched: list[str],
    blocker: str,
    exception: str | None = None,
    warnings: list[str] | None = None,
) -> None:
    scene_dir = _resolve_single_scene_dir(repo_root, args.scene, args.scene_path)
    static_evidence = _static_scene3d_visual_evidence(scene_dir)
    payload: dict[str, Any] = {
        "schema": EXPECTED_SCHEMA,
        "status": "BLOCKED",
        "smoke_status": "MISSING_EXECUTABLE",
        "runtime_available": False,
        "scene": args.scene or (args.scene_path.name if args.scene_path else None),
        "repo_root": str(repo_root),
        "workspace_root": _path_str(workspace_root) if workspace_root else None,
        "executable": None,
        "explicit_executable": str(args.executable) if args.executable else None,
        "searched_paths": list(searched),
        "message": WORKCELL_BUILDER_EXECUTABLE_MISSING_MESSAGE,
        "blockers": [blocker],
        "guidance": _executable_guidance(),
        "warnings": _merge_unique(["runtime_gui_unavailable_static_scene3d_visual_evidence_recorded"], warnings or []),
        "screenshot_available": False,
        "scene_dir": str(scene_dir) if scene_dir else None,
        "static_scene3d_visual_evidence": static_evidence,
        "non_runtime_static_headless_renderability_counts": _non_runtime_static_headless_renderability_counts(static_evidence),
        "render_debug_counters": {
            "runtime_available": False,
            "physical_mesh_items_rendered": 0,
            "primitive_fallback_items_rendered": 0,
            "zones_overlays_rendered": 0,
            "skipped_helper_static_fallback_items": static_evidence.get("skipped_helper_static_fallback_items", 0),
            "unresolved_transform_items": static_evidence.get("unresolved_transform_items", 0),
            "missing_mesh_items": static_evidence.get("missing_mesh_items", 0),
            "static_physical_mesh_items_renderable": static_evidence.get("physical_mesh_items_renderable", 0),
            "static_primitive_fallback_items_renderable": static_evidence.get("primitive_fallback_items_renderable", 0),
            "static_zones_overlays_renderable": static_evidence.get("zones_overlays_renderable", 0),
        },
    }
    if exception:
        payload["subprocess_exception"] = exception
    _add_smoke_report_supplemental_evidence(
        payload,
        screenshot_path=str(args.screenshot) if getattr(args, "screenshot", None) else None,
        screenshot_available=False,
    )
    _add_ros_humble_context(payload)
    _write_json(output, payload)


def _resolve_single_scene_dir(repo_root: Path, scene: str | None, scene_path: Path | None) -> Path | None:
    if scene_path:
        p = scene_path if scene_path.is_absolute() else repo_root / scene_path
        return p.resolve()
    if scene:
        return (repo_root / 'scenes' / scene).resolve()
    return None


def _dims_available(item: dict[str, Any]) -> bool:
    g = str(item.get('geometry_type') or '').strip().lower()
    if g == 'box':
        size = item.get('size')
        return isinstance(size, list) and len(size) >= 3 and all(float(x or 0) > 0 for x in size[:3])
    if g == 'cylinder':
        return float(item.get('radius') or 0) > 0 and float(item.get('length') or 0) > 0
    if g == 'sphere':
        return float(item.get('radius') or 0) > 0
    if g == 'capsule':
        return float(item.get('radius') or 0) > 0 and float(item.get('length') or 0) > 0
    return False


def _static_scene3d_visual_evidence(scene_dir: Path | None) -> dict[str, Any]:
    evidence: dict[str, Any] = {
        'runtime_available': False,
        'physical_mesh_items_rendered': 0,
        'primitive_fallback_items_rendered': 0,
        'zones_overlays_rendered': 0,
        'skipped_helper_static_fallback_items': 0,
        'unresolved_transform_items': 0,
        'missing_mesh_items': 0,
        'physical_mesh_items_renderable': 0,
        'primitive_fallback_items_renderable': 0,
        'zones_overlays_renderable': 0,
        'source': 'generated/scene_visual_mesh_index.json',
        'evidence_kind': 'non_runtime_static_headless_renderability',
        'notes': ['runtime executable unavailable; counts are static/headless renderability evidence, not GUI-render PASS evidence'],
    }
    if scene_dir is None:
        evidence['notes'].append('scene directory could not be resolved')
        return evidence
    index_path = scene_dir / 'generated' / 'scene_visual_mesh_index.json'
    if not index_path.is_file():
        evidence['notes'].append(f'mesh index missing: {index_path}')
        return evidence
    try:
        payload = json.loads(index_path.read_text(encoding='utf-8'))
    except Exception as exc:  # noqa: BLE001
        evidence['notes'].append(f'mesh index unreadable: {exc}')
        return evidence
    items = payload.get('visual_items') or payload.get('items') or []
    if not isinstance(items, list):
        evidence['notes'].append('mesh index visual_items is not a list')
        return evidence
    for raw in items:
        if not isinstance(raw, dict):
            evidence['skipped_helper_static_fallback_items'] += 1
            continue
        geom = str(raw.get('geometry_type') or '').strip().lower()
        category = str(raw.get('category') or '').strip().lower()
        role = str(raw.get('role') or '').strip().lower()
        transform_status = str(raw.get('transform_status') or '').strip().lower()
        warning = ' '.join(str(raw.get(k) or '') for k in ('warning', 'render_skip_reason', 'fallback_reason')).lower()
        if any(token in category or token in role for token in ('overlay', 'zone', 'helper', 'safety')):
            evidence['zones_overlays_renderable'] += 1
            continue
        if transform_status and transform_status not in {'ok', 'resolved', 'static_fallback', 'static_fallback_parent'}:
            evidence['unresolved_transform_items'] += 1
        if 'missing_parent' in warning or 'missing_chain' in warning or 'unresolved transform' in warning:
            evidence['unresolved_transform_items'] += 1
        if bool(raw.get('primitive_fallback')) or transform_status == 'static_fallback':
            if _dims_available(raw):
                evidence['primitive_fallback_items_renderable'] += 1
            else:
                evidence['skipped_helper_static_fallback_items'] += 1
            continue
        if geom == 'mesh':
            resolved = bool(raw.get('resolved'))
            resolved_path = str(raw.get('resolved_source_path') or raw.get('resolved_path') or '').strip()
            if resolved and resolved_path and Path(resolved_path).is_file():
                evidence['physical_mesh_items_renderable'] += 1
            else:
                evidence['missing_mesh_items'] += 1
            continue
        if geom in {'box', 'cylinder', 'sphere', 'capsule'} and _dims_available(raw):
            evidence['primitive_fallback_items_renderable'] += 1
        else:
            evidence['skipped_helper_static_fallback_items'] += 1
    return evidence


def _scene_package_markers_ok(scene_dir: Path) -> tuple[bool, list[str]]:
    required = ["package.xml", "scene_manifest.yaml", "cell_definition.yaml", "launch/demo.launch.py"]
    missing = [name for name in required if not (scene_dir / name).is_file()]
    return (not missing), missing

def _discover_scene_targets(repo_root: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    scenes_root = resolve_scene_root(repo_root)
    for rec in discover_scene3d_scenes(scenes_root):
        rows.append({
            "scene": rec["scene"],
            "scene_path": rec["scene_path"],
            "scene_status": rec["status"],
            "blockers": list(rec.get("blockers") or []),
            "ignore_reason": rec.get("ignore_reason"),
        })
    return sorted(rows, key=lambda x: x["scene"])

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", type=Path, default=_REPO_ROOT)
    ap.add_argument("--workspace-root", type=Path, default=None)
    ap.add_argument("--executable", type=Path, default=None)
    ap.add_argument("--scene", default=None)
    ap.add_argument("--all-scenes", action="store_true")
    ap.add_argument("--scene-path", type=Path, default=None)
    ap.add_argument("--output-dir", type=Path, default=None)
    ap.add_argument("--new-cell-recommended-layout-smoke", action="store_true")
    ap.add_argument("--output", type=Path, default=None)
    ap.add_argument("--screenshot", type=Path, default=None)
    ap.add_argument("--timeout-sec", type=float, default=30.0)
    ap.add_argument("--xvfb", action="store_true")
    args = ap.parse_args()

    if args.all_scenes and args.scene:
        raise SystemExit("Choose only one of --scene or --all-scenes")
    if not args.all_scenes and not args.scene and not args.scene_path and not args.new_cell_recommended_layout_smoke:
        raise SystemExit("Provide one of --scene, --scene-path, --all-scenes, or --new-cell-recommended-layout-smoke")
    if args.all_scenes and args.output is not None:
        raise SystemExit("--output is only valid for single-scene mode")
    if args.all_scenes and args.new_cell_recommended_layout_smoke:
        raise SystemExit("--new-cell-recommended-layout-smoke cannot be combined with --all-scenes")
    if args.all_scenes and args.output_dir is None:
        raise SystemExit("--output-dir is required with --all-scenes")
    if not args.all_scenes and args.output is None:
        raise SystemExit("--output is required unless --all-scenes is used")

    repo_root = resolve_repo_root(explicit_repo_root=args.repo_root)
    workspace_root = resolve_workspace_root(repo_root, args.workspace_root)
    workspace_warnings = _workspace_resolution_warnings(workspace_root, args.workspace_root)
    ros_env = _ros_humble_environment()
    exe = resolve_workcell_builder_executable(workspace_root, args.executable)
    executable_resolution = describe_resolution()
    if exe is None and not args.all_scenes:
        searched = list(executable_resolution.get("searched_executable_paths") or [])
        _write_blocked_executable_payload(
            args.output,
            repo_root=repo_root,
            workspace_root=workspace_root,
            args=args,
            searched=searched,
            blocker="explicit_workcell_builder_executable_missing_or_not_executable" if args.executable else "workcell_builder_executable_missing",
            warnings=workspace_warnings,
        )
        print("status=BLOCKED smoke_status=MISSING_EXECUTABLE")
        print("searched_paths=" + " | ".join(searched))
        return 1

    if not _ros_humble_available() and not args.all_scenes:
        scene_dir = _resolve_single_scene_dir(repo_root, args.scene, args.scene_path)
        static_evidence = _static_scene3d_visual_evidence(scene_dir)
        fail_payload = {
            "schema": EXPECTED_SCHEMA,
            "status": "BLOCKED",
            "runtime_available": False,
            "scene": args.scene or (args.scene_path.name if args.scene_path else None),
            "repo_root": str(repo_root),
            "workspace_root": _path_str(workspace_root),
            "executable": str(exe),
            "blockers": ["ros_humble_unavailable"],
            "warnings": _merge_unique(["runtime_gui_unavailable_static_scene3d_visual_evidence_recorded"], workspace_warnings),
            "screenshot_available": False,
            "scene_dir": str(scene_dir) if scene_dir else None,
            "static_scene3d_visual_evidence": static_evidence,
            "non_runtime_static_headless_renderability_counts": _non_runtime_static_headless_renderability_counts(static_evidence),
            "render_debug_counters": {
                "runtime_available": False,
                "physical_mesh_items_rendered": 0,
                "primitive_fallback_items_rendered": 0,
                "zones_overlays_rendered": 0,
                "skipped_helper_static_fallback_items": static_evidence.get("skipped_helper_static_fallback_items", 0),
                "unresolved_transform_items": static_evidence.get("unresolved_transform_items", 0),
                "missing_mesh_items": static_evidence.get("missing_mesh_items", 0),
                "static_physical_mesh_items_renderable": static_evidence.get("physical_mesh_items_renderable", 0),
                "static_primitive_fallback_items_renderable": static_evidence.get("primitive_fallback_items_renderable", 0),
                "static_zones_overlays_renderable": static_evidence.get("zones_overlays_renderable", 0),
            },
        }
        _add_smoke_report_supplemental_evidence(
            fail_payload,
            screenshot_path=str(args.screenshot) if args.screenshot else None,
            screenshot_available=False,
        )
        _write_json(args.output, fail_payload)
        print("status=BLOCKED smoke_status=ROS_HUMBLE_UNAVAILABLE")
        return 1

    if args.all_scenes:
        out_dir = args.output_dir.resolve()
        scenes = _discover_scene_targets(repo_root)
        per_scene: list[dict[str, Any]] = []
        totals = {"PASS": 0, "FAIL": 0, "BLOCKED": 0, "LEGACY_INCOMPLETE": 0}
        ignored_non_scenes: list[dict[str, Any]] = []
        legacy_incomplete: list[dict[str, Any]] = []
        for item in scenes:
            if item["scene_status"] == "IGNORED_NON_SCENE":
                ignored_non_scenes.append(item)
                continue
            if item["scene_status"] == "LEGACY_INCOMPLETE":
                legacy_incomplete.append(item)
                totals["LEGACY_INCOMPLETE"] += 1
                continue
            scene_name = item["scene"]
            scene_json = out_dir / f"scene3d_gui_smoke_{scene_name}.json"
            scene_png = out_dir / f"scene3d_gui_smoke_{scene_name}.png"
            cmd = [
                sys.executable, str(Path(__file__).resolve()), "--repo-root", str(repo_root),
                "--scene", scene_name, "--output", str(scene_json), "--screenshot", str(scene_png),
            ]
            if workspace_root is not None:
                cmd += ["--workspace-root", str(workspace_root)]
            if exe:
                cmd += ["--executable", str(exe)]
            cmd += ["--timeout-sec", str(args.timeout_sec)]
            if args.xvfb:
                cmd.append("--xvfb")
            proc = None
            run_exception: str | None = None
            try:
                proc = subprocess.run(cmd, cwd=repo_root, capture_output=True, text=True, check=False)
            except (FileNotFoundError, PermissionError) as exc:
                run_exception = _subprocess_exception_to_text(exc)
                _write_blocked_executable_payload(
                    scene_json,
                    repo_root=repo_root,
                    workspace_root=workspace_root,
                    args=argparse.Namespace(
                        scene=scene_name,
                        scene_path=None,
                        executable=args.executable,
                        screenshot=scene_png,
                    ),
                    searched=list(describe_resolution().get("searched_executable_paths") or []),
                    blocker="child_process_spawn_failed",
                    exception=run_exception,
                    warnings=workspace_warnings,
                )
            payload: dict[str, Any] = {}
            if scene_json.exists():
                try:
                    payload = json.loads(scene_json.read_text(encoding="utf-8"))
                except Exception:
                    payload = {}
            smoke_status = str(payload.get("status", "FAIL")).upper()
            returncode = proc.returncode if proc is not None else None
            result_status = "PASS" if (returncode == 0 and smoke_status in {"PASS", "OK"}) else "FAIL"
            if run_exception or smoke_status == "BLOCKED":
                result_status = "BLOCKED"
            if smoke_status == "BLOCKED":
                result_status = "BLOCKED"
            else:
                result_status = "PASS" if (proc.returncode == 0 and smoke_status in {"PASS", "OK"}) else "FAIL"
            blockers = list(payload.get("blockers", [])) if isinstance(payload.get("blockers"), list) else []
            blockers.extend(item.get("blockers", []))
            if item["scene_status"] == "BLOCKED":
                result_status = "BLOCKED"
            totals[result_status] = totals.get(result_status, 0) + 1
            per_scene.append({
                "scene": scene_name,
                "status": result_status,
                "returncode": returncode,
                "smoke_json": str(scene_json),
                "smoke_png": str(scene_png),
                "scene_metadata": item,
                "blockers": blockers,
            })
            if payload:
                payload["scene_level_status"] = result_status
                payload["scene_level_blockers"] = blockers
                _write_json(scene_json, payload)
        summary = {
            "schema": EXPECTED_SCHEMA, "mode": "all_scenes", "output_dir": str(out_dir), "totals": totals, "results": per_scene,
            "repo_root": str(repo_root), "workspace_root": _path_str(workspace_root), "executable": str(exe) if exe else None,
            "executable_resolution": executable_resolution,
            "resolution_warnings": workspace_warnings,
            "supported_scene_count": len(per_scene),
            "legacy_incomplete_count": len(legacy_incomplete),
            "ignored_non_scene_count": len(ignored_non_scenes),
            "legacy_incomplete_scenes": legacy_incomplete,
            "ignored_non_scene_folders": ignored_non_scenes,
        }
        _add_ros_humble_context(summary)
        if not ros_env["ros_humble_available"] and exe is None:
            _record_ros_humble_missing(summary, as_blocker=True)
        _write_json(out_dir / "scene3d_gui_smoke_summary.json", summary)
        md = ["# Scene3D GUI Smoke Summary", "",
              f"- supported_scene_count: {len(per_scene)}",
              f"- PASS: {totals['PASS']}", f"- FAIL: {totals['FAIL']}", f"- BLOCKED: {totals['BLOCKED']}",
              f"- legacy_incomplete_count: {len(legacy_incomplete)}", f"- ignored_non_scene_count: {len(ignored_non_scenes)}",
              "", "| Scene | Status | Return code | JSON | PNG |", "|---|---|---:|---|---|"]
        for r in per_scene:
            md.append(f"| {r['scene']} | {r['status']} | {r['returncode']} | `{r['smoke_json']}` | `{r['smoke_png']}` |")
        if legacy_incomplete:
            md += ["", "## Legacy incomplete scenes"]
            md += [f"- {r['scene']}: {', '.join(r.get('blockers') or ['legacy incomplete'])}" for r in legacy_incomplete]
        if ignored_non_scenes:
            md += ["", "## Ignored non-scene folders"]
            md += [f"- {r['scene']}: {r.get('ignore_reason', 'ignored')}" for r in ignored_non_scenes]
        (out_dir / "scene3d_gui_smoke_summary.md").write_text("\n".join(md) + "\n", encoding="utf-8")
        return 1 if totals["FAIL"] or totals["BLOCKED"] else 0

    if args.scene_path:
        sp = args.scene_path.resolve()
        ok, missing = _scene_package_markers_ok(sp)
        if not ok:
            fail_payload = {
                "schema": EXPECTED_SCHEMA,
                "status": "FAIL",
                "runtime_available": False,
                "scene": args.scene or sp.name,
                "scene_path": str(sp),
                "repo_root": str(repo_root),
                "workspace_root": _path_str(workspace_root),
                "blockers": [f"scene_path_missing_required_files:{','.join(missing)}"],
                "warnings": list(workspace_warnings),
            }
            _add_ros_humble_context(fail_payload)
            if not ros_env["ros_humble_available"]:
                _record_ros_humble_missing(fail_payload, as_blocker=False)
            _add_smoke_report_supplemental_evidence(
                fail_payload,
                screenshot_path=str(args.screenshot) if args.screenshot else None,
                screenshot_available=False,
            )
            _write_json(args.output, fail_payload)
            print("status=FAIL smoke_status=SCENE_PATH_INVALID")
            return 1
        args.scene_path = sp

    if exe is not None and not _executable_can_run(exe):
        fail_payload = {
            "schema": EXPECTED_SCHEMA,
            "status": "FAIL",
            "scene": args.scene or (args.scene_path.name if args.scene_path else None),
            "scene_path": str(args.scene_path) if args.scene_path else None,
            "repo_root": str(repo_root),
            "workspace_root": _path_str(workspace_root),
            "executable": str(exe),
            "blockers": ["explicit_workcell_builder_executable_not_runnable" if args.executable else "resolved_workcell_builder_executable_not_runnable"],
            "warnings": list(workspace_warnings),
            "screenshot_available": False,
        }
        _add_ros_humble_context(fail_payload)
        if not ros_env["ros_humble_available"]:
            _record_ros_humble_missing(fail_payload, as_blocker=True)
        _add_smoke_report_supplemental_evidence(
            fail_payload,
            screenshot_path=str(args.screenshot) if args.screenshot else None,
            screenshot_available=False,
        )
        _write_json(args.output, fail_payload)
        print("status=FAIL smoke_status=EXECUTABLE_NOT_RUNNABLE")
        print("executable=" + str(exe))
        return 1

    cmd = build_cmd(exe, args)
    cmd, xwarn, extra_env = with_xvfb(cmd, args.xvfb)

    stdout_log = args.output.with_suffix(args.output.suffix + ".stdout.log")
    stderr_log = args.output.with_suffix(args.output.suffix + ".stderr.log")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for stale_path in [args.output, stdout_log, stderr_log, args.screenshot]:
        if stale_path and stale_path.exists():
            stale_path.unlink()
    child_env = os.environ.copy()
    child_env.update(extra_env)
    searched_paths = list(executable_resolution.get("searched_executable_paths") or [str(p) for p in _resolve_executable_candidates(workspace_root)])
    try:
        resolution_warnings = _merge_unique(list(executable_resolution.get("warnings") or executable_resolution.get("resolution_warnings") or []), workspace_warnings)
    except NameError:
        resolution_warnings = []
    diag = {
        "schema": EXPECTED_SCHEMA,
        "runtime_available": True,
        "scene": args.scene or (args.scene_path.name if args.scene_path else None),
        "scene_path": str(args.scene_path) if args.scene_path else None,
        "repo_root": str(repo_root),
        "workspace_root": _path_str(workspace_root),
        "executable": str(exe),
        **ros_env,
        "resolved_executable": str(exe),
        "searched_paths": searched_paths,
        "child_command": " ".join(shlex.quote(x) for x in cmd),
        "cwd": str(repo_root),
        "env": {k: child_env.get(k, "") for k in ["DISPLAY", "WAYLAND_DISPLAY", "QT_QPA_PLATFORM", "QT_OPENGL", "LIBGL_ALWAYS_SOFTWARE", "XDG_SESSION_TYPE"]},
        "timeout_sec": args.timeout_sec,
        "stdout_log_path": str(stdout_log),
        "stderr_log_path": str(stderr_log),
        "screenshot_path": str(args.screenshot) if args.screenshot else None,
        "resolution_warnings": resolution_warnings,
    }

    timed_out = False
    rc = None
    stdout = ""
    stderr = ""
    subprocess_exception: str | None = None
    try:
        proc = subprocess.run(cmd, cwd=repo_root, env=child_env, text=True, capture_output=True, timeout=max(0.1, args.timeout_sec), check=False)
        rc = proc.returncode
        stdout, stderr = proc.stdout or "", proc.stderr or ""
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        rc = -1
        stdout, stderr = exc.stdout or "", exc.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode("utf-8", errors="replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode("utf-8", errors="replace")
    except (FileNotFoundError, PermissionError) as exc:
        rc = None
        stdout = ""
        stderr = _subprocess_exception_to_text(exc)
        subprocess_exception = stderr

    stdout_log.parent.mkdir(parents=True, exist_ok=True)
    stdout_log.write_text(stdout, encoding="utf-8")
    stderr_log.write_text(stderr, encoding="utf-8")

    stdout_tail = _tail(stdout)
    stderr_tail = _tail(stderr)
    diag.update({"child_returncode": rc, "timed_out": timed_out, "stdout_tail": stdout_tail, "stderr_tail": stderr_tail, "screenshot_available": bool(args.screenshot and args.screenshot.exists())})

    blockers = list(xwarn)
    warnings: list[str] = list(workspace_warnings)
    if not ros_env["ros_humble_available"]:
        _append_unique(warnings, "ros_humble_missing")
    if timed_out: blockers.append("child_process_timed_out")
    if subprocess_exception:
        blockers.append("child_process_spawn_failed")
        if isinstance(subprocess_exception, str) and subprocess_exception.startswith("FileNotFoundError"):
            blockers.append("workcell_builder_executable_not_found_at_run_time")
        elif isinstance(subprocess_exception, str) and subprocess_exception.startswith("PermissionError"):
            blockers.append("workcell_builder_executable_permission_denied_at_run_time")
    if rc not in (0, None): blockers.append("child_process_returned_nonzero")

    if args.output.exists():
        try:
            payload = json.loads(args.output.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("app smoke JSON root is not an object")
            payload = _enforce_physical_render_evidence(payload)
            app_status = str(payload.get("status", "UNKNOWN") or "UNKNOWN")
            _add_ros_humble_context(payload)
        except Exception as exc:  # noqa: BLE001
            blockers.append(f"app_smoke_json_unreadable:{exc}")
            fail_payload = {
                **diag,
                "status": "FAIL",
                "wrapper_status": "FAIL",
                "blockers": blockers,
                "warnings": warnings,
            }
            _add_smoke_report_supplemental_evidence(
                fail_payload,
                screenshot_path=diag.get("screenshot_path"),
                screenshot_available=diag.get("screenshot_available"),
            )
            _write_json(args.output, fail_payload)
            print(f"status=FAIL smoke_status=APP_JSON_UNREADABLE error={exc}")
            print("child_command=" + diag["child_command"])
            print("stdout_log_path=" + str(stdout_log))
            print("stderr_log_path=" + str(stderr_log))
            return 1

        app_status = str(payload.get("status", "UNKNOWN") or "UNKNOWN")
        wrapper_status = "BLOCKED" if timed_out else ("FAIL" if rc not in (0, None) else "PASS")
        wrapper_evidence = {
            "wrapper_status": wrapper_status,
            "resolved_executable": diag["resolved_executable"],
            "searched_paths": diag["searched_paths"],
            "repo_root": diag["repo_root"],
            "workspace_root": diag["workspace_root"],
            "scene_path": diag["scene_path"],
            "child_command": diag["child_command"],
            "child_returncode": rc,
            "timed_out": timed_out,
            "stdout_log_path": str(stdout_log),
            "stderr_log_path": str(stderr_log),
            "stdout_tail": stdout_tail,
            "stderr_tail": stderr_tail,
            "screenshot_path": diag["screenshot_path"],
            "screenshot_available": diag["screenshot_available"],
        }
        payload.update(wrapper_evidence)
        _add_smoke_report_supplemental_evidence(
            payload,
            screenshot_path=diag.get("screenshot_path"),
            screenshot_available=diag.get("screenshot_available"),
        )
        if any(key in payload for key in ("final_draw_visual_items", "final_draw_diagnostics", "viewport_visible_items")):
            _apply_ur5_final_viewport_payload_contract(payload)
        _apply_generated_urdf_visual_first_drop_smoke_stage(payload)
        _apply_ur5_transform_parity(payload, repo_root=repo_root, args=args)
        _apply_ur5_final_draw_bbox_regression(payload)
        if args.scene_path:
            expected_scene_path = str(args.scene_path.resolve())
            counters = payload.get("counters") if isinstance(payload.get("counters"), dict) else {}
            actual_scene_path = str(counters.get("inspector_scene_path") or counters.get("selected_scene_path") or "").strip()
            if Path(actual_scene_path).as_posix() != Path(expected_scene_path).as_posix():
                scene_path_blockers = list(payload.get("blockers") or [])
                scene_path_blockers.append("explicit_scene_path_not_loaded")
                payload["status"] = "FAIL"
                payload["expected_scene_path"] = expected_scene_path
                payload["actual_scene_path"] = actual_scene_path
                payload["blockers"] = scene_path_blockers
                _write_json(args.output, payload)
                print(f"status=FAIL smoke_status=EXPLICIT_SCENE_PATH_MISMATCH wrapper_status={wrapper_status} expected_scene_path={expected_scene_path} actual_scene_path={actual_scene_path}")
                return 1
        wrapper_status = "PASS" if rc == 0 and str(payload.get("status", app_status)).upper() in {"PASS", "OK"} else "FAIL"
        if str(payload.get("status", app_status)).upper() == "BLOCKED" or timed_out:
            wrapper_status = "BLOCKED"
        payload["wrapper_status"] = wrapper_status
        _write_json(args.output, payload)
        print(f"status={wrapper_status} smoke_status=APP_JSON_PRESENT wrapper_status={wrapper_status} app_status={app_status} returncode={rc} timed_out={timed_out}")
        print("child_command=" + diag["child_command"])
        print("stdout_log_path=" + str(stdout_log))
        print("stderr_log_path=" + str(stderr_log))
        return 0 if wrapper_status == "PASS" else 1

    blockers.append("app_smoke_json_missing")
    if "--scene3d-smoke" in diag["child_command"] and "--smoke-output" in diag["child_command"]:
        blockers.append("app_started_but_no_smoke_report")
    else:
        blockers.append("app_ignored_scene3d_smoke_args")

    fail_payload = {
        **diag,
        "status": "BLOCKED" if subprocess_exception else "FAIL",
        "blockers": blockers,
        "warnings": warnings,
    }
    if not ros_env["ros_humble_available"]:
        _record_ros_humble_missing(fail_payload, as_blocker=bool(args.executable))
    if subprocess_exception:
        fail_payload["subprocess_exception"] = subprocess_exception
        fail_payload["guidance"] = _executable_guidance()
    _add_smoke_report_supplemental_evidence(
        fail_payload,
        screenshot_path=diag.get("screenshot_path"),
        screenshot_available=diag.get("screenshot_available"),
    )
    _write_json(args.output, fail_payload)
    print("status=FAIL smoke_status=WRAPPER_FAIL_JSON")
    print("child_command=" + diag["child_command"])
    print(f"returncode={rc} timed_out={timed_out}")
    print("stdout_log_path=" + str(stdout_log))
    print("stderr_log_path=" + str(stderr_log))
    print("blocker_list=" + " | ".join(blockers))
    return 1

if __name__ == "__main__":
    raise SystemExit(main())
