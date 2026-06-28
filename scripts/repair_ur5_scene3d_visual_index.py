#!/usr/bin/env python3
"""Repair canonical UR5 Scene3D visual-index rows after extraction.

The extractor remains the source of truth for generated ROS/RViz packages.  The
builder viewport also needs a safe preview layer: when required UR5 rows are
missing, collapsed, unrenderable, or geometrically implausible, this script
replaces the UR5 arm preview rows with a stable locked primitive preview.  For
UR5 + Robotiq 2F scenes, it also adds a small locked gripper proxy so the builder
canvas still shows an end-effector instead of a bare wrist when xacro expansion
or mesh handoff dropped the Robotiq visuals.  Non-robot rows such as table,
camera, bins, and zones are preserved.
"""
from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from scripts.scene3d_ur5_preview_rows import (
    REQUIRED_UR5_LINKS,
    REQUIRED_UR5_VISUALS,
    STABLE_UR5_2F_LINKS,
    STABLE_UR5_2F_PREVIEW_VISUALS,
    make_ur5_preview_item,
)

RENDER_REJECT_STATUSES = {"skip", "skipped", "hidden", "reject", "rejected", "failed", "error", "missing", "suppressed"}
UR5_VISUAL_TOKEN_HINTS = (
    "ur_description/meshes/ur5/visual/",
    "package://ur_description/meshes/ur5/visual/",
    "assets/robots/universal_robot/ur_description/meshes/ur5/visual/",
)
MAX_ADJACENT_LINK_DISTANCE_M = 1.25
COLLAPSED_LINK_EPSILON_M = 1e-6
ROBOTIQ_85_RENDER_LINKS = {
    "tool0",
    "gripper_base_link",
    "gripper_finger1_knuckle_link",
    "gripper_finger2_knuckle_link",
    "gripper_finger1_inner_knuckle_link",
    "gripper_finger2_inner_knuckle_link",
    "gripper_finger1_finger_link",
    "gripper_finger2_finger_link",
    "gripper_finger1_finger_tip_link",
    "gripper_finger2_finger_tip_link",
}


def _as_items(payload: dict[str, Any]) -> list[dict[str, Any]]:
    for key in ("visual_items", "items"):
        raw = payload.get(key)
        if isinstance(raw, list):
            return [item for item in raw if isinstance(item, dict)]
    return []


def _token_values(item: dict[str, Any]) -> str:
    keys = (
        "link", "link_name", "canonical_link_name", "object_name", "visual_index_link",
        "visual_index_link_name", "id", "item_id", "display_name", "metadata_tags",
        "package_uri", "mesh_uri", "mesh_source", "mesh_path", "source_path",
        "resolved_source_path", "baked_world_visual_transform_source", "transform_status",
        "link_transform_status",
    )
    values = [str(item.get(key) or "") for key in keys]
    metadata = item.get("metadata")
    if isinstance(metadata, dict):
        values.extend(str(metadata.get(key) or "") for key in ("link", "link_name", "canonical_link_name"))
    return "|".join(values).lower()


def _item_mentions_link(item: dict[str, Any], link: str) -> bool:
    return link.lower() in _token_values(item)


def _item_has_ur5_mesh_evidence(item: dict[str, Any]) -> bool:
    tokens = _token_values(item)
    if any(hint in tokens for hint in UR5_VISUAL_TOKEN_HINTS):
        return True
    mesh_name = str(item.get("mesh_file_name") or item.get("mesh") or "").lower()
    return mesh_name in {str(spec["mesh"]).lower() for spec in REQUIRED_UR5_VISUALS}


def _truthy_or_missing(value: Any) -> bool:
    return value is not False


def _item_is_renderable_required_link(item: dict[str, Any], link: str) -> bool:
    if not _item_mentions_link(item, link):
        return False
    if not _truthy_or_missing(item.get("render_expected")):
        return False
    if not _truthy_or_missing(item.get("resolved")):
        return False
    if item.get("visible") is False or item.get("rendered") is False:
        return False
    status = str(item.get("final_draw_status") or item.get("draw_status") or "").strip().lower()
    if status in RENDER_REJECT_STATUSES:
        return False
    render_skip_reason = str(item.get("render_skip_reason") or item.get("warning") or "").strip().lower()
    if any(token in render_skip_reason for token in ("missing_parent", "file_not_found", "package_not_found", "mesh_parse_failed")):
        return False
    if _item_has_ur5_mesh_evidence(item):
        return True
    geometry_type = str(item.get("geometry_type") or item.get("primitive_geometry_type") or "").strip().lower()
    return geometry_type in {"box", "mesh"} and bool(
        str(item.get("mesh_uri") or item.get("mesh_path") or item.get("package_uri") or item.get("size") or "").strip()
    )


def _present_required_links(items: list[dict[str, Any]]) -> set[str]:
    present: set[str] = set()
    for item in items:
        for link in REQUIRED_UR5_LINKS:
            if _item_is_renderable_required_link(item, link):
                present.add(link)
    return present


def _present_links(items: list[dict[str, Any]], links: tuple[str, ...]) -> set[str]:
    present: set[str] = set()
    for item in items:
        for link in links:
            if _item_mentions_link(item, link) and _truthy_or_missing(item.get("render_expected")):
                present.add(link)
    return present


def _payload_is_ur5_candidate(payload: dict[str, Any], items: list[dict[str, Any]], path: Path) -> bool:
    scene_name = str(payload.get("scene_name") or payload.get("scene") or path.parents[1].name)
    if scene_name.startswith("ur5_"):
        return True
    blob = "|".join(_token_values(item) for item in items)
    if any(hint in blob for hint in UR5_VISUAL_TOKEN_HINTS):
        return True
    return sum(1 for link in REQUIRED_UR5_LINKS if link.lower() in blob) >= 2


def _payload_uses_2f_gripper(payload: dict[str, Any], items: list[dict[str, Any]], path: Path) -> bool:
    scene_name = str(payload.get("scene_name") or payload.get("scene") or path.parents[1].name).lower()
    if "2f" in scene_name or "robotiq" in scene_name:
        return True
    blob = "|".join(_token_values(item) for item in items)
    return any(token in blob for token in ("robotiq_85", "robotiq85", "2f", "two_finger"))


def _finite_vec(value: Any, expected_len: int = 3) -> bool:
    if not isinstance(value, list) or len(value) < expected_len:
        return False
    try:
        return all(math.isfinite(float(value[i])) for i in range(expected_len))
    except Exception:
        return False


def _pose_xyz(item: dict[str, Any], field: str) -> list[float] | None:
    pose = item.get(field)
    if isinstance(pose, dict) and _finite_vec(pose.get("xyz")):
        return [float(v) for v in pose["xyz"][:3]]
    return None


def _distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def _ur5_link_positions(items: list[dict[str, Any]]) -> dict[str, list[float]]:
    positions: dict[str, list[float]] = {}
    for item in items:
        for link in REQUIRED_UR5_LINKS:
            if link in positions or not _item_mentions_link(item, link):
                continue
            for field in ("baked_world_visual_pose", "expected_visual_pose", "link_world_pose", "world_pose", "pose"):
                xyz = _pose_xyz(item, field)
                if xyz is not None:
                    positions[link] = xyz
                    break
    return positions


def _existing_ur5_rows_need_repair(items: list[dict[str, Any]], present: set[str]) -> tuple[bool, list[str]]:
    reasons: list[str] = []
    # RViz-level Scene3D must keep resolved, renderable mesh rows.  Older checked-in
    # indexes can carry rviz-parity seed tokens even when their mesh paths and poses
    # are valid; those tokens alone are not a reason to down-convert the robot to
    # primitive boxes.  Repair only when the rows are missing, collapsed, or
    # geometrically implausible.
    positions = _ur5_link_positions(items)
    if present and len(positions) < min(4, len(present)):
        reasons.append("insufficient_ur5_pose_data")
    if len(positions) >= 4:
        ordered_positions = [positions[link] for link in REQUIRED_UR5_LINKS if link in positions]
        if ordered_positions and all(_distance(ordered_positions[0], p) <= COLLAPSED_LINK_EPSILON_M for p in ordered_positions[1:]):
            reasons.append("collapsed_ur5_link_positions")
        for prev, cur in zip(REQUIRED_UR5_LINKS, REQUIRED_UR5_LINKS[1:]):
            if prev in positions and cur in positions:
                adjacent = _distance(positions[prev], positions[cur])
                if adjacent > MAX_ADJACENT_LINK_DISTANCE_M:
                    reasons.append(f"implausible_adjacent_distance:{prev}->{cur}:{adjacent:.3f}m")
    return bool(reasons), reasons


def _apply_repaired_rows(
    payload: dict[str, Any],
    items: list[dict[str, Any]],
    repaired_specs: list[dict[str, Any]],
    replaced_links: set[str],
) -> list[dict[str, Any]]:
    repaired = [make_ur5_preview_item(spec, index) for index, spec in enumerate(repaired_specs)]
    kept = [
        item for item in items
        if isinstance(item, dict)
        and not any(link.lower() in _token_values(item) for link in replaced_links)
    ]
    repaired_items = repaired + kept
    payload["visual_items"] = repaired_items
    if isinstance(payload.get("items"), list):
        payload["items"] = repaired_items
    return repaired_items


def _refresh_payload_counters(payload: dict[str, Any]) -> None:
    visual_items = payload["visual_items"]
    payload["visual_count"] = len(visual_items)
    payload["candidate_mesh_count"] = len(visual_items)
    payload["emitted_visual_count"] = len(visual_items)
    payload["renderable_item_count"] = len([i for i in visual_items if i.get("render_expected", True)])
    payload["renderable_mesh_count"] = len([i for i in visual_items if i.get("geometry_type") == "mesh" and i.get("render_expected", True)])
    payload["resolved"] = len([i for i in visual_items if i.get("resolved")])
    payload["unresolved"] = len([i for i in visual_items if not i.get("resolved")])


def _drop_repair_blockers(payload: dict[str, Any]) -> None:
    blockers = payload.get("blockers") if isinstance(payload.get("blockers"), list) else []
    stale_tokens = (
        "missing_required_visible_ur5_links",
        "ur5_final_viewport_links_missing",
        "rendered_ur5_link_count_below_7",
        "stale_retained_visual_rows_missing_warning",
        "implausible_adjacent_distance",
        "collapsed_ur5_link_positions",
    )
    payload["blockers"] = [b for b in blockers if not any(token in str(b) for token in stale_tokens)]


def _append_repair_warnings(payload: dict[str, Any], extra_specs: list[dict[str, Any]]) -> None:
    warnings = payload.get("warnings") if isinstance(payload.get("warnings"), list) else []
    note = "ur5_runtime_visual_index_repair_applied: stable primitive builder preview; RViz launch remains authoritative"
    if note not in warnings:
        warnings.append(note)
    if extra_specs:
        gripper_note = "ur5_2f_stable_end_effector_preview_added: locked Robotiq 2F proxy for builder canvas"
        if gripper_note not in warnings:
            warnings.append(gripper_note)
    payload["warnings"] = warnings


def repair_index(path: Path) -> tuple[bool, list[str]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    items = _as_items(payload)
    if not _payload_is_ur5_candidate(payload, items, path):
        return False, []

    present = _present_required_links(items)
    missing = [spec for spec in REQUIRED_UR5_VISUALS if spec["link"] not in present]
    stale_or_implausible, repair_reasons = _existing_ur5_rows_need_repair(items, present)
    repair_arm_rows = bool(missing) or stale_or_implausible
    present_2f_links = _present_links(items, STABLE_UR5_2F_LINKS)
    uses_2f_gripper = _payload_uses_2f_gripper(payload, items, path)
    missing_2f_specs = [spec for spec in STABLE_UR5_2F_PREVIEW_VISUALS if spec["link"] not in present_2f_links]

    # If the core tool anchors are missing, keeping existing finger/knuckle rows creates
    # exactly the exploded gripper seen in Scene3D. Replace the whole Robotiq draw chain
    # with one compact connected proxy instead of preserving loose finger meshes.
    missing_2f_anchors = [link for link in ("tool0", "gripper_base_link") if link not in present_2f_links]
    replace_partial_2f_chain = uses_2f_gripper and bool(missing_2f_anchors)
    should_include_2f_proxy = uses_2f_gripper and (replace_partial_2f_chain or bool(missing_2f_specs))
    extra_specs = list(STABLE_UR5_2F_PREVIEW_VISUALS) if replace_partial_2f_chain else (
        missing_2f_specs if should_include_2f_proxy else []
    )
    if not repair_arm_rows and not extra_specs:
        return False, []

    repaired_specs = (list(REQUIRED_UR5_VISUALS) if repair_arm_rows else []) + list(extra_specs)
    replaced_links: set[str] = set(REQUIRED_UR5_LINKS) if repair_arm_rows else set()
    if extra_specs:
        if replace_partial_2f_chain:
            replaced_links.update(ROBOTIQ_85_RENDER_LINKS)
        else:
            replaced_links.update(spec["link"] for spec in extra_specs)
    _apply_repaired_rows(payload, items, repaired_specs, replaced_links)
    _refresh_payload_counters(payload)
    payload["safe_for_preview"] = True
    payload["stale_index"] = False
    payload["stale_reasons"] = []
    payload["ur5_runtime_repair_applied"] = True
    payload["ur5_runtime_repair_mode"] = "stable_primitive_builder_preview"
    payload["ur5_runtime_repair_added_links"] = [spec["link"] for spec in missing]
    payload["ur5_runtime_repair_added_end_effector_links"] = [spec["link"] for spec in extra_specs]
    if repair_reasons:
        payload["ur5_runtime_repair_reasons"] = repair_reasons
    elif repair_arm_rows:
        payload["ur5_runtime_repair_reasons"] = ["missing_required_ur5_rows"]
    else:
        payload["ur5_runtime_repair_reasons"] = ["missing_ur5_2f_end_effector_preview_rows"]
    payload["ur5_required_links"] = list(REQUIRED_UR5_LINKS)
    if extra_specs:
        payload["ur5_2f_stable_preview_links"] = list(STABLE_UR5_2F_LINKS)
    payload["repair_generated_at"] = datetime.now(timezone.utc).isoformat()
    _drop_repair_blockers(payload)
    _append_repair_warnings(payload, list(extra_specs))
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return True, [spec["link"] for spec in missing]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("index_path", type=Path)
    args = parser.parse_args()
    changed, missing = repair_index(args.index_path.resolve())
    if changed:
        detail = ", ".join(missing) if missing else "stale or implausible UR5 rows replaced"
        print(f"[repair_ur5_scene3d_visual_index] repaired {args.index_path}: {detail}")
    else:
        print(f"[repair_ur5_scene3d_visual_index] no repair needed for {args.index_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
