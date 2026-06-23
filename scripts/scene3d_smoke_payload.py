#!/usr/bin/env python3
"""Payload-only helpers for the Scene3D GUI smoke wrapper.

This module is the first safe extraction seam for
``run_workcell_builder_scene3d_gui_smoke.py``.  Keep it free of subprocess,
Qt, ROS launch, and filesystem side effects so the huge smoke runner can be
split behind unit-tested payload logic before any larger wrapper move.
"""
from __future__ import annotations

from typing import Any

NO_PHYSICAL_RENDER_BLOCKER = "scene_rendered_no_physical_items"


def append_unique(values: list[str], value: str) -> None:
    if value not in values:
        values.append(value)


def as_int(value: Any) -> int:
    try:
        if value is None or isinstance(value, bool):
            return 0
        return int(value)
    except (TypeError, ValueError):
        return 0


def counter(payload: dict[str, Any], *keys: str) -> int:
    sources: list[dict[str, Any]] = [payload]
    for nested_key in ("counters", "render_debug_counters"):
        nested = payload.get(nested_key)
        if isinstance(nested, dict):
            sources.append(nested)
    for source in sources:
        for key in keys:
            if key in source:
                return as_int(source.get(key))
    return 0


def physical_rendered_count(payload: dict[str, Any]) -> int:
    mesh_count = counter(payload, "physical_mesh_items_rendered", "mesh_rendered_count", "mesh_backed_count")
    primitive_count = counter(
        payload,
        "primitive_fallback_items_rendered",
        "primitive_rendered_count",
        "urdf_primitive_rendered_count",
        "valid_physical_fallback_count",
        "physical_fallback_count",
        "physical_fallback_rendered_count",
        "collision_primitive_rendered_count",
    )
    physical_total = counter(payload, "physical_rendered_count", "credible_physical_rendered_count")
    return max(physical_total, mesh_count + primitive_count)


def blocker_list(payload: dict[str, Any]) -> list[str]:
    blockers = payload.get("blockers")
    if not isinstance(blockers, list):
        blockers = []
    normalized: list[str] = []
    for blocker in blockers:
        text = str(blocker).strip()
        if text and text not in normalized:
            normalized.append(text)
    return normalized


def enforce_physical_render_evidence(payload: dict[str, Any]) -> dict[str, Any]:
    """Mark smoke payload as failed when the GUI rendered no physical item.

    The old in-script implementation used ``blockers`` before initializing it.
    This helper keeps the behavior but makes the blocker list normalization
    explicit and unit-testable.
    """
    payload["runtime_available"] = True
    if physical_rendered_count(payload) > 0:
        return payload

    blockers = blocker_list(payload)
    append_unique(blockers, NO_PHYSICAL_RENDER_BLOCKER)
    payload["blockers"] = blockers
    payload["status"] = "FAIL"
    payload["physical_rendered_count"] = 0
    return payload
