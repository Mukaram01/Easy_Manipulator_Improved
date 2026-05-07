#!/usr/bin/env python3
from __future__ import annotations

from typing import Any

ASSET_TYPES: dict[str, dict[str, Any]] = {
    "table": {"supports_dimensions": True, "supports_mesh": True, "preview_status": "supported"},
    "bin": {"supports_dimensions": True, "supports_mesh": True, "preview_status": "supported"},
    "conveyor_placeholder": {"supports_dimensions": True, "supports_mesh": False, "preview_status": "preview_only", "placeholder_only": True},
    "camera_mount": {"supports_dimensions": False, "supports_mesh": True, "preview_status": "supported"},
    "robot_base": {"supports_dimensions": True, "supports_mesh": False, "preview_status": "supported"},
    "object_spawn_area": {"supports_dimensions": True, "supports_mesh": False, "preview_status": "supported"},
    "fixture_placeholder": {"supports_dimensions": True, "supports_mesh": True, "preview_status": "preview_only"},
}


def default_asset_catalog() -> list[dict[str, Any]]:
    return [
        {"type": k, **v} for k, v in ASSET_TYPES.items()
    ]


def build_environment_asset(
    *,
    asset_id: str,
    asset_type: str,
    xyz: list[float],
    rpy: list[float],
    dimensions: list[float] | None = None,
    mesh_path: str | None = None,
    perception_enabled: bool = False,
    metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if asset_type not in ASSET_TYPES:
        raise ValueError(f"Unsupported environment asset type: {asset_type}")
    spec = ASSET_TYPES[asset_type]
    asset: dict[str, Any] = {
        "id": asset_id,
        "name": asset_id,
        "type": asset_type,
        "pose": {"frame": "world", "xyz": xyz, "rpy": rpy},
        "preview_status": spec["preview_status"],
        "runtime_supported": spec["preview_status"] == "supported",
    }
    if dimensions is not None and spec.get("supports_dimensions"):
        asset["dimensions"] = dimensions
    if mesh_path and spec.get("supports_mesh"):
        asset["mesh_path"] = mesh_path
    if asset_type == "conveyor_placeholder":
        asset["notes"] = ["Conveyor is placeholder metadata only and has no runtime physics backend."]
    if asset_type == "camera_mount":
        asset["camera"] = {
            "model": "RealSense D435i",
            "device_id": "intel_realsense_d435i",
            "perception_enabled": bool(perception_enabled),
        }
    if metadata:
        asset["metadata"] = metadata
    return asset


def build_environment_layout(layout_id: str, assets: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "schema_version": "environment_layout/v1",
        "layout_id": layout_id,
        "metadata": {"generated_by": "workcell_builder", "environment_palette_enabled": True},
        "assets": assets,
        "zones": [],
    }
