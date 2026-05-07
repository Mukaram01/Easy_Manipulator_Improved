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


ASSET_ROLES = {
    "support_surface",
    "obstacle",
    "pick_area",
    "place_target",
    "camera_mount",
    "safety_zone",
    "visual_only",
}


class PlacementEditorError(ValueError):
    """Raised when placement editor input is invalid."""


def _as_vec3(values: list[float], *, field_name: str) -> list[float]:
    if len(values) != 3:
        raise PlacementEditorError(f"{field_name} must contain exactly 3 numeric values")
    return [float(v) for v in values]


def create_asset_placement_editor_model(
    *,
    asset: dict[str, Any],
    role: str = "obstacle",
    visible: bool = True,
    collision_enabled: bool = True,
    lock_asset: bool = False,
    scale: float = 1.0,
) -> dict[str, Any]:
    if scale <= 0.0:
        raise PlacementEditorError("scale must be greater than zero")
    if role not in ASSET_ROLES:
        raise PlacementEditorError(f"unsupported asset role: {role}")
    pose = asset.get("pose") if isinstance(asset.get("pose"), dict) else {"frame": "world", "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
    xyz = _as_vec3(list(pose.get("xyz", [0.0, 0.0, 0.0])), field_name="xyz")
    rpy = _as_vec3(list(pose.get("rpy", [0.0, 0.0, 0.0])), field_name="rpy")
    model = {
        "asset_id": asset.get("id", ""),
        "pose": {"frame": str(pose.get("frame", "world")), "xyz": xyz, "rpy": rpy},
        "scale": float(scale),
        "visible": bool(visible),
        "collision_enabled": bool(collision_enabled),
        "lock_asset": bool(lock_asset),
        "role": role,
    }
    dimensions = asset.get("dimensions")
    if isinstance(dimensions, list) and len(dimensions) == 3:
        model["dimensions"] = _as_vec3([float(x) for x in dimensions], field_name="dimensions")
    return model


def apply_quick_placement(asset: dict[str, Any], action: str, assets: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    pose = asset.setdefault("pose", {"frame": "world", "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]})
    xyz = pose.setdefault("xyz", [0.0, 0.0, 0.0])
    table = next((a for a in (assets or []) if a.get("type") == "table"), None)
    robot = next((a for a in (assets or []) if a.get("type") == "robot_base"), None)

    if action == "on_table" and table and table.get("dimensions"):
        table_top = float(table["pose"]["xyz"][2]) + float(table["dimensions"][2])
        xyz[2] = table_top
    elif action == "left_of_robot" and robot:
        xyz[1] = float(robot["pose"]["xyz"][1]) + 0.5
    elif action == "right_of_robot" and robot:
        xyz[1] = float(robot["pose"]["xyz"][1]) - 0.5
    elif action == "in_front_of_robot" and robot:
        xyz[0] = float(robot["pose"]["xyz"][0]) + 0.5
    elif action == "behind_robot" and robot:
        xyz[0] = float(robot["pose"]["xyz"][0]) - 0.5
    elif action == "align_to_table" and table:
        xyz[0] = float(table["pose"]["xyz"][0])
        xyz[1] = float(table["pose"]["xyz"][1])
    elif action == "duplicate_asset":
        duplicated = {**asset, "id": f"{asset.get('id', 'asset')}_copy", "name": f"{asset.get('name', asset.get('id', 'asset'))} Copy", "pose": {**pose, "xyz": [xyz[0] + 0.2, xyz[1], xyz[2]], "rpy": list(pose.get("rpy", [0.0, 0.0, 0.0]))}}
        return duplicated
    return asset


def _bbox(asset: dict[str, Any]) -> tuple[list[float], list[float]] | None:
    dims = asset.get("dimensions")
    pose = asset.get("pose") if isinstance(asset.get("pose"), dict) else None
    if not (isinstance(dims, list) and len(dims) == 3 and pose and isinstance(pose.get("xyz"), list) and len(pose["xyz"]) == 3):
        return None
    half = [float(d) / 2.0 for d in dims]
    xyz = [float(v) for v in pose["xyz"]]
    return ([xyz[i] - half[i] for i in range(3)], [xyz[i] + half[i] for i in range(3)])


def _overlap(a: tuple[list[float], list[float]], b: tuple[list[float], list[float]]) -> bool:
    return all(a[0][i] <= b[1][i] and b[0][i] <= a[1][i] for i in range(3))


def validate_asset_placements(layout: dict[str, Any]) -> list[str]:
    warnings: list[str] = []
    assets = layout.get("assets") if isinstance(layout.get("assets"), list) else []
    if not any(a.get("type") == "robot_base" for a in assets):
        warnings.append("robot base missing")
    if not any(a.get("role") == "pick_area" for a in assets):
        warnings.append("pick area missing")
    if not any(a.get("role") == "place_target" for a in assets):
        warnings.append("place target missing")

    robot = next((a for a in assets if a.get("type") == "robot_base"), None)
    table = next((a for a in assets if a.get("type") == "table"), None)
    table_bbox = _bbox(table) if table else None
    robot_bbox = _bbox(robot) if robot else None

    for asset in assets:
        pose = asset.get("pose") if isinstance(asset.get("pose"), dict) else {}
        xyz = pose.get("xyz", [0, 0, 0])
        if isinstance(xyz, list) and len(xyz) == 3 and float(xyz[2]) < 0.0:
            warnings.append(f"{asset.get('id', 'asset')} below floor")
        if asset.get("type") == "camera_mount" and (not isinstance(pose.get("xyz"), list) or not isinstance(pose.get("rpy"), list)):
            warnings.append(f"camera missing pose: {asset.get('id', 'camera')}")
        if asset.get("type") == "bin":
            bb = _bbox(asset)
            if bb and table_bbox and _overlap(bb, table_bbox):
                warnings.append(f"bin overlaps table: {asset.get('id', 'bin')}")
            if bb and robot_bbox and _overlap(bb, robot_bbox):
                warnings.append(f"bin overlaps robot: {asset.get('id', 'bin')}")
    return warnings


def export_environment_layout_with_placement(layout_id: str, assets: list[dict[str, Any]]) -> dict[str, Any]:
    layout = build_environment_layout(layout_id, assets)
    layout.setdefault("metadata", {})["placement_editor"] = {
        "enabled": True,
        "schema": "workcell_asset_placement/v1",
        "asset_poses": {
            a.get("id", "asset"): {
                "xyz": list((a.get("pose") or {}).get("xyz", [0.0, 0.0, 0.0])),
                "rpy": list((a.get("pose") or {}).get("rpy", [0.0, 0.0, 0.0])),
                "scale": float(a.get("scale", 1.0)),
                "visible": bool(a.get("visible", True)),
                "collision_enabled": bool(a.get("collision_enabled", True)),
                "lock_asset": bool(a.get("lock_asset", False)),
                "role": a.get("role", "obstacle"),
            }
            for a in assets
        },
    }
    layout["metadata"]["placement_warnings"] = validate_asset_placements(layout)
    return layout
