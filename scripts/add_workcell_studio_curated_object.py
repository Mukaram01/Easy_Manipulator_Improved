#!/usr/bin/env python3
"""Plan or persist one curated catalog object in an editable Workcell Studio layout.

Dry-run is the default.  The command writes only
layout/workcell_studio_layout.yaml, and only after the caller explicitly passes
--write.  It never edits generated files, environment.yaml, manifests, robot
configuration, controllers, or motion settings.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
import re
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any

try:
    import yaml  # type: ignore
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore

REPO_ROOT = Path(__file__).resolve().parents[1]
PROFILE = REPO_ROOT / "workcell_builder/workcell_builder/config/asset_profiles/environment_assets.json"
LAYOUT_REL = Path("layout/workcell_studio_layout.yaml")
ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")
MESH_SUFFIXES = {".stl", ".dae", ".obj"}
EDGE_MARGIN = 0.01
CLEARANCE = 0.01
HELPER_TOKENS = ("zone", "keepout", "home_pose", "camera", "reach", "overlay", "helper", "warning")
SUPPORT_TOKENS = ("support_surface", "table_surface", "tabletop", "workbench")
COLORS = {
    "bin": [0.95, 0.68, 0.20, 0.88],
    "tray": [0.22, 0.58, 0.82, 0.88],
    "tote": [0.28, 0.65, 0.48, 0.88],
    "fixture": [0.48, 0.52, 0.58, 1.0],
    "object": [0.91, 0.36, 0.24, 1.0],
}


def _fail(message: str, *, json_output: bool) -> int:
    payload = {"status": "error", "error": message}
    if json_output:
        print(json.dumps(payload, sort_keys=True))
    else:
        print(f"ERROR: {message}", file=sys.stderr)
    return 1


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _num3(value: Any, field: str) -> list[float]:
    if not isinstance(value, list) or len(value) < 3:
        raise ValueError(f"{field} must contain three numbers")
    out = [float(value[i]) for i in range(3)]
    if not all(math.isfinite(v) and v > 0.0 for v in out):
        raise ValueError(f"{field} must contain three finite positive numbers")
    return out


def _pose(item: dict[str, Any]) -> list[float] | None:
    pose = item.get("pose")
    xyz = pose.get("xyz") if isinstance(pose, dict) else item.get("pose_xyz")
    if not isinstance(xyz, list) or len(xyz) < 3:
        return None
    try:
        out = [float(xyz[i]) for i in range(3)]
    except (TypeError, ValueError):
        return None
    return out if all(math.isfinite(v) for v in out) else None


def _dimensions(item: dict[str, Any]) -> list[float] | None:
    raw = item.get("dimensions") or item.get("size")
    try:
        return _num3(raw, "dimensions")
    except ValueError:
        return None


def _identity(item: dict[str, Any]) -> str:
    return " ".join(str(item.get(k, "")).lower().replace("-", "_") for k in ("id", "type", "role", "category"))


def _is_support(item: dict[str, Any]) -> bool:
    text = _identity(item)
    return any(token in text for token in SUPPORT_TOKENS)


def _is_physical_obstacle(item: dict[str, Any]) -> bool:
    text = _identity(item)
    return not _is_support(item) and not any(token in text for token in HELPER_TOKENS) and _pose(item) is not None and _dimensions(item) is not None


def _aabb(center: list[float], dims: list[float], clearance: float = 0.0) -> tuple[list[float], list[float]]:
    half = [dims[i] / 2.0 + clearance for i in range(3)]
    return ([center[i] - half[i] for i in range(3)], [center[i] + half[i] for i in range(3)])


def _overlap(a: tuple[list[float], list[float]], b: tuple[list[float], list[float]]) -> bool:
    return all(min(a[1][i], b[1][i]) - max(a[0][i], b[0][i]) > 1e-9 for i in range(3))


def _candidate_offsets(max_x: float, max_y: float) -> list[tuple[float, float]]:
    step = 0.05
    nx = max(0, int(math.floor(max_x / step)))
    ny = max(0, int(math.floor(max_y / step)))
    values = [(ix * step, iy * step) for ix in range(-nx, nx + 1) for iy in range(-ny, ny + 1)]
    return sorted(values, key=lambda p: (p[0] * p[0] + p[1] * p[1], abs(p[1]), abs(p[0]), p[1], p[0]))


def _find_placement(items: list[dict[str, Any]], dims: list[float]) -> tuple[dict[str, Any], list[float]]:
    supports = [item for item in items if _is_support(item) and _pose(item) is not None and _dimensions(item) is not None]
    supports.sort(key=lambda item: str(item.get("id", "")))
    obstacles = [item for item in items if _is_physical_obstacle(item)]
    for support in supports:
        support_pose = _pose(support)
        support_dims = _dimensions(support)
        assert support_pose is not None and support_dims is not None
        max_x = support_dims[0] / 2.0 - dims[0] / 2.0 - EDGE_MARGIN
        max_y = support_dims[1] / 2.0 - dims[1] / 2.0 - EDGE_MARGIN
        if max_x < 0.0 or max_y < 0.0:
            continue
        z = support_pose[2] + support_dims[2] / 2.0 + dims[2] / 2.0
        for dx, dy in _candidate_offsets(max_x, max_y):
            candidate = [support_pose[0] + dx, support_pose[1] + dy, z]
            candidate_box = _aabb(candidate, dims, CLEARANCE)
            blocked = False
            for obstacle in obstacles:
                obstacle_pose = _pose(obstacle)
                obstacle_dims = _dimensions(obstacle)
                assert obstacle_pose is not None and obstacle_dims is not None
                if _overlap(candidate_box, _aabb(obstacle_pose, obstacle_dims)):
                    blocked = True
                    break
            if not blocked:
                return support, candidate
    raise ValueError("no collision-free support-surface slot is available for this object")


def _catalog() -> dict[str, dict[str, Any]]:
    data = json.loads(PROFILE.read_text(encoding="utf-8"))
    return {str(item.get("asset_id")): item for item in data if isinstance(item, dict) and item.get("asset_id")}


def _validate_asset(asset: dict[str, Any]) -> tuple[list[float], str, str | None]:
    if asset.get("curated_add_enabled") is not True or asset.get("placement_policy") != "support_surface":
        raise ValueError("asset is not approved for the curated add-object workflow")
    dims = _num3(asset.get("default_dimensions_m"), "default_dimensions_m")
    geometry = str(asset.get("geometry_type") or "mesh")
    primitive = None
    if geometry == "urdf_primitive":
        primitive = str(asset.get("primitive_geometry_type") or "")
        if primitive not in {"box", "cylinder", "sphere", "capsule"}:
            raise ValueError("curated primitive asset has no supported primitive_geometry_type")
    else:
        mesh_path = Path(str(asset.get("mesh_path") or ""))
        if mesh_path.is_absolute() or ".." in mesh_path.parts or mesh_path.suffix.lower() not in MESH_SUFFIXES:
            raise ValueError("curated mesh path is unsafe or unsupported")
        if not (REPO_ROOT / mesh_path).is_file():
            raise ValueError(f"curated mesh file is missing: {mesh_path}")
    return dims, geometry, primitive


def _instance_id(asset_id: str, items: list[dict[str, Any]], requested: str | None) -> str:
    used = {str(item.get("id")) for item in items}
    if requested:
        if not ID_RE.fullmatch(requested):
            raise ValueError("instance ID must match [A-Za-z0-9][A-Za-z0-9_-]*")
        if requested in used:
            raise ValueError(f"instance ID already exists: {requested}")
        return requested
    for index in range(1, 1000):
        candidate = f"{asset_id}_{index:02d}"
        if candidate not in used:
            return candidate
    raise ValueError(f"could not allocate a unique instance ID for {asset_id}")


def _layout_item(asset: dict[str, Any], instance_id: str, support_id: str, pose_xyz: list[float], dims: list[float], geometry: str, primitive: str | None) -> dict[str, Any]:
    asset_type = str(asset.get("asset_type") or "object")
    item: dict[str, Any] = {
        "id": instance_id,
        "type": asset_type,
        "role": "pick_object" if asset_type == "object" else asset_type,
        "category": str(asset.get("category") or "Curated Objects"),
        "display_name": f"{asset.get('label', asset.get('asset_id'))} {instance_id.rsplit('_', 1)[-1]}",
        "editable": True,
        "locked": False,
        "source_layer": "editable_layout",
        "source": str(LAYOUT_REL).replace(os.sep, "/"),
        "pose": {"xyz": pose_xyz, "rpy": [0.0, 0.0, 0.0]},
        "dimensions": dims,
        "support_surface_ref": support_id,
        "catalog_asset_id": str(asset["asset_id"]),
        "placement_source": "curated_asset_catalog",
        "material": {"color": COLORS.get(asset_type, COLORS["object"])},
    }
    if geometry == "urdf_primitive":
        item["geometry_type"] = primitive
        item["primitive_geometry_type"] = primitive
    else:
        item["geometry_type"] = "mesh"
        item["mesh_path"] = str(asset["mesh_path"])
    return item


def _atomic_yaml(path: Path, data: dict[str, Any]) -> None:
    assert yaml is not None
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, sort_keys=False, default_flow_style=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp_name, path)
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Safely add one approved curated object to an editable Workcell Studio layout.")
    parser.add_argument("--scene", required=True, type=Path)
    parser.add_argument("--asset-id", required=True)
    parser.add_argument("--instance-id")
    parser.add_argument("--expected-layout-sha256")
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--backup", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    try:
        if yaml is None:
            raise ValueError("PyYAML is required")
        scene = args.scene.resolve()
        layout_path = scene / LAYOUT_REL
        if not scene.is_dir() or not layout_path.is_file():
            raise ValueError(f"editable layout is missing: {layout_path}")
        layout_sha = _sha256(layout_path)
        if args.expected_layout_sha256 and args.expected_layout_sha256 != layout_sha:
            raise ValueError("layout changed after preview; reload and plan the add again")
        layout = yaml.safe_load(layout_path.read_text(encoding="utf-8")) or {}
        if layout.get("schema_version") != "workcell_studio_layout/v1" or not isinstance(layout.get("items"), list):
            raise ValueError("layout must use workcell_studio_layout/v1 with an items list")
        catalog = _catalog()
        asset = catalog.get(args.asset_id)
        if asset is None:
            raise ValueError(f"unknown curated asset: {args.asset_id}")
        dims, geometry, primitive = _validate_asset(asset)
        items = [item for item in layout["items"] if isinstance(item, dict)]
        instance_id = _instance_id(args.asset_id, items, args.instance_id)
        support, pose_xyz = _find_placement(items, dims)
        support_id = str(support.get("id"))
        new_item = _layout_item(asset, instance_id, support_id, pose_xyz, dims, geometry, primitive)
        result = {
            "status": "written" if args.write else "planned",
            "scene_id": scene.name,
            "asset_id": args.asset_id,
            "instance_id": instance_id,
            "support_surface_id": support_id,
            "pose_xyz": pose_xyz,
            "dimensions_m": dims,
            "layout_sha256": layout_sha,
            "layout_path": str(layout_path),
            "license": str(asset.get("license") or "unknown"),
            "source_note": str(asset.get("source_note") or ""),
        }
        if args.write:
            if args.backup:
                stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
                backup = layout_path.with_name(f"{layout_path.name}.{stamp}.bak")
                shutil.copy2(layout_path, backup)
                result["backup_path"] = str(backup)
            layout["items"].append(new_item)
            _atomic_yaml(layout_path, layout)
            result["layout_sha256_after"] = _sha256(layout_path)
        print(json.dumps(result, sort_keys=True) if args.json else json.dumps(result, indent=2, sort_keys=True))
        return 0
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return _fail(str(exc), json_output=args.json)


if __name__ == "__main__":
    raise SystemExit(main())
