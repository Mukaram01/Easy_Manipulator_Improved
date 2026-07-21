#!/usr/bin/env python3
"""Safely plan or persist one curated object in a Workcell Studio layout.

Dry-run is the default. Only layout/workcell_studio_layout.yaml may be written,
and only with --write. Generated files, environment metadata, robot settings,
controllers, launch files and motion configuration are outside this command.
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
import tempfile
from pathlib import Path
from typing import Any

try:
    import yaml  # type: ignore
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "workcell_builder/workcell_builder/config/asset_profiles/environment_assets.json"
CURATION = ROOT / "workcell_builder/workcell_builder/config/asset_profiles/curated_add_objects.json"
LAYOUT_REL = Path("layout/workcell_studio_layout.yaml")
ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")
MESH_SUFFIXES = {".stl", ".dae", ".obj"}
EDGE_MARGIN = CLEARANCE = 0.01
HELPERS = ("zone", "keepout", "home_pose", "camera", "reach", "overlay", "helper", "warning")
SUPPORTS = ("support_surface", "table_surface", "tabletop", "workbench")
COLORS = {
    "bin": [0.95, 0.68, 0.20, 0.88], "tray": [0.22, 0.58, 0.82, 0.88],
    "tote": [0.28, 0.65, 0.48, 0.88], "fixture": [0.48, 0.52, 0.58, 1.0],
    "object": [0.91, 0.36, 0.24, 1.0],
}


def _emit_error(message: str, as_json: bool) -> int:
    if as_json:
        print(json.dumps({"status": "error", "error": message}, sort_keys=True))
    else:
        print(f"ERROR: {message}")
    return 1


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _num3(value: Any, name: str, *, positive: bool = False) -> list[float]:
    if not isinstance(value, list) or len(value) < 3:
        raise ValueError(f"{name} must contain three numbers")
    out = [float(value[i]) for i in range(3)]
    if not all(math.isfinite(v) and (not positive or v > 0.0) for v in out):
        raise ValueError(f"{name} must contain three finite{' positive' if positive else ''} numbers")
    return out


def _pose(item: dict[str, Any]) -> list[float] | None:
    pose = item.get("pose")
    value = pose.get("xyz") if isinstance(pose, dict) else item.get("pose_xyz")
    try:
        return _num3(value, "pose")
    except (TypeError, ValueError):
        return None


def _dims(item: dict[str, Any]) -> list[float] | None:
    try:
        return _num3(item.get("dimensions") or item.get("size"), "dimensions", positive=True)
    except (TypeError, ValueError):
        return None


def _identity(item: dict[str, Any]) -> str:
    return " ".join(str(item.get(k, "")).lower().replace("-", "_") for k in ("id", "type", "role", "category"))


def _is_support(item: dict[str, Any]) -> bool:
    return any(token in _identity(item) for token in SUPPORTS)


def _is_obstacle(item: dict[str, Any]) -> bool:
    text = _identity(item)
    return not _is_support(item) and not any(token in text for token in HELPERS) and _pose(item) is not None and _dims(item) is not None


def _aabb(center: list[float], size: list[float], clearance: float = 0.0) -> tuple[list[float], list[float]]:
    half = [size[i] / 2.0 + clearance for i in range(3)]
    return ([center[i] - half[i] for i in range(3)], [center[i] + half[i] for i in range(3)])


def _overlap(a: tuple[list[float], list[float]], b: tuple[list[float], list[float]]) -> bool:
    return all(min(a[1][i], b[1][i]) - max(a[0][i], b[0][i]) > 1e-9 for i in range(3))


def _offsets(max_x: float, max_y: float) -> list[tuple[float, float]]:
    step = 0.05
    xs = range(-max(0, int(max_x // step)), max(0, int(max_x // step)) + 1)
    ys = range(-max(0, int(max_y // step)), max(0, int(max_y // step)) + 1)
    return sorted(((x * step, y * step) for x in xs for y in ys), key=lambda p: (p[0] ** 2 + p[1] ** 2, abs(p[1]), abs(p[0]), p[1], p[0]))


def _placement(items: list[dict[str, Any]], size: list[float]) -> tuple[str, list[float]]:
    supports = sorted((i for i in items if _is_support(i) and _pose(i) and _dims(i)), key=lambda i: str(i.get("id", "")))
    obstacles = [i for i in items if _is_obstacle(i)]
    for support in supports:
        sp, sd = _pose(support), _dims(support)
        assert sp is not None and sd is not None
        mx, my = sd[0] / 2 - size[0] / 2 - EDGE_MARGIN, sd[1] / 2 - size[1] / 2 - EDGE_MARGIN
        if mx < 0 or my < 0:
            continue
        z = sp[2] + sd[2] / 2 + size[2] / 2
        for dx, dy in _offsets(mx, my):
            candidate = [sp[0] + dx, sp[1] + dy, z]
            box = _aabb(candidate, size, CLEARANCE)
            if not any(_overlap(box, _aabb(_pose(o), _dims(o))) for o in obstacles):  # type: ignore[arg-type]
                return str(support.get("id")), candidate
    raise ValueError("no collision-free support-surface slot is available for this object")


def _catalog() -> dict[str, dict[str, Any]]:
    profiles = json.loads(PROFILE.read_text(encoding="utf-8"))
    curation = json.loads(CURATION.read_text(encoding="utf-8"))
    if curation.get("schema_version") != "workcell_studio_curated_add_objects/v1":
        raise ValueError("curated add-object manifest schema is invalid")
    by_id = {str(v.get("asset_id")): dict(v) for v in profiles if isinstance(v, dict) and v.get("asset_id")}
    approved: dict[str, dict[str, Any]] = {}
    for rule in curation.get("objects", []):
        if not isinstance(rule, dict) or rule.get("placement_policy") != "support_surface":
            continue
        asset_id = str(rule.get("asset_id") or "")
        if asset_id not in by_id:
            raise ValueError(f"curated asset is missing from environment profile: {asset_id}")
        approved[asset_id] = {**by_id[asset_id], **rule, "curated_add_enabled": True}
    return approved


def _validate_asset(asset: dict[str, Any]) -> tuple[list[float], str, str | None]:
    size = _num3(asset.get("default_dimensions_m"), "default_dimensions_m", positive=True)
    geometry = str(asset.get("geometry_type") or "mesh")
    primitive = None
    if geometry == "urdf_primitive":
        primitive = str(asset.get("primitive_geometry_type") or "")
        if primitive not in {"box", "cylinder", "sphere", "capsule"}:
            raise ValueError("curated primitive has no supported primitive_geometry_type")
    else:
        mesh = Path(str(asset.get("mesh_path") or ""))
        if mesh.is_absolute() or ".." in mesh.parts or mesh.suffix.lower() not in MESH_SUFFIXES or not (ROOT / mesh).is_file():
            raise ValueError("curated mesh path is missing, unsafe or unsupported")
    return size, geometry, primitive


def _instance_id(asset_id: str, items: list[dict[str, Any]], requested: str | None) -> str:
    used = {str(item.get("id")) for item in items}
    if requested:
        if not ID_RE.fullmatch(requested) or requested in used:
            raise ValueError("instance ID is invalid or already exists")
        return requested
    for index in range(1, 1000):
        candidate = f"{asset_id}_{index:02d}"
        if candidate not in used:
            return candidate
    raise ValueError(f"could not allocate a unique instance ID for {asset_id}")


def _new_item(asset: dict[str, Any], item_id: str, support_id: str, xyz: list[float], size: list[float], geometry: str, primitive: str | None) -> dict[str, Any]:
    kind = str(asset.get("asset_type") or "object")
    item: dict[str, Any] = {
        "id": item_id, "type": kind, "role": "pick_object" if kind == "object" else kind,
        "category": str(asset.get("category") or "Curated Objects"),
        "display_name": f"{asset.get('label', asset.get('asset_id'))} {item_id.rsplit('_', 1)[-1]}",
        "editable": True, "locked": False, "source_layer": "editable_layout",
        "source": "layout/workcell_studio_layout.yaml",
        "pose": {"xyz": xyz, "rpy": [0.0, 0.0, 0.0]}, "dimensions": size,
        "support_surface_ref": support_id, "catalog_asset_id": str(asset["asset_id"]),
        "placement_source": "curated_asset_catalog", "material": {"color": COLORS.get(kind, COLORS["object"])},
    }
    if geometry == "urdf_primitive":
        item.update({"geometry_type": primitive, "primitive_geometry_type": primitive})
    else:
        item.update({"geometry_type": "mesh", "mesh_path": str(asset["mesh_path"])})
    return item


def _atomic_write(path: Path, data: dict[str, Any]) -> None:
    assert yaml is not None
    fd, temp = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, sort_keys=False, default_flow_style=False)
            stream.flush(); os.fsync(stream.fileno())
        os.replace(temp, path)
        directory = os.open(path.parent, os.O_RDONLY)
        try: os.fsync(directory)
        finally: os.close(directory)
    finally:
        if os.path.exists(temp): os.unlink(temp)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Add one approved catalog object to an editable layout.")
    parser.add_argument("--scene", required=True, type=Path); parser.add_argument("--asset-id", required=True)
    parser.add_argument("--instance-id"); parser.add_argument("--expected-layout-sha256")
    parser.add_argument("--write", action="store_true"); parser.add_argument("--backup", action="store_true")
    parser.add_argument("--json", action="store_true"); args = parser.parse_args(argv)
    try:
        if yaml is None: raise ValueError("PyYAML is required")
        scene = args.scene.resolve(); layout_path = scene / LAYOUT_REL
        if not scene.is_dir() or not layout_path.is_file(): raise ValueError(f"editable layout is missing: {layout_path}")
        before_sha = _sha(layout_path)
        if args.expected_layout_sha256 and args.expected_layout_sha256 != before_sha:
            raise ValueError("layout changed after preview; reload and plan the add again")
        layout = yaml.safe_load(layout_path.read_text(encoding="utf-8")) or {}
        if layout.get("schema_version") != "workcell_studio_layout/v1" or not isinstance(layout.get("items"), list):
            raise ValueError("layout must use workcell_studio_layout/v1 with an items list")
        asset = _catalog().get(args.asset_id)
        if asset is None: raise ValueError(f"asset is not approved for curated add: {args.asset_id}")
        size, geometry, primitive = _validate_asset(asset)
        items = [item for item in layout["items"] if isinstance(item, dict)]
        item_id = _instance_id(args.asset_id, items, args.instance_id)
        support_id, xyz = _placement(items, size)
        result: dict[str, Any] = {
            "status": "written" if args.write else "planned", "scene_id": scene.name,
            "asset_id": args.asset_id, "instance_id": item_id, "support_surface_id": support_id,
            "pose_xyz": xyz, "dimensions_m": size, "layout_sha256": before_sha,
            "layout_path": str(layout_path), "license": str(asset.get("license") or "unknown"),
            "source_note": str(asset.get("source_note") or ""),
        }
        if args.write:
            if args.backup:
                stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
                backup = layout_path.with_name(f"{layout_path.name}.{stamp}.bak")
                shutil.copy2(layout_path, backup); result["backup_path"] = str(backup)
            layout["items"].append(_new_item(asset, item_id, support_id, xyz, size, geometry, primitive))
            _atomic_write(layout_path, layout); result["layout_sha256_after"] = _sha(layout_path)
        print(json.dumps(result, sort_keys=True) if args.json else json.dumps(result, indent=2, sort_keys=True))
        return 0
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return _emit_error(str(exc), args.json)


if __name__ == "__main__":
    raise SystemExit(main())
