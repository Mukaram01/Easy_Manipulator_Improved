#!/usr/bin/env python3
"""Generate deterministic MoveIt collision geometry from the canonical layout.

This is deliberately an offline conversion step.  Workcell Studio Web3D owns
authoring and visualization; the generated manifest is the reviewed input to
the MoveIt PlanningScene publisher and is therefore the collision-truth handoff.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import struct
from typing import Any, Mapping, Sequence

import yaml


SCHEMA = "moveit_planning_scene_manifest/v1"
LAYOUT_SCHEMA = "workcell_studio_layout/v1"
SEMANTIC_ONLY_ROLES = {
    "pick_zone",
    "place_zone",
    "keepout",
    "safety_zone",
    "home_pose",
    "target",
}
PHYSICAL_ROLES = {
    "asset",
    "camera",
    "conveyor",
    "environment_object",
    "fixture",
    "machine",
    "object",
    "pick_object",
    "safety_guard",
    "sensor",
    "support_surface",
    "target_bin",
}


class CollisionManifestError(ValueError):
    """The authored layout cannot produce trustworthy collision geometry."""


@dataclass(frozen=True)
class CollisionSpec:
    id: str
    source_item_id: str
    frame_id: str
    semantic_role: str
    operation: str
    pose: dict[str, list[float]]
    source_geometry: dict[str, Any]
    collision_geometry: dict[str, Any]


def _vector3(value: Any, field: str, *, positive: bool = False) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise CollisionManifestError(f"{field} must be a three-element numeric vector")
    if any(isinstance(component, bool) or not isinstance(component, (int, float)) for component in value):
        raise CollisionManifestError(f"{field} must be a three-element numeric vector")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise CollisionManifestError(f"{field} must contain only finite values")
    if positive and any(component <= 0.0 or component > 1000.0 for component in result):
        raise CollisionManifestError(f"{field} values must be greater than zero and at most 1000 metres")
    return result


def quaternion_from_rpy(rpy: Sequence[float]) -> list[float]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def _quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> list[float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return [
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ]


def _rotate_vector(quaternion: Sequence[float], vector: Sequence[float]) -> tuple[float, float, float]:
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    tx, ty, tz = 2.0 * (qy * vz - qz * vy), 2.0 * (qz * vx - qx * vz), 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def _bounds_from_points(points: Sequence[Sequence[float]]) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if not points:
        raise CollisionManifestError("mesh contains no readable vertices")
    mins = tuple(min(point[axis] for point in points) for axis in range(3))
    maxs = tuple(max(point[axis] for point in points) for axis in range(3))
    if not all(math.isfinite(value) for value in (*mins, *maxs)):
        raise CollisionManifestError("mesh bounds contain non-finite values")
    if any(maxs[axis] - mins[axis] <= 0.0 for axis in range(3)):
        raise CollisionManifestError("mesh bounds are collapsed on one or more axes")
    return mins, maxs


def _stl_bounds(path: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    data = path.read_bytes()
    points: list[tuple[float, float, float]] = []
    if len(data) >= 84:
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        expected_size = 84 + triangle_count * 50
        if triangle_count > 0 and expected_size <= len(data):
            for triangle in range(triangle_count):
                offset = 84 + triangle * 50 + 12
                values = struct.unpack_from("<9f", data, offset)
                points.extend((values[index], values[index + 1], values[index + 2]) for index in (0, 3, 6))
            return _bounds_from_points(points)
    text = data.decode("utf-8", errors="ignore")
    pattern = re.compile(
        r"^\s*vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s*$",
        re.MULTILINE,
    )
    points = [tuple(float(value) for value in match.groups()) for match in pattern.finditer(text)]
    return _bounds_from_points(points)


def _obj_bounds(path: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        fields = line.split()
        if len(fields) >= 4 and fields[0] == "v":
            points.append((float(fields[1]), float(fields[2]), float(fields[3])))
    return _bounds_from_points(points)


def _mesh_bounds(path: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
    try:
        if path.suffix.lower() == ".stl":
            return _stl_bounds(path)
        if path.suffix.lower() == ".obj":
            return _obj_bounds(path)
    except (OSError, ValueError, struct.error, CollisionManifestError):
        return None
    return None


def _resolve_local_mesh(mesh_reference: Any, layout_root: Path | None) -> Path | None:
    if not isinstance(mesh_reference, str) or not mesh_reference.strip():
        return None
    candidate = Path(mesh_reference)
    if candidate.is_absolute():
        return candidate if candidate.is_file() else None
    if layout_root is None or mesh_reference.startswith("package://"):
        return None
    resolved = (layout_root / mesh_reference).resolve()
    return resolved if resolved.is_file() else None


def _collision_policy(item: Mapping[str, Any]) -> tuple[bool, str]:
    collision = item.get("collision")
    if isinstance(collision, Mapping) and collision.get("enabled") is False:
        return False, "collision explicitly disabled"
    role = str(item.get("role") or "").strip().lower()
    item_type = str(item.get("type") or "").strip().lower()
    if role in SEMANTIC_ONLY_ROLES or item_type in SEMANTIC_ONLY_ROLES:
        return False, "semantic authoring marker; not physical collision geometry"
    if role in PHYSICAL_ROLES or item_type in PHYSICAL_ROLES:
        return True, "physical authored item"
    if str(item.get("geometry_type") or "").lower() in {"box", "mesh"}:
        return True, "physical geometry inferred from authored shape"
    return False, "unsupported or non-physical authoring item"


def build_manifest(layout: Mapping[str, Any], *, scene_name: str, source_path: str,
                   source_sha256: str, planning_frame: str = "world",
                   layout_root: Path | None = None) -> dict[str, Any]:
    if not isinstance(layout, Mapping):
        raise CollisionManifestError("layout root must be a mapping")
    schema = layout.get("schema_version") or layout.get("schema")
    if schema != LAYOUT_SCHEMA:
        raise CollisionManifestError(f"layout schema must be {LAYOUT_SCHEMA!r}, got {schema!r}")
    items = layout.get("items")
    if not isinstance(items, list):
        raise CollisionManifestError("layout.items must be a list")

    objects: list[CollisionSpec] = []
    excluded: list[dict[str, str]] = []
    seen: set[str] = set()
    for index, raw_item in enumerate(items):
        if not isinstance(raw_item, Mapping):
            raise CollisionManifestError(f"items[{index}] must be a mapping")
        item_id = raw_item.get("id")
        if not isinstance(item_id, str) or not item_id.strip():
            raise CollisionManifestError(f"items[{index}].id must be a non-empty string")
        if item_id in seen:
            raise CollisionManifestError(f"duplicate layout item id: {item_id!r}")
        seen.add(item_id)
        included, reason = _collision_policy(raw_item)
        if not included:
            excluded.append({"id": item_id, "reason": reason})
            continue

        pose = raw_item.get("pose")
        if not isinstance(pose, Mapping):
            raise CollisionManifestError(f"item {item_id!r}.pose must be a mapping")
        xyz = _vector3(pose.get("xyz"), f"item {item_id!r}.pose.xyz")
        rpy = _vector3(pose.get("rpy"), f"item {item_id!r}.pose.rpy")
        mesh = raw_item.get("mesh") if isinstance(raw_item.get("mesh"), Mapping) else {}
        mesh_ref = mesh.get("path") or mesh.get("uri")
        geometry_type = str(
            raw_item.get("geometry_type")
            or raw_item.get("primitive_geometry_type")
            or ("mesh" if mesh_ref else "box")
        ).lower()
        dimensions = _vector3(raw_item.get("dimensions"), f"item {item_id!r}.dimensions", positive=True)
        source_geometry: dict[str, Any] = {"type": geometry_type}
        collision_xyz = xyz
        collision_quaternion = quaternion_from_rpy(rpy)
        bounds_source = "authored_dimensions"
        if geometry_type == "mesh":
            display_mesh_ref = str(mesh_ref or "")
            local_mesh = _resolve_local_mesh(mesh_ref, layout_root)
            if local_mesh is not None and layout_root is not None:
                try:
                    display_mesh_ref = local_mesh.resolve().relative_to(layout_root.resolve()).as_posix()
                except ValueError:
                    pass
            mesh_scale = _vector3(mesh.get("scale", [1, 1, 1]), f"item {item_id!r}.mesh.scale", positive=True)
            source_geometry["mesh_reference"] = display_mesh_ref
            source_geometry["mesh_scale"] = list(mesh_scale)
            bounds = _mesh_bounds(local_mesh) if local_mesh is not None else None
            if bounds is not None:
                mins, maxs = bounds
                dimensions = tuple((maxs[axis] - mins[axis]) * mesh_scale[axis] for axis in range(3))
                _vector3(dimensions, f"item {item_id!r} derived mesh dimensions", positive=True)
                mesh_center = tuple((mins[axis] + maxs[axis]) * 0.5 * mesh_scale[axis] for axis in range(3))
                mesh_rpy = _vector3(mesh.get("rpy", [0, 0, 0]), f"item {item_id!r}.mesh.rpy")
                mesh_offset = _vector3(mesh.get("origin_offset", [0, 0, 0]), f"item {item_id!r}.mesh.origin_offset")
                owner_q = quaternion_from_rpy(rpy)
                mesh_q = quaternion_from_rpy(mesh_rpy)
                local_center = tuple(mesh_offset[axis] + _rotate_vector(mesh_q, mesh_center)[axis] for axis in range(3))
                rotated_center = _rotate_vector(owner_q, local_center)
                collision_xyz = tuple(xyz[axis] + rotated_center[axis] for axis in range(3))
                collision_quaternion = _quaternion_multiply(owner_q, mesh_q)
                bounds_source = "mesh_vertices"
                source_geometry["source_bounds"] = {"min": list(mins), "max": list(maxs)}

        collision = raw_item.get("collision") if isinstance(raw_item.get("collision"), Mapping) else {}
        collision_mode = str(collision.get("mode") or ("box_proxy" if geometry_type == "mesh" else "primitive_box"))
        if collision_mode not in {"box_proxy", "primitive_box"}:
            raise CollisionManifestError(
                f"item {item_id!r} collision mode {collision_mode!r} is unsupported; "
                "use box_proxy or primitive_box"
            )
        spec = CollisionSpec(
            id=f"workcell::{item_id}",
            source_item_id=item_id,
            frame_id=str(raw_item.get("frame") or planning_frame),
            semantic_role=str(raw_item.get("role") or raw_item.get("type") or "environment_object"),
            operation="ADD",
            pose={"xyz": list(collision_xyz), "rpy": list(rpy), "quaternion_xyzw": collision_quaternion},
            source_geometry=source_geometry,
            collision_geometry={
                "type": "box",
                "dimensions_m": list(dimensions),
                "fidelity": collision_mode,
                "review_required": collision_mode == "box_proxy",
                "bounds_source": bounds_source,
            },
        )
        objects.append(spec)

    objects.sort(key=lambda entry: entry.source_item_id)
    excluded.sort(key=lambda entry: entry["id"])
    if not objects:
        raise CollisionManifestError("layout produced no physical collision objects")
    return {
        "schema_version": SCHEMA,
        "scene_name": scene_name,
        "planning_frame": planning_frame,
        "source": {
            "canonical_layout": source_path,
            "canonical_layout_sha256": source_sha256,
        },
        "truth_boundary": {
            "authoring_visualization": "Workcell Studio Web3D",
            "viewer_aabb_feedback": "advisory_only",
            "collision_and_planning_truth": "MoveIt PlanningScene",
        },
        "objects": [asdict(entry) for entry in objects],
        "excluded_items": excluded,
        "summary": {
            "collision_object_count": len(objects),
            "excluded_item_count": len(excluded),
            "box_proxy_count": sum(entry.collision_geometry["fidelity"] == "box_proxy" for entry in objects),
            "exact_mesh_collision_count": 0,
        },
        "safety": {
            "publishes_robot_motion": False,
            "commands_hardware": False,
            "planning_scene_diff_only": True,
        },
    }


def load_and_build(layout_path: Path, *, scene_name: str | None = None,
                   planning_frame: str = "world") -> dict[str, Any]:
    try:
        source_bytes = layout_path.read_bytes()
        layout = yaml.safe_load(source_bytes)
    except (OSError, yaml.YAMLError) as exc:
        raise CollisionManifestError(f"cannot read canonical layout {layout_path}: {exc}") from exc
    effective_scene = scene_name or (layout.get("scene_name") if isinstance(layout, Mapping) else None) or layout_path.parent.parent.name
    return build_manifest(
        layout,
        scene_name=str(effective_scene),
        source_path="layout/workcell_studio_layout.yaml",
        source_sha256=hashlib.sha256(source_bytes).hexdigest(),
        planning_frame=planning_frame,
        layout_root=layout_path.parent.parent,
    )


def validate_manifest(data: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    if data.get("schema_version") != SCHEMA:
        errors.append(f"schema_version must be {SCHEMA!r}")
    objects = data.get("objects")
    if not isinstance(objects, list) or not objects:
        errors.append("objects must be a non-empty list")
        return errors
    ids: set[str] = set()
    for index, obj in enumerate(objects):
        if not isinstance(obj, Mapping):
            errors.append(f"objects[{index}] must be a mapping")
            continue
        object_id = obj.get("id")
        if not isinstance(object_id, str) or not object_id:
            errors.append(f"objects[{index}].id must be a non-empty string")
        elif object_id in ids:
            errors.append(f"duplicate collision object id {object_id!r}")
        else:
            ids.add(object_id)
        geometry = obj.get("collision_geometry")
        if not isinstance(geometry, Mapping) or geometry.get("type") != "box":
            errors.append(f"objects[{index}] must use supported box collision geometry")
        else:
            try:
                _vector3(geometry.get("dimensions_m"), f"objects[{index}].collision_geometry.dimensions_m", positive=True)
            except CollisionManifestError as exc:
                errors.append(str(exc))
        pose = obj.get("pose")
        if not isinstance(pose, Mapping):
            errors.append(f"objects[{index}].pose must be a mapping")
        else:
            try:
                _vector3(pose.get("xyz"), f"objects[{index}].pose.xyz")
                _vector3(pose.get("rpy"), f"objects[{index}].pose.rpy")
                quaternion = pose.get("quaternion_xyzw")
                if not isinstance(quaternion, list) or len(quaternion) != 4 or any(
                    isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                    for value in quaternion
                ):
                    errors.append(f"objects[{index}].pose.quaternion_xyzw must contain four finite numbers")
            except CollisionManifestError as exc:
                errors.append(str(exc))
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--scene-name")
    parser.add_argument("--planning-frame", default="world")
    parser.add_argument("--check", action="store_true", help="fail when output is missing or stale; do not write")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    try:
        manifest = load_and_build(args.layout, scene_name=args.scene_name, planning_frame=args.planning_frame)
        errors = validate_manifest(manifest)
        if errors:
            raise CollisionManifestError("; ".join(errors))
        rendered = yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True)
        if args.check:
            current = args.output.read_text(encoding="utf-8") if args.output.is_file() else ""
            if current != rendered:
                raise CollisionManifestError(f"collision manifest is missing or stale: {args.output}")
        else:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered, encoding="utf-8")
        result = {"status": "PASS", "output": str(args.output), **manifest["summary"]}
        print(json.dumps(result, indent=2) if args.json else f"PASS: {args.output}")
        return 0
    except (CollisionManifestError, OSError) as exc:
        result = {"status": "FAIL", "error": str(exc)}
        print(json.dumps(result, indent=2) if args.json else f"FAIL: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
