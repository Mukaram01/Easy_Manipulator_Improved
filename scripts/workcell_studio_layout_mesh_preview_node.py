#!/usr/bin/env python3
"""Publish canonical Workcell Studio mesh items as RViz markers.

Parsing and transform functions in this module intentionally have no ROS
dependency, so layout contracts can be checked on development machines that do
not have ROS installed.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path, PurePosixPath
import re
import sys
from typing import Any, Mapping, Sequence
from urllib.parse import urlsplit

import yaml


MARKER_NAMESPACE = "workcell_studio_canonical_mesh"
_PACKAGE_NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")


class LayoutMeshError(ValueError):
    """A layout mesh cannot be represented faithfully in RViz."""


@dataclass(frozen=True)
class MeshMarkerSpec:
    item_id: str
    marker_id: int
    mesh_resource: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    scale: tuple[float, float, float]


def parse_runtime_arguments(
    argv: Sequence[str], remove_ros_args,
) -> tuple[argparse.Namespace, list[str]]:
    """Parse application arguments while preserving the complete ROS argv.

    ``remove_ros_args`` is injected so importing this module (and exercising
    its layout helpers) does not require a ROS installation.  At runtime the
    caller supplies :func:`rclpy.utilities.remove_ros_args`.
    """
    ros_argv = list(argv)
    application_argv = remove_ros_args(args=ros_argv)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("layout", help="canonical workcell_studio_layout.yaml")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--topic", default="workcell_studio/layout_mesh_markers")
    parser.add_argument("--owning-package")
    return parser.parse_args(application_argv[1:]), ros_argv


def _vector3(value: Any, field: str, default: Sequence[float] | None = None) -> tuple[float, float, float]:
    if value is None and default is not None:
        value = default
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise LayoutMeshError(f"{field} must be a three-element numeric vector")
    if any(isinstance(component, bool) or not isinstance(component, (int, float)) for component in value):
        raise LayoutMeshError(f"{field} must be a three-element numeric vector")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise LayoutMeshError(f"{field} must contain only finite numbers")
    return result


def _quaternion_from_rpy(rpy: Sequence[float]) -> tuple[float, float, float, float]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(quaternion: Sequence[float], vector: Sequence[float]) -> tuple[float, float, float]:
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    # Unit-quaternion rotation, expanded to avoid constructing ROS messages.
    tx, ty, tz = 2.0 * (qy * vz - qz * vy), 2.0 * (qz * vx - qx * vz), 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def compose_mesh_pose(
    owner_xyz: Sequence[float],
    owner_rpy: Sequence[float],
    local_offset: Sequence[float],
    local_rpy: Sequence[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Return ``T_owner * T_mesh_local`` as position and xyzw quaternion."""
    owner_q = _quaternion_from_rpy(owner_rpy)
    local_q = _quaternion_from_rpy(local_rpy)
    rotated_offset = _rotate_vector(owner_q, local_offset)
    position = tuple(owner_xyz[index] + rotated_offset[index] for index in range(3))
    return position, _quaternion_multiply(owner_q, local_q)


def _validate_owning_package(owning_package: str | None) -> str | None:
    if owning_package is None:
        return None
    if not isinstance(owning_package, str) or not _PACKAGE_NAME.fullmatch(owning_package):
        raise LayoutMeshError(f"invalid owning package name: {owning_package!r}")
    return owning_package


def resolve_mesh_resource(value: Any, owning_package: str | None = None) -> str:
    """Resolve a safe canonical repository mesh reference to a package URI."""
    owning_package = _validate_owning_package(owning_package)
    if not isinstance(value, str) or not value.strip():
        raise LayoutMeshError("mesh.path or mesh.uri must be a non-empty string")
    resource = value.strip().replace("\\", "/")
    parsed = urlsplit(resource)
    if parsed.query or parsed.fragment:
        raise LayoutMeshError(f"mesh URI query strings and fragments are not allowed: {value!r}")
    if parsed.scheme:
        if parsed.scheme != "package":
            raise LayoutMeshError(f"unsafe or unsupported mesh URI: {value!r}")
        package = parsed.netloc
        asset_path = parsed.path.lstrip("/")
        if not _PACKAGE_NAME.fullmatch(package) or not asset_path:
            raise LayoutMeshError(f"invalid package mesh URI: {value!r}")
        parts = PurePosixPath(asset_path).parts
        if ".." in parts or "." in parts:
            raise LayoutMeshError(f"mesh path traversal is not allowed: {value!r}")
        return f"package://{package}/{asset_path}"

    parts = PurePosixPath(resource).parts
    if resource.startswith("/") or ".." in parts or "." in parts:
        raise LayoutMeshError(f"mesh path traversal or absolute paths are not allowed: {value!r}")
    if len(parts) >= 4 and parts[:2] == ("assets", "environment"):
        package = parts[2]
        remainder = parts[3:]
        if _PACKAGE_NAME.fullmatch(package) and remainder:
            return f"package://{package}/{'/'.join(remainder)}"
    if owning_package is not None and len(parts) > 2 and parts[:2] == ("assets", "imported"):
        return f"package://{owning_package}/assets/imported/{'/'.join(parts[2:])}"
    imported_prefix = ("workcell_builder", "workcell_builder", "assets", "imported")
    if len(parts) > len(imported_prefix) and parts[: len(imported_prefix)] == imported_prefix:
        return f"package://workcell_builder/assets/imported/{'/'.join(parts[len(imported_prefix):])}"
    raise LayoutMeshError(f"mesh path cannot be mapped to a ROS package URI: {value!r}")


def resolve_mesh_package_name(value: Any, owning_package: str | None = None) -> str:
    """Return the validated ROS package owning a canonical mesh resource.

    This deliberately builds on :func:`resolve_mesh_resource` so offline
    generators and the runtime publisher cannot drift into interpreting mesh
    paths differently.
    """
    resource = resolve_mesh_resource(value, owning_package)
    return urlsplit(resource).netloc


def parse_layout_meshes(
    layout: Mapping[str, Any], owning_package: str | None = None,
) -> list[MeshMarkerSpec]:
    """Normalize mesh-backed physical layout items into ROS-independent specs."""
    owning_package = _validate_owning_package(owning_package)
    if not isinstance(layout, Mapping):
        raise LayoutMeshError("layout root must be a mapping")
    items = layout.get("items", [])
    if not isinstance(items, list):
        raise LayoutMeshError("layout items must be a list")

    pending: list[tuple[str, str, tuple[float, float, float], tuple[float, float, float, float], tuple[float, float, float]]] = []
    seen_ids: set[str] = set()
    for index, item in enumerate(items):
        if not isinstance(item, Mapping) or item.get("geometry_type") != "mesh":
            continue
        mesh = item.get("mesh")
        if not isinstance(mesh, Mapping):
            continue
        item_id = item.get("id")
        if not isinstance(item_id, str) or not item_id.strip():
            raise LayoutMeshError(f"items[{index}].id must be a non-empty string")
        if item_id in seen_ids:
            raise LayoutMeshError(f"duplicate mesh item id: {item_id!r}")
        seen_ids.add(item_id)
        raw_resource = mesh.get("path") if mesh.get("path") is not None else mesh.get("uri")
        resource = resolve_mesh_resource(raw_resource, owning_package)
        pose = item.get("pose", {})
        if not isinstance(pose, Mapping):
            raise LayoutMeshError(f"item {item_id!r} pose must be a mapping")
        owner_xyz = _vector3(pose.get("xyz"), f"item {item_id!r} pose.xyz", (0, 0, 0))
        owner_rpy = _vector3(pose.get("rpy"), f"item {item_id!r} pose.rpy", (0, 0, 0))
        offset = _vector3(mesh.get("origin_offset"), f"item {item_id!r} mesh.origin_offset", (0, 0, 0))
        mesh_rpy = _vector3(mesh.get("rpy"), f"item {item_id!r} mesh.rpy", (0, 0, 0))
        scale = _vector3(mesh.get("scale"), f"item {item_id!r} mesh.scale", (1, 1, 1))
        if any(component <= 0 for component in scale):
            raise LayoutMeshError(f"item {item_id!r} mesh.scale values must be positive")
        position, orientation = compose_mesh_pose(owner_xyz, owner_rpy, offset, mesh_rpy)
        pending.append((item_id, resource, position, orientation, scale))

    marker_ids = {item_id: marker_id for marker_id, item_id in enumerate(sorted(item[0] for item in pending))}
    return [MeshMarkerSpec(item_id, marker_ids[item_id], resource, position, orientation, scale)
            for item_id, resource, position, orientation, scale in pending]


def load_layout_meshes(
    layout_path: str | Path, owning_package: str | None = None,
) -> list[MeshMarkerSpec]:
    owning_package = _validate_owning_package(owning_package)
    path = Path(layout_path)
    try:
        layout = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise LayoutMeshError(f"could not read canonical layout {path}: {error}") from error
    return parse_layout_meshes(layout, owning_package)


def build_marker_array(specs: Sequence[MeshMarkerSpec], frame_id: str):
    from visualization_msgs.msg import Marker, MarkerArray

    result = MarkerArray()
    for spec in specs:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = MARKER_NAMESPACE
        marker.id = spec.marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = spec.mesh_resource
        marker.mesh_use_embedded_materials = True
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = spec.position
        (marker.pose.orientation.x, marker.pose.orientation.y,
         marker.pose.orientation.z, marker.pose.orientation.w) = spec.orientation
        marker.scale.x, marker.scale.y, marker.scale.z = spec.scale
        marker.color.r = marker.color.g = marker.color.b = 0.7
        marker.color.a = 1.0
        result.markers.append(marker)
    return result


def run_ros_node(
    layout_path: str, frame_id: str, topic: str, ros_args: Sequence[str],
    owning_package: str | None = None,
) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from visualization_msgs.msg import MarkerArray

    specs = load_layout_meshes(layout_path, owning_package)
    # Pass the untouched launch argv to rclpy so remaps such as ``__node`` are
    # applied rather than merely tolerated by application parsing.
    rclpy.init(args=list(ros_args))
    node = Node("workcell_studio_layout_mesh_preview")
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE
    publisher = node.create_publisher(MarkerArray, topic, qos)
    publisher.publish(build_marker_array(specs, frame_id))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Normal ros2 launch shutdown must not surface as a node failure.
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


def main() -> None:
    from rclpy.utilities import remove_ros_args

    args, ros_args = parse_runtime_arguments(sys.argv, remove_ros_args)
    run_ros_node(args.layout, args.frame_id, args.topic, ros_args, args.owning_package)


if __name__ == "__main__":
    main()
