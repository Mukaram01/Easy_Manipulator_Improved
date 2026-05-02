#!/usr/bin/env python3
"""Validation checks for ur5_2f_sorting_test sorting manifest."""

from pathlib import Path
import re
import sys

import yaml


REQUIRED_OBJECT_FIELDS = ("id", "frame_id", "destination")
REQUIRED_DESTINATION_FIELDS = ("id", "frame_id")
REQUIRED_SCENE_MARKERS = (
    'link name="item_red"',
    'link name="item_blue"',
    'link name="item_green"',
    'link name="bin_a"',
    'link name="bin_b"',
    'link name="reject_bin"',
    'parent="table_"',
)


PROPERTY_PATTERN = re.compile(r'<xacro:property name="([^"]+)" value="([0-9.]+)"/>')
ORIGIN_PATTERN = re.compile(r'<origin xyz="([^"]+)" rpy="0 0 0"/>')


def main() -> int:
    manifest_path = Path(__file__).resolve().parents[1] / "sorting_manifest.yaml"

    with manifest_path.open("r", encoding="utf-8") as handle:
        root = yaml.safe_load(handle)

    manifest = root.get("sorting_manifest") if isinstance(root, dict) else None
    if not isinstance(manifest, dict):
        raise AssertionError("sorting_manifest root mapping is missing")

    objects = manifest.get("objects")
    destinations = manifest.get("destinations")
    routing = manifest.get("routing")

    if not isinstance(objects, list) or not objects:
        raise AssertionError("objects list is missing or empty")
    if not isinstance(destinations, list) or not destinations:
        raise AssertionError("destinations list is missing or empty")
    if not isinstance(routing, list) or not routing:
        raise AssertionError("routing list is missing or empty")

    object_ids = set()
    for obj in objects:
        for field in REQUIRED_OBJECT_FIELDS:
            if not obj.get(field):
                raise AssertionError(f"object missing required field: {field}")
        object_ids.add(obj["id"])

    destination_ids = set()
    destination_frames = {}
    for destination in destinations:
        for field in REQUIRED_DESTINATION_FIELDS:
            if not destination.get(field):
                raise AssertionError(f"destination missing required field: {field}")
        destination_ids.add(destination["id"])
        destination_frames[destination["id"]] = destination["frame_id"]
        offset = destination.get("release_offset_xyz_m", [0.0, 0.0, 0.0])
        if not isinstance(offset, list) or len(offset) != 3 or offset[2] <= 0.0:
            raise AssertionError(
                f"destination '{destination['id']}' has invalid non-positive release_offset z"
            )

    for route in routing:
        object_name = route.get("object")
        destination_name = route.get("destination")
        if not object_name or object_name not in object_ids:
            raise AssertionError(f"routing object '{object_name}' is not defined in objects")
        if not destination_name or destination_name not in destination_ids:
            raise AssertionError(
                f"routing destination '{destination_name}' is not defined in destinations"
            )
        if destination_frames[destination_name] != destination_name:
            raise AssertionError(
                f"routing destination '{destination_name}' resolves to mismatched frame_id "
                f"'{destination_frames[destination_name]}'"
            )

    scene_xacro_path = Path(__file__).resolve().parents[1] / "urdf" / "scene.urdf.xacro"
    scene_xacro_text = scene_xacro_path.read_text(encoding="utf-8")
    for marker in REQUIRED_SCENE_MARKERS:
        if marker not in scene_xacro_text:
            raise AssertionError(f"scene.urdf.xacro is missing required frame/link marker: {marker}")

    properties = dict(PROPERTY_PATTERN.findall(scene_xacro_text))
    table_top_z = float(properties["table_top_z"])

    expected_expressions = {
        "item_red": "${table_top_z + item_red_height / 2}",
        "item_blue": "${table_top_z + item_blue_length / 2}",
        "item_green": "${table_top_z + item_green_radius}",
        "bin_a": "${table_top_z + tray_height}",
        "bin_b": "${table_top_z + tray_height}",
        "reject_bin": "${table_top_z + tray_height}",
    }
    for name, expr in expected_expressions.items():
        if expr not in scene_xacro_text:
            raise AssertionError(f"{name} origin is not aligned from table_top_z expression")

    if table_top_z <= 0.0:
        raise AssertionError("table_top_z must be positive")

    return 0


if __name__ == "__main__":
    sys.exit(main())
