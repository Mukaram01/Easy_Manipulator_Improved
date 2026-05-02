#!/usr/bin/env python3
"""Validation checks for ur5_2f_sorting_test sorting manifest."""

from pathlib import Path
import sys

import yaml


REQUIRED_OBJECT_FIELDS = ("id", "frame_id", "destination")
REQUIRED_DESTINATION_FIELDS = ("id", "frame_id")


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
    for destination in destinations:
        for field in REQUIRED_DESTINATION_FIELDS:
            if not destination.get(field):
                raise AssertionError(f"destination missing required field: {field}")
        destination_ids.add(destination["id"])

    for route in routing:
        object_name = route.get("object")
        destination_name = route.get("destination")
        if not object_name or object_name not in object_ids:
            raise AssertionError(f"routing object '{object_name}' is not defined in objects")
        if not destination_name or destination_name not in destination_ids:
            raise AssertionError(
                f"routing destination '{destination_name}' is not defined in destinations"
            )

    return 0


if __name__ == "__main__":
    sys.exit(main())
