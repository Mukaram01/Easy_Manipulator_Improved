#!/usr/bin/env python3
"""Generate a dry-run sorting task plan from sorting_manifest.yaml."""

from pathlib import Path
import sys

import yaml


def find_manifest_path() -> Path:
    """Find sorting_manifest.yaml from install/share, with source-tree fallback."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = Path(get_package_share_directory("ur5_2f_sorting_test"))
        share_manifest = share_dir / "sorting_manifest.yaml"
        if share_manifest.exists():
            return share_manifest
    except Exception:
        pass

    source_manifest = Path(__file__).resolve().parents[1] / "sorting_manifest.yaml"
    if source_manifest.exists():
        return source_manifest

    raise FileNotFoundError("Could not locate sorting_manifest.yaml in share or source tree")


def format_triplet(values):
    return f"[{values[0]:.3f}, {values[1]:.3f}, {values[2]:.3f}]"


def main() -> int:
    manifest_path = find_manifest_path()

    with manifest_path.open("r", encoding="utf-8") as handle:
        root = yaml.safe_load(handle)

    manifest = root.get("sorting_manifest") if isinstance(root, dict) else None
    if not isinstance(manifest, dict):
        raise AssertionError("sorting_manifest root mapping is missing")

    objects = manifest.get("objects") or []
    destinations = manifest.get("destinations") or []
    routing = manifest.get("routing") or []

    objects_by_id = {obj["id"]: obj for obj in objects if isinstance(obj, dict) and "id" in obj}
    destinations_by_id = {
        destination["id"]: destination
        for destination in destinations
        if isinstance(destination, dict) and "id" in destination
    }

    print("Dry-run sorting task plan")
    print(f"Manifest: {manifest_path}")
    print()

    for step_index, route in enumerate(routing, start=1):
        object_id = route.get("object")
        destination_id = route.get("destination")
        obj = objects_by_id.get(object_id)
        destination = destinations_by_id.get(destination_id)
        if obj is None or destination is None:
            raise AssertionError(f"Invalid route entry: {route}")

        size = obj.get("approximate_size_m", [0.0, 0.0, 0.0])
        release_offset = destination.get("release_offset_xyz_m", [0.0, 0.0, 0.0])

        print(f"{step_index}. {object_id} -> {destination_id}")
        print(f"   source_frame: {obj.get('frame_id')}")
        print(f"   destination_frame: {destination.get('frame_id')}")
        print(f"   approximate_size_m: {format_triplet(size)}")
        print(f"   pick_hint: {obj.get('pick_hint', 'n/a')}")
        print(f"   release_offset_xyz_m: {format_triplet(release_offset)}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
