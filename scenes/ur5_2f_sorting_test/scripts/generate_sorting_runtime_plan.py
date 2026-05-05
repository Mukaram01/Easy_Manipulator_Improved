#!/usr/bin/env python3
"""Generate a dry-run runtime_execution_plan/v1 from sorting_manifest.yaml."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import ast


SCHEMA = "runtime_execution_plan/v1"
SCENE_PACKAGE = "ur5_2f_sorting_test"
PLANNING_FRAME = "world"
MODE = "dry_run"


def find_manifest_path() -> Path:
    """Find sorting_manifest.yaml from install/share, with source-tree fallback."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = Path(get_package_share_directory(SCENE_PACKAGE))
        share_manifest = share_dir / "sorting_manifest.yaml"
        if share_manifest.exists():
            return share_manifest
    except Exception:
        pass

    source_manifest = Path(__file__).resolve().parents[1] / "sorting_manifest.yaml"
    if source_manifest.exists():
        return source_manifest

    raise FileNotFoundError("Could not locate sorting_manifest.yaml in share or source tree")


def _parse_scalar(value: str):
    value = value.strip()
    if value.startswith("[") and value.endswith("]"):
        return ast.literal_eval(value)
    if value.startswith(""") and value.endswith("""):
        return value[1:-1]
    return value


def load_manifest(path: Path) -> dict:
    lines = path.read_text(encoding="utf-8").splitlines()
    manifest = {"objects": [], "destinations": [], "routing": []}
    section = None
    current = None

    for raw in lines:
        line = raw.rstrip()
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped == "sorting_manifest:":
            continue
        if stripped in {"objects:", "destinations:", "routing:"}:
            section = stripped[:-1]
            current = None
            continue
        if section in {"objects", "destinations", "routing"} and stripped.startswith("- "):
            current = {}
            manifest[section].append(current)
            stripped = stripped[2:].strip()
            if stripped:
                key, value = stripped.split(":", 1)
                current[key.strip()] = _parse_scalar(value)
            continue
        if current is not None and ":" in stripped:
            key, value = stripped.split(":", 1)
            current[key.strip()] = _parse_scalar(value)

    if not manifest["objects"] or not manifest["destinations"] or not manifest["routing"]:
        raise AssertionError("sorting_manifest root mapping is missing or incomplete")
    return manifest


def build_runtime_plan(manifest: dict) -> dict:
    objects = manifest.get("objects") or []
    destinations = manifest.get("destinations") or []
    routing = manifest.get("routing") or []

    objects_by_id = {obj["id"]: obj for obj in objects if isinstance(obj, dict) and "id" in obj}
    destinations_by_id = {
        destination["id"]: destination
        for destination in destinations
        if isinstance(destination, dict) and "id" in destination
    }

    steps = []
    for route in routing:
        object_id = route.get("object")
        destination_id = route.get("destination")

        obj = objects_by_id.get(object_id)
        destination = destinations_by_id.get(destination_id)
        if obj is None or destination is None:
            raise AssertionError(f"Invalid route entry: {route}")

        object_frame = obj.get("frame_id", object_id)
        destination_frame = destination.get("frame_id", destination_id)
        pick_hint = obj.get("pick_hint", "top_grasp")
        approximate_size = obj.get("approximate_size_m", [0.0, 0.0, 0.0])
        release_offset = destination.get("release_offset_xyz_m", [0.0, 0.0, 0.0])

        steps.append(
            {
                "id": f"pick_{object_id}",
                "type": "pick",
                "object_id": object_id,
                "object_frame": object_frame,
                "pick_hint": pick_hint,
                "approximate_size_m": approximate_size,
                "destination_id": destination_id,
            }
        )
        steps.append(
            {
                "id": f"place_{object_id}_to_{destination_id}",
                "type": "place",
                "object_id": object_id,
                "destination_id": destination_id,
                "destination_frame": destination_frame,
                "release_offset_xyz_m": release_offset,
            }
        )

    return {
        "schema": SCHEMA,
        "scene_package": SCENE_PACKAGE,
        "mode": MODE,
        "planning_frame": PLANNING_FRAME,
        "steps": steps,
    }


def print_summary(plan: dict, manifest_path: Path) -> None:
    print("Dry-run sorting runtime execution plan preview")
    print(f"Manifest: {manifest_path}")
    print(f"Schema: {plan['schema']}")
    print(f"Scene package: {plan['scene_package']}")
    print(f"Mode: {plan['mode']}")
    print(f"Planning frame: {plan['planning_frame']}")
    print()

    steps = plan.get("steps", [])
    for index in range(0, len(steps), 2):
        pick_step = steps[index]
        place_step = steps[index + 1]
        route_num = (index // 2) + 1
        print(
            f"{route_num}. {pick_step['object_id']} -> {place_step['destination_id']} "
            f"({pick_step['id']}, {place_step['id']})"
        )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="Print full JSON runtime plan.")
    parser.add_argument("--output", type=Path, help="Write JSON runtime plan to a file path.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    manifest_path = find_manifest_path()
    manifest = load_manifest(manifest_path)
    plan = build_runtime_plan(manifest)

    plan_json = json.dumps(plan, indent=2, sort_keys=False)

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(plan_json + "\n", encoding="utf-8")

    if args.json:
        print(plan_json)
    else:
        print_summary(plan, manifest_path)

    return 0


if __name__ == "__main__":
    sys.exit(main())
