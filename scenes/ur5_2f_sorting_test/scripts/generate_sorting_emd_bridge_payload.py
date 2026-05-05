#!/usr/bin/env python3
"""Generate dry-run emd_grasp_bridge_payload/v1 from runtime_execution_plan/v1."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import generate_sorting_runtime_plan as runtime_plan

SCHEMA = "emd_grasp_bridge_payload/v1"
SOURCE_SCHEMA = "runtime_execution_plan/v1"
SCENE_PACKAGE = "ur5_2f_sorting_test"


DEFAULT_WARNINGS = [
    "No live EPD detections used; object frames come from static sorting manifest.",
]


def build_bridge_payload(plan: dict) -> dict:
    if plan.get("schema") != SOURCE_SCHEMA:
        raise AssertionError(f"Unsupported runtime plan schema: {plan.get('schema')}")

    pick_steps = {}
    place_steps = {}
    for step in plan.get("steps", []):
        object_id = step.get("object_id")
        if not object_id:
            continue
        if step.get("type") == "pick":
            pick_steps[object_id] = step
        elif step.get("type") == "place":
            place_steps[object_id] = step

    targets = []
    for object_id, pick_step in pick_steps.items():
        place_step = place_steps.get(object_id)
        if place_step is None:
            raise AssertionError(f"Missing place step for object: {object_id}")

        release_offset = place_step.get("release_offset_xyz_m", [0.0, 0.0, 0.0])
        targets.append(
            {
                "id": object_id,
                "object_id": object_id,
                "object_frame": pick_step.get("object_frame", object_id),
                "pick_hint": pick_step.get("pick_hint", "top_grasp"),
                "approximate_size_m": pick_step.get("approximate_size_m", [0.0, 0.0, 0.0]),
                "grasp_target": {
                    "frame_id": pick_step.get("object_frame", object_id),
                    "pose_source": "scene_frame",
                    "preferred_method": pick_step.get("pick_hint", "top_grasp"),
                },
                "destination": {
                    "id": place_step.get("destination_id"),
                    "frame_id": place_step.get("destination_frame", place_step.get("destination_id")),
                    "release_offset_xyz_m": release_offset,
                },
                "notes": ["dry-run bridge payload only; not executed"],
            }
        )

    return {
        "schema": SCHEMA,
        "scene_package": plan.get("scene_package", SCENE_PACKAGE),
        "mode": "dry_run",
        "source_schema": SOURCE_SCHEMA,
        "targets": targets,
        "warnings": DEFAULT_WARNINGS,
    }


def print_summary(payload: dict, runtime_plan_path: Path | None) -> None:
    print("Dry-run EMD grasp bridge payload preview")
    print(f"Source schema: {payload['source_schema']}")
    print(f"Schema: {payload['schema']}")
    print(f"Scene package: {payload['scene_package']}")
    print(f"Mode: {payload['mode']}")
    print(f"Runtime plan: {runtime_plan_path if runtime_plan_path is not None else 'generated from sorting_manifest.yaml'}")
    print()
    for index, target in enumerate(payload.get("targets", []), start=1):
        destination = target["destination"]
        print(
            f"{index}. {target['object_id']} -> {destination['id']} "
            f"(pick_hint={target['pick_hint']}, release_offset={destination['release_offset_xyz_m']})"
        )
    print()
    for warning in payload.get("warnings", []):
        print(f"warning: {warning}")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--runtime-plan",
        type=Path,
        help="Path to runtime_execution_plan/v1 JSON. If omitted, generate from sorting_manifest.yaml.",
    )
    parser.add_argument("--json", action="store_true", help="Print full JSON bridge payload.")
    parser.add_argument("--output", type=Path, help="Write bridge payload JSON to file path.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])

    runtime_plan_path = args.runtime_plan
    if runtime_plan_path is not None:
        runtime_plan_data = json.loads(runtime_plan_path.read_text(encoding="utf-8"))
    else:
        manifest_path = runtime_plan.find_manifest_path()
        manifest = runtime_plan.load_manifest(manifest_path)
        runtime_plan_data = runtime_plan.build_runtime_plan(manifest)

    payload = build_bridge_payload(runtime_plan_data)
    payload_json = json.dumps(payload, indent=2, sort_keys=False)

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload_json + "\n", encoding="utf-8")

    if args.json:
        print(payload_json)
    else:
        print_summary(payload, runtime_plan_path)

    return 0


if __name__ == "__main__":
    sys.exit(main())
