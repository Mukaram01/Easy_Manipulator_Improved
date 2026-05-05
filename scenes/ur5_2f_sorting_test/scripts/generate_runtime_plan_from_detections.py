#!/usr/bin/env python3
"""Generate runtime_execution_plan/v1 from detected_objects/v1 fixture input."""

from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path
import sys

SCHEMA = "runtime_execution_plan/v1"
INPUT_SCHEMA = "detected_objects/v1"
SCENE_PACKAGE = "ur5_2f_sorting_test"
MODE = "dry_run"
DEFAULT_MIN_CONFIDENCE = 0.5


def find_sorting_manifest_path() -> Path:
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
    if value.startswith('"') and value.endswith('"'):
        return value[1:-1]
    if ":" not in value and value.replace("_", "").isalnum():
        return value
    return value


def load_manifest(path: Path) -> dict:
    lines = path.read_text(encoding="utf-8").splitlines()
    manifest = {
        "objects": [],
        "destinations": [],
        "routing": [],
        "class_routing": {},
        "class_routing_defaults": {},
    }
    section = None
    current = None

    for raw in lines:
        stripped = raw.strip()
        if not stripped or stripped.startswith("#") or stripped == "sorting_manifest:":
            continue
        if stripped in {
            "objects:",
            "destinations:",
            "routing:",
            "class_routing:",
            "class_routing_defaults:",
        }:
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

        if section in {"class_routing", "class_routing_defaults"} and ":" in stripped:
            key, value = stripped.split(":", 1)
            manifest[section][key.strip()] = _parse_scalar(value)
            continue

        if current is not None and ":" in stripped:
            key, value = stripped.split(":", 1)
            current[key.strip()] = _parse_scalar(value)

    return manifest


def load_detections(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != INPUT_SCHEMA:
        raise AssertionError(f"Expected schema {INPUT_SCHEMA}, got {payload.get('schema')}")
    objects = payload.get("objects")
    if not isinstance(objects, list):
        raise AssertionError("detected_objects payload must include objects list")
    return payload


def build_runtime_plan(detections: dict, manifest: dict, min_confidence: float) -> dict:
    destinations = manifest.get("destinations") or []
    class_routing = manifest.get("class_routing") or {}
    defaults = manifest.get("class_routing_defaults") or {}

    destinations_by_id = {entry.get("id"): entry for entry in destinations if isinstance(entry, dict)}
    reject_destination = defaults.get("unknown_class_destination")
    low_conf_destination = defaults.get("low_confidence_destination")

    steps = []
    for detected in detections.get("objects", []):
        detected_id = detected.get("id")
        class_label = detected.get("class_label")
        confidence = float(detected.get("confidence", 0.0))
        object_frame = detected.get("frame_id")
        pose_source = detected.get("pose_source", "unknown")
        approximate_size = detected.get("approximate_size_m", [0.0, 0.0, 0.0])

        if confidence < min_confidence:
            destination_id = low_conf_destination
        else:
            destination_id = class_routing.get(class_label, reject_destination)

        if destination_id not in destinations_by_id:
            raise AssertionError(
                f"Destination '{destination_id}' for detected object '{detected_id}' is undefined"
            )

        destination = destinations_by_id[destination_id]
        destination_frame = destination.get("frame_id", destination_id)
        release_offset = destination.get("release_offset_xyz_m", [0.0, 0.0, 0.0])

        metadata = {
            "detected_object_id": detected_id,
            "class_label": class_label,
            "confidence": confidence,
            "pose_source": pose_source,
            "object_frame": object_frame,
            "approximate_size_m": approximate_size,
        }

        steps.append(
            {
                "id": f"pick_{detected_id}",
                "type": "pick",
                "object_id": detected_id,
                "object_frame": object_frame,
                "pick_hint": "top_grasp",
                **metadata,
                "destination_id": destination_id,
            }
        )
        steps.append(
            {
                "id": f"place_{detected_id}_to_{destination_id}",
                "type": "place",
                "object_id": detected_id,
                "destination_id": destination_id,
                "destination_frame": destination_frame,
                "release_offset_xyz_m": release_offset,
                **metadata,
            }
        )

    return {
        "schema": SCHEMA,
        "scene_package": SCENE_PACKAGE,
        "mode": MODE,
        "planning_frame": detections.get("planning_frame", "world"),
        "source_schema": INPUT_SCHEMA,
        "min_confidence": min_confidence,
        "steps": steps,
    }


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--detections", type=Path, required=True, help="Path to detected_objects/v1 JSON")
    parser.add_argument("--json", action="store_true", help="Print full JSON runtime plan.")
    parser.add_argument("--output", type=Path, help="Write JSON runtime plan to file path.")
    parser.add_argument("--min-confidence", type=float, default=DEFAULT_MIN_CONFIDENCE)
    return parser.parse_args(argv)


def print_summary(plan: dict, detections_path: Path) -> None:
    print("Dry-run runtime plan from detections")
    print(f"Detections: {detections_path}")
    print(f"Schema: {plan['schema']}")
    print(f"Planning frame: {plan['planning_frame']}")
    print()
    place_steps = [s for s in plan.get("steps", []) if s.get("type") == "place"]
    for idx, step in enumerate(place_steps, start=1):
        print(
            f"{idx}. {step['detected_object_id']} ({step['class_label']}, conf={step['confidence']:.2f}) "
            f"-> {step['destination_id']}"
        )


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    manifest = load_manifest(find_sorting_manifest_path())
    detections = load_detections(args.detections)
    plan = build_runtime_plan(detections, manifest, args.min_confidence)
    plan_json = json.dumps(plan, indent=2)

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(plan_json + "\n", encoding="utf-8")

    if args.json:
        print(plan_json)
    else:
        print_summary(plan, args.detections)

    return 0


if __name__ == "__main__":
    sys.exit(main())
