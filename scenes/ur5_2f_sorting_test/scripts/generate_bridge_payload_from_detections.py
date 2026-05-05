#!/usr/bin/env python3
"""Generate offline emd_grasp_bridge_payload/v1 from detected_objects/v1."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import generate_runtime_plan_from_detections as runtime_plan_from_detections
import generate_sorting_emd_bridge_payload as bridge_payload_generator

WARNING_TEXT = "Offline detected_objects/v1 fixture used; no live EPD call was made."


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--detections", type=Path, required=True, help="Path to detected_objects/v1 JSON")
    parser.add_argument("--min-confidence", type=float, default=runtime_plan_from_detections.DEFAULT_MIN_CONFIDENCE)
    parser.add_argument("--json", action="store_true", help="Print full JSON bridge payload")
    parser.add_argument("--runtime-plan-output", type=Path, help="Optional file path for runtime_execution_plan/v1 JSON")
    parser.add_argument("--output", type=Path, help="Optional file path for emd_grasp_bridge_payload/v1 JSON")
    return parser.parse_args(argv)


def build_payload(detections_path: Path, min_confidence: float) -> tuple[dict, dict]:
    manifest = runtime_plan_from_detections.load_manifest(runtime_plan_from_detections.find_sorting_manifest_path())
    detections = runtime_plan_from_detections.load_detections(detections_path)
    plan = runtime_plan_from_detections.build_runtime_plan(detections, manifest, min_confidence)

    payload = bridge_payload_generator.build_bridge_payload(plan)
    payload["source_chain"] = ["detected_objects/v1", "runtime_execution_plan/v1"]

    for target in payload.get("targets", []):
        object_id = target.get("object_id")
        place_step = next(
            (
                step
                for step in plan.get("steps", [])
                if step.get("type") == "place" and step.get("object_id") == object_id
            ),
            None,
        )
        if place_step is not None:
            target["metadata"] = {
                "detected_object_id": place_step.get("detected_object_id"),
                "class_label": place_step.get("class_label"),
                "confidence": place_step.get("confidence"),
                "pose_source": place_step.get("pose_source"),
            }

    warnings = payload.get("warnings", [])
    if WARNING_TEXT not in warnings:
        warnings.append(WARNING_TEXT)
    payload["warnings"] = warnings
    return plan, payload


def print_summary(payload: dict) -> None:
    print("Dry-run bridge payload from offline detections")
    print(f"Schema: {payload.get('schema')}")
    print(f"Source chain: {payload.get('source_chain')}")
    print()
    for idx, target in enumerate(payload.get("targets", []), start=1):
        metadata = target.get("metadata", {})
        destination = target.get("destination", {})
        print(
            f"{idx}. {metadata.get('detected_object_id')} / {metadata.get('class_label')} "
            f"/ conf={float(metadata.get('confidence', 0.0)):.2f} "
            f"/ {destination.get('id')} / release_offset={destination.get('release_offset_xyz_m')}"
        )
    print()
    for warning in payload.get("warnings", []):
        print(f"warning: {warning}")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    plan, payload = build_payload(args.detections, args.min_confidence)

    if args.runtime_plan_output is not None:
        args.runtime_plan_output.parent.mkdir(parents=True, exist_ok=True)
        args.runtime_plan_output.write_text(json.dumps(plan, indent=2) + "\n", encoding="utf-8")

    payload_json = json.dumps(payload, indent=2)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload_json + "\n", encoding="utf-8")

    if args.json:
        print(payload_json)
    else:
        print_summary(payload)
    return 0


if __name__ == "__main__":
    sys.exit(main())
