#!/usr/bin/env python3
"""Validate emd_grasp_bridge_payload/v1 and preview guarded execution handoff."""

from __future__ import annotations

import argparse
import importlib.machinery
import importlib.util
import json
from pathlib import Path
import sys

SCHEMA = "sorting_execution_handoff_preview/v1"
SOURCE_SCHEMA = "emd_grasp_bridge_payload/v1"
SCENE_PACKAGE = "ur5_2f_sorting_test"


def _load_sibling_script_module(module_name: str):
    """Load sibling helper script from source or installed ROS 2 executable layout."""
    script_dir = Path(__file__).resolve().parent
    candidates = [
        script_dir / f"{module_name}.py",
        script_dir / module_name,
    ]

    for candidate in candidates:
        if not candidate.exists():
            continue

        unique_name = f"_ur5_2f_sorting_test_{module_name}"
        if candidate.suffix == ".py":
            spec = importlib.util.spec_from_file_location(unique_name, candidate)
            if spec is None or spec.loader is None:
                continue
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module

        loader = importlib.machinery.SourceFileLoader(unique_name, str(candidate))
        spec = importlib.util.spec_from_loader(unique_name, loader)
        if spec is None:
            continue
        module = importlib.util.module_from_spec(spec)
        loader.exec_module(module)
        return module

    searched = ", ".join(str(candidate) for candidate in candidates)
    raise ModuleNotFoundError(f"Could not load sibling script module '{module_name}'. Searched: {searched}")


bridge_from_detections = _load_sibling_script_module("generate_bridge_payload_from_detections")
bridge_from_manifest = _load_sibling_script_module("generate_sorting_emd_bridge_payload")
runtime_plan = _load_sibling_script_module("generate_sorting_runtime_plan")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source_group = parser.add_mutually_exclusive_group(required=True)
    source_group.add_argument("--bridge-payload", type=Path, help="Path to existing emd_grasp_bridge_payload/v1 JSON")
    source_group.add_argument("--detections", type=Path, help="Path to detected_objects/v1 JSON")
    source_group.add_argument(
        "--from-static-manifest",
        action="store_true",
        help="Generate bridge payload from static sorting_manifest.yaml path",
    )
    parser.add_argument("--json", action="store_true", help="Print full JSON handoff preview")
    parser.add_argument("--output", type=Path, help="Optional file output for handoff preview JSON")
    return parser.parse_args(argv)


def _known_destinations() -> dict[str, str]:
    manifest = runtime_plan.load_manifest(runtime_plan.find_manifest_path())
    destinations = manifest.get("destinations", [])
    return {entry.get("id"): entry.get("frame_id") for entry in destinations if entry.get("id") and entry.get("frame_id")}


def _validate_payload(payload: dict, known_destinations: dict[str, str]) -> None:
    if payload.get("schema") != SOURCE_SCHEMA:
        raise AssertionError(f"Invalid payload schema: expected {SOURCE_SCHEMA}, got {payload.get('schema')}")
    if payload.get("mode") != "dry_run":
        raise AssertionError(f"Invalid payload mode: expected dry_run, got {payload.get('mode')}")

    targets = payload.get("targets")
    if not isinstance(targets, list) or len(targets) == 0:
        raise AssertionError("Payload must contain at least one target")

    seen_object_ids: set[str] = set()
    for idx, target in enumerate(targets, start=1):
        object_id = target.get("object_id")
        if not object_id:
            raise AssertionError(f"Target #{idx} missing object_id")
        if object_id in seen_object_ids:
            raise AssertionError(f"Duplicate object_id found: {object_id}")
        seen_object_ids.add(object_id)

        for required in ("object_frame", "pick_hint"):
            if not target.get(required):
                raise AssertionError(f"Target {object_id} missing {required}")

        grasp_target = target.get("grasp_target", {})
        if not grasp_target.get("frame_id"):
            raise AssertionError(f"Target {object_id} missing grasp_target.frame_id")

        destination = target.get("destination", {})
        destination_id = destination.get("id")
        destination_frame = destination.get("frame_id")
        release_offset = destination.get("release_offset_xyz_m")

        if not destination_id:
            raise AssertionError(f"Target {object_id} missing destination.id")
        if not destination_frame:
            raise AssertionError(f"Target {object_id} missing destination.frame_id")
        if destination_id not in known_destinations or destination_frame != known_destinations[destination_id]:
            raise AssertionError(
                f"Target {object_id} has invalid destination ({destination_id}, {destination_frame}); "
                "must match sorting_manifest.yaml destinations"
            )

        if not isinstance(release_offset, list) or len(release_offset) != 3:
            raise AssertionError(f"Target {object_id} destination.release_offset_xyz_m must be a list of exactly 3 values")
        if not all(isinstance(v, (int, float)) for v in release_offset):
            raise AssertionError(f"Target {object_id} destination.release_offset_xyz_m values must be numeric")
        if release_offset[2] <= 0:
            raise AssertionError(f"Target {object_id} destination.release_offset_xyz_m[2] must be > 0")


def _build_source_payload(args: argparse.Namespace) -> tuple[dict, str]:
    if args.bridge_payload is not None:
        return json.loads(args.bridge_payload.read_text(encoding="utf-8")), "bridge_payload"
    if args.detections is not None:
        _, payload = bridge_from_detections.build_payload(args.detections, bridge_from_detections.runtime_plan_from_detections.DEFAULT_MIN_CONFIDENCE)
        return payload, "detections"

    manifest = runtime_plan.load_manifest(runtime_plan.find_manifest_path())
    plan = runtime_plan.build_runtime_plan(manifest)
    return bridge_from_manifest.build_bridge_payload(plan), "static_manifest"


def build_handoff_preview(payload: dict, source_mode: str) -> dict:
    targets = []
    for target in payload.get("targets", []):
        destination = target.get("destination", {})
        target_entry = {
            "object_id": target.get("object_id"),
            "object_frame": target.get("object_frame"),
            "pick_hint": target.get("pick_hint"),
            "destination_id": destination.get("id"),
            "destination_frame": destination.get("frame_id"),
            "release_offset_xyz_m": destination.get("release_offset_xyz_m"),
        }
        if "metadata" in target:
            target_entry["metadata"] = target["metadata"]
        targets.append(target_entry)

    return {
        "schema": SCHEMA,
        "scene_package": SCENE_PACKAGE,
        "mode": "dry_run",
        "robot_motion_requested": False,
        "execution_ready_preview": True,
        "requires_manual_execution_enable": True,
        "source_payload_schema": SOURCE_SCHEMA,
        "source_mode": source_mode,
        "target_count": len(targets),
        "targets": targets,
        "warnings": [
            "Dry-run preview only; robot motion was not requested.",
            "Manual execution enable is required before any robot motion.",
        ],
    }


def print_summary(preview: dict) -> None:
    print("Guarded sorting execution handoff preview")
    print(f"source_mode: {preview['source_mode']}")
    print(f"schema: {preview['schema']}")
    print(f"robot_motion_requested: {preview['robot_motion_requested']}")
    print(f"execution_ready_preview: {preview['execution_ready_preview']}")
    print()
    for idx, target in enumerate(preview.get("targets", []), start=1):
        print(f"{idx}. {target['object_id']} -> {target['destination_id']}")
        print(f"   object_frame: {target['object_frame']}")
        print(f"   destination_frame: {target['destination_frame']}")
        print(f"   release_offset_xyz_m: {target['release_offset_xyz_m']}")
        print(f"   pick_hint: {target['pick_hint']}")
        if target.get("metadata") is not None:
            print(f"   metadata: {target['metadata']}")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    payload, source_mode = _build_source_payload(args)
    _validate_payload(payload, _known_destinations())
    preview = build_handoff_preview(payload, source_mode)

    payload_json = json.dumps(preview, indent=2)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload_json + "\n", encoding="utf-8")

    if args.json:
        print(payload_json)
    else:
        print_summary(preview)
    return 0


if __name__ == "__main__":
    sys.exit(main())
