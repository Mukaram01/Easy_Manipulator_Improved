#!/usr/bin/env python3
"""Generate runtime-compatible emd_grasp_bridge_payload/v1 for static sorting."""
from __future__ import annotations

import argparse, json
from pathlib import Path
import sys

import importlib.machinery
import importlib.util

SCHEMA_VERSION = "emd_grasp_bridge_payload/v1"
SCENE_PACKAGE = "ur5_2f_sorting_test"
GRASP_CLEARANCE_ABOVE_OBJECT_M = 0.02
PREGRASP_CLEARANCE_M = 0.05
RELEASE_CLEARANCE_M = 0.03


def _load_sibling_script_module(module_name: str):
    """Load a sibling helper script from source tree or installed ROS 2 layout."""
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
    raise ModuleNotFoundError(
        f"Could not load sibling script module '{module_name}'. Searched: {searched}"
    )


runtime_plan = _load_sibling_script_module("generate_sorting_runtime_plan")


def _fallback_pose(frame_id: str, index: int) -> dict:
    return {"frame_id": frame_id, "xyz": [round(0.1 * index, 3), 0.0, 0.1], "rpy": [0.0, 0.0, 0.0]}


def _height_from_size(size: list[float] | tuple[float, ...] | None) -> float:
    if not isinstance(size, (list, tuple)) or len(size) < 3:
        return 0.05
    return float(size[2])


def build_payload(
    plan: dict,
    ros_interface: str,
    service_name: str,
    topic_name: str,
    frame_id: str,
    default_ee_id: str,
    selected_targets: list[str] | None = None,
) -> dict:
    picks = {s["object_id"]: s for s in plan.get("steps", []) if s.get("type") == "pick"}
    places = {s["object_id"]: s for s in plan.get("steps", []) if s.get("type") == "place"}
    warnings: list[str] = []
    targets = []

    selected_set = set(selected_targets or [])
    target_filter_applied = bool(selected_set)

    for idx, object_id in enumerate(sorted(picks.keys()), start=1):
        if target_filter_applied and object_id not in selected_set:
            continue
        pick = picks[object_id]
        place = places.get(object_id)
        if place is None:
            continue
        obj_frame = pick.get("object_frame", object_id)
        dest_frame = place.get("destination_frame", place.get("destination_id"))
        target_pose = _fallback_pose(obj_frame, idx)
        destination_pose = _fallback_pose(dest_frame, idx + 10)
        obj_size = pick.get("approximate_size_m", [0.05, 0.05, 0.05])
        object_height = _height_from_size(obj_size)
        grasp_offset = [0.0, 0.0, round((object_height * 0.5) + GRASP_CLEARANCE_ABOVE_OBJECT_M, 3)]
        pregrasp_offset = [0.0, 0.0, round(grasp_offset[2] + PREGRASP_CLEARANCE_M, 3)]
        release_offset = [0.0, 0.0, round(RELEASE_CLEARANCE_M, 3)]
        rel = place.get("release_offset_xyz_m", [0.0, 0.0, 0.03])
        if isinstance(rel, list) and len(rel) == 3:
            destination_pose["xyz"] = [destination_pose["xyz"][0], destination_pose["xyz"][1], round(destination_pose["xyz"][2] + float(rel[2]) + RELEASE_CLEARANCE_M, 3)]
        warnings.append(f"Using deterministic fallback xyz/rpy for object '{object_id}' and destination '{place.get('destination_id')}'.")

        targets.append({
            "object_id": object_id,
            "target_type": "box",
            "target_pose": target_pose,
            "target_shape": {"type": "box", "dimensions": obj_size},
            "grasp_methods": [{
                "ee_id": default_ee_id,
                "grasp_poses": [{"frame_id": obj_frame, "xyz": grasp_offset, "rpy": [0.0, 0.0, 0.0]}],
                "grasp_ranks": [1.0],
            }],
            "destination_id": place.get("destination_id"),
            "destination_pose": destination_pose,
            "notes": [f"destination-aware release offset: {place.get('release_offset_xyz_m', [0.0,0.0,0.0])}"],
            "diagnostics": {
                "target_object_height_m": round(object_height, 3),
                "generated_grasp_local_offset_xyz_m": grasp_offset,
                "generated_pregrasp_local_offset_xyz_m": pregrasp_offset,
                "generated_destination_release_offset_xyz_m": release_offset,
            },
        })

    status = "PASS" if targets else "WARN"
    return {
        "schema_version": SCHEMA_VERSION,
        "scene_package": SCENE_PACKAGE,
        "mode": "offline",
        "status": status,
        "summary": {"target_count": len(targets), "routing": [f"{t['object_id']}->{t['destination_id']}" for t in targets], "target_filter_applied": target_filter_applied},
        "ros_interface": {
            "service_name": service_name,
            "topic_name": topic_name,
            "message_type": "emd_msgs/msg/GraspTask",
            "service_type": "emd_msgs/srv/GraspRequest",
            "selected": ros_interface,
        },
        "grasp_task": {"task_id": "static_sorting_runtime_bridge", "grasp_targets": targets},
        "warnings": warnings,
        "errors": [],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--ros-interface", choices=["service", "topic"], default="service")
    parser.add_argument("--service-name", default="grasp_requests")
    parser.add_argument("--topic-name", default="grasp_tasks")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--default-ee-id", default="robotiq_2f")
    parser.add_argument("--target", action="append", dest="targets", help="Optional object_id filter; repeat for multiple")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    manifest = runtime_plan.load_manifest(runtime_plan.find_manifest_path())
    plan = runtime_plan.build_runtime_plan(manifest)
    payload = build_payload(plan, args.ros_interface, args.service_name, args.topic_name, args.frame_id, args.default_ee_id, selected_targets=args.targets)
    out = json.dumps(payload, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(out + "\n", encoding="utf-8")
    if args.json:
        print(out)
    else:
        print("Runtime-compatible static sorting bridge payload (offline only)")
        print(f"status: {payload['status']}")
        print(f"targets: {payload['summary']['target_count']}")
        print("No ROS send performed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
