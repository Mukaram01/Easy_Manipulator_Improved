#!/usr/bin/env python3
"""Generate runtime-compatible emd_grasp_bridge_payload/v1 for static sorting."""
from __future__ import annotations

import argparse, json
from pathlib import Path
import sys

import importlib.util

SCHEMA_VERSION = "emd_grasp_bridge_payload/v1"
SCENE_PACKAGE = "ur5_2f_sorting_test"


def _load_runtime_plan_module():
    p = Path(__file__).resolve().parent / "generate_sorting_runtime_plan.py"
    spec = importlib.util.spec_from_file_location("_runtime_plan", p)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module

runtime_plan = _load_runtime_plan_module()


def _fallback_pose(frame_id: str, index: int) -> dict:
    return {"frame_id": frame_id, "xyz": [round(0.1 * index, 3), 0.0, 0.1], "rpy": [0.0, 0.0, 0.0]}


def build_payload(plan: dict, ros_interface: str, service_name: str, topic_name: str, frame_id: str, default_ee_id: str) -> dict:
    picks = {s["object_id"]: s for s in plan.get("steps", []) if s.get("type") == "pick"}
    places = {s["object_id"]: s for s in plan.get("steps", []) if s.get("type") == "place"}
    warnings: list[str] = []
    targets = []

    for idx, object_id in enumerate(sorted(picks.keys()), start=1):
        pick = picks[object_id]
        place = places.get(object_id)
        if place is None:
            continue
        obj_frame = pick.get("object_frame", object_id)
        dest_frame = place.get("destination_frame", place.get("destination_id"))
        target_pose = _fallback_pose(obj_frame, idx)
        destination_pose = _fallback_pose(dest_frame, idx + 10)
        rel = place.get("release_offset_xyz_m", [0.0, 0.0, 0.03])
        if isinstance(rel, list) and len(rel) == 3:
            destination_pose["xyz"] = [destination_pose["xyz"][0], destination_pose["xyz"][1], round(destination_pose["xyz"][2] + float(rel[2]), 3)]
        warnings.append(f"Using deterministic fallback xyz/rpy for object '{object_id}' and destination '{place.get('destination_id')}'.")

        targets.append({
            "object_id": object_id,
            "target_type": "box",
            "target_pose": target_pose,
            "target_shape": {"type": "box", "dimensions": pick.get("approximate_size_m", [0.05, 0.05, 0.05])},
            "grasp_methods": [{
                "ee_id": default_ee_id,
                "grasp_poses": [{"frame_id": obj_frame, "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}],
                "grasp_ranks": [1.0],
            }],
            "destination_id": place.get("destination_id"),
            "destination_pose": destination_pose,
            "notes": [f"destination-aware release offset: {place.get('release_offset_xyz_m', [0.0,0.0,0.0])}"],
        })

    status = "PASS" if len(targets) == 3 else "WARN"
    return {
        "schema_version": SCHEMA_VERSION,
        "scene_package": SCENE_PACKAGE,
        "mode": "offline",
        "status": status,
        "summary": {"target_count": len(targets), "routing": [f"{t['object_id']}->{t['destination_id']}" for t in targets]},
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
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    manifest = runtime_plan.load_manifest(runtime_plan.find_manifest_path())
    plan = runtime_plan.build_runtime_plan(manifest)
    payload = build_payload(plan, args.ros_interface, args.service_name, args.topic_name, args.frame_id, args.default_ee_id)
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
