#!/usr/bin/env python3
"""Replay emd_grasp_bridge_payload/v1 into run_grasp_execution runtime interface."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import validate_cell_definition as cell_yaml


def _load_payload(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Payload file not found: {path}")
    if path.suffix.lower() == ".json":
        data = json.loads(path.read_text(encoding="utf-8"))
    else:
        data, _, _ = cell_yaml.load_yaml(path)
    if not isinstance(data, dict):
        raise ValueError("Payload root must be a mapping/object.")
    return data


def _summarize_target(target: dict[str, Any]) -> dict[str, str]:
    obj_id = str(target.get("object_id") or "(unknown)")
    obj_class = str(target.get("target_type") or "(unknown)")
    grasp_pose = target.get("grasp_methods", [{}])[0] if isinstance(target.get("grasp_methods"), list) and target.get("grasp_methods") else {}
    grasp_poses = grasp_pose.get("grasp_poses") if isinstance(grasp_pose, dict) else []
    grasp_frame = "(missing)"
    if isinstance(grasp_poses, list) and grasp_poses and isinstance(grasp_poses[0], dict):
        grasp_frame = str(grasp_poses[0].get("frame_id") or "(missing)")
    destination_id = str(target.get("destination_id") or "(unknown)")
    dest_pose = target.get("destination_pose") if isinstance(target.get("destination_pose"), dict) else {}
    dest_frame = str(dest_pose.get("frame_id") or "(missing)")
    release_mode = "destination-aware" if isinstance(dest_pose.get("xyz"), list) else "legacy fallback"
    return {
        "object_id": obj_id,
        "object_class": obj_class,
        "grasp_frame": grasp_frame,
        "destination_id": destination_id,
        "destination_frame": dest_frame,
        "release_mode": release_mode,
    }


def _validate(payload: dict[str, Any], scene_package: str) -> tuple[list[str], list[str]]:
    warnings: list[str] = []
    errors: list[str] = []
    if payload.get("schema_version") != "emd_grasp_bridge_payload/v1":
        errors.append("schema_version must be 'emd_grasp_bridge_payload/v1'.")
    grasp_task = payload.get("grasp_task") if isinstance(payload.get("grasp_task"), dict) else {}
    targets = grasp_task.get("grasp_targets") if isinstance(grasp_task.get("grasp_targets"), list) else []
    if not targets:
        errors.append("grasp_task.grasp_targets is missing or empty.")
    payload_scene = payload.get("scene_package")
    if isinstance(payload_scene, str) and payload_scene.strip() and payload_scene.strip() != scene_package:
        warnings.append(f"scene_package mismatch: payload='{payload_scene}' cli='{scene_package}'.")
    for t in targets:
        if not isinstance(t, dict):
            errors.append("grasp_target entry must be a mapping.")
            continue
        dest = t.get("destination_pose") if isinstance(t.get("destination_pose"), dict) else {}
        xyz = dest.get("xyz")
        if not isinstance(xyz, list):
            warnings.append(f"Object '{t.get('object_id', '(unknown)')}' has no destination pose xyz; runtime will use fallback release mode.")
        elif len(xyz) != 3 or not all(isinstance(v, (int, float)) for v in xyz):
            errors.append(f"Object '{t.get('object_id', '(unknown)')}' destination pose xyz must be a 3-number list.")
        if not isinstance(dest.get("frame_id"), str) or not str(dest.get("frame_id")).strip():
            errors.append(f"Object '{t.get('object_id', '(unknown)')}' destination pose frame_id is missing.")
        methods = t.get("grasp_methods") if isinstance(t.get("grasp_methods"), list) else []
        if not methods:
            errors.append(f"Object '{t.get('object_id', '(unknown)')}' has no grasp_methods.")
            continue
        first = methods[0] if isinstance(methods[0], dict) else {}
        poses = first.get("grasp_poses") if isinstance(first.get("grasp_poses"), list) else []
        if not poses or not isinstance(poses[0], dict) or not str(poses[0].get("frame_id") or "").strip():
            errors.append(f"Object '{t.get('object_id', '(unknown)')}' grasp pose frame_id is missing.")
    return warnings, errors


def _send_runtime(payload: dict[str, Any], args: argparse.Namespace) -> tuple[bool, str]:
    try:
        import rclpy
        from rclpy.node import Node
        from emd_msgs.msg import GraspTask, GraspTarget, GraspMethod
        from emd_msgs.srv import GraspRequest
        from geometry_msgs.msg import PoseStamped
        from shape_msgs.msg import SolidPrimitive
    except Exception as exc:
        return False, f"Runtime endpoint unavailable: ROS imports failed ({exc})."

    def make_pose(data: dict[str, Any]) -> Any:
        msg = PoseStamped()
        msg.header.frame_id = str(data.get("frame_id") or args.frame_id)
        xyz = data.get("xyz") if isinstance(data.get("xyz"), list) and len(data["xyz"]) == 3 else [0.0, 0.0, 0.0]
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, xyz)
        msg.pose.orientation.w = 1.0
        return msg

    rclpy.init(args=None)
    node = Node("replay_emd_bridge_payload")
    try:
        task = GraspTask()
        task.task_id = str(payload.get("grasp_task", {}).get("task_id", ""))
        for td in payload.get("grasp_task", {}).get("grasp_targets", []):
            if not isinstance(td, dict):
                continue
            t = GraspTarget()
            t.target_type = str(td.get("target_type", "box"))
            t.target_pose = make_pose(td.get("target_pose", {}))
            p = SolidPrimitive()
            p.type = SolidPrimitive.BOX
            p.dimensions = [float(x) for x in td.get("target_shape", {}).get("dimensions", [0.04, 0.04, 0.08])]
            t.target_shape = p
            for md in td.get("grasp_methods", []):
                if not isinstance(md, dict):
                    continue
                m = GraspMethod()
                m.ee_id = str(md.get("ee_id", "robotiq_2f"))
                m.grasp_poses = [make_pose(pose) for pose in md.get("grasp_poses", []) if isinstance(pose, dict)]
                m.grasp_ranks = [float(x) for x in md.get("grasp_ranks", [1.0])]
                t.grasp_methods.append(m)
            task.grasp_targets.append(t)

        if args.ros_interface == "topic":
            pub = node.create_publisher(GraspTask, args.topic_name, 10)
            rclpy.spin_once(node, timeout_sec=0.1)
            pub.publish(task)
            return True, f"Published grasp task to topic '{args.topic_name}'."
        client = node.create_client(GraspRequest, args.service_name)
        if not client.wait_for_service(timeout_sec=3.0):
            return False, f"Runtime endpoint unavailable: service '{args.service_name}' not available."
        req = GraspRequest.Request()
        req.grasp_targets = task.grasp_targets
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, f"Runtime endpoint unavailable: service '{args.service_name}' timed out."
        res = fut.result()
        return bool(res.success), str(res.message)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--payload", type=Path, required=True)
    parser.add_argument("--scene-package", required=True)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--rate", type=float, default=0.2)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--ros-interface", choices=["service", "topic"], default="service")
    parser.add_argument("--service-name", default="grasp_requests")
    parser.add_argument("--topic-name", default="grasp_tasks")
    parser.add_argument("--frame-id", default="base_link")
    args = parser.parse_args(argv)

    try:
        payload = _load_payload(args.payload)
        warnings, errors = _validate(payload, args.scene_package)
        targets = payload.get("grasp_task", {}).get("grasp_targets", [])
        print("PASS: payload loaded" if not errors else "FAIL: payload validation")
        for item in targets:
            if isinstance(item, dict):
                s = _summarize_target(item)
                print(f" - object={s['object_id']} class={s['object_class']} grasp_frame={s['grasp_frame']} dest={s['destination_id']} dest_frame={s['destination_frame']} release={s['release_mode']}")
                if s["release_mode"] == "destination-aware":
                    print(f"PASS: explicit destination release pose will be used for object='{s['object_id']}' destination='{s['destination_id']}'.")
                else:
                    print(f"WARN: No task destination release pose provided; legacy release fallback will be used for object='{s['object_id']}'.")
        for w in warnings:
            print(f"WARN: {w}")
        if errors:
            for e in errors:
                print(f"FAIL: {e}")
            return 1
        if args.dry_run:
            print("PASS: dry-run only; runtime send skipped")
            return 0
        ok, msg = _send_runtime(payload, args)
        print(("PASS: " if ok else "FAIL: ") + msg)
        return 0 if ok else 1
    except (FileNotFoundError, ValueError, json.JSONDecodeError) as exc:
        print(f"FAIL: {exc}")
        return 1
    except Exception as exc:
        print(f"FAIL: unexpected error: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
