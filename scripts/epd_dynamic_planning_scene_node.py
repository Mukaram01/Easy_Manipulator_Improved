#!/usr/bin/env python3
"""Consume production EPD tracking and update MoveIt's live PlanningScene."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import queue
import sys
import threading
from typing import Any

SCRIPTS = Path(__file__).resolve().parent
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from capture_epd_detected_objects import convert_epd_message_to_detected_objects
from dynamic_object_planning_scene_bridge import apply_and_verify, build_collision_object
from epd_snapshot_adapter import normalize_detected_objects_snapshot


def initial_summary() -> dict[str, Any]:
    return {
        "schema": "workcell_perception_planning_scene_smoke/v1",
        "replay_source": True, "realsense_used": False,
        "objects_received": 0, "objects_normalized": 0,
        "objects_applied": 0, "objects_updated": 0, "objects_removed": 0,
        "tf_failures": 0, "geometry_blocked": 0,
        "planning_scene_verified_ids": [], "duplicate_ids": [],
        "lost_removal_supported": False, "result": "RUNNING",
    }


def record_verified(summary: dict[str, Any], object_id: str) -> None:
    verified = summary["planning_scene_verified_ids"]
    if object_id in verified:
        summary["objects_updated"] += 1
    else:
        verified.append(object_id)
        verified.sort()
    summary["objects_applied"] += 1
    summary["duplicate_ids"] = [item for item in set(verified) if verified.count(item) > 1]


def write_summary(path: Path, summary: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/easy_perception_deployment/epd_tracking_output")
    parser.add_argument("--normalized-topic", default="/workcell_studio/detected_objects")
    parser.add_argument("--scene-package", default="ur5_2f_test")
    parser.add_argument("--planning-frame", default="world")
    parser.add_argument("--camera-id", default="fixture_camera")
    parser.add_argument("--frame-fallback", default="camera_color_optical_frame")
    parser.add_argument("--service", default="/apply_planning_scene")
    parser.add_argument("--verify-service", default="/get_planning_scene")
    parser.add_argument("--timeout-seconds", type=float, default=5.0)
    parser.add_argument("--summary-output", type=Path, default=Path("/tmp/p8b_integration_summary.json"))
    args, ros_args = parser.parse_known_args(argv)

    import rclpy
    from epd_msgs.msg import EPDObjectTracking
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener
    import tf2_geometry_msgs  # noqa: F401

    rclpy.init(args=ros_args)
    node = Node("epd_dynamic_planning_scene_bridge")
    node._external_executor_spinning = True
    buffer = Buffer()
    listener = TransformListener(buffer, node)  # noqa: F841
    publisher = node.create_publisher(String, args.normalized_topic, 10)
    summary = initial_summary()
    seen_message = False

    def transform_pose(pose: Any, target: str) -> Any:
        try:
            return buffer.transform(pose, target, timeout=Duration(seconds=args.timeout_seconds))
        except TransformException as exc:
            raise RuntimeError(str(exc)) from exc

    work: queue.Queue[Any] = queue.Queue()

    def process_message(message: Any) -> None:
        nonlocal seen_message
        seen_message = True
        detected, warnings = convert_epd_message_to_detected_objects(
            message, args.topic, args.scene_package, args.frame_fallback)
        summary["objects_received"] += len(detected["objects"])
        profile = {"scene_id": args.scene_package, "perception": {"camera": {
            "camera_id": args.camera_id, "frame_id": detected["source"]["frame_id"]}}}
        normalized = normalize_detected_objects_snapshot(detected, profile)
        summary["objects_normalized"] += len(normalized["objects"])
        publisher.publish(String(data=json.dumps(detected, sort_keys=True)))
        for observed in normalized["objects"]:
            object_id = str(observed["object_id"])
            built = build_collision_object(normalized, object_id, args.planning_frame, transform_pose)
            if built.status != "PASS":
                summary["tf_failures"] += int("TF unavailable" in built.reason)
                summary["geometry_blocked"] += int("geometry" in built.reason)
                node.get_logger().warning(f"{built.status} {object_id}: {built.reason}")
                continue
            applied = apply_and_verify(
                node, built.collision_object, args.service, args.verify_service, args.timeout_seconds)
            if applied.status == "PASS":
                record_verified(summary, object_id)
            else:
                node.get_logger().error(f"{applied.status} {object_id}: {applied.reason}")
        summary["result"] = "PASS" if summary["objects_applied"] and not summary["duplicate_ids"] else "RUNNING"
        if warnings:
            summary["adapter_warnings"] = warnings
        write_summary(args.summary_output, summary)

    def worker() -> None:
        while rclpy.ok():
            try:
                message = work.get(timeout=0.1)
            except queue.Empty:
                continue
            process_message(message)
            work.task_done()

    def callback(message: Any) -> None:
        work.put(message)

    subscription = node.create_subscription(  # noqa: F841
        EPDObjectTracking, args.topic, callback, qos_profile_sensor_data)
    worker_thread = threading.Thread(target=worker, name="planning_scene_worker", daemon=True)
    worker_thread.start()
    write_summary(args.summary_output, summary)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not seen_message:
            summary["result"] = "FAIL"
        write_summary(args.summary_output, summary)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if summary["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
