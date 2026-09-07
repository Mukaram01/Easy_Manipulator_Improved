#!/usr/bin/env python3
"""Consume production EPD tracking and update MoveIt's live PlanningScene."""

from __future__ import annotations

import argparse
import json
import math
import time
import copy
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
from dynamic_object_planning_scene_bridge import (
    apply_and_verify, build_collision_object, build_remove_collision_object)
from epd_snapshot_adapter import normalize_detected_objects_snapshot


def load_task_contract(scene_package: str) -> dict[str, Any]:
    """Read the generated handoff; never use editor layout for task decisions."""
    import yaml
    path = Path(scene_package)
    if not path.is_dir():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory(scene_package))
    source = path / "cell_definition.yaml"
    if not source.exists():
        raise ValueError(f"generated task handoff missing: {source}; generate the scene first")
    cell = yaml.safe_load(source.read_text(encoding="utf-8"))
    task = cell.get("task") or {}
    if task.get("object_source") != "perception":
        raise ValueError("task.object_source must be perception")
    rule = task.get("object_filter") or {}
    if not str(rule.get("class_id", "")).strip():
        raise ValueError("task.object_filter.class_id must name a configured target class")
    max_age = float(rule.get("max_age_seconds", 2.0))
    if not math.isfinite(max_age) or max_age <= 0:
        raise ValueError("task.object_filter.max_age_seconds must be finite and positive")
    minimum = rule.get("min_confidence")
    if minimum is not None and (not math.isfinite(float(minimum)) or not 0 <= float(minimum) <= 1):
        raise ValueError("task.object_filter.min_confidence must be null or in [0,1]")
    zones = cell.get("environment", {}).get("task_zones", cell.get("task_zones", []))
    indexed = {str(zone["id"]): zone for zone in zones}
    for key in ("pick", "place"):
        ref = task.get(key, {}).get("source_ref" if key == "pick" else "target_ref")
        if ref not in indexed:
            raise ValueError(f"task.{key} semantic zone {ref!r} is missing")
    return {"cell": cell, "task": task, "zones": indexed,
            "filter": rule, "max_age_seconds": max_age, "source": str(source)}


def rotation_rpy(rpy):
    r, p, y = map(float, rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]


def rotation_quaternion(q):
    x, y, z, w = q
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def zone_contains_box(zone, box, tolerance=1e-6):
    """Require all oriented box corners inside the authored semantic free volume."""
    if zone.get("frame", "world") != box["frame_id"]:
        raise ValueError("semantic zone frame differs from planning frame; authored TF required")
    center = zone.get("pose_xyz") or zone.get("pose", {}).get("xyz")
    dimensions = zone.get("dimensions")
    if not center or not dimensions or len(dimensions) != 3 or any(float(v) <= 0 for v in dimensions):
        raise ValueError(f"semantic zone {zone.get('id')} has invalid volume")
    rz = rotation_rpy(zone.get("pose_rpy", [0, 0, 0]))
    ro = rotation_quaternion(box["pose"][3:])
    relative = [box["pose"][i] - float(center[i]) for i in range(3)]
    local = [sum(rz[j][i]*relative[j] for j in range(3)) for i in range(3)]
    extent = [sum(abs(sum(rz[k][i]*ro[k][j] for k in range(3))) * box["dimensions"][j]/2
                  for j in range(3)) for i in range(3)]
    return all(abs(local[i]) + extent[i] <= float(dimensions[i])/2 + tolerance for i in range(3))


def observation_gate(observed, timestamp_ns, now_ns, contract):
    rule = contract["filter"]
    if str(observed.get("label", "")).casefold() != str(rule["class_id"]).casefold():
        return "wrong target class"
    if not isinstance(timestamp_ns, (int, float)) or timestamp_ns <= 0:
        return "source timestamp unavailable"
    age = (now_ns - timestamp_ns) / 1e9
    if age < -0.25 or age > contract["max_age_seconds"]:
        return f"stale perception: observation age {age:.3f}s"
    confidence = observed.get("confidence")
    minimum = rule.get("min_confidence")
    if minimum is not None:
        if confidence is None:
            return "confidence unavailable; configure min_confidence null for sources without scores"
        if not math.isfinite(float(confidence)) or not float(minimum) <= float(confidence) <= 1:
            return "confidence below task threshold or invalid"
    if not observed.get("track_id"):
        return "stable runtime object association unavailable"
    return ""


def _observation_position(observed: dict[str, Any]) -> tuple[float, float, float] | None:
    pose = observed.get("pose") if isinstance(observed.get("pose"), dict) else {}
    values = pose.get("position") or observed.get("centroid")
    if isinstance(values, dict):
        values = [values.get("x"), values.get("y"), values.get("z")]
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        return None
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(value) for value in result) else None


def assign_stable_runtime_id(
    observed: dict[str, Any], previous: dict[str, dict[str, Any]],
    next_ids: dict[str, int], used_ids: set[str], association_distance_m: float = 0.25,
) -> str:
    """Prefer EPD tracking IDs and associate localization observations safely."""
    raw_track = str(observed.get("track_id") or "").strip()
    label = str(observed.get("label") or "unknown").strip().casefold() or "unknown"
    position = _observation_position(observed)
    if raw_track:
        stable_id = f"epd::{raw_track}"
    else:
        stable_id = ""
        if position is not None:
            candidates = []
            for candidate_id, record in previous.items():
                if record.get("label") != label or record.get("position") is None:
                    continue
                distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(position, record["position"])))
                if distance <= association_distance_m:
                    candidates.append((distance, candidate_id))
            if candidates:
                stable_id = min(candidates)[1]
        if not stable_id:
            index = next_ids.get(label, 0)
            while f"epd::{label}::{index}" in used_ids:
                index += 1
            next_ids[label] = index + 1
            stable_id = f"epd::{label}::{index}"
    if stable_id in used_ids:
        suffix = 1
        candidate = f"{stable_id}::{suffix}"
        while candidate in used_ids:
            suffix += 1
            candidate = f"{stable_id}::{suffix}"
        stable_id = candidate
    observed["object_id"] = stable_id
    observed["track_id"] = stable_id
    previous[stable_id] = {"label": label, "position": position}
    used_ids.add(stable_id)
    return stable_id


def initial_summary() -> dict[str, Any]:
    return {
        "schema": "workcell_perception_planning_scene_smoke/v1",
        "source": "live_epd", "motion_command_sent": False,
        "objects_received": 0, "objects_normalized": 0,
        "objects_applied": 0, "objects_updated": 0, "objects_removed": 0,
        "lost_ids_received": [], "removed_ids": [], "removal_noops": 0,
        "tf_failures": 0, "geometry_blocked": 0,
        "planning_scene_verified_ids": [], "duplicate_ids": [],
        "lost_removal_supported": True, "result": "RUNNING",
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


def should_remove(summary: dict[str, Any], applied_ids: set[str], object_id: str) -> bool:
    """Record a loss and return whether this process has an object to remove."""
    if object_id not in summary["lost_ids_received"]:
        summary["lost_ids_received"].append(object_id)
    if object_id in applied_ids:
        return True
    summary["removal_noops"] += 1
    return False


def same_physical_box(first: Any, second: Any,
                      position_tolerance: float = 0.08,
                      dimension_tolerance: float = 0.04) -> bool:
    """Return whether two collision boxes are the same tracked physical object."""
    if first is None or second is None or not first.primitives or not second.primitives:
        return False
    first_dims = list(first.primitives[0].dimensions)
    second_dims = list(second.primitives[0].dimensions)
    if len(first_dims) != 3 or len(second_dims) != 3:
        return False
    if any(abs(a - b) > dimension_tolerance for a, b in zip(first_dims, second_dims)):
        return False
    first_pose = first.primitive_poses[0] if first.primitive_poses else first.pose
    second_pose = second.primitive_poses[0] if second.primitive_poses else second.pose
    a = first_pose.position
    b = second_pose.position
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2
            <= position_tolerance ** 2)


def write_summary(path: Path, summary: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/easy_perception_deployment/epd_localize_output")
    parser.add_argument("--message-type", choices=("auto", "localization", "tracking"), default="auto")
    parser.add_argument("--normalized-topic", default="/workcell_studio/detected_objects")
    parser.add_argument("--scene-package", default="ur5_2f_test")
    parser.add_argument("--planning-frame", default="world")
    parser.add_argument("--camera-id", default="fixture_camera")
    parser.add_argument("--frame-fallback", default="camera_color_optical_frame")
    parser.add_argument("--service", default="/apply_planning_scene")
    parser.add_argument("--verify-service", default="/get_planning_scene")
    parser.add_argument("--timeout-seconds", type=float, default=5.0)
    parser.add_argument("--summary-output", type=Path, default=Path("/tmp/workcell_perception_summary.json"))
    parser.add_argument("--snapshot-output", type=Path, default=Path("/tmp/workcell_perception_snapshot.json"))
    args, ros_args = parser.parse_known_args(argv)
    contract = load_task_contract(args.scene_package)

    import rclpy
    from epd_msgs.msg import EPDObjectLocalization, EPDObjectTracking
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
    applied_ids: set[str] = set()
    latest_objects: dict[str, Any] = {}
    ownership = {"id": "", "object": None}
    seen_message = False
    latest_snapshot = {"objects": []}
    last_stamp_ns = 0
    scene_lock = threading.RLock()
    previous_associations: dict[str, dict[str, Any]] = {}
    next_association_ids: dict[str, int] = {}

    node.declare_parameter("owned_object_id", "")

    from rcl_interfaces.msg import SetParametersResult

    def ownership_parameters(parameters: list[Any]) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name != "owned_object_id":
                continue
            claimed_id = str(parameter.value).strip()
            if claimed_id and claimed_id not in applied_ids:
                return SetParametersResult(successful=False, reason="object is not currently eligible/applied")
            ownership["id"] = claimed_id
            ownership["object"] = latest_objects.get(claimed_id) if claimed_id else None
            summary["owned_object_id"] = claimed_id
            summary["ownership_active"] = bool(claimed_id)
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(ownership_parameters)

    def transform_pose(pose: Any, target: str) -> Any:
        try:
            return buffer.transform(pose, target, timeout=Duration(seconds=args.timeout_seconds))
        except TransformException as exc:
            raise RuntimeError(str(exc)) from exc

    work: queue.Queue[Any] = queue.Queue(maxsize=1)

    def process_message(message: Any) -> None:
        nonlocal seen_message, latest_snapshot, last_stamp_ns
        seen_message = True
        detected, warnings = convert_epd_message_to_detected_objects(
            message, args.topic, args.scene_package, args.frame_fallback)
        summary["objects_received"] += len(detected["objects"])
        profile = {"scene_id": args.scene_package, "perception": {"camera": {
            "camera_id": args.camera_id, "frame_id": detected["source"]["frame_id"]}}}
        normalized = normalize_detected_objects_snapshot(detected, profile)
        summary["objects_normalized"] += len(normalized["objects"])
        from perceived_object_grasp_plan import _collision_object_dict
        eligible = []
        rejected = {}
        stamp_ns = detected["source"].get("source_stamp_ns")
        last_stamp_ns = stamp_ns or 0
        used_runtime_ids: set[str] = set()
        for observed in normalized["objects"]:
            assign_stable_runtime_id(observed, previous_associations, next_association_ids, used_runtime_ids)
            reason = observation_gate(observed, stamp_ns, node.get_clock().now().nanoseconds, contract)
            if reason:
                rejected[str(observed["object_id"])] = reason
                continue
            object_id = str(observed["object_id"])
            built = build_collision_object(normalized, object_id, args.planning_frame, transform_pose)
            if built.status != "PASS":
                summary["tf_failures"] += int("TF unavailable" in built.reason)
                summary["geometry_blocked"] += int("geometry" in built.reason)
                node.get_logger().warning(f"{built.status} {object_id}: {built.reason}")
                continue
            box = _collision_object_dict(built.collision_object)
            pick_zone = contract["zones"][contract["task"]["pick"]["source_ref"]]
            try:
                if not zone_contains_box(pick_zone, box):
                    rejected[object_id] = "outside pick zone (complete perceived box must fit)"
                    continue
            except ValueError as exc:
                rejected[object_id] = str(exc)
                continue
            latest_objects[object_id] = built.collision_object
            if ownership["id"] == object_id and ownership["object"] is None:
                ownership["object"] = built.collision_object
            if ownership["id"] == object_id or (ownership["object"] is not None and same_physical_box(
                    built.collision_object, ownership["object"])):
                summary["ownership_suppressed"] = summary.get("ownership_suppressed", 0) + 1
                if object_id in applied_ids and object_id != ownership["id"]:
                    removal = build_remove_collision_object(object_id)
                    removed = apply_and_verify(
                        node, removal.collision_object, args.service,
                        args.verify_service, args.timeout_seconds)
                    if removed.status == "PASS":
                        applied_ids.remove(object_id)
                        summary["planning_scene_verified_ids"] = [
                            item for item in summary["planning_scene_verified_ids"]
                            if item != object_id]
                continue
            applied = apply_and_verify(
                node, built.collision_object, args.service, args.verify_service, args.timeout_seconds)
            if applied.status == "PASS":
                record_verified(summary, object_id)
                applied_ids.add(object_id)
                item = copy.deepcopy(observed)
                item["pose"] = {"frame_id": args.planning_frame, "position": box["pose"][:3],
                                "orientation_xyzw": box["pose"][3:]}
                eligible.append(item)
            else:
                node.get_logger().error(f"{applied.status} {object_id}: {applied.reason}")
        eligible_ids = {str(item["object_id"]) for item in eligible}
        removals = set(normalized.get("lost_object_ids", [])) | (applied_ids - eligible_ids)
        for raw_id in sorted(removals):
            object_id = str(raw_id)
            if object_id == ownership["id"]:
                continue
            if not should_remove(summary, applied_ids, object_id):
                node.get_logger().info(f"REMOVE no-op {object_id}: not applied by this process")
                continue
            removal = build_remove_collision_object(object_id)
            removed = apply_and_verify(
                node, removal.collision_object, args.service, args.verify_service, args.timeout_seconds)
            if removed.status == "PASS":
                applied_ids.remove(object_id)
                summary["objects_removed"] += 1
                if object_id not in summary["removed_ids"]:
                    summary["removed_ids"].append(object_id)
                summary["planning_scene_verified_ids"] = [
                    item for item in summary["planning_scene_verified_ids"] if item != object_id]
            else:
                node.get_logger().error(f"{removed.status} {object_id}: {removed.reason}")
        summary["lost_ids_received"].sort()
        summary["removed_ids"].sort()
        latest_snapshot = copy.deepcopy(normalized)
        latest_snapshot["objects"] = eligible
        latest_snapshot["frame_id"] = args.planning_frame
        latest_snapshot["timestamp"] = stamp_ns
        latest_snapshot["task_source"] = contract["source"]
        latest_snapshot["rejected_objects"] = rejected
        # Publish the normalized runtime contract after stable association and
        # planning-frame conversion. Raw EPD IDs would make localization-only
        # updates look like new collision objects.
        publisher.publish(String(data=json.dumps(latest_snapshot, sort_keys=True)))
        summary["eligible_ids"] = sorted(eligible_ids)
        summary["rejected_objects"] = rejected
        summary["result"] = "READY" if eligible else "BLOCKED"
        summary["blocker"] = "" if eligible else "no eligible live perception target"
        write_summary(args.snapshot_output, latest_snapshot)
        if warnings:
            summary["adapter_warnings"] = warnings
        write_summary(args.summary_output, summary)

    def worker() -> None:
        while rclpy.ok():
            try:
                message = work.get(timeout=0.1)
            except queue.Empty:
                if last_stamp_ns and node.get_clock().now().nanoseconds - last_stamp_ns > contract["max_age_seconds"]*1e9:
                    latest_snapshot["objects"] = []
                    latest_snapshot["rejected_objects"] = {"all": "stale perception"}
                    write_summary(args.snapshot_output, latest_snapshot)
                    for object_id in list(applied_ids):
                        if object_id != ownership["id"]:
                            removal = build_remove_collision_object(object_id)
                            result = apply_and_verify(node, removal.collision_object, args.service, args.verify_service, args.timeout_seconds)
                            if result.status == "PASS":
                                applied_ids.remove(object_id)
                    summary["result"] = "BLOCKED"
                    summary["blocker"] = "stale perception"
                    write_summary(args.summary_output, summary)
                continue
            try:
                process_message(message)
            except Exception as exc:
                summary["result"] = "BLOCKED"
                summary["blocker"] = str(exc)
                latest_snapshot["objects"] = []
                write_summary(args.snapshot_output, latest_snapshot)
                write_summary(args.summary_output, summary)
                node.get_logger().error(f"perception update blocked: {exc}")
            finally:
                work.task_done()

    def callback(message: Any) -> None:
        try:
            work.put_nowait(message)
        except queue.Full:
            try:
                work.get_nowait()
                work.task_done()
            except queue.Empty:
                pass
            work.put_nowait(message)

    message_type = args.message_type
    if message_type == "auto":
        message_type = "localization" if "localize" in args.topic.lower() else "tracking"
    subscription = node.create_subscription(  # noqa: F841
        EPDObjectLocalization if message_type == "localization" else EPDObjectTracking,
        args.topic, callback, qos_profile_sensor_data)
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
