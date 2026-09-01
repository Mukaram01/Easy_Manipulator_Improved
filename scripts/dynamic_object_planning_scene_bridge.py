#!/usr/bin/env python3
"""Transform one normalized observed object and apply it to MoveIt's PlanningScene."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time
from typing import Any, Callable, Mapping, Sequence

import yaml

from epd_snapshot_adapter import validate_normalized_snapshot


@dataclass
class CollisionObjectResult:
    status: str
    reason: str
    collision_object: Any | None = None


def _finite(values: Any, size: int) -> list[float] | None:
    if not isinstance(values, list) or len(values) != size:
        return None
    try:
        parsed = [float(value) for value in values]
    except (TypeError, ValueError):
        return None
    return parsed if all(math.isfinite(value) for value in parsed) else None


def build_collision_object(snapshot: Mapping[str, Any], object_id: str, planning_frame: str,
                           transform_pose: Callable[[Any, str], Any] | None = None) -> CollisionObjectResult:
    """Build a box CollisionObject without applying it or commanding robot motion."""
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive

    errors = validate_normalized_snapshot(dict(snapshot))
    if errors:
        return CollisionObjectResult("FAIL", "invalid normalized observation: " + "; ".join(errors))
    if not planning_frame.strip():
        return CollisionObjectResult("FAIL", "configured MoveIt planning frame is empty")
    observed = next((item for item in snapshot["objects"]
                     if str(item.get("object_id") or item.get("track_id")) == object_id), None)
    if observed is None:
        return CollisionObjectResult("FAIL", f"observed object {object_id!r} was not found")

    dimensions = observed.get("dimensions_xyz")
    if dimensions is None:
        return CollisionObjectResult("BLOCKED", "collision geometry unavailable: observed dimensions are missing")
    dimensions_xyz = _finite(dimensions, 3)
    if dimensions_xyz is None or any(value <= 0.0 for value in dimensions_xyz):
        return CollisionObjectResult("FAIL", "invalid collision geometry: dimensions must be finite and positive")
    if observed.get("shape") != "box":
        return CollisionObjectResult("BLOCKED", "collision geometry unavailable: only observed box geometry is supported")

    pose_stamped = PoseStamped()
    pose_data = observed.get("pose") if isinstance(observed.get("pose"), Mapping) else None
    if pose_data:
        position = _finite(pose_data.get("position"), 3)
        quaternion = _finite(pose_data.get("orientation_xyzw"), 4)
        source_frame = str(pose_data.get("frame_id") or snapshot.get("frame_id") or "")
    else:
        position = _finite(observed.get("centroid"), 3)
        quaternion = [0.0, 0.0, 0.0, 1.0]
        source_frame = str(snapshot.get("frame_id") or "")
    if position is None or quaternion is None:
        return CollisionObjectResult("FAIL", "invalid/non-finite observed pose or centroid")
    pose_stamped.header.frame_id = source_frame
    pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z = position
    (pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
     pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w) = quaternion

    if source_frame != planning_frame:
        if transform_pose is None:
            return CollisionObjectResult("BLOCKED", f"TF unavailable: cannot transform {source_frame!r} to planning frame {planning_frame!r}")
        try:
            pose_stamped = transform_pose(pose_stamped, planning_frame)
        except Exception as exc:
            return CollisionObjectResult("BLOCKED", f"TF unavailable: cannot transform {source_frame!r} to {planning_frame!r}: {exc}")
    pose_stamped.header.frame_id = planning_frame

    collision_object = CollisionObject()
    collision_object.header.frame_id = planning_frame
    collision_object.id = object_id
    collision_object.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = dimensions_xyz
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(pose_stamped.pose)
    return CollisionObjectResult("PASS", "dynamic object ready for PlanningScene ADD/update", collision_object)


def apply_and_verify(node: Any, collision_object: Any, service: str, verify_service: str,
                     timeout_seconds: float) -> CollisionObjectResult:
    """Apply ADD/update and verify the stable object ID is queryable in MoveIt."""
    import rclpy
    from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene

    client = node.create_client(ApplyPlanningScene, service)
    if not client.wait_for_service(timeout_sec=timeout_seconds):
        return CollisionObjectResult("FAIL", f"MoveIt application failure: service {service!r} unavailable")
    request = ApplyPlanningScene.Request()
    request.scene = PlanningScene()
    request.scene.is_diff = True
    request.scene.world.collision_objects.append(collision_object)
    future = client.call_async(request)
    if getattr(node, "_external_executor_spinning", False):
        deadline = time.monotonic() + timeout_seconds
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
    else:
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_seconds)
    if not future.done() or future.result() is None or future.result().success is not True:
        return CollisionObjectResult("FAIL", "MoveIt application failure: PlanningScene diff was rejected")

    verifier = node.create_client(GetPlanningScene, verify_service)
    if not verifier.wait_for_service(timeout_sec=timeout_seconds):
        return CollisionObjectResult("FAIL", f"MoveIt application failure: verification service {verify_service!r} unavailable")
    verify_request = GetPlanningScene.Request()
    verify_request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    verify_future = verifier.call_async(verify_request)
    if getattr(node, "_external_executor_spinning", False):
        deadline = time.monotonic() + timeout_seconds
        while not verify_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
    else:
        rclpy.spin_until_future_complete(node, verify_future, timeout_sec=timeout_seconds)
    if not verify_future.done() or verify_future.result() is None:
        return CollisionObjectResult("FAIL", "MoveIt application failure: PlanningScene verification timed out")
    matches = [obj for obj in verify_future.result().scene.world.collision_objects if obj.id == collision_object.id]
    if len(matches) != 1:
        return CollisionObjectResult("FAIL", f"MoveIt application failure: expected one object {collision_object.id!r}, found {len(matches)}")
    return CollisionObjectResult("PASS", "dynamic object inserted/updated and verified in PlanningScene", matches[0])


def _load_snapshot(path: Path) -> Mapping[str, Any]:
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(loaded, Mapping):
        raise ValueError("normalized snapshot root must be a mapping")
    return loaded


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--object-id", required=True)
    parser.add_argument("--planning-frame", required=True)
    parser.add_argument("--service", default="/apply_planning_scene")
    parser.add_argument("--verify-service", default="/get_planning_scene")
    parser.add_argument("--timeout-seconds", type=float, default=5.0)
    parser.add_argument("--validate-only", action="store_true")
    args, ros_args = parser.parse_known_args(argv)

    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformException, TransformListener
    import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped TF support

    rclpy.init(args=ros_args)
    node = Node("dynamic_object_planning_scene_bridge")
    buffer = Buffer()
    listener = TransformListener(buffer, node)  # noqa: F841

    def transform_pose(pose: Any, target_frame: str) -> Any:
        deadline = time.monotonic() + args.timeout_seconds
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                return buffer.transform(pose, target_frame, timeout=Duration(seconds=0.0))
            except TransformException as exc:
                last_error = exc
                rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
        raise RuntimeError(last_error or "transform timed out")

    try:
        result = build_collision_object(_load_snapshot(args.snapshot), args.object_id, args.planning_frame, transform_pose)
        if result.status != "PASS" or args.validate_only:
            print(f"{result.status}: {result.reason}")
            return 0 if result.status == "PASS" else 1
        applied = apply_and_verify(node, result.collision_object, args.service, args.verify_service, args.timeout_seconds)
        print(f"{applied.status}: {applied.reason}")
        return 0 if applied.status == "PASS" else 1
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
