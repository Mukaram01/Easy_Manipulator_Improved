#!/usr/bin/env python3
"""Apply a reviewed Workcell Studio collision manifest to MoveIt's PlanningScene."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

import yaml

from generate_moveit_collision_manifest import SCHEMA, validate_manifest


def parse_runtime_arguments(argv: Sequence[str], remove_ros_args) -> tuple[argparse.Namespace, list[str]]:
    ros_argv = list(argv)
    application_argv = remove_ros_args(args=ros_argv)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--service", default="/apply_planning_scene")
    parser.add_argument("--verify-service", default="/get_planning_scene")
    parser.add_argument("--timeout-seconds", type=float, default=45.0)
    parser.add_argument("--validate-only", action="store_true")
    return parser.parse_args(application_argv[1:]), ros_argv


def load_manifest(path: Path) -> Mapping[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot read planning-scene manifest {path}: {exc}") from exc
    if not isinstance(data, Mapping):
        raise ValueError("planning-scene manifest root must be a mapping")
    errors = validate_manifest(data)
    if errors:
        raise ValueError("invalid planning-scene manifest: " + "; ".join(errors))
    if data.get("schema_version") != SCHEMA:
        raise ValueError(f"unsupported planning-scene manifest schema: {data.get('schema_version')!r}")
    return data


def build_planning_scene(manifest: Mapping[str, Any]):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from shape_msgs.msg import SolidPrimitive

    scene = PlanningScene()
    scene.is_diff = True
    for record in manifest["objects"]:
        collision_object = CollisionObject()
        collision_object.header.frame_id = str(record["frame_id"])
        collision_object.id = str(record["id"])
        collision_object.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in record["collision_geometry"]["dimensions_m"]]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [float(value) for value in record["pose"]["xyz"]]
        quaternion = [float(value) for value in record["pose"]["quaternion_xyzw"]]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        scene.world.collision_objects.append(collision_object)
    return scene


def apply_manifest(manifest: Mapping[str, Any], service: str, verify_service: str, timeout_seconds: float,
                   ros_argv: Sequence[str]) -> None:
    import rclpy
    from moveit_msgs.srv import ApplyPlanningScene
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene
    from rclpy.node import Node

    rclpy.init(args=list(ros_argv))
    node = Node("workcell_studio_planning_scene_loader")
    client = node.create_client(ApplyPlanningScene, service)
    try:
        if not client.wait_for_service(timeout_sec=max(0.1, timeout_seconds)):
            raise RuntimeError(f"MoveIt service {service!r} was not available within {timeout_seconds:g}s")
        request = ApplyPlanningScene.Request()
        request.scene = build_planning_scene(manifest)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.1, timeout_seconds))
        if not future.done():
            raise RuntimeError(f"MoveIt service {service!r} did not answer within {timeout_seconds:g}s")
        response = future.result()
        if response is None or response.success is not True:
            raise RuntimeError("MoveIt rejected the authored collision-object PlanningScene diff")
        verifier = node.create_client(GetPlanningScene, verify_service)
        if not verifier.wait_for_service(timeout_sec=max(0.1, timeout_seconds)):
            raise RuntimeError(f"MoveIt verification service {verify_service!r} was not available")
        verify_request = GetPlanningScene.Request()
        verify_request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        verify_future = verifier.call_async(verify_request)
        rclpy.spin_until_future_complete(node, verify_future, timeout_sec=max(0.1, timeout_seconds))
        if not verify_future.done() or verify_future.result() is None:
            raise RuntimeError("MoveIt did not return PlanningScene world geometry for verification")
        expected_ids = {str(record["id"]) for record in manifest["objects"]}
        actual_ids = {obj.id for obj in verify_future.result().scene.world.collision_objects}
        missing_ids = sorted(expected_ids - actual_ids)
        if missing_ids:
            raise RuntimeError(f"MoveIt PlanningScene verification is missing collision objects: {missing_ids}")
        node.get_logger().info(
            f"Applied and verified {len(expected_ids)} authored collision objects; MoveIt is planning truth"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    from rclpy.utilities import remove_ros_args

    args, ros_argv = parse_runtime_arguments(sys.argv, remove_ros_args)
    try:
        manifest = load_manifest(args.manifest)
        if args.validate_only:
            print(f"PASS: {len(manifest['objects'])} collision objects")
            return 0
        apply_manifest(manifest, args.service, args.verify_service, args.timeout_seconds, ros_argv)
        return 0
    except (ValueError, RuntimeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
