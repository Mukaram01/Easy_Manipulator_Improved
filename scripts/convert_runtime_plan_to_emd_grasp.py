#!/usr/bin/env python3
"""Convert runtime_execution_plan/v1 payloads into emd_grasp_bridge_payload/v1 (offline-first)."""

from __future__ import annotations

import argparse
import json
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import validate_cell_definition as cell_yaml

DEFAULT_OBJECT_DIMS = [0.04, 0.04, 0.08]


def _load_yaml_or_json(path: Path) -> dict[str, Any]:
    if path.suffix.lower() == ".json":
        loaded = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(loaded, dict):
            raise ValueError("Runtime plan JSON root must be an object.")
        return loaded
    loaded, _, _ = cell_yaml.load_yaml(path)
    if not isinstance(loaded, dict):
        raise ValueError("Runtime plan YAML root must be a mapping.")
    return loaded


def _as_xyz_rpy(pose: dict[str, Any]) -> tuple[list[float] | None, list[float] | None, str | None]:
    frame_id = pose.get("frame_id") if isinstance(pose.get("frame_id"), str) and pose.get("frame_id").strip() else None
    xyz = pose.get("xyz") if isinstance(pose.get("xyz"), list) else None
    rpy = pose.get("rpy") if isinstance(pose.get("rpy"), list) else None
    if xyz is None and isinstance(pose.get("position"), dict):
        p = pose["position"]
        if all(k in p for k in ("x", "y", "z")):
            xyz = [float(p["x"]), float(p["y"]), float(p["z"])]
    if rpy is None and isinstance(pose.get("orientation"), dict):
        o = pose["orientation"]
        if all(k in o for k in ("roll", "pitch", "yaw")):
            rpy = [float(o["roll"]), float(o["pitch"]), float(o["yaw"])]
    if xyz is not None and len(xyz) == 3:
        xyz = [float(x) for x in xyz]
    else:
        xyz = None
    if rpy is not None and len(rpy) == 3:
        rpy = [float(x) for x in rpy]
    else:
        rpy = None
    return xyz, rpy, frame_id


def _normalize_dimensions(raw: Any) -> list[float] | None:
    if isinstance(raw, list) and raw:
        return [float(x) for x in raw]
    if isinstance(raw, dict):
        if all(k in raw for k in ("x", "y", "z")):
            return [float(raw["x"]), float(raw["y"]), float(raw["z"])]
        if all(k in raw for k in ("radius", "height")):
            return [float(raw["height"]), float(raw["radius"])]
    return None


def _shape_for_object(shape_hint: str, dimensions: list[float], warnings: list[str], object_id: str) -> tuple[str, list[float]]:
    hint = shape_hint.strip().lower()
    if hint in {"cylinder", "can", "tube"}:
        if len(dimensions) >= 2:
            return "CYLINDER", dimensions[:2]
        warnings.append(f"Object '{object_id}' requested cylinder shape but dimensions were incomplete; falling back to BOX.")
        return "BOX", dimensions[:3]
    if hint in {"sphere", "ball"}:
        if len(dimensions) == 1:
            return "SPHERE", [dimensions[0]]
        if len(dimensions) >= 3 and abs(dimensions[0] - dimensions[1]) < 1e-6 and abs(dimensions[1] - dimensions[2]) < 1e-6:
            return "SPHERE", [dimensions[0] / 2.0]
        warnings.append(f"Object '{object_id}' requested sphere shape but radius/diameter was unclear; falling back to BOX.")
        return "BOX", dimensions[:3]
    return "BOX", dimensions[:3]


def _synth_grasp_pose(xyz: list[float], rpy: list[float]) -> tuple[list[float], list[float]]:
    return [xyz[0], xyz[1], xyz[2] + 0.05], [rpy[0], rpy[1], rpy[2]]


def _is_rejected(obj: dict[str, Any], step: dict[str, Any]) -> bool:
    destination_id = (
        step.get("routing", {}).get("destination_id")
        if isinstance(step.get("routing"), dict)
        else None
    )
    class_id = str(obj.get("class_id", "")).strip().lower()
    if isinstance(destination_id, str) and ("reject" in destination_id or destination_id.startswith("unknown")):
        return True
    return class_id in {"unknown", "reject", "rejected"}


def _build_bridge_payload(plan: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    warnings: list[str] = []
    errors: list[str] = []

    if plan.get("schema_version") != "runtime_execution_plan/v1":
        errors.append("Input schema_version must be runtime_execution_plan/v1.")

    steps = plan.get("steps") if isinstance(plan.get("steps"), list) else []
    planning_frame = str(plan.get("metadata", {}).get("planning_frame") or "world")
    targets: list[dict[str, Any]] = []
    skipped = 0

    for step in steps:
        if not isinstance(step, dict):
            continue
        task_name = str(step.get("task", "")).lower()
        if not task_name.startswith("pick"):
            continue

        obj = step.get("object") if isinstance(step.get("object"), dict) else {}
        object_id = str(obj.get("id") or step.get("step_id") or f"object_{len(targets)+1:03d}")

        if _is_rejected(obj, step):
            skipped += 1
            warnings.append(f"Skipping rejected/unknown object '{object_id}'.")
            continue

        pose = obj.get("pose") if isinstance(obj.get("pose"), dict) else {}
        xyz, rpy, pose_frame = _as_xyz_rpy(pose)
        frame_id = str(obj.get("frame_id") or pose_frame or args.frame_id)
        if xyz is None or rpy is None:
            errors.append(f"Object '{object_id}' is missing a complete pose (xyz+rpy).")
            continue

        dims = _normalize_dimensions(obj.get("dimensions"))
        if dims is None:
            dims = [float(x) for x in args.default_object_dimensions]
            warnings.append(f"Object '{object_id}' missing dimensions; using fallback {dims}.")

        shape_hint = str(obj.get("shape") or obj.get("class_id") or "box")
        shape_type, shape_dims = _shape_for_object(shape_hint, dims, warnings, object_id)
        if len(shape_dims) < 3 and shape_type == "BOX":
            shape_dims = [float(x) for x in args.default_object_dimensions]

        preferred_ee = obj.get("preferred_end_effector")
        ee_id = str(preferred_ee or args.default_ee_id)

        grasp_pose = obj.get("grasp_pose") if isinstance(obj.get("grasp_pose"), dict) else {}
        gx, gr, gframe = _as_xyz_rpy(grasp_pose)
        if gx is None or gr is None:
            gx, gr = _synth_grasp_pose(xyz, rpy)
            warnings.append(f"Object '{object_id}' missing grasp pose; synthesized conservative grasp pose.")
        grasp_frame = str(gframe or frame_id)

        routing = step.get("routing") if isinstance(step.get("routing"), dict) else {}
        destination_id = routing.get("destination_id") if isinstance(routing.get("destination_id"), str) else None
        destination = routing.get("destination") if isinstance(routing.get("destination"), dict) else {}
        destination_resolved = (
            routing.get("destination_resolved")
            if isinstance(routing.get("destination_resolved"), dict)
            else {}
        )

        destination_name = (
            destination_resolved.get("destination_name")
            if isinstance(destination_resolved.get("destination_name"), str)
            else destination.get("label")
        )
        destination_label = (
            destination_resolved.get("destination_label")
            if isinstance(destination_resolved.get("destination_label"), str)
            else destination.get("label")
        )

        resolved_pose = destination_resolved.get("pose") if isinstance(destination_resolved.get("pose"), dict) else {}
        resolved_frame = (
            destination_resolved.get("frame_id")
            if isinstance(destination_resolved.get("frame_id"), str)
            else None
        )
        resolved_xyz = resolved_pose.get("xyz") if isinstance(resolved_pose.get("xyz"), list) else None
        resolved_quat = (
            resolved_pose.get("quaternion_xyzw")
            if isinstance(resolved_pose.get("quaternion_xyzw"), list)
            else None
        )

        destination_pose = {
            "frame_id": resolved_frame or destination.get("frame") or planning_frame,
            "xyz": resolved_xyz if resolved_xyz is not None else destination.get("pose_xyz") if isinstance(destination.get("pose_xyz"), list) else None,
            "rpy": resolved_pose.get("rpy") if isinstance(resolved_pose.get("rpy"), list) else destination.get("pose_rpy") if isinstance(destination.get("pose_rpy"), list) else None,
            "quaternion_xyzw": resolved_quat,
        }
        destination_safety_warnings = (
            destination_resolved.get("safety_warnings")
            if isinstance(destination_resolved.get("safety_warnings"), list)
            else []
        )
        for item in destination_safety_warnings:
            if isinstance(item, str):
                warnings.append(f"Object '{object_id}': {item}")

        if destination_pose["xyz"] is not None and destination_pose["rpy"] is not None:
            warnings.append(
                f"Object '{object_id}': using explicit destination release pose for destination '{destination_id}'."
            )
        else:
            warnings.append(
                f"Object '{object_id}': destination has no pose; falling back to release_x_offset/release_use_grasp_z."
            )

        if destination_pose["frame_id"] != planning_frame:
            warnings.append(
                f"Object '{object_id}': destination pose frame '{destination_pose['frame_id']}' does not match planning frame '{planning_frame}'."
            )

        target = {
            "object_id": object_id,
            "target_type": str(obj.get("class_id") or obj.get("shape") or "box"),
            "target_pose": {"frame_id": frame_id, "xyz": xyz, "rpy": rpy},
            "target_shape": {"type": shape_type, "dimensions": shape_dims},
            "grasp_methods": [
                {
                    "ee_id": ee_id,
                    "grasp_poses": [{"frame_id": grasp_frame, "xyz": gx, "rpy": gr}],
                    "grasp_ranks": [1.0],
                }
            ],
            "destination_id": destination_id,
            "destination_name": destination_name,
            "destination_label": destination_label,
            "destination_pose": destination_pose,
            "destination_approach": destination_resolved.get("approach"),
            "destination_retreat": destination_resolved.get("retreat"),
            "destination_safety_warnings": destination_safety_warnings,
            "notes": [
                "Destination pose preserved in bridge payload.",
                "Current emd_msgs/GraspTarget has no explicit release/place pose field; runtime uses release_x_offset/release_use_grasp_z fallback.",
                "TODO(adapter-boundary): extend runtime interface to consume destination_pose without breaking GraspRequest compatibility.",
            ],
        }
        targets.append(target)

    status = "PASS"
    if errors:
        status = "FAIL"
    elif warnings:
        status = "WARN"
    if args.strict and warnings:
        status = "FAIL"
        errors.append("Strict mode treats warnings as failures.")

    task_id = str(plan.get("task_recipe", {}).get("id") if isinstance(plan.get("task_recipe"), dict) else "")
    if not task_id:
        task_id = f"bridge_{uuid.uuid4().hex[:8]}"

    payload = {
        "schema_version": "emd_grasp_bridge_payload/v1",
        "source_runtime_plan": str(args.runtime_plan),
        "mode": args.mode,
        "status": status,
        "summary": {
            "pick_steps_seen": len([s for s in steps if isinstance(s, dict) and str(s.get("task", "")).lower().startswith("pick")]),
            "grasp_targets": len(targets),
            "skipped_objects": skipped,
            "generated_at_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        },
        "ros_interface": {
            "service_name": args.service_name,
            "topic_name": args.topic_name,
            "message_type": "emd_msgs/msg/GraspTask",
            "service_type": "emd_msgs/srv/GraspRequest",
            "selected": args.ros_interface,
        },
        "grasp_task": {"task_id": task_id, "grasp_targets": targets},
        "runtime_release_adapter_boundary": {
            "explicit_release_pose_supported_in_runtime": False,
            "active_release_strategy": "release_x_offset/release_use_grasp_z",
            "why": "emd_msgs/msg/GraspTarget and emd_msgs/srv/GraspRequest do not carry release destination pose fields.",
            "todo": "Add non-breaking runtime interface extension to consume destination_pose when available.",
        },
        "warnings": warnings,
        "errors": errors,
    }
    return payload


def _to_ros_and_send(payload: dict[str, Any], args: argparse.Namespace) -> tuple[bool, str]:
    try:
        import rclpy
        from rclpy.node import Node
        from emd_msgs.msg import GraspTask, GraspTarget, GraspMethod
        from emd_msgs.srv import GraspRequest
        from geometry_msgs.msg import PoseStamped
        from shape_msgs.msg import SolidPrimitive
    except Exception as exc:  # pragma: no cover - env dependent
        return False, f"ROS mode requested but ROS Python imports failed: {exc}"

    def make_pose(data: dict[str, Any]) -> Any:
        msg = PoseStamped()
        msg.header.frame_id = str(data.get("frame_id") or args.frame_id)
        xyz = data.get("xyz") if isinstance(data.get("xyz"), list) and len(data["xyz"]) == 3 else [0.0, 0.0, 0.0]
        rpy = data.get("rpy") if isinstance(data.get("rpy"), list) and len(data["rpy"]) == 3 else [0.0, 0.0, 0.0]
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        # Conservative fallback: pass roll/pitch/yaw as xyzw identity with yaw ignored for bridge preview.
        msg.pose.orientation.w = 1.0
        _ = rpy
        return msg

    rclpy.init(args=None)
    node = Node("runtime_plan_to_emd_bridge")
    try:
        task = GraspTask()
        task.task_id = str(payload.get("grasp_task", {}).get("task_id", ""))
        for target_data in payload.get("grasp_task", {}).get("grasp_targets", []):
            target = GraspTarget()
            target.target_type = str(target_data.get("target_type", "box"))
            target.target_pose = make_pose(target_data.get("target_pose", {}))
            primitive = SolidPrimitive()
            shape_type = str(target_data.get("target_shape", {}).get("type", "BOX")).upper()
            if shape_type == "CYLINDER":
                primitive.type = SolidPrimitive.CYLINDER
            elif shape_type == "SPHERE":
                primitive.type = SolidPrimitive.SPHERE
            else:
                primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(x) for x in target_data.get("target_shape", {}).get("dimensions", [])]
            target.target_shape = primitive

            for method_data in target_data.get("grasp_methods", []):
                method = GraspMethod()
                method.ee_id = str(method_data.get("ee_id", args.default_ee_id))
                method.grasp_poses = [make_pose(pose_data) for pose_data in method_data.get("grasp_poses", [])]
                method.grasp_ranks = [float(x) for x in method_data.get("grasp_ranks", [1.0])]
                target.grasp_methods.append(method)
            task.grasp_targets.append(target)

        if args.ros_interface == "topic":
            publisher = node.create_publisher(GraspTask, args.topic_name, 10)
            rclpy.spin_once(node, timeout_sec=0.1)
            publisher.publish(task)
            return True, f"Published grasp task to topic '{args.topic_name}'."

        client = node.create_client(GraspRequest, args.service_name)
        if not client.wait_for_service(timeout_sec=3.0):
            return False, f"Service '{args.service_name}' not available before timeout."
        req = GraspRequest.Request()
        req.grasp_targets = task.grasp_targets
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return False, f"Service '{args.service_name}' call timed out or failed."
        res = fut.result()
        return bool(res.success), str(res.message)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _emit(payload: dict[str, Any], output: Path | None, as_json: bool) -> None:
    rendered = json.dumps(payload, indent=2, sort_keys=True)
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")
    if as_json or output is None:
        print(rendered)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-plan", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--dry-run", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--mode", choices=["offline", "ros"], default="offline")
    parser.add_argument("--ros-interface", choices=["service", "topic"], default="service")
    parser.add_argument("--service-name", default="grasp_requests")
    parser.add_argument("--topic-name", default="grasp_tasks")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--default-ee-id", default="robotiq_2f")
    parser.add_argument("--default-object-dimensions", nargs="+", type=float, default=DEFAULT_OBJECT_DIMS)
    args = parser.parse_args(argv)

    try:
        plan = _load_yaml_or_json(args.runtime_plan)
        payload = _build_bridge_payload(plan, args)

        if args.mode == "ros" and not args.dry_run:
            ok, message = _to_ros_and_send(payload, args)
            if not ok:
                payload["status"] = "FAIL"
                payload.setdefault("errors", []).append(message)
            else:
                payload.setdefault("warnings", []).append(message)
        elif args.mode == "ros":
            payload.setdefault("warnings", []).append(
                "ROS mode selected with dry-run enabled; ROS send skipped by design."
            )

        _emit(payload, args.output, args.json)

        status = str(payload.get("status", "FAIL"))
        return 1 if status == "FAIL" else 0
    except Exception as exc:
        failure = {
            "schema_version": "emd_grasp_bridge_payload/v1",
            "status": "FAIL",
            "error": str(exc),
        }
        print(json.dumps(failure, indent=2, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
