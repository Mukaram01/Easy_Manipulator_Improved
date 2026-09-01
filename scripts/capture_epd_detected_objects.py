#!/usr/bin/env python3
"""Capture live EPD localization output into detected_objects/v1 YAML/JSON."""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from validate_detected_objects import validate_detected_objects


def _now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _get(obj: Any, key: str, default: Any = None) -> Any:
    if isinstance(obj, dict):
        return obj.get(key, default)
    return getattr(obj, key, default)


def _to_float(value: Any) -> float | None:
    try:
        return float(value)
    except Exception:
        return None


def _message_objects(msg: Any) -> list[Any]:
    for key in ("objects", "localized_objects", "tracked_objects"):
        value = _get(msg, key)
        if isinstance(value, list) and value:
            return value
    if _get(msg, "name") is not None and _get(msg, "centroid") is not None:
        return [msg]
    return []


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _rpy_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _object_to_detected(raw: Any, index: int, frame_id: str) -> tuple[dict[str, Any] | None, list[str]]:
    warnings: list[str] = []
    centroid = _get(raw, "centroid")
    x = _to_float(_get(centroid, "x"))
    y = _to_float(_get(centroid, "y"))
    z = _to_float(_get(centroid, "z"))
    if x is None or y is None or z is None:
        return None, [f"Object {index} missing centroid fields x/y/z"]

    name = _get(raw, "name") or _get(raw, "label") or _get(raw, "class_id") or f"object_{index:03d}"
    class_id = _get(raw, "class_id") or _get(raw, "label") or _get(raw, "name") or "unknown"

    dims = None
    bbox = _get(raw, "bounding_box")
    if bbox is not None:
        bx = _to_float(_get(bbox, "x"))
        by = _to_float(_get(bbox, "y"))
        bz = _to_float(_get(bbox, "z"))
        if bx and by and bz:
            dims = {"x": bx, "y": by, "z": bz}

    if dims is None:
        lx = _to_float(_get(raw, "length"))
        wy = _to_float(_get(raw, "breadth")) or _to_float(_get(raw, "width"))
        hz = _to_float(_get(raw, "height"))
        if lx and wy and hz:
            dims = {"x": lx, "y": wy, "z": hz}

    message_pose = _get(raw, "pose")
    orientation = _get(message_pose, "orientation")
    quaternion = [
        _to_float(_get(orientation, "x")), _to_float(_get(orientation, "y")),
        _to_float(_get(orientation, "z")), _to_float(_get(orientation, "w")),
    ]
    if any(value is None for value in quaternion):
        quaternion = [0.0, 0.0, 0.0, 1.0]
    raw_pose = {
        "frame_id": frame_id, "xyz": [x, y, z],
        "rpy": list(_rpy_from_quaternion(*quaternion)),
        "orientation_xyzw": quaternion,
    }
    obj = {
        "object_id": f"obj_{index:03d}",
        "name": str(name),
        "class_id": str(class_id),
        "confidence": _to_float(_get(raw, "confidence")),
        "pose": raw_pose.copy(),
        "raw_pose": raw_pose,
        "centroid": {"x": x, "y": y, "z": z},
        "dimensions": dims,
        "shape": {"type": str(_get(raw, "shape") or "box")},
        "attributes": {
            "colour": str(_get(raw, "colour") or "unknown"),
            "shape": str(_get(raw, "shape") or "box"),
            "material": str(_get(raw, "material") or "unknown"),
        },
        "raw": {"source_message_type": str(type(raw)).replace("<class '", "").replace("'>", "")},
    }
    return obj, warnings


def _normalize_pose_with_tf(
    obj: dict[str, Any],
    target_frame: str,
    tf_timeout_sec: float,
    transform_pose: Callable[[str, str, list[float], list[float], float], tuple[list[float], list[float], str]],
) -> tuple[str, str]:
    pose = obj.get("pose") if isinstance(obj.get("pose"), dict) else {}
    src_frame = str(pose.get("frame_id") or "").strip()
    xyz = pose.get("xyz")
    rpy = pose.get("rpy")
    if not src_frame or not isinstance(xyz, list) or len(xyz) < 3:
        return "FAIL", "Object pose missing frame_id/xyz for TF normalization"
    if not isinstance(rpy, list) or len(rpy) < 3:
        rpy = [0.0, 0.0, 0.0]
    transformed_xyz, transformed_rpy, msg = transform_pose(src_frame, target_frame, xyz, rpy, tf_timeout_sec)
    obj["pose"] = {"frame_id": target_frame, "xyz": transformed_xyz, "rpy": transformed_rpy}
    return "PASS", msg


def convert_epd_message_to_detected_objects(
    msg: Any,
    topic: str,
    scene_package: str,
    frame_fallback: str,
) -> tuple[dict[str, Any], list[str]]:
    warnings: list[str] = []
    header = _get(msg, "header")
    message_frame = str(_get(header, "frame_id") or "").strip()
    if not message_frame:
        message_frame = frame_fallback
        warnings.append(f"EPD frame_id missing; using fallback '{frame_fallback}'")

    objects = []
    stable_ids = list(_get(msg, "object_ids", []) or [])
    for idx, raw in enumerate(_message_objects(msg), start=1):
        obj, obj_warnings = _object_to_detected(raw, idx, message_frame)
        warnings.extend(obj_warnings)
        if obj is not None:
            if idx <= len(stable_ids):
                stable_id = str(stable_ids[idx - 1])
                obj["object_id"] = stable_id
                obj["tracking_id"] = stable_id
            if obj.get("confidence") is None:
                obj.pop("confidence", None)
                obj["attributes"]["confidence_available"] = False
            objects.append(obj)

    stamp = _get(header, "stamp")
    sec = _get(stamp, "sec")
    nanosec = _get(stamp, "nanosec")
    source_stamp_ns = None
    if sec is not None and nanosec is not None:
        source_stamp_ns = int(sec) * 1_000_000_000 + int(nanosec)

    payload = {
        "schema_version": "detected_objects/v1",
        "source": {
            "type": "epd_localization",
            "topic": topic,
            "scene_package": scene_package,
            "frame_id": message_frame,
            "captured_at": _now(),
            "source_stamp_ns": source_stamp_ns,
        },
        "objects": objects,
    }
    return payload, warnings


def _write_output(payload: dict[str, Any], output: Path, as_json: bool) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if as_json:
        output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    else:
        try:
            import yaml  # type: ignore

            output.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
        except Exception:
            output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def create_qos_profile(reliability: str, depth: int):
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # type: ignore

    chosen = "best_effort" if reliability == "auto" else reliability
    rel_policy = ReliabilityPolicy.BEST_EFFORT if chosen == "best_effort" else ReliabilityPolicy.RELIABLE
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=max(1, int(depth)),
        reliability=rel_policy,
        durability=DurabilityPolicy.VOLATILE,
    ), chosen


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/easy_perception_deployment/epd_localize_output")
    parser.add_argument("--output", type=Path, default=Path("/tmp/mvp1/live_detected_objects.yaml"))
    parser.add_argument("--scene-package", required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--min-objects", type=int, default=1)
    parser.add_argument("--frame-fallback", default="camera_depth_optical_frame")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--qos-reliability", choices=("auto", "best_effort", "reliable"), default="best_effort")
    parser.add_argument("--qos-depth", type=int, default=10)
    parser.add_argument("--target-frame", default="world")
    parser.add_argument("--tf-timeout", type=float, default=2.0)
    parser.add_argument("--require-transform", action="store_true", default=True)
    parser.add_argument("--allow-untransformed", action="store_true")
    args = parser.parse_args(argv)

    try:
        import rclpy  # type: ignore
        from rclpy.node import Node  # type: ignore
        from epd_msgs.msg import EPDObjectLocalization  # type: ignore

        msg_type = EPDObjectLocalization
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "error": f"ROS/EPD imports failed: {exc}"}, indent=2))
        return 2

    rclpy.init(args=None)
    latest_message: Any | None = None

    class CaptureNode(Node):
        def __init__(self) -> None:
            super().__init__("capture_epd_detected_objects")
            qos_profile, qos_selected = create_qos_profile(args.qos_reliability, args.qos_depth)
            self.qos_selected = qos_selected
            self.create_subscription(msg_type, args.topic, self._cb, qos_profile)

        def _cb(self, msg: Any) -> None:
            nonlocal latest_message
            latest_message = msg

    tf_ready = False
    tf_buffer = None
    node = CaptureNode()
    if args.capture_live if hasattr(args, "capture_live") else True:
        try:
            from tf2_ros import Buffer, TransformListener  # type: ignore
            tf_buffer = Buffer()
            _tf_listener = TransformListener(tf_buffer, node)
            tf_ready = True
        except Exception:
            tf_ready = False

    def _transform_pose(src_frame: str, target_frame: str, xyz: list[float], rpy: list[float], timeout_sec: float):
        if tf_buffer is None:
            raise RuntimeError("TF2 listener is unavailable")
        from geometry_msgs.msg import PoseStamped  # type: ignore
        from rclpy.duration import Duration  # type: ignore
        pose = PoseStamped()
        pose.header.frame_id = src_frame
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        qx, qy, qz, qw = _quaternion_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = qx, qy, qz, qw
        transformed = tf_buffer.transform(pose, target_frame, timeout=Duration(seconds=float(timeout_sec)))
        out_xyz = [float(transformed.pose.position.x), float(transformed.pose.position.y), float(transformed.pose.position.z)]
        out_rpy = list(_rpy_from_quaternion(
            float(transformed.pose.orientation.x),
            float(transformed.pose.orientation.y),
            float(transformed.pose.orientation.z),
            float(transformed.pose.orientation.w),
        ))
        return out_xyz, out_rpy, f"Pose transformed {src_frame} -> {target_frame}"
    try:
        end_ns = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
        payload = None
        warnings: list[str] = []
        while rclpy.ok() and node.get_clock().now().nanoseconds <= end_ns:
            rclpy.spin_once(node, timeout_sec=0.1)
            if latest_message is None:
                continue
            payload, warnings = convert_epd_message_to_detected_objects(
                latest_message, args.topic, args.scene_package, args.frame_fallback
            )
            if len(payload.get("objects", [])) >= max(1, args.min_objects):
                if args.once:
                    break
                break

        if payload is None or len(payload.get("objects", [])) < max(1, args.min_objects):
            print(json.dumps({"status": "FAIL", "error": f"No detections meeting min-objects={args.min_objects} on {args.topic}", "qos_reliability": node.qos_selected, "qos_depth": max(1, int(args.qos_depth))}, indent=2))
            return 1

        transform_status = "PASS"
        transform_message = "No objects required normalization"
        source_frame = payload.get("source", {}).get("frame_id")
        if payload.get("objects"):
            statuses: list[str] = []
            messages: list[str] = []
            for obj in payload.get("objects", []):
                try:
                    status, message = _normalize_pose_with_tf(obj, args.target_frame, args.tf_timeout, _transform_pose)
                except Exception as exc:
                    status, message = "FAIL", f"TF transform failed: {exc}"
                statuses.append(status)
                messages.append(message)
            transform_status = "PASS" if all(s == "PASS" for s in statuses) else "FAIL"
            transform_message = "; ".join(messages)
            if transform_status == "FAIL" and args.allow_untransformed:
                transform_status = "WARN"
                transform_message += "; continuing because --allow-untransformed"
            if transform_status == "FAIL" and (args.require_transform and not args.allow_untransformed):
                print(json.dumps({"status": "FAIL", "error": transform_message}, indent=2))
                return 1
        payload["source"]["transform"] = {
            "requested_target_frame": args.target_frame,
            "status": transform_status,
            "source_frame": source_frame,
            "target_frame": args.target_frame,
            "message": transform_message,
            "tf_listener_ready": tf_ready,
        }

        validation = validate_detected_objects(payload, strict=False, allow_generate_ids=True)
        warnings.extend(validation.warnings)

        if args.dry_run:
            print(json.dumps({"status": "FAIL" if transform_status == "FAIL" else ("WARN" if (warnings or transform_status == "WARN") else "PASS"), "dry_run": True, "output": str(args.output), "payload": payload, "warnings": warnings, "qos_reliability": node.qos_selected, "qos_depth": max(1, int(args.qos_depth))}, indent=2))
            return 0

        _write_output(payload, args.output, args.json)
        print(json.dumps({"status": "FAIL" if transform_status == "FAIL" else ("WARN" if (warnings or transform_status == "WARN") else "PASS"), "output": str(args.output), "objects": len(payload.get("objects", [])), "warnings": warnings, "qos_reliability": node.qos_selected, "qos_depth": max(1, int(args.qos_depth))}, indent=2))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
