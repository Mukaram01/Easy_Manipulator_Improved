#!/usr/bin/env python3
"""Capture live EPD localization output into detected_objects/v1 YAML/JSON."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

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
        wy = _to_float(_get(raw, "width"))
        hz = _to_float(_get(raw, "height"))
        if lx and wy and hz:
            dims = {"x": lx, "y": wy, "z": hz}

    obj = {
        "object_id": f"obj_{index:03d}",
        "name": str(name),
        "class_id": str(class_id),
        "confidence": _to_float(_get(raw, "confidence")),
        "pose": {"frame_id": frame_id, "xyz": [x, y, z], "rpy": [0.0, 0.0, 0.0]},
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
    for idx, raw in enumerate(_message_objects(msg), start=1):
        obj, obj_warnings = _object_to_detected(raw, idx, message_frame)
        warnings.extend(obj_warnings)
        if obj is not None:
            objects.append(obj)

    payload = {
        "schema_version": "detected_objects/v1",
        "source": {
            "type": "epd_localization",
            "topic": topic,
            "scene_package": scene_package,
            "frame_id": message_frame,
            "captured_at": _now(),
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
            self.create_subscription(msg_type, args.topic, self._cb, 10)

        def _cb(self, msg: Any) -> None:
            nonlocal latest_message
            latest_message = msg

    node = CaptureNode()
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
            print(json.dumps({"status": "FAIL", "error": f"No detections meeting min-objects={args.min_objects} on {args.topic}"}, indent=2))
            return 1

        validation = validate_detected_objects(payload, strict=False, allow_generate_ids=True)
        warnings.extend(validation.warnings)

        if args.dry_run:
            print(json.dumps({"status": "WARN" if warnings else "PASS", "dry_run": True, "output": str(args.output), "payload": payload, "warnings": warnings}, indent=2))
            return 0

        _write_output(payload, args.output, args.json)
        print(json.dumps({"status": "WARN" if warnings else "PASS", "output": str(args.output), "objects": len(payload.get("objects", [])), "warnings": warnings}, indent=2))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
