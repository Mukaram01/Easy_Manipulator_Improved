#!/usr/bin/env python3
"""Capture EPD localization/tracking ROS messages into detected_objects/v1 snapshots."""

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

import validate_cell_definition as cell_yaml


def _now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _float_or_none(value: Any) -> float | None:
    try:
        return float(value)
    except Exception:
        return None


def _derive_dimensions_from_segmented_pcl(segmented_pcl: Any) -> tuple[dict[str, float] | None, str | None]:
    points = []
    if isinstance(segmented_pcl, list):
        points = segmented_pcl
    elif hasattr(segmented_pcl, "points"):
        points = list(getattr(segmented_pcl, "points"))

    triples: list[tuple[float, float, float]] = []
    for p in points:
        if isinstance(p, dict):
            xyz = (_float_or_none(p.get("x")), _float_or_none(p.get("y")), _float_or_none(p.get("z")))
        else:
            xyz = (_float_or_none(getattr(p, "x", None)), _float_or_none(getattr(p, "y", None)), _float_or_none(getattr(p, "z", None)))
        if all(v is not None for v in xyz):
            triples.append((float(xyz[0]), float(xyz[1]), float(xyz[2])))

    if len(triples) < 2:
        return None, "segmented_pcl unavailable/insufficient for dimensions"

    xs = [t[0] for t in triples]
    ys = [t[1] for t in triples]
    zs = [t[2] for t in triples]
    dims = {"x": max(xs) - min(xs), "y": max(ys) - min(ys), "z": max(zs) - min(zs)}
    if any(v <= 0 for v in dims.values()):
        return None, "derived point-cloud dimensions were non-positive"
    return dims, None


def _extract_object(raw: Any, index: int, frame_id: str, strict: bool) -> tuple[dict[str, Any] | None, list[str]]:
    warnings: list[str] = []
    name = getattr(raw, "name", None) or getattr(raw, "class_id", None) or f"object_{index:03d}"
    class_id = getattr(raw, "class_id", None) or getattr(raw, "name", None) or "unknown"
    confidence = _float_or_none(getattr(raw, "confidence", None))

    centroid_raw = getattr(raw, "centroid", None)
    centroid = {
        "x": _float_or_none(getattr(centroid_raw, "x", None)),
        "y": _float_or_none(getattr(centroid_raw, "y", None)),
        "z": _float_or_none(getattr(centroid_raw, "z", None)),
    }

    if any(v is None for v in centroid.values()):
        warnings.append(f"Object {index} missing centroid values")
        return None, warnings

    pose = {
        "frame_id": str(getattr(raw, "frame_id", None) or frame_id),
        "xyz": [centroid["x"], centroid["y"], centroid["z"]],
        "rpy": [0.0, 0.0, 0.0],
    }

    dimensions = None
    bbox = getattr(raw, "bounding_box", None)
    if bbox is not None and all(hasattr(bbox, a) for a in ("x", "y", "z")):
        dimensions = {"x": float(bbox.x), "y": float(bbox.y), "z": float(bbox.z)}
    elif all(hasattr(raw, a) for a in ("length", "width", "height")):
        dimensions = {"x": float(raw.length), "y": float(raw.width), "z": float(raw.height)}
    else:
        dimensions, dim_warn = _derive_dimensions_from_segmented_pcl(getattr(raw, "segmented_pcl", None))
        if dim_warn:
            warnings.append(f"Object {index} {dim_warn}")

    if dimensions is None and strict:
        warnings.append(f"Object {index} missing dimensions in strict mode")
        return None, warnings

    obj = {
        "object_id": f"obj_{index:03d}",
        "name": str(name),
        "class_id": str(class_id),
        "confidence": confidence,
        "pose": pose,
        "centroid": centroid,
        "dimensions": dimensions,
        "shape": {"type": "box"},
        "attributes": {
            "colour": str(getattr(raw, "colour", "unknown")),
            "shape": str(getattr(raw, "shape", "box")),
            "material": str(getattr(raw, "material", "unknown")),
        },
        "raw": {
            "source_message_type": str(type(raw)).replace("<class '", "").replace("'>", ""),
        },
    }
    return obj, warnings


def _snapshot_from_message(msg: Any, topic: str, camera: str, default_frame_id: str, strict: bool) -> tuple[dict[str, Any], list[str]]:
    warnings: list[str] = []
    objects_raw = []
    for key in ("objects", "localized_objects", "tracked_objects"):
        val = getattr(msg, key, None)
        if isinstance(val, list) and val:
            objects_raw = val
            break

    if not objects_raw and all(hasattr(msg, k) for k in ("name", "centroid")):
        objects_raw = [msg]

    frame_id = default_frame_id
    header = getattr(msg, "header", None)
    if header is not None and hasattr(header, "frame_id") and str(header.frame_id).strip():
        frame_id = str(header.frame_id)

    objects: list[dict[str, Any]] = []
    for idx, raw_obj in enumerate(objects_raw, start=1):
        obj, obj_warnings = _extract_object(raw_obj, idx, frame_id, strict)
        warnings.extend(obj_warnings)
        if obj is not None:
            objects.append(obj)

    payload = {
        "schema_version": "detected_objects/v1",
        "source": {
            "type": "epd_localization",
            "topic": topic,
            "camera": camera,
            "frame_id": frame_id,
            "captured_at": _now(),
        },
        "objects": objects,
    }
    return payload, warnings


def _write_output(payload: dict[str, Any], output: Path, as_json: bool) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if as_json:
        output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return
    try:
        import yaml  # type: ignore

        output.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    except Exception:
        output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/easy_perception_deployment/epd_localize_output")
    parser.add_argument("--camera", default="intel_realsense_d435i")
    parser.add_argument("--frame-id", default="camera_depth_optical_frame")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--samples", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--output", type=Path, default=Path("reports/detected_objects/latest_detected_objects.yaml"))
    args = parser.parse_args(argv)

    # Lazy ROS imports for offline-safe unit tests.
    try:
        import rclpy  # type: ignore
        from rclpy.node import Node  # type: ignore

        msg_type = None
        try:
            from epd_msgs.msg import EPDObjectLocalization as msg_type  # type: ignore
        except Exception:
            from epd_msgs.msg import EPDObjectTracking as msg_type  # type: ignore
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "error": f"ROS import failed (expected in offline mode): {exc}"}, indent=2))
        return 2

    rclpy.init(args=None)
    messages: list[Any] = []

    class CaptureNode(Node):
        def __init__(self) -> None:
            super().__init__("capture_epd_detected_objects")
            self.create_subscription(msg_type, args.topic, self._cb, 10)

        def _cb(self, msg: Any) -> None:
            messages.append(msg)

    node = CaptureNode()
    try:
        end_time = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
        target = 1 if args.once else max(1, int(args.samples))
        while rclpy.ok() and len(messages) < target:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds > end_time:
                break

        if not messages:
            print(json.dumps({"status": "FAIL", "error": f"No messages received on {args.topic}"}, indent=2))
            return 1

        payload, warnings = _snapshot_from_message(messages[-1], args.topic, args.camera, args.frame_id, args.strict)
        _write_output(payload, args.output, as_json=args.json)
        report = {
            "status": "WARN" if warnings else "PASS",
            "output": str(args.output),
            "objects": len(payload.get("objects", [])),
            "warnings": warnings,
        }
        print(json.dumps(report, indent=2))
        return 0 if not (args.strict and warnings) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
