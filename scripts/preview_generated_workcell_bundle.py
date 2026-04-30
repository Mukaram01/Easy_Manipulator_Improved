#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import sys

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
from validate_detected_objects import _load_yaml_or_json

SCHEMA_VERSION = "generated_workcell_visual_preview/v1"


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise SystemExit(f"FAIL: missing required file: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _load_yaml(path: Path, required: bool = True) -> dict[str, Any]:
    if not path.exists():
        if required:
            raise SystemExit(f"FAIL: missing required file: {path}")
        return {}
    doc, _, _ = _load_yaml_or_json(path)
    return doc


def _pose_from(item: dict[str, Any]) -> dict[str, float]:
    pose = item.get("pose") or {}
    if isinstance(pose, dict) and isinstance(pose.get("xyz"), list) and len(pose.get("xyz")) == 3:
        xyz = pose.get("xyz")
        p = {"x": xyz[0], "y": xyz[1], "z": xyz[2]}
    else:
        p = pose.get("position") if isinstance(pose, dict) else {}
        p = p or item.get("position") or {}
    q = pose.get("orientation") if isinstance(pose, dict) else {}
    q = q or item.get("orientation") or {}
    if isinstance(p, list) and len(p) == 3:
        p = {"x": p[0], "y": p[1], "z": p[2]}
    if not isinstance(p, dict):
        p = {}
    if not isinstance(q, dict):
        q = {}
    return {
        "x": float(p.get("x", 0.0)), "y": float(p.get("y", 0.0)), "z": float(p.get("z", 0.0)),
        "qx": float(q.get("x", 0.0)), "qy": float(q.get("y", 0.0)), "qz": float(q.get("z", 0.0)), "qw": float(q.get("w", 1.0)),
    }




def _dims(raw: Any, defaults: tuple[float, float, float]) -> dict[str, float]:
    if isinstance(raw, dict):
        return {"x": float(raw.get("x", defaults[0])), "y": float(raw.get("y", defaults[1])), "z": float(raw.get("z", defaults[2]))}
    if isinstance(raw, list) and len(raw) == 3:
        return {"x": float(raw[0]), "y": float(raw[1]), "z": float(raw[2])}
    return {"x": defaults[0], "y": defaults[1], "z": defaults[2]}

def build_marker_specs(summary: dict[str, Any], env: dict[str, Any], dest: dict[str, Any], detected: dict[str, Any], selected_object: str | None, selected_destination: str | None) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    planning_frame = summary.get("planning_frame", "world")
    markers: list[dict[str, Any]] = []

    markers.append({"ns": "frame", "type": "sphere", "id": 1, "frame_id": planning_frame, "pose": {"x": 0, "y": 0, "z": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1}, "scale": {"x": 0.04, "y": 0.04, "z": 0.04}, "color": {"r": 1, "g": 1, "b": 1, "a": 1}, "label": planning_frame})

    env_objects = env.get("objects", env.get("environment_objects", [])) or []
    for i, o in enumerate(env_objects, start=10):
        scale = _dims(o.get("dimensions"), (0.2, 0.2, 0.1))
        markers.append({"ns": "environment", "type": o.get("primitive", "cube"), "id": i, "frame_id": planning_frame, "pose": _pose_from(o), "scale": scale, "color": {"r": 0.3, "g": 0.6, "b": 0.9, "a": 0.5}, "label": o.get("id", f"env_{i}")})

    destinations = dest.get("destinations", []) or []
    for i, d in enumerate(destinations, start=100):
        markers.append({"ns": "destination", "type": "cylinder", "id": i, "frame_id": planning_frame, "pose": _pose_from(d), "scale": {"x": 0.10, "y": 0.10, "z": 0.02}, "color": {"r": 0.2, "g": 0.9, "b": 0.3, "a": 0.8}, "label": d.get("id", d.get("name", f"dest_{i}"))})

    detected_list = detected.get("objects", detected.get("detected_objects", [])) or []
    for i, o in enumerate(detected_list, start=200):
        scale = _dims(o.get("dimensions"), (0.04, 0.04, 0.08))
        markers.append({"ns": "detected", "type": "cube", "id": i, "frame_id": planning_frame, "pose": _pose_from(o), "scale": scale, "color": {"r": 0.95, "g": 0.7, "b": 0.2, "a": 0.8}, "label": o.get("object_id", o.get("id", f"obj_{i}"))})

    selected_obj = next((o for o in detected_list if (o.get("object_id") or o.get("id")) == selected_object), None)
    selected_dest = next((d for d in destinations if (d.get("id") or d.get("name")) == selected_destination), None)
    task_preview = {"selected_object": selected_object, "selected_destination": selected_destination, "pick_pose": _pose_from(selected_obj) if selected_obj else {}, "release_pose": _pose_from(selected_dest) if selected_dest else {}}
    if selected_obj:
        markers.append({"ns": "task", "type": "arrow", "id": 300, "frame_id": planning_frame, "pose": _pose_from(selected_obj), "scale": {"x": 0.15, "y": 0.02, "z": 0.02}, "color": {"r": 1.0, "g": 0.1, "b": 0.1, "a": 1.0}, "label": "pick"})
    if selected_dest:
        markers.append({"ns": "task", "type": "arrow", "id": 301, "frame_id": planning_frame, "pose": _pose_from(selected_dest), "scale": {"x": 0.15, "y": 0.02, "z": 0.02}, "color": {"r": 0.1, "g": 1.0, "b": 0.1, "a": 1.0}, "label": "release"})

    return markers, task_preview


def _publish_markers(marker_specs: list[dict[str, Any]], topic: str, frame_id: str) -> None:
    import rclpy
    from rclpy.node import Node
    from visualization_msgs.msg import Marker, MarkerArray

    class PreviewNode(Node):
        def __init__(self) -> None:
            super().__init__("generated_workcell_preview")
            self.pub = self.create_publisher(MarkerArray, topic, 10)

        def publish_once(self) -> None:
            arr = MarkerArray()
            for spec in marker_specs:
                m = Marker()
                m.header.frame_id = spec.get("frame_id", frame_id)
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = spec["ns"]
                m.id = int(spec["id"])
                m.action = Marker.ADD
                m.type = {"arrow": Marker.ARROW, "cube": Marker.CUBE, "sphere": Marker.SPHERE, "cylinder": Marker.CYLINDER}.get(spec["type"], Marker.CUBE)
                p = spec["pose"]
                m.pose.position.x, m.pose.position.y, m.pose.position.z = p["x"], p["y"], p["z"]
                m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = p["qx"], p["qy"], p["qz"], p["qw"]
                s = spec["scale"]
                m.scale.x, m.scale.y, m.scale.z = s["x"], s["y"], s["z"]
                c = spec["color"]
                m.color.r, m.color.g, m.color.b, m.color.a = c["r"], c["g"], c["b"], c["a"]
                arr.markers.append(m)
            self.pub.publish(arr)

    rclpy.init()
    node = PreviewNode()
    for _ in range(5):
        node.publish_once()
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.15)
    node.destroy_node()
    rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Visual preview for generated workcell bundles")
    p.add_argument("--workcell", type=Path, required=True)
    p.add_argument("--json", action="store_true")
    p.add_argument("--publish-markers", action="store_true")
    p.add_argument("--marker-topic", default="/generated_workcell/markers")
    p.add_argument("--show-task", action="store_true")
    p.add_argument("--selected-object")
    p.add_argument("--selected-destination")
    args = p.parse_args(argv)

    gen = args.workcell / "generated"
    summary = _load_json(gen / "generated_workcell_summary.json")
    env = _load_yaml(gen / "generated_environment_objects.yaml", required=True)
    dest = _load_yaml(gen / "generated_destinations.yaml", required=True)
    detected = _load_yaml(gen / "generated_detected_objects_example.yaml", required=False)

    markers, task = build_marker_specs(summary, env, dest, detected, args.selected_object if args.show_task else None, args.selected_destination if args.show_task else None)
    payload = {
        "schema_version": SCHEMA_VERSION,
        "workcell": str(args.workcell),
        "planning_frame": summary.get("planning_frame", "world"),
        "marker_topic": args.marker_topic,
        "objects_visualized": len(env.get("objects", env.get("environment_objects", [])) or []),
        "destinations_visualized": len(dest.get("destinations", []) or []),
        "detected_objects_visualized": len(detected.get("objects", detected.get("detected_objects", [])) or []),
        "task_preview": task,
        "warnings": [],
        "blockers": [],
        "marker_specs": markers,
    }

    out = gen / "visual_preview_summary.json"
    out.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if args.publish_markers:
        try:
            _publish_markers(markers, args.marker_topic, payload["planning_frame"])
        except Exception as exc:
            payload["warnings"].append(f"ROS marker publishing unavailable: {exc}")
            out.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
