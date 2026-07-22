#!/usr/bin/env python3
from __future__ import annotations
import json
import time
from typing import Any

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except Exception:
    rclpy = None
    class Node:
        pass
    class String:
        def __init__(self, data=""):
            self.data = data


def _as_float(v: Any, default: float | None = None):
    try:
        return float(v)
    except Exception:
        return default


def _normalize_detection(obj: dict, default_zone: str, min_confidence: float) -> dict | None:
    confidence = _as_float(obj.get("confidence", obj.get("score", 0.0)), 0.0)
    if confidence < min_confidence:
        return None
    est_cam = obj.get("estimated_xyz_camera") or obj.get("centroid") or obj.get("position")
    est_world = obj.get("estimated_xyz_world") or obj.get("world_position")
    return {
        "id": str(obj.get("id", obj.get("object_id", f"det_{int(time.time() * 1000)}"))),
        "class_label": str(obj.get("class_label", obj.get("label", "unknown"))),
        "confidence": confidence,
        "center_px": obj.get("center_px"),
        "bbox_px": obj.get("bbox_px"),
        "estimated_xyz_camera": est_cam,
        "estimated_xyz_world": est_world,
        "zone_hint": obj.get("zone_hint", default_zone),
        "tracking_id": obj.get("tracking_id", obj.get("track_id")),
        "segmented_cloud_available": bool(obj.get("segmented_cloud_available", obj.get("has_cloud", False))),
    }


def build_snapshot_from_epd_payload(payload: dict, camera: str, camera_frame: str, default_zone: str, min_confidence: float = 0.0, scene_id: str = "") -> dict:
    objs = payload.get("objects", payload.get("detections", []))
    detections = []
    for o in objs:
        if isinstance(o, dict):
            d = _normalize_detection(o, default_zone, min_confidence)
            if d is not None:
                detections.append(d)
    normalized_objects = []
    for d in detections:
        position = d.get("estimated_xyz_world") or d.get("estimated_xyz_camera")
        item = {
            "object_id": d.get("id"),
            "track_id": d.get("tracking_id"),
            "label": d.get("class_label"),
            "confidence": d.get("confidence"),
            "attributes": {"zone_hint": d.get("zone_hint"), "segmented_cloud_available": d.get("segmented_cloud_available")},
        }
        if position is not None:
            item["pose"] = {"frame_id": camera_frame, "position": position, "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]}
        normalized_objects.append(item)
    return {
        "schema_version": "workcell_perception_snapshot/v1",
        "source": "epd_live",
        "runtime_mode": "live_adapter_metadata_only",
        "scene_id": scene_id,
        "camera_id": camera,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "frame_id": camera_frame,
        "objects": normalized_objects,
    }


class EpdToWorkcellSnapshotNode(Node):
    def __init__(self):
        super().__init__("epd_to_workcell_snapshot_node")
        self.declare_parameter("localization_topic", "/easy_perception_deployment/epd_localize_output")
        self.declare_parameter("tracking_topic", "/easy_perception_deployment/epd_tracking_output")
        self.declare_parameter("use_tracking", False)
        self.declare_parameter("output_snapshot_topic", "/workcell_studio/epd_detection_snapshot_json")
        self.declare_parameter("output_status_topic", "/workcell_studio/epd_connector_status")
        self.declare_parameter("scene_id", "")
        self.declare_parameter("camera_name", "realsense_d435i_1")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("default_zone_hint", "detection_zone_1")
        self.declare_parameter("min_confidence", 0.0)
        self.declare_parameter("publish_period_s", 0.1)
        self.declare_parameter("stale_timeout_s", 2.0)

        self.last_payload = {"detections": []}
        self.last_msg_time = 0.0
        self.last_error = ""

        self.snapshot_pub = self.create_publisher(String, self.get_parameter("output_snapshot_topic").value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter("output_status_topic").value, 10)
        topic = self.get_parameter("tracking_topic").value if self.get_parameter("use_tracking").value else self.get_parameter("localization_topic").value
        self.sub = self.create_subscription(String, topic, self.on_epd_json, 20)
        self.timer = self.create_timer(float(self.get_parameter("publish_period_s").value), self.on_timer)

    def on_epd_json(self, msg: String):
        try:
            self.last_payload = json.loads(msg.data)
            self.last_msg_time = time.time()
            self.last_error = ""
        except Exception as exc:
            self.last_error = str(exc)

    def on_timer(self):
        snap = build_snapshot_from_epd_payload(
            self.last_payload,
            self.get_parameter("camera_name").value,
            self.get_parameter("camera_frame").value,
            self.get_parameter("default_zone_hint").value,
            float(self.get_parameter("min_confidence").value),
            self.get_parameter("scene_id").value,
        )
        self.snapshot_pub.publish(String(data=json.dumps(snap)))
        age = 1e9 if self.last_msg_time <= 0 else max(0.0, time.time() - self.last_msg_time)
        status = {
            "epd_connected": age <= float(self.get_parameter("stale_timeout_s").value),
            "source_topic": self.sub.topic_name,
            "last_msg_age_s": round(age, 3),
            "detections": len(snap.get("objects", [])),
            "last_error": self.last_error,
            "camera": self.get_parameter("camera_name").value,
            "output_topic": self.get_parameter("output_snapshot_topic").value,
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main():
    if rclpy is None:
        raise RuntimeError("rclpy is required to run epd_to_workcell_snapshot_node")
    rclpy.init()
    node = EpdToWorkcellSnapshotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
