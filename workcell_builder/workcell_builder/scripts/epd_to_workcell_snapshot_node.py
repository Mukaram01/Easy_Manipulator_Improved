#!/usr/bin/env python3
from __future__ import annotations
import json
import time
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:
    yaml = None

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

SCHEMA = "workcell_perception_snapshot/v1"
STATES = {"WAITING", "READY", "STALE", "FAILED", "DISABLED"}
MODES = {"disabled", "replay", "live"}


def _as_float(v: Any, default: float | None = None):
    try:
        return float(v)
    except Exception:
        return default


def _timestamp_value(value: Any) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value or "").strip()
    if not text:
        raise ValueError("timestamp is required")
    try:
        return float(text)
    except Exception:
        pass
    from datetime import datetime, timezone
    iso = text.replace("Z", "+00:00")
    return datetime.fromisoformat(iso).astimezone(timezone.utc).timestamp()


def _load_config(path: str | Path | None) -> dict[str, Any]:
    if not path:
        return {}
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"perception adapter config not found: {p}")
    text = p.read_text(encoding="utf-8")
    if yaml is not None:
        data = yaml.safe_load(text) or {}
    else:
        data = json.loads(text) if text.strip() else {}
    if not isinstance(data, dict):
        raise ValueError(f"perception adapter config must be a mapping: {p}")
    return data


def _normalize_mode(value: Any, status: str = "") -> str:
    raw = str(value or "").strip().lower()
    aliases = {"off": "disabled", "none": "disabled", "not_applicable": "disabled", "replayed_snapshot": "replay", "epd_snapshot": "replay", "live_epd": "live"}
    if not raw:
        raw = "disabled" if status == "NOT_APPLICABLE" else "live"
    mode = aliases.get(raw, raw)
    if mode not in MODES:
        raise ValueError(f"unsupported perception source mode: {value}")
    return mode


def validate_snapshot(snapshot: dict[str, Any], *, scene_id: str = "", camera_id: str = "", previous_timestamp: float | None = None) -> list[str]:
    errors: list[str] = []
    if not isinstance(snapshot, dict):
        return ["snapshot must be a mapping"]
    if snapshot.get("schema_version") != SCHEMA:
        errors.append(f"schema_version must be {SCHEMA}")
    if scene_id and snapshot.get("scene_id") != scene_id:
        errors.append(f"scene_id mismatch: expected {scene_id}, got {snapshot.get('scene_id')}")
    if camera_id and snapshot.get("camera_id") != camera_id:
        errors.append(f"camera_id mismatch: expected {camera_id}, got {snapshot.get('camera_id')}")
    for key in ("scene_id", "camera_id", "timestamp", "frame_id"):
        if not snapshot.get(key):
            errors.append(f"{key} is required")
    try:
        ts = _timestamp_value(snapshot.get("timestamp"))
        if previous_timestamp is not None and ts < previous_timestamp:
            errors.append("timestamp moved backward")
    except Exception as exc:
        errors.append(f"timestamp is invalid: {exc}")
    objects = snapshot.get("objects")
    if not isinstance(objects, list):
        errors.append("objects must be a list")
        return errors
    seen: set[str] = set()
    for idx, obj in enumerate(objects):
        if not isinstance(obj, dict):
            errors.append(f"objects[{idx}] must be a mapping")
            continue
        oid = obj.get("object_id") or obj.get("track_id")
        if not oid:
            errors.append(f"objects[{idx}] requires object_id or track_id")
        elif str(oid) in seen:
            errors.append(f"duplicate object id: {oid}")
        else:
            seen.add(str(oid))
        if not obj.get("label"):
            errors.append(f"objects[{idx}].label is required")
        conf = _as_float(obj.get("confidence"), None)
        if conf is None or conf < 0.0 or conf > 1.0:
            errors.append(f"objects[{idx}].confidence must be in [0, 1]")
        if not isinstance(obj.get("pose"), dict) and "centroid" not in obj:
            errors.append(f"objects[{idx}] requires pose or centroid")
    return errors


def _normalize_detection(obj: dict, default_zone: str, min_confidence: float) -> dict | None:
    confidence = _as_float(obj.get("confidence", obj.get("score", 0.0)), 0.0)
    if confidence < min_confidence:
        return None
    est_cam = obj.get("estimated_xyz_camera") or obj.get("centroid") or obj.get("position")
    est_world = obj.get("estimated_xyz_world") or obj.get("world_position")
    return {"id": str(obj.get("id", obj.get("object_id", obj.get("track_id", f"det_{int(time.time() * 1000)}")))), "class_label": str(obj.get("class_label", obj.get("label", "unknown"))), "confidence": confidence, "center_px": obj.get("center_px"), "bbox_px": obj.get("bbox_px"), "estimated_xyz_camera": est_cam, "estimated_xyz_world": est_world, "zone_hint": obj.get("zone_hint", default_zone), "tracking_id": obj.get("tracking_id", obj.get("track_id")), "segmented_cloud_available": bool(obj.get("segmented_cloud_available", obj.get("has_cloud", False)))}


def build_snapshot_from_epd_payload(payload: dict, camera: str, camera_frame: str, default_zone: str, min_confidence: float = 0.0, scene_id: str = "") -> dict:
    objs = payload.get("objects", payload.get("detections", []))
    detections = [_normalize_detection(o, default_zone, min_confidence) for o in objs if isinstance(o, dict)]
    detections = [d for d in detections if d is not None]
    normalized_objects = []
    for d in detections:
        position = d.get("estimated_xyz_world") or d.get("estimated_xyz_camera")
        item = {"object_id": d.get("id"), "track_id": d.get("tracking_id"), "label": d.get("class_label"), "confidence": d.get("confidence"), "attributes": {"zone_hint": d.get("zone_hint"), "segmented_cloud_available": d.get("segmented_cloud_available")}}
        if position is not None:
            item["pose"] = {"frame_id": camera_frame, "position": position, "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]}
        else:
            item["centroid"] = [0.0, 0.0, 0.0]
        normalized_objects.append(item)
    now = time.time()
    return {"schema_version": SCHEMA, "source": "epd_live", "runtime_mode": "live", "scene_id": scene_id, "camera_id": camera, "camera": camera, "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)), "timestamp_sec": now, "frame_id": camera_frame, "camera_frame": camera_frame, "objects": normalized_objects, "detections": detections}


class PerceptionSourceAdapter:
    def __init__(self, config: dict[str, Any] | None = None):
        self.configure(config or {})

    @classmethod
    def from_config_file(cls, path: str | Path):
        return cls(_load_config(path))

    def configure(self, cfg: dict[str, Any]):
        self.config = cfg
        self.mode = _normalize_mode(cfg.get("mode") or cfg.get("source_mode") or cfg.get("perception"), cfg.get("status", ""))
        self.scene_id = str(cfg.get("scene_id") or "")
        camera = cfg.get("camera") if isinstance(cfg.get("camera"), dict) else {}
        epd = cfg.get("epd_input") if isinstance(cfg.get("epd_input"), dict) else {}
        replay = cfg.get("replay") if isinstance(cfg.get("replay"), dict) else {}
        self.camera_id = str(cfg.get("camera_id") or camera.get("camera_id") or camera.get("id") or "")
        self.camera_frame = str(cfg.get("frame_id") or camera.get("frame_id") or "camera_color_optical_frame")
        self.topic = str(cfg.get("topic") or epd.get("topic") or "/easy_perception_deployment/epd_localize_output")
        self.output_topic = str(cfg.get("output_snapshot_topic") or "/workcell_studio/epd_detection_snapshot_json")
        self.status_topic = str(cfg.get("output_status_topic") or "/workcell_studio/epd_connector_status")
        self.rate_hz = float(replay.get("rate_hz", cfg.get("replay_rate_hz", 1.0)) or 1.0)
        self.single_step = bool(replay.get("single_step", cfg.get("single_step", False)))
        self.loop = bool(replay.get("loop", cfg.get("loop", False)))
        self.replay_path = replay.get("path") or cfg.get("replay_path") or cfg.get("snapshot_path")
        self.freshness_timeout_s = float(cfg.get("freshness_timeout_s", cfg.get("stale_timeout_s", 2.0)) or 2.0)
        self.min_confidence = float(cfg.get("confidence_threshold", 0.0) or 0.0)
        self.default_zone_hint = str(cfg.get("default_zone_hint", "detection_zone_1"))
        self.state = "DISABLED" if self.mode == "disabled" else "WAITING"
        self.reason = "perception disabled" if self.mode == "disabled" else "waiting for snapshot"
        self.last_timestamp_value: float | None = None
        self.last_wall_time = 0.0
        self.last_snapshot: dict[str, Any] | None = None
        self.replay_snapshots: list[dict[str, Any]] = []
        self.replay_index = 0
        if self.mode == "replay":
            self.replay_snapshots = self._load_replay()

    def _load_replay(self) -> list[dict[str, Any]]:
        if not self.replay_path:
            self.state, self.reason = "FAILED", "replay.path is required"
            return []
        p = Path(self.replay_path)
        try:
            if p.suffix.lower() == ".jsonl":
                snaps = [json.loads(line) for line in p.read_text(encoding="utf-8").splitlines() if line.strip()]
            else:
                data = json.loads(p.read_text(encoding="utf-8"))
                snaps = data if isinstance(data, list) else data.get("snapshots", [data])
            prev = None
            for snap in snaps:
                errors = validate_snapshot(snap, scene_id=self.scene_id, camera_id=self.camera_id, previous_timestamp=prev)
                if errors:
                    raise ValueError("; ".join(errors))
                prev = _timestamp_value(snap.get("timestamp"))
            return list(snaps)
        except Exception as exc:
            self.state, self.reason = "FAILED", f"malformed replay {p}: {exc}"
            return []

    def clear(self, reason: str):
        self.last_snapshot = None; self.last_timestamp_value = None; self.last_wall_time = 0.0; self.replay_index = 0; self.reason = reason

    def accept_live_payload(self, payload: dict[str, Any]) -> dict[str, Any] | None:
        if self.mode != "live":
            return None
        snap = payload if payload.get("schema_version") == SCHEMA else build_snapshot_from_epd_payload(payload, self.camera_id, self.camera_frame, self.default_zone_hint, self.min_confidence, self.scene_id)
        errors = validate_snapshot(snap, scene_id=self.scene_id, camera_id=self.camera_id, previous_timestamp=self.last_timestamp_value)
        if errors:
            self.state, self.reason = "FAILED", "; ".join(errors)
            return None
        self.last_snapshot = snap; self.last_timestamp_value = _timestamp_value(snap["timestamp"]); self.last_wall_time = time.time(); self.state = "READY"; self.reason = "live snapshot ready"
        return snap

    def next_replay_snapshot(self) -> dict[str, Any] | None:
        if self.mode != "replay" or self.state == "FAILED":
            return None
        if not self.replay_snapshots:
            self.state, self.reason = "FAILED", "replay contains no snapshots"; return None
        if self.replay_index >= len(self.replay_snapshots):
            if not self.loop:
                self.state, self.reason = "STALE", "replay exhausted"; return None
            self.replay_index = 0
        snap = self.replay_snapshots[self.replay_index]
        self.replay_index += 1
        self.last_snapshot = snap; self.last_timestamp_value = _timestamp_value(snap["timestamp"]); self.last_wall_time = time.time(); self.state = "READY"; self.reason = "replay snapshot ready"
        return snap

    def refresh_staleness(self, now: float | None = None):
        if self.mode == "live" and self.last_wall_time and (now or time.time()) - self.last_wall_time > self.freshness_timeout_s:
            self.state, self.reason = "STALE", "freshness timeout exceeded"

    def status(self) -> dict[str, Any]:
        self.refresh_staleness()
        snap = self.last_snapshot or {}
        return {"mode": self.mode, "state": self.state, "scene_id": self.scene_id, "camera_id": self.camera_id, "last_timestamp": snap.get("timestamp"), "object_count": len(snap.get("objects", [])) if isinstance(snap.get("objects"), list) else 0, "reason": self.reason, "epd_connected": self.state == "READY", "last_msg_age_s": 0.0 if self.last_wall_time else 1e9, "last_error": self.reason if self.state == "FAILED" else ""}


class EpdToWorkcellSnapshotNode(Node):
    def __init__(self):
        super().__init__("epd_to_workcell_snapshot_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("localization_topic", "/easy_perception_deployment/epd_localize_output")
        self.declare_parameter("output_snapshot_topic", "/workcell_studio/epd_detection_snapshot_json")
        self.declare_parameter("output_status_topic", "/workcell_studio/epd_connector_status")
        self.declare_parameter("scene_id", "")
        self.declare_parameter("camera_name", "realsense_d435i_1")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("default_zone_hint", "detection_zone_1")
        self.declare_parameter("min_confidence", 0.0)
        self.declare_parameter("publish_period_s", 0.1)
        self.declare_parameter("stale_timeout_s", 2.0)
        cfg = _load_config(self.get_parameter("config_path").value) if self.get_parameter("config_path").value else {}
        cfg.setdefault("scene_id", self.get_parameter("scene_id").value); cfg.setdefault("camera_id", self.get_parameter("camera_name").value); cfg.setdefault("frame_id", self.get_parameter("camera_frame").value); cfg.setdefault("topic", self.get_parameter("localization_topic").value); cfg.setdefault("stale_timeout_s", self.get_parameter("stale_timeout_s").value); cfg.setdefault("confidence_threshold", self.get_parameter("min_confidence").value)
        self.adapter = PerceptionSourceAdapter(cfg)
        self.snapshot_pub = self.create_publisher(String, self.adapter.output_topic, 10)
        self.status_pub = self.create_publisher(String, self.adapter.status_topic, 10)
        self.sub = None if self.adapter.mode != "live" else self.create_subscription(String, self.adapter.topic, self.on_epd_json, 20)
        period = 1.0 / max(self.adapter.rate_hz, 1e-6) if self.adapter.mode == "replay" else float(self.get_parameter("publish_period_s").value)
        self.timer = self.create_timer(period, self.on_timer)

    def on_epd_json(self, msg: String):
        try:
            snap = self.adapter.accept_live_payload(json.loads(msg.data))
            if snap:
                self.snapshot_pub.publish(String(data=json.dumps(snap)))
        except Exception as exc:
            self.adapter.state, self.adapter.reason = "FAILED", str(exc)

    def on_timer(self):
        if self.adapter.mode == "replay" and not self.adapter.single_step:
            snap = self.adapter.next_replay_snapshot()
            if snap:
                self.snapshot_pub.publish(String(data=json.dumps(snap)))
        self.status_pub.publish(String(data=json.dumps(self.adapter.status())))


def main():
    if rclpy is None:
        raise RuntimeError("rclpy is required to run epd_to_workcell_snapshot_node")
    rclpy.init(); node = EpdToWorkcellSnapshotNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
