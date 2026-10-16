#!/usr/bin/env python3
"""Mirror EPD per-detection readiness into a Workcell Studio status artifact.

Consumes ``/easy_perception_deployment/detection_readiness``
(``diagnostic_msgs/DiagnosticArray``, one status per detector proposal, EPD PR #234)
and atomically writes ``<scene>/preview/epd_detection_readiness.json`` for the
Builder Perception Status panel.

This is diagnostic evidence for the operator only. It never publishes, never feeds
grasp planning, and never authorises motion: planning keeps consuming EPD's
validated-pose outputs through the existing adapters and safety gates.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
import time
from pathlib import Path
from typing import Any

try:
    import rclpy
    import rclpy.executors
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from diagnostic_msgs.msg import DiagnosticArray
except Exception:
    rclpy = None

    class Node:
        pass

SCHEMA = "workcell_epd_detection_readiness/v1"
DEFAULT_TOPIC = "/easy_perception_deployment/detection_readiness"
STATUS_PREFIX = "easy_perception_deployment/detection/"
STATES = ("DETECTED", "LOCALISED", "POSE_VALID", "GRASP_CANDIDATE", "REJECTED")
POSITIONED = {"LOCALISED", "POSE_VALID", "GRASP_CANDIDATE"}
ORIENTED = {"POSE_VALID", "GRASP_CANDIDATE"}


def _float(text: Any) -> float | None:
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _vector(text: Any) -> list[float] | None:
    parts = str(text or "").split(",")
    if len(parts) != 3:
        return None
    values = [_float(p) for p in parts]
    return None if any(v is None for v in values) else values  # type: ignore[return-value]


def _parse_object(status: dict[str, Any], errors: list[str]) -> dict[str, Any] | None:
    name = str(status.get("name", ""))
    if not name.startswith(STATUS_PREFIX):
        return None  # not a per-detection status; ignore other diagnostics
    try:
        index = int(name[len(STATUS_PREFIX):])
    except ValueError:
        errors.append(f"malformed detection status name: {name}")
        return None
    v = {str(k): str(val) for k, val in (status.get("values") or {}).items()}
    state = v.get("state", "")
    if state not in STATES:
        errors.append(f"detection {index}: unknown state {state!r}")
        state = "UNKNOWN"
    position = None
    if v.get("position_valid") == "true":
        position = _vector(v.get("centroid_m"))
        if state not in POSITIONED or position is None:
            errors.append(f"detection {index}: position_valid inconsistent with state {state}")
            position = None
    elif state in POSITIONED:
        errors.append(f"detection {index}: state {state} without a valid position")
    orientation_valid = False
    if v.get("orientation_valid") == "true":
        axis = _vector(v.get("axis"))
        norm = math.sqrt(sum(a * a for a in axis)) if axis else 0.0
        if state in ORIENTED and axis is not None and abs(norm - 1.0) < 1e-2:
            orientation_valid = True
        else:
            errors.append(f"detection {index}: orientation_valid inconsistent with state {state}")
    reasons = v.get("reasons", "")
    return {
        "index": index,
        "observation_id": v.get("observation_id", ""),
        "class": v.get("class", ""),
        "confidence": _float(v.get("confidence")),
        "track_id": v.get("track_id") or None,  # EPD tracking mode only; never inferred here
        "state": state,
        "position_m": position,
        "orientation_valid": orientation_valid,
        "reasons": [] if reasons in ("", "none") else reasons.split(","),
        "note": v.get("note", ""),
        "roi": v.get("roi", ""),
        # Readiness is never a planning input, even for GRASP_CANDIDATE.
        "planning_input": False,
    }


class ReadinessMonitor:
    """Frame-by-frame readiness state; never associates objects across frames itself."""

    def __init__(self, stale_timeout_s: float = 2.0, expected_frame_id: str = "",
                 topic: str = DEFAULT_TOPIC):
        self.stale_timeout_s = float(stale_timeout_s)
        self.expected_frame_id = expected_frame_id
        self.topic = topic
        self.last_stamp_ns: int | None = None
        self.last_observation_id: int | None = None
        self.last_wall_time: float | None = None
        self.frame_id = ""
        self.camera_id = ""
        self.objects: list[dict[str, Any]] = []
        self.contract_errors: list[str] = []
        self.epd_session = 0
        self.counters = {k: 0 for k in ("accepted", "duplicate", "out_of_order", "invalid", "epd_restarts")}
        self.last_rejection = ""

    def ingest(self, report: dict[str, Any], now: float) -> str:
        stamp = report.get("stamp") or (0, 0)
        stamp_ns = int(stamp[0]) * 1_000_000_000 + int(stamp[1])
        frame_id = str(report.get("frame_id", ""))
        if stamp_ns <= 0 or not frame_id:
            return self._reject("invalid", "report without source stamp or frame_id")
        if self.expected_frame_id and frame_id != self.expected_frame_id:
            return self._reject("invalid", f"frame_id {frame_id} != expected {self.expected_frame_id}")
        errors: list[str] = []
        objects = [o for o in (_parse_object(s, errors) for s in report.get("statuses", [])) if o]
        objects.sort(key=lambda o: o["index"])
        obs_ids = {int(o["observation_id"]) for o in objects if o["observation_id"].isdigit()}
        if len(obs_ids) > 1:
            return self._reject("invalid", "mixed observation ids in one report")
        observation_id = next(iter(obs_ids), None)
        outcome = "accepted"
        if self.last_stamp_ns is not None:
            if stamp_ns == self.last_stamp_ns:
                return self._reject("duplicate", "duplicate source stamp")
            if stamp_ns < self.last_stamp_ns:
                return self._reject("out_of_order", "source stamp older than last accepted report")
            if (observation_id is not None and self.last_observation_id is not None and
                    observation_id <= self.last_observation_id):
                # Newer sensor time but a reset counter: EPD restarted. Its track IDs restart too.
                self.counters["epd_restarts"] += 1
                self.epd_session += 1
                outcome = "restart"
        self.counters["accepted"] += 1
        self.last_stamp_ns = stamp_ns
        self.last_observation_id = observation_id if observation_id is not None else self.last_observation_id
        self.last_wall_time = now
        self.frame_id = frame_id
        self.camera_id = str(report.get("hardware_id", "") or self.camera_id)
        self.objects = objects
        self.contract_errors = errors
        return outcome

    def _reject(self, kind: str, why: str) -> str:
        self.counters[kind] += 1
        self.last_rejection = why
        return kind

    def connection(self, now: float, publisher_count: int | None) -> tuple[str, str]:
        if self.last_wall_time is None:
            if publisher_count == 0:
                return ("UNAVAILABLE", "no publisher on readiness topic: EPD not running, "
                        "hybrid refinement off, or EPD without per-detection readiness")
            return ("UNKNOWN", "waiting for first readiness report")
        age = now - self.last_wall_time
        if age > self.stale_timeout_s:
            why = f"no readiness report for {age:.1f} s"
            if publisher_count == 0:
                why += "; EPD publisher disconnected"
            return ("STALE", why)
        return ("FRESH", "readiness reports current")

    def snapshot(self, now: float, publisher_count: int | None = None) -> dict[str, Any]:
        state, reason = self.connection(now, publisher_count)
        fresh = state == "FRESH"
        objects = self.objects if fresh else []  # stale data is never shown as current
        counts = {s: sum(1 for o in objects if o["state"] == s) for s in STATES}
        return {
            "schema_version": SCHEMA,
            "source_topic": self.topic,
            "written_wall_time": now,
            "connection_state": state,
            "connection_reason": reason,
            "publisher_count": publisher_count,
            "camera_id": self.camera_id,
            "frame_id": self.frame_id,
            "source_stamp_ns": self.last_stamp_ns,
            "observation_id": self.last_observation_id,
            "last_report_age_s": None if self.last_wall_time is None else now - self.last_wall_time,
            "epd_session": self.epd_session,
            "objects": objects,
            "last_object_count": len(self.objects),
            "state_counts": counts,
            "counters": dict(self.counters),
            "last_rejection": self.last_rejection,
            "contract_errors": self.contract_errors if fresh else [],
            "motion_authority": False,
            "note": "Diagnostic evidence only. Grasp planning consumes EPD validated-pose outputs; "
                    "GRASP_CANDIDATE does not imply reachability or collision freedom.",
        }


def report_from_msg(msg: Any) -> dict[str, Any]:
    return {
        "stamp": (msg.header.stamp.sec, msg.header.stamp.nanosec),
        "frame_id": msg.header.frame_id,
        "hardware_id": msg.status[0].hardware_id if msg.status else "",
        "statuses": [{"name": s.name, "level": int.from_bytes(s.level, "little") if isinstance(s.level, bytes) else s.level,
                      "message": s.message, "values": {kv.key: kv.value for kv in s.values}} for s in msg.status],
    }


def write_atomic(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=path.name, dir=str(path.parent))
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=1)
    os.replace(tmp, path)


class EpdReadinessMonitorNode(Node):
    def __init__(self):
        super().__init__("epd_readiness_monitor_node")
        self.declare_parameter("readiness_topic", DEFAULT_TOPIC)
        self.declare_parameter("output_path", "")
        self.declare_parameter("scene_path", "")
        # CPU EPD with hybrid refinement measured ~4-5 s per frame on the D435i replay.
        self.declare_parameter("stale_timeout_s", 10.0)
        self.declare_parameter("expected_frame_id", "")
        self.declare_parameter("write_period_s", 0.5)
        output = str(self.get_parameter("output_path").value)
        scene = str(self.get_parameter("scene_path").value)
        if not output and not scene:
            raise RuntimeError("set scene_path or output_path")
        self.output = Path(output) if output else Path(scene) / "preview" / "epd_detection_readiness.json"
        self.topic = str(self.get_parameter("readiness_topic").value)
        self.monitor = ReadinessMonitor(float(self.get_parameter("stale_timeout_s").value),
                                        str(self.get_parameter("expected_frame_id").value), self.topic)
        # Subscription only. This node creates no publishers and no service clients.
        self.sub = self.create_subscription(
            DiagnosticArray, self.topic, self.on_report,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.timer = self.create_timer(float(self.get_parameter("write_period_s").value), self.on_timer)

    def on_report(self, msg):
        outcome = self.monitor.ingest(report_from_msg(msg), time.time())
        if outcome not in ("accepted", "restart"):
            self.get_logger().warn(f"readiness report {outcome}: {self.monitor.last_rejection}")

    def on_timer(self):
        write_atomic(self.output, self.monitor.snapshot(time.time(), self.count_publishers(self.topic)))


def main():
    if rclpy is None:
        raise RuntimeError("rclpy and diagnostic_msgs are required to run epd_readiness_monitor_node")
    rclpy.init()
    node = EpdReadinessMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
