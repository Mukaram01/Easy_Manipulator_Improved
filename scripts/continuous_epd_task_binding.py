"""Preflight and ROS header guard for the EXISTING continuous EPD PlanningScene bridge.

No ROS imports, no runtime commands, no geometry defaults. The real camera
origin is not attested by a topic/frame match.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

from capture_selected_live_epd import source_binding


def _mapping(value: Any, label: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"LIVE_BINDING_INVALID: {label} must be a mapping")
    return value


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError(f"LIVE_BINDING_INVALID: {label} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"LIVE_BINDING_INVALID: {label} must be finite")
    return result


def _same_optional_number(a: Any, b: Any) -> bool:
    if a is None or b is None:
        return a is None and b is None
    try:
        return _number(a, "minimum confidence") == _number(b, "minimum confidence")
    except ValueError:
        return False


def load_bound_live_task(scene: Path) -> dict[str, Any]:
    """Bind the source selected by TaskIntent v2 to the generated scene exactly.

    In particular, do not trust the bridge's legacy fixture camera/topic
    defaults, and do not silently accept stale generated task filters.
    """
    scene = Path(scene).resolve()
    intent, authored_camera, epd, _ = source_binding(scene)
    if intent.get("schema") != "workcell_builder_task_intent/v2":
        raise ValueError("TASK_V2_REQUIRED: save/migrate the cell's authoritative TaskIntent v2")
    selection = _mapping(_mapping(intent.get("pick"), "pick").get("selection"), "pick.selection")
    filter_ = _mapping(selection.get("object_filter"), "pick.selection.object_filter")
    if selection.get("source_type") != "perception" or selection.get("source_ref") != "detected_objects/v1":
        raise ValueError("PICK_SOURCE_MISMATCH: saved live task must use detected_objects/v1")
    target = _mapping(_mapping(intent.get("place"), "place").get("target"), "place.target")
    safety = _mapping(intent.get("safety"), "safety")
    if safety.get("real_hardware_enabled") is not False or safety.get("require_fake_hardware") is not True:
        raise ValueError("FAKE_HARDWARE_REQUIRED: real robot execution remains locked")
    required_class = filter_.get("class_id")
    if not isinstance(required_class, str) or not required_class.strip():
        raise ValueError("PICK_CLASS_REQUIRED: author a nonempty target class")
    max_age = _number(filter_.get("max_age_seconds"), "max_age_seconds")
    if max_age <= 0:
        raise ValueError("FRESHNESS_POLICY_REQUIRED: max_age_seconds must be positive")
    min_conf = filter_.get("min_confidence")
    if min_conf is not None and not 0 <= _number(min_conf, "min_confidence") <= 1:
        raise ValueError("CONFIDENCE_POLICY_INVALID: min_confidence must be in [0,1] or null")

    definition_path = scene / "cell_definition.yaml"
    if not definition_path.is_file():
        raise ValueError("GENERATED_SCENE_MISSING: generate the current scene before continuous EPD")
    definition = _mapping(yaml.safe_load(definition_path.read_text(encoding="utf-8")), "cell_definition")
    cell = _mapping(definition.get("cell"), "cell_definition.cell")
    if cell.get("id") != scene.name:
        raise ValueError("SCENE_ID_MISMATCH: generated scene ID differs from the selected scene directory")
    planning_frame = cell.get("planning_frame")
    if planning_frame != "world":
        raise ValueError("PLANNING_FRAME_UNSUPPORTED: world-only task binding until qualified frame parity")

    generated = _mapping(definition.get("task"), "cell_definition.task")
    generated_filter = _mapping(generated.get("object_filter"), "cell_definition.task.object_filter")
    generated_pick = _mapping(generated.get("pick"), "cell_definition.task.pick")
    generated_place = _mapping(generated.get("place"), "cell_definition.task.place")
    if generated.get("object_source") != "perception":
        raise ValueError("GENERATED_PICK_SOURCE_MISMATCH: generated task must use perception")
    parity = {
        "pick zone": (selection.get("zone_ref"), generated_pick.get("source_ref")),
        "target class": (required_class, generated_filter.get("class_id")),
        "destination": (target.get("region_ref"), generated_place.get("region_ref")),
    }
    for name, (saved, generated_value) in parity.items():
        if saved != generated_value:
            raise ValueError(f"TASK_GENERATION_STALE: {name} in generated scene differs from saved TaskIntent")
    if _number(generated_filter.get("max_age_seconds"), "generated max_age_seconds") != max_age:
        raise ValueError("TASK_GENERATION_STALE: generated observation freshness differs from TaskIntent")
    if not _same_optional_number(min_conf, generated_filter.get("min_confidence")):
        raise ValueError("TASK_GENERATION_STALE: generated confidence requirement differs from TaskIntent")

    scene_camera = _mapping(definition.get("camera"), "cell_definition.camera")
    perception = _mapping(definition.get("perception"), "cell_definition.perception")
    generated_epd = _mapping(perception.get("epd_input"), "cell_definition.perception.epd_input")
    for name, saved, generated_value in (
        ("camera ID", authored_camera["camera_id"], scene_camera.get("id")),
        ("perception camera ID", authored_camera["camera_id"], perception.get("camera_id")),
        ("optical frame", authored_camera["frame_id"], scene_camera.get("frame_id")),
        ("perception frame", authored_camera["frame_id"], perception.get("frame_id")),
        ("EPD topic", epd["topic"], generated_epd.get("topic")),
        ("EPD message type", "epd_msgs/msg/EPDObject" + (
            "Localization" if epd["mode"] == "localization" else "Tracking"), generated_epd.get("message_type")),
    ):
        if not saved or saved != generated_value:
            raise ValueError(f"CAMERA_GENERATION_STALE: {name} differs between scene/TaskIntent and generated package")

    return {
        "scene": str(scene), "scene_id": cell["id"],
        "camera_source": authored_camera["camera_source"],
        "camera_id": authored_camera["camera_id"], "frame_id": authored_camera["frame_id"],
        "topic": epd["topic"], "message_type": epd["mode"], "planning_frame": planning_frame,
        "target_class": required_class, "pick_zone": selection["zone_ref"],
        "max_age_seconds": max_age, "minimum_confidence": min_conf,
        "status": "PREFLIGHT_CONFIGURED_ONLY",
        "source_identity_attested": False, "planning_ready": False, "robot_motion_commanded": False,
    }


def bound_epd_message_error(message: Any, bound: dict[str, Any], now_ns: int,
                            last_stamp_ns: int = 0) -> str:
    """Validate genuine ROS header provenance and acquisition time before updating scene.

    Uses caller's ROS clock domain. Wrong simulated clock domain blocks instead
    of stamping the current wall time or accepting a replay.
    """
    header = getattr(message, "header", None)
    frame = getattr(header, "frame_id", None)
    if frame != bound.get("frame_id"):
        return "CAMERA_FRAME_MISMATCH: EPD message header frame differs from saved camera"
    stamp = getattr(header, "stamp", None)
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if any(isinstance(v, bool) or not isinstance(v, int) for v in (sec, nanosec)):
        return "EPD_STAMP_MISSING: source acquisition stamp is required"
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return "EPD_STAMP_INVALID: invalid seconds/nanoseconds"
    timestamp_ns = sec * 1_000_000_000 + nanosec
    if timestamp_ns <= 0:
        return "EPD_STAMP_MISSING: zero acquisition stamp"
    if timestamp_ns <= last_stamp_ns:
        return "EPD_OUT_OF_ORDER: duplicate or backward ROS acquisition stamp"
    max_age = bound["max_age_seconds"]
    age_s = (now_ns - timestamp_ns) / 1e9
    if age_s < -0.05 or age_s > max_age:
        return f"EPD_STALE_OR_CLOCK_MISMATCH: age {age_s:.3f}s exceeds shared ROS clock window"
    return ""
