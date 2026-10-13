#!/usr/bin/env python3
"""Bounded, read-only live EPD capture for an explicitly saved Workcell Studio task.

Never launches a camera/EPD/robot; never edits TaskIntent, geometry or a replay file.
A successful capture is a candidate for plan-only input, not a safety certificate.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import yaml

SCRIPTS = Path(__file__).resolve().parent
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))


def source_binding(scene: Path) -> tuple[dict, dict, dict, dict]:
    from workcell_camera_probe import camera_config

    task_path = scene / "config/workcell_builder_task_intent.yaml"
    task = yaml.safe_load(task_path.read_text(encoding="utf-8"))
    if not isinstance(task, dict):
        raise ValueError("TASK_INVALID: saved TaskIntent must be a mapping")
    binding = task.get("perception")
    if not isinstance(binding, dict) or binding.get("mode") != "live_epd":
        raise ValueError("LIVE_TASK_REQUIRED: save a live task before capturing EPD objects")
    selected = binding.get("camera_source")
    if selected not in ("simulated_rgbd", "realsense_d435i"):
        raise ValueError("CAMERA_SOURCE_REQUIRED: select a supported live camera in Task settings")
    camera = camera_config(scene, selected)
    if not camera["camera_id"]:
        raise ValueError("CAMERA_ID_MISSING: a physical scene camera_id is required")

    document = yaml.safe_load((scene / "environment.yaml").read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("EPD_CONFIG_MISSING: scene environment must be a mapping")
    root = document.get("environment", document)
    if not isinstance(root, dict):
        raise ValueError("EPD_CONFIG_MISSING: scene environment must be a mapping")
    config = root.get("perception") or document.get("perception", {})
    if not isinstance(config, dict):
        raise ValueError("EPD_CONFIG_MISSING: scene perception configuration must be a mapping")
    epd = config.get("epd_input")
    if not isinstance(epd, dict):
        raise ValueError("EPD_CONFIG_MISSING: configure the EPD localization/tracking topic in the scene")
    msg_type = epd.get("message_type")
    if msg_type == "epd_msgs/msg/EPDObjectLocalization":
        mode = "localization"
    elif msg_type == "epd_msgs/msg/EPDObjectTracking":
        mode = "tracking"
    else:
        raise ValueError("EPD_MESSAGE_TYPE_UNSUPPORTED: use the existing EPD localization/tracking output")
    topic = epd.get("topic")
    if not isinstance(topic, str) or not topic.startswith("/"):
        raise ValueError("EPD_TOPIC_MISSING: configure an absolute EPD output topic")
    return task, camera, {"topic": topic, "mode": mode}, config


def qualify_capture(payload: dict, task: dict, camera: dict, *, now: float | None = None) -> dict:
    """Reject unmeasured, stale, wrong-frame, wrong-provenance and untransformed output."""
    now = time.time() if now is None else now
    if not isinstance(payload, dict) or payload.get("schema_version") != "detected_objects/v1":
        raise ValueError("DETECTIONS_SCHEMA_INVALID: require detected_objects/v1")
    source = payload.get("source")
    if not isinstance(source, dict) or source.get("mode") != "live_epd":
        raise ValueError("LIVE_INPUT_REQUIRED: an EPD live capture is required, not replay")
    if source.get("frame_id") != camera["frame_id"]:
        raise ValueError("CAMERA_FRAME_MISMATCH: EPD header frame differs from saved camera optical frame")
    transform = source.get("transform")
    if not isinstance(transform, dict) or transform.get("status") != "PASS" or transform.get("target_frame") != "world":
        raise ValueError("TF_UNVERIFIED: all object poses must be transformed to world at observation time")
    stamp = source.get("source_stamp_ns")
    if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
        raise ValueError("EPD_STAMP_MISSING: sensor acquisition timestamp is required")
    age = now - stamp / 1e9
    limit = task.get("pick", {}).get("selection", {}).get("object_filter", {}).get("max_age_seconds")
    if isinstance(limit, bool) or not isinstance(limit, (int, float)) or not math.isfinite(limit) or limit <= 0:
        raise ValueError("FRESHNESS_POLICY_REQUIRED: author a positive observation max_age_seconds")
    if age < -0.05 or age > limit:
        raise ValueError(f"EPD_STALE: capture age {age:.3f}s violates the saved {limit}s freshness rule")
    objects = payload.get("objects")
    if not isinstance(objects, list) or not objects:
        raise ValueError("OBJECTS_MISSING: no measured EPD objects were captured")
    for index, obj in enumerate(objects):
        if not isinstance(obj, dict):
            raise ValueError(f"OBJECT_INVALID: objects[{index}] must be a mapping")
        pose = obj.get("pose")
        if not isinstance(pose, dict) or pose.get("frame_id") != "world":
            raise ValueError(f"POSE_UNTRANSFORMED: objects[{index}] does not have a world-frame pose")
        for label, vec in (("pose.xyz", pose.get("xyz")), ("pose.rpy", pose.get("rpy"))):
            if not isinstance(vec, list) or len(vec) != 3 or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in vec
            ):
                raise ValueError(f"OBJECT_GEOMETRY_INVALID: objects[{index}].{label} must be finite XYZ/RPY")
        dims = obj.get("dimensions")
        if isinstance(dims, dict):
            dims = [dims.get(x) for x in ("x", "y", "z")]
        if not isinstance(dims, list) or len(dims) != 3 or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0 for v in dims
        ):
            raise ValueError(f"MEASURED_GEOMETRY_MISSING: objects[{index}] lacks strictly positive observed dimensions")
        obj_stamp = obj.get("timestamp")
        if isinstance(obj_stamp, bool) or not isinstance(obj_stamp, (int, float)) or not math.isfinite(obj_stamp) or abs(obj_stamp - stamp / 1e9) > 1e-6:
            raise ValueError(f"OBJECT_STAMP_INVALID: objects[{index}] must preserve the EPD acquisition time")
    # This describes the *selected transport*, NOT hardware identity attestation.
    # No camera/EPD data is falsified and a replay is never labelled live.
    source["camera_source"] = camera["camera_source"]
    source["configured_camera_id"] = camera["camera_id"]
    source["camera_source_verification"] = "configured_ros_topic_only_not_hardware_attested"
    return payload


def capture_task(scene: Path, *, timeout: float = 10.0,
                 runner=subprocess.run, clock=time.time) -> dict[str, Any]:
    """Capture a candidate under a new evidence directory; never overwrite task files."""
    scene = scene.resolve()
    task, camera, epd, config = source_binding(scene)
    if not 1.0 <= timeout <= 30:
        raise ValueError("CAPTURE_TIMEOUT_INVALID: allowed 1–30 seconds")
    capture_root = scene / "captures" / "epd"
    capture_root.mkdir(parents=True, exist_ok=True)
    evidence = Path(tempfile.mkdtemp(prefix="live_", dir=capture_root))
    raw_path = evidence / "detected_objects.yaml"
    diagnostics = evidence / "diagnostics.json"
    command = [sys.executable, str(SCRIPTS / "capture_epd_detected_objects.py"),
               "--scene-package", str(scene), "--topic", epd["topic"],
               "--message-type", epd["mode"], "--output", str(raw_path),
               "--diagnostics-output", str(diagnostics), "--target-frame", "world",
               "--timeout", str(timeout), "--tf-timeout", "2", "--min-objects", "1", "--once"]
    try:
        process = runner(command, capture_output=True, text=True, timeout=timeout + 8, check=False)
    except subprocess.TimeoutExpired as exc:
        raise ValueError(f"EPD_CAPTURE_TIMEOUT: no qualified observation within {timeout}s") from exc
    if process.returncode or not raw_path.is_file():
        raise ValueError(
            "EPD_CAPTURE_FAILED: no qualified ROS output (check EPD topic, TF and diagnostics at "
            + str(diagnostics) + "); exit=" + str(process.returncode))
    candidate = yaml.safe_load(raw_path.read_text(encoding="utf-8"))
    qualified = qualify_capture(candidate, task, camera, now=clock())
    # Candidate output is a new artifact; no existing scene/task file is rewritten.
    # The original ROS capture diagnostic and output remain available for debugging.
    qualified_path = evidence / "qualified_detected_objects.yaml"
    qualified_path.write_text(yaml.safe_dump(qualified, sort_keys=False), encoding="utf-8")

    # Exact shared selection/zone binding: no fresh timestamp rewriting or fallback.
    from task_intent_authoring import planning_input
    trial = copy.deepcopy(task)
    trial["perception"]["replay_path"] = str(qualified_path)
    planning_input(scene, trial)
    return {
        "status": "CAPTURE_QUALIFIED_FOR_TASK_INPUT",
        "capture_path": str(qualified_path),
        "diagnostics_path": str(diagnostics),
        "camera_source": camera["camera_source"],
        "configured_camera_id": camera["camera_id"],
        "object_count": len(qualified["objects"]),
        "source_identity_attested": False,
        "planning_ready": False,
        "robot_motion_commanded": False,
        "note": "Current, world-transformed, measured task input candidate; camera hardware origin and physical grasp remain unqualified. Select and Save this path explicitly in Builder.",
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args(argv)
    try:
        result = capture_task(args.scene, timeout=args.timeout)
    except (ValueError, OSError, TypeError, KeyError, AttributeError, yaml.YAMLError) as exc:
        result = {"status": "BLOCKED", "error": str(exc),
                  "planning_ready": False, "robot_motion_commanded": False}
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == "CAPTURE_QUALIFIED_FOR_TASK_INPUT" else 1


if __name__ == "__main__":
    raise SystemExit(main())
