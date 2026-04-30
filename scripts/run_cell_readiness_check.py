#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

from run_generated_cell_acceptance import run_acceptance
from validate_detected_objects import _load_yaml_or_json
from workcell_discovery import discover_all


def _check_topic_exists(topic: str) -> bool:
    proc = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        return False
    topics = {line.strip() for line in proc.stdout.splitlines() if line.strip()}
    return topic in topics


def _check_tf_available(target_frame: str, camera_frame: str) -> bool:
    proc = subprocess.run(
        ["ros2", "run", "tf2_ros", "tf2_echo", target_frame, camera_frame, "--once"],
        capture_output=True,
        text=True,
        check=False,
        timeout=8,
    )
    return proc.returncode == 0


def _status_rank(status: str) -> int:
    return {"PASS": 0, "WARN": 1, "FAIL": 2}.get(status, 2)


def _set_check(checks: dict[str, dict[str, str]], name: str, status: str, message: str, blockers: list[str], warnings: list[str]) -> None:
    checks[name] = {"status": status, "message": message}
    if status == "FAIL":
        blockers.append(f"{name}: {message}")
    elif status == "WARN":
        warnings.append(f"{name}: {message}")


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Conservative generated-cell readiness preflight checker")
    p.add_argument("--scene-package", required=True)
    p.add_argument("--task-recipe", type=Path, required=True)
    p.add_argument("--detected-objects", type=Path)
    p.add_argument("--live", action="store_true")
    p.add_argument("--epd-topic", default="/easy_perception_deployment/epd_localize_output")
    p.add_argument("--camera-rgb-topic", default="/camera/camera/color/image_raw")
    p.add_argument("--camera-depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    p.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    p.add_argument("--pointcloud-topic", default="/camera/camera/depth/color/points")
    p.add_argument("--target-frame", default="world")
    p.add_argument("--camera-frame", default="camera_depth_optical_frame")
    p.add_argument("--check-tf", action="store_true")
    p.add_argument("--check-ros-topics", action="store_true")
    p.add_argument("--check-runtime", action="store_true")
    p.add_argument("--json", action="store_true")
    args = p.parse_args(argv)

    checks: dict[str, dict[str, str]] = {}
    blockers: list[str] = []
    warnings: list[str] = []

    scenes = {s["package_name"] for s in discover_all().get("scenes", [])}
    if args.scene_package in scenes:
        _set_check(checks, "scene_discovery", "PASS", "Scene package discovered.", blockers, warnings)
    else:
        _set_check(checks, "scene_discovery", "FAIL", f"Scene package '{args.scene_package}' was not discovered.", blockers, warnings)

    if not args.task_recipe.exists():
        _set_check(checks, "task_recipe", "FAIL", f"Task recipe not found: {args.task_recipe}", blockers, warnings)
    else:
        try:
            acceptance, _ = run_acceptance(args.scene_package, args.task_recipe, args.detected_objects or args.task_recipe, Path("/tmp/cell_readiness_preflight"), strict=False)
            if acceptance.get("status") == "FAIL":
                _set_check(checks, "generated_cycle_dry_run", "FAIL", "Generated-cell dry-run validation failed.", blockers, warnings)
            else:
                _set_check(checks, "generated_cycle_dry_run", "PASS", "Generated-cell dry-run validation passed.", blockers, warnings)
            _set_check(checks, "task_recipe", "PASS", "Task recipe exists and is readable.", blockers, warnings)
        except Exception as exc:  # noqa: BLE001
            _set_check(checks, "task_recipe", "FAIL", f"Task recipe validation failed: {exc}", blockers, warnings)
            _set_check(checks, "generated_cycle_dry_run", "WARN", "Dry-run validation skipped due to task validation failure.", blockers, warnings)

    if args.detected_objects:
        if not args.detected_objects.exists():
            _set_check(checks, "detected_objects", "FAIL", f"Detected objects file not found: {args.detected_objects}", blockers, warnings)
        else:
            try:
                data, _, _ = _load_yaml_or_json(args.detected_objects)
                count = len(data.get("objects", [])) if isinstance(data, dict) and isinstance(data.get("objects"), list) else 0
                st = "PASS" if count > 0 else "WARN"
                _set_check(checks, "detected_objects", st, f"Detected objects file is valid with {count} object(s).", blockers, warnings)
            except Exception as exc:  # noqa: BLE001
                _set_check(checks, "detected_objects", "FAIL", f"Detected objects validation failed: {exc}", blockers, warnings)
    else:
        _set_check(checks, "detected_objects", "PASS", "No detected objects file provided (optional).", blockers, warnings)

    if args.live:
        need_topic_check = args.check_ros_topics or True
        if need_topic_check:
            camera_topics = [args.camera_rgb_topic, args.camera_depth_topic, args.camera_info_topic, args.pointcloud_topic]
            missing_camera = [t for t in camera_topics if not _check_topic_exists(t)]
            if missing_camera:
                _set_check(checks, "camera_topics", "FAIL", f"Missing required camera topics: {missing_camera}", blockers, warnings)
            else:
                _set_check(checks, "camera_topics", "PASS", "All required camera topics are visible.", blockers, warnings)

            if _check_topic_exists(args.epd_topic):
                _set_check(checks, "epd_topic", "PASS", "EPD topic is visible.", blockers, warnings)
            else:
                _set_check(checks, "epd_topic", "FAIL", f"Missing required EPD topic: {args.epd_topic}", blockers, warnings)

        if args.check_tf:
            if _check_tf_available(args.target_frame, args.camera_frame):
                _set_check(checks, "tf_camera_to_world", "PASS", f"TF available: {args.camera_frame} -> {args.target_frame}", blockers, warnings)
            else:
                _set_check(checks, "tf_camera_to_world", "FAIL", f"Missing TF from {args.camera_frame} to {args.target_frame}", blockers, warnings)
        else:
            _set_check(checks, "tf_camera_to_world", "WARN", "TF check skipped (enable --check-tf for live execution readiness).", blockers, warnings)
    else:
        _set_check(checks, "camera_topics", "PASS", "Offline mode: ROS camera topics not required.", blockers, warnings)
        _set_check(checks, "epd_topic", "PASS", "Offline mode: EPD topic not required.", blockers, warnings)
        _set_check(checks, "tf_camera_to_world", "PASS", "Offline mode: TF check not required.", blockers, warnings)

    if args.check_runtime:
        runtime_ok = _check_topic_exists("/joint_states")
        _set_check(checks, "runtime_topics", "PASS" if runtime_ok else "WARN", "Runtime topics found." if runtime_ok else "Runtime topics/services are not visible.", blockers, warnings)
    else:
        _set_check(checks, "runtime_topics", "PASS", "Runtime checks not requested.", blockers, warnings)

    statuses = [v["status"] for v in checks.values()]
    overall = "FAIL" if "FAIL" in statuses else ("WARN" if "WARN" in statuses else "PASS")
    next_action = "Proceed with requested stage." if overall == "PASS" else ("Resolve warnings before trusting motion/replay." if overall == "WARN" else "Resolve blockers before continuing.")
    report = {
        "schema_version": "cell_readiness/v1",
        "status": overall,
        "scene_package": args.scene_package,
        "mode": "live" if args.live else "offline",
        "checks": checks,
        "blockers": blockers,
        "warnings": warnings,
        "next_recommended_action": next_action,
    }
    print(json.dumps(report, indent=2, sort_keys=True) if args.json else f"{overall}: {next_action}")
    return 1 if overall == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
