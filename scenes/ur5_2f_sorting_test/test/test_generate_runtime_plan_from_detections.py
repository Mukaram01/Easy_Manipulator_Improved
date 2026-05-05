#!/usr/bin/env python3
"""Validate offline detected_objects/v1 -> runtime_execution_plan/v1 generation."""

import json
from pathlib import Path
import subprocess
import sys


def main() -> int:
    scene_root = Path(__file__).resolve().parents[1]
    script_path = scene_root / "scripts" / "generate_runtime_plan_from_detections.py"
    fixture_path = scene_root / "fixtures" / "detected_objects_sample.json"

    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    if fixture.get("schema") != "detected_objects/v1":
        raise AssertionError("fixture schema mismatch")

    subprocess.run(
        [sys.executable, str(script_path), "--detections", str(fixture_path)],
        check=True,
        capture_output=True,
        text=True,
    )

    json_result = subprocess.run(
        [sys.executable, str(script_path), "--detections", str(fixture_path), "--json"],
        check=True,
        capture_output=True,
        text=True,
    )
    plan = json.loads(json_result.stdout)
    if plan.get("schema") != "runtime_execution_plan/v1":
        raise AssertionError("schema mismatch")

    place_steps = [step for step in plan.get("steps", []) if step.get("type") == "place"]
    route_by_class = {step.get("class_label"): step.get("destination_id") for step in place_steps}
    if route_by_class.get("red_block") != "bin_a":
        raise AssertionError("red_block did not route to bin_a")
    if route_by_class.get("blue_block") != "bin_b":
        raise AssertionError("blue_block did not route to bin_b")
    if route_by_class.get("green_block") != "reject_bin":
        raise AssertionError("green_block did not route to reject_bin")

    for step in place_steps:
        release_offset = step.get("release_offset_xyz_m")
        if not isinstance(release_offset, list) or len(release_offset) != 3 or release_offset[2] <= 0.0:
            raise AssertionError("place release_offset z must be positive")

    low_conf_fixture = dict(fixture)
    low_conf_fixture["objects"] = [dict(fixture["objects"][0], confidence=0.2)]
    unknown_fixture = dict(fixture)
    unknown_fixture["objects"] = [dict(fixture["objects"][0], class_label="mystery_block")]

    temp_dir = scene_root / "test" / "_tmp"
    temp_dir.mkdir(parents=True, exist_ok=True)
    low_conf_path = temp_dir / "low_conf.json"
    unknown_path = temp_dir / "unknown.json"
    low_conf_path.write_text(json.dumps(low_conf_fixture), encoding="utf-8")
    unknown_path.write_text(json.dumps(unknown_fixture), encoding="utf-8")

    low_conf_json = subprocess.run(
        [sys.executable, str(script_path), "--detections", str(low_conf_path), "--json"],
        check=True,
        capture_output=True,
        text=True,
    )
    unknown_json = subprocess.run(
        [sys.executable, str(script_path), "--detections", str(unknown_path), "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    low_place = [s for s in json.loads(low_conf_json.stdout).get("steps", []) if s.get("type") == "place"][0]
    unknown_place = [s for s in json.loads(unknown_json.stdout).get("steps", []) if s.get("type") == "place"][0]

    if low_place.get("destination_id") != "reject_bin":
        raise AssertionError("low confidence object did not route to reject_bin")
    if unknown_place.get("destination_id") != "reject_bin":
        raise AssertionError("unknown class object did not route to reject_bin")

    return 0


if __name__ == "__main__":
    sys.exit(main())
