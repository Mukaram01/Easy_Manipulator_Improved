#!/usr/bin/env python3
"""Validate prepare_static_sorting_execution behavior."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile


def main() -> int:
    scene_root = Path(__file__).resolve().parents[1]
    script_path = scene_root / "scripts" / "prepare_static_sorting_execution.py"

    subprocess.run([sys.executable, str(script_path)], check=True, capture_output=True, text=True)

    json_result = subprocess.run([sys.executable, str(script_path), "--json"], check=True, capture_output=True, text=True)
    report = json.loads(json_result.stdout)

    if report.get("schema") != "static_sorting_execution_preparation/v1":
        raise AssertionError("schema mismatch")
    if report.get("target_count") != 3:
        raise AssertionError("target_count must be 3")
    if report.get("execution_enabled") is not False:
        raise AssertionError("execution_enabled must be false by default")
    if report.get("robot_motion_requested") is not False:
        raise AssertionError("robot_motion_requested must be false")

    launch_command = " ".join(report.get("launch_command", []))
    for token in (
        "run_grasp_execution",
        "grasp_execution.launch.py",
        "scene_package:=ur5_2f_sorting_test",
        "launch_rviz:=true",
    ):
        if token not in launch_command:
            raise AssertionError(f"launch command missing token: {token}")

    manual_result = subprocess.run(
        [sys.executable, str(script_path), "--json", "--manual-enable-execution"], check=True, capture_output=True, text=True
    )
    manual_report = json.loads(manual_result.stdout)
    if manual_report.get("execution_enabled") is not True:
        raise AssertionError("execution_enabled must be true with --manual-enable-execution")
    if manual_report.get("robot_motion_requested") is not False:
        raise AssertionError("robot_motion_requested must remain false")

    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp = Path(tmp_dir)
        payload_output = tmp / "payload.json"
        handoff_output = tmp / "handoff.json"
        report_output = tmp / "report.json"

        subprocess.run(
            [
                sys.executable,
                str(script_path),
                "--payload-output",
                str(payload_output),
                "--handoff-output",
                str(handoff_output),
                "--output",
                str(report_output),
            ],
            check=True,
            capture_output=True,
            text=True,
        )

        payload = json.loads(payload_output.read_text(encoding="utf-8"))
        handoff = json.loads(handoff_output.read_text(encoding="utf-8"))
        written_report = json.loads(report_output.read_text(encoding="utf-8"))

        if payload.get("schema") != "emd_grasp_bridge_payload/v1":
            raise AssertionError("payload schema mismatch")
        if handoff.get("schema") != "sorting_execution_handoff_preview/v1":
            raise AssertionError("handoff schema mismatch")
        if written_report.get("schema") != "static_sorting_execution_preparation/v1":
            raise AssertionError("written report schema mismatch")

    return 0


if __name__ == "__main__":
    sys.exit(main())
