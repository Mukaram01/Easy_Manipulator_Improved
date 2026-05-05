#!/usr/bin/env python3
"""Validate robot_studio_lite dashboard behavior."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile


def main() -> int:
    scene_root = Path(__file__).resolve().parents[1]
    script_path = scene_root / "scripts" / "robot_studio_lite.py"

    subprocess.run([sys.executable, str(script_path)], check=True, capture_output=True, text=True)

    json_result = subprocess.run([sys.executable, str(script_path), "--json"], check=True, capture_output=True, text=True)
    dashboard = json.loads(json_result.stdout)

    if dashboard.get("schema") != "robot_studio_lite_dashboard/v1":
        raise AssertionError("dashboard schema mismatch")

    safety = dashboard.get("safety", {})
    if safety.get("robot_motion_requested") is not False:
        raise AssertionError("robot_motion_requested must be false")
    if safety.get("live_epd_called") is not False:
        raise AssertionError("live_epd_called must be false")
    if safety.get("execution_enabled") is not False:
        raise AssertionError("execution_enabled must be false")

    if dashboard.get("bridge_payload", {}).get("target_count") != 3:
        raise AssertionError("bridge_payload target_count must be 3")
    if dashboard.get("handoff_preview", {}).get("execution_ready_preview") is not True:
        raise AssertionError("handoff_preview.execution_ready_preview must be true")

    quick_result = subprocess.run([sys.executable, str(script_path), "--quick", "--json"], check=True, capture_output=True, text=True)
    quick_dashboard = json.loads(quick_result.stdout)
    if quick_dashboard.get("layout_validation", {}).get("status") != "skipped_quick_mode":
        raise AssertionError("quick mode did not skip layout validation")

    with tempfile.TemporaryDirectory() as tmp_dir:
        output_path = Path(tmp_dir) / "dashboard.json"
        subprocess.run([sys.executable, str(script_path), "--json", "--output", str(output_path)], check=True, capture_output=True, text=True)
        written = json.loads(output_path.read_text(encoding="utf-8"))
        if written.get("schema") != "robot_studio_lite_dashboard/v1":
            raise AssertionError("written dashboard schema mismatch")

    return 0


if __name__ == "__main__":
    sys.exit(main())
