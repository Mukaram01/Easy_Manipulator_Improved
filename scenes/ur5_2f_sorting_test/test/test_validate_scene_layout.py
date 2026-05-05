#!/usr/bin/env python3
"""Run physical layout validator and assert success report markers."""

from pathlib import Path
import subprocess
import sys


def main() -> int:
    script_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "validate_scene_layout.py"
    )
    result = subprocess.run(
        [sys.executable, str(script_path)],
        check=True,
        capture_output=True,
        text=True,
    )
    output = result.stdout
    required = (
        "table_top_z:",
        "item_red: bottom_z=",
        "item_blue: bottom_z=",
        "item_green: bottom_z=",
        "bin_a: tray_bottom_z=",
        "bin_b: tray_bottom_z=",
        "reject_bin: tray_bottom_z=",
        "pickup_items_non_overlapping: YES",
        "destination_trays_non_overlapping: YES",
        "robot_mount: OK",
    )
    for marker in required:
        if marker not in output:
            raise AssertionError(f"missing marker in validator output: {marker}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
