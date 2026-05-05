#!/usr/bin/env python3
"""Validate dry-run runtime_execution_plan/v1 preview generation."""

import json
from pathlib import Path
import subprocess
import sys


def main() -> int:
    script_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "generate_sorting_runtime_plan.py"
    )

    subprocess.run([sys.executable, str(script_path)], check=True, capture_output=True, text=True)

    json_result = subprocess.run(
        [sys.executable, str(script_path), "--json"],
        check=True,
        capture_output=True,
        text=True,
    )
    plan = json.loads(json_result.stdout)

    if plan.get("schema") != "runtime_execution_plan/v1":
        raise AssertionError("schema mismatch")

    steps = plan.get("steps")
    if not isinstance(steps, list) or len(steps) != 6:
        raise AssertionError(f"expected 6 steps total, found: {len(steps) if isinstance(steps, list) else 'n/a'}")

    place_steps = {step.get("object_id"): step for step in steps if step.get("type") == "place"}

    expected_routes = {
        "item_red": "bin_a",
        "item_blue": "bin_b",
        "item_green": "reject_bin",
    }
    for object_id, destination_id in expected_routes.items():
        step = place_steps.get(object_id)
        if step is None:
            raise AssertionError(f"missing place step for {object_id}")
        if step.get("destination_id") != destination_id:
            raise AssertionError(f"wrong destination for {object_id}: {step.get('destination_id')}")

        release_offset = step.get("release_offset_xyz_m")
        if not isinstance(release_offset, list) or len(release_offset) != 3:
            raise AssertionError(f"invalid release offset for {object_id}")
        if release_offset[2] <= 0.0:
            raise AssertionError(f"release_offset z must be positive for {object_id}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
