#!/usr/bin/env python3
"""Validate dry-run emd_grasp_bridge_payload/v1 preview generation."""

import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def main() -> int:
    script_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "generate_sorting_emd_bridge_payload.py"
    )
    runtime_plan_script = script_path.parent / "generate_sorting_runtime_plan.py"

    subprocess.run([sys.executable, str(script_path)], check=True, capture_output=True, text=True)

    json_result = subprocess.run(
        [sys.executable, str(script_path), "--json"],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(json_result.stdout)

    if payload.get("schema") != "emd_grasp_bridge_payload/v1":
        raise AssertionError("schema mismatch")
    if payload.get("source_schema") != "runtime_execution_plan/v1":
        raise AssertionError("source_schema mismatch")

    targets = payload.get("targets")
    if not isinstance(targets, list) or len(targets) != 3:
        raise AssertionError(f"expected 3 targets total, found: {len(targets) if isinstance(targets, list) else 'n/a'}")

    expected_routes = {
        "item_red": "bin_a",
        "item_blue": "bin_b",
        "item_green": "reject_bin",
    }
    expected_sizes = {
        "item_red": [0.05, 0.05, 0.05],
        "item_blue": [0.05, 0.05, 0.05],
        "item_green": [0.05, 0.05, 0.05],
    }

    by_object = {target.get("object_id"): target for target in targets}
    for object_id, destination_id in expected_routes.items():
        target = by_object.get(object_id)
        if target is None:
            raise AssertionError(f"missing target for {object_id}")

        if target.get("destination", {}).get("id") != destination_id:
            raise AssertionError(f"wrong destination for {object_id}: {target.get('destination', {}).get('id')}")

        if target.get("pick_hint") != "top_grasp":
            raise AssertionError(f"pick_hint not preserved for {object_id}")

        if target.get("approximate_size_m") != expected_sizes[object_id]:
            raise AssertionError(f"approximate_size_m not preserved for {object_id}")

        release_offset = target.get("destination", {}).get("release_offset_xyz_m")
        if not isinstance(release_offset, list) or len(release_offset) != 3:
            raise AssertionError(f"invalid release offset for {object_id}")
        if release_offset[2] <= 0.0:
            raise AssertionError(f"release_offset z must be positive for {object_id}")

    warnings = payload.get("warnings")
    if not isinstance(warnings, list) or not any("No live EPD detections used" in w for w in warnings):
        raise AssertionError("missing dry-run warning about live EPD detections")

    runtime_plan_result = subprocess.run(
        [sys.executable, str(runtime_plan_script), "--json"],
        check=True,
        capture_output=True,
        text=True,
    )
    runtime_plan_data = json.loads(runtime_plan_result.stdout)

    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_scripts = Path(tmp_dir)
        payload_script_copy = tmp_scripts / "generate_sorting_emd_bridge_payload.py"
        runtime_plan_copy = tmp_scripts / "generate_sorting_runtime_plan"
        runtime_plan_json = tmp_scripts / "runtime_plan.json"

        shutil.copy2(script_path, payload_script_copy)
        shutil.copy2(runtime_plan_script, runtime_plan_copy)
        runtime_plan_json.write_text(json.dumps(runtime_plan_data), encoding="utf-8")

        installed_layout_result = subprocess.run(
            [
                sys.executable,
                str(payload_script_copy),
                "--runtime-plan",
                str(runtime_plan_json),
                "--json",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        installed_layout_payload = json.loads(installed_layout_result.stdout)
        if installed_layout_payload.get("schema") != "emd_grasp_bridge_payload/v1":
            raise AssertionError("installed-layout schema mismatch")

    return 0


if __name__ == "__main__":
    sys.exit(main())
