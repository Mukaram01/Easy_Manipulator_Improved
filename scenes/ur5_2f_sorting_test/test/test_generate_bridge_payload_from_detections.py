#!/usr/bin/env python3
"""Validate detected_objects/v1 -> runtime_execution_plan/v1 -> emd_grasp_bridge_payload/v1."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile


def main() -> int:
    scene_root = Path(__file__).resolve().parents[1]
    script_path = scene_root / "scripts" / "generate_bridge_payload_from_detections.py"
    fixture_path = scene_root / "fixtures" / "detected_objects_sample.json"

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
    payload = json.loads(json_result.stdout)

    if payload.get("schema") != "emd_grasp_bridge_payload/v1":
        raise AssertionError("final schema mismatch")

    chain = payload.get("source_chain")
    if chain != ["detected_objects/v1", "runtime_execution_plan/v1"]:
        raise AssertionError(f"source_chain mismatch: {chain}")

    by_class = {target.get("metadata", {}).get("class_label"): target for target in payload.get("targets", [])}
    if by_class.get("red_block", {}).get("destination", {}).get("id") != "bin_a":
        raise AssertionError("red_block target did not route to bin_a")
    if by_class.get("blue_block", {}).get("destination", {}).get("id") != "bin_b":
        raise AssertionError("blue_block target did not route to bin_b")
    if by_class.get("green_block", {}).get("destination", {}).get("id") != "reject_bin":
        raise AssertionError("green_block target did not route to reject_bin")

    for target in payload.get("targets", []):
        metadata = target.get("metadata", {})
        for key in ("detected_object_id", "class_label", "confidence"):
            if key not in metadata:
                raise AssertionError(f"missing target metadata key: {key}")

    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    low_conf_fixture = dict(fixture)
    low_conf_fixture["objects"] = [dict(fixture["objects"][0], confidence=0.2)]
    unknown_fixture = dict(fixture)
    unknown_fixture["objects"] = [dict(fixture["objects"][0], class_label="mystery_block")]

    with tempfile.TemporaryDirectory() as tmp_dir:
        temp_dir = Path(tmp_dir)
        low_conf_path = temp_dir / "low_conf.json"
        unknown_path = temp_dir / "unknown.json"
        runtime_plan_output = temp_dir / "runtime_plan.json"
        bridge_output = temp_dir / "bridge_payload.json"

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

        low_target = json.loads(low_conf_json.stdout)["targets"][0]
        unknown_target = json.loads(unknown_json.stdout)["targets"][0]
        if low_target.get("destination", {}).get("id") != "reject_bin":
            raise AssertionError("low confidence fixture case did not route to reject_bin")
        if unknown_target.get("destination", {}).get("id") != "reject_bin":
            raise AssertionError("unknown class fixture case did not route to reject_bin")

        subprocess.run(
            [
                sys.executable,
                str(script_path),
                "--detections",
                str(fixture_path),
                "--runtime-plan-output",
                str(runtime_plan_output),
                "--output",
                str(bridge_output),
            ],
            check=True,
            capture_output=True,
            text=True,
        )

        runtime_plan = json.loads(runtime_plan_output.read_text(encoding="utf-8"))
        bridge_payload = json.loads(bridge_output.read_text(encoding="utf-8"))
        if runtime_plan.get("schema") != "runtime_execution_plan/v1":
            raise AssertionError("runtime plan output schema mismatch")
        if bridge_payload.get("schema") != "emd_grasp_bridge_payload/v1":
            raise AssertionError("bridge output schema mismatch")

    return 0


if __name__ == "__main__":
    sys.exit(main())
