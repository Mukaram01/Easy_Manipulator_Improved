#!/usr/bin/env python3
"""Validate guarded execution handoff preview generation."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile


def run_json(script_path: Path, args: list[str]) -> dict:
    result = subprocess.run([sys.executable, str(script_path), *args, "--json"], check=True, capture_output=True, text=True)
    return json.loads(result.stdout)


def expect_fail(script_path: Path, args: list[str], expected_text: str) -> None:
    result = subprocess.run([sys.executable, str(script_path), *args, "--json"], capture_output=True, text=True)
    if result.returncode == 0:
        raise AssertionError("expected failure but command succeeded")
    if expected_text not in (result.stderr + result.stdout):
        raise AssertionError(f"expected failure text not found: {expected_text}")


def main() -> int:
    scene_root = Path(__file__).resolve().parents[1]
    preview_script = scene_root / "scripts" / "preview_sorting_execution_handoff.py"
    bridge_script = scene_root / "scripts" / "generate_sorting_emd_bridge_payload.py"
    fixture_path = scene_root / "fixtures" / "detected_objects_sample.json"

    static_preview = run_json(preview_script, ["--from-static-manifest"])
    detections_preview = run_json(preview_script, ["--detections", str(fixture_path)])

    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp = Path(tmp_dir)
        bridge_path = tmp / "bridge_payload.json"
        subprocess.run([sys.executable, str(bridge_script), "--output", str(bridge_path)], check=True, capture_output=True, text=True)

        bridge_preview = run_json(preview_script, ["--bridge-payload", str(bridge_path)])

        expected_ids_by_source = {
            "static_manifest": {"item_red", "item_blue", "item_green"},
            "bridge_payload": {"item_red", "item_blue", "item_green"},
            "detections": {"det_red_001", "det_blue_001", "det_green_001"},
        }

        for preview in (static_preview, detections_preview, bridge_preview):
            if preview.get("schema") != "sorting_execution_handoff_preview/v1":
                raise AssertionError("preview schema mismatch")
            if preview.get("robot_motion_requested") is not False:
                raise AssertionError("robot_motion_requested should be false")
            if preview.get("requires_manual_execution_enable") is not True:
                raise AssertionError("requires_manual_execution_enable should be true")
            if preview.get("execution_ready_preview") is not True:
                raise AssertionError("execution_ready_preview should be true")

            targets = preview.get("targets", [])
            object_ids = {target.get("object_id") for target in targets}
            expected_ids = expected_ids_by_source[preview.get("source_mode")]
            if expected_ids - object_ids:
                raise AssertionError("expected targets are missing")
            for target in targets:
                if target.get("release_offset_xyz_m", [0.0, 0.0, 0.0])[2] <= 0:
                    raise AssertionError("release offset z must be positive")

        json_output_path = tmp / "preview.json"
        subprocess.run(
            [sys.executable, str(preview_script), "--from-static-manifest", "--output", str(json_output_path), "--json"],
            check=True,
            capture_output=True,
            text=True,
        )
        written_json = json.loads(json_output_path.read_text(encoding="utf-8"))
        if written_json.get("schema") != "sorting_execution_handoff_preview/v1":
            raise AssertionError("output JSON file schema mismatch")

        invalid_schema = json.loads(bridge_path.read_text(encoding="utf-8"))
        invalid_schema["schema"] = "bad_schema/v1"
        invalid_schema_path = tmp / "invalid_schema.json"
        invalid_schema_path.write_text(json.dumps(invalid_schema), encoding="utf-8")
        expect_fail(preview_script, ["--bridge-payload", str(invalid_schema_path)], "Invalid payload schema")

        duplicate_payload = json.loads(bridge_path.read_text(encoding="utf-8"))
        duplicate_payload["targets"].append(dict(duplicate_payload["targets"][0]))
        duplicate_path = tmp / "duplicate.json"
        duplicate_path.write_text(json.dumps(duplicate_payload), encoding="utf-8")
        expect_fail(preview_script, ["--bridge-payload", str(duplicate_path)], "Duplicate object_id")

        invalid_destination = json.loads(bridge_path.read_text(encoding="utf-8"))
        invalid_destination["targets"][0]["destination"]["id"] = "invalid_bin"
        invalid_destination_path = tmp / "invalid_destination.json"
        invalid_destination_path.write_text(json.dumps(invalid_destination), encoding="utf-8")
        expect_fail(preview_script, ["--bridge-payload", str(invalid_destination_path)], "invalid destination")

    return 0


if __name__ == "__main__":
    sys.exit(main())
