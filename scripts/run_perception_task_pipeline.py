#!/usr/bin/env python3
"""Offline-first orchestration of detected_objects -> runtime plan -> EMD bridge payload."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

SCRIPTS_DIR = Path(__file__).resolve().parent
VALIDATOR = SCRIPTS_DIR / "validate_detected_objects.py"
ADAPTER = SCRIPTS_DIR / "run_task_recipe_adapter.py"
BRIDGE = SCRIPTS_DIR / "convert_runtime_plan_to_emd_grasp.py"


def _run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, capture_output=True, text=True, check=False)


def _json_from_stdout(text: str) -> dict[str, Any]:
    start = text.find("{")
    if start < 0:
        return {}
    return json.loads(text[start:])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-recipe", type=Path, required=True)
    parser.add_argument("--detected-objects", type=Path, required=True)
    parser.add_argument("--project-dir", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("reports/runtime_pipeline"))
    parser.add_argument("--dry-run", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--send-to-ros", action="store_true", default=False)
    parser.add_argument("--ros-interface", choices=["service", "topic"], default="service")
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    validation_report = args.output_dir / "detected_objects_validation_report.json"
    runtime_plan = args.output_dir / "runtime_execution_plan.json"
    bridge_payload = args.output_dir / "emd_grasp_bridge_payload.json"
    summary_md = args.output_dir / "summary.md"

    validate_cmd = [sys.executable, str(VALIDATOR), str(args.detected_objects), "--json"]
    if args.strict:
        validate_cmd.append("--strict")
    validate_proc = _run(validate_cmd)
    validate_json = _json_from_stdout(validate_proc.stdout)
    validation_report.write_text(json.dumps(validate_json, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    adapter_cmd = [
        sys.executable,
        str(ADAPTER),
        "--task-recipe",
        str(args.task_recipe),
        "--objects",
        str(args.detected_objects),
        "--output",
        str(runtime_plan),
        "--json",
        "--mode",
        "offline",
    ]
    if args.project_dir:
        adapter_cmd.extend(["--project-dir", str(args.project_dir)])
    if args.strict:
        adapter_cmd.append("--strict")
    if not args.dry_run:
        adapter_cmd.append("--no-dry-run")
    adapter_proc = _run(adapter_cmd)
    adapter_json = _json_from_stdout(adapter_proc.stdout)

    bridge_cmd = [
        sys.executable,
        str(BRIDGE),
        "--runtime-plan",
        str(runtime_plan),
        "--output",
        str(bridge_payload),
        "--json",
        "--mode",
        "offline",
        "--ros-interface",
        args.ros_interface,
    ]
    if args.strict:
        bridge_cmd.append("--strict")
    bridge_proc = _run(bridge_cmd)
    bridge_json = _json_from_stdout(bridge_proc.stdout)

    ros_proc = None
    ros_note = "ROS send skipped (default)."
    if args.send_to_ros:
        ros_cmd = [
            sys.executable,
            str(BRIDGE),
            "--runtime-plan",
            str(runtime_plan),
            "--output",
            str(bridge_payload),
            "--json",
            "--mode",
            "ros",
            "--ros-interface",
            args.ros_interface,
            "--no-dry-run",
        ]
        ros_proc = _run(ros_cmd)
        ros_note = "ROS send explicitly requested."

    summary_lines = [
        "# Perception Task Pipeline Summary",
        "",
        f"- detected_objects input: `{args.detected_objects}`",
        f"- task_recipe input: `{args.task_recipe}`",
        f"- validation status: `{validate_json.get('status', 'UNKNOWN')}`",
        f"- runtime plan routed_count: `{adapter_json.get('summary', {}).get('routed_count', 'n/a')}`",
        f"- bridge status: `{bridge_json.get('status', 'UNKNOWN')}`",
        f"- ros: {ros_note}",
        "",
        "## Outputs",
        f"- `{validation_report}`",
        f"- `{runtime_plan}`",
        f"- `{bridge_payload}`",
    ]
    summary_md.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")

    payload = {
        "schema_version": "perception_task_pipeline_report/v1",
        "status": "PASS",
        "steps": {
            "validate_detected_objects": {"returncode": validate_proc.returncode, "status": validate_json.get("status")},
            "task_recipe_adapter": {"returncode": adapter_proc.returncode},
            "runtime_to_emd_bridge": {"returncode": bridge_proc.returncode, "status": bridge_json.get("status")},
            "ros_send": {"requested": args.send_to_ros, "returncode": None if ros_proc is None else ros_proc.returncode},
        },
        "artifacts": {
            "validation_report": str(validation_report),
            "runtime_execution_plan": str(runtime_plan),
            "emd_grasp_bridge_payload": str(bridge_payload),
            "summary": str(summary_md),
        },
    }

    if any(code != 0 for code in (validate_proc.returncode, adapter_proc.returncode, bridge_proc.returncode)):
        payload["status"] = "FAIL"
    elif validate_json.get("status") == "WARN" or bridge_json.get("status") == "WARN":
        payload["status"] = "WARN"

    print(json.dumps(payload, indent=2, sort_keys=True))
    return 1 if payload["status"] == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
