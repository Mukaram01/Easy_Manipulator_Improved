#!/usr/bin/env python3
"""Prepare static sorting execution artifacts without requesting robot motion."""

from __future__ import annotations

import argparse
import importlib.machinery
import importlib.util
import json
from pathlib import Path
import sys

SCHEMA = "static_sorting_execution_preparation/v1"
SCENE_PACKAGE = "ur5_2f_sorting_test"
LAUNCH_COMMAND = [
    "ros2",
    "launch",
    "run_grasp_execution",
    "grasp_execution.launch.py",
    "scene_package:=ur5_2f_sorting_test",
    "launch_rviz:=true",
]


def _load_sibling_script_module(module_name: str):
    """Load sibling helper script from source or installed ROS 2 executable layout."""
    script_dir = Path(__file__).resolve().parent
    candidates = [
        script_dir / f"{module_name}.py",
        script_dir / module_name,
    ]

    for candidate in candidates:
        if not candidate.exists():
            continue

        unique_name = f"_ur5_2f_sorting_test_{module_name}"
        if candidate.suffix == ".py":
            spec = importlib.util.spec_from_file_location(unique_name, candidate)
            if spec is None or spec.loader is None:
                continue
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module

        loader = importlib.machinery.SourceFileLoader(unique_name, str(candidate))
        spec = importlib.util.spec_from_loader(unique_name, loader)
        if spec is None:
            continue
        module = importlib.util.module_from_spec(spec)
        loader.exec_module(module)
        return module

    searched = ", ".join(str(candidate) for candidate in candidates)
    raise ModuleNotFoundError(f"Could not load sibling script module '{module_name}'. Searched: {searched}")


runtime_plan = _load_sibling_script_module("generate_sorting_runtime_plan")
bridge_payload = _load_sibling_script_module("generate_sorting_emd_bridge_payload")
handoff_preview = _load_sibling_script_module("preview_sorting_execution_handoff")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="Print preparation report as JSON")
    parser.add_argument("--output", type=Path, help="Write preparation report JSON to path")
    parser.add_argument("--payload-output", type=Path, help="Write emd_grasp_bridge_payload/v1 JSON to path")
    parser.add_argument("--handoff-output", type=Path, help="Write sorting_execution_handoff_preview/v1 JSON to path")
    parser.add_argument("--print-launch-command", action="store_true", help="Print exact manual launch command")
    parser.add_argument("--manual-enable-execution", action="store_true", help="Manually mark execution enabled")
    return parser.parse_args(argv)


def build_report(manual_enable_execution: bool) -> tuple[dict, dict, dict]:
    manifest = runtime_plan.load_manifest(runtime_plan.find_manifest_path())
    plan = runtime_plan.build_runtime_plan(manifest)
    payload = bridge_payload.build_bridge_payload(plan)
    preview = handoff_preview.build_handoff_preview(payload, "static_manifest")

    targets = []
    for entry in preview.get("targets", []):
        targets.append(
            {
                "object_id": entry.get("object_id"),
                "object_frame": entry.get("object_frame"),
                "destination_id": entry.get("destination_id"),
                "destination_frame": entry.get("destination_frame"),
                "pick_hint": entry.get("pick_hint"),
                "release_offset_xyz_m": entry.get("release_offset_xyz_m"),
            }
        )

    execution_enabled = bool(manual_enable_execution)
    report = {
        "schema": SCHEMA,
        "scene_package": SCENE_PACKAGE,
        "mode": "dry_run",
        "execution_enabled": execution_enabled,
        "robot_motion_requested": False,
        "manual_execution_required": True,
        "target_count": len(targets),
        "targets": targets,
        "launch_command": LAUNCH_COMMAND,
        "payload_output_path": None,
        "handoff_output_path": None,
        "warnings": [
            "Preparation only; robot motion was not requested.",
            "Run the launch command manually and verify RViz before any execution.",
        ],
    }

    if execution_enabled:
        report["warnings"].append("Execution was manually enabled for planning metadata only; this script did not move the robot.")

    return report, payload, preview


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _print_report_text(report: dict) -> None:
    print("Static sorting execution preparation report")
    print(f"scene_package: {report['scene_package']}")
    print(f"schema: {report['schema']}")
    print(f"execution_enabled: {report['execution_enabled']}")
    print(f"robot_motion_requested: {report['robot_motion_requested']}")
    print(f"manual_execution_required: {report['manual_execution_required']}")
    print(f"target_count: {report['target_count']}")
    print("targets:")
    for target in report.get("targets", []):
        print(f"  - {target['object_id']} -> {target['destination_id']} ({target['destination_frame']})")

    print("launch_command:")
    print("  " + " ".join(report["launch_command"]))
    print(f"payload_output_path: {report.get('payload_output_path')}")
    print(f"handoff_output_path: {report.get('handoff_output_path')}")
    print("warnings:")
    for warning in report.get("warnings", []):
        print(f"  - {warning}")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    report, payload, preview = build_report(args.manual_enable_execution)

    if args.payload_output is not None:
        _write_json(args.payload_output, payload)
        report["payload_output_path"] = str(args.payload_output)

    if args.handoff_output is not None:
        _write_json(args.handoff_output, preview)
        report["handoff_output_path"] = str(args.handoff_output)

    if args.output is not None:
        _write_json(args.output, report)

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        _print_report_text(report)

    if args.print_launch_command:
        print("\nManual next command:")
        print(" ".join(LAUNCH_COMMAND))
        if args.manual_enable_execution:
            print("WARNING: This script did not move the robot; run launch manually after RViz verification.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
