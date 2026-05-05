#!/usr/bin/env python3
"""Safe sequential wrapper around manual_static_sorting_executor."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys

SCHEMA = "static_sorting_sequence_runner/v1"
SCENE_PACKAGE = "ur5_2f_sorting_test"
DEFAULT_PAYLOAD_OUTPUT_DIR = Path("/tmp/ur5_2f_sorting_sequence")


def _load_sibling_script_module(module_name: str):
    script_dir = Path(__file__).resolve().parent
    candidate = script_dir / f"{module_name}.py"
    unique_name = f"_ur5_2f_sorting_test_{module_name}"
    spec = importlib.util.spec_from_file_location(unique_name, candidate)
    if spec is None or spec.loader is None:
        raise ModuleNotFoundError(f"Could not load sibling script module '{module_name}'")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


runtime_plan = _load_sibling_script_module("generate_sorting_runtime_plan")
manual_executor = _load_sibling_script_module("manual_static_sorting_executor")


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--json", action="store_true")
    p.add_argument("--print-commands", action="store_true")
    p.add_argument("--target", choices=["item_red", "item_blue", "item_green"])
    p.add_argument("--all", action="store_true", dest="all_targets")
    p.add_argument("--payload-output-dir", type=Path, default=DEFAULT_PAYLOAD_OUTPUT_DIR)
    p.add_argument("--require-active-runtime", action="store_true")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--confirm-runtime-send", action="store_true")
    p.add_argument("--allow-batch-runtime-send", action="store_true")
    return p.parse_args(argv)


def _sequence() -> list[dict]:
    manifest = runtime_plan.load_manifest(runtime_plan.find_manifest_path())
    plan = runtime_plan.build_runtime_plan(manifest)
    return [
        {"target_id": step.get("object_id"), "destination_id": step.get("destination_id")}
        for step in plan.get("steps", []) if step.get("type") == "pick"
    ]


def _selected_targets(args: argparse.Namespace) -> list[dict]:
    seq = _sequence()
    if args.target and args.all_targets:
        raise ValueError("Use either --target or --all, not both.")
    if args.target:
        return [entry for entry in seq if entry["target_id"] == args.target]
    return seq if args.all_targets or not args.target else []


def _manual_args(args: argparse.Namespace, target_id: str, payload_path: Path) -> argparse.Namespace:
    batch_send = args.all_targets and args.execute and args.confirm_runtime_send and args.allow_batch_runtime_send
    single_send = (not args.all_targets) and args.execute and args.confirm_runtime_send
    do_send = batch_send or single_send
    return argparse.Namespace(
        json=True,
        output=None,
        prepare_output=None,
        require_active_runtime=args.require_active_runtime,
        manual_enable_execution=do_send,
        execute=do_send,
        confirm_runtime_send=do_send,
        skip_send_dry_run_validation=False,
        runtime_payload=None,
        payload_output=payload_path,
        ros_interface="service",
        service_name="grasp_requests",
        topic_name="grasp_tasks",
        targets=[target_id],
    )


def build_report(args: argparse.Namespace) -> tuple[dict, int]:
    selected = _selected_targets(args)
    args.payload_output_dir.mkdir(parents=True, exist_ok=True)
    blocked_batch = args.all_targets and args.execute and args.confirm_runtime_send and not args.allow_batch_runtime_send
    mode = "runtime_send" if args.execute and args.confirm_runtime_send else "dry_run"
    report = {
        "schema": SCHEMA,
        "scene_package": SCENE_PACKAGE,
        "mode": mode,
        "target_count": len(selected),
        "selected_targets": [e["target_id"] for e in selected],
        "sequence": [],
        "safety": {
            "execution_requested": bool(args.execute),
            "runtime_send_confirmed": bool(args.confirm_runtime_send),
            "batch_runtime_send_allowed": bool(args.allow_batch_runtime_send),
            "robot_motion_requested": bool(args.execute and args.confirm_runtime_send and ((not args.all_targets) or args.allow_batch_runtime_send)),
        },
        "result": {"status": "dry_run_only"},
        "warnings": [],
    }
    if blocked_batch:
        report["result"] = {"status": "blocked"}
        report["warnings"].append("Batch runtime send blocked: add --allow-batch-runtime-send to execute with --all.")
        return report, 2

    for entry in selected:
        payload_path = args.payload_output_dir / f"{entry['target_id']}_runtime_payload.json"
        m_args = _manual_args(args, entry["target_id"], payload_path)
        m_report, m_rc = manual_executor.build_report(m_args)
        seq_entry = {
            "target_id": entry["target_id"],
            "destination_id": entry["destination_id"],
            "payload_path": str(payload_path),
            "manual_executor_command": ["ros2", "run", SCENE_PACKAGE, "manual_static_sorting_executor", "--target", entry["target_id"]],
            "manual_executor_result": m_report.get("result", {}),
        }
        if "runtime_send" in m_report:
            seq_entry["runtime_send"] = m_report["runtime_send"]
        report["sequence"].append(seq_entry)
        if m_rc != 0:
            report["result"] = {"status": "sequence_runtime_send_failed", "failed_target": entry["target_id"]}
            report["warnings"].append(f"Sequence stopped at failed target: {entry['target_id']}")
            return report, 2

    if args.execute and args.confirm_runtime_send:
        report["result"] = {"status": "sequence_runtime_send_succeeded"}
    return report, 0


def _print_commands() -> None:
    print("Terminal 1 (runtime): ros2 launch easy_manipulation_deployment grasp_execution_real.launch.py scene_package:=ur5_2f_sorting_test release_payload_file:=<path-to-release-payload.json>")
    print("Dry-run: ros2 run ur5_2f_sorting_test static_sorting_sequence_runner --all --json")
    print("Single-target execute: ros2 run ur5_2f_sorting_test static_sorting_sequence_runner --target item_red --execute --confirm-runtime-send --require-active-runtime --json")
    print("All-target execute (guarded): ros2 run ur5_2f_sorting_test static_sorting_sequence_runner --all --execute --confirm-runtime-send --allow-batch-runtime-send --require-active-runtime --json")
    print("NOTE: --all runtime execution is BLOCKED unless --allow-batch-runtime-send is provided.")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.print_commands:
        _print_commands()
    try:
        report, rc = build_report(args)
    except ValueError as exc:
        report = {"schema": SCHEMA, "scene_package": SCENE_PACKAGE, "result": {"status": "blocked"}, "warnings": [str(exc)]}
        rc = 2
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(json.dumps(report, indent=2))
    return rc


if __name__ == "__main__":
    sys.exit(main())
