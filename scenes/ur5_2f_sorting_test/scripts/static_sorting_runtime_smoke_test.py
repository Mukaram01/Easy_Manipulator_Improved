#!/usr/bin/env python3
"""Safe static sorting runtime smoke-test helper."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shlex
import subprocess
import sys

SCHEMA = "static_sorting_runtime_smoke_test/v1"
SCENE_PACKAGE = "ur5_2f_sorting_test"
DEFAULT_PAYLOAD_PATH = Path("/tmp/ur5_2f_sorting_runtime_bridge_payload.json")
SEQUENTIAL_ORDER = ["item_red", "item_blue", "item_green"]


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--target", choices=SEQUENTIAL_ORDER, help="Single target for repeatable smoke test")
    p.add_argument("--all", action="store_true", help="Preview all known targets conservatively")
    p.add_argument("--allow-batch-runtime-send", action="store_true", help="Allow runtime send when --all is selected")
    p.add_argument("--payload-output", type=Path, default=DEFAULT_PAYLOAD_PATH)
    p.add_argument("--json", action="store_true")
    p.add_argument("--print-commands", action="store_true")
    p.add_argument("--require-active-runtime", action="store_true")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--confirm-runtime-send", action="store_true")
    return p.parse_args(argv)


def _run_subprocess(cmd: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, check=False, capture_output=True, text=True)


def _launch_command(payload_path: Path) -> str:
    return (
        "ros2 launch ur5_2f_sorting_test demo.launch.py "
        "explicit_release_pose_source:=bridge_payload "
        f"explicit_release_pose_bridge_payload_path:={payload_path}"
    )


def _executor_command(args: argparse.Namespace, targets: list[str]) -> str:
    cmd = ["ros2", "run", SCENE_PACKAGE, "manual_static_sorting_executor"]
    for t in targets:
        cmd += ["--target", t]
    cmd += ["--require-active-runtime", "--manual-enable-execution", "--execute", "--confirm-runtime-send", "--json"]
    return " ".join(shlex.quote(x) for x in cmd)


def build_report(args: argparse.Namespace) -> tuple[dict, int]:
    targets = SEQUENTIAL_ORDER[:] if args.all else ([args.target] if args.target else [SEQUENTIAL_ORDER[0]])
    payload_output = args.payload_output
    send_guarded = args.require_active_runtime and args.execute and args.confirm_runtime_send
    batch_blocked = args.all and send_guarded and not args.allow_batch_runtime_send

    cmd = [
        "ros2", "run", SCENE_PACKAGE, "manual_static_sorting_executor",
        "--payload-output", str(payload_output), "--json",
    ]
    for t in targets:
        cmd += ["--target", t]
    if args.require_active_runtime:
        cmd.append("--require-active-runtime")
    if send_guarded and not batch_blocked:
        cmd += ["--manual-enable-execution", "--execute", "--confirm-runtime-send"]

    rep = {
        "schema": SCHEMA,
        "payload_output": str(payload_output),
        "target_count": len(targets),
        "selected_target_ids": targets,
        "recommended_sequential_order": SEQUENTIAL_ORDER,
        "replay_dry_run_status": "unknown",
        "runtime_check_status": "not_requested",
        "robot_motion_requested": False,
        "final_send_command": _executor_command(args, targets),
        "final_send_command_executed": False,
        "runtime_send": {"executed": False, "message": "No runtime send executed in dry-run mode."},
        "launch_command": _launch_command(payload_output),
        "execution_command": _executor_command(args, targets),
        "warnings": [],
        "targets": [],
    }

    if args.all:
        rep["warnings"].append("Conservative mode: recommended sequential test order is item_red -> item_blue -> item_green.")
    if batch_blocked:
        rep["warnings"].append("Batch runtime send blocked for --all. Add --allow-batch-runtime-send to permit it.")

    run = _run_subprocess(cmd)
    if run.returncode != 0:
        rep["replay_dry_run_status"] = "fail"
        rep["runtime_check_status"] = "failed_or_missing" if args.require_active_runtime else "not_requested"
        rep["warnings"].append((run.stderr or run.stdout).strip())
        return rep, 2

    payload = json.loads(run.stdout)
    rep["replay_dry_run_status"] = payload.get("payload_validation", {}).get("dry_run_replay_status", "unknown")
    runtime_checks = payload.get("runtime_checks", {})
    rep["runtime_check_status"] = "requested" if args.require_active_runtime else "not_requested"
    if args.require_active_runtime:
        rep["runtime_check_status"] = "pass" if payload.get("result", {}).get("status") != "runtime_missing" else "runtime_missing"
    rep["robot_motion_requested"] = bool(payload.get("robot_motion_requested", False))
    rep["final_send_command_executed"] = bool(payload.get("execution_attempted", False))
    rep["manual_executor_result"] = payload.get("result", {})
    rep["manual_executor_command"] = payload.get("runtime_send", {}).get("command", [])
    if payload.get("execution_attempted", False):
        rep["runtime_send"] = payload.get("runtime_send", {"executed": True})

    for t in payload.get("grasp_task", {}).get("grasp_targets", []):
        rep["targets"].append({
            "object_id": t.get("object_id"),
            "destination_id": t.get("destination_id"),
            "destination_pose": {"frame_id": (t.get("destination_pose") or {}).get("frame_id")},
        })

    # manual executor report doesn't expose grasp_targets directly; read payload file for observability.
    try:
        payload_json = json.loads(Path(payload.get("payload", {}).get("path", str(payload_output))).read_text(encoding="utf-8"))
        rep["targets"] = [{
            "object_id": gt.get("object_id"),
            "destination_id": gt.get("destination_id"),
            "destination_pose": {"frame_id": (gt.get("destination_pose") or {}).get("frame_id")},
        } for gt in payload_json.get("grasp_task", {}).get("grasp_targets", [])]
    except Exception as exc:
        rep["warnings"].append(f"Could not read payload target observability fields: {exc}")

    return rep, 0


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    report, code = build_report(args)

    should_print_commands = args.print_commands or True
    if should_print_commands and not args.json:
        print(f"Terminal 1: {report['launch_command']}")
        print(f"Terminal 2: {report['execution_command']}")

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(json.dumps(report, indent=2))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
