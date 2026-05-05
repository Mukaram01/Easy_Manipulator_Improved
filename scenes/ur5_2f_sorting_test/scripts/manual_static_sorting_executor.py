#!/usr/bin/env python3
"""Manually gated static sorting execution adapter with safe dry-run defaults."""

from __future__ import annotations

import argparse
import importlib.machinery
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

SCHEMA = "manual_static_sorting_executor/v1"
SCENE_PACKAGE = "ur5_2f_sorting_test"
DEFAULT_SERVICE = "grasp_requests"
DEFAULT_TOPIC = "grasp_tasks"


class ManualExecutorError(RuntimeError):
    """Raised when safety gates intentionally block execution."""


def _load_sibling_script_module(module_name: str):
    script_dir = Path(__file__).resolve().parent
    candidates = [script_dir / f"{module_name}.py", script_dir / module_name]

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

    raise ModuleNotFoundError(f"Could not load sibling script module '{module_name}'")


runtime_plan = _load_sibling_script_module("generate_sorting_runtime_plan")


ANSI_ESCAPE_RE = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON output")
    parser.add_argument("--output", type=Path, help="Write machine-readable JSON output to file")
    parser.add_argument("--prepare-output", type=Path, help="Write selected static preparation targets JSON to file")
    parser.add_argument("--require-active-runtime", action="store_true", help="Require active runtime checks to pass")
    parser.add_argument("--manual-enable-execution", action="store_true", help="Explicitly enable execution mode")
    parser.add_argument("--execute", action="store_true", help="Request execution attempt (still safety-gated)")
    parser.add_argument("--confirm-runtime-send", action="store_true", help="Final confirmation required before runtime send")
    parser.add_argument("--skip-send-dry-run-validation", action="store_true", help="Skip dry-run replay validation before runtime send")
    parser.add_argument("--runtime-payload", type=Path, help="Use an existing runtime-compatible payload")
    parser.add_argument("--payload-output", type=Path, help="Write generated payload to this path")
    parser.add_argument("--ros-interface", choices=["service", "topic"], default="service")
    parser.add_argument("--service-name", default=DEFAULT_SERVICE)
    parser.add_argument("--topic-name", default=DEFAULT_TOPIC)
    parser.add_argument("--target", action="append", dest="targets", help="Optional object_id filter; repeat for multiple")
    return parser.parse_args(argv)


def _initial_runtime_checks() -> dict:
    return {
        "checked": False,
        "grasp_execution_node": "unknown",
        "joint_states": "unknown",
        "controller_manager": "unknown",
        "ur5_arm_controller": "unknown",
        "grasp_requests_service": "unknown",
        "grasp_tasks_topic": "unknown",
    }


def _strip_ansi(value: str) -> str:
    return ANSI_ESCAPE_RE.sub("", value)


def _run_ros2(args: list[str]) -> subprocess.CompletedProcess:
    env = os.environ.copy()
    env.update({"NO_COLOR": "1", "CLICOLOR": "0", "CLICOLOR_FORCE": "0", "PY_COLORS": "0", "RCUTILS_COLORIZED_OUTPUT": "0", "TERM": "dumb"})
    return subprocess.run(["ros2", *args], check=False, capture_output=True, text=True, timeout=6, env=env)


def _run_subprocess(cmd: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, check=False, capture_output=True, text=True)


def _parse_controller_state(list_controllers_stdout: str, controller_name: str = "ur5_arm_controller") -> str:
    clean_stdout = _strip_ansi(list_controllers_stdout)
    for raw_line in clean_stdout.splitlines():
        cols = raw_line.strip().split()
        if cols and cols[0] == controller_name:
            return cols[2].strip().lower() if len(cols) >= 3 else "inactive"
    return "missing"


def _runtime_checks(service_name: str, topic_name: str) -> tuple[dict, list[str]]:
    checks = _initial_runtime_checks()
    warnings: list[str] = []
    checks["checked"] = True
    try:
        nodes = set(line.strip() for line in _run_ros2(["node", "list"]).stdout.splitlines() if line.strip())
        topics = set(line.strip() for line in _run_ros2(["topic", "list"]).stdout.splitlines() if line.strip())
        services = set(line.strip() for line in _run_ros2(["service", "list"]).stdout.splitlines() if line.strip())
        ctrl = _run_ros2(["control", "list_controllers"])
        checks["grasp_execution_node"] = "present" if "/grasp_execution_node" in nodes else "missing"
        checks["joint_states"] = "present" if "/joint_states" in topics else "missing"
        checks["controller_manager"] = "present" if any("controller_manager" in svc for svc in services) else "missing"
        checks["grasp_requests_service"] = "present" if f"/{service_name}" in services else "missing"
        checks["grasp_tasks_topic"] = "present" if f"/{topic_name}" in topics else "missing"
        checks["ur5_arm_controller"] = _parse_controller_state(ctrl.stdout) if ctrl.returncode == 0 else "missing"
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        warnings.append(f"ROS runtime checks unavailable: {exc}")
        for key in checks:
            if key != "checked":
                checks[key] = "missing"
    return checks, warnings


def _load_targets(selected: list[str] | None) -> list[dict]:
    manifest = runtime_plan.load_manifest(runtime_plan.find_manifest_path())
    plan = runtime_plan.build_runtime_plan(manifest)
    all_targets = [
        {"object_id": step.get("object_id"), "object_frame": step.get("object_frame"), "pick_hint": step.get("pick_hint"), "destination_id": step.get("destination_id")}
        for step in plan.get("steps", []) if step.get("type") == "pick"
    ]
    by_id = {t.get("object_id"): t for t in all_targets}
    if not selected:
        return all_targets
    missing = [target for target in selected if target not in by_id]
    if missing:
        raise ManualExecutorError(f"Unknown target object_id(s): {', '.join(missing)}")
    return [by_id[target] for target in selected]


def _find_replay_script() -> Path:
    repo_root = Path(__file__).resolve().parents[3]
    path = repo_root / "scripts" / "replay_emd_bridge_payload.py"
    if not path.exists():
        raise ManualExecutorError("Could not locate scripts/replay_emd_bridge_payload.py from repository root.")
    return path


def _default_report(args: argparse.Namespace) -> dict:
    return {
        "schema": SCHEMA,
        "scene_package": SCENE_PACKAGE,
        "execution_enabled": bool(args.manual_enable_execution),
        "execute_requested": bool(args.execute),
        "runtime_send_confirmed": bool(args.confirm_runtime_send),
        "execution_attempted": False,
        "robot_motion_requested": False,
        "ros_interface": args.ros_interface,
        "service_name": args.service_name,
        "topic_name": args.topic_name,
        "payload": {"path": "", "source": "generated"},
        "payload_validation": {"dry_run_replay_status": "skipped", "command": []},
        "runtime_checks": _initial_runtime_checks(),
        "target_count": 0,
        "selected_targets": [],
        "result": {"status": "dry_run_only"},
        "warnings": [],
    }


def _generate_payload(path: Path) -> None:
    cmd = ["ros2", "run", SCENE_PACKAGE, "generate_static_sorting_runtime_bridge_payload", "--output", str(path)]
    result = _run_subprocess(cmd)
    if result.returncode != 0:
        raise ManualExecutorError(f"Payload generation failed: {' '.join(cmd)} :: {result.stderr.strip() or result.stdout.strip()}")


def _build_replay_cmd(replay_script: Path, payload: Path, args: argparse.Namespace, dry_run: bool) -> list[str]:
    cmd = [sys.executable, str(replay_script), "--payload", str(payload), "--scene-package", SCENE_PACKAGE, "--ros-interface", args.ros_interface]
    if args.ros_interface == "service":
        cmd += ["--service-name", args.service_name]
    else:
        cmd += ["--topic-name", args.topic_name]
    if dry_run:
        cmd.append("--dry-run")
    return cmd


def build_report(args: argparse.Namespace) -> tuple[dict, int]:
    report = _default_report(args)
    selected_targets = _load_targets(args.targets)
    report["selected_targets"] = selected_targets
    report["target_count"] = len(selected_targets)

    final_send_flags = args.require_active_runtime and args.manual_enable_execution and args.execute and args.confirm_runtime_send
    if args.execute and not args.manual_enable_execution:
        report["result"] = {"status": "blocked"}
        report["warnings"].append("Execution blocked: --execute requires --manual-enable-execution.")
        return report, 2
    if args.confirm_runtime_send and (not args.execute or not args.manual_enable_execution):
        report["result"] = {"status": "blocked"}
        report["warnings"].append("Execution blocked: --confirm-runtime-send requires --manual-enable-execution and --execute.")
        return report, 2
    if args.skip_send_dry_run_validation and not final_send_flags:
        report["result"] = {"status": "blocked"}
        report["warnings"].append("Execution blocked: --skip-send-dry-run-validation is only allowed with full guarded send flags.")
        return report, 2

    # payload source
    if args.runtime_payload:
        payload_path = args.runtime_payload
        if not payload_path.exists():
            report["result"] = {"status": "failed_safely"}
            report["warnings"].append(f"Provided payload does not exist: {payload_path}")
            return report, 2
        report["payload"] = {"path": str(payload_path), "source": "provided"}
    else:
        payload_path = args.payload_output if args.payload_output else Path(tempfile.gettempdir()) / "ur5_2f_sorting_runtime_bridge_payload.json"
        _generate_payload(payload_path)
        report["payload"] = {"path": str(payload_path), "source": "generated"}

    replay_script = _find_replay_script()
    dry_run_cmd = _build_replay_cmd(replay_script, payload_path, args, dry_run=True)
    report["payload_validation"]["command"] = dry_run_cmd

    should_validate = not (args.skip_send_dry_run_validation and final_send_flags)
    if should_validate:
        dry = _run_subprocess(dry_run_cmd)
        if dry.returncode != 0:
            report["payload_validation"]["dry_run_replay_status"] = "fail"
            report["result"] = {"status": "payload_validation_failed"}
            report["warnings"].append("Dry-run replay validation failed; payload will not be sent.")
            if dry.stderr.strip():
                report["warnings"].append(dry.stderr.strip())
            return report, 2
        report["payload_validation"]["dry_run_replay_status"] = "pass"
    else:
        report["payload_validation"]["dry_run_replay_status"] = "skipped"
        report["warnings"].append("Dry-run validation was skipped by explicit override; sending may move the robot in active runtime.")

    if args.execute and args.manual_enable_execution and not args.confirm_runtime_send:
        report["result"] = {"status": "blocked"}
        report["warnings"].append("Execution blocked: --confirm-runtime-send is required before any runtime send.")
        return report, 2
    if args.manual_enable_execution and not args.execute:
        report["result"] = {"status": "ready_for_manual_runtime"}
        report["warnings"].append("Ready for manual runtime send. Final command may send to /grasp_requests and move the robot in active runtime.")
        report["warnings"].append("Next command: ros2 run ur5_2f_sorting_test manual_static_sorting_executor --require-active-runtime --manual-enable-execution --execute --confirm-runtime-send")
        return report, 0
    if not final_send_flags:
        report["result"] = {"status": "dry_run_only"}
        return report, 0

    checks, warnings = _runtime_checks(args.service_name, args.topic_name)
    report["runtime_checks"] = checks
    report["warnings"].extend(warnings)
    runtime_ready = checks["grasp_execution_node"] == "present" and checks["joint_states"] == "present" and checks["controller_manager"] == "present" and checks["ur5_arm_controller"] == "active"
    interface_ready = checks["grasp_requests_service"] == "present" if args.ros_interface == "service" else checks["grasp_tasks_topic"] == "present"
    if not (runtime_ready and interface_ready):
        report["result"] = {"status": "runtime_missing"}
        report["warnings"].append("Runtime missing or incomplete; safe block before send.")
        return report, 2

    send_cmd = _build_replay_cmd(replay_script, payload_path, args, dry_run=False)
    report["execution_attempted"] = True
    report["robot_motion_requested"] = True
    send = _run_subprocess(send_cmd)
    if send.returncode == 0:
        report["result"] = {"status": "sent_to_runtime"}
        return report, 0
    report["result"] = {"status": "runtime_send_failed"}
    if send.stderr.strip():
        report["warnings"].append(send.stderr.strip())
    return report, 2


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _print_text(report: dict) -> None:
    print(json.dumps(report, indent=2))


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    report, exit_code = build_report(args)
    if args.prepare_output is not None:
        _write_json(args.prepare_output, {"schema": "manual_static_sorting_prepare_output/v1", "scene_package": SCENE_PACKAGE, "selected_targets": report["selected_targets"], "target_count": report["target_count"]})
    if args.output is not None:
        _write_json(args.output, report)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        _print_text(report)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
