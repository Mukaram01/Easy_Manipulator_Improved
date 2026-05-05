#!/usr/bin/env python3
"""Manually gated static sorting execution adapter with safe dry-run defaults."""

from __future__ import annotations

import argparse
import importlib.machinery
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

SCHEMA = "manual_static_sorting_executor/v1"
SCENE_PACKAGE = "ur5_2f_sorting_test"
LAUNCH_COMMAND = [
    "ros2",
    "launch",
    "run_grasp_execution",
    "grasp_execution.launch.py",
    "scene_package:=ur5_2f_sorting_test",
    "launch_rviz:=true",
]


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


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON output")
    parser.add_argument("--output", type=Path, help="Write machine-readable JSON output to file")
    parser.add_argument("--prepare-output", type=Path, help="Write selected static preparation targets JSON to file")
    parser.add_argument("--require-active-runtime", action="store_true", help="Require active runtime checks to pass")
    parser.add_argument("--manual-enable-execution", action="store_true", help="Explicitly enable execution mode")
    parser.add_argument("--execute", action="store_true", help="Request execution attempt (still safety-gated)")
    parser.add_argument("--target", action="append", dest="targets", help="Optional object_id filter; repeat for multiple")
    return parser.parse_args(argv)


def _initial_runtime_checks() -> dict:
    return {
        "checked": False,
        "grasp_execution_node": "unknown",
        "joint_states": "unknown",
        "controller_manager": "unknown",
        "ur5_arm_controller": "unknown",
    }


def _run_ros2(args: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(["ros2", *args], check=False, capture_output=True, text=True, timeout=4)


def _parse_controller_state(list_controllers_stdout: str, controller_name: str = "ur5_arm_controller") -> str:
    for raw_line in list_controllers_stdout.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        columns = line.split()
        if not columns or columns[0] != controller_name:
            continue
        if len(columns) >= 3:
            return columns[2].strip().lower()
        return "inactive"
    return "missing"


def _runtime_checks() -> tuple[dict, list[str]]:
    checks = _initial_runtime_checks()
    warnings: list[str] = []
    checks["checked"] = True

    try:
        node_out = _run_ros2(["node", "list"])
        nodes = set(line.strip() for line in node_out.stdout.splitlines() if line.strip()) if node_out.returncode == 0 else set()
        checks["grasp_execution_node"] = "present" if "/grasp_execution_node" in nodes else "missing"

        topic_out = _run_ros2(["topic", "list"])
        topics = set(line.strip() for line in topic_out.stdout.splitlines() if line.strip()) if topic_out.returncode == 0 else set()
        checks["joint_states"] = "present" if "/joint_states" in topics else "missing"

        service_out = _run_ros2(["service", "list"])
        services = set(line.strip() for line in service_out.stdout.splitlines() if line.strip()) if service_out.returncode == 0 else set()
        checks["controller_manager"] = "present" if any("controller_manager" in svc for svc in services) else "missing"

        ctrl_out = _run_ros2(["control", "list_controllers"])
        if ctrl_out.returncode == 0:
            checks["ur5_arm_controller"] = _parse_controller_state(ctrl_out.stdout)
        else:
            checks["ur5_arm_controller"] = "missing"
            warnings.append("ros2 control CLI unavailable; ur5_arm_controller status treated as missing.")

    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        warnings.append(f"ROS runtime checks unavailable: {exc}")
        checks["grasp_execution_node"] = "missing"
        checks["joint_states"] = "missing"
        checks["controller_manager"] = "missing"
        checks["ur5_arm_controller"] = "missing"

    return checks, warnings


def _load_targets(selected: list[str] | None) -> list[dict]:
    manifest = runtime_plan.load_manifest(runtime_plan.find_manifest_path())
    plan = runtime_plan.build_runtime_plan(manifest)
    all_targets = []
    for step in plan.get("steps", []):
        if step.get("type") != "pick":
            continue
        all_targets.append(
            {
                "object_id": step.get("object_id"),
                "object_frame": step.get("object_frame"),
                "pick_hint": step.get("pick_hint"),
                "destination_id": step.get("destination_id"),
            }
        )
    by_id = {t.get("object_id"): t for t in all_targets}
    if not selected:
        return all_targets
    missing = [target for target in selected if target not in by_id]
    if missing:
        raise ManualExecutorError(f"Unknown target object_id(s): {', '.join(missing)}")
    return [by_id[target] for target in selected]


def _default_report() -> dict:
    return {
        "schema": SCHEMA,
        "scene_package": SCENE_PACKAGE,
        "mode": "dry_run",
        "execution_enabled": False,
        "execute_requested": False,
        "execution_attempted": False,
        "robot_motion_requested": False,
        "target_count": 0,
        "selected_targets": [],
        "launch_command": LAUNCH_COMMAND,
        "runtime_checks": _initial_runtime_checks(),
        "result": {"status": "dry_run_only"},
        "warnings": [],
    }


def build_report(args: argparse.Namespace) -> tuple[dict, int]:
    report = _default_report()
    report["execution_enabled"] = bool(args.manual_enable_execution)
    report["execute_requested"] = bool(args.execute)

    try:
        selected_targets = _load_targets(args.targets)
    except ManualExecutorError as exc:
        report["result"] = {"status": "blocked"}
        report["warnings"].append(str(exc))
        return report, 2

    report["selected_targets"] = selected_targets
    report["target_count"] = len(selected_targets)

    if args.execute and not args.manual_enable_execution:
        report["mode"] = "execution_requested"
        report["result"] = {"status": "blocked"}
        report["warnings"].append("Execution blocked: --execute requires --manual-enable-execution.")
        return report, 2

    if args.manual_enable_execution and not args.execute:
        report["mode"] = "manual_enabled"
        report["result"] = {"status": "ready_for_manual_runtime"}
        report["warnings"].append("Execution is enabled, but robot motion is still blocked until you also pass --execute.")
        report["warnings"].append("Next explicit command: ros2 run ur5_2f_sorting_test manual_static_sorting_executor --manual-enable-execution --execute")
        report["warnings"].append("Generate runtime-shaped payload: ros2 run ur5_2f_sorting_test generate_static_sorting_runtime_bridge_payload --output /tmp/ur5_2f_sorting_runtime_bridge_payload.json")
        report["warnings"].append("Validate payload only (no send): python3 scripts/replay_emd_bridge_payload.py --payload /tmp/ur5_2f_sorting_runtime_bridge_payload.json --scene-package ur5_2f_sorting_test --dry-run")

    must_check_runtime = args.require_active_runtime or args.execute
    if must_check_runtime:
        checks, warnings = _runtime_checks()
        report["runtime_checks"] = checks
        report["warnings"].extend(warnings)
        runtime_ready = (
            checks["grasp_execution_node"] == "present"
            and checks["joint_states"] == "present"
            and checks["controller_manager"] == "present"
            and checks["ur5_arm_controller"] == "active"
        )
        if not runtime_ready:
            report["result"] = {"status": "runtime_missing"}
            report["warnings"].append(
                "Launch the scene first with: ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_2f_sorting_test launch_rviz:=true"
            )
            return report, 2

    if args.execute and args.manual_enable_execution:
        report["mode"] = "execution_requested"
        report["result"] = {"status": "execution_api_missing"}
        report["warnings"].append("Execution API call was intentionally skipped: no documented dry-run/test-safe execution API was found.")
        report["warnings"].append("Future manual send command only (not executed): python3 scripts/replay_emd_bridge_payload.py --payload /tmp/ur5_2f_sorting_runtime_bridge_payload.json --scene-package ur5_2f_sorting_test --ros-interface service --service-name grasp_requests")
        report["warnings"].append("No robot motion was requested. This adapter will not guess or call unknown services/actions.")
        return report, 2

    return report, 0


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _print_text(report: dict) -> None:
    print("Manual static sorting executor report")
    print(f"scene_package: {report['scene_package']}")
    print(f"schema: {report['schema']}")
    print(f"mode: {report['mode']}")
    print(f"execution_enabled: {report['execution_enabled']}")
    print(f"execute_requested: {report['execute_requested']}")
    print(f"execution_attempted: {report['execution_attempted']}")
    print(f"robot_motion_requested: {report['robot_motion_requested']}")
    print(f"target_count: {report['target_count']}")
    print("selected_targets:")
    for target in report["selected_targets"]:
        print(f"  - {target['object_id']} -> {target['destination_id']}")
    print("launch_command:")
    print("  " + " ".join(report["launch_command"]))
    print("available_ros_checks:")
    print("  - ros2 node list")
    print("  - ros2 topic list")
    print("  - ros2 service list")
    print("  - ros2 control list_controllers")
    print("runtime_checks:")
    for key, value in report["runtime_checks"].items():
        print(f"  {key}: {value}")
    print(f"result_status: {report['result']['status']}")
    print("warnings:")
    for warning in report["warnings"]:
        print(f"  - {warning}")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    report, exit_code = build_report(args)

    if args.prepare_output is not None:
        _write_json(
            args.prepare_output,
            {
                "schema": "manual_static_sorting_prepare_output/v1",
                "scene_package": SCENE_PACKAGE,
                "selected_targets": report["selected_targets"],
                "target_count": report["target_count"],
            },
        )

    if args.output is not None:
        _write_json(args.output, report)

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        _print_text(report)

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
