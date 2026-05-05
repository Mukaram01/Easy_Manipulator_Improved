#!/usr/bin/env python3
"""Validate manual_static_sorting_executor safety-gated behavior."""

import json
from pathlib import Path
import subprocess
import sys


def _run(script: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(script), *args], check=check, capture_output=True, text=True)


def main() -> int:
    scene_root = Path(__file__).resolve().parents[1]
    script_path = scene_root / "scripts" / "manual_static_sorting_executor.py"

    default = _run(script_path, "--json")
    report = json.loads(default.stdout)
    if report.get("schema") != "manual_static_sorting_executor/v1":
        raise AssertionError("schema mismatch")
    if report.get("robot_motion_requested") is not False:
        raise AssertionError("robot_motion_requested must be false")
    if report.get("execution_attempted") is not False:
        raise AssertionError("execution_attempted must be false")
    if report.get("execution_enabled") is not False:
        raise AssertionError("execution_enabled must be false")

    launch_command = " ".join(report.get("launch_command", []))
    expected_launch = "ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_2f_sorting_test launch_rviz:=true"
    if launch_command != expected_launch:
        raise AssertionError("launch command mismatch")

    execute_blocked = _run(script_path, "--json", "--execute", check=False)
    if execute_blocked.returncode == 0:
        raise AssertionError("--execute without manual enable must fail")
    execute_report = json.loads(execute_blocked.stdout)
    if execute_report.get("result", {}).get("status") != "blocked":
        raise AssertionError("--execute must be blocked")

    manual_enabled = _run(script_path, "--json", "--manual-enable-execution")
    manual_report = json.loads(manual_enabled.stdout)
    if manual_report.get("execution_enabled") is not True:
        raise AssertionError("manual enable should set execution_enabled true")
    if manual_report.get("execution_attempted") is not False:
        raise AssertionError("manual enable alone must not attempt execution")

    selected = _run(script_path, "--json", "--target", "item_red")
    selected_report = json.loads(selected.stdout)
    selected_targets = selected_report.get("selected_targets", [])
    if len(selected_targets) != 1 or selected_targets[0].get("object_id") != "item_red":
        raise AssertionError("target filtering failed for item_red")

    invalid = _run(script_path, "--json", "--target", "not_real", check=False)
    if invalid.returncode == 0:
        raise AssertionError("invalid target must fail")


    import importlib.util
    spec = importlib.util.spec_from_file_location("m", script_path)
    m = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(m)
    sample = """joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster  active
ur5_arm_controller      joint_trajectory_controller/JointTrajectoryController  active
"""
    if m._parse_controller_state(sample) != "active":
        raise AssertionError("controller parser failed to detect active state")

    coloured_sample = """\x1b[92mjoint_state_broadcaster\x1b[0m joint_state_broadcaster/JointStateBroadcaster          \x1b[92mactive\x1b[0m
\x1b[92mur5_arm_controller     \x1b[0m joint_trajectory_controller/JointTrajectoryController  \x1b[92mactive\x1b[0m
"""
    if m._parse_controller_state(coloured_sample) != "active":
        raise AssertionError("controller parser failed to detect ANSI-coloured active state")

    runtime_required = _run(script_path, "--json", "--require-active-runtime", check=False)
    runtime_report = json.loads(runtime_required.stdout)
    if runtime_required.returncode == 0:
        raise AssertionError("runtime should be missing in test environment")
    if runtime_report.get("result", {}).get("status") != "runtime_missing":
        raise AssertionError("require-active-runtime must safely report runtime_missing")

    return 0


if __name__ == "__main__":
    sys.exit(main())
