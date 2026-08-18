#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Callable

import yaml  # type: ignore

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from supported_scene_catalog import SupportedSceneEntry, default_catalog_path, load_supported_scene_catalog

DEFAULT_JSON_OUTPUT = Path("build/workcell_studio/rviz_moveit_simulation_launch_report.json")
DEFAULT_TIMEOUT_SEC = 45
TAIL_CHARS = 5000
PASS = "PASS"
FAIL = "FAIL"
BLOCKED = "BLOCKED"

REAL_HARDWARE_TOKENS = [
    "use_fake_hardware:=false",
    "fake_hardware:=false",
    "use_mock_hardware:=false",
    "hardware_mode:=real",
    "driver_mode:=real",
    "hardware_driver",
    "ur_robot_driver",
    "ethercat",
    "canopen",
]
MOVEIT_REQUIRED_CAPABILITY = "fake_hardware_launch"
REQUIRED_SCENES = {"ur5_2f_test", "ur5_3f_test", "suction_test", "ur10_2f_test", "ur3_suction_test"}
CANONICAL_UR5_2F_CONTROLLERS = (
    "joint_state_broadcaster",
    "ur5_arm_controller",
    "ur5_gripper_controller",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate supported-scene fake-hardware MoveIt acceptance.")
    parser.add_argument("--scene", help="Single supported scene name")
    parser.add_argument("--all", action="store_true", dest="run_all", help="Run against all supported scenes that require MoveIt")
    parser.add_argument("--catalog", type=Path, default=None, help="Supported scene catalog path")
    parser.add_argument("--timeout-sec", type=int, default=DEFAULT_TIMEOUT_SEC)
    parser.add_argument("--headless", action="store_true", help="Run bounded smoke without RViz")
    parser.add_argument("--launch-rviz", action="store_true", help="Pass launch_rviz:=true when a scene declares that argument")
    parser.add_argument("--json-output", type=Path, default=DEFAULT_JSON_OUTPUT)
    parser.add_argument("--dry-run", action="store_true", help="Run static/default/model checks only")
    return parser.parse_args()


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _string_list(value: Any) -> list[str]:
    return [str(v) for v in value] if isinstance(value, list) else []


def scene_requires_moveit(entry: SupportedSceneEntry) -> bool:
    return MOVEIT_REQUIRED_CAPABILITY in entry.required_capabilities


def _declared_launch_args(launch_text: str) -> dict[str, str | None]:
    args: dict[str, str | None] = {}
    for match in re.finditer(r'DeclareLaunchArgument\(\s*["\']([^"\']+)["\'](?P<body>.*?)\)', launch_text, re.S):
        body = match.group("body")
        default = re.search(r'default_value\s*=\s*["\']([^"\']+)["\']', body)
        args[match.group(1)] = default.group(1) if default else None
    return args


def validate_launch_contract(entry: SupportedSceneEntry, scene_dir: Path, launch_file: Path, launch_rviz: bool) -> tuple[list[str], list[str], dict[str, Any]]:
    failures: list[str] = []
    evidence: list[str] = []
    meta: dict[str, Any] = {"declared_launch_arguments": {}}
    if not launch_file.is_file():
        return ["missing launch/demo.launch.py"], evidence, meta
    text = launch_file.read_text(encoding="utf-8")
    args = _declared_launch_args(text)
    meta["declared_launch_arguments"] = args
    fake_default = args.get("use_fake_hardware")
    if fake_default is None:
        failures.append("launch argument use_fake_hardware is missing")
    elif str(fake_default).lower() != "true":
        failures.append(f"use_fake_hardware default must be true; got {fake_default!r}")
    else:
        evidence.append("fake_hardware_default_true")
    if "use_fake_hardware" not in entry.fake_hardware_launch_command or "use_fake_hardware:=true" not in entry.fake_hardware_launch_command:
        failures.append("catalog fake_hardware_launch_command must pass use_fake_hardware:=true")
    if any(tok in entry.fake_hardware_launch_command.lower() for tok in REAL_HARDWARE_TOKENS):
        failures.append("catalog launch command contains real-hardware token")
    if re.search(r"allow_trajectory_execution[\"']?\s*[:=]\s*True", text):
        failures.append("launch enables trajectory execution; acceptance must not permit motion")
    if "allow_trajectory_execution" in text and re.search(r"FollowJointTrajectory|joint_trajectory_controller", text):
        evidence.append("trajectory_execution_disabled_checked")
    if launch_rviz and "launch_rviz" not in args:
        failures.append("--launch-rviz requested but launch file does not declare launch_rviz")
    return failures, evidence, meta


def validate_scene_model(entry: SupportedSceneEntry, scene_dir: Path, launch_file: Path) -> tuple[list[str], list[str], dict[str, Any]]:
    failures: list[str] = []
    evidence: list[str] = []
    manifest = _load_yaml(scene_dir / "scene_manifest.yaml")
    cell = _load_yaml(scene_dir / "cell_definition.yaml")
    launch_text = launch_file.read_text(encoding="utf-8") if launch_file.exists() else ""
    robot = manifest.get("robot") if isinstance(manifest.get("robot"), dict) else {}
    tool = manifest.get("end_effector") if isinstance(manifest.get("end_effector"), dict) else {}
    cell_robot = cell.get("robot") if isinstance(cell.get("robot"), dict) else {}
    cell_tool = cell.get("end_effector") if isinstance(cell.get("end_effector"), dict) else {}
    planning_group = robot.get("planning_group") or cell_robot.get("planning_group")
    base_link = robot.get("base_frame") or cell_robot.get("base_frame") or "base_link"
    tool_root = tool.get("grasp_frame") or tool.get("mount_link") or cell_tool.get("grasp_frame") or cell_robot.get("tool_link")
    joints = re.findall(r'["\']([A-Za-z0-9_]+_joint)["\']', launch_text)
    arm_joints = [j for j in joints if not j.startswith("gripper_")]
    srdf_xacro = scene_dir / "urdf" / "arm_hand.srdf.xacro"
    srdf_text = srdf_xacro.read_text(encoding="utf-8") if srdf_xacro.exists() else ""
    urdf_xacro = scene_dir / "urdf" / "scene.urdf.xacro"
    urdf_text = urdf_xacro.read_text(encoding="utf-8") if urdf_xacro.exists() else ""
    if not planning_group:
        failures.append("missing planning_group in scene_manifest.yaml/cell_definition.yaml")
    elif planning_group not in srdf_text and planning_group not in launch_text and "xacro:include" not in srdf_text:
        failures.append(f"planning group {planning_group!r} not found in SRDF/launch metadata")
    else:
        evidence.append("planning_group_matches_metadata")
    if not tool_root:
        failures.append("missing tool planning metadata: grasp_frame/mount_link/tool_link")
    elif tool_root not in srdf_text and tool_root not in urdf_text and tool_root not in launch_text:
        failures.append(f"tool root/link {tool_root!r} not found in generated URDF/SRDF/launch metadata")
    else:
        evidence.append("tool_link_matches_metadata")
    if not base_link:
        failures.append("missing robot base frame metadata")
    elif base_link not in urdf_text and base_link not in launch_text:
        failures.append(f"base link {base_link!r} not found in generated URDF/launch metadata")
    else:
        evidence.append("base_link_matches_metadata")
    if not arm_joints:
        failures.append("no arm joint names found in launch controller metadata")
    elif srdf_text and "_joint" in srdf_text and not all(j in srdf_text for j in arm_joints[:6]):
        missing = [j for j in arm_joints[:6] if j not in srdf_text]
        failures.append(f"robot/SRDF joint mismatch: {', '.join(missing)}")
    else:
        evidence.append("controller_joints_match_srdf")
    return failures, evidence, {"planning_group": planning_group, "base_link": base_link, "tool_root": tool_root, "controller_joints": sorted(set(joints))}


def _text(value: str | bytes | None) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


def _run_ros(args: list[str], timeout: int) -> tuple[int, str, str]:
    try:
        proc = subprocess.run(args, capture_output=True, text=True, timeout=max(1, timeout))
        return proc.returncode, proc.stdout or "", proc.stderr or ""
    except subprocess.TimeoutExpired as exc:
        return 124, _text(exc.stdout), _text(exc.stderr)


def _wait_for_ros_check(
    args: list[str],
    timeout_sec: int,
    predicate: Callable[[str], bool] | None = None,
) -> tuple[int, str, str]:
    """Poll a ROS CLI check so launch startup races do not become false failures."""
    deadline = time.monotonic() + max(1, timeout_sec)
    last: tuple[int, str, str] = (124, "", "")
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return last
        attempt_timeout = max(1, min(3, int(remaining + 0.999)))
        last = _run_ros(args, attempt_timeout)
        rc, out, _ = last
        if rc == 0 and (predicate is None or predicate(out)):
            return last
        if time.monotonic() >= deadline:
            return last
        time.sleep(0.25)


def _controller_is_active(output: str, controller_name: str) -> bool:
    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith(controller_name) and re.search(r"\bactive\b", stripped):
            return True
    return False


def _expected_controllers(scene_name: str) -> tuple[str, ...]:
    return CANONICAL_UR5_2F_CONTROLLERS if scene_name == "ur5_2f_test" else ()


def run_headless_smoke(scene_name: str, command: str, timeout_sec: int) -> tuple[str, list[str], list[str], dict[str, Any]]:
    if shutil.which("ros2") is None:
        return BLOCKED, ["ros2 executable not found; source ROS 2 Humble and the workspace install/setup.bash"], [], {"launched": False, "terminated": True}
    proc: subprocess.Popen[str] | None = None
    diagnostics: dict[str, Any] = {"launched": False, "terminated": False, "checks": {}}
    blockers: list[str] = []
    evidence: list[str] = []
    try:
        proc = subprocess.Popen(shlex.split(command), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid)
        diagnostics["launched"] = True

        startup_budget = max(4, min(12, timeout_sec // 3 or 4))
        node_predicate = lambda out: "robot_state_publisher" in out and "move_group" in out
        nodes = _wait_for_ros_check(["ros2", "node", "list"], startup_budget, node_predicate)
        diagnostics["checks"]["nodes"] = {"returncode": nodes[0], "stdout_tail": nodes[1][-1000:], "stderr_tail": nodes[2][-1000:]}

        per_check = max(2, min(6, timeout_sec // 7 or 2))
        ordinary_checks = {
            "robot_description": ["ros2", "param", "get", "/move_group", "robot_description"],
            "robot_description_semantic": ["ros2", "param", "get", "/move_group", "robot_description_semantic"],
            "joint_states": ["ros2", "topic", "list"],
            "tf": ["ros2", "topic", "echo", "/tf_static", "--once"],
        }
        for key, cmd in ordinary_checks.items():
            rc, out, err = _wait_for_ros_check(cmd, per_check)
            diagnostics["checks"][key] = {"returncode": rc, "stdout_tail": out[-1000:], "stderr_tail": err[-1000:]}

        expected_controllers = _expected_controllers(scene_name)
        controller_budget = max(5, min(12, timeout_sec // 3 or 5))
        controller_predicate = None
        if expected_controllers:
            controller_predicate = lambda out: all(_controller_is_active(out, name) for name in expected_controllers)
        controllers = _wait_for_ros_check(
            ["ros2", "control", "list_controllers", "-c", "/controller_manager", "--spin-time", "1"],
            controller_budget,
            controller_predicate,
        )
        diagnostics["checks"]["controllers"] = {
            "returncode": controllers[0],
            "stdout_tail": controllers[1][-1000:],
            "stderr_tail": controllers[2][-1000:],
            "expected_active": list(expected_controllers),
        }

        nodes_out = diagnostics["checks"]["nodes"]["stdout_tail"]
        if "robot_state_publisher" in nodes_out:
            evidence.append("robot_state_publisher_available")
        else:
            blockers.append("robot_state_publisher node did not become available")
        if "move_group" in nodes_out:
            evidence.append("move_group_available")
        else:
            blockers.append("move_group node did not become available")
        if diagnostics["checks"]["robot_description"]["returncode"] == 0:
            evidence.append("robot_description_loaded")
        else:
            blockers.append("robot_description parameter not available on /move_group")
        if diagnostics["checks"]["robot_description_semantic"]["returncode"] == 0:
            evidence.append("robot_description_semantic_loaded")
        else:
            blockers.append("robot_description_semantic parameter not available on /move_group")
        joint_topics = diagnostics["checks"]["joint_states"]["stdout_tail"]
        if "/joint_states" in joint_topics or f"/{scene_name}/joint_states" in joint_topics:
            evidence.append("joint_states_topic_available")
        else:
            blockers.append("/joint_states topic did not appear")
        if diagnostics["checks"]["tf"]["returncode"] == 0:
            evidence.append("tf_available")
        else:
            blockers.append("TF did not provide a base/tool transform sample")

        controller_rc = diagnostics["checks"]["controllers"]["returncode"]
        controller_out = diagnostics["checks"]["controllers"]["stdout_tail"]
        if controller_rc == 0:
            if expected_controllers:
                inactive = [name for name in expected_controllers if not _controller_is_active(controller_out, name)]
                if inactive:
                    blockers.append("required fake controllers are not active: " + ", ".join(inactive))
                else:
                    evidence.append("fake_controllers_listed")
                    evidence.extend(f"{name}_active" for name in expected_controllers)
            else:
                evidence.append("fake_controllers_listed")
        elif controller_rc == 124:
            blockers.append("controller_manager/list_controllers did not become ready before timeout")
        else:
            blockers.append("fake controllers could not be listed")

        return (FAIL if blockers else PASS), blockers, evidence, diagnostics
    except (FileNotFoundError, PermissionError, OSError) as exc:
        return BLOCKED, [f"fake-hardware launch could not start: {exc}"], evidence, diagnostics
    finally:
        if proc is not None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.communicate(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
                try:
                    proc.communicate(timeout=2)
                except Exception:
                    pass
            diagnostics["terminated"] = True


def command_for_scene(entry: SupportedSceneEntry, launch_rviz: bool) -> str:
    command = entry.fake_hardware_launch_command
    if not launch_rviz and "launch_rviz:=" in command:
        command = re.sub(r"launch_rviz:=\S+", "launch_rviz:=false", command)
    elif launch_rviz and "launch_rviz:=" in command:
        command = re.sub(r"launch_rviz:=\S+", "launch_rviz:=true", command)
    return command


def run_scene(entry: SupportedSceneEntry, timeout_sec: int, launch_rviz: bool, dry_run: bool, repo_root: Path) -> dict[str, Any]:
    scene_dir = repo_root / entry.scene_path
    launch_file = scene_dir / "launch" / "demo.launch.py"
    command = command_for_scene(entry, launch_rviz)
    result: dict[str, Any] = {"scene": entry.scene_name, "command": command, "status": PASS, "timeout": timeout_sec, "launch_file_path": str(launch_file), "use_fake_hardware": True, "launch_rviz": launch_rviz, "evidence": [], "blockers": [], "warnings": [], "model": {}, "launch_contract": {}, "smoke": {}}
    launch_failures, launch_evidence, launch_meta = validate_launch_contract(entry, scene_dir, launch_file, launch_rviz)
    model_failures, model_evidence, model_meta = validate_scene_model(entry, scene_dir, launch_file)
    result["launch_contract"] = launch_meta
    result["model"] = model_meta
    result["evidence"].extend(launch_evidence + model_evidence)
    result["blockers"].extend(launch_failures + model_failures)
    if result["blockers"]:
        result["status"] = FAIL
        return result
    if dry_run:
        result["status"] = PASS
        result["warnings"].append("dry_run_static_acceptance_only")
        return result
    status, smoke_blockers, smoke_evidence, diagnostics = run_headless_smoke(entry.scene_name, command, timeout_sec)
    result["status"] = status
    result["blockers"].extend(smoke_blockers)
    result["evidence"].extend(smoke_evidence)
    result["smoke"] = diagnostics
    return result


def load_targets(repo_root: Path, catalog_path: Path | None, scene: str | None, run_all: bool) -> list[SupportedSceneEntry]:
    catalog, entries, errors = load_supported_scene_catalog((catalog_path or default_catalog_path(repo_root)).resolve())
    if errors:
        raise SystemExit("Supported scene catalog is invalid:\n" + "\n".join(f"- {e}" for e in errors))
    moveit_entries = [e for e in entries if e.enabled and scene_requires_moveit(e)]
    missing = sorted(REQUIRED_SCENES - {e.scene_name for e in moveit_entries})
    if missing:
        raise SystemExit("Supported MoveIt scene registry missing required scenes: " + ", ".join(missing))
    if scene:
        matches = [e for e in moveit_entries if e.scene_name == scene]
        if not matches:
            raise SystemExit(f"Unknown or non-MoveIt scene: {scene}")
        return matches
    if run_all:
        return sorted(moveit_entries, key=lambda e: e.scene_name)
    raise SystemExit("Exactly one of --scene or --all must be provided")


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    targets = load_targets(repo_root, args.catalog, args.scene, args.run_all)
    launch_rviz = bool(args.launch_rviz and not args.headless)
    results = [run_scene(e, args.timeout_sec, launch_rviz, args.dry_run, repo_root) for e in targets]
    args.json_output.parent.mkdir(parents=True, exist_ok=True)
    payload = {"schema": "rviz_moveit_simulation_launch_report/v1", "dry_run": args.dry_run, "timeout_sec": args.timeout_sec, "launch_rviz": launch_rviz, "results": results}
    args.json_output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    for r in results:
        reason = "; ".join(r.get("blockers") or r.get("warnings") or ["ok"])
        print(f"{r['scene']}: {r['status']} - {reason}")
    return 1 if any(r["status"] != PASS for r in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
