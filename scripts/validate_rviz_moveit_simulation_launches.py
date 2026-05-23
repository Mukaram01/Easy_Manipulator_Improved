#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shlex
import signal
import subprocess
import sys
from pathlib import Path
from typing import Any

DEFAULT_JSON_OUTPUT = Path("build/workcell_studio/rviz_moveit_simulation_launch_report.json")
DEFAULT_TIMEOUT_SEC = 45
TAIL_CHARS = 5000

REAL_HARDWARE_TOKENS = [
    "use_fake_hardware:=false",
    "fake_hardware:=false",
    "use_mock_hardware:=false",
    "hardware_mode:=real",
    "driver_mode:=real",
    "hardware_driver",
    "ur_robot_driver",
    "realsense",
    "ethercat",
    "canopen",
]

PASS_EVIDENCE_PATTERNS = {
    "robot_description": ["robot_description"],
    "robot_state_publisher": ["robot_state_publisher"],
    "move_group": ["move_group"],
    "fake_controller": ["fake", "controller_manager", "ros2_control"],
}

WARN_EVIDENCE_PATTERNS = {
    "rviz": ["rviz"],
}

BLOCKER_PATTERNS = {
    "missing_launch_or_package": ["package '", "not found", "launch file", "does not exist"],
    "xacro_urdf_failure": ["xacro", "urdf", "error", "failed"],
    "controller_manager_crash": ["controller_manager", "segmentation fault", "crash", "fatal"],
    "parse_failure": ["traceback", "syntaxerror", "unable to parse"],
    "real_driver_attempt": ["ur_robot_driver", "real hardware", "hardware interface", "ethercat", "canopen"],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate fake-hardware RViz/MoveIt simulation launches for scenes.")
    parser.add_argument("--scene", help="Single scene name under scenes/<scene>/")
    parser.add_argument("--all", action="store_true", dest="run_all", help="Run against all discovered scenes")
    parser.add_argument("--timeout-sec", type=int, default=DEFAULT_TIMEOUT_SEC)
    parser.add_argument("--headless", action="store_true", help="Imply launch_rviz:=false")
    parser.add_argument("--launch-rviz", action="store_true", help="Manual GUI mode launch_rviz:=true")
    parser.add_argument("--json-output", type=Path, default=DEFAULT_JSON_OUTPUT)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def discover_scenes(root: Path) -> list[dict[str, Any]]:
    scenes_dir = root / "scenes"
    if not scenes_dir.is_dir():
        return []
    discovered: list[dict[str, Any]] = []
    for path in sorted(scenes_dir.iterdir()):
        if not path.is_dir():
            continue
        launch_file = path / "launch" / "demo.launch.py"
        discovered.append({
            "scene": path.name,
            "scene_path": path,
            "launch_file": launch_file,
            "supported": launch_file.is_file(),
        })
    return discovered


def collect_evidence(blob: str) -> tuple[list[str], list[str], list[str]]:
    low = blob.lower()
    evidence: list[str] = []
    warnings: list[str] = []
    blockers: list[str] = []

    for key, patterns in PASS_EVIDENCE_PATTERNS.items():
        if all(p in low for p in patterns):
            evidence.append(key)

    for key, patterns in WARN_EVIDENCE_PATTERNS.items():
        if all(p in low for p in patterns):
            evidence.append(key)

    for key, patterns in BLOCKER_PATTERNS.items():
        if all(p in low for p in patterns):
            blockers.append(key)

    if "rviz" not in low:
        warnings.append("rviz_not_seen_in_logs")

    return evidence, blockers, warnings


def safety_violations(command: str) -> list[str]:
    lower = command.lower()
    return [token for token in REAL_HARDWARE_TOKENS if token in lower]


def _run_truth_preview_guard(scene_name: str, repo_root: Path, workspace_root: Path) -> tuple[bool, dict[str, Any]]:
    cmd = [
        sys.executable,
        str(repo_root / "scripts" / "validate_rviz_truth_preview_command.py"),
        scene_name,
        "--repo-root",
        str(repo_root),
        "--workspace-root",
        str(workspace_root),
        "--check-ros2-prefix",
        "--json",
    ]
    proc = subprocess.run(cmd, cwd=repo_root, capture_output=True, text=True)
    payload: dict[str, Any] = {
        "status": "FAIL",
        "blockers": [f"truth_preview_guard_invocation_failed: exit={proc.returncode}"],
        "stdout": proc.stdout.strip(),
        "stderr": proc.stderr.strip(),
    }
    if proc.stdout.strip():
        try:
            payload = json.loads(proc.stdout)
        except json.JSONDecodeError:
            payload["blockers"].append("truth_preview_guard_invalid_json")
    ok = proc.returncode == 0 and str(payload.get("status", "")).upper() == "PASS"
    return ok, payload


def run_scene(scene_name: str, launch_file: Path, timeout_sec: int, launch_rviz: bool, dry_run: bool, repo_root: Path, workspace_root: Path) -> dict[str, Any]:
    command = f"ros2 launch {scene_name} demo.launch.py use_fake_hardware:=true launch_rviz:={'true' if launch_rviz else 'false'}"
    print(command)

    result: dict[str, Any] = {
        "scene": scene_name,
        "command": command,
        "status": "SKIP",
        "timeout": timeout_sec,
        "launch_file_path": str(launch_file),
        "use_fake_hardware": True,
        "launch_rviz": launch_rviz,
        "evidence": [],
        "blockers": [],
        "warnings": [],
        "stdout_tail": "",
        "stderr_tail": "",
        "truth_preview_guard": {},
    }

    if not launch_file.is_file():
        result["status"] = "SKIP"
        result["warnings"].append("unsupported_metadata: missing launch/demo.launch.py")
        return result

    violations = safety_violations(command)
    if violations:
        result["status"] = "FAIL"
        result["blockers"].append(f"safety_rejected: {', '.join(violations)}")
        return result

    guard_ok, guard_payload = _run_truth_preview_guard(scene_name, repo_root, workspace_root)
    result["truth_preview_guard"] = guard_payload
    if not guard_ok:
        result["status"] = "FAIL"
        guard_blockers = guard_payload.get("blockers", [])
        if isinstance(guard_blockers, list):
            result["blockers"].extend([f"truth_preview_guard: {str(item)}" for item in guard_blockers])
        else:
            result["blockers"].append("truth_preview_guard: failed")
        return result

    if dry_run:
        result["status"] = "SKIP"
        result["warnings"].append("dry_run")
        return result

    try:
        proc = subprocess.Popen(
            shlex.split(command),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            preexec_fn=os.setsid,
        )
        try:
            stdout, stderr = proc.communicate(timeout=max(1, timeout_sec))
            timed_out = False
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            stdout, stderr = proc.communicate(timeout=5)
            timed_out = True

        combined = f"{stdout}\n{stderr}"
        evidence, blockers, warnings = collect_evidence(combined)
        result["evidence"] = evidence
        result["blockers"] = blockers
        result["warnings"] = warnings
        result["stdout_tail"] = stdout[-TAIL_CHARS:]
        result["stderr_tail"] = stderr[-TAIL_CHARS:]

        if timed_out and {"robot_description", "robot_state_publisher", "move_group", "fake_controller"}.issubset(set(evidence)) and not blockers:
            result["status"] = "PASS"
        elif proc.returncode == 0 and {"robot_description", "robot_state_publisher", "move_group"}.issubset(set(evidence)) and not blockers:
            result["status"] = "PASS"
        elif evidence and not blockers:
            result["status"] = "WARN"
            if timed_out:
                result["warnings"].append("timeout_before_full_readiness_confirmation")
        else:
            result["status"] = "FAIL"
            if proc.returncode not in (0, None):
                result["blockers"].append(f"early_exit_code:{proc.returncode}")
            if timed_out and not evidence:
                result["blockers"].append("timeout_no_readiness_evidence")

    except FileNotFoundError:
        result["status"] = "FAIL"
        result["blockers"].append("missing ros2 executable / environment not sourced")
    except Exception as exc:  # broad catch for robust reporting
        result["status"] = "FAIL"
        result["blockers"].append(f"launch_execution_exception: {exc}")

    return result


def main() -> int:
    args = parse_args()

    if bool(args.scene) == bool(args.run_all):
        raise SystemExit("Exactly one of --scene or --all must be provided")

    launch_rviz = True if args.launch_rviz else False
    if args.headless:
        launch_rviz = False

    repo_root = Path(__file__).resolve().parents[1]
    workspace_root = repo_root
    scenes = discover_scenes(repo_root)
    scene_map = {item["scene"]: item for item in scenes}

    targets: list[dict[str, Any]]
    if args.scene:
        if args.scene not in scene_map:
            raise SystemExit(f"Unknown scene: {args.scene}")
        targets = [scene_map[args.scene]]
    else:
        targets = scenes

    results = [
        run_scene(
            scene_name=item["scene"],
            launch_file=item["launch_file"],
            timeout_sec=args.timeout_sec,
            launch_rviz=launch_rviz,
            dry_run=args.dry_run,
            repo_root=repo_root,
            workspace_root=workspace_root,
        )
        for item in targets
    ]

    args.json_output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema": "rviz_moveit_simulation_launch_report/v1",
        "dry_run": args.dry_run,
        "timeout_sec": args.timeout_sec,
        "launch_rviz": launch_rviz,
        "results": results,
    }
    args.json_output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    statuses = {entry["status"] for entry in results}
    if "FAIL" in statuses:
        return 1
    if statuses == {"SKIP"}:
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
