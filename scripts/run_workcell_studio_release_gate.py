#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

PASS = "PASS"
FAIL = "FAIL"
BLOCKED = "BLOCKED"
NA = "NOT_APPLICABLE"
REQUIRED = {"static", "runtime", "demo"}

REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class CheckSpec:
    check_id: str
    title: str
    profiles: tuple[str, ...]
    command: list[str] | None = None
    required: bool = True
    timeout_sec: int = 120
    requires_ros: bool = False
    requires_qt: bool = False
    reason: str = ""


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run the integrated Workcell Studio simulation-first release gate.")
    p.add_argument("--profile", choices=sorted(REQUIRED), default="static")
    p.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    p.add_argument("--workspace-root", type=Path, default=None, help="ROS workspace root; defaults to repo parent when install/setup.bash exists, otherwise repo root.")
    p.add_argument("--output-dir", type=Path, default=Path("build/workcell_studio_release_gate"))
    p.add_argument("--json-output", type=Path, default=None)
    p.add_argument("--markdown-output", type=Path, default=None)
    p.add_argument("--timeout-sec", type=int, default=120)
    p.add_argument("--scene", default="ur5_2f_test", help="Canonical scene for demo/runtime focused checks.")
    p.add_argument("--dry-run-runtime", action="store_true", help="Emit runtime/demo commands but mark ROS/Qt execution checks BLOCKED unless dependencies are present.")
    return p.parse_args()


def _rel(path: Path, root: Path) -> str:
    try:
        return str(path.resolve().relative_to(root.resolve()))
    except ValueError:
        return str(path)


def _cmd_text(cmd: list[str]) -> str:
    return " ".join(subprocess.list2cmdline([part]) for part in cmd)


def _run(cmd: list[str], cwd: Path, timeout_sec: int) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, text=True, capture_output=True, timeout=timeout_sec, check=False)


def _git_sha(repo_root: Path) -> str:
    try:
        return _run(["git", "rev-parse", "HEAD"], repo_root, 10).stdout.strip() or "unknown"
    except Exception:
        return "unknown"


def _workspace_root(repo_root: Path, explicit: Path | None) -> Path:
    if explicit:
        return explicit
    if (repo_root.parent / "install" / "setup.bash").exists():
        return repo_root.parent
    return repo_root


def environment(repo_root: Path, workspace_root: Path) -> dict[str, Any]:
    setup = workspace_root / "install" / "setup.bash"
    return {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "commit_sha": _git_sha(repo_root),
        "repo_root": str(repo_root),
        "workspace_root": str(workspace_root),
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "display": os.environ.get("DISPLAY", ""),
        "qt_qpa_platform": os.environ.get("QT_QPA_PLATFORM", ""),
        "ros2_found": shutil.which("ros2") is not None,
        "colcon_found": shutil.which("colcon") is not None,
        "workspace_setup": str(setup),
        "workspace_setup_exists": setup.exists(),
    }


def specs(repo_root: Path, workspace_root: Path, out: Path, scene: str, timeout: int) -> list[CheckSpec]:
    py = sys.executable
    return [
        CheckSpec("supported_scene_reproducibility", "supported-scene reproducibility", ("static", "runtime", "demo"), [py, "scripts/validate_supported_scenes_readiness.py", "--repo-root", str(repo_root), "--workspace-root", str(workspace_root), "--json", "--skip-build", "--skip-launch-smoke", "--timeout-sec", str(timeout)], timeout_sec=timeout),
        CheckSpec("scene_regeneration_reproducibility", "scene generation reproducibility", ("static",), [py, "scripts/validate_all_scene_builder_reproducibility.py", "--json", "--strict", "--output", str(out / "scene_reproducibility.json")], timeout_sec=timeout),
        CheckSpec("schema_assets_ownership_robot_tool", "schema, assets, ownership, robot/tool readiness", ("static",), [py, "scripts/validate_capability_contracts.py", "catalog/capabilities", "--json", "--strict"], timeout_sec=timeout),
        CheckSpec("asset_catalog", "asset catalog readiness", ("static",), [py, "scripts/validate_workcell_asset_catalog.py", "--repo-root", str(repo_root), "--strict"], timeout_sec=timeout),
        CheckSpec("product_view_save_roundtrip", "Product View save round-trip", ("static", "runtime", "demo"), [py, "scripts/run_cross_scene_product_view_save_roundtrip_acceptance.py", "--output", str(out / "product_view_roundtrip.json")], timeout_sec=timeout),
        CheckSpec("browser_to_qt_scene_ready", "browser-to-Qt scene_ready contract", ("static", "runtime", "demo"), [py, "-m", "pytest", "tests/test_embedded_web3d_status_sync.py", "tests/test_scene3d_gui_smoke_json_output_contract.py"], timeout_sec=timeout),
        CheckSpec("perception_disabled_and_replay", "perception disabled and replay acceptance", ("static", "runtime", "demo"), [py, "-m", "pytest", "tests/test_workcell_builder_acceptance_perception_replay.py", "tests/test_perception_profile.py"], timeout_sec=timeout),
        CheckSpec("safety_defaults", "real hardware and automatic execution stay disabled", ("static", "runtime", "demo"), None),
        CheckSpec("workcell_builder_build", "Workcell Builder build health", ("runtime", "demo"), ["bash", "-lc", "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select workcell_builder"], timeout_sec=max(timeout, 300), requires_ros=True),
        CheckSpec("workcell_builder_launch", "Workcell Builder launch health", ("runtime", "demo"), [py, "scripts/check_workcell_builder_gui_acceptance.py"], timeout_sec=timeout, requires_qt=True),
        CheckSpec("fake_hardware_moveit", "fake-hardware MoveIt acceptance", ("runtime", "demo"), [py, "scripts/validate_rviz_moveit_simulation_launches.py", "--scene", scene, "--headless", "--json-output", str(out / "moveit_acceptance.json"), "--timeout-sec", str(timeout)], timeout_sec=timeout, requires_ros=True),
        CheckSpec("gripper_suction_plan_smoke", "gripper and suction pick/place plan-only smoke", ("runtime", "demo"), [py, "scripts/run_fake_pick_place_smoke_acceptance.py", "--scene", scene, "--json-output", str(out / "pick_place_smoke.json"), "--timeout-sec", str(timeout)], timeout_sec=timeout, requires_ros=True),
        CheckSpec("demo_e2e_simulation", "canonical end-to-end simulation acceptance", ("demo",), [py, "scripts/run_workcell_fake_hardware_smoke.py", "--scene-dir", f"scenes/{scene}", "--workspace", str(repo_root), "--run-launch", "--skip-colcon", "--headless", "--strict", "--timeout-seconds", str(timeout), "--print-summary"], timeout_sec=timeout, requires_ros=True),
    ]


def _unsafe_launch_default(text: str, name: str, unsafe: str) -> bool:
    normalized = text.replace('"', "'")
    for line in normalized.splitlines():
        if name in line and "default_value" in line and unsafe in line.lower():
            return True
    return False


def safety_check(repo_root: Path) -> tuple[str, list[str]]:
    launch_files = list((repo_root / "scenes").glob("*/launch/*.launch.py")) + list((repo_root / "launch").glob("*.launch.py"))
    blockers: list[str] = []
    for path in launch_files:
        text = path.read_text(encoding="utf-8", errors="ignore")
        if _unsafe_launch_default(text, "use_fake_hardware", "false"):
            blockers.append(f"{_rel(path, repo_root)} defaults use_fake_hardware to false")
        if _unsafe_launch_default(text, "auto_execute", "true"):
            blockers.append(f"{_rel(path, repo_root)} defaults auto_execute to true")
    return (PASS if not blockers else FAIL), blockers


def blocked_by_environment(spec: CheckSpec, env: dict[str, Any]) -> str | None:
    if spec.requires_ros:
        if env.get("ros_distro") and env.get("ros_distro") != "humble":
            return f"ROS_DISTRO is {env.get('ros_distro')!r}; source ROS 2 Humble."
        if not env.get("ros2_found"):
            return "ros2 executable not found; source /opt/ros/humble/setup.bash and the workspace."
        if not env.get("workspace_setup_exists"):
            return f"workspace setup file missing: {env.get('workspace_setup')}"
    if spec.requires_qt and not (env.get("display") or env.get("qt_qpa_platform") == "offscreen"):
        return "Qt display unavailable; set DISPLAY or QT_QPA_PLATFORM=offscreen for bounded smoke."
    return None


def run_gate(args: argparse.Namespace) -> dict[str, Any]:
    repo_root = args.repo_root.resolve()
    workspace_root = _workspace_root(repo_root, args.workspace_root).resolve()
    out = (repo_root / args.output_dir).resolve() if not args.output_dir.is_absolute() else args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    env = environment(repo_root, workspace_root)
    rows: list[dict[str, Any]] = []
    for spec in specs(repo_root, workspace_root, out, args.scene, args.timeout_sec):
        if args.profile not in spec.profiles:
            rows.append({"id": spec.check_id, "title": spec.title, "status": NA, "required": spec.required, "command": "", "reason": "not part of profile"})
            continue
        if spec.command is None:
            status, blockers = safety_check(repo_root)
            rows.append({"id": spec.check_id, "title": spec.title, "status": status, "required": spec.required, "command": "internal safety scan", "reason": "; ".join(blockers) if blockers else "safe defaults preserved"})
            continue
        blocker = blocked_by_environment(spec, env)
        if blocker:
            rows.append({"id": spec.check_id, "title": spec.title, "status": BLOCKED, "required": spec.required, "command": _cmd_text(spec.command), "reason": blocker})
            continue
        started = time.time()
        try:
            proc = _run(spec.command, repo_root, spec.timeout_sec)
            blob = (proc.stdout + proc.stderr).strip().replace("\n", " ")[:500]
            rows.append({"id": spec.check_id, "title": spec.title, "status": PASS if proc.returncode == 0 else FAIL, "required": spec.required, "command": _cmd_text(spec.command), "returncode": proc.returncode, "duration_sec": round(time.time() - started, 2), "reason": "completed" if proc.returncode == 0 else blob})
        except subprocess.TimeoutExpired:
            rows.append({"id": spec.check_id, "title": spec.title, "status": BLOCKED, "required": spec.required, "command": _cmd_text(spec.command), "reason": f"timed out after {spec.timeout_sec}s; launched process was terminated by subprocess timeout"})
    required_statuses = [row["status"] for row in rows if row["required"]]
    overall = FAIL if FAIL in required_statuses else (BLOCKED if BLOCKED in required_statuses else PASS)
    return {"schema": "workcell_studio_release_gate/v1", "profile": args.profile, "overall_status": overall, "environment": env, "checks": rows, "reports": {"json": str(args.json_output or out / f"release_gate_{args.profile}.json"), "markdown": str(args.markdown_output or out / f"release_gate_{args.profile}.md")}}


def write_markdown(path: Path, report: dict[str, Any]) -> None:
    lines = ["# Workcell Studio Release Gate", "", f"Overall status: **{report['overall_status']}**", f"Profile: `{report['profile']}`", f"Commit: `{report['environment']['commit_sha']}`", "", "## Environment", ""]
    for key in ["repo_root", "workspace_root", "platform", "python", "ros_distro", "ros2_found", "colcon_found", "workspace_setup_exists", "display", "qt_qpa_platform"]:
        lines.append(f"- {key}: `{report['environment'].get(key)}`")
    lines += ["", "## Checks", "", "| Check | Status | Required | Command | Reason |", "|---|---:|---:|---|---|"]
    for row in report["checks"]:
        lines.append(f"| {row['title']} | {row['status']} | {row['required']} | `{row.get('command','')}` | {row.get('reason','')} |")
    lines += ["", "## Manual runtime checklist (Ubuntu 22.04 / ROS 2 Humble)", "", "1. `cd /home/user/workcell_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select workcell_builder`", "2. `source install/setup.bash`", "3. `cd src/easy_manipulation_deployment && python3 scripts/run_workcell_studio_release_gate.py --profile runtime --workspace-root /home/user/workcell_ws`", "4. For canonical demo acceptance, run `python3 scripts/run_workcell_studio_release_gate.py --profile demo --workspace-root /home/user/workcell_ws --scene ur5_2f_test`.", "5. Confirm all launched fake-hardware processes exit after the bounded checks and no real-hardware flags were enabled."]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    report = run_gate(args)
    json_path = args.json_output or Path(report["reports"]["json"])
    md_path = args.markdown_output or Path(report["reports"]["markdown"])
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_markdown(md_path, report)
    for row in report["checks"]:
        print(f"{row['status']:14} {row['id']}: {row.get('reason','')}")
    print(f"overall: {report['overall_status']}")
    print(f"json: {json_path}")
    print(f"markdown: {md_path}")
    return 0 if report["overall_status"] == PASS else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        os.kill(os.getpid(), signal.SIGTERM)
