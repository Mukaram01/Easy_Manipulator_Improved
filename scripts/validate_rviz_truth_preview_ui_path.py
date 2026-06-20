#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shlex
import subprocess
from pathlib import Path
from typing import Any

REQUIRED_TOKENS = ("use_fake_hardware:=true", "launch_rviz:=true")
FORBIDDEN_TOKENS = (
    "use_fake_hardware:=false",
    "fake_hardware:=false",
    "real_hardware:=true",
    "use_real_hardware:=true",
    "hardware_mode:=real",
    "driver_mode:=real",
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Validate RViz truth preview UI launch path")
    p.add_argument("scene_pkg", nargs="?", help="Scene package name, e.g. ur5_2f_test")
    p.add_argument("--scene", dest="scene_opt", help="Scene package name, e.g. ur5_2f_test")
    p.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    p.add_argument("--workspace-root", type=Path, default=None)
    p.add_argument("--check-ros2-prefix", action="store_true")
    p.add_argument("--json", action="store_true")
    return p.parse_args()


def _build_launch_command(scene_pkg: str) -> str:
    return f"ros2 launch {scene_pkg} demo.launch.py use_fake_hardware:=true launch_rviz:=true"


def _runner_commands(workspace_root: Path, launch_command: str) -> dict[str, str]:
    ws = str(workspace_root)
    return {
        "dry_run": f"cd {shlex.quote(ws)} && source install/setup.bash && {launch_command}",
        "run": f"cd {shlex.quote(ws)} && source install/setup.bash && {launch_command}",
    }


def main() -> int:
    args = parse_args()
    scene_pkg = (args.scene_opt or args.scene_pkg or "").strip()
    if not scene_pkg:
        raise SystemExit("scene package is required: pass positional <scene_pkg> or --scene <scene_pkg>")
    repo_root = args.repo_root.resolve()
    workspace_root = args.workspace_root.resolve() if args.workspace_root else repo_root
    scene_dir = workspace_root / "scenes" / scene_pkg

    launch_command = _build_launch_command(scene_pkg)
    runner = _runner_commands(workspace_root, launch_command)

    blockers: list[str] = []
    warnings: list[str] = []

    package_xml = scene_dir / "package.xml"
    launch_file = scene_dir / "launch" / "demo.launch.py"
    setup_bash = workspace_root / "install" / "setup.bash"

    if not package_xml.is_file():
        blockers.append("scene package.xml missing")
    if not launch_file.is_file():
        blockers.append("launch/demo.launch.py missing")
    if not setup_bash.is_file():
        blockers.append("install/setup.bash missing under workspace root")

    if launch_command != _build_launch_command(scene_pkg):
        blockers.append("launch command build is non-deterministic")

    lower = launch_command.lower()
    for token in REQUIRED_TOKENS:
        if token not in lower:
            blockers.append(f"required token missing: {token}")
    forbidden = [t for t in FORBIDDEN_TOKENS if t in lower]
    if forbidden:
        blockers.append("unsafe false-hardware token(s) rejected: " + ", ".join(forbidden))

    ros2_prefix: dict[str, Any] | None = None
    if args.check_ros2_prefix:
        proc = subprocess.run(["bash", "-lc", f"source {shlex.quote(str(setup_bash))} && ros2 pkg prefix {shlex.quote(scene_pkg)}"], cwd=repo_root, capture_output=True, text=True, check=False)
        ros2_prefix = {
            "checked": True,
            "returncode": proc.returncode,
            "prefix": proc.stdout.strip(),
            "stderr": proc.stderr.strip(),
            "ok": proc.returncode == 0 and bool(proc.stdout.strip()),
        }
        if not ros2_prefix["ok"]:
            warnings.append("ros2 pkg prefix check failed (workspace may be unsourced or package unbuilt)")

    payload: dict[str, Any] = {
        "schema": "rviz_truth_preview_ui_path_validation/v1",
        "scene_pkg": scene_pkg,
        "workspace_root": str(workspace_root),
        "required_artifacts": {
            "package_xml": str(package_xml),
            "launch_demo": str(launch_file),
            "workspace_setup": str(setup_bash),
        },
        "exact_launch_command": launch_command,
        "runner": runner,
        "status": "PASS" if not blockers else "FAIL",
        "blockers": blockers,
        "warnings": warnings,
    }
    if ros2_prefix is not None:
        payload["ros2_pkg_prefix"] = ros2_prefix

    text = json.dumps(payload, separators=(",", ":")) if args.json else json.dumps(payload, indent=2)
    print(text)
    return 0 if not blockers else 1


if __name__ == "__main__":
    raise SystemExit(main())
    if not repo_root.exists():
        blockers.append("repo root does not exist")
    if not workspace_root.exists():
        blockers.append("workspace root does not exist")
