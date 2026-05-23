#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from pathlib import Path
from typing import Any

REQUIRED_TOKEN = "use_fake_hardware:=true"
FORBIDDEN_TOKENS = [
    "use_fake_hardware:=false",
    "fake_hardware:=false",
    "real_hardware:=true",
    "use_real_hardware:=true",
    "hardware_mode:=real",
    "driver_mode:=real",
    "use_mock_hardware:=false",
    "mock_hardware:=false",
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Validate scene RViz truth preview launch command safety")
    p.add_argument("scene_name", help="Scene/package name")
    p.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    p.add_argument("--workspace-root", type=Path, default=None)
    p.add_argument("--check-ros2-prefix", action="store_true", help="Run 'ros2 pkg prefix <scene_pkg>' for visibility diagnostics")
    p.add_argument("--json", action="store_true", help="Emit compact JSON only")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    workspace_root = args.workspace_root.resolve() if args.workspace_root else repo_root
    scene_pkg = args.scene_name
    scene_dir = workspace_root / "scenes" / scene_pkg

    launch_command = f"ros2 launch {scene_pkg} demo.launch.py use_fake_hardware:=true launch_rviz:=true"

    blockers: list[str] = []
    warnings: list[str] = []

    package_xml = scene_dir / "package.xml"
    launch_file = scene_dir / "launch" / "demo.launch.py"
    if not package_xml.is_file():
        blockers.append(f"missing required artifact: {package_xml}")
    if not launch_file.is_file():
        blockers.append(f"missing required artifact: {launch_file}")

    lowered_command = launch_command.lower()
    if REQUIRED_TOKEN not in lowered_command:
        blockers.append(f"required token missing: {REQUIRED_TOKEN}")

    present_forbidden = [token for token in FORBIDDEN_TOKENS if token in lowered_command]
    if present_forbidden:
        blockers.append("forbidden token(s) present: " + ", ".join(present_forbidden))

    ros2_visible: dict[str, Any] | None = None
    if args.check_ros2_prefix:
        if shutil.which("ros2") is None:
            ros2_visible = {"checked": True, "visible": False, "reason": "ros2 executable not found"}
            warnings.append("ros2 executable not found; package visibility could not be checked")
        else:
            proc = subprocess.run(["ros2", "pkg", "prefix", scene_pkg], cwd=repo_root, capture_output=True, text=True)
            prefix = proc.stdout.strip()
            ros2_visible = {
                "checked": True,
                "visible": proc.returncode == 0 and bool(prefix),
                "returncode": proc.returncode,
                "prefix": prefix,
                "stderr": proc.stderr.strip(),
            }
            if proc.returncode != 0:
                warnings.append("scene package not visible to ros2 package index")

    payload: dict[str, Any] = {
        "schema": "rviz_truth_preview_command_validation/v1",
        "scene_pkg": scene_pkg,
        "repo_root": str(repo_root),
        "workspace_root": str(workspace_root),
        "scene_dir": str(scene_dir),
        "required_artifacts": {
            "package_xml": str(package_xml),
            "launch_demo": str(launch_file),
            "exists": package_xml.is_file() and launch_file.is_file(),
        },
        "launch_command": launch_command,
        "required_tokens": [REQUIRED_TOKEN],
        "forbidden_tokens": FORBIDDEN_TOKENS,
        "status": "PASS" if not blockers else "FAIL",
        "blockers": blockers,
        "warnings": warnings,
    }
    if ros2_visible is not None:
        payload["ros2_package_visibility"] = ros2_visible

    if args.json:
        print(json.dumps(payload, separators=(",", ":")))
    else:
        print(json.dumps(payload, indent=2))

    return 0 if not blockers else 1


if __name__ == "__main__":
    raise SystemExit(main())
