#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

from workcell_studio_error_messages import get_message

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
TMP_ROOT = Path("/tmp/workcell_studio_scratch_smoke")


BUILD_CMD_TEMPLATE = "colcon build --symlink-install --packages-select <scene_name>"
PKG_DISCOVERY_TEMPLATE = "ros2 pkg prefix <scene_name>"
LAUNCH_SHOW_ARGS_TEMPLATE = "ros2 launch <scene_name> demo.launch.py --show-args"
LAUNCH_SMOKE_TEMPLATE = "ros2 launch <scene_name> demo.launch.py use_fake_hardware:=true launch_rviz:=false"

REPORT_FIELDS = [
    "scene_name",
    "workspace",
    "scene_dir",
    "workspace_scene_dir",
    "placement_method",
    "build_command",
    "build_returncode",
    "package_discovery_command",
    "package_discovery_returncode",
    "launch_show_args_command",
    "launch_show_args_returncode",
    "launch_smoke_command",
    "launch_smoke_returncode",
    "ready_for_rviz_moveit",
    "blockers",
    "warnings",
    "logs_tail",
]


def _run(cmd: list[str], timeout: int | None = None) -> tuple[int, str]:
    p = subprocess.run(cmd, capture_output=True, text=True, check=False, timeout=timeout)
    return p.returncode, (p.stdout + "\n" + p.stderr).strip()


def _tail(text: str, lines: int = 80) -> str:
    parts = [ln for ln in text.splitlines() if ln.strip()]
    return "\n".join(parts[-lines:])


def _append_blocker(report: dict, code: str, detail: str = "") -> None:
    msg = get_message(code)
    if detail:
        msg["detail"] = detail
    report["blockers"].append(msg)


def _find_workspace_scenes_root(workspace: Path) -> Path | None:
    preferred = workspace / "src" / "easy_manipulation_deployment" / "scenes"
    fallback = workspace / "src" / "scenes"
    if preferred.parent.exists():
        preferred.mkdir(parents=True, exist_ok=True)
        return preferred
    if fallback.parent.exists():
        fallback.mkdir(parents=True, exist_ok=True)
        return fallback
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workspace", type=Path, required=True)
    ap.add_argument("--scene-name", default="scratch_ur5_2f_smoke")
    ap.add_argument("--timeout-sec", type=int, default=30)
    ap.add_argument("--json-out", type=Path, default=TMP_ROOT / "smoke_report.json")
    args = ap.parse_args()

    report = {k: "" for k in REPORT_FIELDS}
    report.update({
        "scene_name": args.scene_name,
        "workspace": str(args.workspace),
        "ready_for_rviz_moveit": False,
        "blockers": [],
        "warnings": [],
        "logs_tail": {},
        "build_returncode": -1,
        "package_discovery_returncode": -1,
        "launch_show_args_returncode": -1,
        "launch_smoke_returncode": -1,
    })

    if not args.workspace.exists():
        _append_blocker(report, "MISSING_WORKSPACE", f"Workspace does not exist: {args.workspace}")
    if not (args.workspace / "src").is_dir():
        _append_blocker(report, "MISSING_WORKSPACE_SRC", f"Workspace src/ missing: {args.workspace / 'src'}")
    if shutil.which("ros2") is None:
        _append_blocker(report, "MISSING_ROS2")
    if shutil.which("colcon") is None:
        _append_blocker(report, "MISSING_COLCON")

    scenes_root = _find_workspace_scenes_root(args.workspace) if (args.workspace / "src").exists() else None
    if scenes_root is None:
        _append_blocker(report, "MISSING_WORKSPACE_SRC", "Could not resolve scenes root under workspace/src")

    TMP_ROOT.mkdir(parents=True, exist_ok=True)
    scene_acceptance_json = TMP_ROOT / "scratch_acceptance_report.json"
    gen_cmd = [
        sys.executable,
        str(SCRIPTS_DIR / "generate_scratch_cell_acceptance.py"),
        "--scene-name",
        args.scene_name,
        "--output-root",
        str(TMP_ROOT),
        "--json-out",
        str(scene_acceptance_json),
    ]
    gen_rc, gen_out = _run(gen_cmd)
    report["logs_tail"]["generate_scratch_cell_acceptance"] = _tail(gen_out)
    if gen_rc != 0:
        _append_blocker(report, "VALIDATION_BLOCKED", "Scratch cell generation failed before build/run smoke checks.")

    scene_dir = ""
    if scene_acceptance_json.exists():
        payload = json.loads(scene_acceptance_json.read_text(encoding="utf-8"))
        scene_dir = payload.get("scene_dir", "")
    report["scene_dir"] = scene_dir

    if scene_dir and scenes_root is not None:
        src = Path(scene_dir)
        dst = scenes_root / args.scene_name
        report["workspace_scene_dir"] = str(dst)
        if dst.exists() or dst.is_symlink():
            if dst.is_symlink() or dst.is_file():
                dst.unlink()
            else:
                shutil.rmtree(dst)
        try:
            os.symlink(src, dst, target_is_directory=True)
            report["placement_method"] = "symlink"
        except OSError:
            shutil.copytree(src, dst)
            report["placement_method"] = "copy"

    if report["blockers"]:
        out = json.dumps(report, indent=2)
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(out + "\n", encoding="utf-8")
        print(out)
        return 1

    build_cmd = ["colcon", "build", "--symlink-install", "--packages-select", args.scene_name]
    report["build_command"] = " ".join(build_cmd)
    build_rc, build_out = _run(build_cmd, timeout=max(args.timeout_sec * 4, 60))
    report["build_returncode"] = build_rc
    report["logs_tail"]["build"] = _tail(build_out)
    if build_rc != 0:
        _append_blocker(report, "COLCON_BUILD_FAILED", _tail(build_out))

    pkg_cmd = ["ros2", "pkg", "prefix", args.scene_name]
    report["package_discovery_command"] = " ".join(pkg_cmd)
    pkg_rc, pkg_out = _run(pkg_cmd)
    report["package_discovery_returncode"] = pkg_rc
    report["logs_tail"]["package_discovery"] = _tail(pkg_out)
    if pkg_rc != 0:
        _append_blocker(report, "ROS_PACKAGE_NOT_DISCOVERABLE", _tail(pkg_out))

    show_args_cmd = ["ros2", "launch", args.scene_name, "demo.launch.py", "--show-args"]
    report["launch_show_args_command"] = " ".join(show_args_cmd)
    show_rc, show_out = _run(show_args_cmd, timeout=args.timeout_sec)
    report["launch_show_args_returncode"] = show_rc
    report["logs_tail"]["launch_show_args"] = _tail(show_out)
    if show_rc != 0:
        _append_blocker(report, "LAUNCH_SHOW_ARGS_FAILED", _tail(show_out))

    smoke_cmd = [
        "ros2", "launch", args.scene_name, "demo.launch.py",
        "use_fake_hardware:=true", "launch_rviz:=false",
    ]
    report["launch_smoke_command"] = " ".join(smoke_cmd)
    smoke_rc, smoke_out = _run(smoke_cmd, timeout=args.timeout_sec)
    report["launch_smoke_returncode"] = smoke_rc
    report["logs_tail"]["launch_smoke"] = _tail(smoke_out)
    if smoke_rc != 0:
        _append_blocker(report, "LAUNCH_SMOKE_FAILED", _tail(smoke_out))

    report["ready_for_rviz_moveit"] = not report["blockers"]
    out = json.dumps(report, indent=2)
    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    args.json_out.write_text(out + "\n", encoding="utf-8")
    print(out)
    return 0 if report["ready_for_rviz_moveit"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
