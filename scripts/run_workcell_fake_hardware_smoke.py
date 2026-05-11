#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

PASS = "WORKCELL_FAKE_HARDWARE_SMOKE: PASS"
WARN = "WORKCELL_FAKE_HARDWARE_SMOKE: WARN"
FAIL = "WORKCELL_FAKE_HARDWARE_SMOKE: FAIL"
SKIP = "WORKCELL_FAKE_HARDWARE_SMOKE: SKIP"

FORBIDDEN_LOG_MARKERS = ["use_fake_hardware:=false", "real_hardware_enabled: true", "move_group_action"]


def _run(cmd: list[str], cwd: Path | None = None, timeout: int = 120) -> tuple[int, str]:
    proc = subprocess.run(cmd, cwd=str(cwd) if cwd else None, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout, check=False)
    return proc.returncode, proc.stdout


def _has(text: str, needle: str) -> bool:
    return needle.lower() in text.lower()


def _status(errors: list[str], warnings: list[str], skipped: bool) -> str:
    if errors:
        return "FAIL"
    if skipped:
        return "SKIP"
    if warnings:
        return "WARN"
    return "PASS"


def main() -> int:
    p = argparse.ArgumentParser(description="Safe headless fake-hardware smoke acceptance gate")
    p.add_argument("--scene-dir")
    p.add_argument("--workspace", default="~/workcell_ws")
    p.add_argument("--generate-golden-demo", action="store_true")
    p.add_argument("--use-golden-demo", action="store_true")
    p.add_argument("--skip-colcon", action="store_true")
    p.add_argument("--skip-launch", action="store_true")
    p.add_argument("--run-colcon", action="store_true")
    p.add_argument("--run-launch", action="store_true")
    p.add_argument("--timeout-seconds", type=int, default=30)
    p.add_argument("--headless", action="store_true")
    p.add_argument("--strict", action="store_true")
    p.add_argument("--print-summary", action="store_true")
    args = p.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    workspace = Path(args.workspace).expanduser().resolve()
    warnings: list[str] = []
    errors: list[str] = []
    skipped = False

    if args.generate_golden_demo:
        out_dir = Path("/tmp/workcell_golden_demo")
        if out_dir.exists():
            shutil.rmtree(out_dir)
        rc, out = _run([sys.executable, str(repo_root / "scripts/generate_golden_workcell_demo.py"), "--output-dir", str(out_dir), "--scene-name", "ur5_2f_golden_demo", "--validate", "--overwrite", "--no-rviz"], timeout=120)
        if rc != 0:
            errors.append(f"golden demo generation failed: {out}")
        args.scene_dir = str(out_dir / "ur5_2f_golden_demo")

    if args.use_golden_demo and not args.scene_dir:
        args.scene_dir = "/tmp/workcell_golden_demo/ur5_2f_golden_demo"

    if not args.scene_dir:
        errors.append("--scene-dir required unless --generate-golden-demo is used")
        scene_dir = Path(".")
    else:
        scene_dir = Path(args.scene_dir).expanduser().resolve()

    env_file = scene_dir / "environment.yaml"
    task_recipe = scene_dir / "config/task_recipe.yaml"
    summary_json = scene_dir / "workcell_studio_summary.json"
    summary_md = scene_dir / "workcell_studio_summary.md"
    preview_svg = scene_dir / "preview/workcell_preview.svg"
    preview_html = scene_dir / "preview/workcell_preview.html"

    if not scene_dir.exists():
        errors.append(f"scene directory missing: {scene_dir}")
    if not env_file.exists():
        errors.append("environment.yaml missing")

    env_text = env_file.read_text(encoding="utf-8") if env_file.exists() else ""
    for required in [
        "schema_version: workcell_scene/v1",
        "real_hardware_enabled: false",
        "runtime_execution_enabled: false",
        "motion_command_sent: false",
        "moveit_plan_service_called: false",
    ]:
        if required not in env_text:
            errors.append(f"missing required safety/schema marker: {required}")

    if not task_recipe.exists():
        warnings.append("task_recipe.yaml missing")
    for f in [summary_json, summary_md, preview_svg, preview_html]:
        if not f.exists():
            warnings.append(f"artifact missing: {f.name}")

    rc, out = _run([sys.executable, str(repo_root / "scripts/validate_workcell_scene.py"), "--scene-dir", str(scene_dir)] + (["--strict"] if args.strict else []), timeout=60)
    if rc != 0:
        errors.append("scene schema validator failed")
    if "WORKCELL_SCENE_SCHEMA:" not in out:
        warnings.append("scene schema marker missing")

    rc, out = _run([sys.executable, str(repo_root / "scripts/validate_workcell_asset_catalog.py"), "--repo-root", str(repo_root)] + (["--strict"] if args.strict else []), timeout=60)
    if rc != 0:
        (errors if args.strict else warnings).append("asset catalog validator reported issues")

    launch_text = (summary_md.read_text(encoding="utf-8") if summary_md.exists() else "") + "\n" + (summary_json.read_text(encoding="utf-8") if summary_json.exists() else "")
    if "demo.launch.py" not in launch_text:
        warnings.append("fake-hardware launch command not found in summary artifacts")
    if "launch_rviz" not in launch_text:
        warnings.append("launch_rviz guidance not found")
    if "use_fake_hardware:=true" not in launch_text:
        warnings.append("use_fake_hardware:=true guidance not found")

    if args.run_colcon and not args.skip_colcon:
        if not workspace.exists():
            (errors if args.strict else warnings).append(f"workspace missing for colcon check: {workspace}")
        else:
            rc, out = _run(["bash", "-lc", "source /opt/ros/humble/setup.bash && colcon build --event-handlers console_direct+ --packages-up-to ur5_2f_golden_demo"], cwd=workspace, timeout=1800)
            if rc != 0:
                (errors if args.strict else warnings).append("colcon build unavailable/failed")

    if args.run_launch and not args.skip_launch:
        if not workspace.exists():
            (errors if args.strict else warnings).append(f"workspace missing for launch check: {workspace}")
        else:
            pkg = scene_dir.name
            launch_cmd = f"source /opt/ros/humble/setup.bash && [ -f install/setup.bash ] && source install/setup.bash && timeout {args.timeout_seconds} ros2 launch {pkg} demo.launch.py use_fake_hardware:=true launch_rviz:=false"
            rc, out = _run(["bash", "-lc", launch_cmd], cwd=workspace, timeout=max(args.timeout_seconds + 20, 60))
            low = out.lower()
            for marker in FORBIDDEN_LOG_MARKERS:
                if marker in low:
                    errors.append(f"forbidden runtime marker seen in launch logs: {marker}")
            if rc not in (0, 124):
                (errors if args.strict else warnings).append(f"launch failed with exit code {rc}")
    else:
        skipped = True

    status = _status(errors, warnings, skipped)
    marker = {"PASS": PASS, "WARN": WARN, "FAIL": FAIL, "SKIP": SKIP}[status]
    report = {
        "schema": "workcell_fake_hardware_smoke/v1",
        "scene_dir": str(scene_dir),
        "status": status,
        "headless": True if args.headless or not args.run_launch else args.headless,
        "run_launch": bool(args.run_launch and not args.skip_launch),
        "safety": {
            "real_hardware_enabled": False,
            "runtime_execution_enabled": False,
            "motion_command_sent": False,
            "moveit_plan_service_called": False,
        },
        "warnings": warnings,
        "errors": errors,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
    }
    (scene_dir / "fake_hardware_smoke_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    (scene_dir / "fake_hardware_smoke_report.md").write_text(f"# Fake Hardware Smoke Report\n\n- Status: **{status}**\n- Marker: `{marker}`\n- Scene: `{scene_dir}`\n", encoding="utf-8")

    print(marker)
    if args.print_summary:
        print(json.dumps(report, indent=2))
    return 1 if status == "FAIL" or (args.strict and status in {"WARN"}) else 0


if __name__ == "__main__":
    raise SystemExit(main())
