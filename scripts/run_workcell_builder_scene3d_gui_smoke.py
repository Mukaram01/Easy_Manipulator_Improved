#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_SCHEMA = "workcell_studio_scene3d_gui_smoke/v1"
PASS_STATES = {"ok", "pass", "passed"}


def resolve_workcell_builder() -> Path | str:
    install_bin = ROOT / "install/workcell_builder/lib/workcell_builder/workcell_builder"
    if install_bin.is_file():
        return install_bin
    fallback = shutil.which("workcell_builder")
    if fallback:
        return fallback
    raise FileNotFoundError(
        "unable to resolve workcell_builder executable: checked install/workcell_builder first, then PATH"
    )


def build_cmd(exe: Path | str, args: argparse.Namespace) -> list[str]:
    cmd = [str(exe)]
    if args.scene:
        cmd += ["--scene", args.scene]
    if args.new_cell_recommended_layout_smoke:
        cmd.append("--new-cell-recommended-layout-smoke")
    if args.output:
        cmd += ["--output", str(args.output)]
    if args.screenshot:
        cmd += ["--screenshot", str(args.screenshot)]
    cmd.append("--exit-after-smoke")
    return cmd


def with_xvfb(cmd: list[str], use_xvfb: bool) -> tuple[list[str], list[str]]:
    warns: list[str] = []
    if not use_xvfb:
        return cmd, warns
    xvfb_run = shutil.which("xvfb-run")
    if xvfb_run:
        return [xvfb_run, "-a"] + cmd, warns
    warns.append("WARN: --xvfb requested but xvfb-run is unavailable; continuing without xvfb")
    return cmd, warns


def validate_smoke_json(path: Path) -> tuple[list[str], list[str], str]:
    blockers: list[str] = []
    warnings: list[str] = []
    status = "UNKNOWN"

    if not path.is_file():
        return [f"missing smoke JSON output: {path}"], warnings, status

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return [f"invalid JSON: {path} ({exc})"], warnings, status

    if not isinstance(payload, dict):
        return [f"invalid JSON payload type: expected object at {path}"], warnings, status

    schema = payload.get("schema")
    if schema != EXPECTED_SCHEMA:
        blockers.append(f"schema mismatch: expected {EXPECTED_SCHEMA!r}, got {schema!r}")

    missing_or_bad: list[str] = []
    for field in ["viewport_received_count", "render_cache_count", "rendered_count", "hierarchy_rows_count", "selectable_count"]:
        value = payload.get(field)
        if not isinstance(value, int) or value < 0:
            missing_or_bad.append(field)
    raw_status = payload.get("status")
    if not isinstance(raw_status, str):
        missing_or_bad.append("status")
    else:
        status = raw_status
    if missing_or_bad:
        blockers.append("missing/invalid required fields: " + ", ".join(missing_or_bad))

    if isinstance(raw_status, str) and raw_status.lower() not in PASS_STATES:
        blockers.append(f"status is FAIL-like: {raw_status!r}")

    warnings.extend(payload.get("warnings", []) if isinstance(payload.get("warnings"), list) else [])
    return blockers, warnings, status


def main() -> int:
    ap = argparse.ArgumentParser(description="Run Workcell Builder Scene3D GUI smoke and validate output")
    ap.add_argument("--scene", default=None)
    ap.add_argument("--new-cell-recommended-layout-smoke", action="store_true")
    ap.add_argument("--output", type=Path, default=ROOT / "build/workcell_studio/scene3d_gui_smoke.json")
    ap.add_argument("--screenshot", type=Path, default=None)
    ap.add_argument("--timeout-sec", type=float, default=30.0)
    ap.add_argument("--xvfb", action="store_true")
    args = ap.parse_args()

    blockers: list[str] = []
    warnings: list[str] = []
    artifacts = {"json": str(args.output), "screenshot": str(args.screenshot) if args.screenshot else None}

    try:
        exe = resolve_workcell_builder()
    except FileNotFoundError as exc:
        print(f"status=FAIL; blockers=[{exc}]")
        return 2

    cmd = build_cmd(exe, args)
    cmd, xvfb_warnings = with_xvfb(cmd, args.xvfb)
    warnings.extend(xvfb_warnings)

    start = time.time()
    try:
        proc = subprocess.run(cmd, text=True, capture_output=True, timeout=max(0.1, args.timeout_sec), check=False)
    except subprocess.TimeoutExpired:
        blockers.append(f"process timed out after {args.timeout_sec}s")
        proc = None
    elapsed = time.time() - start

    if proc is not None and proc.returncode != 0:
        blockers.append(f"process exited non-zero: rc={proc.returncode}")

    json_blockers, json_warnings, status = validate_smoke_json(args.output)
    blockers.extend(json_blockers)
    warnings.extend(json_warnings)

    if proc is not None and proc.stderr.strip():
        warnings.append("stderr present (truncated): " + proc.stderr.strip()[:200])

    final_status = "PASS" if not blockers else "FAIL"
    print(f"status={final_status} smoke_status={status} elapsed_sec={elapsed:.2f}")
    print(f"blockers={len(blockers)} warnings={len(warnings)}")
    if blockers:
        print("blocker_list=" + " | ".join(blockers))
    if warnings:
        print("warning_list=" + " | ".join(str(w) for w in warnings))
    print("artifacts=" + json.dumps(artifacts, sort_keys=True))

    return 0 if not blockers else 1


if __name__ == "__main__":
    raise SystemExit(main())
