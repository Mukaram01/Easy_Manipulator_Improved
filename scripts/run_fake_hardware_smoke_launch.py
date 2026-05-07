#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, os, signal, subprocess, sys, time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

FORBIDDEN = [
    "use_fake_hardware:=false", "real_hardware", "runtime execution", "execute runtime",
    "epd", "realsense", "gazebo", "isaac", "ros2 action send_goal", "ros2 topic pub", "move_group"
]


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _contains_forbidden(cmd: str) -> list[str]:
    low = cmd.lower()
    return [t for t in FORBIDDEN if t in low]


def _detect(stdout: str, stderr: str, token: str) -> str | bool:
    blob = f"{stdout}\n{stderr}".lower()
    if not blob.strip():
        return "unknown"
    return token in blob


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", type=Path, required=True)
    ap.add_argument("--output-dir", type=Path, required=True)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--timeout-s", type=int, default=20)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    dry_run = True if (not a.execute and not a.dry_run) else a.dry_run
    out = a.output_dir; out.mkdir(parents=True, exist_ok=True)
    stdout_path = out / "captured_stdout.log"
    stderr_path = out / "captured_stderr.log"

    session = json.loads(a.session.read_text(encoding="utf-8"))
    cmd = (((session.get("rviz_moveit") or {}).get("suggested_launch") or {}).get("command") or "")
    errs: list[str] = []
    warns: list[str] = []
    unsafe_tokens = _contains_forbidden(cmd)
    if session.get("schema") != "rviz_moveit_plan_preview_session/v1": errs.append("session schema mismatch")
    if ((session.get("session") or {}).get("launch_allowed")) is True: errs.append("session.launch_allowed must remain false for preparation artifacts")
    s = session.get("safety") or {}
    for k in ["motion_started", "moveit_service_called", "ros_launch_started", "runtime_io_applied"]:
        if s.get(k) not in [False, None]: errs.append(f"unsafe session safety flag: {k}")
    if s.get("fake_hardware_required") is False: errs.append("fake_hardware_required must be true")
    if "use_fake_hardware:=true" not in cmd: errs.append("suggested command must include use_fake_hardware:=true")
    if unsafe_tokens: errs.append(f"forbidden command token(s): {unsafe_tokens}")

    actually_launched = False
    launch_started = False
    ret: int | None = None
    launch_exit_ok = False
    start = _now(); end = start
    captured_out = ""; captured_err = ""

    if not dry_run and not errs:
        try:
            p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid)
            launch_started = True
            actually_launched = True
            try:
                captured_out, captured_err = p.communicate(timeout=max(1, a.timeout_s))
                ret = p.returncode
                launch_exit_ok = True
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                try:
                    captured_out, captured_err = p.communicate(timeout=5)
                except Exception:
                    pass
                warns.append(f"launch timeout reached ({a.timeout_s}s); process group terminated")
                ret = p.returncode if p.returncode is not None else -15
                launch_exit_ok = True
        except FileNotFoundError:
            errs.append("launch command failed: command not found / ROS environment likely not sourced")
        except Exception as exc:
            errs.append(f"launch failed: {exc}")
    elif not dry_run and errs:
        warns.append("execute requested but blocked by safety checks")
    end = _now()

    if actually_launched:
        stdout_path.write_text(captured_out, encoding="utf-8")
        stderr_path.write_text(captured_err, encoding="utf-8")

    status = "FAIL" if errs else ("WARN" if warns else ("SKIPPED" if dry_run else "PASS"))
    report: dict[str, Any] = {
        "schema": "fake_hardware_smoke_launch_report/v1",
        "source": {"plan_preview_session": str(a.session), "suggested_command": cmd, "scene_package": (session.get("source") or {}).get("scene_package", "")},
        "run": {"mode": "fake_hardware_smoke_check", "actually_launched": actually_launched, "dry_run_only": dry_run, "timeout_s": a.timeout_s, "started_at": start, "ended_at": end, "return_code": ret},
        "checks": {
            "command_safety_checked": True,
            "command_contains_fake_hardware_true": "use_fake_hardware:=true" in cmd,
            "forbidden_real_hardware_tokens_absent": not unsafe_tokens,
            "launch_started": launch_started,
            "launch_exited_cleanly_or_timeout_terminated": launch_exit_ok,
            "move_group_seen_in_output": _detect(captured_out, captured_err, "move_group"),
            "rviz_seen_in_output": _detect(captured_out, captured_err, "rviz"),
            "robot_state_publisher_seen_in_output": _detect(captured_out, captured_err, "robot_state_publisher"),
        },
        "safety": {"motion_command_sent": False, "moveit_plan_service_called": False, "runtime_execution_called": False, "real_hardware_enabled": False, "runtime_io_applied": False},
        "artifacts": {"captured_stdout_log": str(stdout_path), "captured_stderr_log": str(stderr_path)},
        "result": {"status": status, "warnings": warns, "errors": errs, "suggested_next_actions": ["Use --dry-run first", "Only use --execute in fake-hardware workspaces", "This smoke check does not validate planning or safety"]},
    }
    jpath = out / "fake_hardware_smoke_launch_report.json"
    mpath = out / "fake_hardware_smoke_launch_report.md"
    jpath.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    mpath.write_text(f"# Fake Hardware Smoke Launch Report\n\n- Status: **{status}**\n- Session: `{a.session}`\n- Dry-run: `{dry_run}`\n- Actually launched: `{actually_launched}`\n- Timeout(s): `{a.timeout_s}`\n- Errors: `{errs or ['(none)']}`\n- Warnings: `{warns or ['(none)']}`\n", encoding="utf-8")
    payload = {"status": status, "report_json": str(jpath), "report_markdown": str(mpath)}
    print(json.dumps(payload, indent=2) if a.json else f"{status}: {jpath}")
    return 1 if status == "FAIL" else 0

if __name__ == "__main__":
    raise SystemExit(main())
