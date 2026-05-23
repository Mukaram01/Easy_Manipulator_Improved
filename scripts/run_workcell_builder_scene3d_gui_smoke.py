#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, os, shlex, shutil, subprocess, sys, time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
from scripts.workcell_studio_script_bootstrap import ensure_repo_root_on_sys_path
ensure_repo_root_on_sys_path(__file__)
from scripts.workcell_studio_path_resolver import resolve_repo_root, resolve_workspace_root, resolve_workcell_builder_executable

EXPECTED_SCHEMA = "workcell_studio_scene3d_gui_smoke/v1"

def _tail(text: str, lines: int = 40) -> str:
    parts = (text or "").splitlines()
    return "\n".join(parts[-lines:])

def _resolve_executable_candidates(workspace_root: Path) -> list[Path]:
    c = []
    path_hit = shutil.which("workcell_builder")
    if path_hit: c.append(Path(path_hit))
    c.extend([workspace_root / "install/workcell_builder/bin/workcell_builder", workspace_root / "install/workcell_builder/lib/workcell_builder/workcell_builder", workspace_root / "build/workcell_builder/workcell_builder"])
    return c

def build_cmd(exe: Path | str, args: argparse.Namespace) -> list[str]:
    cmd = [str(exe), "--scene3d-smoke"]
    if args.new_cell_recommended_layout_smoke:
        cmd.append("--new-cell-recommended-layout-smoke")
    elif args.scene:
        cmd += ["--scene", args.scene]
    cmd += ["--smoke-output", str(args.output)]
    if args.screenshot:
        cmd += ["--smoke-screenshot", str(args.screenshot)]
    cmd.append("--exit-after-smoke")
    return cmd

def with_xvfb(cmd: list[str], use_xvfb: bool) -> tuple[list[str], list[str]]:
    if not use_xvfb:
        return cmd, []
    xvfb_run = shutil.which("xvfb-run")
    if xvfb_run:
        return [xvfb_run, "-a"] + cmd, []
    return cmd, ["xvfb_requested_but_unavailable"]

def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", type=Path, default=_REPO_ROOT)
    ap.add_argument("--workspace-root", type=Path, default=_REPO_ROOT)
    ap.add_argument("--executable", type=Path, default=None)
    ap.add_argument("--scene", default=None)
    ap.add_argument("--new-cell-recommended-layout-smoke", action="store_true")
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--screenshot", type=Path, default=None)
    ap.add_argument("--timeout-sec", type=float, default=30.0)
    ap.add_argument("--xvfb", action="store_true")
    args = ap.parse_args()

    repo_root = resolve_repo_root(explicit_repo_root=args.repo_root)
    workspace_root = resolve_workspace_root(repo_root, args.workspace_root)
    exe = args.executable or resolve_workcell_builder_executable(workspace_root)
    cmd = build_cmd(exe, args)
    cmd, xwarn = with_xvfb(cmd, args.xvfb)

    stdout_log = args.output.with_suffix(args.output.suffix + ".stdout.log")
    stderr_log = args.output.with_suffix(args.output.suffix + ".stderr.log")
    diag = {
        "schema": EXPECTED_SCHEMA,
        "scene": args.scene,
        "repo_root": str(repo_root),
        "workspace_root": str(workspace_root),
        "executable": str(exe),
        "child_command": " ".join(shlex.quote(x) for x in cmd),
        "cwd": str(repo_root),
        "env": {k: os.environ.get(k, "") for k in ["DISPLAY", "WAYLAND_DISPLAY", "QT_QPA_PLATFORM", "XDG_SESSION_TYPE"]},
        "timeout_sec": args.timeout_sec,
        "stdout_log_path": str(stdout_log),
        "stderr_log_path": str(stderr_log),
        "screenshot_path": str(args.screenshot) if args.screenshot else None,
    }

    timed_out = False
    rc = None
    stdout = ""
    stderr = ""
    try:
        proc = subprocess.run(cmd, cwd=repo_root, text=True, capture_output=True, timeout=max(0.1, args.timeout_sec), check=False)
        rc = proc.returncode
        stdout, stderr = proc.stdout or "", proc.stderr or ""
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        rc = -1
        stdout, stderr = exc.stdout or "", exc.stderr or ""

    stdout_log.parent.mkdir(parents=True, exist_ok=True)
    stdout_log.write_text(stdout, encoding="utf-8")
    stderr_log.write_text(stderr, encoding="utf-8")

    stdout_tail = _tail(stdout)
    stderr_tail = _tail(stderr)
    diag.update({"child_returncode": rc, "timed_out": timed_out, "stdout_tail": stdout_tail, "stderr_tail": stderr_tail, "screenshot_available": bool(args.screenshot and args.screenshot.exists())})

    blockers = list(xwarn)
    warnings: list[str] = []
    if timed_out: blockers.append("child_process_timed_out")
    if rc not in (0, None): blockers.append("child_process_returned_nonzero")

    if args.output.exists():
        app_status="UNKNOWN"
        try:
            app_status=json.loads(args.output.read_text(encoding="utf-8")).get("status","UNKNOWN")
        except Exception:
            pass
        print(f"status=PASS smoke_status=APP_JSON_PRESENT wrapper_status=PASS app_status={app_status} returncode={rc} timed_out={timed_out}")
        print("child_command=" + diag["child_command"])
        print("stdout_log_path=" + str(stdout_log))
        print("stderr_log_path=" + str(stderr_log))
        return 0 if rc == 0 else 1

    blockers.append("app_smoke_json_missing")
    if "--scene3d-smoke" in diag["child_command"] and "--smoke-output" in diag["child_command"]:
        blockers.append("app_started_but_no_smoke_report")
    else:
        blockers.append("app_ignored_scene3d_smoke_args")

    fail_payload = {
        **diag,
        "status": "FAIL",
        "blockers": blockers,
        "warnings": warnings,
    }
    _write_json(args.output, fail_payload)
    print("status=FAIL smoke_status=WRAPPER_FAIL_JSON")
    print("child_command=" + diag["child_command"])
    print(f"returncode={rc} timed_out={timed_out}")
    print("stdout_log_path=" + str(stdout_log))
    print("stderr_log_path=" + str(stderr_log))
    print("blocker_list=" + " | ".join(blockers))
    return 1

if __name__ == "__main__":
    raise SystemExit(main())
