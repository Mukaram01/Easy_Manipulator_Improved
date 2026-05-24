#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, os, shlex, shutil, subprocess, sys, time
from pathlib import Path
from typing import Any

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
from scripts.workcell_studio_script_bootstrap import ensure_repo_root_on_sys_path
ensure_repo_root_on_sys_path(__file__)
from scripts.workcell_studio_path_resolver import resolve_repo_root, resolve_workspace_root, resolve_workcell_builder_executable
from scripts.scene_root_resolver import resolve_scene_root
from scripts.scene3d_scene_discovery import discover_scene3d_scenes

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

def _discover_scene_targets(repo_root: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    scenes_root = resolve_scene_root(repo_root)
    for rec in discover_scene3d_scenes(scenes_root):
        rows.append({
            "scene": rec["scene"],
            "scene_path": rec["scene_path"],
            "scene_status": rec["status"],
            "blockers": list(rec.get("blockers") or []),
            "ignore_reason": rec.get("ignore_reason"),
        })
    return sorted(rows, key=lambda x: x["scene"])

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", type=Path, default=_REPO_ROOT)
    ap.add_argument("--workspace-root", type=Path, default=_REPO_ROOT)
    ap.add_argument("--executable", type=Path, default=None)
    ap.add_argument("--scene", default=None)
    ap.add_argument("--all-scenes", action="store_true")
    ap.add_argument("--output-dir", type=Path, default=None)
    ap.add_argument("--new-cell-recommended-layout-smoke", action="store_true")
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--screenshot", type=Path, default=None)
    ap.add_argument("--timeout-sec", type=float, default=30.0)
    ap.add_argument("--xvfb", action="store_true")
    args = ap.parse_args()

    if args.all_scenes and args.scene:
        raise SystemExit("Choose only one of --scene or --all-scenes")
    if not args.all_scenes and not args.scene and not args.new_cell_recommended_layout_smoke:
        raise SystemExit("Provide one of --scene, --all-scenes, or --new-cell-recommended-layout-smoke")
    if args.all_scenes and args.output_dir is None:
        raise SystemExit("--output-dir is required with --all-scenes")

    repo_root = resolve_repo_root(explicit_repo_root=args.repo_root)
    workspace_root = resolve_workspace_root(repo_root, args.workspace_root)
    exe = args.executable or resolve_workcell_builder_executable(workspace_root)

    if args.all_scenes:
        out_dir = args.output_dir.resolve()
        scenes = _discover_scene_targets(repo_root)
        per_scene: list[dict[str, Any]] = []
        totals = {"PASS": 0, "FAIL": 0, "BLOCKED": 0, "LEGACY_INCOMPLETE": 0}
        ignored_non_scenes: list[dict[str, Any]] = []
        legacy_incomplete: list[dict[str, Any]] = []
        for item in scenes:
            if item["scene_status"] == "IGNORED_NON_SCENE":
                ignored_non_scenes.append(item)
                continue
            if item["scene_status"] == "LEGACY_INCOMPLETE":
                legacy_incomplete.append(item)
                totals["LEGACY_INCOMPLETE"] += 1
                continue
            scene_name = item["scene"]
            scene_json = out_dir / f"scene3d_gui_smoke_{scene_name}.json"
            scene_png = out_dir / f"scene3d_gui_smoke_{scene_name}.png"
            cmd = [
                sys.executable, str(Path(__file__).resolve()), "--repo-root", str(repo_root), "--workspace-root", str(workspace_root),
                "--scene", scene_name, "--output", str(scene_json), "--screenshot", str(scene_png),
            ]
            if exe:
                cmd += ["--executable", str(exe)]
            if args.xvfb:
                cmd.append("--xvfb")
            proc = subprocess.run(cmd, cwd=repo_root, capture_output=True, text=True, check=False)
            payload: dict[str, Any] = {}
            if scene_json.exists():
                try:
                    payload = json.loads(scene_json.read_text(encoding="utf-8"))
                except Exception:
                    payload = {}
            smoke_status = str(payload.get("status", "FAIL")).upper()
            result_status = "PASS" if (proc.returncode == 0 and smoke_status in {"PASS", "OK"}) else "FAIL"
            blockers = list(payload.get("blockers", [])) if isinstance(payload.get("blockers"), list) else []
            blockers.extend(item.get("blockers", []))
            if item["scene_status"] == "BLOCKED":
                result_status = "BLOCKED"
            totals[result_status] = totals.get(result_status, 0) + 1
            per_scene.append({
                "scene": scene_name,
                "status": result_status,
                "returncode": proc.returncode,
                "smoke_json": str(scene_json),
                "smoke_png": str(scene_png),
                "scene_metadata": item,
                "blockers": blockers,
            })
            if payload:
                payload["scene_level_status"] = result_status
                payload["scene_level_blockers"] = blockers
                _write_json(scene_json, payload)
        summary = {
            "schema": EXPECTED_SCHEMA, "mode": "all_scenes", "output_dir": str(out_dir), "totals": totals, "results": per_scene,
            "supported_scene_count": len(per_scene),
            "legacy_incomplete_count": len(legacy_incomplete),
            "ignored_non_scene_count": len(ignored_non_scenes),
            "legacy_incomplete_scenes": legacy_incomplete,
            "ignored_non_scene_folders": ignored_non_scenes,
        }
        _write_json(out_dir / "scene3d_gui_smoke_summary.json", summary)
        md = ["# Scene3D GUI Smoke Summary", "",
              f"- supported_scene_count: {len(per_scene)}",
              f"- PASS: {totals['PASS']}", f"- FAIL: {totals['FAIL']}", f"- BLOCKED: {totals['BLOCKED']}",
              f"- legacy_incomplete_count: {len(legacy_incomplete)}", f"- ignored_non_scene_count: {len(ignored_non_scenes)}",
              "", "| Scene | Status | Return code | JSON | PNG |", "|---|---|---:|---|---|"]
        for r in per_scene:
            md.append(f"| {r['scene']} | {r['status']} | {r['returncode']} | `{r['smoke_json']}` | `{r['smoke_png']}` |")
        if legacy_incomplete:
            md += ["", "## Legacy incomplete scenes"]
            md += [f"- {r['scene']}: {', '.join(r.get('blockers') or ['legacy incomplete'])}" for r in legacy_incomplete]
        if ignored_non_scenes:
            md += ["", "## Ignored non-scene folders"]
            md += [f"- {r['scene']}: {r.get('ignore_reason', 'ignored')}" for r in ignored_non_scenes]
        (out_dir / "scene3d_gui_smoke_summary.md").write_text("\n".join(md) + "\n", encoding="utf-8")
        return 1 if totals["FAIL"] or totals["BLOCKED"] else 0

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
