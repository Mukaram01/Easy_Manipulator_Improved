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
from scripts.workcell_studio_path_resolver import describe_resolution, resolve_repo_root, resolve_workspace_root, resolve_workcell_builder_executable, workcell_builder_executable_candidates
from scripts.scene_root_resolver import resolve_scene_root
from scripts.scene3d_scene_discovery import discover_scene3d_scenes

EXPECTED_SCHEMA = "workcell_studio_scene3d_gui_smoke/v1"

def _tail(text: str, lines: int = 40) -> str:
    parts = (text or "").splitlines()
    return "\n".join(parts[-lines:])

def _resolve_executable_candidates(workspace_root: Path | None) -> list[Path]:
    return workcell_builder_executable_candidates(workspace_root)

def build_cmd(exe: Path | str, args: argparse.Namespace) -> list[str]:
    cmd = [str(exe), "--scene3d-smoke"]
    if args.new_cell_recommended_layout_smoke:
        cmd.append("--new-cell-recommended-layout-smoke")
    elif args.scene:
        cmd += ["--scene", args.scene]
    if args.scene_path:
        cmd += ["--scene-path", str(args.scene_path)]
    cmd += ["--smoke-output", str(args.output)]
    if args.screenshot:
        cmd += ["--smoke-screenshot", str(args.screenshot)]
    cmd.append("--exit-after-smoke")
    return cmd

def with_xvfb(cmd: list[str], use_xvfb: bool) -> tuple[list[str], list[str], dict[str, str]]:
    if not use_xvfb:
        return cmd, [], {}
    xvfb_run = shutil.which("xvfb-run")
    if xvfb_run:
        return [xvfb_run, "-a"] + cmd, [], {}
    return cmd, ["xvfb_requested_but_unavailable_using_qt_offscreen"], {
        "QT_QPA_PLATFORM": "offscreen",
        "QT_OPENGL": "software",
        "LIBGL_ALWAYS_SOFTWARE": "1",
    }

def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")



def _resolve_single_scene_dir(repo_root: Path, scene: str | None, scene_path: Path | None) -> Path | None:
    if scene_path:
        p = scene_path if scene_path.is_absolute() else repo_root / scene_path
        return p.resolve()
    if scene:
        return (repo_root / 'scenes' / scene).resolve()
    return None


def _dims_available(item: dict[str, Any]) -> bool:
    g = str(item.get('geometry_type') or '').strip().lower()
    if g == 'box':
        size = item.get('size')
        return isinstance(size, list) and len(size) >= 3 and all(float(x or 0) > 0 for x in size[:3])
    if g == 'cylinder':
        return float(item.get('radius') or 0) > 0 and float(item.get('length') or 0) > 0
    if g == 'sphere':
        return float(item.get('radius') or 0) > 0
    if g == 'capsule':
        return float(item.get('radius') or 0) > 0 and float(item.get('length') or 0) > 0
    return False


def _static_scene3d_visual_evidence(scene_dir: Path | None) -> dict[str, Any]:
    evidence: dict[str, Any] = {
        'runtime_available': False,
        'physical_mesh_items_rendered': 0,
        'primitive_fallback_items_rendered': 0,
        'zones_overlays_rendered': 0,
        'skipped_helper_static_fallback_items': 0,
        'unresolved_transform_items': 0,
        'missing_mesh_items': 0,
        'physical_mesh_items_renderable': 0,
        'primitive_fallback_items_renderable': 0,
        'zones_overlays_renderable': 0,
        'source': 'generated/scene_visual_mesh_index.json',
        'notes': ['runtime executable unavailable; counts are static/headless renderability evidence, not GUI-render PASS evidence'],
    }
    if scene_dir is None:
        evidence['notes'].append('scene directory could not be resolved')
        return evidence
    index_path = scene_dir / 'generated' / 'scene_visual_mesh_index.json'
    if not index_path.is_file():
        evidence['notes'].append(f'mesh index missing: {index_path}')
        return evidence
    try:
        payload = json.loads(index_path.read_text(encoding='utf-8'))
    except Exception as exc:  # noqa: BLE001
        evidence['notes'].append(f'mesh index unreadable: {exc}')
        return evidence
    items = payload.get('visual_items') or payload.get('items') or []
    if not isinstance(items, list):
        evidence['notes'].append('mesh index visual_items is not a list')
        return evidence
    for raw in items:
        if not isinstance(raw, dict):
            evidence['skipped_helper_static_fallback_items'] += 1
            continue
        geom = str(raw.get('geometry_type') or '').strip().lower()
        category = str(raw.get('category') or '').strip().lower()
        role = str(raw.get('role') or '').strip().lower()
        transform_status = str(raw.get('transform_status') or '').strip().lower()
        warning = ' '.join(str(raw.get(k) or '') for k in ('warning', 'render_skip_reason', 'fallback_reason')).lower()
        if any(token in category or token in role for token in ('overlay', 'zone', 'helper', 'safety')):
            evidence['zones_overlays_renderable'] += 1
            continue
        if transform_status and transform_status not in {'ok', 'resolved', 'static_fallback', 'static_fallback_parent'}:
            evidence['unresolved_transform_items'] += 1
        if 'missing_parent' in warning or 'missing_chain' in warning or 'unresolved transform' in warning:
            evidence['unresolved_transform_items'] += 1
        if bool(raw.get('primitive_fallback')) or transform_status == 'static_fallback':
            if _dims_available(raw):
                evidence['primitive_fallback_items_renderable'] += 1
            else:
                evidence['skipped_helper_static_fallback_items'] += 1
            continue
        if geom == 'mesh':
            resolved = bool(raw.get('resolved'))
            resolved_path = str(raw.get('resolved_source_path') or raw.get('resolved_path') or '').strip()
            if resolved and resolved_path and Path(resolved_path).is_file():
                evidence['physical_mesh_items_renderable'] += 1
            else:
                evidence['missing_mesh_items'] += 1
            continue
        if geom in {'box', 'cylinder', 'sphere', 'capsule'} and _dims_available(raw):
            evidence['primitive_fallback_items_renderable'] += 1
        else:
            evidence['skipped_helper_static_fallback_items'] += 1
    return evidence


def _scene_package_markers_ok(scene_dir: Path) -> tuple[bool, list[str]]:
    required = ["package.xml", "scene_manifest.yaml", "cell_definition.yaml", "launch/demo.launch.py"]
    missing = [name for name in required if not (scene_dir / name).is_file()]
    return (not missing), missing

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
    ap.add_argument("--workspace-root", type=Path, default=None)
    ap.add_argument("--executable", type=Path, default=None)
    ap.add_argument("--scene", default=None)
    ap.add_argument("--all-scenes", action="store_true")
    ap.add_argument("--scene-path", type=Path, default=None)
    ap.add_argument("--output-dir", type=Path, default=None)
    ap.add_argument("--new-cell-recommended-layout-smoke", action="store_true")
    ap.add_argument("--output", type=Path, default=None)
    ap.add_argument("--screenshot", type=Path, default=None)
    ap.add_argument("--timeout-sec", type=float, default=30.0)
    ap.add_argument("--xvfb", action="store_true")
    args = ap.parse_args()

    if args.all_scenes and args.scene:
        raise SystemExit("Choose only one of --scene or --all-scenes")
    if not args.all_scenes and not args.scene and not args.scene_path and not args.new_cell_recommended_layout_smoke:
        raise SystemExit("Provide one of --scene, --scene-path, --all-scenes, or --new-cell-recommended-layout-smoke")
    if args.all_scenes and args.output is not None:
        raise SystemExit("--output is only valid for single-scene mode")
    if args.all_scenes and args.new_cell_recommended_layout_smoke:
        raise SystemExit("--new-cell-recommended-layout-smoke cannot be combined with --all-scenes")
    if args.all_scenes and args.output_dir is None:
        raise SystemExit("--output-dir is required with --all-scenes")
    if not args.all_scenes and args.output is None:
        raise SystemExit("--output is required unless --all-scenes is used")

    repo_root = resolve_repo_root(explicit_repo_root=args.repo_root)
    workspace_root = resolve_workspace_root(repo_root, args.workspace_root) or repo_root
    exe = resolve_workcell_builder_executable(workspace_root, args.executable)
    executable_resolution = describe_resolution()
    if exe is None and not args.all_scenes:
        searched = list(executable_resolution.get("searched_executable_paths") or [str(p) for p in _resolve_executable_candidates(workspace_root)])
        scene_dir = _resolve_single_scene_dir(repo_root, args.scene, args.scene_path)
        static_evidence = _static_scene3d_visual_evidence(scene_dir)
        fail_payload = {
            "schema": EXPECTED_SCHEMA,
            "status": "FAIL",
            "scene": args.scene or (args.scene_path.name if args.scene_path else None),
            "repo_root": str(repo_root),
            "workspace_root": str(workspace_root),
            "executable": None,
            "searched_paths": searched,
            "executable_resolution": executable_resolution,
            "blockers": ["unable_to_resolve_workcell_builder_executable"],
            "warnings": ["runtime_gui_unavailable_static_scene3d_visual_evidence_recorded"],
            "screenshot_available": False,
            "scene_dir": str(scene_dir) if scene_dir else None,
            "static_scene3d_visual_evidence": static_evidence,
            "render_debug_counters": {
                "runtime_available": False,
                "physical_mesh_items_rendered": 0,
                "primitive_fallback_items_rendered": 0,
                "zones_overlays_rendered": 0,
                "skipped_helper_static_fallback_items": static_evidence.get("skipped_helper_static_fallback_items", 0),
                "unresolved_transform_items": static_evidence.get("unresolved_transform_items", 0),
                "missing_mesh_items": static_evidence.get("missing_mesh_items", 0),
                "static_physical_mesh_items_renderable": static_evidence.get("physical_mesh_items_renderable", 0),
                "static_primitive_fallback_items_renderable": static_evidence.get("primitive_fallback_items_renderable", 0),
                "static_zones_overlays_renderable": static_evidence.get("zones_overlays_renderable", 0),
            },
        }
        _write_json(args.output, fail_payload)
        print("status=FAIL smoke_status=MISSING_EXECUTABLE")
        print("searched_paths=" + " | ".join(searched))
        return 1

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
            cmd += ["--timeout-sec", str(args.timeout_sec)]
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
            "repo_root": str(repo_root), "workspace_root": str(workspace_root), "executable": str(exe) if exe else None,
            "executable_resolution": executable_resolution,
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

    if args.scene_path:
        sp = args.scene_path.resolve()
        ok, missing = _scene_package_markers_ok(sp)
        if not ok:
            fail_payload = {
                "schema": EXPECTED_SCHEMA,
                "status": "FAIL",
                "scene": args.scene or sp.name,
                "scene_path": str(sp),
                "blockers": [f"scene_path_missing_required_files:{','.join(missing)}"],
                "warnings": [],
            }
            _write_json(args.output, fail_payload)
            print("status=FAIL smoke_status=SCENE_PATH_INVALID")
            return 1
        args.scene_path = sp

    cmd = build_cmd(exe, args)
    cmd, xwarn, extra_env = with_xvfb(cmd, args.xvfb)

    stdout_log = args.output.with_suffix(args.output.suffix + ".stdout.log")
    stderr_log = args.output.with_suffix(args.output.suffix + ".stderr.log")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for stale_path in [args.output, stdout_log, stderr_log, args.screenshot]:
        if stale_path and stale_path.exists():
            stale_path.unlink()
    child_env = os.environ.copy()
    child_env.update(extra_env)
    diag = {
        "schema": EXPECTED_SCHEMA,
        "scene": args.scene or (args.scene_path.name if args.scene_path else None),
        "scene_path": str(args.scene_path) if args.scene_path else None,
        "repo_root": str(repo_root),
        "workspace_root": str(workspace_root),
        "executable": str(exe),
        "searched_paths": list(executable_resolution.get("searched_executable_paths") or []),
        "executable_resolution": executable_resolution,
        "child_command": " ".join(shlex.quote(x) for x in cmd),
        "cwd": str(repo_root),
        "env": {k: child_env.get(k, "") for k in ["DISPLAY", "WAYLAND_DISPLAY", "QT_QPA_PLATFORM", "QT_OPENGL", "LIBGL_ALWAYS_SOFTWARE", "XDG_SESSION_TYPE"]},
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
        proc = subprocess.run(cmd, cwd=repo_root, env=child_env, text=True, capture_output=True, timeout=max(0.1, args.timeout_sec), check=False)
        rc = proc.returncode
        stdout, stderr = proc.stdout or "", proc.stderr or ""
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        rc = -1
        stdout, stderr = exc.stdout or "", exc.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode("utf-8", errors="replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode("utf-8", errors="replace")

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
        payload: dict[str, Any] = {}
        try:
            payload=json.loads(args.output.read_text(encoding="utf-8"))
            app_status=payload.get("status","UNKNOWN")
        except Exception:
            payload = {}
        if args.scene_path:
            expected_scene_path = str(args.scene_path.resolve())
            counters = payload.get("counters") if isinstance(payload.get("counters"), dict) else {}
            actual_scene_path = str(counters.get("inspector_scene_path") or counters.get("selected_scene_path") or "").strip()
            if Path(actual_scene_path).as_posix() != Path(expected_scene_path).as_posix():
                blockers = list(payload.get("blockers") or [])
                blockers.append("explicit_scene_path_not_loaded")
                payload["status"] = "FAIL"
                payload["expected_scene_path"] = expected_scene_path
                payload["actual_scene_path"] = actual_scene_path
                payload["blockers"] = blockers
                _write_json(args.output, payload)
                print(f"status=FAIL smoke_status=EXPLICIT_SCENE_PATH_MISMATCH expected_scene_path={expected_scene_path} actual_scene_path={actual_scene_path}")
                return 1
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
