#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

from scripts.workcell_studio_path_resolver import resolve_repo_root, resolve_workspace_root, resolve_workcell_builder_executable
ROOT = Path(__file__).resolve().parents[1]
EXPECTED_SCHEMA = "workcell_studio_scene3d_gui_smoke/v1"
PASS_STATES = {"ok", "pass", "passed"}


def _is_truthy_env(name: str) -> bool:
    value = str(os.environ.get(name, "")).strip().lower()
    return value in {"1", "true", "yes", "on"}


def _resolve_executable_candidates(workspace_root: Path) -> list[Path]:
    candidates: list[Path] = []
    path_hit = shutil.which("workcell_builder")
    if path_hit:
        candidates.append(Path(path_hit))
    candidates.extend([
        workspace_root / "install/workcell_builder/lib/workcell_builder/workcell_builder",
        workspace_root / "build/workcell_builder/workcell_builder",
        workspace_root / "build/workcell_builder/workcell_builder_node",
    ])
    dedup: list[Path] = []
    seen: set[str] = set()
    for candidate in candidates:
        key = str(candidate)
        if key not in seen:
            seen.add(key)
            dedup.append(candidate)
    return dedup


def resolve_workcell_builder(workspace_root: Path) -> tuple[Path | None, list[Path]]:
    candidates = _resolve_executable_candidates(workspace_root)
    for candidate in candidates:
        if candidate.is_file():
            return candidate, candidates
    return None, candidates


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

    required_fields = [
        "scene",
        "status",
        "screenshot_path",
        "unique_visible_item_count",
        "mesh_rendered_count",
        "generated_fallback_count",
        "editable_layout_count",
        "primitive_fallback_count",
        "overlay_count",
        "labels_drawn",
        "labels_suppressed_overlap",
        "hierarchy_child_row_count",
        "selected_scene_name",
        "selected_item_id",
        "blockers",
        "warnings",
    ]
    missing_or_bad: list[str] = [field for field in required_fields if field not in payload]

    for field in ["viewport_received_count", "render_cache_count", "rendered_count", "hierarchy_rows_count", "selectable_count"]:
        if field in payload and (not isinstance(payload.get(field), int) or int(payload.get(field)) < 0):
            missing_or_bad.append(field)

    int_fields = [
        "unique_visible_item_count",
        "mesh_rendered_count",
        "generated_fallback_count",
        "editable_layout_count",
        "primitive_fallback_count",
        "overlay_count",
        "labels_drawn",
        "labels_suppressed_overlap",
        "hierarchy_child_row_count",
    ]
    for field in int_fields:
        value = payload.get(field)
        if not isinstance(value, int) or value < 0:
            missing_or_bad.append(field)
    raw_status = payload.get("status")
    if not isinstance(raw_status, str):
        missing_or_bad.append("status")
    else:
        status = raw_status
    for string_field in ["scene", "screenshot_path", "selected_scene_name", "selected_item_id"]:
        if not isinstance(payload.get(string_field), str):
            missing_or_bad.append(string_field)
    for list_field in ["blockers", "warnings"]:
        if not isinstance(payload.get(list_field), list):
            missing_or_bad.append(list_field)
    if missing_or_bad:
        blockers.append("missing/invalid required fields: " + ", ".join(sorted(set(missing_or_bad))))

    if isinstance(raw_status, str) and raw_status.lower() not in PASS_STATES:
        blockers.append(f"status is FAIL-like: {raw_status!r}")

    scene_name = str(payload.get("scene") or "").strip()
    unique_visible_item_count = int(payload.get("unique_visible_item_count") or 0)
    if scene_name in {"ur5_2f_test", "ur5_2f_sorting_test"} and unique_visible_item_count <= 1:
        blockers.append(f"{scene_name}: unique_visible_item_count <= 1 ({unique_visible_item_count})")

    total_renderable = sum(int(payload.get(k) or 0) for k in ["mesh_rendered_count", "generated_fallback_count", "editable_layout_count", "primitive_fallback_count"])
    if total_renderable == 0:
        blockers.append("mesh_rendered_count + generated_fallback_count + editable_layout_count + primitive_fallback_count == 0")

    hierarchy_child_row_count = int(payload.get("hierarchy_child_row_count") or 0)
    if hierarchy_child_row_count == 0:
        blockers.append("hierarchy_child_row_count == 0")

    selected_scene_name = str(payload.get("selected_scene_name") or "").strip()
    if not selected_scene_name:
        blockers.append("selected_scene_name is empty")

    label_mode_value = " ".join(
        str(payload.get(k) or "").strip().lower()
        for k in ("label_mode", "labels_mode", "overlay_label_mode")
    )
    labels_drawn = int(payload.get("labels_drawn") or 0)
    if "selected" in label_mode_value and labels_drawn > 1:
        blockers.append(f"labels_drawn excessive for Selected-label mode: {labels_drawn}")

    missing_required_classification = payload.get("missing_required_classification_fields")
    if isinstance(missing_required_classification, list) and missing_required_classification:
        blockers.append(
            "items missing required classification fields: " + ", ".join(str(v) for v in missing_required_classification)
        )
    classification_missing_count = payload.get("classification_missing_count")
    if isinstance(classification_missing_count, int) and classification_missing_count > 0:
        blockers.append(f"items missing required classification fields (count={classification_missing_count})")

    warnings.extend(payload.get("warnings", []) if isinstance(payload.get("warnings"), list) else [])
    return blockers, warnings, status


def main() -> int:
    ap = argparse.ArgumentParser(description="Run Workcell Builder Scene3D GUI smoke and validate output")
    ap.add_argument("--repo-root", type=Path, default=ROOT)
    ap.add_argument("--workspace-root", type=Path, default=ROOT)
    ap.add_argument("--executable", type=Path, default=None)
    ap.add_argument("--scene", default=None)
    ap.add_argument("--new-cell-recommended-layout-smoke", action="store_true")
    ap.add_argument("--output", type=Path, default=ROOT / "build/workcell_studio/scene3d_gui_smoke.json")
    ap.add_argument("--screenshot", type=Path, default=None)
    ap.add_argument("--timeout-sec", type=float, default=30.0)
    ap.add_argument("--xvfb", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    blockers: list[str] = []
    warnings: list[str] = []
    artifacts = {"json": str(args.output), "screenshot": str(args.screenshot) if args.screenshot else None}
    extra_resolution = {"repo_root": None, "workspace_root": None}

    repo_root = resolve_repo_root(explicit_repo_root=args.repo_root)
    extra_resolution["repo_root"] = str(repo_root)
    extra_resolution["workspace_root"] = str(workspace_root) if workspace_root else None
    workspace_root = resolve_workspace_root(repo_root, args.workspace_root)
    if args.executable is not None:
        exe = args.executable
        searched_candidates: list[Path] = [args.executable]
    else:
        exe = resolve_workcell_builder_executable(workspace_root)
        searched_candidates = _resolve_executable_candidates(workspace_root or repo_root)

    if exe is None or not Path(exe).is_file():
        searched = " | ".join(str(p) for p in searched_candidates)
        message = f"unable to resolve workcell_builder executable; searched={searched}"
        allow_downgrade = bool(args.dry_run or _is_truthy_env("CI"))
        if allow_downgrade:
            warnings.append("DOWNGRADED_MISSING_EXECUTABLE")
            warnings.append(message)
            print("status=WARN smoke_status=SKIP elapsed_sec=0.00")
            print(f"blockers=0 warnings={len(warnings)}")
            print("warning_list=" + " | ".join(str(w) for w in warnings))
            print("artifacts=" + json.dumps({**artifacts, **extra_resolution}, sort_keys=True))
            return 0
        print(f"status=FAIL; blockers=[{message}]")
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
    print("artifacts=" + json.dumps({**artifacts, **extra_resolution}, sort_keys=True))

    return 0 if not blockers else 1


if __name__ == "__main__":
    raise SystemExit(main())
