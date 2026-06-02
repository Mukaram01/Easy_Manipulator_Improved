#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any

import yaml

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from scripts.validate_scene3d_visual_quality_matrix import evaluate_scene  # noqa: E402
from scripts.workcell_studio_path_resolver import (  # noqa: E402
    resolve_repo_root,
    resolve_workspace_root,
    resolve_workcell_builder_executable,
)

SCHEMA = "workcell_studio_scene3d_visual_quality_screenshots/v1"
SMOKE_SCHEMA = "workcell_studio_scene3d_gui_smoke/v1"


_COUNTER_KEYS = (
    "preview_items_count",
    "total_payload_count",
    "viewport_received_count",
    "render_cache_count",
    "visible_count",
    "rendered_count",
    "skipped_count",
    "unique_visible_item_count",
    "mesh_source_count",
    "mesh_backed_count",
    "mesh_rendered_count",
    "urdf_primitive_source_count",
    "urdf_primitive_rendered_count",
    "primitive_rendered_count",
    "primitive_fallback_count",
    "placeholder_count",
    "missing_geometry_count",
    "wireframe_fallback_count",
    "overlay_helper_count",
    "locked_generated_urdf_visual_count",
    "editable_layout_count",
    "visual_quality_status",
    "visual_quality_warnings",
    "last_paint_completed",
    "smoke_fallback_render_used",
    "paint_cycle_completed",
)

_PRIMITIVE_KEYS = (
    "urdf_primitive_source_count",
    "urdf_primitive_rendered_count",
    "primitive_rendered_count",
    "primitive_fallback_count",
    "placeholder_count",
    "missing_geometry_count",
    "wireframe_fallback_count",
)

_MESH_FAILURE_REASON_KEYS = (
    "reason_code",
    "failure_reason_code",
    "mesh_failure_reason_code",
    "mesh_failure_reason",
    "failure_reason",
    "unresolved_reason",
    "reason",
    "warning",
)

_MESH_HINT_KEYS = ("mesh", "mesh_path", "mesh_uri", "mesh_filename", "filename", "resource", "uri", "mesh_extension")

_BLOCKER_CATEGORY_KEYS = (
    "missing_screenshot",
    "smoke_json_missing",
    "smoke_json_unreadable",
    "mesh_source_not_rendered",
    "urdf_primitive_source_not_rendered",
    "placeholder_missing_geometry_dominates",
    "overlay_helper_dominates",
    "no_physical_scene_items_rendered",
)

_BLOCKER_ACTION_TEXT = {
    "missing_screenshot": "capture or attach the Scene3D smoke screenshot",
    "smoke_json_missing": "run the Scene3D GUI smoke command so the smoke JSON is written",
    "smoke_json_unreadable": "fix or regenerate the malformed Scene3D smoke JSON",
    "mesh_source_not_rendered": "fix mesh handoff/rendering so mesh-backed source items render",
    "urdf_primitive_source_not_rendered": "fix URDF primitive rendering counters so primitive source items render",
    "placeholder_missing_geometry_dominates": "replace dominant placeholder/missing-geometry visuals with mesh or primitive geometry",
    "overlay_helper_dominates": "render physical scene items; overlay helpers cannot prove visual quality",
    "no_physical_scene_items_rendered": "render at least one physical mesh, primitive, or fallback scene item",
}


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _read_json(path: Path) -> dict[str, Any]:
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"_load_error": str(exc)}
    return loaded if isinstance(loaded, dict) else {"_load_error": "JSON root is not an object"}


def _scene_entries(catalog_path: Path) -> list[dict[str, Any]]:
    try:
        catalog = yaml.safe_load(catalog_path.read_text(encoding="utf-8")) or {}
    except Exception as exc:
        raise SystemExit(f"Unable to read supported scenes catalog {catalog_path}: {exc}") from exc
    scenes = catalog.get("scenes")
    if not isinstance(scenes, list):
        raise SystemExit(f"Supported scenes catalog has no scenes list: {catalog_path}")
    entries: list[dict[str, Any]] = []
    for entry in scenes:
        if not isinstance(entry, dict):
            continue
        enabled = bool(entry.get("enabled", True))
        status = str(entry.get("status") or "").strip().lower()
        support_level = str(entry.get("support_level") or "").strip().lower()
        if enabled and status not in {"disabled", "ignored"} and support_level not in {"disabled", "ignored"}:
            entries.append(entry)
    return entries


def _resolve_scene_dir(repo_root: Path, scene_name: str, scene_path: str | None = None) -> Path:
    raw = Path(scene_path) if scene_path else Path("scenes") / scene_name
    return raw if raw.is_absolute() else repo_root / raw


def _dedupe_targets(targets: list[dict[str, Any]]) -> list[dict[str, Any]]:
    seen: set[tuple[str, str]] = set()
    out: list[dict[str, Any]] = []
    for target in targets:
        key = (str(target.get("target_kind")), str(target.get("scene_path") or target.get("scene_name")))
        if key in seen:
            continue
        seen.add(key)
        out.append(target)
    return out


def collect_targets(repo_root: Path, scenes: list[str], supported_scenes: Path | None, synthetic_fixture: Path | None) -> list[dict[str, Any]]:
    targets: list[dict[str, Any]] = []
    for scene in scenes:
        scene_name = scene.strip()
        if not scene_name:
            continue
        targets.append({
            "target_kind": "explicit_scene",
            "scene_name": scene_name,
            "scene_path": str(_resolve_scene_dir(repo_root, scene_name)),
        })
    if supported_scenes is not None:
        catalog_path = supported_scenes if supported_scenes.is_absolute() else repo_root / supported_scenes
        for entry in _scene_entries(catalog_path):
            scene_name = str(entry.get("scene_name") or entry.get("package_name") or "").strip()
            if not scene_name:
                continue
            targets.append({
                "target_kind": "supported_scene_catalog",
                "scene_name": scene_name,
                "scene_path": str(_resolve_scene_dir(repo_root, scene_name, entry.get("scene_path"))),
                "catalog_entry": entry,
            })
    if synthetic_fixture is not None:
        fixture = synthetic_fixture if synthetic_fixture.is_absolute() else repo_root / synthetic_fixture
        scene_dir = fixture if fixture.is_dir() else fixture.parent
        targets.append({
            "target_kind": "synthetic_fixture",
            "scene_name": scene_dir.name or "synthetic_visual_quality_fixture",
            "scene_path": str(scene_dir),
            "synthetic_fixture": str(fixture),
        })
    return _dedupe_targets(targets)


def _scene_package_missing(scene_dir: Path) -> list[str]:
    required = ["package.xml", "scene_manifest.yaml", "cell_definition.yaml", "launch/demo.launch.py"]
    return [name for name in required if not (scene_dir / name).is_file()]


def _mesh_index_path(scene_dir: Path, fixture_path: Path | None = None) -> Path:
    if fixture_path is not None and fixture_path.is_file() and fixture_path.suffix == ".json":
        return fixture_path
    if scene_dir.is_file():
        return scene_dir
    return scene_dir / "generated" / "scene_visual_mesh_index.json"


def _has_mesh_hint(item: dict[str, Any]) -> bool:
    for key in _MESH_HINT_KEYS:
        value = item.get(key)
        if isinstance(value, str) and value.strip():
            return True
        if isinstance(value, dict) and any(str(value.get(k) or "").strip() for k in ("filename", "uri", "path")):
            return True
    geometry = item.get("geometry")
    return isinstance(geometry, dict) and _has_mesh_hint(geometry)


def _reason_code_for_item(item: dict[str, Any]) -> str:
    for key in _MESH_FAILURE_REASON_KEYS:
        value = item.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip().lower().replace(" ", "_")
    if item.get("resolved") is False:
        return "unresolved_mesh_without_reason_code"
    return "unknown"


def mesh_failure_summary(mesh_index_path: Path) -> dict[str, Any]:
    if not mesh_index_path.exists():
        return {"mesh_index_path": str(mesh_index_path), "total_failed_meshes": 0, "by_reason_code": {"mesh_index_missing": 1}}
    payload = _read_json(mesh_index_path)
    if payload.get("_load_error"):
        return {"mesh_index_path": str(mesh_index_path), "total_failed_meshes": 0, "by_reason_code": {"mesh_index_unreadable": 1}, "error": payload["_load_error"]}
    reasons: Counter[str] = Counter()
    top_reason = payload.get("fallback_reason")
    if isinstance(top_reason, str) and top_reason.strip() and not payload.get("safe_for_preview", True):
        reasons[top_reason.strip().lower().replace(" ", "_")] += 1
    for raw in [*(payload.get("visual_items") if isinstance(payload.get("visual_items"), list) else []), *(payload.get("items") if isinstance(payload.get("items"), list) else [])]:
        if not isinstance(raw, dict):
            continue
        if raw.get("resolved") is False and (_has_mesh_hint(raw) or raw.get("render_expected", False)):
            reasons[_reason_code_for_item(raw)] += 1
    unresolved = payload.get("unresolved")
    if isinstance(unresolved, list):
        for raw in unresolved:
            if isinstance(raw, dict):
                reasons[_reason_code_for_item(raw)] += 1
            elif isinstance(raw, str) and raw.strip():
                reasons[raw.strip().lower().replace(" ", "_")] += 1
    return {
        "mesh_index_path": str(mesh_index_path),
        "total_failed_meshes": int(sum(reasons.values())),
        "by_reason_code": dict(sorted(reasons.items())),
    }


def _counter_summary(smoke_payload: dict[str, Any]) -> dict[str, Any]:
    counters = smoke_payload.get("counters")
    if not isinstance(counters, dict):
        counters = smoke_payload.get("render_debug_counters") if isinstance(smoke_payload.get("render_debug_counters"), dict) else {}
    return {key: counters[key] for key in _COUNTER_KEYS if key in counters}


def _primitive_summary(counter_summary: dict[str, Any]) -> dict[str, Any]:
    out = {key: counter_summary[key] for key in _PRIMITIVE_KEYS if key in counter_summary}
    if "primitive_rendered_count" not in out and "urdf_primitive_rendered_count" in out:
        out["primitive_rendered_count"] = out["urdf_primitive_rendered_count"]
    return out


def _as_int(value: Any, default: int = 0) -> int:
    try:
        if value is None or isinstance(value, bool):
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _add_reason(reasons: list[str], reason: str) -> None:
    if reason and reason not in reasons:
        reasons.append(reason)


def _normalize_blocker_text(text: str) -> str | None:
    lowered = text.strip().lower()
    if not lowered:
        return None
    if "smoke_json_missing" in lowered:
        return "smoke_json_missing"
    if "smoke_json_unreadable" in lowered:
        return "smoke_json_unreadable"
    if "screenshot_missing" in lowered or "missing_screenshot" in lowered:
        return "missing_screenshot"
    if "mesh_source_not_rendered" in lowered or "mesh_source_count > 0 requires mesh_rendered_count > 0" in lowered:
        return "mesh_source_not_rendered"
    if (
        "urdf_primitive_source_not_rendered" in lowered
        or "primitive_source_not_rendered" in lowered
        or "primitive_source_count > 0 requires primitive_rendered_count > 0" in lowered
    ):
        return "urdf_primitive_source_not_rendered"
    if (
        "placeholder_missing_geometry_dominates" in lowered
        or "valid urdf primitives must not increment placeholder_count" in lowered
        or "wireframe_fallback_count dominates" in lowered
    ):
        return "placeholder_missing_geometry_dominates"
    if "overlay_helper_dominates" in lowered or ("overlay helper" in lowered and "dominat" in lowered):
        return "overlay_helper_dominates"
    if "no_physical_scene_items_rendered" in lowered:
        return "no_physical_scene_items_rendered"
    return lowered.split(":", 1)[0].strip().replace(" ", "_")


def _normalized_blocker_reasons(
    *,
    wrapper_blockers: list[str],
    smoke_payload: dict[str, Any],
    visual_quality: dict[str, Any],
    counter_summary: dict[str, Any],
    screenshot: Path,
) -> list[str]:
    reasons: list[str] = []
    candidate_text: list[str] = [str(x) for x in wrapper_blockers]
    for key in ("blockers", "blocker_reasons"):
        values = smoke_payload.get(key)
        if isinstance(values, list):
            candidate_text.extend(str(x) for x in values)
    for key in ("blockers", "blocker_reasons"):
        values = visual_quality.get(key)
        if isinstance(values, list):
            candidate_text.extend(str(x) for x in values)
    for text in candidate_text:
        normalized = _normalize_blocker_text(text)
        if normalized:
            _add_reason(reasons, normalized)

    if not screenshot.exists():
        _add_reason(reasons, "missing_screenshot")
    if smoke_payload.get("_load_error"):
        _add_reason(reasons, "smoke_json_unreadable")

    mesh_source_count = max(_as_int(counter_summary.get("mesh_source_count")), _as_int(visual_quality.get("mesh_source_count")))
    mesh_rendered_count = max(_as_int(counter_summary.get("mesh_rendered_count")), _as_int(visual_quality.get("mesh_rendered_count")))
    primitive_source_count = max(
        _as_int(counter_summary.get("urdf_primitive_source_count")),
        _as_int(counter_summary.get("primitive_source_count")),
        _as_int(visual_quality.get("primitive_source_count")),
    )
    primitive_rendered_count = max(
        _as_int(counter_summary.get("urdf_primitive_rendered_count")),
        _as_int(counter_summary.get("primitive_rendered_count")),
        _as_int(visual_quality.get("primitive_rendered_count")),
    )
    placeholder_count = max(_as_int(counter_summary.get("placeholder_count")), _as_int(visual_quality.get("placeholder_count")))
    missing_geometry_count = max(_as_int(counter_summary.get("missing_geometry_count")), _as_int(visual_quality.get("missing_geometry_count")))
    wireframe_fallback_count = max(_as_int(counter_summary.get("wireframe_fallback_count")), _as_int(visual_quality.get("wireframe_fallback_count")))
    overlay_helper_count = _as_int(counter_summary.get("overlay_helper_count"))
    rendered_count = max(_as_int(counter_summary.get("rendered_count")), _as_int(visual_quality.get("rendered_count")))
    physical_rendered_count = mesh_rendered_count + primitive_rendered_count + placeholder_count + wireframe_fallback_count
    source_count = mesh_source_count + primitive_source_count

    if mesh_source_count > 0 and mesh_rendered_count <= 0:
        _add_reason(reasons, "mesh_source_not_rendered")
    if primitive_source_count > 0 and primitive_rendered_count <= 0:
        _add_reason(reasons, "urdf_primitive_source_not_rendered")
    placeholder_or_missing_count = placeholder_count + missing_geometry_count + wireframe_fallback_count
    credible_rendered_count = mesh_rendered_count + primitive_rendered_count
    if placeholder_or_missing_count > max(credible_rendered_count, 0) and placeholder_or_missing_count > 0:
        _add_reason(reasons, "placeholder_missing_geometry_dominates")
    if overlay_helper_count > max(physical_rendered_count, 0) and overlay_helper_count > 0:
        _add_reason(reasons, "overlay_helper_dominates")
    if source_count > 0 and physical_rendered_count <= 0:
        _add_reason(reasons, "no_physical_scene_items_rendered")
    if rendered_count > 0 and overlay_helper_count >= rendered_count and physical_rendered_count <= 0:
        _add_reason(reasons, "no_physical_scene_items_rendered")

    return reasons


def _blocker_reason_summary(results: list[dict[str, Any]]) -> dict[str, int]:
    counts = {key: 0 for key in _BLOCKER_CATEGORY_KEYS}
    for result in results:
        reason_set = set(str(reason) for reason in result.get("blocker_reasons", []))
        for key in _BLOCKER_CATEGORY_KEYS:
            if key in reason_set:
                counts[key] += 1
    return counts


def _blocker_reason_text(reasons: list[str]) -> str:
    if not reasons:
        return "none"
    return "; ".join(_BLOCKER_ACTION_TEXT.get(reason, reason) for reason in reasons)


def _run_smoke_for_target(
    *,
    repo_root: Path,
    workspace_root: Path,
    executable: Path | None,
    target: dict[str, Any],
    output_dir: Path,
    timeout_sec: float,
    xvfb: bool,
) -> tuple[int, Path, Path, dict[str, Any], list[str], str]:
    scene_name = str(target["scene_name"])
    safe_name = scene_name.replace(os.sep, "_")
    smoke_json = output_dir / f"scene3d_gui_smoke_{safe_name}.json"
    screenshot = output_dir / f"scene3d_gui_smoke_{safe_name}.png"
    blockers: list[str] = []

    scene_dir = Path(str(target["scene_path"]))
    command = [
        sys.executable,
        str(repo_root / "scripts" / "run_workcell_builder_scene3d_gui_smoke.py"),
        "--repo-root",
        str(repo_root),
        "--workspace-root",
        str(workspace_root),
        "--output",
        str(smoke_json),
        "--screenshot",
        str(screenshot),
        "--timeout-sec",
        str(timeout_sec),
    ]
    if executable is not None:
        command.extend(["--executable", str(executable)])

    if target.get("target_kind") == "synthetic_fixture":
        missing = _scene_package_missing(scene_dir)
        if missing:
            blockers.append("synthetic_fixture_not_scene_package_for_gui_smoke:" + ",".join(missing))
            payload = {
                "schema": SMOKE_SCHEMA,
                "status": "BLOCKED",
                "scene": scene_name,
                "scene_path": str(scene_dir),
                "blockers": blockers,
                "warnings": [],
                "screenshot_path": str(screenshot),
                "screenshot_available": False,
            }
            _write_json(smoke_json, payload)
            return 1, smoke_json, screenshot, payload, blockers, "synthetic fixture lacks package markers; GUI smoke not launched"
        command.extend(["--scene-path", str(scene_dir)])
    else:
        command.extend(["--scene", scene_name])

    if xvfb:
        command.append("--xvfb")
    proc = subprocess.run(command, cwd=repo_root, capture_output=True, text=True, check=False)
    payload = _read_json(smoke_json) if smoke_json.exists() else {}
    if not smoke_json.exists():
        blockers.append("smoke_json_missing")
    return proc.returncode, smoke_json, screenshot, payload, blockers, " ".join(shlex.quote(x) for x in command)


def build_result_for_target(
    *,
    repo_root: Path,
    workspace_root: Path,
    executable: Path | None,
    target: dict[str, Any],
    output_dir: Path,
    timeout_sec: float,
    xvfb: bool,
) -> dict[str, Any]:
    rc, smoke_json, screenshot, smoke_payload, wrapper_blockers, command = _run_smoke_for_target(
        repo_root=repo_root,
        workspace_root=workspace_root,
        executable=executable,
        target=target,
        output_dir=output_dir,
        timeout_sec=timeout_sec,
        xvfb=xvfb,
    )
    scene_dir = Path(str(target["scene_path"]))
    fixture_path = Path(str(target["synthetic_fixture"])) if target.get("synthetic_fixture") else None
    mesh_index = _mesh_index_path(scene_dir, fixture_path)
    visual_quality = evaluate_scene(
        scene_name=str(target["scene_name"]),
        scene_dir=scene_dir,
        mesh_index_path=mesh_index,
        smoke_json_path=smoke_json,
        screenshot_path=screenshot,
        synthetic_fixture=target.get("target_kind") == "synthetic_fixture",
    )
    counter_summary = _counter_summary(smoke_payload)
    all_blockers = list(wrapper_blockers)
    if isinstance(smoke_payload.get("blockers"), list):
        all_blockers.extend(str(x) for x in smoke_payload["blockers"])
    all_blockers.extend(str(x) for x in visual_quality.get("blockers", []))
    all_blocker_reasons = _normalized_blocker_reasons(
        wrapper_blockers=wrapper_blockers,
        smoke_payload=smoke_payload,
        visual_quality=visual_quality,
        counter_summary=counter_summary,
        screenshot=screenshot,
    )
    blocker_categories = {key: key in all_blocker_reasons for key in _BLOCKER_CATEGORY_KEYS}
    status = "PASS" if rc == 0 and not all_blockers and str(smoke_payload.get("status", "")).upper() in {"PASS", "OK"} else "FAIL"
    if str(smoke_payload.get("status", "")).upper() == "BLOCKED" or wrapper_blockers or all_blocker_reasons:
        status = "BLOCKED"
    return {
        "scene": target["scene_name"],
        "target_kind": target["target_kind"],
        "scene_path": str(scene_dir),
        "status": status,
        "returncode": rc,
        "command": command,
        "smoke_json": str(smoke_json),
        "screenshot_path": str(screenshot),
        "screenshot_available": bool(screenshot.exists()),
        "visual_quality_counter_summary": counter_summary,
        "primitive_render_summary": _primitive_summary(counter_summary),
        "mesh_failure_summary_by_reason_code": mesh_failure_summary(mesh_index),
        "visual_quality_evaluation": visual_quality,
        "blockers": all_blockers,
        "blocker_reasons": all_blocker_reasons,
        "blocker_categories": blocker_categories,
        "blocker_text": _blocker_reason_text(all_blocker_reasons),
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture Scene3D GUI smoke screenshots and visual-quality summaries for a scene list.")
    parser.add_argument("--repo-root", type=Path, default=_REPO_ROOT)
    parser.add_argument("--workspace-root", type=Path, default=_REPO_ROOT)
    parser.add_argument("--executable", type=Path, default=None)
    parser.add_argument("--scene", action="append", default=[], help="Scene package name. May be repeated.")
    parser.add_argument("--supported-scenes", type=Path, default=None, help="Append enabled scenes from a supported_scenes.yaml catalog.")
    parser.add_argument("--synthetic-fixture", type=Path, default=None, help="Append a generated synthetic fixture directory or mesh-index JSON.")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--xvfb", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    repo_root = resolve_repo_root(explicit_repo_root=args.repo_root)
    workspace_root = resolve_workspace_root(repo_root, args.workspace_root)
    executable = args.executable or resolve_workcell_builder_executable(workspace_root)
    targets = collect_targets(repo_root, args.scene, args.supported_scenes, args.synthetic_fixture)
    if not targets:
        raise SystemExit("Provide at least one --scene, --supported-scenes catalog, or --synthetic-fixture path")
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    results = [
        build_result_for_target(
            repo_root=repo_root,
            workspace_root=workspace_root,
            executable=executable,
            target=target,
            output_dir=output_dir,
            timeout_sec=args.timeout_sec,
            xvfb=args.xvfb,
        )
        for target in targets
    ]
    totals = dict(Counter(str(result["status"]) for result in results))
    for key in ("PASS", "FAIL", "BLOCKED"):
        totals.setdefault(key, 0)
    blocker_reason_summary = _blocker_reason_summary(results)
    payload = {
        "schema": SCHEMA,
        "repo_root": str(repo_root),
        "workspace_root": str(workspace_root),
        "executable": str(executable) if executable else None,
        "output_dir": str(output_dir),
        "scene_count": len(results),
        "totals": totals,
        "blocker_reason_summary": blocker_reason_summary,
        **blocker_reason_summary,
        "results": results,
        "pass": totals.get("FAIL", 0) == 0 and totals.get("BLOCKED", 0) == 0,
    }
    _write_json(output_dir / "scene3d_visual_quality_screenshots_summary.json", payload)
    md = [
        "# Scene3D Visual Quality Screenshots",
        "",
        f"- scene_count: {len(results)}",
        f"- PASS: {totals['PASS']}",
        f"- FAIL: {totals['FAIL']}",
        f"- BLOCKED: {totals['BLOCKED']}",
        "",
        "## Blocker category summary",
        "",
        *[f"- {key}: {blocker_reason_summary[key]}" for key in _BLOCKER_CATEGORY_KEYS],
        "",
        "| Scene | Status | Smoke JSON | Screenshot | Blockers | Mesh failure reasons |",
        "|---|---|---|---|---|---|",
    ]
    for result in results:
        reasons = result["mesh_failure_summary_by_reason_code"].get("by_reason_code", {})
        reason_text = ", ".join(f"{k}={v}" for k, v in reasons.items()) or "none"
        blocker_reason_text = result.get("blocker_text") or _blocker_reason_text([str(reason) for reason in result.get("blocker_reasons", [])])
        md.append(
            f"| {result['scene']} | {result['status']} | `{result['smoke_json']}` | "
            f"`{result['screenshot_path']}` | {blocker_reason_text} | {reason_text} |"
        )
    (output_dir / "scene3d_visual_quality_screenshots_summary.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0 if payload["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
