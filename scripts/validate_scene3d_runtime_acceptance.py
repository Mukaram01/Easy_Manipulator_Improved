#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

import sys

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from scripts.workcell_studio_script_bootstrap import ensure_repo_root_on_sys_path

ensure_repo_root_on_sys_path(__file__)

from scripts.scene_root_resolver import resolve_scene_root
from scripts.scene3d_scene_discovery import discover_scene3d_scenes

SCHEMA = "workcell_studio_scene3d_runtime_acceptance/v1"
CANONICAL_LAYERS = {"editable_layout", "mesh_preview", "locked_generated_urdf_visual", "primitive_fallback", "overlay"}


def normalize_layer_token(value: str | None) -> str:
    token = (value or "").strip().lower().replace("-", "_").replace(" ", "_")
    if token in {"generated_preview", "generated_urdf_visual", "locked_generated_urdf"}:
        return "locked_generated_urdf_visual"
    if token == "legacy_static_fallback":
        return "primitive_fallback"
    if token in {"overlays", "helper_overlay"}:
        return "overlay"
    return token


def scene_dir(scenes_root: Path, scene: str) -> Path:
    return scenes_root / scene


def load_yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}


def load_json(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}


def _runtime_counter(payload: dict, key: str) -> int:
    if not isinstance(payload, dict):
        return 0
    if key in payload:
        return int(payload.get(key) or 0)
    counters = payload.get("counters")
    if isinstance(counters, dict):
        alias = {"hierarchy_rows_count": "hierarchy_rows"}.get(key, key)
        return int(counters.get(alias) or 0)
    return 0


def runtime_smoke_json_default(repo_root: Path, scenes_root: Path, scene: str) -> Path:
    candidates = [
        repo_root / "build/workcell_studio/scene3d_gui_smoke.json",
        repo_root / f"build/workcell_studio/scene3d_gui_smoke_{scene}.json",
        scene_dir(scenes_root, scene) / "generated/scene3d_gui_smoke.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def evaluate_scene(repo_root: Path, scenes_root: Path, main_path: Path, preview_path: Path, viewport_path: Path, scene: str, smoke_json_path: str | None) -> dict:
    sdir = scene_dir(scenes_root, scene)
    blockers: list[str] = []

    if not sdir.exists():
        blockers.append(f"scene directory is missing: {sdir}")
        return {
            "scene": scene,
            "scene_path": str(sdir),
            "blockers": blockers,
            "counts": {},
            "visibility_contract": {
                "input_items_count": 0,
                "visible_after_default_filters": 0,
                "hidden_by_filters_count": 0,
            },
            "layers": {},
            "sources": {},
            "pass": False,
        }

    layout_path = sdir / "layout/workcell_studio_layout.yaml"
    mesh_index_path = sdir / "generated/scene_visual_mesh_index.json"
    preview_metadata_path = sdir / "generated/scene_preview_metadata.json"
    urdf_generated_layout_path = sdir / "layout/workcell_studio_layout.generated.yaml"
    overlay_sources = [
        sdir / "generated/epd_snapshot.json",
        sdir / "generated/detections_snapshot.json",
        sdir / "config/epd_snapshot.json",
    ]

    if not layout_path.exists():
        blockers.append(f"missing layout: {layout_path}")
        layout = {}
    else:
        layout = load_yaml(layout_path)

    if not mesh_index_path.exists():
        blockers.append(f"missing mesh index: {mesh_index_path}")
        mesh_index = {}
    else:
        mesh_index = load_json(mesh_index_path)

    preview_metadata = load_json(preview_metadata_path) if preview_metadata_path.exists() else {}
    generated_layout = load_yaml(urdf_generated_layout_path) if urdf_generated_layout_path.exists() else {}

    layout_items = layout.get("items") or []
    editable_layout_count = len(layout_items)

    visual_items = mesh_index.get("visual_items") or []
    safe_for_preview = bool(mesh_index.get("safe_for_preview"))
    mesh_preview_count = len(visual_items) if safe_for_preview else 0
    primitive_fallback_count = int(mesh_index.get("unresolved_placeholder_count") or 0)

    generated_items = generated_layout.get("items") or []
    locked_generated_urdf_visual_count = sum(
        1 for item in generated_items if normalize_layer_token(item.get("source")) == "locked_generated_urdf_visual"
    )

    overlay_count = 0
    if preview_metadata:
        overlay_items = preview_metadata.get("overlays")
        if isinstance(overlay_items, list):
            overlay_count = len(overlay_items)
    if overlay_count == 0:
        overlay_count = sum(1 for p in overlay_sources if p.exists())

    source_layer_counts: dict[str, int] = {}
    for item in (mesh_index.get("items") or []):
        layer = normalize_layer_token(item.get("source_layer") or item.get("item_source"))
        if layer:
            source_layer_counts[layer] = source_layer_counts.get(layer, 0) + 1
    missing_count = sum(count for layer, count in source_layer_counts.items() if layer not in CANONICAL_LAYERS)

    input_items_count = editable_layout_count + primitive_fallback_count + mesh_preview_count + locked_generated_urdf_visual_count + overlay_count + missing_count
    visible_after_default_filters = editable_layout_count + mesh_preview_count
    hidden_by_filters_count = max(input_items_count - visible_after_default_filters, 0)

    if not blockers and visible_after_default_filters == 0:
        blockers.append("no visible scene items after default filters")

    smoke_path = Path(smoke_json_path) if smoke_json_path else runtime_smoke_json_default(repo_root, scenes_root, scene)
    runtime_evidence: dict = {"path": str(smoke_path), "valid": False}
    required_runtime_fields = [
        "viewport_received_count",
        "render_cache_count",
        "rendered_count",
        "hierarchy_rows_count",
        "selectable_count",
        "status",
    ]
    runtime_payload = {}
    if not smoke_path.exists():
        blockers.append(f"missing runtime smoke evidence JSON: {smoke_path}")
        runtime_evidence["error"] = "missing"
    else:
        runtime_payload = load_json(smoke_path)
        schema = runtime_payload.get("schema")
        if schema != "workcell_studio_scene3d_gui_smoke/v1":
            blockers.append(
                "runtime smoke evidence has unexpected schema: "
                f"{schema!r} (expected 'workcell_studio_scene3d_gui_smoke/v1')"
            )

        invalid_fields: list[str] = []
        for field in required_runtime_fields:
            if field not in runtime_payload:
                if field != "status" and _runtime_counter(runtime_payload, field) >= 0:
                    continue
                invalid_fields.append(field)
                continue
            value = runtime_payload[field]
            if field == "status":
                if not isinstance(value, str):
                    invalid_fields.append(field)
            else:
                if not isinstance(value, int) or value < 0:
                    invalid_fields.append(field)
        if invalid_fields:
            blockers.append(f"runtime smoke evidence missing/invalid fields: {', '.join(invalid_fields)}")
        elif runtime_payload.get("status") not in {"ok", "pass", "passed"}:
            blockers.append(f"runtime smoke evidence status is not passing: {runtime_payload.get('status')!r}")
        else:
            runtime_evidence["valid"] = True
            runtime_evidence["status"] = runtime_payload.get("status")
            runtime_evidence["counts"] = {k: runtime_payload.get(k) for k in required_runtime_fields if k != "status"}

    required_positive_counts = {
        "viewport_received_count": _runtime_counter(runtime_payload, "viewport_received_count"),
        "render_cache_count": _runtime_counter(runtime_payload, "render_cache_count"),
        "visible_after_default_filters": int(visible_after_default_filters),
        "rendered_count": _runtime_counter(runtime_payload, "rendered_count"),
        "selectable_count": _runtime_counter(runtime_payload, "selectable_count"),
        "hierarchy_rows_count": _runtime_counter(runtime_payload, "hierarchy_rows_count"),
        "mesh_rendered_count": _runtime_counter(runtime_payload, "mesh_rendered_count"),
        "primitive_fallback_count": int(primitive_fallback_count),
        "locked_generated_urdf_visual_count": int(locked_generated_urdf_visual_count),
    }
    for key, value in required_positive_counts.items():
        if key in {"primitive_fallback_count", "locked_generated_urdf_visual_count"}:
            continue
        if value <= 0 and visible_after_default_filters > 0:
            blockers.append(f"acceptance gate failed: {key} must be > 0 (got {value})")
    if visible_after_default_filters > 0 and all(
        required_positive_counts[k] <= 0 for k in ("viewport_received_count", "render_cache_count", "rendered_count", "selectable_count", "hierarchy_rows_count")
    ):
        blockers.append("visible candidates exist but runtime counters are all zero")

    secondary_checks = {
        "grid_axes_enabled": ("draw_ground_grid_pass" in viewport_path.read_text(encoding="utf-8") and "draw_world_axes_pass" in viewport_path.read_text(encoding="utf-8")),
        "orbit_pan_zoom_handlers_exist": all(t in viewport_path.read_text(encoding="utf-8") for t in ["mouseMoveEvent", "wheelEvent", "pan_mode"]),
        "selection_callback_wired": "select_cb" in preview_path.read_text(encoding="utf-8"),
        "inspector_callback_wired": (
            "update_scene_builder_inspector_for_selected_item" in main_path.read_text(encoding="utf-8")
            or "apply_scene_selection(" in main_path.read_text(encoding="utf-8")
        ),
        "drag_commit_callback_wired": "transform_changed_cb" in viewport_path.read_text(encoding="utf-8"),
        "hierarchy_selection_sync_wired": "apply_scene_selection(" in main_path.read_text(encoding="utf-8"),
        "layer_toggles_not_default_hide_all": (
            "apply_scene3d_preview_layer_filters" in main_path.read_text(encoding="utf-8")
            and "visible item count after filters" in main_path.read_text(encoding="utf-8")
        ),
        "viewport_diagnostics_summary_present": "Scene3D runtime render: received=" in viewport_path.read_text(encoding="utf-8"),
    }

    runtime_counts = runtime_payload if isinstance(runtime_payload, dict) else {}
    return {
        "scene": scene,
        "scene_path": str(sdir),
        "blockers": blockers,
        "counts": {
            "editable_layout_count": editable_layout_count,
            "primitive_fallback_count": primitive_fallback_count,
            "mesh_preview_count": mesh_preview_count,
            "locked_generated_urdf_visual_count": locked_generated_urdf_visual_count,
            "overlay_count": overlay_count,
            "missing_count": missing_count,
            "viewport_received_count": _runtime_counter(runtime_counts, "viewport_received_count"),
            "render_cache_count": _runtime_counter(runtime_counts, "render_cache_count"),
            "visible_after_default_filters": visible_after_default_filters,
            "rendered_count": _runtime_counter(runtime_counts, "rendered_count"),
            "selectable_count": _runtime_counter(runtime_counts, "selectable_count"),
            "hierarchy_rows_count": _runtime_counter(runtime_counts, "hierarchy_rows_count"),
            "mesh_rendered_count": _runtime_counter(runtime_counts, "mesh_rendered_count"),
        },
        "visibility_contract": {
            "input_items_count": input_items_count,
            "visible_after_default_filters": visible_after_default_filters,
            "hidden_by_filters_count": hidden_by_filters_count,
        },
        "layers": {
            "editable_layout_visible": editable_layout_count,
            "mesh_preview_visible_if_safe": mesh_preview_count,
            "locked_generated_urdf_visual_hidden_default": locked_generated_urdf_visual_count,
            "overlay_items": overlay_count,
        },
        "sources": {
            "layout": str(layout_path),
            "mesh_index": str(mesh_index_path),
            "preview_metadata": str(preview_metadata_path) if preview_metadata_path.exists() else None,
            "generated_layout": str(urdf_generated_layout_path) if urdf_generated_layout_path.exists() else None,
            "overlay_files": [str(p) for p in overlay_sources if p.exists()],
        },
        "source_layer_counts": source_layer_counts,
        "runtime_evidence": runtime_evidence,
        "secondary_checks": secondary_checks,
        "status": "PASS" if not blockers else "FAIL",
        "pass": not blockers,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", action="append", default=[])
    ap.add_argument("--all-scenes", action="store_true", help="evaluate all discovered scenes")
    ap.add_argument("--repo-root", default=str(Path(__file__).resolve().parents[1]))
    ap.add_argument("--workspace-root", default=None)
    ap.add_argument("--workcell-builder-executable", "--executable", default=None)
    ap.add_argument("--output-dir", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--markdown", default=None)
    ap.add_argument("--smoke-json", default=None, help="path to GUI smoke evidence JSON (workcell_studio_scene3d_gui_smoke/v1)")
    ap.add_argument("--smoke-dir", default=None, help="directory containing per-scene smoke JSONs named scene3d_gui_smoke_<scene>.json")
    args = ap.parse_args()

    repo_root = Path(args.repo_root).resolve()
    scenes_root = resolve_scene_root(repo_root)
    main_path = repo_root / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
    preview_path = repo_root / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp"
    viewport_path = repo_root / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp"

    if not scenes_root.exists():
        ap.error(f"scenes root does not exist: {scenes_root}")

    discovered = discover_scene3d_scenes(scenes_root)
    discovered_map = {d["scene"]: d for d in discovered}
    if args.all_scenes and args.smoke_json:
        ap.error("--smoke-json is only valid for single-scene mode; use --smoke-dir with --all-scenes")
    if args.all_scenes:
        scenes = [d["scene"] for d in discovered if d["status"] != "IGNORED_NON_SCENE"]
    else:
        scenes = args.scene or ["ur5_2f_test"]

    unique_scenes: list[str] = []
    for s in scenes:
        if s not in unique_scenes:
            unique_scenes.append(s)

    results = []
    skipped_reason_histogram: dict[str, int] = {}
    for s in unique_scenes:
        discovery = discovered_map.get(s)
        if discovery and discovery["status"] in {"BLOCKED", "LEGACY_INCOMPLETE"}:
            reasons = discovery.get("blockers") or [f"scene discovery status: {discovery['status']}"]
            for reason in reasons:
                skipped_reason_histogram[reason] = skipped_reason_histogram.get(reason, 0) + 1
            results.append(
                {
                    "scene": s,
                    "scene_path": discovery["scene_path"],
                    "detected_files": discovery["detected_files"],
                    "source_layers_found": discovery["source_layers_found"],
                    "status": discovery["status"],
                    "blockers": reasons,
                    "counts": {},
                    "visibility_contract": {},
                    "layers": {},
                    "sources": {},
                    "runtime_evidence": {"valid": False, "skipped": True},
                    "secondary_checks": {},
                    "pass": False,
                }
            )
            continue
        smoke_json_path = args.smoke_json
        if args.all_scenes and args.smoke_dir:
            smoke_json_path = str(Path(args.smoke_dir) / f"scene3d_gui_smoke_{s}.json")
        evaluated = evaluate_scene(repo_root, scenes_root, main_path, preview_path, viewport_path, s, smoke_json_path)
        if discovery:
            evaluated["detected_files"] = discovery["detected_files"]
            evaluated["source_layers_found"] = discovery["source_layers_found"]
        results.append(evaluated)
    blockers = [f"{r['scene']}: {b}" for r in results for b in r.get("blockers", [])]

    payload = {
        "schema": SCHEMA,
        "repo_root": str(repo_root),
        "scenes_root": str(scenes_root),
        "discovered_scenes": discovered,
        "scenes": results,
        "skipped_reason_histogram": skipped_reason_histogram,
        "blockers": blockers,
        "pass": not blockers,
    }

    out_json = Path(args.json) if args.json else repo_root / "build/workcell_studio/scene3d_runtime_acceptance.json"
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    lines = ["# Scene3D Runtime Acceptance", f"- schema: `{SCHEMA}`", f"- overall_pass: `{payload['pass']}`", ""]
    lines.append("## Blockers")
    if blockers:
        for b in blockers:
            lines.append(f"- FAIL: {b}")
    else:
        lines.append("- none")
    lines.append("")
    lines.append("## Skipped reason histogram")
    if skipped_reason_histogram:
        for reason, count in sorted(skipped_reason_histogram.items()):
            lines.append(f"- {reason}: {count}")
    else:
        lines.append("- none")

    for r in results:
        lines.append("")
        lines.append(f"## {r['scene']}")
        lines.append(f"### Status: {r.get('status', 'FAIL')}")
        if r.get("detected_files") is not None:
            lines.append("### Detected files")
            for k, v in r["detected_files"].items():
                lines.append(f"- {k}: {v}")
        if r.get("source_layers_found") is not None:
            lines.append(f"### Source layers found: {r['source_layers_found']}")
        lines.append("### Counts")
        for k, v in r["counts"].items():
            lines.append(f"- {k}: {v}")
        lines.append("### Visibility contract")
        for k, v in r["visibility_contract"].items():
            lines.append(f"- {k}: {v}")
        lines.append("### Layer summary")
        for k, v in r["layers"].items():
            lines.append(f"- {k}: {v}")
        lines.append("### Source summary")
        for k, v in r["sources"].items():
            lines.append(f"- {k}: {v}")
        lines.append("### Runtime smoke evidence")
        for k, v in r["runtime_evidence"].items():
            lines.append(f"- {k}: {v}")
        lines.append("### Secondary diagnostics")
        for k, v in r["secondary_checks"].items():
            lines.append(f"- {k}: {'PASS' if v else 'FAIL'}")

    out_md = Path(args.markdown) if args.markdown else repo_root / "build/workcell_studio/scene3d_runtime_acceptance.md"
    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"wrote {out_json}")
    print(f"wrote {out_md}")
    return 0 if payload["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
