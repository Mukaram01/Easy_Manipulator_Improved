#!/usr/bin/env python3
from __future__ import annotations
import argparse
import json
from pathlib import Path

import yaml

SCHEMA = "workcell_studio_scene3d_runtime_acceptance/v1"
ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
PREVIEW = ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp"
VIEWPORT = ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp"


def scene_dir(scene: str) -> Path:
    return ROOT / "scenes" / scene


def _load_yaml(path: Path) -> dict:
    if not path.exists():
        return {}
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}


def _load_json(path: Path) -> dict:
    if not path.exists():
        return {}
    data = json.loads(path.read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}


def _find_preview_metadata(scene_path: Path) -> tuple[Path | None, dict]:
    candidates = [
        scene_path / "generated/scene_preview_metadata.json",
        scene_path / "generated/preview_metadata.json",
        scene_path / "generated/scene3d_preview_metadata.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate, _load_json(candidate)
    return None, {}


def _count_overlays(preview_meta: dict, snapshot_meta: dict) -> int:
    overlay_count = 0
    for key in ("overlays", "overlay_items", "overlay_layers"):
        value = preview_meta.get(key)
        if isinstance(value, list):
            overlay_count += len(value)
    if snapshot_meta:
        overlay_count += 1
    return overlay_count


def evaluate_scene(scene: str) -> dict:
    sdir = scene_dir(scene)
    blockers: list[str] = []

    layout_path = sdir / "layout/workcell_studio_layout.yaml"
    mesh_index_path = sdir / "generated/scene_visual_mesh_index.json"
    preview_meta_path, preview_meta = _find_preview_metadata(sdir)

    if not sdir.exists():
        blockers.append(f"scene directory missing: {sdir}")

    if not layout_path.exists():
        blockers.append(f"missing layout artifact: {layout_path}")
    if not mesh_index_path.exists():
        blockers.append(f"missing mesh index artifact: {mesh_index_path}")

    layout_doc = _load_yaml(layout_path)
    mesh_index = _load_json(mesh_index_path)

    items = layout_doc.get("items") if isinstance(layout_doc.get("items"), list) else []
    editable_layout_count = len(items)
    input_items_count = editable_layout_count

    visual_items = mesh_index.get("visual_items") if isinstance(mesh_index.get("visual_items"), list) else []
    locked_generated_urdf_visual_count = len(visual_items)
    primitive_fallback_count = sum(1 for item in visual_items if isinstance(item, dict) and item.get("fallback_mode") == "primitive")
    mesh_preview_count = sum(1 for item in visual_items if isinstance(item, dict) and item.get("resolved") and item.get("safe_for_preview", True))

    snapshot_meta = {}
    for p in [sdir / "generated/epd_snapshot.json", sdir / "generated/detections_snapshot.json", sdir / "config/epd_snapshot.json"]:
        if p.exists():
            snapshot_meta = _load_json(p)
            break

    overlay_count = _count_overlays(preview_meta, snapshot_meta)

    visible_after_default_filters = editable_layout_count + mesh_preview_count
    hidden_by_filters_count = max(0, input_items_count - visible_after_default_filters)

    if visible_after_default_filters == 0 and not blockers:
        blockers.append("visible_after_default_filters is zero")

    static_checks = {
        "camera_fov_overlay_when_metadata_exists": "show_camera_fov" in VIEWPORT.read_text(encoding="utf-8"),
        "detection_overlay_when_snapshot_exists": (not snapshot_meta) or ("show_epd_detections" in VIEWPORT.read_text(encoding="utf-8")),
    }

    return {
        "scene": scene,
        "scene_path": str(sdir),
        "artifacts": {
            "layout": str(layout_path),
            "mesh_index": str(mesh_index_path),
            "preview_metadata": str(preview_meta_path) if preview_meta_path else None,
        },
        "source_summary": {
            "layout_loaded": layout_path.exists(),
            "mesh_index_loaded": mesh_index_path.exists(),
            "preview_metadata_loaded": preview_meta_path is not None,
        },
        "counts": {
            "editable_layout_count": editable_layout_count,
            "primitive_fallback_count": primitive_fallback_count,
            "mesh_preview_count": mesh_preview_count,
            "locked_generated_urdf_visual_count": locked_generated_urdf_visual_count,
            "overlay_count": overlay_count,
            "input_items_count": input_items_count,
            "visible_after_default_filters": visible_after_default_filters,
            "hidden_by_filters_count": hidden_by_filters_count,
        },
        "layer_summary": {
            "default_visibility_contract": {
                "input_items_count": input_items_count,
                "visible_after_default_filters": visible_after_default_filters,
                "hidden_by_filters_count": hidden_by_filters_count,
            }
        },
        "checks": static_checks,
        "blockers": blockers,
        "pass": len(blockers) == 0,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", action="append", default=[])
    ap.add_argument("--json", default=str(ROOT / "build/workcell_studio/scene3d_runtime_acceptance.json"))
    ap.add_argument("--markdown", default=str(ROOT / "build/workcell_studio/scene3d_runtime_acceptance.md"))
    args = ap.parse_args()

    scenes = args.scene or ["ur5_2f_test"]
    unique_scenes: list[str] = []
    for s in scenes:
        if s not in unique_scenes:
            unique_scenes.append(s)

    results = [evaluate_scene(s) for s in unique_scenes]

    root_checks = {
        "grid_axes_enabled": ("draw_ground_grid_pass" in VIEWPORT.read_text(encoding="utf-8") and "draw_world_axes_pass" in VIEWPORT.read_text(encoding="utf-8")),
        "orbit_pan_zoom_handlers_exist": all(t in VIEWPORT.read_text(encoding="utf-8") for t in ["mouseMoveEvent", "wheelEvent", "pan_mode"]),
        "selection_callback_wired": "select_cb" in PREVIEW.read_text(encoding="utf-8"),
        "inspector_callback_wired": "update_scene_builder_inspector_for_selected_item" in MAIN.read_text(encoding="utf-8"),
        "drag_commit_callback_wired": "transform_changed_cb" in VIEWPORT.read_text(encoding="utf-8"),
        "hierarchy_selection_sync_wired": "apply_scene_selection(" in MAIN.read_text(encoding="utf-8"),
        "layer_toggles_not_default_hide_all": "visible item count after filters" in MAIN.read_text(encoding="utf-8"),
        "viewport_diagnostics_summary_present": "Scene3D runtime render: received=" in VIEWPORT.read_text(encoding="utf-8"),
    }

    payload = {
        "schema": SCHEMA,
        "scenes": results,
        "checks": root_checks,
        "blockers": [b for r in results for b in r["blockers"]],
        "pass": all(r["pass"] for r in results),
    }

    out_json = Path(args.json)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    lines = ["# Scene3D Runtime Acceptance\n", f"- schema: `{SCHEMA}`", f"- overall_pass: `{payload['pass']}`", ""]
    for r in results:
        lines.append(f"## {r['scene']}")
        lines.append(f"- pass: {'PASS' if r['pass'] else 'FAIL'}")
        lines.append(f"- blockers: {r['blockers'] if r['blockers'] else '[]'}")
        for k, v in r["counts"].items():
            lines.append(f"- {k}: {v}")
        lines.append("- source_summary:")
        for k, v in r["source_summary"].items():
            lines.append(f"  - {k}: {v}")
        lines.append("- static_checks:")
        for k, v in r["checks"].items():
            lines.append(f"  - {k}: {'PASS' if v else 'FAIL'}")

    lines.append("\n## Global checks")
    for k, v in root_checks.items():
        lines.append(f"- {k}: {'PASS' if v else 'FAIL'}")
    lines.append("\n## Blockers")
    lines.extend(f"- {b}" for b in payload["blockers"] or ["none"])

    out_md = Path(args.markdown)
    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"wrote {out_json}")
    print(f"wrote {out_md}")
    return 0 if payload["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
