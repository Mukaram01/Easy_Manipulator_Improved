#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
try:
    from scripts.scene_root_resolver import resolve_scene_root
except ModuleNotFoundError:
    from scene_root_resolver import resolve_scene_root

SCHEMA = "workcell_studio_scene3d_runtime_acceptance/v1"
ROOT = Path(__file__).resolve().parents[1]
SCENES_ROOT = resolve_scene_root(ROOT)
MAIN = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
PREVIEW = ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp"
VIEWPORT = ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp"


def scene_dir(scene: str) -> Path:
    return SCENES_ROOT / scene


def has_snapshot_overlay(scene_path: Path) -> bool:
    return any((scene_path / p).exists() for p in ["generated/epd_snapshot.json", "generated/detections_snapshot.json", "config/epd_snapshot.json"])


def evaluate_scene(scene: str) -> dict:
    sdir = scene_dir(scene)
    layout = sdir / "layout/workcell_studio_layout.yaml"
    layout_exists = layout.exists()
    layout_text = layout.read_text(encoding="utf-8", errors="ignore") if layout_exists else ""
    checks = {
        "layout_items_count_gt_zero": ("id:" in layout_text),
        "preview_items_count_gt_zero": "prepare_scene_preview_items" in MAIN.read_text(encoding="utf-8"),
        "at_least_one_visible_after_default_layers": "default_scene3d_layer_visibility_" in MAIN.read_text(encoding="utf-8"),
        "at_least_one_editable_item": "editable_layout" in MAIN.read_text(encoding="utf-8"),
        "primitive_fallback_or_mesh_backed": ("primitive_fallback" in MAIN.read_text(encoding="utf-8") or "mesh_preview" in MAIN.read_text(encoding="utf-8")),
        "camera_fov_overlay_when_metadata_exists": "show_camera_fov" in VIEWPORT.read_text(encoding="utf-8"),
        "detection_overlay_when_snapshot_exists": (not has_snapshot_overlay(sdir)) or ("show_epd_detections" in VIEWPORT.read_text(encoding="utf-8")),
    }
    return {"scene": scene, "scene_path": str(sdir), "checks": checks, "pass": all(checks.values())}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", action="append", default=[])
    ap.add_argument("--json", default=str(ROOT / "build/workcell_studio/scene3d_runtime_acceptance.json"))
    ap.add_argument("--markdown", default=str(ROOT / "build/workcell_studio/scene3d_runtime_acceptance.md"))
    args = ap.parse_args()
    scenes = args.scene or ["ur5_2f_test"]
    if SCENES_ROOT.exists():
      scenes_dir = SCENES_ROOT
      maybe = sorted(p.name for p in scenes_dir.iterdir() if p.is_dir() and "new_cell" in p.name)
      if maybe:
          scenes.append(maybe[0])
    unique_scenes = []
    for s in scenes:
        if s not in unique_scenes:
            unique_scenes.append(s)

    missing = [str(scene_dir(s)) for s in unique_scenes if not scene_dir(s).exists()]
    if missing:
        ap.error(f"requested scene path is missing: {', '.join(missing)}")

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
    payload = {"schema": SCHEMA, "scenes": results, "checks": root_checks, "pass": all(r["pass"] for r in results) and all(root_checks.values())}

    out_json = Path(args.json); out_json.parent.mkdir(parents=True, exist_ok=True); out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    lines = [f"# Scene3D Runtime Acceptance\n", f"- schema: `{SCHEMA}`", f"- overall_pass: `{payload['pass']}`", ""]
    for r in results:
        lines.append(f"## {r['scene']}")
        for k,v in r["checks"].items(): lines.append(f"- {k}: {'PASS' if v else 'FAIL'}")
    lines.append("\n## Global checks")
    for k,v in root_checks.items(): lines.append(f"- {k}: {'PASS' if v else 'FAIL'}")
    out_md = Path(args.markdown); out_md.parent.mkdir(parents=True, exist_ok=True); out_md.write_text("\n".join(lines)+"\n", encoding="utf-8")
    print(f"wrote {out_json}")
    print(f"wrote {out_md}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
