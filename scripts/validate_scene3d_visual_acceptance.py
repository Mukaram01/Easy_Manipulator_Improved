#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path


def _get(d: dict, *keys, default=0):
    for k in keys:
        if isinstance(d, dict) and k in d:
            return d[k]
    return default


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke-json", required=True, type=Path)
    ap.add_argument("--screenshot", required=True, type=Path)
    args = ap.parse_args()

    smoke = json.loads(args.smoke_json.read_text(encoding="utf-8"))
    counters = smoke.get("render_debug_counters", {}) if isinstance(smoke, dict) else {}

    blockers: list[str] = []
    status = str(_get(smoke, "status", default="")).upper()
    if status and status == "FAIL":
      blockers.append("scene3d smoke reported FAIL")

    if str(_get(smoke, "schema", default="")) != "workcell_studio_scene3d_gui_smoke/v1":
      blockers.append("unexpected smoke schema")

    if not bool(_get(smoke, "scene3d_rendered", "last_paint_completed", default=counters.get("last_paint_completed", False))):
      blockers.append("scene3d not rendered")

    mesh_rendered = int(_get(smoke, "mesh_rendered_count", default=counters.get("mesh_rendered_count", 0)))
    if mesh_rendered <= 0:
      blockers.append("mesh_rendered_count <= 0")

    generated_count = int(_get(smoke, "generated_urdf_visual_count", "locked_generated_urdf_visual_count", default=counters.get("locked_generated_urdf_visual_count", 0)))
    if generated_count <= 0:
      blockers.append("generated URDF visual count <= 0")

    overlay_count = int(_get(smoke, "overlay_count", default=counters.get("overlay_count", 0)))
    visible_count = max(1, int(_get(smoke, "visible_count", default=counters.get("visible_count", 0))))
    if overlay_count > int(visible_count * 0.7):
      blockers.append("helper overlays dominate visible set")

    placeholder_count = int(_get(smoke, "placeholder_count", "primitive_fallback_count", default=counters.get("placeholder_count", 0)))
    rendered_count = max(1, int(_get(smoke, "rendered_count", default=counters.get("rendered_count", 0))))
    if placeholder_count > int(rendered_count * 0.5):
      blockers.append("fallback placeholders dominate rendered solids")

    viewport_source = str(_get(smoke, "active_viewport_source", "viewport_source", default="")).lower()
    if viewport_source and ("scene3d" not in viewport_source or "visible" not in viewport_source):
      blockers.append("active viewport source is not the visible Scene3D viewport")

    if not args.screenshot.exists() or args.screenshot.stat().st_size <= 0:
      blockers.append("screenshot missing or empty")

    out = {
      "status": "PASS" if not blockers else "FAIL",
      "blockers": blockers,
      "smoke_json": str(args.smoke_json),
      "screenshot": str(args.screenshot),
      "mesh_rendered_count": mesh_rendered,
      "generated_urdf_visual_count": generated_count,
      "overlay_count": overlay_count,
      "placeholder_count": placeholder_count,
    }
    print(json.dumps(out, indent=2))
    return 0 if not blockers else 1


if __name__ == "__main__":
    raise SystemExit(main())
