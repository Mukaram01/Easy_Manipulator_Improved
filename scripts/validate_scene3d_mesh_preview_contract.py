#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp"
MAINWINDOW = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"

def must(cond, msg):
    if not cond:
        raise AssertionError(msg)

def fn_body(src: str, signature_prefix: str) -> str:
    m = re.search(rf"{re.escape(signature_prefix)}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}", src, re.DOTALL)
    must(m is not None, f"function not found: {signature_prefix}")
    return m.group("body")

def main():
    src = CPP.read_text(encoding="utf-8")
    body = fn_body(src, "bool Scene3DViewportWidget::draw_mesh_preview_if_available")
    for token in [
        "MeshPreviewMode::Primitives",
        "MeshPreviewMode::Meshes",
        "MeshPreviewMode::Auto",
        "ensure_mesh_cached(",
        "for (const auto & tri : cache.mesh.triangles)",
        "glBegin(GL_TRIANGLES)",
        "compute_mesh_bounds_for_test",
    ]:
        must(token in src if token == "compute_mesh_bounds_for_test" else token in body, f"missing token: {token}")

    loop_pos = body.find("for (const auto & tri : cache.mesh.triangles)")
    cube_pos = body.find("draw_unit_cube_triangles(")
    must(loop_pos != -1, "triangle loop missing")
    must(cube_pos == -1 or cube_pos < loop_pos, "unit cube fallback must not be used in successful mesh path")

    inv_body = fn_body(src, "void Scene3DViewportWidget::invalidate_mesh_cache")
    must("mesh_cache_.clear();" in inv_body, "reload does not clear mesh_cache_")
    must("warned_mesh_fallbacks_.clear();" in inv_body, "reload does not clear warning-once set")

    mw_src = MAINWINDOW.read_text(encoding="utf-8")
    for token in ["scene_visual_mesh_index.json", "urdf_visual", "Preview warning: URDF visual unresolved", "p.locked = true"]:
        must(token in mw_src, f"missing mainwindow token: {token}")
    must("use_fake_hardware:=true" in mw_src, "fake hardware launch token missing")

    print("scene3d mesh preview contract: OK")

if __name__ == "__main__":
    main()
