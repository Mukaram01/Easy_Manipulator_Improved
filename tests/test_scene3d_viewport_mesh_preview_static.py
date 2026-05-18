from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp"


def _draw_mesh_preview_fn_body() -> str:
    src = CPP.read_text(encoding="utf-8")
    match = re.search(
        r"bool\s+Scene3DViewportWidget::draw_mesh_preview_if_available\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}",
        src,
        re.DOTALL,
    )
    assert match, "draw_mesh_preview_if_available definition not found"
    return match.group("body")


def test_draw_mesh_preview_uses_mesh_cache_helper():
    body = _draw_mesh_preview_fn_body()
    assert "ensure_mesh_cached(" in body


def test_draw_mesh_preview_has_explicit_mesh_preview_mode_branches():
    body = _draw_mesh_preview_fn_body()
    for mode in ["Primitives", "Meshes", "Auto"]:
        token = f"MeshPreviewMode::{mode}"
        assert token in body, f"expected explicit branch token in draw path: {token}"


def test_draw_mesh_preview_draws_cached_triangles_not_just_unit_cube_fallback():
    body = _draw_mesh_preview_fn_body()
    assert "for (const auto & tri :" in body
    assert ".mesh.triangles" in body

    fallback_pos = body.find("draw_unit_cube_triangles(")
    triangles_loop_pos = body.find("for (const auto & tri :")
    assert fallback_pos != -1, "expected primitive fallback call for invalid mesh path"
    assert triangles_loop_pos != -1, "expected explicit triangle iteration draw path"
    assert triangles_loop_pos > fallback_pos, "expected non-fallback mesh draw path after fallback guard"
