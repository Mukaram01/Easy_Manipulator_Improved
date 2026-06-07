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


def test_draw_mesh_preview_draws_cached_triangles_without_counting_invalid_mesh_as_rendered():
    body = _draw_mesh_preview_fn_body()
    assert "for (const auto & tri :" in body
    assert ".mesh.triangles" in body

    invalid_guard_pos = body.find("if (!cache.valid || cache.mesh.triangles.isEmpty())")
    triangles_loop_pos = body.find("for (const auto & tri :")
    assert invalid_guard_pos != -1, "expected invalid mesh guard before triangle rendering"
    assert "draw_unit_cube_triangles(color);" not in body, "invalid meshes must not be counted as successful mesh rendering"
    assert "return false;" in body[invalid_guard_pos:triangles_loop_pos], "invalid meshes should fall through to primitive fallback accounting"
    assert triangles_loop_pos > invalid_guard_pos, "expected non-fallback mesh draw path after invalid mesh guard"


def test_generated_bounds_suppression_and_mesh_reject_codes_are_specific():
    src = CPP.read_text(encoding="utf-8")
    assert "return !item_has_mesh_uri_or_path(it) && !item_has_valid_urdf_primitive(it);" in src
    assert "REJECT_RAW_GENERATED_BOUNDS_SUPPRESSED: generated bounds item has no mesh URI/path and no URDF primitive" in src
    assert "REJECT_MESH_PARSE_FAILED" in src
    assert "REJECT_MESH_BOUNDS_FAILED" in src
    assert "semantic_mesh_fallback" in src
    assert "if (out_primitive_fallback_count) ++(*out_primitive_fallback_count);" in src
