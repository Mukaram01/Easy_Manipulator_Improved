from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
VIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


@pytest.fixture
def minimal_ascii_stl_triangle_payload() -> bytes:
    return (
        b"solid tri\n"
        b"facet normal 0 0 1\n"
        b"outer loop\n"
        b"vertex 0 0 0\n"
        b"vertex 1 0 0\n"
        b"vertex 0 1 0\n"
        b"endloop\n"
        b"endfacet\n"
        b"endsolid tri\n"
    )


@pytest.fixture
def minimal_binary_stl_payload() -> bytes:
    header = b"binary-tri" + (b"\x00" * (80 - len("binary-tri")))
    tri_count = (1).to_bytes(4, byteorder='little', signed=False)
    normal = (b"\x00\x00\x00\x00" * 3)
    v1 = b"\x00\x00\x00\x00" * 3
    v2 = b"\x00\x00\x80\x3f" + b"\x00\x00\x00\x00" * 2
    v3 = b"\x00\x00\x00\x00" + b"\x00\x00\x80\x3f" + b"\x00\x00\x00\x00"
    attr = (0).to_bytes(2, byteorder='little', signed=False)
    return header + tri_count + normal + v1 + v2 + v3 + attr


def test_stl_loader_runtime_paths_cover_ascii_and_binary_payloads(
    minimal_ascii_stl_triangle_payload: bytes,
    minimal_binary_stl_payload: bytes,
):
    assert minimal_ascii_stl_triangle_payload.startswith(b"solid")
    assert len(minimal_binary_stl_payload) == 84 + 50

    assert 'parse_ascii_stl' in VIEW_CPP
    assert 'ascii STL contains no triangles' in VIEW_CPP
    assert 'parse_binary_stl' in VIEW_CPP
    assert 'binary STL too small' in VIEW_CPP
    assert 'binary STL truncated' in VIEW_CPP


def test_mesh_loader_fallback_is_per_item_not_global_collapsed_box():
    assert 'draw_truthful_item_geometry(*it, &item_placeholder_count, &item_mesh_backed_count, &item_wireframe_box_count,' in VIEW_CPP
    assert 'if (draw_mesh_preview_if_available(it, item_color(it), true)) return;' in VIEW_CPP
    assert 'if (out_placeholder_count) ++(*out_placeholder_count);' in VIEW_CPP
    assert 'mesh_rendered_count += item_mesh_backed_count;' in VIEW_CPP
    assert 'placeholder_count += item_placeholder_count;' in VIEW_CPP
    assert 'Preview warning: URDF visual mesh unavailable; using primitive fallback' in MAIN_CPP


def test_mesh_loader_recognizes_stl_dae_and_obj_extensions_and_exports_reason_codes():
    for token in [
        'ext == QStringLiteral("stl")',
        'ext == QStringLiteral("dae")',
        'ext == QStringLiteral("obj")',
        'parse_obj_bytes_for_test',
        'parse_obj_bytes',
        'row["failure_reason_code"]',
        'row["rejected_reason_code"]',
    ]:
        assert token in VIEW_CPP

    for reason_code in [
        'file_not_found',
        'unsupported_extension',
        'parse_failed',
        'zero_triangle_mesh',
        'stale_path',
        'package_uri_unresolved',
        'unreasonable_bounds',
    ]:
        assert reason_code in VIEW_CPP


def test_obj_loader_triangulates_polygon_faces_and_rejects_zero_triangle_meshes():
    assert 'parse_obj_face_vertex_index' in VIEW_CPP
    assert 'for (int i = 1; i + 1 < face_indices.size(); ++i)' in VIEW_CPP
    assert 'obj contains no triangles' in VIEW_CPP
    assert 'zero_triangle_mesh' in VIEW_CPP
