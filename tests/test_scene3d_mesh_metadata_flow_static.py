from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CANVAS_CPP = ROOT / "workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp"
MAINWINDOW_CPP = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"


def test_layout_mesh_metadata_is_read_into_canvas_model():
    src = CANVAS_CPP.read_text(encoding="utf-8")
    for token in [
        "item.has_mesh_metadata = true;",
        "item.mesh_scale_x =",
        "item.mesh_scale_y =",
        "item.mesh_scale_z =",
        "item.mesh_r =",
        "item.mesh_p =",
        "item.mesh_y =",
        "item.has_origin_offset = true;",
        "item.origin_offset_x =",
        "item.origin_offset_y =",
        "item.origin_offset_z =",
    ]:
        assert token in src


def test_mainwindow_maps_mesh_metadata_to_preview_item():
    src = MAINWINDOW_CPP.read_text(encoding="utf-8")
    for token in [
        "p.has_mesh_metadata = item.has_mesh_metadata;",
        "p.mesh_r = item.mesh_r;",
        "p.mesh_p = item.mesh_p;",
        "p.mesh_y = item.mesh_y;",
        "p.has_origin_offset = item.has_origin_offset;",
        "p.origin_offset_x = item.origin_offset_x;",
        "p.origin_offset_y = item.origin_offset_y;",
        "p.origin_offset_z = item.origin_offset_z;",
    ]:
        assert token in src
