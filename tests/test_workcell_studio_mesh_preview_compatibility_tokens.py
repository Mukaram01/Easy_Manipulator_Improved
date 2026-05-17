from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_canvas_and_preview_item_mesh_fields_have_legacy_safe_defaults():
    canvas_h = (ROOT / "workcell_builder/workcell_builder/include/workcell_studio_canvas_model.hpp").read_text(encoding="utf-8")
    preview_h = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.h").read_text(encoding="utf-8")

    for token in [
        "std::string mesh_path;",
        "std::string mesh_type;",
        "double mesh_scale_x{1.0}, mesh_scale_y{1.0}, mesh_scale_z{1.0};",
        "double mesh_r{0.0}, mesh_p{0.0}, mesh_y{0.0};",
        "bool mesh_available{false};",
        "std::string mesh_load_warning;",
    ]:
        assert token in canvas_h

    for token in [
        "QString mesh_path;",
        "QString mesh_type;",
        "double mesh_scale_x{ 1.0 }, mesh_scale_y{ 1.0 }, mesh_scale_z{ 1.0 };",
        "double mesh_roll{ 0.0 }, mesh_pitch{ 0.0 }, mesh_yaw{ 0.0 };",
        "bool mesh_available{ false };",
        "QString mesh_load_warning;",
    ]:
        assert token in preview_h


def test_mainwindow_maps_mesh_preview_fields_and_legacy_primitive_fallback_path_remains():
    mainwindow_cpp = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    preview_cpp = (ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")

    for token in [
        "p.mesh_path = QString::fromStdString(item.mesh_path);",
        "p.mesh_type = QString::fromStdString(item.mesh_type);",
        "p.mesh_scale_x = item.mesh_scale_x;",
        "p.mesh_scale_y = item.mesh_scale_y;",
        "p.mesh_scale_z = item.mesh_scale_z;",
        "p.mesh_roll = item.mesh_r;",
        "p.mesh_pitch = item.mesh_p;",
        "p.mesh_yaw = item.mesh_y;",
        "p.mesh_available = item.mesh_available;",
        "p.mesh_load_warning = QString::fromStdString(item.mesh_load_warning);",
    ]:
        assert token in mainwindow_cpp

    for token in ["draw_box(", "draw_table_slab", "draw_object_cube", "draw_conveyor"]:
        assert token in preview_cpp
