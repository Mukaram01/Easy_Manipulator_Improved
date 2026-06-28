from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEWPORT = (ROOT / "workcell_builder" / "workcell_builder" / "gui" / "scene3d_viewport_widget.cpp").read_text(encoding="utf-8")
PREVIEW = (ROOT / "workcell_builder" / "workcell_builder" / "gui" / "scene_preview_widget.cpp").read_text(encoding="utf-8")
MAINWINDOW = (ROOT / "workcell_builder" / "workcell_builder" / "gui" / "mainwindow.cpp").read_text(encoding="utf-8")


def test_product_fit_frames_physical_workcell_and_editor_anchors() -> None:
    assert "product_physical_initial_fit_robot_included" in VIEWPORT
    assert "NormalizedRole::PickZone" in VIEWPORT
    assert "NormalizedRole::PlaceZone" in VIEWPORT
    assert "NormalizedRole::SafetyZone" in VIEWPORT
    assert "NormalizedRole::HomePose" in VIEWPORT
    assert "* 1.22" not in VIEWPORT
    assert "const double base_fit_distance = product_radius / qTan(fov * 0.5);" in VIEWPORT
    assert "qMax(qMax(base_fit_distance * 2.4, product_radius * 5.0), 4.0)" in VIEWPORT
    assert "yaw_ = -0.86" in VIEWPORT
    assert "pitch_ = -0.60" in VIEWPORT
    assert "viewport->fit_product_view();" in MAINWINDOW


def test_product_view_defaults_show_safety_and_use_subtle_floor_grid() -> None:
    assert "v->show_safety = true;" in PREVIEW
    assert "const int grid_extent = debug_overlays_mode ? 20 : 6;" in VIEWPORT
    assert "QColor(15, 23, 42, debug_overlays_mode ? 38 : 24)" in VIEWPORT


def test_semantic_primitives_use_product_materials_not_warning_boxes() -> None:
    assert "QColor(34, 197, 94, 46)" in VIEWPORT
    assert "QColor(168, 85, 247, 44)" in VIEWPORT
    assert "QColor(251, 191, 36, 170)" in VIEWPORT
    assert "const QColor body(17, 24, 39, 225);" in VIEWPORT
    assert "const QColor tabletop_fill(126, 118, 105, 228);" in VIEWPORT
