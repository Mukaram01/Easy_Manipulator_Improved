from pathlib import Path


def test_scene_preview_widget_uses_qt5_compatible_qpolygonf_construction():
    src = Path("workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")
    assert "QPolygonF{" not in src, "Qt5 build regression: initializer-list QPolygonF{} is not supported"
    assert "QPolygonF poly" in src, "Expected explicit Qt5-compatible QPolygonF variable"


def test_mainwindow_has_no_invalid_item_based_preview_selection():
    src = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    assert "select_preview_item(item->" not in src, "Build regression: undefined item used for preview selection"
