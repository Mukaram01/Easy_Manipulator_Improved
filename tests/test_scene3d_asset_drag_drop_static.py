from pathlib import Path


def test_scene3d_asset_drag_drop_tokens_exist():
    main_cpp = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    viewport_h = Path("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.h").read_text(encoding="utf-8")
    viewport_cpp = Path("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")

    assert "application/x-workcell-asset-catalog-item" in main_cpp
    assert "QDrag" in main_cpp and "QMimeData" in main_cpp
    assert "disabled_reason" in main_cpp and "Cannot place here" in main_cpp
    assert "asset_drop_cb" in viewport_h
    assert "dragEnterEvent" in viewport_h and "dropEvent" in viewport_h
    assert "Drop to place" in viewport_cpp
    assert "drag_asset_preview_visible_" in viewport_h
    assert "workcell_studio_next_id" in main_cpp
    assert "mark_layout_dirty" in main_cpp
    assert "parse_collada_bytes_for_test" in viewport_cpp
    assert "ext == QStringLiteral(\"dae\")" in viewport_cpp
    assert "unsupported mesh format" in viewport_cpp
    assert "Locked URDF" in viewport_cpp
    assert "Overlays %1" in viewport_cpp
    assert "Items %1 • Mesh %2 • Boxes %3 • Missing %4" in viewport_cpp
