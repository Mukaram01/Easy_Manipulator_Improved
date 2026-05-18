from pathlib import Path


def test_scene3d_transform_gizmo_tokens_exist():
    viewport_h = Path("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.h").read_text(encoding="utf-8")
    preview_cpp = Path("workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")
    main_cpp = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")

    assert "enum class GizmoMode" in viewport_h
    assert "enum class SnapMode" in viewport_h
    assert "transform_changed_cb" in viewport_h
    assert '"Move"' in preview_cpp and '"Rotate"' in preview_cpp
    assert "Scene3D Gizmo Transform" in main_cpp
