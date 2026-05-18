from pathlib import Path


def test_scene3d_transform_gizmo_tokens_exist():
    viewport_h = Path("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.h").read_text(encoding="utf-8")
    viewport_cpp = Path("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")
    editor_cpp = Path("workcell_builder/workcell_builder/gui/environment_layout_editor.cpp").read_text(encoding="utf-8")
    preview_cpp = Path("workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")
    main_cpp = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")

    assert "enum class GizmoMode" in viewport_h
    assert "enum class SnapMode" in viewport_h
    assert "transform_changed_cb" in viewport_h
    assert "pick_gizmo_axis_at_screen" in viewport_h
    assert "pick_gizmo_rotation_ring_at_screen" in viewport_h
    assert "active_gizmo_handle" in viewport_h
    assert ("snap_translation_value" in viewport_h) or ("Snap:" in preview_cpp)
    assert ("snap_rotation_value" in viewport_h) or ("Rotate" in preview_cpp)
    assert "Qt::Key_Escape" in viewport_cpp and ("restore" in viewport_cpp or "cancel" in viewport_cpp)
    assert "mouseReleaseEvent" in viewport_cpp and ("commit" in viewport_cpp or "transform_changed_cb" in viewport_cpp)
    assert "Locked:" in viewport_cpp or "Locked:" in editor_cpp
    assert "inspector" in main_cpp and ("refresh_selection_transform_editor_from_item" in main_cpp or "apply_inspector_pose_to_item" in main_cpp)
    assert "mark_layout_dirty" in main_cpp
    assert '"Move"' in preview_cpp and '"Rotate"' in preview_cpp
    assert "Scene3D Gizmo Transform" in main_cpp
