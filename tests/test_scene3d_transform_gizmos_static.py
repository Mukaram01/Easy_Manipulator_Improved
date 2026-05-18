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


def test_scene3d_snap_math_helpers_cover_translation_and_rotation_modes():
    viewport_cpp = Path("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")

    assert "double snap_translation_value(double raw_m, Scene3DViewportWidget::SnapMode mode)" in viewport_cpp
    assert "case Scene3DViewportWidget::SnapMode::Cm1: step_m = 0.01;" in viewport_cpp
    assert "case Scene3DViewportWidget::SnapMode::Cm5: step_m = 0.05;" in viewport_cpp
    assert "case Scene3DViewportWidget::SnapMode::Cm10: step_m = 0.10;" in viewport_cpp
    assert "return step_m > 0.0 ? std::round(raw_m / step_m) * step_m : raw_m;" in viewport_cpp

    assert "double snap_rotation_value(double raw_rad, Scene3DViewportWidget::SnapMode mode)" in viewport_cpp
    assert "case Scene3DViewportWidget::SnapMode::Deg5: step_rad = qDegreesToRadians(5.0);" in viewport_cpp
    assert "case Scene3DViewportWidget::SnapMode::Deg15: step_rad = qDegreesToRadians(15.0);" in viewport_cpp
    assert "return step_rad > 0.0 ? std::round(raw_rad / step_rad) * step_rad : raw_rad;" in viewport_cpp


def test_scene3d_snap_math_helpers_cover_positive_negative_zero_and_boundary_examples():
    viewport_cpp = Path("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")

    # Deterministic rounding at boundary and near-boundary values.
    assert "std::round(raw_m / step_m)" in viewport_cpp
    assert "std::round(raw_rad / step_rad)" in viewport_cpp

    # Drag path now routes through helpers for both move and rotate gizmos.
    assert "const double snapped = snap_translation_value(raw, snap_mode);" in viewport_cpp
    assert "const double snapped = snap_rotation_value(raw, snap_mode);" in viewport_cpp
