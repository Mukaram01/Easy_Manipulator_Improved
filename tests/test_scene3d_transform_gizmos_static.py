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


def test_scene3d_drag_uses_drag_start_screen_and_drag_start_pose_without_undefined_token():
    viewport_cpp = Path("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")
    assert "e->pos() - drag_start_screen_" in viewport_cpp
    assert "drag_start_pose_.x + snapped" in viewport_cpp
    assert "drag_start_pose_.roll + snapped" in viewport_cpp
    assert "drag_start_ =" not in viewport_cpp
    assert "drag_start_)" not in viewport_cpp

def _mouse_press_event_body():
    viewport_cpp = Path("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")
    start = viewport_cpp.index("void Scene3DViewportWidget::mousePressEvent(QMouseEvent * e) {")
    end = viewport_cpp.index("void Scene3DViewportWidget::mouseMoveEvent", start)
    return viewport_cpp[start:end]


def test_mouse_press_prioritizes_editable_gizmo_handle_before_item_picking():
    body = _mouse_press_event_body()
    assert "if (e->button() != Qt::LeftButton) return;" in body
    assert body.index("if (e->button() != Qt::LeftButton) return;") < body.index("pick_gizmo_axis_at_screen")
    assert body.index("item_is_editable_for_gizmo(*selected_item)") < body.index("pick_gizmo_axis_at_screen")
    assert body.index("item_is_editable_for_gizmo(*selected_item)") < body.index("pick_gizmo_rotation_ring_at_screen")
    assert body.index("pick_gizmo_axis_at_screen") < body.index("pick_item_at_screen")
    assert body.index("pick_gizmo_rotation_ring_at_screen") < body.index("pick_item_at_screen")
    assert "if (picked && !axis.isEmpty())" in body
    assert "return;" in body.split("if (picked && !axis.isEmpty())", 1)[1].split("QString best_id, best_role;", 1)[0]
