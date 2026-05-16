from pathlib import Path

CPP = Path("workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")
MAIN = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
UI = Path("workcell_builder/workcell_builder/gui/scene_select.ui").read_text(encoding="utf-8")
PREVIEW = Path("workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")
VIEW3D_CPP = Path("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")
VIEW3D_H = Path("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.h").read_text(encoding="utf-8")


def test_navigation_tokens_present():
    required = [
        "Fit Cell",
        "Fit Selection",
        "Reset View",
        "Zoom In",
        "Zoom Out",
        "Zoom 100%",
        "mouse wheel zoom",
        "right-drag pan",
    ]
    for token in required:
        assert token in CPP or token in UI


def test_no_long_warning_drawtext_path_and_warning_logic_present():
    assert "Show Warnings" in MAIN
    assert "Toggle Warnings" in MAIN
    assert "setToolTip" in MAIN


def test_label_mode_tristate_tokens_present():
    for token in [
        "ScenePreviewWidget::LabelMode::Off",
        "ScenePreviewWidget::LabelMode::SelectedOnly",
        "ScenePreviewWidget::LabelMode::All",
    ]:
        assert token in MAIN


def test_fit_scene_and_minimap_viewport_behavior_tokens_present():
    assert 'if (gi->data(RoleRole).toString() != "asset") continue;' in MAIN
    for token in [
        "minimap_view_->setVisible(false);",
        "const QRect viewport = digital_twin_canvas_->viewport()->rect();",
        "const QRectF visible_rect = digital_twin_canvas_->mapToScene(viewport).boundingRect();",
        "minimap_scene_->addRect(visible_rect",
    ]:
        assert token in MAIN


def test_real_3d_viewport_and_camera_depth_tokens_present():
    for token in ["class Scene3DViewportWidget : public QOpenGLWidget", "glEnable(GL_DEPTH_TEST)", "out_proj.perspective(50.0f", "set_isometric_view()"]:
        assert token in VIEW3D_H or token in VIEW3D_CPP


def test_orbit_pan_zoom_interaction_tokens_present():
    for token in [
        "mouseMoveEvent",
        "yaw_ += d.x() * 0.01",
        "pitch_ = qBound",
        "Qt::MiddleButton",
        "Qt::ShiftModifier",
        "orbit_offset_ +=",
        "wheelEvent",
        "distance_ = qBound(min_distance_, distance_ * zoom_factor, max_distance_);",
    ]:
        assert token in VIEW3D_CPP


def test_camera_fit_and_view_matrices_tokens_present():
    for token in [
        "out_view.lookAt(eye, orbit_offset_, QVector3D(0.0f, 1.0f, 0.0f));",
        "scene_radius_ = radius;",
        "orbit_offset_ = (bmin + bmax) * 0.5f;",
        "distance_ = qBound(min_distance_, fit_distance, max_distance_);",
    ]:
        assert token in VIEW3D_CPP


def test_fit_scene_excludes_overlay_only_items_tokens_present():
    for token in [
        "FIT_PHYSICAL_ONLY_FILTER",
        "FIT_EXCLUDE_OVERLAY_ONLY",
        "if (!include_in_fit_bounds_physical_only(it)) continue;",
        "FIT_FALLBACK_ISO_IF_NO_PHYSICAL",
        "if (!has_physical_item) { set_isometric_view(); return; }",
    ]:
        assert token in VIEW3D_CPP


def test_fallback_mode_and_banner_tokens_present():
    for token in ["2D Layout", "2D fallback preview active", "3D View unavailable, using 2D Layout"]:
        assert token in PREVIEW
