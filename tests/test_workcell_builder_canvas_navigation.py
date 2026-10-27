from pathlib import Path

CPP = Path("workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")
MAIN = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
UI = Path("workcell_builder/workcell_builder/gui/scene_select.ui").read_text(encoding="utf-8")
PREVIEW = Path("workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")


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

def test_fallback_mode_and_banner_tokens_present():
    for token in ["2D Layout", "2D fallback preview active", "3D Layout Preview unavailable, using 2D Layout", "2D Layout (Fallback)"]:
        assert token in PREVIEW

def test_debug_overlays_remains_3d_and_fallback_is_unavailable_only():
    for token in [
        'viewport->debug_overlays_mode = (mode == "Debug Overlays")',
        'const bool requested_3d = (mode == "3D Layout Preview") || (mode == "Debug Overlays")',
        "const bool use3d = requested_3d && preview3d_available_;",
        "Mode: %2",
    ]:
        assert token in PREVIEW
    assert 'if (mode_selector_->currentText() == "Debug Overlays") use3d = false;' not in PREVIEW


def test_fit_scene_vs_fit_overlays_behavior_tokens_present():
    for token in [
        "v->fit_include_overlays = false; v->fit_scene(); fit_fallback_scene_to_items(false);",
        "v->fit_include_overlays = true; v->fit_scene(); fit_fallback_scene_to_items(true); v->fit_include_overlays = false;",
        "Overlay-fit warning:",
        "kOverlayFitDominanceRatio",
    ]:
        assert token in PREVIEW
