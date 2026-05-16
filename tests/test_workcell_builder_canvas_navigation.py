from pathlib import Path

CPP = Path("workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")
MAIN = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
UI = Path("workcell_builder/workcell_builder/gui/scene_select.ui").read_text(encoding="utf-8")


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
    assert "drawText(" not in MAIN
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
