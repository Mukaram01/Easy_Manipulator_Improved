from __future__ import annotations

from scripts.validate_scene_builder_ui_acceptance import (
    compose_layout_status_text,
    recommendation_action_for_preview_only,
    run_checks,
    selected_state_summary,
    workflow_rail_state_map,
)


def _base_fixture() -> dict[str, str]:
    return {
        "mainwindow_cpp": "\n".join(
            [
                'QSplitter *split = new QSplitter(this);',
                'split->setStretchFactor(0, 3);',
                'QToolButton *more = new QToolButton(this);',
                'more->setText("More Actions");',
                'QMenu *menu = new QMenu(this);',
                'QString previewTitle = "3D Layout Preview";',
                'QString fallback = "2D Layout";',
                'QString focus = "Focus Canvas";',
                'QString hide = "Hide";',
                'QString show = "Show";',
                'auto cmd = "ros2 launch scene demo.launch.py use_fake_hardware:=true";',
                'QString xyz = "XYZ";',
                'QString rpy = "RPY";',
                'QString x_units = "X position in metres";',
                'QString yaw_units = "Yaw in radians";',
                'QLabel *g = new QLabel("Gizmo:");',
                'QLabel *s = new QLabel("Snap:");',
                'QString mode_move = "Move";',
                'QString mode_rotate = "Rotate";',
                'QString title = "Scene3D Gizmo Transform";',
                'QString pick_axis = "pick_gizmo_axis_at_screen";',
                'QString pick_ring = "pick_gizmo_rotation_ring_at_screen";',
                'QString active_handle = "active_gizmo_handle";',
                'QString snap_t = "snap_translation_value";',
                'QString snap_r = "snap_rotation_value";',
                'viewport->enable_grid(true);',
                'viewport->set_floor_visible(true);',
                'viewport->show_axes(true);',
                'QString labels = "important-only";',
                'QString dense = "dense-hide";',
                'QString lm = "Label mode";',
                'QString lmd = "LabelMode::Important";',
                'QString promote = "Create editable layout from preview";',
                'if (e->key() == Qt::Key_Escape) { restore(); }',
                'void mouseReleaseEvent(QMouseEvent*) { /* single-commit */ commit(); }',
                'if (item->locked()) { status->setText("Locked:"); }',
                'inspector->sync();',
                'mark layout dirty;',
                'QString dnd = "application/x-workcell-asset-catalog-item";',
                'void dragEnterEvent(QDragEnterEvent*){}',
                'void dropEvent(QDropEvent*){}',
                'QString drop = "Drop to place table";',
                'auto cb = "asset_drop_cb";',
                'QString top_home = "Studio Home";',
                'QString top_new = "New Cell";',
                'QString top_scenes = "Scenes/Open";',
                'QString top_run = "Run Next";',
                'QString top_more = "More";',
                'QComboBox *mode = new QComboBox(this); mode->setObjectName("Mode");',
                'QComboBox *view = new QComboBox(this); view->setObjectName("View");',
                'QString side_tab = "Actions";',
                'QString section_layout = "Layout";',
                'QString grouped = "Grouped sections";',
                'QString parity = "Canvas/Generated Parity";',
                'QString undo_layout = "Undo Layout Edit";',
                'QString redo_layout = "Redo Layout Edit";',
            ]
        ),
        "preview_cpp": "Preview panel supports 2D Layout and 3D Layout Preview. Actions Layout Generate Validate Simulate Export Diagnostics",
        "layout_editor_cpp": "metres radians",
    }


def test_validator_passes_with_required_tokens_present():
    checks = run_checks(_base_fixture())
    assert all(c.ok for c in checks), [f"{c.name}: {c.details}" for c in checks if not c.ok]


def test_validator_fails_with_clear_diagnostics_when_tokens_or_patterns_missing():
    broken = _base_fixture()
    broken["mainwindow_cpp"] = broken["mainwindow_cpp"].replace("Focus Canvas", "")
    broken["mainwindow_cpp"] += '\nconnect(btn, &QPushButton::click);\naction_button->setFixedWidth(120);\n'

    checks = run_checks(broken)
    failed = {c.name: c.details for c in checks if not c.ok}

    assert "Focus Canvas action wording" in failed
    assert any("Focus Canvas" in d for d in failed["Focus Canvas action wording"])
    assert "fixed-width button anti-pattern checks" in failed


def test_validator_fails_when_undo_redo_are_direct_canvas_buttons():
    broken = _base_fixture()
    broken["mainwindow_cpp"] += '\nundo_layout_button_ = new QPushButton("Undo", scene_builder);\nlayout_controls->addWidget(undo_layout_button_);\nredo_layout_button_ = new QPushButton("Redo", scene_builder);\nlayout_controls->addWidget(redo_layout_button_);\n'
    checks = run_checks(broken)
    failed = {c.name: c.details for c in checks if not c.ok}
    assert "no always-visible direct undo/redo canvas-footer buttons" in failed


def test_validator_fails_when_forbidden_top_bar_primary_actions_exist():
    broken = _base_fixture()
    broken["mainwindow_cpp"] += '\nQAction *validate = toolbar->addAction("Validate");\n'
    checks = run_checks(broken)
    failed = {c.name: c.details for c in checks if not c.ok}
    assert "forbidden direct top-bar primary actions" in failed


def test_validator_reports_new_scene3d_acceptance_failures_when_markers_absent():
    broken = _base_fixture()
    broken["mainwindow_cpp"] = "\n".join(
        line
        for line in broken["mainwindow_cpp"].splitlines()
        if "pick_gizmo_axis_at_screen" not in line and "Qt::Key_Escape" not in line and "Locked:" not in line
    )

    checks = run_checks(broken)
    failed = {c.name: c.details for c in checks if not c.ok}

    assert "scene3d pick handle API markers" in failed
    assert "escape cancel restore path markers" in failed
    assert "locked item gating/status markers" in failed


def test_repository_ui_acceptance_validator_runs_on_current_sources():
    checks = run_checks()
    assert checks
    assert any(c.name == "fake hardware launch token" for c in checks)


def test_compose_layout_status_text_distinguishes_editable_and_preview_fallback_counts():
    assert compose_layout_status_text(4, 0) == "Editable assets: 4"
    assert compose_layout_status_text(4, 2) == "Editable assets: 4 | Preview-only fallback assets: 2"


def test_selected_scene_and_selected_item_have_separate_state_behavior():
    assert selected_state_summary("factory_scene", None) == "Selected scene: factory_scene"
    assert selected_state_summary("factory_scene", "table_asset") == "Selected item: table_asset"
    assert selected_state_summary(None, None) == "No selection"


def test_workflow_rail_state_map_includes_editable_layout_row_and_preview_mapping():
    editable = workflow_rail_state_map(editable_layout_ready=True, preview_only_scene=False)
    preview = workflow_rail_state_map(editable_layout_ready=False, preview_only_scene=True)

    assert editable["Editable layout"] == "done"
    assert editable["Review"] == "ready"
    assert preview["Editable layout"] == "recommended"
    assert preview["Review"] == "blocked"


def test_recommendation_action_for_preview_only_scene_is_explicit():
    assert recommendation_action_for_preview_only(True) == "Create editable layout from preview"
    assert recommendation_action_for_preview_only(False) == "Continue editing layout"
