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
                'QString top_scenes = "Scenes";',
                'QString top_run = "Run Next";',
                'QString top_more = "More";',
                'scenes_open_button->setText("Scenes");',
                'run_next_button->setText("Run Next");',
                'more_button->setText("More");',
                'scene_builder_secondary_overflow_menu_->addMenu(overlays_menu)->setText("Overlays");',
                'scene_builder_secondary_overflow_menu_->addMenu(visual_modes_menu)->setText("Visual Modes");',
                'scene_builder_secondary_overflow_menu_->addMenu(canvas_more_menu)->setText("Layout/Edit Settings");',
                'QComboBox *mode = new QComboBox(this); mode->setObjectName("Mode");',
                'QComboBox *view = new QComboBox(this); view->setObjectName("View");',
                'QString side_tab = "Actions";',
                'QString section_layout = "Layout";',
                'QString grouped = "Grouped sections";',
                'QString parity = "Canvas/Generated Parity";',
                'QString undo_layout = "Undo Layout Edit";',
                'QString redo_layout = "Redo Layout Edit";',
                'dashboard_scene_actions_button_ = new QToolButton(dashboard_selected_scene_card_);',
                'dashboard_scene_actions_button_->setText("Scene Actions");',
                'dashboard_scene_actions_menu_ = new QMenu(dashboard_scene_actions_button_);',
                'dashboard_open_scene_action_ = dashboard_scene_actions_menu_->addAction("Open in Scene Builder");',
                'dashboard_validate_action_ = dashboard_scene_actions_menu_->addAction("Validate");',
                'dashboard_plan_action_ = dashboard_scene_actions_menu_->addAction("Plan / Simulate");',
                'dashboard_export_action_ = dashboard_scene_actions_menu_->addAction("Export");',
                'dashboard_scene_actions_menu_->addSeparator();',
                'dashboard_delete_action_ = dashboard_scene_actions_menu_->addAction("Delete Scene");',
                'dashboard_scene_actions_button_->setMenu(dashboard_scene_actions_menu_);',
                'connect(dashboard_open_scene_action_, &QAction::triggered, this, [this](){ open_scene_builder_for_selected_scene("Dashboard Open in Scene Builder"); });',
                'connect(dashboard_validate_action_, &QAction::triggered, this, [this](){ if (action_validate_offline_) action_validate_offline_->trigger(); });',
                'connect(dashboard_plan_action_, &QAction::triggered, this, [this](){ if (action_simulate_plan_preview_) action_simulate_plan_preview_->trigger(); });',
                'connect(dashboard_export_action_, &QAction::triggered, this, [this](){ if (action_export_open_page_) action_export_open_page_->trigger(); });',
                'connect(dashboard_delete_action_, &QAction::triggered, this, &MainWindow::delete_selected_scene);',
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
    broken["mainwindow_cpp"] += '\nheader_layout->addWidget(new QPushButton("Validate", scene_builder));\n'
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


def test_validator_fails_for_top_header_label_hacks():
    broken = _base_fixture()
    broken["mainwindow_cpp"] = broken["mainwindow_cpp"].replace('scenes_open_button->setText("Scenes");','scenes_open_button->setText("Scenes/Open");')
    broken["mainwindow_cpp"] = broken["mainwindow_cpp"].replace('run_next_button->setText("Run Next");','run_next_button->setText("Run Next_");')
    checks = run_checks(broken)
    failed = {c.name: c.details for c in checks if not c.ok}

    assert "top-header labels are exact and menu-hint free" in failed
    assert any("forbidden top-header label hack" in d for d in failed["top-header labels are exact and menu-hint free"])




def test_validator_fails_when_new_forbidden_always_visible_buttons_exist():
    broken = _base_fixture()
    broken["mainwindow_cpp"] += '\nheader_layout->addWidget(new QPushButton("Delete Scene", scene_builder));\nheader_layout->addWidget(new QPushButton("Select", scene_builder));\nheader_layout->addWidget(new QPushButton("Place Asset", scene_builder));\nheader_layout->addWidget(new QPushButton("Move", scene_builder));\nheader_layout->addWidget(new QPushButton("Inspect", scene_builder));\nheader_layout->addWidget(new QPushButton("Save Layout", scene_builder));\nheader_layout->addWidget(new QPushButton("Undo", scene_builder));\nheader_layout->addWidget(new QPushButton("Redo", scene_builder));\nheader_layout->addWidget(new QPushButton("Validate", scene_builder));\nheader_layout->addWidget(new QPushButton("Plan", scene_builder));\nheader_layout->addWidget(new QPushButton("Simulate", scene_builder));\nheader_layout->addWidget(new QPushButton("Export", scene_builder));\ndashboard_open_scene_button_ = new QPushButton("Open in Scene Builder", dashboard_selected_scene_card_);\ndashboard_validate_button_ = new QPushButton("Validate", dashboard_selected_scene_card_);\ndashboard_plan_button_ = new QPushButton("Plan / Simulate", dashboard_selected_scene_card_);\ndashboard_export_button_ = new QPushButton("Export", dashboard_selected_scene_card_);\ndashboard_delete_button_ = new QPushButton("Delete Scene", dashboard_selected_scene_card_);\n'
    checks = run_checks(broken)
    failed = {c.name: c.details for c in checks if not c.ok}

    assert "forbidden direct top-bar primary actions" in failed
    assert "forbidden always-visible selected-scene action buttons" in failed


def test_validator_allows_forbidden_labels_in_actions_tab_or_menus_only():
    fixture = _base_fixture()
    fixture["mainwindow_cpp"] += '\nQToolButton *dashboard_scene_actions_button_ = new QToolButton(this);\nQMenu *dashboard_scene_actions_menu_ = new QMenu(this);\ndashboard_scene_actions_menu_->addAction("Open in Scene Builder");\ndashboard_scene_actions_menu_->addAction("Validate");\ndashboard_scene_actions_menu_->addAction("Plan / Simulate");\ndashboard_scene_actions_menu_->addAction("Export");\ndashboard_scene_actions_menu_->addSeparator();\ndashboard_scene_actions_menu_->addAction("Delete Scene");\n'
    checks = run_checks(fixture)
    failed = {c.name: c.details for c in checks if not c.ok}
    assert "forbidden direct top-bar primary actions" not in failed
    assert "forbidden always-visible selected-scene action buttons" not in failed

def test_validator_fails_when_secondary_canvas_controls_are_added_as_direct_toolbar_widgets():
    broken = _base_fixture()
    broken["mainwindow_cpp"] += '\ncontrols->addWidget(scene_builder_overlays_button_);\ncontrols->addWidget(scene_builder_canvas_more_button_);\ncontrols->addWidget(scene_builder_visual_modes_button_);\ncontrols->addWidget(toggle_grid_box_);\ncontrols->addWidget(snap_to_grid_box_);\ncontrols->addWidget(fine_move_mode_box_);\ncontrols->addWidget(unlock_robot_base_box_);\ncontrols->addWidget(toggle_labels_box_);\ncontrols->addWidget(toggle_warnings_box_);\ncontrols->addWidget(show_minimap_box_);\n'
    checks = run_checks(broken)
    failed = {c.name: c.details for c in checks if not c.ok}
    assert "secondary canvas controls are not direct always-visible toolbar widgets" in failed


def test_validator_allows_secondary_canvas_controls_inside_more_menu_surfaces():
    fixture = _base_fixture()
    fixture["mainwindow_cpp"] += '\nscene_builder_secondary_overflow_menu_->addMenu(overlays_menu)->setText("Overlays");\nscene_builder_secondary_overflow_menu_->addMenu(visual_modes_menu)->setText("Visual Modes");\nscene_builder_secondary_overflow_menu_->addMenu(canvas_more_menu)->setText("Layout/Edit Settings");\n'
    checks = run_checks(fixture)
    failed = {c.name: c.details for c in checks if not c.ok}
    assert "secondary canvas controls remain available in menu surfaces" not in failed


def test_repository_contract_rejects_reintroduced_selected_scene_direct_buttons():
    checks = run_checks()
    failed = {c.name: c.details for c in checks if not c.ok}
    assert "forbidden always-visible selected-scene action buttons" not in failed
    assert "selected-scene card exposes only Scene Actions control with menu wiring" not in failed


def test_validator_rejects_explicit_trailing_underscore_nav_labels():
    broken = _base_fixture()
    broken["mainwindow_cpp"] += '\nQLabel *hack1 = new QLabel("Run Next_");\nQLabel *hack2 = new QLabel("More_");\nQLabel *hack3 = new QLabel("Scenes/Open_");\n'
    checks = run_checks(broken)
    failed = {c.name: c.details for c in checks if not c.ok}

    assert "top-header labels reject trailing underscore hacks" in failed



def test_validator_repository_run_has_no_visible_top_level_false_positive_failures():
    checks = run_checks()
    visible_check = next(c for c in checks if c.name == "allowed visible top-level set only")
    assert visible_check.ok, visible_check.details

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
