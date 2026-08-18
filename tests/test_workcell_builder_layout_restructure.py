from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
VIEW3D = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


def test_scene_builder_uses_main_splitter_layout():
    for token in [
        'scene_splitter->setObjectName("sceneBuilderMainSplitter")',
        'left_panel->setObjectName("sceneBuilderLeftPanel")',
        'center_panel->setObjectName("sceneBuilderProductViewPanel")',
        'right_panel->setObjectName("sceneBuilderRightPanel")',
        'scene_splitter->addWidget(left_panel);',
        'scene_splitter->addWidget(center_panel);',
        'scene_splitter->addWidget(right_panel);',
        'scene_splitter->setStretchFactor(1, 8);',
    ]:
        assert token in MAIN


def test_left_and_right_tabs_exist():
    for token in [
        'scene_builder_left_tabs_->addTab(scene_tab, "Scene")',
        'scene_builder_left_tabs_->addTab(assets_tab, "Assets")',
        'scene_builder_left_tabs_->addTab(files_tab, "Workflow")',
        'assets_tab_layout->addWidget(catalog_card, 1)',
    ]:
        assert token in MAIN
    for token in ['addTab(selection_tab, "Selection")', 'addTab(workflow_tab, "Workflow")', 'addTab(readiness_tab, "Readiness")']:
        assert token in MAIN
    assert 'addTab(actions_tab, "Actions")' not in MAIN


def test_scene_tree_headers_reflect_current_name_type_state_contract():
    for token in [
        'scene_hierarchy_tree_->setHeaderLabels({"Name", "Type", "State"})',
        'scene_hierarchy_tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);',
        'scene_hierarchy_tree_->header()->setSectionResizeMode(1, QHeaderView::Interactive);',
        'scene_hierarchy_tree_->header()->setSectionResizeMode(2, QHeaderView::Fixed);',
        'scene_hierarchy_tree_->setColumnWidth(1, 120);',
        'scene_hierarchy_tree_->setColumnWidth(2, 72);',
    ]:
        assert token in MAIN


def test_scene_semantic_role_selector_exposes_canonical_a9_roles():
    for token in [
        '{"Generic asset", "asset"}',
        '{"Pick object", "pick_object"}',
        '{"Target bin", "target_bin"}',
        '{"Fixture", "fixture"}',
        '{"Support surface", "support_surface"}',
        '{"Camera", "camera"}',
        '{"Sensor", "sensor"}',
        '{"Machine", "machine"}',
        '{"Conveyor", "conveyor"}',
        '{"Pick zone", "pick_zone"}',
        '{"Place zone", "place_zone"}',
        '{"Safety guard", "safety_guard"}',
        '{"Keepout marker", "keepout"}',
        '{"Home pose marker", "home_pose"}',
        '{"Environment object", "environment_object"}',
    ]:
        assert token in MAIN
    assert 'Filenames never infer pick/task semantics.' in MAIN


def test_inspector_scroll_and_activity_log_drawer_exist():
    for token in ['QScrollArea(right_panel)', 'setWidgetResizable(true)', 'sceneBuilderLogDrawer', 'Activity Log', 'sceneBuilderLogsButton']:
        assert token in MAIN


def test_canvas_more_menu_and_safety_text_present():
    for token in ['Canvas More', 'Duplicate Selected', 'Remove Selected Layout Item', 'Revert Layout', 'Run Layout Merge', 'Open Merge Report', 'Copy Merge Summary', 'Export Canvas Snapshot', 'Fake Hardware', 'No Robot Motion']:
        assert token in MAIN


def test_scene_preview_selected_state_updates_and_compact_inspector_pose_grid_present():
    for token in [
        'scene_preview_widget_->set_scene_selected(true);',
        'scene_preview_widget_->set_scene_selected(false);',
        'auto * pose_grid = new QGridLayout();',
        'scene_builder_log_toggle_button_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);',
    ]:
        assert token in MAIN


def test_workflow_tab_keeps_scene_artifact_access_without_polluting_hierarchy():
    for token in [
        'scene_files_tree_->setObjectName("studioSceneFilesTree")',
        'scene_files_tree_->setHeaderLabels({"Artifact", "Relative Path", "Status"})',
        'new QPushButton("Open Scene Folder", files_card)',
        'new QPushButton("Copy Scene Path", files_card)',
        'new QPushButton("Refresh Files", files_card)',
        'populate_scene_files_tab();',
        'scene_builder_left_tabs_->addTab(files_tab, "Workflow")',
    ]:
        assert token in MAIN


def test_selection_sync_invokes_apply_scene_selection_from_preview_signal():
    for token in [
        "connect(scene_preview_widget_, &ScenePreviewWidget::preview_item_selected, this, [this](const QString &id, const QString &role){",
        "apply_scene_selection(id, role, id.trimmed().isEmpty(), false);",
    ]:
        assert token in MAIN


def test_selection_callback_path_preserved_without_requiring_log_wording():
    assert 'if (pick_item_at_screen(e->pos(), best_id, best_role) && !best_id.isEmpty() && select_cb) select_cb(best_id, best_role);' in VIEW3D
    assert 'connect(scene_preview_widget_, &ScenePreviewWidget::preview_item_selected' in MAIN
    assert 'apply_scene_selection(id, role, id.trimmed().isEmpty(), false);' in MAIN
