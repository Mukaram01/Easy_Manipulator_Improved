from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
VIEW3D = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


def test_scene_builder_uses_main_splitter_layout():
    for token in [
        'sceneBuilderMainSplitter',
        'new QSplitter(Qt::Horizontal, scene_shell)',
        'setSizes({300, 1180, 360})',
    ]:
        assert token in MAIN


def test_left_and_right_tabs_exist():
    for token in ['addTab(scene_tab, "Scene")', 'addTab(assets_tab, "Assets")', 'addTab(files_tab, "Files")']:
        assert token in MAIN
    for token in ['addTab(selection_tab, "Selection")', 'addTab(workflow_tab, "Workflow")', 'addTab(readiness_tab, "Readiness")']:
        assert token in MAIN
    assert 'addTab(actions_tab, "Actions")' not in MAIN


def test_scene_tree_headers_include_name_role_status():
    for token in [
        'scene_hierarchy_tree_->setHeaderLabels({"Name", "Role", "Status"})',
        'header->setSectionResizeMode(0, QHeaderView::Interactive);',
        'header->setSectionResizeMode(1, QHeaderView::Interactive);',
        'header->setSectionResizeMode(2, QHeaderView::Interactive);',
        'header->setStretchLastSection(false);',
        'scene_hierarchy_tree_->setColumnWidth(0, 200);',
        'scene_hierarchy_tree_->setColumnWidth(1, 120);',
    ]:
        assert token in MAIN


def test_scene_population_role_taxonomy_tokens_present():
    for token in [
        'return QString("robot")',
        'return QString("end_effector/tool")',
        'return QString("camera")',
        'return QString("support_surface/table")',
        'return QString("pick source/zone")',
        'return QString("place target/bin")',
        'return QString("safety zone")',
    ]:
        assert token in MAIN


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


def test_scene_tree_excludes_file_artifact_rows_but_files_tab_keeps_required_artifacts():
    for token in [
        'const QSet<QString> excluded_scene_file_artifacts = {',
        '"package.xml",',
        '"CMakeLists.txt",',
        '"launch/demo.launch.py",',
        '"environment.yaml",',
        '"scene_manifest.yaml"',
    ]:
        assert token in MAIN

    for token in [
        '{"ROS Package File", "package.xml"}',
        '{"CMake File", "CMakeLists.txt"}',
        '{"Demo Launch", "launch/demo.launch.py"}',
        '{"Environment YAML", "environment.yaml"}',
        '{"Scene Manifest", "scene_manifest.yaml"}',
    ]:
        assert token in MAIN


def test_selection_sync_invokes_apply_scene_selection_from_preview_signal():
    for token in [
        "connect(scene_preview_widget_, &ScenePreviewWidget::preview_item_selected, this, [this](const QString &id, const QString &role){",
        "apply_scene_selection(id, role, id.trimmed().isEmpty(), false);",
    ]:
        assert token in MAIN


def test_selection_callback_path_and_log_format_tokens_preserved():
    for token in [
        'if (pick_item_at_screen(e->pos(), best_id, best_role) && !best_id.isEmpty() && select_cb) select_cb(best_id, best_role);',
        'append_studio_log(QString("Selected item: %1 (%2)").arg(selected_id, selected_role));',
    ]:
        assert token in VIEW3D or token in MAIN
