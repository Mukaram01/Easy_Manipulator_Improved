from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_scene_builder_uses_main_splitter_layout():
    for token in [
        'sceneBuilderMainSplitter',
        'QSplitter(Qt::Horizontal, scene_shell)',
        'setSizes({290, 980, 350})',
    ]:
        assert token in MAIN


def test_left_and_right_tabs_exist():
    for token in ['addTab(scene_tab, "Scene")', 'addTab(assets_tab, "Assets")', 'addTab(files_tab, "Files")']:
        assert token in MAIN
    for token in ['addTab(selection_tab, "Selection")', 'addTab(task_tab, "Task")', 'addTab(readiness_tab, "Readiness")', 'addTab(actions_tab, "Actions")']:
        assert token in MAIN


def test_scene_tree_headers_include_name_role_status():
    for token in [
        'scene_hierarchy_tree_->setHeaderLabels({"Name", "Role", "Status"})',
    ]:
        assert token in MAIN


def test_scene_population_role_taxonomy_tokens_present():
    for token in [
        'return QString("robot")',
        'return QString("end_effector/tool")',
        'return QString("camera")',
        'return QString("support_surface/table")',
        'return QString("pick_source/pick_zone")',
        'return QString("place_target/bin")',
        'return QString("safety_zone")',
        'return QString("unknown")',
    ]:
        assert token in MAIN


def test_inspector_scroll_and_activity_log_toggle_exist():
    for token in ['QScrollArea(right_panel)', 'setWidgetResizable(true)', 'Show Log', 'Hide Log', '<b>Activity Log</b>']:
        assert token in MAIN


def test_canvas_more_menu_and_safety_text_present():
    for token in ['Canvas More...', 'Fake Hardware', 'No Robot Motion']:
        assert token in MAIN
