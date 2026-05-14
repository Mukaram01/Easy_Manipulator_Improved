from pathlib import Path

SRC_BROWSER = Path('workcell_builder/workcell_builder/src_workcell_studio_scene_browser.cpp').read_text(encoding='utf-8')
MAIN_CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_source_includes_workspace_scene_priority_paths():
    assert 'src" / "easy_manipulation_deployment" / "scenes' in SRC_BROWSER
    assert 'src" / "scenes' in SRC_BROWSER


def test_dashboard_no_scenes_message_shows_searched_paths_and_hint():
    assert 'Searched:' in MAIN_CPP
    assert 'Check selected workspace or symlink ~/workcell_ws/src/scenes' in MAIN_CPP


def test_dashboard_safety_and_non_plain_table_ui_tokens_present():
    assert 'Fake Hardware | No Robot Motion' in MAIN_CPP
    assert 'QTableWidget{background:#1f2937' in MAIN_CPP


def test_scene_validation_tokens_present():
    for token in ['environment.yaml', 'scene_manifest.yaml', 'launch" / "demo.launch.py', 'environment.urdf.xacro']:
        assert token in SRC_BROWSER
