from pathlib import Path

CMAKE = Path('workcell_builder/workcell_builder/CMakeLists.txt').read_text(encoding='utf-8')
MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_helper_scripts_installed_to_share_scripts():
    for script in [
        'workcell_studio_layout_merge.py',
        'validate_workcell_studio_generated_scene.py',
        'workcell_studio_demo_mode.py',
        'workcell_studio_preview_launch.py',
        'run_workcell_studio_golden_flow.py',
    ]:
        assert script in CMAKE
    assert 'DESTINATION share/${PROJECT_NAME}/scripts' in CMAKE


def test_missing_helper_script_message_is_clear():
    assert 'Could not find Workcell Studio helper script' in MAIN
