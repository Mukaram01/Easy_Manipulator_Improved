from pathlib import Path


def test_original_scene_flow_actions_still_exist():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    for label in [
        'New Cell',
        'Open Cell',
        'Refresh Scenes',
        'Open Scene Folder',
        'Save / Generate environment.yaml',
        'Generate Full Scene Package',
        'Copy Fake-Hardware Launch Command',
    ]:
        assert label in ui


def test_real_generation_buttons_remain_wired_to_existing_handlers():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'on_generate_canonical_files_clicked()' in cpp
    assert 'on_generate_yaml_clicked();' in cpp
    assert 'on_generate_workcell_package_clicked()' in cpp
    assert 'on_generate_files_clicked();' in cpp
