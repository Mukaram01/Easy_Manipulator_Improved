from pathlib import Path


def test_start_tab_workflow_and_primary_actions_present():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    assert 'Recommended Workflow' in ui
    assert 'Save / Generate environment.yaml' in ui
    assert 'Generate Full Scene Package' in ui
    assert 'Copy Fake-Hardware Launch Command' in ui


def test_unwired_workcell_studio_shell_controls_are_hidden_in_rescue_mode():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'ui->asset_browser_group->hide();' in cpp
    assert 'ui->inspector_group->hide();' in cpp
