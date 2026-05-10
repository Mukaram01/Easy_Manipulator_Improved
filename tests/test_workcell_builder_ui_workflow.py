from pathlib import Path


def test_start_tab_workflow_and_primary_actions_present():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    assert 'Recommended Workflow' in ui
    assert 'Save / Generate environment.yaml' in ui
    assert 'Generate Full Scene Package' in ui
    assert 'Copy Fake-Hardware Launch Command' in ui


def test_placeholder_buttons_marked_coming_soon_or_disabled():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'placeholder_buttons' in cpp
    assert '(coming soon)' in cpp
    assert 'button->setDisabled(true);' in cpp
