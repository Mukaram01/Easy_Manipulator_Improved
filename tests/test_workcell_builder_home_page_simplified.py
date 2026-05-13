from pathlib import Path

UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text()
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()


def test_home_primary_actions_present():
    for token in ['New Cell', 'Open Cell', 'Refresh Scenes']:
        assert token in UI


def test_assets_and_scenes_not_permanent_side_strips():
    assert 'ui->asset_browser_group->hide();' in CPP
    assert 'ui->inspector_group->hide();' in CPP
    assert 'assetsPageLayout' in UI
    assert 'scenesPageLayout' in UI


def test_logs_collapsible_panel_exists():
    assert 'QToolBox' in UI
    assert 'Logs' in UI
