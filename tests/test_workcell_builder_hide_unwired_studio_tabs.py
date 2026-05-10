from pathlib import Path


def test_unwired_tabs_are_removed_from_default_visible_workflow():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for tab in ['ingredients_tab', 'layout_tab', 'task_tab', 'perception_roi_tab', 'grasp_tab']:
        assert f'ui->workflow_tabs->removeTab(ui->workflow_tabs->indexOf(ui->{tab}));' in cpp


def test_generate_tab_remains_for_real_actions():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'ui->workflow_tabs->setTabText(' in cpp
    assert '"Generate"' in cpp
