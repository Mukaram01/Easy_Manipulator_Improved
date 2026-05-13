from pathlib import Path


def test_scene_templates_and_open_wizard_visible_and_recommended_layout_wired():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'Scenario Templates' in ui
    assert 'Conveyor Sorting - Live EPD Preview' in ui
    assert 'Open Wizard: Conveyor Sorting - Live EPD Preview' in ui
    assert 'use_recommended_layout' in ui
    assert 'on_use_recommended_layout_clicked' in cpp
    assert 'Recommended layout applied' in cpp
    assert 'Select or create a scenario before applying layout' in cpp


def test_main_page_scenario_templates_section_not_empty():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    assert 'name="scenario_templates_group"' in ui
    assert 'create_conveyor_sorting_live_epd_preview' in ui
