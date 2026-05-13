from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()


def test_workflow_tabs_not_removed():
    assert 'removeTab' not in CPP


def test_disabled_buttons_are_explicitly_explained():
    assert 'Disabled: this control requires feature-complete editor integration and is intentionally blocked.' in CPP


def test_key_buttons_connected():
    for token in [
        'on_create_scenario_template_clicked',
        'on_create_conveyor_sorting_live_epd_preview_clicked',
        'on_use_recommended_layout_clicked',
        'on_refresh_scenes_button_clicked',
        'on_open_scene_folder_clicked',
    ]:
        assert token in CPP
