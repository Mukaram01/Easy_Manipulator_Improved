from pathlib import Path


def test_asset_browser_categories_present_in_source():
    text = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text()
    for token in ['robot', 'end_effector', 'camera_sensor']:
        assert token in text
    model = Path('workcell_builder/workcell_builder/gui/asset_catalog_model.cpp').read_text()
    for token in ['conveyor', 'camera_sensor', 'environment_object']:
        assert token in model
