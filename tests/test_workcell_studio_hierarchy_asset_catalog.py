from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_scans_asset_roots_and_scene_files():
    assert 'easy_manipulation_deployment" / "assets' in CPP
    assert '"src" / "assets"' in CPP
    assert 'environment.yaml' in CPP
    assert 'scene_manifest.yaml' in CPP


def test_hierarchy_groups_and_catalog_categories_present():
    for token in [
        'Robot', 'End Effector', 'Camera / Sensor', 'Conveyor', 'Work Table / Surface',
        'Pick Source / Bin', 'Place Target / Fixture', 'Safety'
    ]:
        assert token in CPP
    for token in ['Robots', 'End Effectors', 'Sensors', 'Tables', 'Conveyors', 'Bins', 'Fixtures', 'Custom']:
        assert token in CPP


def test_tree_widgets_and_selected_item_actions_present():
    assert 'QTreeWidget' in CPP
    assert 'Open Asset Folder' in CPP
    assert 'Copy Asset Path' in CPP
    assert 'Add to Canvas' in CPP


def test_safety_text_remains_present():
    assert 'Fake Hardware' in CPP
    assert 'No Robot Motion' in CPP
