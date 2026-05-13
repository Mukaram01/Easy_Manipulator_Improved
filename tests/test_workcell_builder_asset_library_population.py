from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()


def test_asset_library_has_startup_population_entries():
    for item in ['UR5', 'Robotiq 2F', 'Table', 'Bin', 'RealSense D435i']:
        assert item in CPP
    assert 'initialize_asset_library' in CPP
