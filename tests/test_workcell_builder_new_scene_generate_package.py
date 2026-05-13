from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_generate_full_scene_package_routes_to_shared_handler():
    assert 'on_generate_full_scene_package_start_clicked' in CPP
    assert 'on_generate_files_clicked();' in CPP
