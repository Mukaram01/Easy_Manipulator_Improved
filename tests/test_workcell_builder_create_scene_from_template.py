from pathlib import Path
H = Path('workcell_builder/workcell_builder/gui/scene_select.h').read_text()
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_canonical_helper_signatures_exist():
    for token in [
        'bool create_scene_from_template(',
        'bool apply_recommended_layout_to_scene(',
        'bool save_new_scene_yaml(',
        'bool validate_new_scene(',
        'bool generate_full_scene_package_from_scene(',
        'void update_new_scene_lifecycle_and_canvas(',
        'std::string sanitize_scene_name(',
    ]:
        assert token in H or token in CPP

def test_safety_defaults_persist_in_layout():
    assert 'fake_hardware_first: true' in CPP
    assert 'runtime_execution_enabled: false' in CPP
