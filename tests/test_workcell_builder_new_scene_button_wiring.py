from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_new_scene_buttons_route_to_canonical_helpers():
    for token in [
        'create_scene_from_template(',
        'apply_recommended_layout_to_scene(',
        'save_new_scene_yaml(',
        'generate_full_scene_package_from_scene(',
        'update_new_scene_lifecycle_and_canvas(',
    ]:
        assert token in CPP

def test_copy_command_buttons_do_not_silently_noop():
    assert 'Copy Build Command blocked: generate package first.' in CPP
    assert 'Copy Launch Command blocked: generate package first.' in CPP
