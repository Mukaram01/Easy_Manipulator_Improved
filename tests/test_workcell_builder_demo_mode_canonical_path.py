from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_demo_mode_calls_canonical_helpers():
    for token in [
        'create_scene_from_template(',
        'apply_recommended_layout_to_scene(',
        'save_new_scene_yaml(',
        'validate_new_scene(',
        'generate_full_scene_package_from_scene(',
        'update_new_scene_lifecycle_and_canvas(',
    ]:
        assert token in CPP

def test_demo_mode_has_single_run_function_and_no_parallel_path_tokens():
    assert 'run_demo_action(bool validate, bool generate)' in CPP
