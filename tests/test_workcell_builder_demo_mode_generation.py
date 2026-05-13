from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_demo_generation_flow_tokens_exist():
    for token in [
        'demo_ur5_2f_pick_place',
        'sanitize_scene_name(',
        'save_new_scene_yaml(',
        'validate_new_scene(',
        'generate_full_scene_package_from_scene(',
        'export_workcell_layout_preview(',
        'colcon build --symlink-install --packages-select',
        'ros2 launch ',
    ]:
        assert token in CPP
