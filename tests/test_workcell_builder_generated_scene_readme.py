from pathlib import Path


def test_generated_scene_readme_and_session_summary_tokens_present_in_scene_select_cpp():
    text = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for token in [
        'README.md',
        'builder_session_summary.json',
        'scene_name',
        'selected_robot',
        'selected_tool',
        'fake_hardware_default',
        'colcon build --symlink-install --packages-select ',
        'ros2 launch ',
        'use_fake_hardware:=true',
        'no robot motion command',
    ]:
        assert token in text
