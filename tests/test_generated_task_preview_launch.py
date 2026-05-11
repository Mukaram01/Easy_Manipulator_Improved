from pathlib import Path

TEMPLATE = Path('workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py')


def test_generated_demo_launch_uses_generated_preview_helper():
    launch = TEMPLATE.read_text(encoding='utf-8')
    assert 'workcell_visual_scene_publisher.py' in launch
    assert 'task_recipe_visualizer_node.py' not in launch
    assert 'Task preview helper not found; continuing without task preview.' in launch


def test_generated_demo_launch_supports_task_preview_toggle():
    launch = TEMPLATE.read_text(encoding='utf-8')
    assert 'DeclareLaunchArgument(' in launch
    assert '"launch_task_preview"' in launch
    assert 'condition=IfCondition(launch_task_preview)' in launch
