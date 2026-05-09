from pathlib import Path
REPO_ROOT = Path(__file__).resolve().parents[1]

def test_collision_launch_arg_present():
    txt=(REPO_ROOT/'workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py').read_text(encoding='utf-8')
    assert 'publish_collision_objects' in txt
    assert 'workcell_collision_scene_publisher.py' in txt
