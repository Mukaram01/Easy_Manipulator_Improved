from pathlib import Path

def test_launch_has_publish_perception_replay_default_false():
    t=Path('workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py').read_text(encoding='utf-8')
    assert 'publish_perception_replay' in t and 'default_value="false"' in t
