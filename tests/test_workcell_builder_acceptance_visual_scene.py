from pathlib import Path
REPO_ROOT = Path(__file__).resolve().parents[1]

def test_fake_hardware_default_remains_true():
    txt=(REPO_ROOT/'workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py').read_text(encoding='utf-8')
    assert 'DeclareLaunchArgument(\n            "use_fake_hardware"' in txt
    assert 'default_value="true"' in txt
