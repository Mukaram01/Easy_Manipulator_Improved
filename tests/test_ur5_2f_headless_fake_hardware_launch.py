from pathlib import Path
from types import SimpleNamespace

from scripts import validate_rviz_moveit_simulation_launches as validator


ROOT = Path(__file__).resolve().parents[1]
SCENE_DIR = ROOT / "scenes" / "ur5_2f_test"
LAUNCH_FILE = SCENE_DIR / "launch" / "demo.launch.py"


def test_canonical_launch_declares_real_rviz_toggle():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert "from launch.conditions import IfCondition" in source
    assert 'launch_rviz = LaunchConfiguration("launch_rviz")' in source
    assert 'DeclareLaunchArgument(\n            "launch_rviz"' in source
    assert 'default_value="true"' in source

    rviz_block = source.split("rviz_node = Node(", 1)[1].split("\n    return [", 1)[0]
    assert 'package="rviz2"' in rviz_block
    assert "condition=IfCondition(launch_rviz)" in rviz_block

    move_group_block = source.split("move_group = Node(", 1)[1].split("rviz_config_file", 1)[0]
    assert "IfCondition(launch_rviz)" not in move_group_block
    assert '"allow_trajectory_execution": False' in source
    assert '"moveit_manage_controllers": False' in source


def test_validator_headless_command_matches_declared_launch_contract():
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    declared = validator._declared_launch_args(source)
    assert declared["use_fake_hardware"] == "true"
    assert declared["launch_rviz"] == "true"

    entry = SimpleNamespace(
        fake_hardware_launch_command=(
            "ros2 launch ur5_2f_test demo.launch.py "
            "use_fake_hardware:=true launch_rviz:=true"
        )
    )
    headless = validator.command_for_scene(entry, launch_rviz=False)
    visual = validator.command_for_scene(entry, launch_rviz=True)

    assert "use_fake_hardware:=true" in headless
    assert "launch_rviz:=false" in headless
    assert "launch_rviz:=true" in visual

    failures, evidence, metadata = validator.validate_launch_contract(
        entry, SCENE_DIR, LAUNCH_FILE, launch_rviz=True
    )
    assert failures == []
    assert "fake_hardware_default_true" in evidence
    assert metadata["declared_launch_arguments"]["launch_rviz"] == "true"


def test_headless_toggle_only_gates_rviz_not_fake_hardware_nodes():
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    setup = source.split("def _launch_setup(context):", 1)[1].split(
        "def generate_launch_description():", 1
    )[0]

    for required_node in [
        "static_tf = Node(",
        "robot_state_publisher = Node(",
        "joint_state_publisher = Node(",
        "move_group = Node(",
    ]:
        block = setup.split(required_node, 1)[1].split("\n    )", 1)[0]
        assert "IfCondition(launch_rviz)" not in block

    rviz_block = setup.split("rviz_node = Node(", 1)[1].split("\n    return [", 1)[0]
    assert "condition=IfCondition(launch_rviz)" in rviz_block
