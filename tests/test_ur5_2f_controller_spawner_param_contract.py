from __future__ import annotations

from pathlib import Path


def _launch_text() -> str:
    repo = Path(__file__).resolve().parents[1]
    launch = repo / "scenes" / "ur5_2f_test" / "launch" / "demo.launch.py"
    return launch.read_text(encoding="utf-8")


def test_ur5_2f_spawners_forward_controller_parameter_file() -> None:
    text = _launch_text()

    assert 'controllers_config_path = os.path.join(' in text
    assert text.count('"--param-file",\n            controllers_config_path,') == 3

    for controller in (
        "joint_state_broadcaster",
        "ur5_arm_controller",
        "ur5_gripper_controller",
    ):
        marker = f'"{controller}",\n            "--controller-manager",\n            "/controller_manager",\n            "--param-file",\n            controllers_config_path,'
        assert marker in text


def test_ros2_control_node_does_not_apply_global_controller_manager_name_remap() -> None:
    text = _launch_text()
    control_block = text.split('executable="ros2_control_node",', 1)[1].split(
        "joint_state_broadcaster_spawner = Node(", 1
    )[0]

    # launch_ros Node(name=...) becomes a __node remap for the whole process.
    # ros2_control_node creates controller lifecycle nodes in that process, so
    # keeping this unnamed preserves ur5_arm_controller / ur5_gripper_controller
    # identities and lets their YAML sections bind on ROS 2 Humble.
    assert 'name="controller_manager"' not in control_block
    assert '("~/robot_description", "/robot_description")' in control_block
