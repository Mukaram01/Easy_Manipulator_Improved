from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "ur5_2f_test"
UR_CONTROL_XACRO = (
    ROOT
    / "assets"
    / "robots"
    / "universal_robot"
    / "ur_description"
    / "urdf"
    / "ur.ros2_control.xacro"
)


def test_fake_hardware_launch_uses_non_deprecated_static_tf_and_intentional_epd_octomap_policy():
    text = (SCENE / "launch" / "demo.launch.py").read_text(encoding="utf-8")

    assert '"--frame-id", world_frame' in text
    assert '"--child-frame-id", "workcell_reference"' in text
    assert 'arguments=["0", "0", "0", "0", "0", "0", world_frame, "workcell_reference"]' not in text

    assert '"octomap_resolution": 0.1' in text
    assert '"octomap_frame": world_frame' in text
    assert '"sensors": ["workcell_epd_collision_objects"]' in text
    assert '"sensor_plugin": "~workcell_epd_collision_objects_are_planning_truth"' in text


def test_fake_controllers_select_safe_trajectory_end_behavior():
    controllers = yaml.safe_load(
        (SCENE / "config" / "ros2_controllers.yaml").read_text(encoding="utf-8")
    )

    for name in ("ur5_arm_controller", "ur5_gripper_controller"):
        params = controllers[name]["ros__parameters"]
        assert params["allow_nonzero_velocity_at_trajectory_end"] is False


def test_ur_generic_system_uses_current_mock_contract_without_real_driver_io_interfaces():
    text = UR_CONTROL_XACRO.read_text(encoding="utf-8")

    assert '<plugin>mock_components/GenericSystem</plugin>' in text
    assert '<param name="mock_sensor_commands">${fake_sensor_commands}</param>' in text
    assert '<param name="fake_sensor_commands">' not in text
    assert '<xacro:unless value="${use_fake_hardware or sim_gazebo or sim_ignition}">\n        <sensor name="${tf_prefix}tcp_fts_sensor">' in text

    # Keep the public xacro argument for callers that still use the old UR name;
    # only the ros2_control GenericSystem parameter is modernized.
    assert 'use_fake_hardware:=false fake_sensor_commands:=false' in text
    assert '<plugin>ur_robot_driver/URPositionHardwareInterface</plugin>' in text


def test_octomap_skip_contract_matches_moveit_humble_middleware_semantics():
    # MoveIt Humble explicitly skips updater plugin names whose first character
    # is '~'. Keep that contract obvious here so nobody replaces it with a fake
    # plugin class and turns an informational no-Octomap policy into a load error.
    launch = (SCENE / "launch" / "demo.launch.py").read_text(encoding="utf-8")
    assert '"sensor_plugin": "~workcell_epd_collision_objects_are_planning_truth"' in launch
