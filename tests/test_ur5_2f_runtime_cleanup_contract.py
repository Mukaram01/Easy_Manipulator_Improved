from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "ur5_2f_test"


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
