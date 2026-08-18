from __future__ import annotations

from pathlib import Path


def test_ur5_2f_spawners_forward_controller_parameter_file() -> None:
    repo = Path(__file__).resolve().parents[1]
    launch = repo / "scenes" / "ur5_2f_test" / "launch" / "demo.launch.py"
    text = launch.read_text(encoding="utf-8")

    assert 'controllers_config_path = os.path.join(' in text
    assert text.count('"--param-file",\n            controllers_config_path,') == 3

    for controller in (
        "joint_state_broadcaster",
        "ur5_arm_controller",
        "ur5_gripper_controller",
    ):
        marker = f'"{controller}",\n            "--controller-manager",\n            "/controller_manager",\n            "--param-file",\n            controllers_config_path,'
        assert marker in text
