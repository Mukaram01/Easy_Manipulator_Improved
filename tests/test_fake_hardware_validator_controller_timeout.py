from pathlib import Path


def test_controller_check_uses_direct_ros_service_instead_of_ros2_control_cli() -> None:
    repo = Path(__file__).resolve().parents[1]
    validator = repo / "scripts" / "validate_rviz_moveit_simulation_launches.py"
    text = validator.read_text(encoding="utf-8")

    assert "from controller_manager_msgs.srv import ListControllers" in text
    assert "_query_controller_states(controller_budget)" in text
    assert '"transport": "rclpy_service"' in text
    assert '"service": "/controller_manager/list_controllers"' in text

    # Controller acceptance must not depend on a fresh ros2controlcli subprocess.
    assert '["ros2", "control", "list_controllers"' not in text
    assert "--spin-time" not in text
    assert "attempt_timeout_cap" not in text

    # Ordinary ROS CLI probes remain bounded independently.
    assert 'nodes = _wait_for_ros_check(["ros2", "node", "list"], startup_budget, node_predicate)' in text
