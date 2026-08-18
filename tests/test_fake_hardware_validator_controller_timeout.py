from pathlib import Path


def test_controller_cli_gets_longer_per_attempt_timeout_than_ordinary_ros_checks() -> None:
    repo = Path(__file__).resolve().parents[1]
    validator = repo / "scripts" / "validate_rviz_moveit_simulation_launches.py"
    text = validator.read_text(encoding="utf-8")

    assert "attempt_timeout_cap: int = 3" in text
    assert "min(attempt_timeout_cap, int(remaining + 0.999))" in text

    controller_marker = "controllers = _wait_for_ros_check("
    start = text.index(controller_marker)
    fragment = text[start : start + 520]
    assert "controller_budget" in fragment
    assert "attempt_timeout_cap=controller_budget" in fragment

    # Ordinary ROS checks intentionally retain the short default cap.
    ordinary_marker = 'nodes = _wait_for_ros_check(["ros2", "node", "list"], startup_budget, node_predicate)'
    assert ordinary_marker in text
