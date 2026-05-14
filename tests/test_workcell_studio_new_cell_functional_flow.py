from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_new_cell_flow_has_safety_and_launch_commands():
    src = (ROOT / "workcell_builder/workcell_builder/src_workcell_studio_template_instantiator.cpp").read_text(encoding="utf-8")
    assert "fake_hardware_first: true" in src
    assert "runtime_execution_enabled: false" in src
    assert "motion_command_sent: false" in src
    assert "ros2 launch " in src
    assert "use_fake_hardware:=true" in src
