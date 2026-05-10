from pathlib import Path
from scripts.workcell_builder_acceptance_check import build_golden_scene, launch_smoke


def test_generated_scene_launch_smoke_acceptance(tmp_path):
    scene = build_golden_scene('golden_ur5_2f_cell', tmp_path / 'scenes', tmp_path / 'assets')
    ok, errs = launch_smoke(scene)
    assert ok, errs


def test_launch_template_skips_invalid_gripper_controller_without_failing():
    content = Path('workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py').read_text(encoding='utf-8')
    assert "gripper controller skipped" in content
    assert "Skipping controller {controller_name}: joints missing from robot_description:" in content
