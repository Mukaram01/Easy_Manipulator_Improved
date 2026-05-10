from pathlib import Path

from scripts.workcell_builder_acceptance_check import build_golden_scene, package_consistency


def test_generated_package_consistency(tmp_path):
    scene = build_golden_scene('golden_ur5_2f_cell', tmp_path / 'scenes', tmp_path / 'assets')
    ok, errs = package_consistency(scene)
    assert ok, errs


def test_generated_template_package_artifacts_are_present():
    root = Path('workcell_builder/workcell_builder/templates/ros2/humble')
    assert (root / 'launch/demo.launch.py').exists()
    launch_text = (root / 'launch/demo.launch.py').read_text(encoding='utf-8')
    assert 'load_yaml(robot_moveit_pkg, "config/kinematics.yaml")' in launch_text
