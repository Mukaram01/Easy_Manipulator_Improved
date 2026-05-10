from pathlib import Path


def test_humble_demo_launch_has_controller_joint_filtering_and_skip_warning():
    launch_text = Path('workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py').read_text(encoding='utf-8')
    assert '_filter_controller_configs' in launch_text
    assert 'Skipping controller {controller_name}: joints missing from robot_description:' in launch_text
    assert 'controller_configs, controller_filter_warnings, controller_statuses = _filter_controller_configs(' in launch_text


def test_humble_demo_launch_reports_arm_ready_and_fake_hardware_status():
    launch_text = Path('workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py').read_text(encoding='utf-8')
    assert 'arm controller ready' in launch_text
    assert 'fake hardware launch available' in launch_text
    assert 'default_value="true"' in launch_text
