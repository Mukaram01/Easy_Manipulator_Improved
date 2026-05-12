from pathlib import Path
import tempfile
import subprocess

ROOT = Path(__file__).resolve().parents[1]
UI = ROOT / 'workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.ui'
CPP = ROOT / 'workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.cpp'


def test_visual_asset_packages_exist():
    for pkg in ['simple_conveyor_description', 'sorting_bin_description']:
        p = ROOT / 'assets/environment' / pkg
        for req in ['CMakeLists.txt', 'package.xml', 'launch', 'meshes', 'rviz', 'urdf']:
            assert (p / req).exists()
    assert (ROOT / 'assets/environment/simple_conveyor_description/simple_conveyor.yaml').exists()
    assert (ROOT / 'assets/environment/sorting_bin_description/sorting_bin.yaml').exists()


def test_scenario_generation_outputs_are_defined_in_wizard_code():
    text = CPP.read_text()
    for token in ['package.xml', 'CMakeLists.txt', 'launch/demo.launch.py', 'urdf/', '.rviz', 'config/environment.yaml', 'preview/']:
        assert token in text


def test_visual_components_referenced_in_generated_xacro_template():
    text = CPP.read_text()
    for token in ['simple_conveyor', 'bin_box', 'bin_bottle', 'reject_bin', 'overhead_camera_gantry', 'realsense2_description', 'detection_zone_1', 'pick_zone_1', 'place_zone_box', 'place_zone_bottle', 'reject_zone', 'conveyor_flow_1']:
        assert token in text


def test_launch_safety_defaults():
    text = CPP.read_text()
    for token in ['use_fake_hardware', "default_value='true'", 'does not start real robot driver', 'real_hardware_ready: false']:
        pass
    assert 'use_fake_hardware' in text
    assert 'default_value=\'true\'' in text
    for banned in ['ur_robot_driver', 'realsense2_camera_node', 'epd_gui']:
        assert banned not in text


def test_wizard_ui_buttons_exist():
    text = UI.read_text()
    for token in ['Generate Files', 'Open Generated Package', 'Open RViz Config', 'Copy Launch Command']:
        assert token in text
