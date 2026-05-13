from pathlib import Path

DEFAULT = '-1.5708 -1.5708 0'


def test_scene_xacro_default_mount_rpy_is_nonzero():
    text = Path('workcell_builder/workcell_builder/include/scene_xacro_parser.h').read_text(encoding='utf-8')
    assert 'fallback.roll = -1.5708F' in text
    assert 'fallback.pitch = -1.5708F' in text


def test_yaml_default_mount_rpy_matches_scene_default():
    text = Path('workcell_builder/workcell_builder/include/yaml_parser/generate_yaml.h').read_text(encoding='utf-8')
    assert 'origin.roll = -1.5708F' in text
    assert 'origin.pitch = -1.5708F' in text


def test_wizard_exposes_gripper_mount_rpy_and_default_button():
    ui = Path('workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.ui').read_text(encoding='utf-8')
    cpp = Path('workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.cpp').read_text(encoding='utf-8')
    assert 'Gripper Mount RPY' in ui
    assert 'Use Default Gripper Orientation' in ui
    assert '-1.5708,-1.5708,0' in cpp


def test_regression_no_identity_fallback_warning_for_ee_mount_origin():
    text = Path('workcell_builder/workcell_builder/include/scene_xacro_parser.h').read_text(encoding='utf-8')
    assert 'defaulting to identity origin xyz/rpy (0 0 0)' not in text
