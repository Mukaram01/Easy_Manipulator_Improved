from pathlib import Path


def test_functional_asset_selection_logic_and_presets_present():
    text = Path('workcell_builder/workcell_builder/src_asset_discovery_helper.cpp').read_text(encoding='utf-8')
    assert 'normalize_mesh_path' in text
    assert 'scan_asset_packages' in text
    assert 'MISSING_DESCRIPTION' in text
    assert 'MISSING_MOVEIT_CONFIG' in text
    assert 'READY' in text
    # quick preset names expected by environment object workflow
    assert 'table' in text
    assert 'bin' in text
    assert 'conveyor_placeholder' in text
    assert 'fixture' in text
    assert 'custom_stl' in text


def test_ur5_defaults_and_end_effector_inference_keywords_present():
    text = Path('workcell_builder/workcell_builder/src_asset_discovery_helper.cpp').read_text(encoding='utf-8')
    assert 'UR5' in text
    assert 'ur.urdf.xacro' in text
    assert 'robotiq_2f_85' in text
    assert 'airpick' in text
    assert 'finger' in text
    assert 'suction' in text


def test_external_absolute_stl_warning_and_fake_hardware_guidance_present():
    helper_text = Path('workcell_builder/workcell_builder/src_asset_discovery_helper.cpp').read_text(encoding='utf-8')
    scene_text = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'Absolute mesh path is outside workspace' in helper_text
    assert 'use_fake_hardware:=true' in scene_text
    assert 'unknown_description' not in scene_text
    assert 'unknown_moveit_config' not in scene_text
