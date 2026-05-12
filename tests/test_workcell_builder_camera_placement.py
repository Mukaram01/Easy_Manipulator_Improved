from pathlib import Path


def test_camera_model_files_exist():
    assert Path('workcell_builder/workcell_builder/include/workcell_camera_model.hpp').exists()
    assert Path('workcell_builder/workcell_builder/src_workcell_camera_model.cpp').exists()


def test_camera_metadata_tokens_present():
    text = Path('workcell_builder/workcell_builder/src_workcell_camera_model.cpp').read_text(encoding='utf-8') + Path('workcell_builder/workcell_builder/include/workcell_camera_model.hpp').read_text(encoding='utf-8')
    for token in ['realsense2_description', 'metadata_only', 'parent_object', 'parent_link', 'generate_camera_fixed_joint', 'generate_camera_xacro_include']:
        assert token in text


def test_scene_status_camera_checks_present():
    text = Path('workcell_builder/workcell_builder/src_workcell_scene_status.cpp').read_text(encoding='utf-8')
    for token in ['Camera configured', 'Camera package found', 'Parent mount object found', 'Parent mount link found', 'Runtime driver']:
        assert token in text


def test_bundle_mentions_camera_package_dependency():
    text = Path('workcell_builder/workcell_builder/src_workcell_scene_bundle.cpp').read_text(encoding='utf-8')
    assert 'required_camera_packages' in text
    assert 'realsense2_description' in text
