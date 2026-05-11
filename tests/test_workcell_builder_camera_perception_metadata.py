from pathlib import Path


def test_camera_helper_files_and_cmake_wiring_exist():
    assert Path('workcell_builder/workcell_builder/include/camera_perception_profile.hpp').exists()
    assert Path('workcell_builder/workcell_builder/src_camera_perception_profile.cpp').exists()
    assert 'src_camera_perception_profile.cpp' in Path('workcell_builder/workcell_builder/CMakeLists.txt').read_text(encoding='utf-8')


def test_camera_catalog_and_strings_and_epd_separation_markers_exist():
    assert Path('workcell_builder/workcell_builder/config/camera_profiles/realsense_d435i.json').exists()
    assert Path('workcell_builder/workcell_builder/config/camera_profiles/generic_rgbd_camera.json').exists()
    scene = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for needle in [
        'Camera / Perception', 'RealSense D435i', 'Validate Camera', 'Apply Camera Defaults',
        'Perception Metadata Export', 'EPD Adapter Metadata', 'EPD remains external/separate'
    ]:
        assert needle in scene


def test_schema_validator_preview_and_export_markers_exist():
    docs = Path('docs/manuals/WORKCELL_SCENE_SCHEMA_V1.md').read_text(encoding='utf-8')
    assert 'camera:' in docs and 'optical_frame_id' in docs and 'epd_input_hint' in docs
    val = Path('scripts/validate_workcell_scene.py').read_text(encoding='utf-8')
    for m in ['camera_id must be non-empty', 'camera pose must be six finite numbers', 'missing pointcloud topic', 'missing rgb/depth topics']:
        assert m in val
    assert Path('workcell_builder/workcell_builder/config/epd_adapter_metadata.json').exists()
