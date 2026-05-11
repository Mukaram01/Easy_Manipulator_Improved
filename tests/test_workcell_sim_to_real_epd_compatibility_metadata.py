from pathlib import Path

def txt(p): return Path(p).read_text(encoding='utf-8')

def test_helper_and_cmake_and_statuses_exist():
    assert Path('workcell_builder/workcell_builder/include/deployment_compatibility_profile.hpp').exists()
    assert Path('workcell_builder/workcell_builder/src_deployment_compatibility_profile.cpp').exists()
    cm = txt('workcell_builder/workcell_builder/CMakeLists.txt')
    assert 'src_deployment_compatibility_profile.cpp' in cm
    blob = txt('workcell_builder/workcell_builder/include/deployment_compatibility_profile.hpp')
    for s in ['SIM_READY','REAL_HARDWARE_METADATA_READY','REAL_HARDWARE_DRIVER_REQUIRED','REAL_HARDWARE_CONFIG_INCOMPLETE','EPD_METADATA_READY','EPD_METADATA_INCOMPLETE','UNSUPPORTED_ROBOT_FAMILY','DELTA_ROBOT_METADATA_ONLY']:
        assert s in blob or s in txt('workcell_builder/workcell_builder/src_deployment_compatibility_profile.cpp')

def test_profiles_schema_dashboard_and_separation_markers():
    profiles = txt('workcell_builder/workcell_builder/config/compatibility_profiles/robots/ur5.json') + txt('workcell_builder/workcell_builder/config/compatibility_profiles/tools/onrobot_airpick.json') + txt('workcell_builder/workcell_builder/config/camera_profiles/realsense_d435i.json')
    for k in ['robot_family','real_driver_required','driver_package_hint','io_required','io_type','calibration_required','real_camera_driver_required','epd_compatible','deployment_notes']:
        assert k in profiles
    d = txt('workcell_builder/workcell_builder/config/compatibility_profiles/robots/generic_delta_robot.json')
    assert 'delta_parallel' in d and 'metadata_only' in d and 'not launch-ready' in d.lower()
    assert 'deployment:' in txt('docs/manuals/WORKCELL_SCENE_SCHEMA_V1.md')
    assert 'deployment:' in txt('scripts/validate_workcell_scene.py')
    db = txt('workcell_builder/workcell_builder/src_validation_dashboard_model.cpp')
    for row in ['Simulation Readiness','Real Hardware Metadata','Robot Driver Requirements','Tool I/O Requirements','Camera Calibration Requirements','EPD Compatibility Metadata']:
        assert row in db
    full = '\n'.join([txt('scripts/validate_workcell_builder_healthcheck.py'), txt('scripts/validate_workcell_scene.py'), txt('workcell_builder/workcell_builder/src_validation_dashboard_model.cpp')]).lower()
    for forbidden in ['onnxruntime','import onnx','easy_perception_deployment run.launch.py','pip install ur']:
        assert forbidden not in full
