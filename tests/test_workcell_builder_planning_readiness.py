from pathlib import Path

def txt(p):
    return Path(p).read_text(encoding='utf-8')

def test_files_and_build_wiring_present():
    assert Path('workcell_builder/workcell_builder/include/planning_readiness.hpp').exists()
    assert Path('workcell_builder/workcell_builder/src_planning_readiness.cpp').exists()
    cm = txt('workcell_builder/workcell_builder/CMakeLists.txt')
    assert 'src_planning_readiness.cpp' in cm

def test_ui_static_markers_present():
    blob = txt('workcell_builder/workcell_builder/gui/scene_select.ui') + txt('workcell_builder/workcell_builder/gui/scene_select.cpp')
    for s in ['Check Planning Readiness','Generate Dry-Run Planning Request','Open Planning Readiness Report','dry_run_readiness_only']:
        assert s in blob

def test_serialization_safety_markers_present():
    src = txt('workcell_builder/workcell_builder/src_planning_readiness.cpp')
    for s in ['can_attempt_plan','can_execute','robot_motion_commanded','moveit_execute_called','gripper_command_sent']:
        assert s in src

def test_catalog_and_roadmap_mentions_present():
    cat = txt('catalog/scenarios/industrial_scenarios.yaml')
    assert 'dry_run_planning_check_supported' in cat
    assert 'real_hardware_ready: false' in cat
    m = txt('docs/roadmap/WORKCELL_STUDIO_CAPABILITY_MATRIX.md') + txt('docs/roadmap/WORKCELL_STUDIO_TODO.md')
    assert 'dry_run_planning_check' in m

def test_scene_status_has_planning_readiness_items():
    s = txt('workcell_builder/workcell_builder/src_workcell_scene_status.cpp')
    for q in ['Planning readiness report available','Dry-run request available','Execution disabled']:
        assert q in s
