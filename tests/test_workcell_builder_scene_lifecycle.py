from pathlib import Path
HPP = Path('workcell_builder/workcell_builder/include/workcell_scene_status.hpp').read_text()
CPP = Path('workcell_builder/workcell_builder/src_workcell_scene_status.cpp').read_text()
GUI = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_lifecycle_states_and_snapshot_fields_present():
    for token in ['NO_SCENE_SELECTED','TEMPLATE_SELECTED','SCENE_SCAFFOLD','YAML_MISSING','YAML_INVALID_REPAIRABLE','YAML_READY','LAYOUT_READY','TASK_READY','VALIDATION_WARNINGS','VALIDATION_BLOCKED','GENERATED_PACKAGE_READY','BUILD_READY','LAUNCH_READY','LEGACY_SCENE_NEEDS_REPAIR']:
        assert token in HPP
    for field in ['label','severity','message','next_action','allow_generation','allow_bundle_export','launch_validation_meaningful']:
        assert field in HPP

def test_missing_launch_and_rviz_before_generation_not_error():
    assert 'Launch assets are not ready yet. This is expected before generation/launch validation.' in GUI
    assert 'demo.rviz missing. Next recommended action' in GUI

def test_generated_package_state_and_launch_blocker_modelled():
    assert 'Generated Package Ready' in CPP
    assert 'launch/demo.launch.py exists' in CPP
