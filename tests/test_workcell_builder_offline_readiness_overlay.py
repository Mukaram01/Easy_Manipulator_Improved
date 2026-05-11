from pathlib import Path


def test_overlay_files_and_cmake_wiring():
    assert Path('workcell_builder/workcell_builder/include/offline_readiness_overlay.hpp').exists()
    assert Path('workcell_builder/workcell_builder/src_offline_readiness_overlay.cpp').exists()
    cmake = Path('workcell_builder/workcell_builder/CMakeLists.txt').read_text(encoding='utf-8')
    assert 'src_offline_readiness_overlay.cpp' in cmake


def test_overlay_status_and_helpers_present():
    txt = Path('workcell_builder/workcell_builder/src_offline_readiness_overlay.cpp').read_text(encoding='utf-8') + Path('workcell_builder/workcell_builder/include/offline_readiness_overlay.hpp').read_text(encoding='utf-8')
    for m in ['READY','WARN','BLOCKED','OUTSIDE_REACH','OUTSIDE_WORKSPACE','OVERLAP_WARNING','CAMERA_WARNING','SAFETY_ZONE_WARNING','estimate_robot_reach_envelope','evaluate_workspace_bounds','evaluate_simple_overlap_warnings','evaluate_camera_placement_warnings','evaluate_task_pick_place_reach']:
        assert m in txt


def test_visual_editor_overlay_strings_present():
    txt = Path('workcell_builder/workcell_builder/gui/environment_layout_editor.cpp').read_text(encoding='utf-8')
    for m in ['Readiness Overlay','Show Reach Envelope','Show Workspace Bounds','Show Safety Zones','Refresh Readiness Overlay']:
        assert m in txt


def test_schema_validator_and_summary_overlay_markers_present():
    assert 'workspace:' in Path('docs/manuals/WORKCELL_SCENE_SCHEMA_V1.md').read_text(encoding='utf-8')
    assert 'workspace' in Path('scripts/validate_workcell_scene.py').read_text(encoding='utf-8')
    scene = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for m in ['readiness_overlay_status','reach_warnings','workspace_warnings','overlap_warnings','camera_warnings','task_target_warnings']:
        assert m in scene
