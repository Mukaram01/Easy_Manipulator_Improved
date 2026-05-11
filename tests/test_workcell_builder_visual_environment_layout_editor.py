from pathlib import Path


def test_visual_layout_editor_files_and_cmake_wiring():
    h = Path('workcell_builder/workcell_builder/include/environment_layout_editor.hpp')
    cpp = Path('workcell_builder/workcell_builder/gui/environment_layout_editor.cpp')
    cmake = Path('workcell_builder/workcell_builder/CMakeLists.txt').read_text(encoding='utf-8')
    assert h.exists() and cpp.exists()
    assert 'gui/environment_layout_editor.cpp' in cmake


def test_visual_layout_editor_qt_markers_and_actions():
    txt = Path('workcell_builder/workcell_builder/gui/environment_layout_editor.cpp').read_text(encoding='utf-8')
    for needle in [
        'QGraphicsView', 'QGraphicsScene', 'Open Visual Layout Editor', 'Top-down Layout',
        'Snap to Grid', 'Save Layout to Environment YAML', 'Reload From Environment YAML'
    ]:
        assert needle in txt


def test_visual_layout_editor_model_and_conversion_markers():
    txt = Path('workcell_builder/workcell_builder/gui/environment_layout_editor.cpp').read_text(encoding='utf-8')
    hdr = Path('workcell_builder/workcell_builder/include/environment_layout_editor.hpp').read_text(encoding='utf-8')
    for needle in ['ObjectPlacementModel', 'PlacedObject', 'world_metres_to_canvas_pixels', 'canvas_pixels_to_world_metres', 'o.x', 'o.y']:
        assert needle in txt or needle in hdr


def test_scene_preview_summary_readiness_markers_for_visual_layout():
    txt = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for needle in ['visual_layout_editor_used', 'placed_object_count', 'placed_object_positions', 'Object table_01 @ x=', 'Save Layout to Environment YAML']:
        assert needle in txt


def test_no_pyyaml_or_forbidden_motion_apis_or_alias_policy_changes():
    pkg = Path('workcell_builder/workcell_builder/package.xml').read_text(encoding='utf-8')
    assert 'python3-yaml' not in pkg and 'PyYAML' not in pkg
    for forbidden in ['GetMotionPlan', 'execute_trajectory', 'FollowJointTrajectory', '/plan_kinematic_path']:
        assert forbidden.lower() not in Path('workcell_builder/workcell_builder/gui/environment_layout_editor.cpp').read_text(encoding='utf-8').lower()
    fix = Path('scripts/fix_workspace_layout.sh').read_text(encoding='utf-8')
    assert 'ensure_workspace_alias "assets"' in fix and 'ensure_workspace_alias "scenes"' in fix
