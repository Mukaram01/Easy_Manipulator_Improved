from pathlib import Path


def test_custom_stl_ui_and_primitive_types_exist():
    txt = Path('workcell_builder/workcell_builder/gui/addobject.cpp').read_text(encoding='utf-8')
    assert 'Create Custom STL / Create Primitive Object' in txt
    for t in ['box', 'table', 'bin/tray', 'conveyor_placeholder', 'fixture_plate']:
        assert t in txt


def test_generated_stl_writer_exists_and_markers_present():
    h = Path('workcell_builder/workcell_builder/include/generated_stl_writer.hpp').read_text(encoding='utf-8')
    cpp = Path('workcell_builder/workcell_builder/src_generated_stl_writer.cpp').read_text(encoding='utf-8')
    for needle in ['PrimitiveSpec', 'validate_primitive_spec', 'sanitize_object_name', 'write_ascii_stl', 'write_box_mesh', 'write_table_mesh', 'write_bin_mesh', 'write_fixture_plate_mesh', 'write_conveyor_placeholder_mesh']:
        assert needle in h or needle in cpp
    for marker in ['solid ', 'facet normal', 'vertex', 'endsolid']:
        assert marker in cpp


def test_generated_mesh_path_and_summary_preview_markers_exist():
    scene = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for needle in ['meshes/generated_objects/', 'custom_stl: bin_01', 'generated mesh: meshes/generated_objects/bin_01.stl']:
        assert needle in scene


def test_no_runtime_motion_apis_in_custom_stl_path_and_fake_hardware_guidance_present():
    txt = Path('workcell_builder/workcell_builder/src_generated_stl_writer.cpp').read_text(encoding='utf-8') + Path('workcell_builder/workcell_builder/gui/addobject.cpp').read_text(encoding='utf-8')
    for forbidden in ['GetMotionPlan', 'execute_trajectory', 'FollowJointTrajectory', '/plan_kinematic_path']:
        assert forbidden.lower() not in txt.lower()
    scene = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'use_fake_hardware:=true' in scene


def test_workspace_alias_policy_untouched():
    txt = Path('scripts/fix_workspace_layout.sh').read_text(encoding='utf-8')
    assert 'src/assets' in txt and 'src/scenes' in txt
