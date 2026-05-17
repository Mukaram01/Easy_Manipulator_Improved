from pathlib import Path


def test_object_placement_manager_ui_strings_exist():
    txt = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8') + Path('workcell_builder/workcell_builder/gui/addobject.cpp').read_text(encoding='utf-8')
    for needle in [
        'Object Placement Manager',
        'Placed Objects',
        'Add Asset Object',
        'Import STL to Asset Library',
        'Duplicate Object',
        'Remove Object',
        'Edit Pose',
        'Refresh Preview',
        'Open RViz STL Preview',
    ]:
        assert needle in txt


def test_object_placement_helper_exists_and_import_path_policy_present():
    h = Path('workcell_builder/workcell_builder/include/object_placement_model.hpp').read_text(encoding='utf-8')
    cpp = Path('workcell_builder/workcell_builder/src_object_placement_model.cpp').read_text(encoding='utf-8')
    for marker in ['PlacedObject', 'ObjectPlacementModel', 'sanitize_object_name', 'validate_placed_object', 'normalize_mesh_path_for_scene', 'import_stl_to_asset_library']:
        assert marker in h or marker in cpp
    assert 'easy_manipulation_deployment/assets/environment/custom_meshes' in h or 'easy_manipulation_deployment/assets/environment/custom_meshes' in cpp


def test_placed_objects_and_generated_primitive_markers_in_generation_outputs():
    scene = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for marker in ['placed_objects:', 'asset_stl', 'generated_primitive', 'external_stl_warning', 'meshes/generated_objects/']:
        assert marker in scene


def test_no_new_symlink_policy_or_motion_api_in_object_placement_path_and_fake_hardware_guidance():
    model = Path('workcell_builder/workcell_builder/src_object_placement_model.cpp').read_text(encoding='utf-8')
    for forbidden in ['GetMotionPlan', 'execute_trajectory', 'FollowJointTrajectory', '/plan_kinematic_path']:
        assert forbidden.lower() not in model.lower()
    fix = Path('scripts/fix_workspace_layout.sh').read_text(encoding='utf-8')
    assert 'ensure_workspace_alias "assets"' in fix and 'ensure_workspace_alias "scenes"' in fix
    scene = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'use_fake_hardware:=true' in scene
