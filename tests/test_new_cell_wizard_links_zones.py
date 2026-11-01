from pathlib import Path

CPP=Path('workcell_builder/workcell_builder/gui/new_cell_wizard.cpp').read_text(encoding='utf-8')
H=Path('workcell_builder/workcell_builder/gui/new_cell_wizard.h').read_text(encoding='utf-8')

def test_robot_and_tool_choices_are_discovered_not_embedded():
    # Equipment, frames and defaults come from instantiate_workcell_studio_profile.py --equipment.
    assert '"--equipment"' in CPP and 'equipment_combination' in H
    for vendor in ['"UR5"', '"UR10"', '"robotiq_85"', '"Franka / Panda"', '"Robotiq"', '"ee_palm"', '"manipulator"',
                   '"scaled_joint_trajectory_controller"', '-1.5708', '"ur5_2f_workbench"']:
        assert vendor not in CPP, vendor

def test_environment_table_parent_child_and_roles():
    for t in ['Parent Object','Parent Link','Child Link','Joint Type','Semantic Role','workbench_01','source_bin_01','place_fixture_01','camera_01']:
        assert t in CPP

def test_pick_zone_camera_default_and_place_target_filtering():
    for t in ['Camera view zone','Pick zone is defined by the selected camera view','pick_camera_','place_target_','reject_target']:
        assert t in CPP

def test_readiness_and_package_validation_tokens():
    for t in ['READY','WARNINGS','BLOCKED','is_valid_package_name']:
        assert t in CPP
