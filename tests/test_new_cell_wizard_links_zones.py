from pathlib import Path

CPP=Path('workcell_builder/workcell_builder/gui/new_cell_wizard.cpp').read_text(encoding='utf-8')
H=Path('workcell_builder/workcell_builder/gui/new_cell_wizard.h').read_text(encoding='utf-8')

def test_robot_link_defaults_present():
    for t in ['default_robot_base_link','default_robot_tip_link','UR5','UR10','UR3','manipulator']:
        assert t in CPP or t in H

def test_gripper_defaults_present():
    for t in ['default_gripper_rpy_text','-1.5708, -1.5708, 0','default_end_effector_attach_link']:
        assert t in CPP

def test_environment_table_parent_child_and_roles():
    for t in ['Parent Object','Parent Link','Child Link','Joint Type','Semantic Role','workbench_01','source_bin_01','place_fixture_01','camera_01']:
        assert t in CPP

def test_pick_zone_camera_default_and_place_target_filtering():
    for t in ['Camera view zone','Pick zone is defined by the selected camera view','pick_camera_','place_target_','reject_target']:
        assert t in CPP

def test_readiness_and_package_validation_tokens():
    for t in ['READY','WARNINGS','BLOCKED','is_valid_package_name']:
        assert t in CPP
