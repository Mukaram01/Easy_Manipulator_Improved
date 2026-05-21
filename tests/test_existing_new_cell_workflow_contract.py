from pathlib import Path

MAINWINDOW_CPP = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
DOC = Path("docs/manuals/WORKCELL_STUDIO_NEW_CELL_FROM_SCRATCH.md").read_text(encoding="utf-8")


def test_existing_new_cell_workflow_stage_contract_tokens_present():
    for token in [
        "Workspace -> New Cell -> Layout -> Task Intent -> Generate Scene Package -> Validate -> Plan & Simulate",
        "Cell basics: scene_name robot tool end_effector base_link tool_link robot_base_xyz_rpy tool_mount_xyz_rpy",
        "Layout: environment_asset primitive_fallback pick_zone place_zone camera_pose camera_fov conveyor_placeholder spawn_line",
        "Task intent: task_intent grasp_strategy top_grasp_2f suction_top approach_distance retract_distance",
        "Generate/Validate/Plan: generated_package_path validation_output plan_simulate_handoff fake_hardware_first",
    ]:
        assert token in MAINWINDOW_CPP


def test_new_cell_doc_generated_file_contract_and_validation_command():
    required = [
        "environment.yaml",
        "cell_definition.yaml",
        "scene_manifest.yaml",
        "layout/workcell_studio_layout.yaml",
        "task/workcell_builder_task_intent.yaml",
        "task/task_recipe_from_builder_intent.yaml",
        "plan_preview/offline_plan_preview_request.yaml",
        "launch/demo.launch.py",
        "python3 scripts/run_workcell_studio_scene_readiness_gate.py --dry-run-launches",
    ]
    for token in required:
        assert token in DOC


def test_fake_hardware_and_no_real_driver_tokens():
    forbidden = ["use_fake_hardware:=false", "fake_hardware:=false", "ur_robot_driver", "ethercat", "canopen"]
    assert "fake_hardware_first" in MAINWINDOW_CPP
    for token in forbidden:
        assert token not in DOC.lower()
