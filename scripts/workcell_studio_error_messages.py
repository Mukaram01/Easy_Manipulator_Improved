#!/usr/bin/env python3
from __future__ import annotations
from typing import Any

STANDARD_FIELDS = [
    "code","severity","title","detail","why_it_matters","next_action",
    "recovery_command","related_file","related_page",
]


def make_message(code:str,severity:str,title:str,detail:str,why:str,next_action:str,recovery_command:str="",related_file:str="",related_page:str="New Cell") -> dict[str, Any]:
    return {
        "code": code,
        "severity": severity,
        "title": title,
        "detail": detail,
        "why_it_matters": why,
        "next_action": next_action,
        "recovery_command": recovery_command,
        "related_file": related_file,
        "related_page": related_page,
    }


MESSAGES = {
    "MISSING_WORKSPACE": make_message("MISSING_WORKSPACE","BLOCKER","Workspace not selected","No workspace path was provided or it does not exist.","New Cell outputs cannot be created without a valid workspace.","Select a valid workspace folder and re-run the action.","python3 scripts/generate_scratch_cell_acceptance.py --scene-name <scene> --output-root <workspace>","<workspace>","Dashboard > New Cell"),
    "INVALID_SCENE_NAME": make_message("INVALID_SCENE_NAME","ERROR","Scene name is not ROS-safe","Scene name must be lowercase and contain only letters, numbers, and underscores.","Invalid names break package generation and ROS package discovery.","Rename the scene using lowercase snake_case.","python3 scripts/generate_scratch_cell_acceptance.py --scene-name scratch_ur5_2f --output-root <workspace>","scene name","Dashboard > New Cell"),
    "SCENE_ALREADY_EXISTS": make_message("SCENE_ALREADY_EXISTS","WARNING","Scene folder already exists","A scene with the same name already exists in the target workspace.","Overwriting can destroy existing scene files.","Use a unique scene name or let the workflow create a safe suffix.","python3 scripts/generate_scratch_cell_acceptance.py --scene-name <unique_scene> --output-root <workspace>","<workspace>/<scene>","Dashboard > New Cell"),
    "MISSING_ENVIRONMENT_LAYOUT": make_message("MISSING_ENVIRONMENT_LAYOUT","BLOCKER","Missing environment_layout.yaml","environment_layout.yaml is not present for the selected scene.","Task intent binding and validation require a saved layout.","Open Scene Builder and click Save Layout.","python3 scripts/validate_environment_layout.py <scene>/environment_layout.yaml --json","environment_layout.yaml","Scene Builder"),
    "MISSING_SAVED_LAYOUT": make_message("MISSING_SAVED_LAYOUT","BLOCKER","Missing saved layout","layout/workcell_studio_layout.yaml is not present or valid for the selected modern scene.","Task intent binding and validation require a saved authored layout.","Open Scene Builder and click Save Layout.","Open and save the scene in Workcell Builder","layout/workcell_studio_layout.yaml","Scene Builder"),
    "MALFORMED_ENVIRONMENT_LAYOUT": make_message("MALFORMED_ENVIRONMENT_LAYOUT","ERROR","Malformed environment_layout.yaml","The layout file exists but cannot be parsed or required keys are missing.","Validation and pick/place binding cannot resolve layout IDs.","Fix YAML syntax and required fields, then run layout validation.","python3 scripts/validate_environment_layout.py <scene>/environment_layout.yaml --json","environment_layout.yaml","Scene Builder"),
    "MISSING_TASK_INTENT": make_message("MISSING_TASK_INTENT","BLOCKER","Missing task intent","config/workcell_builder_task_intent.yaml is missing.","Plan generation needs pick/place intent bindings.","Click Generate/Update Task Intent in New Cell checklist.","python3 scripts/create_or_update_builder_task_intent.py --scene-dir <scene>","config/workcell_builder_task_intent.yaml","Task Intent"),
    "TASK_PICK_PLACE_NOT_IN_LAYOUT": make_message("TASK_PICK_PLACE_NOT_IN_LAYOUT","ERROR","Task intent references unknown layout ID","Task intent pick/place IDs were not found in the saved canonical layout.","Planner cannot resolve source/target zones.","Bind pick/place zones in Scene Builder, then regenerate task intent.","python3 scripts/create_or_update_builder_task_intent.py --scene-dir <scene>","config/workcell_builder_task_intent.yaml","Scene Builder"),
    "MISSING_PACKAGE_XML": make_message("MISSING_PACKAGE_XML","BLOCKER","Missing package.xml","Generated scene package is missing package.xml.","ROS cannot discover or build the scene package.","Run Generate Scene Package and verify package output folder.","python3 scripts/generate_workcell_from_cell_definition.py <scene>/cell_definition.yaml --output-dir <workspace> --package-name <scene> --force","package.xml","Generate Scene Package"),
    "MISSING_CMAKELISTS": make_message("MISSING_CMAKELISTS","BLOCKER","Missing CMakeLists.txt","Generated scene package is missing CMakeLists.txt.","colcon build cannot compile or index the package.","Run Generate Scene Package again and inspect generation logs.","python3 scripts/generate_workcell_from_cell_definition.py <scene>/cell_definition.yaml --output-dir <workspace> --package-name <scene> --force","CMakeLists.txt","Generate Scene Package"),
    "MISSING_DEMO_LAUNCH": make_message("MISSING_DEMO_LAUNCH","BLOCKER","Missing launch/demo.launch.py","launch/demo.launch.py was not generated.","Plan & Simulate requires the demo launch entry point.","Generate Scene Package before opening Plan & Simulate.","python3 scripts/audit_new_cell_file_outputs.py --scene-dir <scene> --scene-name <scene_name> --json-out <scene>/file_output_audit.json","launch/demo.launch.py","Plan & Simulate"),
    "MISSING_FAKE_HARDWARE_ARG": make_message("MISSING_FAKE_HARDWARE_ARG","ERROR","Missing fake-hardware launch argument","Launch command does not include use_fake_hardware:=true.","Safety policy requires fake hardware mode for this workflow.","Update launch invocation to include use_fake_hardware:=true.","ros2 launch <scene> demo.launch.py use_fake_hardware:=true launch_rviz:=true","launch/demo.launch.py","Plan & Simulate"),
    "MISSING_ROS2": make_message("MISSING_ROS2","BLOCKER","ROS 2 CLI not found","The ros2 command is not available in PATH.","Build, package discovery, and launch checks require ROS 2.","Install/source ROS 2 Humble and reopen the terminal.","source /opt/ros/humble/setup.bash","shell PATH","Plan & Simulate"),
    "MISSING_COLCON": make_message("MISSING_COLCON","BLOCKER","colcon not found","The colcon command is not available in PATH.","Scene package smoke build cannot run without colcon.","Install colcon tools and retry the smoke audit.","python3 -m pip install -U colcon-common-extensions","shell PATH","Plan & Simulate"),
    "MISSING_WORKSPACE_SRC": make_message("MISSING_WORKSPACE_SRC","BLOCKER","Workspace src/ missing","The selected workspace does not have a src/ folder.","ROS 2 packages must live under workspace/src to be built and discovered.","Create or select a valid ROS 2 workspace with src/.","mkdir -p <workspace>/src","<workspace>/src","Dashboard > New Cell"),
    "COLCON_BUILD_FAILED": make_message("COLCON_BUILD_FAILED","BLOCKER","colcon build failed","The scene package failed to build in the selected workspace.","A failed build blocks package discovery and launch checks.","Review the build tail, fix blockers, then rerun smoke test.","colcon build --symlink-install --packages-select <scene_name>","build log","Generate Scene Package"),
    "ROS_PACKAGE_NOT_DISCOVERABLE": make_message("ROS_PACKAGE_NOT_DISCOVERABLE","BLOCKER","ROS package not discoverable","ros2 pkg prefix could not find the generated scene package.","If package discovery fails, launch commands cannot resolve the package.","Source workspace install/setup.bash and verify package install.","source <workspace>/install/setup.bash && ros2 pkg prefix <scene_name>","install/setup.bash","Plan & Simulate"),
    "LAUNCH_SHOW_ARGS_FAILED": make_message("LAUNCH_SHOW_ARGS_FAILED","BLOCKER","Launch argument inspection failed","ros2 launch --show-args failed for demo.launch.py.","Launch argument parsing must work before fake-hardware smoke launch.","Fix launch/package issues, then rerun the smoke audit.","ros2 launch <scene_name> demo.launch.py --show-args","launch/demo.launch.py","Plan & Simulate"),
    "LAUNCH_SMOKE_FAILED": make_message("LAUNCH_SMOKE_FAILED","BLOCKER","Fake-hardware launch smoke failed","Short launch smoke with fake hardware did not complete successfully.","Plan & Simulate readiness requires a passing fake-hardware smoke launch.","Inspect launch logs and retry with fake hardware only.","ros2 launch <scene_name> demo.launch.py use_fake_hardware:=true launch_rviz:=false","launch/demo.launch.py","Plan & Simulate"),
    "VALIDATION_BLOCKED": make_message("VALIDATION_BLOCKED","BLOCKER","Validation blocked","One or more blockers prevent offline validation from passing.","The scene is not ready for Plan & Simulate.","Resolve the first blocker, then rerun file-output and state audits.","python3 scripts/audit_new_cell_state_transitions.py --scene-dir <scene> --scene-name <scene_name> --json-out <scene>/state_transition_audit.json","file_output_audit.json","Validate"),
}


def get_message(code:str, **overrides: Any) -> dict[str, Any]:
    base = dict(MESSAGES[code])
    base.update({k: v for k, v in overrides.items() if v is not None})
    return base
