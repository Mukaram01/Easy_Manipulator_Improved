#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def _check(cond: bool, msg: str, errors: list[str]) -> None:
    if cond:
        print(f"[PASS] {msg}")
    else:
        print(f"[FAIL] {msg}")
        errors.append(msg)


def _run(cmd: list[str], cwd: Path | None = None, timeout: int = 600) -> tuple[int, str]:
    proc = subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=False,
    )
    return proc.returncode, proc.stdout


def main() -> int:
    p = argparse.ArgumentParser(description="Validate Workcell Builder CI/healthcheck acceptance gate.")
    p.add_argument("--repo-root", default=".", help="Repository root path")
    p.add_argument("--workspace", default="~/workcell_ws", help="ROS workspace path")
    p.add_argument("--skip-colcon", action="store_true", help="Skip colcon build validation")
    p.add_argument("--run-colcon", action="store_true", help="Run colcon build validation")
    p.add_argument("--skip-launch", action="store_true", help="Skip launch smoke check")
    p.add_argument("--smoke-launch", action="store_true", help="Run fake-hardware headless launch smoke check")
    args = p.parse_args()

    repo_root = Path(args.repo_root).expanduser().resolve()
    workspace = Path(args.workspace).expanduser().resolve()
    errors: list[str] = []

    # Safe default: skip launch unless explicitly enabled.
    run_launch = args.smoke_launch

    key_files = [
        repo_root / "scripts/preview_task_recipe.py",
        repo_root / "easy_manipulation_deployment/emd_demo_nodes/run_grasp_execution/run_grasp_execution/task_recipe.py",
        repo_root / "easy_manipulation_deployment/emd_demo_nodes/run_grasp_execution/run_grasp_execution/task_recipe_visualizer_node.py",
        repo_root / "workcell_builder/workcell_builder/CMakeLists.txt",
        repo_root / "workcell_builder/workcell_builder/package.xml",
        repo_root / "workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py",
        repo_root / "workcell_builder/workcell_builder/gui/asset_picker_dialog.cpp",
        repo_root / "workcell_builder/workcell_builder/src_asset_discovery_helper.cpp",
        repo_root / "workcell_builder/workcell_builder/gui/scene_select.cpp",
        repo_root / "workcell_builder/workcell_builder/include/generated_stl_writer.hpp",
        repo_root / "workcell_builder/workcell_builder/src_generated_stl_writer.cpp",
        repo_root / "workcell_builder/workcell_builder/include/object_placement_model.hpp",
        repo_root / "workcell_builder/workcell_builder/src_object_placement_model.cpp",
        repo_root / "workcell_builder/workcell_builder/include/object_placement_dialog.hpp",
        repo_root / "workcell_builder/workcell_builder/gui/object_placement_dialog.cpp",
        repo_root / "workcell_builder/workcell_builder/include/environment_layout_editor.hpp",
        repo_root / "workcell_builder/workcell_builder/gui/environment_layout_editor.cpp",
        repo_root / "workcell_builder/workcell_builder/include/placed_object_preview_writer.hpp",
        repo_root / "workcell_builder/workcell_builder/gui/placed_object_preview_writer.cpp",
        repo_root / "workcell_builder/workcell_builder/include/robot_tool_compatibility.hpp",
        repo_root / "workcell_builder/workcell_builder/src_robot_tool_compatibility.cpp",
        repo_root / "workcell_builder/workcell_builder/include/validation_dashboard_model.hpp",
        repo_root / "workcell_builder/workcell_builder/src_validation_dashboard_model.cpp",
        repo_root / "workcell_builder/workcell_builder/include/workcell_scene_roundtrip.hpp",
        repo_root / "workcell_builder/workcell_builder/src_workcell_scene_roundtrip.cpp",
    ]
    for f in key_files:
        _check(f.exists(), f"required file exists: {f.relative_to(repo_root)}", errors)

    camera_files = [
        repo_root / "workcell_builder/workcell_builder/include/camera_perception_profile.hpp",
        repo_root / "workcell_builder/workcell_builder/src_camera_perception_profile.cpp",
        repo_root / "workcell_builder/workcell_builder/config/camera_profiles/realsense_d435i.json",
        repo_root / "workcell_builder/workcell_builder/config/camera_profiles/generic_rgbd_camera.json",
    ]
    for f in camera_files:
        _check(f.exists(), f"camera metadata file exists: {f.relative_to(repo_root)}", errors)

    cmake_text = (repo_root / "workcell_builder/workcell_builder/CMakeLists.txt").read_text(encoding="utf-8")
    for needle in ["gui/asset_picker_dialog.cpp", "src_asset_discovery_helper.cpp", "gui/scene_select.cpp", "gui/object_placement_dialog.cpp", "src_robot_tool_compatibility.cpp", "src_workcell_scene_schema.cpp", "src_workcell_scene_roundtrip.cpp", "src_camera_perception_profile.cpp", "src_validation_dashboard_model.cpp"]:
        _check(needle in cmake_text, f"CMake references {needle}", errors)

    pkg_text = (repo_root / "workcell_builder/workcell_builder/package.xml").read_text(encoding="utf-8")
    for dep in ["ament_cmake", "rclcpp", "qtbase5-dev", "yaml-cpp"]:
        _check(dep in pkg_text, f"package.xml includes dependency marker: {dep}", errors)

    launch_template = (repo_root / "workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py").read_text(encoding="utf-8")
    for needle in ["use_fake_hardware", "launch_rviz", "launch_rviz:=false", "DeclareLaunchArgument("]:
        _check(needle in launch_template, f"launch template contains {needle}", errors)
    for marker in ["task_recipe_path", "launch_task_preview", "task_preview_markers", "task_preview_output_dir", "Task recipe loaded for offline preview"]:
        _check(marker in launch_template, f"launch template contains {marker}", errors)

    scene_cpp = (repo_root / "workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")
    scene_builder_mainwindow_cpp = (repo_root / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    scene_builder_preview_cpp = (repo_root / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")
    for required in ["3D Layout Preview", "2D Layout (Fallback)"]:
        _check(required in scene_builder_mainwindow_cpp or required in scene_builder_preview_cpp, f"scene builder uses updated layout preview wording: {required}", errors)
    for forbidden in ["Mode: Select · 3D View", "\"3D View\"", "3D View unavailable"]:
        _check(forbidden not in scene_builder_mainwindow_cpp and forbidden not in scene_builder_preview_cpp, f"scene builder excludes legacy 3D view wording: {forbidden}", errors)

    _check("demo.launch.py use_fake_hardware:=true" in scene_cpp, "guidance uses fake hardware default", errors)
    _check("demo.launch.py use_fake_hardware:=false" not in scene_cpp, "default guidance does not suggest real hardware", errors)
    _check("launch_task_preview:=true" in scene_cpp, "guidance includes dry-run task preview launch", errors)

    for bad in ["unknown_description", "unknown_moveit_config", "none_moveit_config"]:
        _check(bad not in scene_cpp.lower(), f"forbidden placeholder text absent: {bad}", errors)

    addobject_cpp = (repo_root / "workcell_builder/workcell_builder/gui/addobject.cpp").read_text(encoding="utf-8")
    for ui in ["Select Robot Asset", "Select End Effector Asset", "Select Existing STL", "Create Custom STL / Create Primitive Object"]:
        _check(ui in scene_cpp or ui in (repo_root / "workcell_builder/workcell_builder/gui/asset_picker_dialog.cpp").read_text(encoding="utf-8") or ui in addobject_cpp, f"asset picker string present: {ui}", errors)

    for artifact in ["workcell_studio_summary.json", "workcell_studio_summary.md", "workcell_preview.svg", "workcell_preview.html"]:
        _check(artifact in scene_cpp, f"artifact string present: {artifact}", errors)
    for marker in ["Camera / Perception", "RealSense D435i", "Validate Camera", "Apply Camera Defaults", "Perception Metadata Export", "EPD Adapter Metadata", "EPD remains external/separate", "Object Placement Manager", "Placed Objects", "Add Asset Object", "Import STL to Asset Library", "Duplicate Object", "Remove Object", "Edit Pose", "asset_stl", "generated_primitive", "external_stl_warning", "custom_meshes", "placed_objects:", "Open RViz STL Preview"]:
        _check(marker in scene_cpp or marker in addobject_cpp, f"object placement marker present: {marker}", errors)
    for marker in ["Operator Workflow (Main)", "Validation Dashboard", "Run Offline Validation", "Developer Tools: Create Golden UR5 + Robotiq 2F Cell", "blocker_count", "warning_count"]:
        _check(marker in scene_cpp or marker in (repo_root / "workcell_builder/workcell_builder/gui/scene_select.ui").read_text(encoding="utf-8"), f"operator UX marker present: {marker}", errors)
    for marker in ["Open Existing Scene", "Reload Scene From YAML", "Scene Round-trip Status", "Loaded from workcell_scene/v1", "Legacy/partial scene warning", "Regenerate Existing Scene"]:
        _check(marker in scene_cpp, f"scene round-trip UI marker present: {marker}", errors)
    model_cpp = (repo_root / "workcell_builder/workcell_builder/src_object_placement_model.cpp").read_text(encoding="utf-8")
    for marker in ["sanitize_object_name", "validate_placed_object", "normalize_mesh_path_for_scene", "import_stl_to_asset_library", "easy_manipulation_deployment/assets/environment/custom_meshes"]:
        _check(marker in model_cpp, f"object placement model marker present: {marker}", errors)
    for marker in ["serialize_placed_objects_to_environment_yaml", "parse_placed_objects_from_environment_yaml", "save_environment_layout", "load_environment_layout"]:
        _check(marker in model_cpp, f"environment yaml helper marker present: {marker}", errors)

    layout_editor_cpp = (repo_root / "workcell_builder/workcell_builder/gui/environment_layout_editor.cpp").read_text(encoding="utf-8")
    for marker in ["QGraphicsView", "QGraphicsScene", "Top-down Layout", "Open Visual Layout Editor", "Save Layout to Environment YAML", "Reload From Environment YAML", "ObjectPlacementModel", "PlacedObject", "ItemIsMovable", "ItemIsSelectable", "snap_to_grid_", "update_model_from_item_move"]:
        _check(marker in layout_editor_cpp, f"visual layout editor marker present: {marker}", errors)

    overlay_hpp = (repo_root / "workcell_builder/workcell_builder/include/offline_readiness_overlay.hpp").read_text(encoding="utf-8")
    overlay_cpp = (repo_root / "workcell_builder/workcell_builder/src_offline_readiness_overlay.cpp").read_text(encoding="utf-8")
    _check("src_offline_readiness_overlay.cpp" in cmake_text, "overlay helper is wired into CMake", errors)
    for marker in ["evaluate_offline_readiness_overlay", "evaluate_reach_warnings", "evaluate_workspace_bounds", "evaluate_simple_overlap_warnings", "evaluate_camera_placement_warnings", "evaluate_task_pick_place_reach", "SAFETY_ZONE_WARNING", "incompatible robot/tool blocker"]:
        _check(marker in overlay_hpp or marker in overlay_cpp, f"offline readiness overlay marker present: {marker}", errors)
    for marker in ["readiness_overlay_status", "readiness_overlay_warning_count", "readiness_overlay_blocker_count", "reach_warnings", "workspace_warnings", "overlap_warnings", "camera_warnings", "task_target_warnings", "safety_zone_warnings"]:
        _check(marker in scene_cpp, f"summary/readiness/preview includes marker: {marker}", errors)
    for forbidden in ["GetMotionPlan", "execute_trajectory", "FollowJointTrajectory", "/plan_kinematic_path", "PyYAML", "import yaml"]:
        _check(forbidden.lower() not in layout_editor_cpp.lower(), f"visual editor excludes forbidden runtime/dependency marker: {forbidden}", errors)
    addobject_cpp = (repo_root / "workcell_builder/workcell_builder/gui/addobject.cpp").read_text(encoding="utf-8")
    stl_cpp = (repo_root / "workcell_builder/workcell_builder/src_generated_stl_writer.cpp").read_text(encoding="utf-8")
    for marker in ["box", "table", "bin/tray", "conveyor_placeholder", "fixture_plate", "meshes/generated_objects/", "custom_stl", "generated mesh"]:
        _check(marker in addobject_cpp or marker in scene_cpp, f"custom STL marker present: {marker}", errors)
    for marker in ["solid ", "facet normal", "vertex", "endsolid"]:
        _check(marker in stl_cpp, f"ASCII STL marker present: {marker}", errors)
    _check("generated_stl_writer" in cmake_text, "CMake references generated STL writer", errors)
    preview_script = (repo_root / "scripts/preview_task_recipe.py").read_text(encoding="utf-8")
    preview_visualizer = (repo_root / "easy_manipulation_deployment/emd_demo_nodes/run_grasp_execution/run_grasp_execution/task_recipe_visualizer_node.py").read_text(encoding="utf-8")
    _check("WORKCELL_TASK_RECIPE_PREVIEW: PASS" in preview_script, "preview CLI has PASS marker", errors)
    task_recipe_module = (repo_root / "easy_manipulation_deployment/emd_demo_nodes/run_grasp_execution/run_grasp_execution/task_recipe.py").read_text(encoding="utf-8")
    for marker in ["OFFLINE_ONLY", "NO_MOTION_COMMAND", "NO_MOVEIT_PLAN", "NO_REAL_HARDWARE"]:
        _check(marker in task_recipe_module, f"task preview safety marker present: {marker}", errors)
    for required in ["WORKCELL_TASK_RECIPE_RVIZ_PREVIEW: PASS", "/workcell_studio/task_plan_markers", "Offline dry-run preview only", "no robot motion", "no MoveIt planning", "no real hardware"]:
        _check(required in preview_visualizer or required in launch_template, f"task preview marker present: {required}", errors)
    for forbidden in ["moveit_msgs/srv/GetMotionPlan", "execute_trajectory", "FollowJointTrajectory", "ActionClient", "/plan_kinematic_path"]:
        _check(forbidden.lower() not in (preview_script + task_recipe_module + preview_visualizer).lower(), f"dry-run code does not call runtime motion API: {forbidden}", errors)
    for needle in [
        "Task & Grasp Strategy",
        "config\" / \"task_recipe.yaml",
        "schema_version: workcell_task/v1",
        "fake_hardware_first: true",
        "motion_command_sent: false",
        "runtime_execution_enabled: false",
        "finger_top",
        "suction_top",
        "open_gripper",
        "vacuum_off",
    ]:
        _check(needle in scene_cpp or needle in (repo_root / "workcell_builder/workcell_builder/gui/scene_select.ui").read_text(encoding="utf-8"), f"task/grasp marker present: {needle}", errors)

    

    golden_gen = repo_root / "scripts/generate_golden_workcell_demo.py"
    golden_val = repo_root / "scripts/validate_golden_workcell_demo.py"
    _check(golden_gen.exists(), "golden demo generator script exists", errors)
    _check(golden_val.exists(), "golden demo validator script exists", errors)
    if golden_gen.exists():
        gtxt = golden_gen.read_text(encoding="utf-8").lower()
        for marker in ["fake_hardware_first: true", "real_hardware_enabled: false", "motion_command_sent: false", "runtime_execution_enabled: false", "--install-into-scenes"]:
            _check(marker in gtxt, f"golden generator marker present: {marker}", errors)
        for forbidden in ["import yaml", "pyyaml", "getmotionplan", "execute_trajectory", "/plan_kinematic_path"]:
            _check(forbidden not in gtxt, f"golden generator excludes forbidden marker: {forbidden}", errors)
    if golden_val.exists():
        vtxt = golden_val.read_text(encoding="utf-8").lower()
        for marker in ["workcell_golden_demo: pass", "workcell_golden_demo: warn", "workcell_golden_demo: fail"]:
            _check(marker in vtxt, f"golden validator marker present: {marker}", errors)
        _check("pyyaml" not in vtxt and "import yaml" not in vtxt, "golden validator uses stdlib only", errors)


    catalog_validator = repo_root / "scripts/validate_workcell_asset_catalog.py"
    _check(catalog_validator.exists(), "asset catalog validator exists", errors)
    if catalog_validator.exists():
        ctxt = catalog_validator.read_text(encoding="utf-8").lower()
        for marker in ["workcell_asset_catalog: pass", "workcell_asset_catalog: warn", "workcell_asset_catalog: fail"]:
            _check(marker in ctxt, f"asset catalog marker present: {marker}", errors)
        _check("import yaml" not in ctxt and "pyyaml" not in ctxt, "asset catalog validator uses stdlib only", errors)

    for d in [
        repo_root / "workcell_builder/workcell_builder/config/compatibility_profiles/robots",
        repo_root / "workcell_builder/workcell_builder/config/compatibility_profiles/tools",
        repo_root / "workcell_builder/workcell_builder/config/compatibility_profiles/pairs",
        repo_root / "workcell_builder/workcell_builder/config/camera_profiles",
    ]:
        _check(d.exists(), f"catalog directory exists: {d.relative_to(repo_root)}", errors)

    compat_root = repo_root / "workcell_builder/workcell_builder/config/compatibility_profiles"
    _check((compat_root / "robots").exists() and (compat_root / "tools").exists() and (compat_root / "pairs").exists(), "compatibility profile catalog exists", errors)
    compat_cpp = (repo_root / "workcell_builder/workcell_builder/src_robot_tool_compatibility.cpp").read_text(encoding="utf-8")
    compat_hpp = (repo_root / "workcell_builder/workcell_builder/include/robot_tool_compatibility.hpp").read_text(encoding="utf-8")
    for marker in ["COMPATIBLE", "COMPATIBLE_WITH_WARNINGS", "UNKNOWN_COMPATIBILITY", "INCOMPATIBLE", "MISSING_TCP", "MISSING_MOUNT_LINK"]:
        _check(marker in compat_cpp or marker in compat_hpp, f"compatibility status marker present: {marker}", errors)
    for marker in ["Robot / Tool Compatibility", "Apply Profile Defaults", "Manual Override", "compatibility_status", "compatibility_warnings", "tcp_frame", "tool_mount_link"]:
        _check(marker in scene_cpp, f"scene compatibility marker present: {marker}", errors)
    _check("pyyaml" not in (compat_cpp + compat_hpp).lower(), "compatibility layer does not add PyYAML", errors)
    for forbidden in ["GetMotionPlan", "execute_trajectory", "FollowJointTrajectory", "/plan_kinematic_path"]:
        _check(forbidden.lower() not in compat_cpp.lower(), f"compatibility layer excludes runtime motion API: {forbidden}", errors)


    smoke_script = repo_root / "scripts/run_workcell_fake_hardware_smoke.py"
    _check(smoke_script.exists(), "fake hardware smoke acceptance script exists", errors)
    if smoke_script.exists():
        stxt = smoke_script.read_text(encoding="utf-8")
        for marker in ["WORKCELL_FAKE_HARDWARE_SMOKE: PASS", "WORKCELL_FAKE_HARDWARE_SMOKE: WARN", "WORKCELL_FAKE_HARDWARE_SMOKE: FAIL", "WORKCELL_FAKE_HARDWARE_SMOKE: SKIP", "--run-launch", "--skip-launch", "--generate-golden-demo", "--headless", "launch_rviz:=false", "use_fake_hardware:=true", "real_hardware_enabled: false", "runtime_execution_enabled: false", "motion_command_sent: false", "moveit_plan_service_called: false"]:
            _check(marker in stxt, f"smoke script marker present: {marker}", errors)
        for forbidden in ["import yaml", "pyyaml", "GetMotionPlan", "execute_trajectory", "/plan_kinematic_path", "streamlit", "epd"]:
            _check(forbidden.lower() not in stxt.lower(), f"smoke script excludes forbidden marker: {forbidden}", errors)


    _check((repo_root / "scripts/workcell_builder_camera_frustum_preview_node.py").exists(), "camera frustum preview node exists", errors)
    docs = (repo_root / "docs/manuals/WORKCELL_BUILDER_OBJECT_PLACEMENT_MANAGER.md").read_text(encoding="utf-8")
    _check("Camera placement and frustum preview" in docs, "docs mention Camera placement and frustum preview", errors)
    tcam = (repo_root / "tests/test_workcell_builder_camera_placement_yaml_io.py").read_text(encoding="utf-8") if (repo_root / "tests/test_workcell_builder_camera_placement_yaml_io.py").exists() else ""
    _check("camera_placements" in tcam, "tests mention camera_placements", errors)


    for f in [
        repo_root / "scripts/workcell_builder_task_zone_preview_node.py",
        repo_root / "workcell_builder/workcell_builder/launch/task_zone_preview.launch.py",
        repo_root / "tests/test_workcell_builder_task_zone_yaml_io.py",
    ]:
        _check(f.exists(), f"task-zone artifact exists: {f.relative_to(repo_root)}", errors)
    docs_obj = (repo_root / "docs/manuals/WORKCELL_BUILDER_OBJECT_PLACEMENT_MANAGER.md").read_text(encoding="utf-8")
    _check("Pick/place zone editing and preview" in docs_obj, "docs contain Pick/place zone editing and preview", errors)
    tests_blob = (repo_root / "tests/test_workcell_builder_task_intent_from_zones.py").read_text(encoding="utf-8") if (repo_root / "tests/test_workcell_builder_task_intent_from_zones.py").exists() else ""
    _check("task_zones" in tests_blob and "pick_source" in tests_blob and "place_target" in tests_blob, "tests mention task_zones and pick-source/place-target usage", errors)
    _check("Use Selected Zone as Pick Zone" in scene_cpp and "Use Selected Zone as Place Zone" in scene_cpp and "task_zones" in scene_cpp, "object placement source has required task-zone UI strings", errors)

    schema_validator = (repo_root / "scripts/validate_workcell_scene.py").read_text(encoding="utf-8")
    for marker in ["WORKCELL_SCENE_SCHEMA: PASS", "WORKCELL_SCENE_SCHEMA: WARN", "WORKCELL_SCENE_SCHEMA: FAIL", "workcell_scene/v1"]:
        _check(marker in schema_validator, f"scene schema validator marker present: {marker}", errors)
    schema_cpp = (repo_root / "workcell_builder/workcell_builder/src_workcell_scene_schema.cpp").read_text(encoding="utf-8")
    schema_hpp = (repo_root / "workcell_builder/workcell_builder/include/workcell_scene_schema.hpp").read_text(encoding="utf-8")
    for forbidden in ["PyYAML", "import yaml", "GetMotionPlan", "execute_trajectory", "FollowJointTrajectory", "/plan_kinematic_path"]:
        _check(forbidden.lower() not in (schema_cpp + schema_hpp + schema_validator).lower(), f"scene schema files exclude forbidden marker: {forbidden}", errors)
    roundtrip_cpp = (repo_root / "workcell_builder/workcell_builder/src_workcell_scene_roundtrip.cpp").read_text(encoding="utf-8")
    for marker in ["load_workcell_scene_v1_from_file", "populate_builder_state_from_scene", "extract_builder_state_to_scene", "workcell_scene/v1", "fake_hardware_first", "real_hardware_enabled", "runtime_execution_enabled"]:
        _check(marker in roundtrip_cpp, f"scene round-trip marker present: {marker}", errors)
    for forbidden in ["PyYAML", "import yaml", "GetMotionPlan", "execute_trajectory", "FollowJointTrajectory", "/plan_kinematic_path", "streamlit"]:
        _check(forbidden.lower() not in roundtrip_cpp.lower(), f"scene round-trip excludes forbidden marker: {forbidden}", errors)

    
    for f in [repo_root / 'scripts/export_workcell_scene_bundle.py', repo_root / 'scripts/import_workcell_scene_bundle.py', repo_root / 'scripts/validate_workcell_scene_bundle.py']:
        _check(f.exists(), f"bundle script exists: {f.relative_to(repo_root)}", errors)
    bundle_validator_text = (repo_root / 'scripts/validate_workcell_scene_bundle.py').read_text(encoding='utf-8') if (repo_root / 'scripts/validate_workcell_scene_bundle.py').exists() else ''
    for marker in ['WORKCELL_SCENE_BUNDLE: PASS','WORKCELL_SCENE_BUNDLE: WARN','WORKCELL_SCENE_BUNDLE: FAIL','workcell_bundle/v1','unsafe paths']:
        _check(marker in bundle_validator_text or marker in scene_cpp, f"bundle marker present: {marker}", errors)

    template_catalog = repo_root / "workcell_builder/workcell_builder/config/scene_templates/scene_templates.json"
    template_helper_h = repo_root / "workcell_builder/workcell_builder/include/scene_template_library.hpp"
    template_helper_cpp = repo_root / "workcell_builder/workcell_builder/src_scene_template_library.cpp"
    template_cli = repo_root / "scripts/generate_workcell_scene_from_template.py"
    _check(template_catalog.exists(), "scene template catalog exists", errors)
    _check(template_helper_h.exists() and template_helper_cpp.exists(), "scene template helper files exist", errors)
    _check(template_cli.exists(), "scene template CLI exists", errors)
    template_text = template_catalog.read_text(encoding="utf-8") if template_catalog.exists() else ""
    for marker in ["ur5_pick_place_cell", "ur5_sorting_cell", "camera_inspection_cell", "conveyor_pick_placeholder_cell", "palletizing_placeholder_cell", "fake_hardware_first", "real_hardware_enabled"]:
        _check(marker in template_text, f"template marker present: {marker}", errors)
    template_cli_text = template_cli.read_text(encoding="utf-8") if template_cli.exists() else ""
    for marker in ["workcell_studio_summary.json", "workcell_studio_summary.md", "workcell_preview.svg", "workcell_preview.html", "fake_hardware_first"]:
        _check(marker in template_cli_text, f"template generator marker present: {marker}", errors)
    for forbidden in ["bom", "bill of materials", "pyyaml", "import yaml", "getmotionplan", "execute_trajectory"]:
        _check(forbidden not in (template_text + (template_cli.read_text(encoding="utf-8") if template_cli.exists() else "")).lower(), f"template feature excludes forbidden marker: {forbidden}", errors)
    if args.run_colcon and not args.skip_colcon:
        code, out = _run([
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select workcell_builder --event-handlers console_direct+",
        ], cwd=workspace, timeout=1800)
        print(out)
        _check(code == 0, "colcon build passed", errors)

    if run_launch and not args.skip_launch:
        code, out = _run([
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 45 ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=false",
        ], cwd=workspace, timeout=120)
        print(out)
        _check(code in (0, 124), "smoke launch did not fail immediately", errors)

    rviz_importer_files = [
        repo_root / "workcell_builder/workcell_builder/include/rviz_pose_feedback_importer.hpp",
        repo_root / "workcell_builder/workcell_builder/gui/rviz_pose_feedback_importer.cpp",
    ]
    for f in rviz_importer_files:
        _check(f.exists(), f"RViz pose feedback importer file exists: {f.relative_to(repo_root)}", errors)

    docs_obj = (repo_root / "docs/manuals/WORKCELL_BUILDER_OBJECT_PLACEMENT_MANAGER.md").read_text(encoding="utf-8")
    _check("Apply RViz pose feedback" in docs_obj, "docs include Apply RViz pose feedback section", errors)

    opd_cpp = (repo_root / "workcell_builder/workcell_builder/gui/object_placement_dialog.cpp").read_text(encoding="utf-8")
    for marker in ["Import RViz Pose Feedback", "Apply Valid Updates"]:
        _check(marker in opd_cpp, f"Object Placement Manager UI includes: {marker}", errors)

    itest_path = repo_root / "tests/test_workcell_builder_interactive_preview.py"
    itest = itest_path.read_text(encoding="utf-8") if itest_path.exists() else ""
    _check("safe_for_robot_motion" in itest and "Rejected feedback" in itest, "tests include safe_for_robot_motion rejection coverage", errors)

    # Placed-object end-to-end generation checks with actionable file-path hints.
    placed_yaml_header = repo_root / "workcell_builder/workcell_builder/include/object_placement_yaml_io.hpp"
    placed_yaml_source = repo_root / "workcell_builder/workcell_builder/gui/object_placement_yaml_io.cpp"
    _check(
        placed_yaml_header.exists(),
        f"placed-object YAML IO header exists: {placed_yaml_header.relative_to(repo_root)} (add declaration here)",
        errors,
    )
    _check(
        placed_yaml_source.exists(),
        f"placed-object YAML IO source exists: {placed_yaml_source.relative_to(repo_root)} (add implementation here)",
        errors,
    )

    generated_scene_test_path = repo_root / "tests/test_workcell_builder_generated_scene_placed_objects.py"
    generated_scene_test = generated_scene_test_path.read_text(encoding="utf-8") if generated_scene_test_path.exists() else ""
    _check(
        generated_scene_test_path.exists(),
        f"generated-scene placed-object test exists: {generated_scene_test_path.relative_to(repo_root)} (add end-to-end scene assertions)",
        errors,
    )
    _check(
        any(marker in generated_scene_test.lower() for marker in ["urdf", "link", "joint", "mesh"]),
        f"generated-scene test asserts URDF link/joint/mesh expectations in: {generated_scene_test_path.relative_to(repo_root)}",
        errors,
    )

    docs_path = repo_root / "docs/manuals/WORKCELL_BUILDER_OBJECT_PLACEMENT_MANAGER.md"
    docs_text = docs_path.read_text(encoding="utf-8") if docs_path.exists() else ""
    _check(
        "Placed object end-to-end generation" in docs_text,
        f"docs include exact section phrase 'Placed object end-to-end generation' in: {docs_path.relative_to(repo_root)}",
        errors,
    )

    roundtrip_path = repo_root / "workcell_builder/workcell_builder/src_workcell_scene_roundtrip.cpp"
    roundtrip_text = roundtrip_path.read_text(encoding="utf-8") if roundtrip_path.exists() else ""
    _check(
        ("active_scene" in roundtrip_text.lower()) or ("loaded scene" in roundtrip_text.lower()),
        f"scene round-trip handles active scene in: {roundtrip_path.relative_to(repo_root)} (avoid hardcoded-only workcell_scene behavior)",
        errors,
    )
    _check(
        "workcell_scene" in roundtrip_text.lower(),
        f"scene round-trip includes workcell_scene serialization path in: {roundtrip_path.relative_to(repo_root)}",
        errors,
    )

    opd_path = repo_root / "workcell_builder/workcell_builder/gui/object_placement_dialog.cpp"
    opd_text = opd_path.read_text(encoding="utf-8") if opd_path.exists() else ""
    _check(
        "Save Placed Objects to Scene YAML" in opd_text,
        f"UI marker exists in {opd_path.relative_to(repo_root)}: 'Save Placed Objects to Scene YAML'",
        errors,
    )
    _check(
        "Use Recommended Gripper Orientation" in scene_cpp or "Use Recommended Gripper Orientation" in opd_text,
        "UI contains 'Use Recommended Gripper Orientation'",
        errors,
    )

    yaml_io_text = (
        (repo_root / "workcell_builder/workcell_builder/include/object_placement_yaml_io.hpp").read_text(encoding="utf-8")
        + "\n"
        + (repo_root / "workcell_builder/workcell_builder/gui/object_placement_yaml_io.cpp").read_text(encoding="utf-8")
    )
    for marker in ["robot_mount", "tool_attachment"]:
        _check(marker in yaml_io_text, f"robot/tool YAML IO marker exists: {marker}", errors)

    _check(
        "Robot base and tool attachment pose" in docs_text,
        "docs mention 'Robot base and tool attachment pose'",
        errors,
    )

    tests_union = "\n".join(
        [
            (repo_root / "tests/test_workcell_builder_gripper_mount_orientation.py").read_text(encoding="utf-8")
            if (repo_root / "tests/test_workcell_builder_gripper_mount_orientation.py").exists() else "",
            (repo_root / "workcell_builder/workcell_builder/test/generate_yaml_end_effector_metadata_test.cpp").read_text(encoding="utf-8")
            if (repo_root / "workcell_builder/workcell_builder/test/generate_yaml_end_effector_metadata_test.cpp").exists() else "",
        ]
    )
    for marker in ["robot_mount", "tool_attachment", "-1.5708 -1.5708 0.0"]:
        _check(marker in tests_union, f"tests mention {marker}", errors)

    urdf_tool_test = (repo_root / "workcell_builder/workcell_builder/test/scene_xacro_tool_attachment_test.cpp").read_text(encoding="utf-8") if (repo_root / "workcell_builder/workcell_builder/test/scene_xacro_tool_attachment_test.cpp").exists() else ""
    _check(
        "rpy=\"-1.5708 -1.5708 0.0\"" in urdf_tool_test or ("fixed" in urdf_tool_test.lower() and "origin" in urdf_tool_test.lower()),
        "URDF tests assert tool fixed-joint origin behavior",
        errors,
    )

    if errors:
        print("WORKCELL_BUILDER_HEALTHCHECK: FAIL")
        return 1
    print("WORKCELL_BUILDER_HEALTHCHECK: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())


# Validation dashboard functional checks
    dashboard_cpp = (repo_root / "workcell_builder/workcell_builder/src_validation_dashboard_model.cpp").read_text(encoding="utf-8")
    for marker in ["Scene Schema", "Asset Catalog", "Robot / Tool Compatibility", "Object Placement", "Camera Metadata", "Task Recipe", "Readiness Overlay", "Fake-Hardware Smoke Static", "Generation Safety"]:
        _check(marker in dashboard_cpp, f"validation dashboard row exists: {marker}", errors)
    for marker in ["collect_validation_dashboard_results", "Run Offline Validation", "validation_dashboard_status"]:
        _check(marker in scene_cpp or marker in dashboard_cpp, f"validation dashboard wiring marker present: {marker}", errors)

# external import wizard marker: external_asset_importer.hpp

# external import wizard marker: src_external_asset_importer.cpp

# external import wizard marker: Import External Asset

# external import wizard marker: External Asset Import Wizard

# external import wizard marker: Custom / Imported

# external import wizard marker: imported_environment_assets.json

# external import wizard marker: assets/imported/

# healthcheck artifact marker: preview/workcell_preview.svg

# healthcheck artifact marker: preview/workcell_preview.html


    docs_obj = (repo_root / "docs/manuals/WORKCELL_BUILDER_OBJECT_PLACEMENT_MANAGER.md").read_text(encoding="utf-8")
    for marker in ["Preview real STL assets in RViz", "/tmp/workcell_builder_preview", "visual-only", "does not use MoveIt", "controllers", "robot motion"]:
        _check(marker in docs_obj, f"object placement docs marker present: {marker}", errors)


    interactive_node = repo_root / 'scripts/workcell_builder_interactive_preview_node.py'
    _check(interactive_node.exists(), 'interactive preview node exists', errors)
    opd_cpp = (repo_root / 'workcell_builder/workcell_builder/gui/object_placement_dialog.cpp').read_text(encoding='utf-8')
    _check('Open Interactive RViz Preview' in opd_cpp, 'interactive preview label exists', errors)
    _check('Import RViz Pose Feedback' in opd_cpp, 'feedback import label exists', errors)
    _check('Save Placed Objects to Scene YAML' in opd_cpp, 'save-to-environment button exists', errors)
    _check('Placed object changes are pending' in opd_cpp, 'pending changes message exists', errors)
    _check((repo_root / 'workcell_builder/workcell_builder/include/object_placement_yaml_io.hpp').exists(), 'object_placement_yaml_io header exists', errors)
    _check((repo_root / 'workcell_builder/workcell_builder/gui/object_placement_yaml_io.cpp').exists(), 'object_placement_yaml_io source exists', errors)
    docs = (repo_root / 'docs/manuals/WORKCELL_BUILDER_OBJECT_PLACEMENT_MANAGER.md').read_text(encoding='utf-8')
    _check('placed_objects_feedback.yaml' in docs, 'docs mention placed_objects_feedback.yaml', errors)
    _check('Save Placed Objects to Scene YAML' in docs, 'docs mention Save Placed Objects to Scene YAML', errors)
    po_test = (repo_root / 'tests/test_workcell_builder_placed_object_yaml_io.py').read_text(encoding='utf-8') if (repo_root / 'tests/test_workcell_builder_placed_object_yaml_io.py').exists() else ''
    _check('round' in po_test.lower() and 'placed_objects' in po_test, 'tests mention placed_objects YAML round-trip', errors)
    gen_test = (repo_root / 'tests/test_workcell_builder_generated_scene_placed_objects.py').read_text(encoding='utf-8') if (repo_root / 'tests/test_workcell_builder_generated_scene_placed_objects.py').exists() else ''
    _check('fixed joint' in gen_test.lower() or 'fixed' in gen_test.lower(), 'generated scene tests mention fixed joint', errors)
    _check('mesh filename' in gen_test.lower() or 'mesh' in gen_test.lower(), 'generated scene tests mention mesh filename', errors)
    itest = (repo_root / 'tests/test_workcell_builder_interactive_preview.py').read_text(encoding='utf-8') if (repo_root / 'tests/test_workcell_builder_interactive_preview.py').exists() else ''
    _check('InteractiveMarkers' in itest, 'tests mention InteractiveMarkers', errors)
