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
        repo_root / "workcell_builder/workcell_builder/include/robot_tool_compatibility.hpp",
        repo_root / "workcell_builder/workcell_builder/src_robot_tool_compatibility.cpp",
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
    for needle in ["gui/asset_picker_dialog.cpp", "src_asset_discovery_helper.cpp", "gui/scene_select.cpp", "gui/object_placement_dialog.cpp", "src_robot_tool_compatibility.cpp", "src_workcell_scene_schema.cpp", "src_camera_perception_profile.cpp"]:
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
    _check("demo.launch.py use_fake_hardware:=true" in scene_cpp, "guidance uses fake hardware default", errors)
    _check("demo.launch.py use_fake_hardware:=false" not in scene_cpp, "default guidance does not suggest real hardware", errors)
    _check("launch_task_preview:=true" in scene_cpp, "guidance includes dry-run task preview launch", errors)

    for bad in ["unknown_description", "unknown_moveit_config", "none_moveit_config"]:
        _check(bad not in scene_cpp.lower(), f"forbidden placeholder text absent: {bad}", errors)

    addobject_cpp = (repo_root / "workcell_builder/workcell_builder/gui/addobject.cpp").read_text(encoding="utf-8")
    for ui in ["Select Robot Asset", "Select End Effector Asset", "Select Existing STL", "Create Custom STL / Create Primitive Object"]:
        _check(ui in scene_cpp or ui in (repo_root / "workcell_builder/workcell_builder/gui/asset_picker_dialog.cpp").read_text(encoding="utf-8") or ui in addobject_cpp, f"asset picker string present: {ui}", errors)

    for artifact in ["workcell_studio_summary.json", "workcell_studio_summary.md", "preview/workcell_preview.svg", "preview/workcell_preview.html"]:
        _check(artifact in scene_cpp, f"artifact string present: {artifact}", errors)
    for marker in ["Camera / Perception", "RealSense D435i", "Validate Camera", "Apply Camera Defaults", "Perception Metadata Export", "EPD Adapter Metadata", "EPD remains external/separate", "Object Placement Manager", "Placed Objects", "Add Asset Object", "Import STL to Asset Library", "Duplicate Object", "Remove Object", "Edit Pose", "asset_stl", "generated_primitive", "external_stl_warning", "custom_meshes", "placed_objects:"]:
        _check(marker in scene_cpp or marker in addobject_cpp, f"object placement marker present: {marker}", errors)
    model_cpp = (repo_root / "workcell_builder/workcell_builder/src_object_placement_model.cpp").read_text(encoding="utf-8")
    for marker in ["sanitize_object_name", "validate_placed_object", "normalize_mesh_path_for_scene", "import_stl_to_asset_library", "easy_manipulation_deployment/assets/environment/custom_meshes"]:
        _check(marker in model_cpp, f"object placement model marker present: {marker}", errors)
    for marker in ["serialize_placed_objects_to_environment_yaml", "parse_placed_objects_from_environment_yaml", "save_environment_layout", "load_environment_layout"]:
        _check(marker in model_cpp, f"environment yaml helper marker present: {marker}", errors)

    layout_editor_cpp = (repo_root / "workcell_builder/workcell_builder/gui/environment_layout_editor.cpp").read_text(encoding="utf-8")
    for marker in ["QGraphicsView", "QGraphicsScene", "Top-down Layout", "Open Visual Layout Editor", "Save Layout to Environment YAML", "Reload From Environment YAML", "ObjectPlacementModel", "PlacedObject", "ItemIsMovable", "ItemIsSelectable", "snap_to_grid_", "update_model_from_item_move"]:
        _check(marker in layout_editor_cpp, f"visual layout editor marker present: {marker}", errors)
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

    schema_validator = (repo_root / "scripts/validate_workcell_scene.py").read_text(encoding="utf-8")
    for marker in ["WORKCELL_SCENE_SCHEMA: PASS", "WORKCELL_SCENE_SCHEMA: WARN", "WORKCELL_SCENE_SCHEMA: FAIL", "workcell_scene/v1"]:
        _check(marker in schema_validator, f"scene schema validator marker present: {marker}", errors)
    schema_cpp = (repo_root / "workcell_builder/workcell_builder/src_workcell_scene_schema.cpp").read_text(encoding="utf-8")
    schema_hpp = (repo_root / "workcell_builder/workcell_builder/include/workcell_scene_schema.hpp").read_text(encoding="utf-8")
    for forbidden in ["PyYAML", "import yaml", "GetMotionPlan", "execute_trajectory", "FollowJointTrajectory", "/plan_kinematic_path"]:
        _check(forbidden.lower() not in (schema_cpp + schema_hpp + schema_validator).lower(), f"scene schema files exclude forbidden marker: {forbidden}", errors)

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

    if errors:
        print("WORKCELL_BUILDER_HEALTHCHECK: FAIL")
        return 1
    print("WORKCELL_BUILDER_HEALTHCHECK: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())

