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
    ]
    for f in key_files:
        _check(f.exists(), f"required file exists: {f.relative_to(repo_root)}", errors)

    cmake_text = (repo_root / "workcell_builder/workcell_builder/CMakeLists.txt").read_text(encoding="utf-8")
    for needle in ["gui/asset_picker_dialog.cpp", "src_asset_discovery_helper.cpp", "gui/scene_select.cpp"]:
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
    for marker in ["Object Placement Manager", "Placed Objects", "Add Asset Object", "Import STL to Asset Library", "Duplicate Object", "Remove Object", "Edit Pose", "asset_stl", "generated_primitive", "external_stl_warning", "custom_meshes", "placed_objects:"]:
        _check(marker in scene_cpp or marker in addobject_cpp, f"object placement marker present: {marker}", errors)
    model_cpp = (repo_root / "workcell_builder/workcell_builder/src_object_placement_model.cpp").read_text(encoding="utf-8")
    for marker in ["sanitize_object_name", "validate_placed_object", "normalize_mesh_path_for_scene", "import_stl_to_asset_library", "easy_manipulation_deployment/assets/environment/custom_meshes"]:
        _check(marker in model_cpp, f"object placement model marker present: {marker}", errors)
    
        _check(artifact in scene_cpp, f"artifact string present: {artifact}", errors)
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
