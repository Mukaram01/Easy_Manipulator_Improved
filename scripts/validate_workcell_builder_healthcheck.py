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
        repo_root / "workcell_builder/workcell_builder/CMakeLists.txt",
        repo_root / "workcell_builder/workcell_builder/package.xml",
        repo_root / "workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py",
        repo_root / "workcell_builder/workcell_builder/gui/asset_picker_dialog.cpp",
        repo_root / "workcell_builder/workcell_builder/src_asset_discovery_helper.cpp",
        repo_root / "workcell_builder/workcell_builder/gui/scene_select.cpp",
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

    scene_cpp = (repo_root / "workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")
    _check("demo.launch.py use_fake_hardware:=true" in scene_cpp, "guidance uses fake hardware default", errors)
    _check("demo.launch.py use_fake_hardware:=false" not in scene_cpp, "default guidance does not suggest real hardware", errors)

    for bad in ["unknown_description", "unknown_moveit_config", "none_moveit_config"]:
        _check(bad not in scene_cpp.lower(), f"forbidden placeholder text absent: {bad}", errors)

    for ui in ["Select Robot Asset", "Select End Effector Asset", "Select Existing STL"]:
        _check(ui in scene_cpp or ui in (repo_root / "workcell_builder/workcell_builder/gui/asset_picker_dialog.cpp").read_text(encoding="utf-8"), f"asset picker string present: {ui}", errors)

    for artifact in ["workcell_studio_summary.json", "workcell_studio_summary.md", "preview/workcell_preview.svg", "preview/workcell_preview.html"]:
        _check(artifact in scene_cpp, f"artifact string present: {artifact}", errors)

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
