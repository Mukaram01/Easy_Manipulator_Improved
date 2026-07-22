from __future__ import annotations

import json
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import validate_rviz_moveit_simulation_launches as v


def _scene(tmp_path: Path, *, fake_default: str = "true", srdf_joint: str = "wrist_3_joint", tool: str = "tool0") -> tuple[SimpleNamespace, Path, Path]:
    root = tmp_path / "repo"
    scene = root / "scenes" / "ur5_2f_test"
    (scene / "launch").mkdir(parents=True)
    (scene / "urdf").mkdir()
    launch = scene / "launch" / "demo.launch.py"
    launch.write_text(
        f'''
from launch.actions import DeclareLaunchArgument
planning_group = "manipulator"
robot_base_link = "base_link"
tool_mount_link = "{tool}"
def generate_launch_description():
    return [DeclareLaunchArgument("use_fake_hardware", default_value="{fake_default}")]
moveit_simple_controller_manager = {{"controller_names": ["ur5_arm_controller"], "ur5_arm_controller": {{"joints": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]}}}}
trajectory_execution = {{"allow_trajectory_execution": False}}
''',
        encoding="utf-8",
    )
    (scene / "scene_manifest.yaml").write_text(
        f"robot:\n  planning_group: manipulator\n  base_frame: base_link\nend_effector:\n  grasp_frame: {tool}\n",
        encoding="utf-8",
    )
    (scene / "cell_definition.yaml").write_text(
        f"robot:\n  planning_group: manipulator\n  base_frame: base_link\n  tool_link: {tool}\nend_effector:\n  grasp_frame: {tool}\n",
        encoding="utf-8",
    )
    (scene / "urdf" / "scene.urdf.xacro").write_text(f"<robot><link name='base_link'/><link name='{tool}'/></robot>", encoding="utf-8")
    (scene / "urdf" / "arm_hand.srdf.xacro").write_text(
        f"<robot><group name='manipulator'><joint name='shoulder_pan_joint'/><joint name='shoulder_lift_joint'/><joint name='elbow_joint'/><joint name='wrist_1_joint'/><joint name='wrist_2_joint'/><joint name='{srdf_joint}'/></group><end_effector parent_link='{tool}'/></robot>",
        encoding="utf-8",
    )
    entry = SimpleNamespace(
        scene_name="ur5_2f_test",
        scene_path="scenes/ur5_2f_test",
        fake_hardware_launch_command="ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=false",
        required_capabilities=("fake_hardware_launch",),
        enabled=True,
    )
    return entry, scene, launch


def test_fake_hardware_default_accepted(tmp_path: Path) -> None:
    entry, scene, launch = _scene(tmp_path)
    failures, evidence, _ = v.validate_launch_contract(entry, scene, launch, False)
    assert failures == []
    assert "fake_hardware_default_true" in evidence


def test_real_hardware_default_rejected(tmp_path: Path) -> None:
    entry, scene, launch = _scene(tmp_path, fake_default="false")
    failures, _, _ = v.validate_launch_contract(entry, scene, launch, False)
    assert any("use_fake_hardware default must be true" in f for f in failures)


def test_robot_srdf_joint_mismatch_fails(tmp_path: Path) -> None:
    entry, scene, launch = _scene(tmp_path, srdf_joint="wrong_joint")
    failures, _, _ = v.validate_scene_model(entry, scene, launch)
    assert any("robot/SRDF joint mismatch" in f for f in failures)


def test_missing_tool_planning_metadata_fails(tmp_path: Path) -> None:
    entry, scene, launch = _scene(tmp_path, tool="missing_tool0")
    (scene / "scene_manifest.yaml").write_text("robot:\n  planning_group: manipulator\n  base_frame: base_link\nend_effector: {}\n", encoding="utf-8")
    (scene / "cell_definition.yaml").write_text("robot:\n  planning_group: manipulator\n  base_frame: base_link\nend_effector: {}\n", encoding="utf-8")
    failures, _, _ = v.validate_scene_model(entry, scene, launch)
    assert "missing tool planning metadata" in "\n".join(failures)


def test_unavailable_ros_environment_reports_blocked(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(v.shutil, "which", lambda name: None)
    status, blockers, _, diagnostics = v.run_headless_smoke("ur5_2f_test", "ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true", 1)
    assert status == v.BLOCKED
    assert "ros2 executable not found" in blockers[0]
    assert diagnostics["terminated"] is True


def test_launched_process_always_terminates(monkeypatch: pytest.MonkeyPatch) -> None:
    class DummyProc:
        pid = 12345
        def __init__(self, *args, **kwargs):
            self.stdout = None
            self.stderr = None
        def communicate(self, timeout=None):
            return "", ""
    monkeypatch.setattr(v.shutil, "which", lambda name: "/usr/bin/ros2")
    monkeypatch.setattr(v.subprocess, "Popen", DummyProc)
    monkeypatch.setattr(v.os, "getpgid", lambda pid: pid)
    killed = []
    monkeypatch.setattr(v.os, "killpg", lambda pgid, sig: killed.append((pgid, sig)))
    def fake_run(args, capture_output, text, timeout):
        return subprocess.CompletedProcess(args, 0, "/move_group\n/robot_state_publisher\n/joint_states\n", "")
    monkeypatch.setattr(v.subprocess, "run", fake_run)
    status, _, _, diagnostics = v.run_headless_smoke("ur5_2f_test", "ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true", 3)
    assert diagnostics["terminated"] is True
    assert killed
