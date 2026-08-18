from __future__ import annotations

import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

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


def test_controller_state_check_requires_active_state() -> None:
    states = {
        "joint_state_broadcaster": "active",
        "ur5_arm_controller": "active",
        "ur5_gripper_controller": "inactive",
    }
    inactive = v._inactive_controllers(states, v.CANONICAL_UR5_2F_CONTROLLERS)
    assert inactive == ["ur5_gripper_controller"]


def test_launched_process_accepts_ready_canonical_fake_runtime_and_always_terminates(monkeypatch: pytest.MonkeyPatch) -> None:
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
        if args[:3] == ["ros2", "node", "list"]:
            out = "/move_group\n/ur5_2f_test_robot_state_publisher\n/controller_manager\n"
        elif args[:3] == ["ros2", "param", "get"]:
            out = "String value is: loaded\n"
        elif args[:3] == ["ros2", "topic", "list"]:
            out = "/joint_states\n/tf\n/tf_static\n"
        elif args[:4] == ["ros2", "topic", "echo", "/tf_static"]:
            out = "transforms:\n- header:\n    frame_id: world\n  child_frame_id: base_link\n"
        else:
            out = ""
        return subprocess.CompletedProcess(args, 0, out, "")

    monkeypatch.setattr(v.subprocess, "run", fake_run)
    monkeypatch.setattr(
        v,
        "_query_controller_states",
        lambda timeout_sec: (
            0,
            {
                "joint_state_broadcaster": "active",
                "ur5_arm_controller": "active",
                "ur5_gripper_controller": "active",
            },
            "",
        ),
    )
    status, blockers, evidence, diagnostics = v.run_headless_smoke(
        "ur5_2f_test",
        "ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true",
        15,
    )
    assert status == v.PASS
    assert blockers == []
    assert "joint_state_broadcaster_active" in evidence
    assert "ur5_arm_controller_active" in evidence
    assert "ur5_gripper_controller_active" in evidence
    assert diagnostics["checks"]["controllers"]["transport"] == "rclpy_service"
    assert diagnostics["terminated"] is True
    assert killed


def test_canonical_ur5_2f_launch_declares_real_fake_controller_runtime() -> None:
    repo = Path(__file__).resolve().parents[1]
    scene = repo / "scenes" / "ur5_2f_test"
    launch_text = (scene / "launch" / "demo.launch.py").read_text(encoding="utf-8")
    controllers = yaml.safe_load((scene / "config" / "ros2_controllers.yaml").read_text(encoding="utf-8"))

    assert 'package="controller_manager"' in launch_text
    assert 'executable="ros2_control_node"' in launch_text
    assert 'condition=IfCondition(use_fake_hardware)' in launch_text
    assert '"joint_state_broadcaster", "--controller-manager", "/controller_manager"' in launch_text
    assert '"ur5_arm_controller", "--controller-manager", "/controller_manager"' in launch_text
    assert '"ur5_gripper_controller", "--controller-manager", "/controller_manager"' in launch_text
    assert '"allow_trajectory_execution": False' in launch_text

    manager = controllers["controller_manager"]["ros__parameters"]
    assert manager["joint_state_broadcaster"]["type"] == "joint_state_broadcaster/JointStateBroadcaster"
    assert manager["ur5_arm_controller"]["type"] == "joint_trajectory_controller/JointTrajectoryController"
    assert manager["ur5_gripper_controller"]["type"] == "joint_trajectory_controller/JointTrajectoryController"
    assert controllers["ur5_arm_controller"]["ros__parameters"]["command_interfaces"] == ["position"]
    assert controllers["ur5_gripper_controller"]["ros__parameters"]["command_interfaces"] == ["position"]


def test_canonical_ur5_2f_package_declares_fake_controller_dependencies() -> None:
    repo = Path(__file__).resolve().parents[1]
    package_text = (repo / "scenes" / "ur5_2f_test" / "package.xml").read_text(encoding="utf-8")
    for dependency in (
        "controller_manager",
        "joint_state_broadcaster",
        "joint_trajectory_controller",
        "hardware_interface",
    ):
        assert f"<exec_depend>{dependency}</exec_depend>" in package_text


def test_main_returns_nonzero_for_blocked_acceptance(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    args = SimpleNamespace(
        catalog=None,
        scene="ur5_2f_test",
        run_all=False,
        timeout_sec=1,
        headless=True,
        launch_rviz=False,
        json_output=tmp_path / "report.json",
        dry_run=False,
    )
    monkeypatch.setattr(v, "parse_args", lambda: args)
    monkeypatch.setattr(v, "load_targets", lambda *unused: [SimpleNamespace()])
    monkeypatch.setattr(
        v,
        "run_scene",
        lambda *unused: {
            "scene": "ur5_2f_test",
            "status": v.BLOCKED,
            "blockers": ["environment unavailable"],
            "warnings": [],
        },
    )

    assert v.main() == 1
