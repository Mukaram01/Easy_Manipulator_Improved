from __future__ import annotations

from scripts import validate_rviz_moveit_simulation_launches as v


def test_headless_smoke_does_not_use_undrained_launch_pipes(monkeypatch):
    popen_kwargs = {}

    class DummyProc:
        pid = 4242

        def communicate(self, timeout=None):
            return None, None

    def fake_popen(*args, **kwargs):
        popen_kwargs.update(kwargs)
        return DummyProc()

    controllers = "\n".join(
        [
            "joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active",
            "ur5_arm_controller[joint_trajectory_controller/JointTrajectoryController] active",
            "ur5_gripper_controller[joint_trajectory_controller/JointTrajectoryController] active",
        ]
    )

    def fake_wait(args, timeout_sec, predicate=None):
        if args[:3] == ["ros2", "node", "list"]:
            out = "/move_group\n/ur5_2f_test_robot_state_publisher\n/controller_manager\n"
        elif args[:3] == ["ros2", "param", "get"]:
            out = "String value is: loaded\n"
        elif args[:3] == ["ros2", "topic", "list"]:
            out = "/joint_states\n/tf\n/tf_static\n"
        elif args[:4] == ["ros2", "topic", "echo", "/tf_static"]:
            out = "transforms:\n- header:\n    frame_id: world\n  child_frame_id: base_link\n"
        elif args[:3] == ["ros2", "control", "list_controllers"]:
            out = controllers
        else:
            out = ""
        return 0, out, ""

    monkeypatch.setattr(v.shutil, "which", lambda name: "/usr/bin/ros2")
    monkeypatch.setattr(v.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(v, "_wait_for_ros_check", fake_wait)
    monkeypatch.setattr(v.os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(v.os, "killpg", lambda pgid, sig: None)

    status, blockers, evidence, diagnostics = v.run_headless_smoke(
        "ur5_2f_test",
        "ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=false",
        15,
    )

    assert status == v.PASS
    assert blockers == []
    assert "ur5_arm_controller_active" in evidence
    assert "ur5_gripper_controller_active" in evidence
    assert popen_kwargs["stdout"] is not v.subprocess.PIPE
    assert popen_kwargs["stderr"] is not v.subprocess.PIPE
    assert hasattr(popen_kwargs["stdout"], "write")
    assert hasattr(popen_kwargs["stderr"], "write")
    assert diagnostics["terminated"] is True
    assert "launch_stdout_tail" in diagnostics
    assert "launch_stderr_tail" in diagnostics
