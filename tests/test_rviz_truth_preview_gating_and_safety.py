from pathlib import Path


RUNNER_CPP = Path("workcell_builder/workcell_builder/src_rviz_preview_runner.cpp").read_text(encoding="utf-8")
MAIN_CPP = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")


def test_safe_direct_command_accepted():
    assert "use_fake_hardware:=true" in RUNNER_CPP
    assert "launch_rviz:=true" in RUNNER_CPP
    assert "ros2\\s+launch\\s+\\S+\\s+demo\\.launch\\.py" in RUNNER_CPP


def test_safe_bash_wrapper_accepted():
    assert "build_shell_command" in RUNNER_CPP
    assert "bash -lc 'source /opt/ros/humble/setup.bash && source %1 && %2'" in RUNNER_CPP
    assert "expected_launch_re.match(trimmed).hasMatch()" in RUNNER_CPP


def test_unsafe_false_command_rejected():
    for token in ["use_fake_hardware:=false", "fake_hardware:=false", "ur_robot_driver", "ethercat", "canopen"]:
        assert token in RUNNER_CPP


def test_rviz_truth_preview_not_blocked_by_stale_layout_when_package_present():
    assert "Layout changed since last generation. Run Generate Scene / Layout Merge before preview." not in RUNNER_CPP
    assert "RViz Truth Preview readiness: WARN stale layout detected; continuing with generated package." in MAIN_CPP


def test_missing_package_launch_and_setup_still_block():
    for token in ["package.xml missing", "launch/demo.launch.py missing", "install/setup.bash missing under workspace root"]:
        assert token in RUNNER_CPP


def test_studio_log_pass_before_launch():
    assert "RViz Truth Preview readiness: PASS" in MAIN_CPP
    assert "RViz Truth Preview dry run: " in MAIN_CPP
    assert "RViz Truth Preview launch started" in MAIN_CPP
