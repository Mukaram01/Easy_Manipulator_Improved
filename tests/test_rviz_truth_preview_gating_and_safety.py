from pathlib import Path


RUNNER_CPP = Path("workcell_builder/workcell_builder/src_rviz_preview_runner.cpp").read_text(encoding="utf-8")
MAIN_CPP = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")


def test_safe_direct_command_accepted():
    assert "use_fake_hardware:=true" in RUNNER_CPP
    assert "launch_rviz:=true" in RUNNER_CPP
    assert "ros2\\s+launch\\s+\\S+\\s+\\S+\\.launch\\.py" in RUNNER_CPP


def test_safe_bash_wrapper_accepted():
    assert "build_shell_command" in RUNNER_CPP
    assert "bash -lc 'source /opt/ros/humble/setup.bash && source %1 && %2'" in RUNNER_CPP
    assert "expected_launch_re.match(trimmed).hasMatch()" in RUNNER_CPP


def test_unsafe_false_command_rejected():
    for token in ["use_fake_hardware:=false", "fake_hardware:=false", "ur_robot_driver", "ethercat", "canopen"]:
        assert token in RUNNER_CPP


def test_stale_generated_package_blocks_build_and_run():
    assert "Generated scene package is stale. Run Generate Scene Package before Build & Run RViz." in MAIN_CPP


def test_source_requirements_block_but_install_overlay_is_created_by_build():
    for token in ["package.xml missing", "CMakeLists.txt missing", "launch/demo.launch.py missing"]:
        assert token in RUNNER_CPP
    assert 'blocked("install/setup.bash missing under workspace root")' not in RUNNER_CPP


def test_build_then_discovery_then_safe_launch_contract():
    assert "colcon build --symlink-install --packages-up-to '%2'" in RUNNER_CPP
    assert "colcon build --symlink-install --packages-select '%2'" not in RUNNER_CPP
    assert "ros2 pkg prefix '%2'" in RUNNER_CPP
    assert "source /opt/ros/humble/setup.bash && source '%1' && exec %2" in RUNNER_CPP
    assert MAIN_CPP.index('set_preview_state("BUILD_RUNNING")') < MAIN_CPP.index('set_preview_state("PACKAGE_CHECK_RUNNING")')
    assert MAIN_CPP.index('set_preview_state("PACKAGE_CHECK_RUNNING")') < MAIN_CPP.index('set_preview_state("PREVIEW_LAUNCHING")')


def test_gui_owns_one_async_process_and_controlled_stop():
    header = Path("workcell_builder/workcell_builder/gui/mainwindow.h").read_text(encoding="utf-8")
    assert "QProcess * preview_process_" in header
    workflow = MAIN_CPP[MAIN_CPP.index("void MainWindow::run_preview_build") : MAIN_CPP.index("void MainWindow::write_preview_launch_transcript")]
    assert 'preview_process_->start("/bin/bash"' in workflow
    assert "waitForFinished" not in workflow
    assert "preview_process_->terminate()" in workflow
    assert "preview_process_->kill()" in workflow
    assert "killall" not in workflow and "pkill ros2" not in workflow
    assert "closeEvent(QCloseEvent * event)" in MAIN_CPP


def test_main_action_toggles_without_duplicate_scene_copy():
    assert 'setText(active ? "Stop RViz" : "Build & Run RViz")' in MAIN_CPP
    assert 'a Workcell Studio-owned process is already active' in MAIN_CPP
    workflow = MAIN_CPP[MAIN_CPP.index("void MainWindow::run_preview_build") : MAIN_CPP.index("void MainWindow::write_preview_launch_transcript")]
    assert "src/scenes" not in workflow
    assert "active_preview_scene_" in workflow
    assert "active_preview_workspace_root_" in workflow


def test_studio_log_reports_build_and_launch_stages():
    for stage in ["Checking scene...", "Building ", "Build succeeded", "Launching RViz...", "RViz running", "RViz stopped"]:
        assert stage in MAIN_CPP
