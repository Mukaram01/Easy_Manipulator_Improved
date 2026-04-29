from pathlib import Path


def test_no_hardcoded_parallel_workers_2_in_docs_or_scripts():
    roots = [Path("README.md"), Path("fix_and_build_humble.sh"), Path("scripts")]
    for root in roots:
        if root.is_file():
            text = root.read_text()
            assert "--parallel-workers 2" not in text
            continue
        for path in root.rglob("*"):
            if path.is_file() and path.suffix in {".sh", ".md", ".py", ".txt"}:
                assert "--parallel-workers 2" not in path.read_text(), str(path)


def test_fix_and_build_uses_external_osqp_strategy():
    text = Path("fix_and_build_humble.sh").read_text()
    assert "build/_external/osqp" in text
    assert "build/_external/osqp-eigen" in text
    assert "hide_source_only_vendor_dependencies" in text
    assert "src/osqp" in text and "COLCON_IGNORE" in text and "AMENT_IGNORE" in text
    assert "/usr/local/lib/cmake/osqp" in text
    assert "/usr/local/lib/cmake/OsqpEigen" in text


def test_prepare_osqp_stack_expects_v1_symbols_not_legacy_auxil():
    text = Path("fix_and_build_humble.sh").read_text()
    assert "osqp_api_types.h" in text
    for symbol in ["OSQPSolver", "OSQPInt", "OSQPCscMatrix"]:
        assert symbol in text
    assert "auxil.h" not in text
    assert "OSQP 0.6.3 + OsqpEigen 0.8.0" not in text


def test_summary_expected_matrix_uses_v1_stack():
    text = Path("fix_and_build_humble.sh").read_text()
    assert "expected_matrix" in text
    assert "OSQP=v1.x, OsqpEigen=v0.11.x" in text
    assert "OSQP=0.6.3" not in text


def test_rosdep_skip_keys_include_required_variants():
    text = Path("fix_and_build_humble.sh").read_text()
    for key in ["osqp", "osqp_vendor", "osqp-eigen", "osqp_eigen", "qpoases"]:
        assert key in text


def test_trajopt_required_packages_are_exposed_and_verified():
    text = Path("scripts/fix_workspace_layout.sh").read_text()
    assert "required=(trajopt trajopt_common trajopt_sco trajopt_ifopt trajopt_sqp)" in text
    assert "Verified TrajOpt package:" in text
    assert "$SRC_DIR/tesseract_planning" in text
    assert "$BACKUP_DIR/original" in text


def test_epd_underlay_validation_uses_local_setup_and_epd_msgs():
    text = Path("fix_and_build_humble.sh").read_text()
    assert "install/local_setup.bash" in text
    assert "ros2 pkg prefix epd_msgs" in text


def test_default_colcon_build_sets_release_type():
    text = Path("fix_and_build_humble.sh").read_text()
    assert "CMAKE_BUILD_TYPE=\"Release\"" in text
    assert "--cmake-args -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE" in text
    assert "--cmake-build-type" in text


def test_full_profile_without_gui_skips_rviz_and_examples_together():
    text = Path("fix_and_build_humble.sh").read_text()
    assert "requested_skips+=(tesseract_qt qtadvanceddocking QtADS tesseract_rviz tesseract_ros_examples tesseract_planning_server)" in text
    assert "Dependency-safe skip: tesseract_ros_examples must be skipped when tesseract_rviz is skipped." in text


def test_skip_list_is_filtered_to_discovered_packages_before_colcon():
    text = Path("fix_and_build_humble.sh").read_text()
    assert "colcon list --base-paths src --names-only" in text
    assert "if printf '%s\\n' \"${all_packages[@]}\" | grep -Fxq \"$pkg\"; then" in text
    assert "COLCON_SKIP_PACKAGES_USED=(\"${discovered[@]:-}\")" in text


def test_target_workspace_install_is_not_used_as_underlay_before_build():
    text = Path("fix_and_build_humble.sh").read_text()
    assert "capture_target_workspace_env_contamination" in text
    assert "unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH" in text
    assert "source '$ros_setup'" in text
    assert "source '$EPD_UNDERLAY/install/local_setup.bash'" in text
    assert "target workspace install path was present in the shell environment before build" in text


def test_readme_documents_full_non_gui_default_path():
    text = Path("README.md").read_text()
    assert "full non-GUI" in text
    assert "RViz/Tesseract examples are skipped by default" in text
    assert "Do not source `~/workcell_ws/install/setup.bash` until after the installer completes." in text


def test_required_planner_and_runtime_packages_not_in_default_skip_list():
    text = Path("fix_and_build_humble.sh").read_text()
    skip_block = "requested_skips+=(tesseract_qt qtadvanceddocking QtADS tesseract_rviz tesseract_ros_examples tesseract_planning_server)"
    assert skip_block in text
    for pkg in [
        "tesseract_motion_planners",
        "tesseract_task_composer",
        "tesseract_rosutils",
        "tesseract_monitoring",
        "trajopt",
        "trajopt_ifopt",
        "trajopt_sqp",
        "trajopt_sco",
        "emd_grasp_planner",
        "emd_grasp_execution",
        "emd_waypoint_execution",
        "run_grasp_planner",
        "run_grasp_execution",
        "run_waypoint_execution",
        "workcell_builder",
    ]:
        assert pkg not in skip_block
