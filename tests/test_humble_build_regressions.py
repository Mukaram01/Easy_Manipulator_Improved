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
    assert "hide_workspace_osqp_sources" in text
    assert "src/osqp" in text and "COLCON_IGNORE" in text and "AMENT_IGNORE" in text
    assert "/usr/local/lib/cmake/osqp" in text
    assert "/usr/local/lib/cmake/OsqpEigen" in text


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
