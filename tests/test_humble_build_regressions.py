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


def test_airpick_macro_only_wrapper_supplies_prefix_and_parent_defaults():
    text = Path("workcell_builder/workcell_builder/gui/addendeffector.cpp").read_text()
    assert "wrapper_xml += \" prefix=\\\"\\\"\"" in text
    assert "wrapper_xml += \" parent=\\\"tool0\\\"\"" in text


def test_existing_scene_directory_is_rejected_before_generation_starts():
    text = Path("workcell_builder/workcell_builder/gui/addscene.cpp").read_text()
    assert "Scene directory already exists:" in text
    assert "Please choose a different scene name or delete the existing scene first." in text


def test_humble_prereqs_check_moveit_ros_perception_for_octomap():
    text = Path("fix_and_build_humble.sh").read_text()
    assert "ros2 pkg prefix moveit_ros_perception" in text
    assert "Missing MoveIt perception package required for octomap pointcloud updates: ros-humble-moveit-ros-perception" in text
    assert "missing_tools+=(ros-humble-moveit-ros-perception)" in text


def _ament_build_type(package_xml: Path) -> str:
    import xml.etree.ElementTree as ET

    root = ET.parse(package_xml).getroot()
    export = root.find("export")
    if export is not None:
        build_type = export.find("build_type")
        if build_type is not None and build_type.text:
            return build_type.text.strip()
    return "ament_cmake"


def _is_colcon_ignored(path: Path) -> bool:
    return any((parent / "COLCON_IGNORE").exists() for parent in [path, *path.parents])


def test_every_discovered_ament_cmake_package_has_cmakelists():
    missing = []
    for package_xml in Path(".").rglob("package.xml"):
        if any(part.startswith(".") for part in package_xml.parts):
            continue
        if _is_colcon_ignored(package_xml.parent):
            continue
        if _ament_build_type(package_xml) == "ament_cmake" and not (package_xml.parent / "CMakeLists.txt").is_file():
            missing.append(str(package_xml.parent))
    assert missing == []


def test_ur_description_is_installable_ament_cmake_description_package():
    package_dir = Path("assets/robots/universal_robot/ur_description")
    cmake = (package_dir / "CMakeLists.txt").read_text()
    package_xml = (package_dir / "package.xml").read_text()

    assert "<name>ur_description</name>" in package_xml
    assert "project(ur_description)" in cmake
    assert "find_package(ament_cmake REQUIRED)" in cmake
    assert "ament_package()" in cmake
    assert "install(FILES package.xml" in cmake

    for resource_dir in ["meshes", "urdf", "config", "launch", "rviz"]:
        assert (package_dir / resource_dir).is_dir(), resource_dir
        assert resource_dir in cmake
    assert (package_dir / "urdf" / "ur.urdf.xacro").is_file()
    assert any((package_dir / "meshes" / "ur5").rglob("*"))


def test_ur_description_is_not_hidden_from_colcon():
    package_dir = Path("assets/robots/universal_robot/ur_description")
    assert not (package_dir / "COLCON_IGNORE").exists()
    assert not (package_dir / "AMENT_IGNORE").exists()


def test_scene_preview_widget_targets_link_qt_network():
    cmake = Path("workcell_builder/workcell_builder/CMakeLists.txt").read_text()
    assert "gui/scene_preview_widget.cpp" in cmake
    assert "find_package(Qt5 COMPONENTS Widgets Concurrent Svg OpenGL Network REQUIRED)" in cmake
    assert "target_link_libraries(workcell_builder" in cmake and "Qt5::Network" in cmake
    assert "ament_add_gtest(workcell_scene_preview_widget_ui_test" in cmake
    assert "target_link_libraries(workcell_scene_preview_widget_ui_test Qt5::Widgets Qt5::OpenGL Qt5::Network OpenGL::GL)" in cmake


def test_scene_preview_widget_still_uses_qtcp_socket_for_server_probe():
    source = Path("workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text()
    assert "#include <QTcpSocket>" in source
    assert "QTcpSocket socket" in source


def test_embedded_workcell_builder_asset_copies_are_not_colcon_packages():
    # These files are installed as workcell_builder runtime assets. The canonical
    # buildable description packages live under the repository-level assets/ tree.
    assert Path("workcell_builder/workcell_builder/assets/COLCON_IGNORE").is_file()
    assert not Path("assets/COLCON_IGNORE").exists()


def test_discovered_package_names_are_unique():
    import xml.etree.ElementTree as ET

    packages_by_name = {}
    for package_xml in Path(".").rglob("package.xml"):
        if any(part.startswith(".") for part in package_xml.parts):
            continue
        if _is_colcon_ignored(package_xml.parent):
            continue
        name = ET.parse(package_xml).getroot().findtext("name")
        packages_by_name.setdefault(name, []).append(str(package_xml.parent))
    duplicates = {name: paths for name, paths in packages_by_name.items() if len(paths) > 1}
    assert duplicates == {}


def test_scene_generated_directories_are_installed_conditionally():
    """Scene generated/ output is optional in a clean checkout."""
    import re

    def exists_guarded_generated_installs(cmake_text: str) -> bool:
        lines = cmake_text.splitlines()
        for index, line in enumerate(lines):
            if not re.search(r"if\s*\([^)]*EXISTS[^)]*generated[^)]*\)", line):
                continue
            guarded_block = "\n".join(lines[index : index + 8])
            if re.search(r"install\s*\(\s*DIRECTORY\s+generated\b", guarded_block):
                return True
        return False

    offenders = []
    for cmake_path in sorted(Path("scenes").glob("*/CMakeLists.txt")):
        text = cmake_path.read_text()
        has_helper = re.search(r"install_scene_directory_if_present\s*\(\s*generated\s*\)", text)
        has_exists_guard = exists_guarded_generated_installs(text)
        if not (has_helper or has_exists_guard):
            offenders.append(str(cmake_path))

    assert offenders == []


def test_humble_ci_imports_source_dependencies_before_layout_fix():
    text = Path(".github/workflows/humble-ci.yml").read_text()
    import_idx = text.index("vcs import --recursive --skip-existing src < src/easy_manipulation_deployment/dependencies/emd_epd_ws.repos")
    layout_idx = text.index("./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh")
    rosdep_idx = text.index("rosdep install --from-paths src --ignore-src -yr --rosdistro humble")
    assert import_idx < layout_idx < rosdep_idx


def test_humble_ci_rosdep_failure_runs_workspace_diagnostics():
    text = Path(".github/workflows/humble-ci.yml").read_text()
    assert "diagnose_rosdep_workspace.py" in text
    assert "--os-codename jammy" in text
    assert "--rosdistro humble" in text
    assert "--skip-keys \"qt_advanced_docking tesseract_visualization\"" in text


def test_rosdep_diagnostics_reports_source_providers_and_declaring_packages(tmp_path, monkeypatch, capsys):
    import importlib.util

    script = Path("scripts/diagnose_rosdep_workspace.py")
    spec = importlib.util.spec_from_file_location("diagnose_rosdep_workspace", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    src = tmp_path / "src"
    provider = src / "provider"
    consumer = src / "consumer"
    provider.mkdir(parents=True)
    consumer.mkdir(parents=True)
    (provider / "package.xml").write_text("""
<package format='3'>
  <name>source_only_dep</name>
  <version>0.0.0</version>
  <description>provider</description>
  <maintainer email='dev@example.com'>Dev</maintainer>
  <license>Apache-2.0</license>
</package>
""")
    (consumer / "package.xml").write_text("""
<package format='3'>
  <name>consumer</name>
  <version>0.0.0</version>
  <description>consumer</description>
  <maintainer email='dev@example.com'>Dev</maintainer>
  <license>Apache-2.0</license>
  <depend>source_only_dep</depend>
  <exec_depend>skipped_dep</exec_depend>
</package>
""")

    monkeypatch.setattr(
        "sys.argv",
        [
            str(script),
            "--src",
            str(src),
            "--rosdistro",
            "humble",
            "--os-codename",
            "jammy",
            "--skip-keys",
            "skipped_dep",
        ],
    )

    assert module.main() == 0
    output = capsys.readouterr().out
    assert "SOURCE source_only_dep: provided by" in output
    assert "declared by" in output and "consumer/package.xml" in output
    assert "SKIP skipped_dep" in output


def test_humble_rosdep_overrides_resolve_jammy_taskflow_and_gperftools_without_changing_jazzy():
    text = Path("scripts/rosdep_overrides.yaml").read_text()
    assert "libtaskflow-cpp-dev:" in text
    assert "jammy: []" in text
    assert "noble: [libtaskflow-cpp-dev]" in text
    assert "taskflow:\n  ubuntu:" in text
    assert "noble: [libtaskflow-dev]" in text
    assert "gperftools:" in text
    assert "jammy: [google-perftools, libunwind-15-dev, libgoogle-perftools-dev]" in text
    assert "noble: [google-perftools, libgoogle-perftools-dev]" in text
    assert "libunwind-15-dev on Jammy" in text
    assert "imported tesseract_planning" in text


def test_humble_ci_exports_source_taskflow_cmake_package_before_build():
    text = Path(".github/workflows/humble-ci.yml").read_text()
    assert "ensure_taskflow_cmake_package.sh --export >> \"$GITHUB_ENV\"" in text
    assert text.index("ensure_taskflow_cmake_package.sh --export") < text.index("rosdep install --from-paths src --ignore-src -yr --rosdistro humble")


def test_rosdep_diagnostics_fails_when_no_source_packages_discovered(tmp_path, monkeypatch, capsys):
    import importlib.util

    script = Path("scripts/diagnose_rosdep_workspace.py")
    spec = importlib.util.spec_from_file_location("diagnose_rosdep_workspace_empty", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    src = tmp_path / "src"
    src.mkdir()
    monkeypatch.setattr(
        "sys.argv",
        [
            str(script),
            "--src",
            str(src),
            "--rosdistro",
            "humble",
            "--os-codename",
            "jammy",
        ],
    )

    assert module.main() == 1
    captured = capsys.readouterr()
    assert "discovered source packages: 0" in captured.out
    assert "No source packages discovered" in captured.err


def test_mainwindow_qt5_scene_builder_regressions_are_fixed():
    text = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text()
    assert "trimmed('_')" not in text
    assert "build_workcell_studio_canvas_model(s.scene_dir, QString::fromStdString(s.scene_name))" not in text
    assert "stem = stem.trimmed();" in text
    assert "stem.startsWith(QLatin1Char('_'))" in text
    assert "stem.remove(0, 1);" in text
    assert "stem.endsWith(QLatin1Char('_'))" in text
    assert "stem.chop(1);" in text
    assert "build_workcell_studio_canvas_model(s.scene_dir, s.scene_name)" in text
