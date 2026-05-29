from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import yaml

SCHEMA = "workcell_studio_supported_scenes/v1"
AUTHORING = ["environment.yaml", "layout/workcell_studio_layout.yaml"]
GENERATED = ["cell_definition.yaml", "scene_manifest.yaml", "urdf/scene.urdf.xacro", "launch/demo.launch.py", "generated/scene_visual_mesh_index.json"]

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "validate_supported_scenes_readiness.py"


def _run(tmp_path: Path, registry: Path, extra: list[str] | None = None) -> dict:
    cmd = [
        sys.executable,
        str(SCRIPT),
        "--repo-root",
        str(tmp_path),
        "--workspace-root",
        str(tmp_path),
        "--registry",
        str(registry),
        "--json",
        "--skip-build",
        "--skip-launch-smoke",
    ]
    cmd.extend(extra or [])
    run = subprocess.run(cmd, capture_output=True, text=True, check=False)
    return json.loads(run.stdout)


def test_disabled_and_experimental_skipped(tmp_path: Path, make_minimal_scene, make_supported_scene_entry, write_scene_catalog):
    make_minimal_scene(tmp_path / "scenes/ok_scene", scene_name="ok_scene")
    reg = write_scene_catalog(
        tmp_path / "registry.yaml",
        [
            make_supported_scene_entry("disabled_scene", scene_path="scenes/disabled_scene", enabled=False),
            make_supported_scene_entry("exp_scene", scene_path="scenes/ok_scene", support_level="experimental", package_name="ok_scene"),
        ],
    )
    payload = _run(tmp_path, reg)
    assert payload["summary"]["skipped"] == 2


def test_missing_scene_is_blocked(tmp_path: Path, make_supported_scene_entry, write_scene_catalog):
    reg = write_scene_catalog(tmp_path / "registry.yaml", [make_supported_scene_entry("missing", scene_path="scenes/missing")])
    payload = _run(tmp_path, reg)
    assert payload["per_scene"][0]["status"] == "BLOCKED"


def test_supported_scene_with_missing_required_file_and_empty_known_blocker_fails_clearly(
    tmp_path: Path, make_minimal_scene, make_supported_scene_entry, write_scene_catalog
):
    make_minimal_scene(
        tmp_path / "scenes/a",
        scene_name="a",
        missing_authoring_files=["environment.yaml"],
    )
    reg = write_scene_catalog(tmp_path / "registry.yaml", [make_supported_scene_entry("a", scene_path="scenes/a", known_blocker="")])
    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]
    assert row["status"] == "FAIL"
    assert "missing_required_file: environment.yaml" in row["blockers"]
    assert "catalog known_blocker is empty for a supported scene with missing required files" in row["blockers"]


def test_blocked_scene_reports_known_blocker(tmp_path: Path, make_minimal_scene, make_supported_scene_entry, write_scene_catalog):
    make_minimal_scene(tmp_path / "scenes/blocked", scene_name="blocked", missing_generated_files=["launch/demo.launch.py"])
    reg = write_scene_catalog(
        tmp_path / "registry.yaml",
        [
            make_supported_scene_entry(
                "blocked",
                scene_path="scenes/blocked",
                status="blocked",
                known_blocker="waiting on generated launch migration",
            )
        ],
    )
    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]
    assert row["status"] == "BLOCKED"
    assert row["known_blocker"] == "waiting on generated launch migration"
    assert "known_blocker: waiting on generated launch migration" in row["blockers"]


def test_fake_hardware_launch_command_requires_fake_hardware_true(tmp_path: Path, make_supported_scene_entry, write_scene_catalog):
    reg = write_scene_catalog(
        tmp_path / "registry.yaml",
        [
            make_supported_scene_entry(
                "unsafe_scene",
                fake_hardware_launch_command="ros2 launch unsafe_scene demo.launch.py launch_rviz:=true",
            )
        ],
    )
    payload = _run(tmp_path, reg)
    assert payload["summary"]["blocked"] == 1
    assert "unsafe_scene: fake_hardware_launch_command must explicitly set use_fake_hardware:=true" in payload["catalog_errors"]


def test_package_and_build_names_must_match_package_xml(
    tmp_path: Path, make_minimal_scene, make_supported_scene_entry, write_scene_catalog
):
    make_minimal_scene(tmp_path / "scenes/pkg_mismatch", scene_name="pkg_mismatch", package_xml_name="actual_pkg")
    reg = write_scene_catalog(
        tmp_path / "registry.yaml",
        [make_supported_scene_entry("pkg_mismatch", scene_path="scenes/pkg_mismatch", package_name="catalog_pkg", build_package_name="catalog_pkg")],
    )
    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]
    assert row["status"] == "FAIL"
    assert "package_name mismatch: catalog package_name='catalog_pkg' package.xml name='actual_pkg'" in row["blockers"]
    assert "build_package_name mismatch: catalog build_package_name='catalog_pkg' package.xml name='actual_pkg'" in row["blockers"]


def test_cmake_project_name_must_match_package_xml(tmp_path: Path, make_minimal_scene, make_supported_scene_entry, write_scene_catalog):
    make_minimal_scene(tmp_path / "scenes/cmake_mismatch", scene_name="cmake_mismatch", cmake_project_name="wrong_project")
    reg = write_scene_catalog(tmp_path / "registry.yaml", [make_supported_scene_entry("cmake_mismatch", scene_path="scenes/cmake_mismatch")])
    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]
    assert row["status"] == "FAIL"
    assert "CMake project mismatch: CMakeLists.txt project='wrong_project' package.xml name='cmake_mismatch'" in row["blockers"]


def test_experimental_included_with_flag(tmp_path: Path, make_minimal_scene, make_supported_scene_entry, write_scene_catalog):
    make_minimal_scene(tmp_path / "scenes/exp", scene_name="exp")
    reg = write_scene_catalog(
        tmp_path / "registry.yaml",
        [make_supported_scene_entry("exp", scene_path="scenes/exp", support_level="experimental")],
    )
    payload = _run(tmp_path, reg, ["--include-experimental"])
    assert payload["per_scene"][0]["status"] in {"PASS", "PASS_WITH_WARNINGS"}


def test_summary_counts(tmp_path: Path, make_minimal_scene, make_supported_scene_entry, write_scene_catalog):
    make_minimal_scene(tmp_path / "scenes/pass_scene", scene_name="pass_scene")
    reg = write_scene_catalog(
        tmp_path / "registry.yaml",
        [
            make_supported_scene_entry("pass_scene", scene_path="scenes/pass_scene"),
            make_supported_scene_entry("missing_scene", scene_path="scenes/missing_scene"),
            make_supported_scene_entry("disabled_scene", scene_path="scenes/disabled_scene", enabled=False),
        ],
    )
    payload = _run(tmp_path, reg)
    assert payload["summary"] == {"total": 3, "pass": 1, "fail": 0, "blocked": 1, "warnings": 0, "skipped": 1}


def test_default_catalog_declares_required_contract_fields():
    catalog = yaml.safe_load((ROOT / "scenes" / "supported_scenes.yaml").read_text(encoding="utf-8"))
    assert catalog["schema_version"] == SCHEMA
    assert catalog.get("source_of_truth") is True
    entries = catalog.get("scenes")
    assert isinstance(entries, list) and entries
    required = {
        "scene_name",
        "package_name",
        "authoring_files",
        "generated_files",
        "validation_command",
        "build_package_name",
        "fake_hardware_launch_command",
        "status",
        "known_blocker",
    }
    for entry in entries:
        assert required.issubset(entry), entry
        assert entry["scene_name"] in entry["validation_command"]
        assert entry["build_package_name"] in entry["build_command"]
        assert "use_fake_hardware:=true" in entry["fake_hardware_launch_command"]
