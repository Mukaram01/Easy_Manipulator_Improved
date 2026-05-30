from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import yaml

SCHEMA = "workcell_studio_supported_scenes/v1"
AUTHORING = ["environment.yaml", "layout/workcell_studio_layout.yaml"]
GENERATED = ["cell_definition.yaml", "launch/demo.launch.py", "urdf/scene.urdf.xacro", "generated/scene_visual_mesh_index.json"]


def _entry(name: str, scene_path: str | None = None, **overrides):
    package = overrides.pop("package_name", name)
    entry = {
        "scene_name": name,
        "package_name": package,
        "scene_path": scene_path or f"scenes/{name}",
        "support_level": "supported",
        "status": "supported",
        "known_blocker": "",
        "authoring_files": AUTHORING,
        "generated_files": GENERATED,
        "validation_command": f"python3 scripts/validate_builder_generated_scene.py scenes/{name} --json",
        "build_package_name": package,
        "build_command": f"colcon build --symlink-install --packages-select {package}",
        "fake_hardware_launch_command": f"ros2 launch {package} demo.launch.py use_fake_hardware:=true launch_rviz:=true",
    }
    entry.update(overrides)
    return entry


def _catalog(entries):
    return {"schema_version": SCHEMA, "scenes": entries}

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "validate_supported_scenes_readiness.py"


def _write_scene(scene_dir: Path, name: str) -> None:
    scene_dir.mkdir(parents=True, exist_ok=True)
    (scene_dir / "package.xml").write_text(f"<package><name>{name}</name></package>\n", encoding="utf-8")
    (scene_dir / "CMakeLists.txt").write_text(f"project({name})\nament_package()\n", encoding="utf-8")
    (scene_dir / "environment.yaml").write_text("name: env\n", encoding="utf-8")
    (scene_dir / "cell_definition.yaml").write_text("robot: ur5\nend_effector: suction\nenvironment: demo\n", encoding="utf-8")
    (scene_dir / "launch").mkdir(exist_ok=True)
    (scene_dir / "launch/demo.launch.py").write_text("use_fake_hardware launch_rviz robot_state_publisher rviz xacro\n", encoding="utf-8")
    (scene_dir / "urdf").mkdir(exist_ok=True)
    (scene_dir / "urdf/scene.urdf.xacro").write_text("<robot name='x'></robot>\n", encoding="utf-8")
    (scene_dir / "layout").mkdir(exist_ok=True)
    (scene_dir / "layout/workcell_studio_layout.yaml").write_text("layout: ok\n", encoding="utf-8")
    (scene_dir / "generated").mkdir(exist_ok=True)
    (scene_dir / "generated/scene_package_readiness.json").write_text(json.dumps({"package_name": name}), encoding="utf-8")
    (scene_dir / "generated/scene_visual_mesh_index.json").write_text(json.dumps({"items": [{"render_expected": True}]}), encoding="utf-8")


def _run(tmp_path: Path, registry: Path, extra: list[str] | None = None) -> dict:
    cmd = [sys.executable, str(SCRIPT), "--repo-root", str(tmp_path), "--workspace-root", str(tmp_path), "--registry", str(registry), "--json", "--skip-build", "--skip-launch-smoke"]
    cmd.extend(extra or [])
    run = subprocess.run(cmd, capture_output=True, text=True, check=False)
    return json.loads(run.stdout)


def _row_by_scene(payload: dict, scene_name: str) -> dict:
    rows = {row["scene_name"]: row for row in payload["per_scene"]}
    return rows[scene_name]


def _assert_readiness_fields(row: dict) -> None:
    assert "blocker" in row
    assert "missing_authoring_files" in row
    assert "missing_generated_files" in row
    assert "validation_result" in row
    assert "package_contract" in row
    assert "mesh_index_validation" in row


def test_disabled_and_experimental_skipped(tmp_path: Path):
    _write_scene(tmp_path / "scenes/ok_scene", "ok_scene")
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry("disabled_scene", "scenes/disabled_scene", enabled=False),
        _entry("exp_scene", "scenes/ok_scene", support_level="experimental"),
    ])), encoding="utf-8")
    payload = _run(tmp_path, reg)
    assert payload["summary"]["skipped"] == 2

    disabled = _row_by_scene(payload, "disabled_scene")
    _assert_readiness_fields(disabled)
    assert disabled["status"] == "SKIPPED"
    assert disabled["validation_result"] == "SKIPPED"
    assert disabled["blocker"] == "scene_disabled_in_catalog"
    assert disabled["missing_authoring_files"] == AUTHORING
    assert disabled["missing_generated_files"] == GENERATED

    experimental = _row_by_scene(payload, "exp_scene")
    _assert_readiness_fields(experimental)
    assert experimental["status"] == "SKIPPED"
    assert experimental["validation_result"] == "SKIPPED"
    assert experimental["blocker"] == ""
    assert experimental["missing_authoring_files"] == []
    assert experimental["missing_generated_files"] == []


def test_missing_scene_is_blocked(tmp_path: Path):
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([_entry("missing", "scenes/missing")])), encoding="utf-8")
    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]
    _assert_readiness_fields(row)
    assert row["status"] == "BLOCKED"
    assert row["validation_result"] == "BLOCKED"
    assert row["blocker"].startswith("scene_path_missing:")
    assert row["missing_authoring_files"] == AUTHORING
    assert row["missing_generated_files"] == GENERATED


def test_supported_scene_with_missing_required_files_and_empty_blocker_fails_clearly(tmp_path: Path, minimal_scene_factory):
    _, entry = minimal_scene_factory(
        "fixture_missing_required",
        missing_authoring_files=["environment.yaml"],
        missing_generated_files=["generated/scene_visual_mesh_index.json"],
        known_blocker="",
    )
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([entry])), encoding="utf-8")
    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]
    _assert_readiness_fields(row)
    assert row["status"] == "FAIL"
    assert row["validation_result"] == "FAIL"
    assert row["known_blocker"] == ""
    assert row["missing_authoring_files"] == ["environment.yaml"]
    assert row["missing_generated_files"] == ["generated/scene_visual_mesh_index.json"]
    assert row["blocker"].startswith("missing_required_file: environment.yaml; missing_required_file: generated/scene_visual_mesh_index.json")
    assert "missing_required_file: environment.yaml" in row["blockers"]


def test_experimental_included_with_flag(tmp_path: Path):
    _write_scene(tmp_path / "scenes/exp", "exp")
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([_entry("exp", "scenes/exp", support_level="experimental")])), encoding="utf-8")
    payload = _run(tmp_path, reg, ["--include-experimental"])
    row = payload["per_scene"][0]
    _assert_readiness_fields(row)
    assert row["status"] in {"PASS", "PASS_WITH_WARNINGS"}
    assert row["validation_result"] == row["guided_build_launch_readiness"]["status"]
    assert row["missing_authoring_files"] == []
    assert row["missing_generated_files"] == []


def test_blocked_scene_reports_known_blocker(tmp_path: Path, minimal_scene_factory):
    _, entry = minimal_scene_factory(
        "fixture_blocked",
        catalog_status="blocked",
        known_blocker="awaiting generated launch repair",
    )
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([entry])), encoding="utf-8")
    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]
    _assert_readiness_fields(row)
    assert row["status"] == "BLOCKED"
    assert row["validation_result"] == "BLOCKED"
    assert row["blocker"].startswith("awaiting generated launch repair")
    assert row["missing_authoring_files"] == []
    assert row["missing_generated_files"] == []


def test_catalog_blocked_scene_reports_catalog_blocker_and_skips_guided_validator(tmp_path: Path):
    scene_dir = tmp_path / "scenes/blocked_static"
    _write_scene(scene_dir, "blocked_static")
    (scene_dir / "environment.yaml").unlink()
    known_blocker = "awaiting scene regeneration after robot-tool metadata repair"
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry("blocked_static", "scenes/blocked_static", status="blocked", known_blocker=known_blocker),
    ])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["status"] == "BLOCKED"
    assert row["validation_result"] == "BLOCKED"
    assert row["known_blocker"] == known_blocker
    assert row["blocker"] == known_blocker
    assert row["blockers"] == [known_blocker]
    assert row["static_validation"] == {"status": "FAIL", "missing_files": ["environment.yaml"]}
    assert row["missing_authoring_files"] == ["environment.yaml"]
    assert row["commands_run"] == []
    assert row["guided_build_launch_readiness"] == {"status": "SKIPPED"}
    assert row["mesh_index_validation"]["status"] == "PASS"


def test_catalog_rejects_supported_scene_with_stale_known_blocker(tmp_path: Path):
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry("stale", "scenes/stale", status="supported", known_blocker="old blocker should be cleared"),
    ])), encoding="utf-8")

    payload = _run(tmp_path, reg)

    assert payload["summary"]["blocked"] == 1
    assert payload["per_scene"] == []
    assert "catalog_errors" in payload
    assert any(
        "stale: known_blocker must be empty when status is 'supported'" in error
        for error in payload["catalog_errors"]
    )


def test_catalog_rejects_blocked_scene_without_known_blocker(tmp_path: Path):
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry("blocked_without_reason", "scenes/blocked_without_reason", status="blocked", known_blocker=""),
    ])), encoding="utf-8")

    payload = _run(tmp_path, reg)

    assert payload["summary"]["blocked"] == 1
    assert any(
        "blocked_without_reason: known_blocker must be non-empty when status is 'blocked'" in error
        for error in payload["catalog_errors"]
    )


def test_catalog_rejects_fake_hardware_launch_command_without_fake_hardware_true(tmp_path: Path, minimal_scene_factory):
    _, entry = minimal_scene_factory(
        "fixture_unsafe_launch",
        fake_hardware_launch_command="ros2 launch fixture_unsafe_launch demo.launch.py launch_rviz:=true",
    )
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([entry])), encoding="utf-8")

    payload = _run(tmp_path, reg)

    assert payload["summary"]["blocked"] == 1
    assert payload["per_scene"] == []
    assert any(
        "fixture_unsafe_launch: fake_hardware_launch_command must explicitly set use_fake_hardware:=true" in error
        for error in payload["catalog_errors"]
    )

def test_catalog_rejects_unknown_status_and_support_level(tmp_path: Path):
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry("unknown_contract", "scenes/unknown_contract", status="needs_triage", support_level="beta"),
    ])), encoding="utf-8")

    payload = _run(tmp_path, reg)

    assert payload["summary"]["blocked"] == 1
    assert any(
        "unknown_contract: status must be one of [blocked, disabled, supported]; got 'needs_triage'" in error
        for error in payload["catalog_errors"]
    )
    assert any(
        "unknown_contract: support_level must be one of [experimental, supported]; got 'beta'" in error
        for error in payload["catalog_errors"]
    )


def test_summary_counts(tmp_path: Path):
    _write_scene(tmp_path / "scenes/pass_scene", "pass_scene")
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry("pass_scene", "scenes/pass_scene"),
        _entry("missing_scene", "scenes/missing_scene"),
        _entry("disabled_scene", "scenes/disabled_scene", enabled=False),
    ])), encoding="utf-8")
    payload = _run(tmp_path, reg)
    assert payload["summary"] == {"total": 3, "pass": 1, "fail": 0, "blocked": 1, "warnings": 1, "skipped": 1}


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


def test_malformed_mesh_index_reports_clear_contract_failure(tmp_path: Path, minimal_scene_factory):
    _, entry = minimal_scene_factory("fixture_bad_mesh", mesh_index_payload="{not-json")
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([entry])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["status"] == "FAIL"
    assert row["mesh_index_validation"]["status"] == "INVALID_JSON"
    assert any("mesh_index_validation_invalid_json: generated/scene_visual_mesh_index.json" in b for b in row["blockers"])


def test_zero_renderable_mesh_index_reports_clear_contract_failure(tmp_path: Path, minimal_scene_factory):
    _, entry = minimal_scene_factory("fixture_empty_mesh", mesh_index_payload={"visual_items": []})
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([entry])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["status"] == "FAIL"
    assert row["mesh_index_validation"]["status"] == "NO_RENDERABLE_ITEMS"
    assert row["mesh_index_validation"]["renderable_items"] == 0
    assert "mesh_index_validation_no_renderable_items: generated/scene_visual_mesh_index.json contains 0 renderable items" in row["blockers"]


def test_missing_mesh_index_reports_clear_validation_failure(tmp_path: Path, minimal_scene_factory):
    _, entry = minimal_scene_factory(
        "fixture_missing_mesh",
        missing_generated_files=["generated/scene_visual_mesh_index.json"],
    )
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([entry])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["status"] == "FAIL"
    assert row["mesh_index_validation"]["status"] == "MISSING_FILE"
    assert "mesh_index_validation_missing_file: generated/scene_visual_mesh_index.json" in row["blockers"]


def test_malformed_mesh_index_item_entries_are_blockers(tmp_path: Path, minimal_scene_factory):
    _, entry = minimal_scene_factory(
        "fixture_malformed_items",
        mesh_index_payload={"items": [{"render_expected": True}, "not-an-object"]},
    )
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([entry])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["status"] == "FAIL"
    assert row["mesh_index_validation"]["status"] == "MALFORMED_ITEMS"
    assert row["mesh_index_validation"]["malformed_items"] == [{"index": 1, "reason": "item_not_object"}]
    assert "mesh_index_validation_malformed_items: generated/scene_visual_mesh_index.json contains 1 malformed item entries" in row["blockers"]


def test_blocked_scene_preserves_known_blocker_and_reports_mesh_issue(tmp_path: Path):
    _write_scene(tmp_path / "scenes/blocked_bad_mesh", "blocked_bad_mesh")
    (tmp_path / "scenes/blocked_bad_mesh/generated/scene_visual_mesh_index.json").write_text(
        json.dumps({"visual_items": []}), encoding="utf-8"
    )
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry("blocked_bad_mesh", "scenes/blocked_bad_mesh", status="blocked", known_blocker="awaiting launch repair"),
    ])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["status"] == "BLOCKED"
    assert row["blockers"][0] == "awaiting launch repair"
    assert row["mesh_index_validation"]["status"] == "NO_RENDERABLE_ITEMS"
    assert "mesh_index_validation_no_renderable_items: generated/scene_visual_mesh_index.json contains 0 renderable items" in row["blockers"]


def test_package_xml_name_mismatch_blocks_supported_scene(tmp_path: Path, minimal_scene_factory):
    _, entry = minimal_scene_factory(
        "fixture_pkg_mismatch",
        package_xml_name="actual_package",
        package_name="catalog_package",
        build_package_name="actual_package",
        build_command="colcon build --symlink-install --packages-select actual_package",
        fake_hardware_launch_command="ros2 launch catalog_package demo.launch.py use_fake_hardware:=true launch_rviz:=true",
    )
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([entry])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["status"] == "BLOCKED"
    assert row["validation_result"] == "BLOCKED"
    assert row["package_contract"]["package_xml_name"] == "actual_package"
    assert row["package_contract"]["status"] == "BLOCKED"
    assert any("catalog_package_name_mismatch" in blocker for blocker in row["blockers"])


def test_build_package_name_mismatch_blocks_supported_scene(tmp_path: Path, minimal_scene_factory):
    _, entry = minimal_scene_factory(
        "fixture_build_pkg_mismatch",
        package_xml_name="actual_package",
        package_name="actual_package",
        build_package_name="stale_build_package",
        build_command="colcon build --symlink-install --packages-select stale_build_package",
        fake_hardware_launch_command="ros2 launch actual_package demo.launch.py use_fake_hardware:=true launch_rviz:=true",
    )
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([entry])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["status"] == "BLOCKED"
    assert row["validation_result"] == "BLOCKED"
    assert row["package_contract"]["package_xml_name"] == "actual_package"
    assert row["package_contract"]["build_command_packages_select"] == ["stale_build_package"]
    assert any("catalog_build_package_name_mismatch" in blocker for blocker in row["blockers"])


def test_build_command_package_selection_mismatch_blocks_supported_scene(tmp_path: Path):
    _write_scene(tmp_path / "scenes/build_command_mismatch", "actual_package")
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry(
            "build_command_mismatch",
            "scenes/build_command_mismatch",
            package_name="actual_package",
            build_package_name="actual_package",
            build_command="colcon build --symlink-install --packages-select stale_build_package",
            fake_hardware_launch_command="ros2 launch actual_package demo.launch.py use_fake_hardware:=true launch_rviz:=true",
        ),
    ])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["status"] == "BLOCKED"
    assert row["validation_result"] == "BLOCKED"
    assert row["package_contract"]["build_command_packages_select"] == ["stale_build_package"]
    assert any("build_command_package_selection_mismatch" in blocker for blocker in row["blockers"])


def test_fake_hardware_launch_package_mismatch_blocks_supported_scene(tmp_path: Path):
    _write_scene(tmp_path / "scenes/launch_pkg_mismatch", "actual_package")
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry(
            "launch_pkg_mismatch",
            "scenes/launch_pkg_mismatch",
            package_name="actual_package",
            build_package_name="actual_package",
            build_command="colcon build --symlink-install --packages-select actual_package",
            fake_hardware_launch_command="ros2 launch stale_launch_package demo.launch.py use_fake_hardware:=true launch_rviz:=true",
        ),
    ])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["status"] == "BLOCKED"
    assert row["validation_result"] == "BLOCKED"
    assert row["package_contract"]["fake_hardware_launch_package"] == "stale_launch_package"
    assert any("fake_hardware_launch_package_mismatch" in blocker for blocker in row["blockers"])


def test_blocked_scene_preserves_known_blocker_and_reports_package_mismatch(tmp_path: Path):
    _write_scene(tmp_path / "scenes/blocked_pkg_mismatch", "actual_package")
    known_blocker = "awaiting package regeneration"
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry(
            "blocked_pkg_mismatch",
            "scenes/blocked_pkg_mismatch",
            status="blocked",
            known_blocker=known_blocker,
            package_name="catalog_package",
            build_package_name="actual_package",
            build_command="colcon build --symlink-install --packages-select actual_package",
            fake_hardware_launch_command="ros2 launch catalog_package demo.launch.py use_fake_hardware:=true launch_rviz:=true",
        ),
    ])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["status"] == "BLOCKED"
    assert row["blocker"] == known_blocker
    assert row["blockers"][0] == known_blocker
    assert row["package_contract"]["status"] == "BLOCKED"
    assert any("catalog_package_name_mismatch" in blocker for blocker in row["blockers"])
