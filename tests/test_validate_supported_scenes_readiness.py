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


def test_disabled_and_experimental_skipped(tmp_path: Path):
    _write_scene(tmp_path / "scenes/ok_scene", "ok_scene")
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry("disabled_scene", "scenes/disabled_scene", enabled=False),
        _entry("exp_scene", "scenes/ok_scene", support_level="experimental"),
    ])), encoding="utf-8")
    payload = _run(tmp_path, reg)
    assert payload["summary"]["skipped"] == 2


def test_missing_scene_is_blocked(tmp_path: Path):
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([_entry("missing", "scenes/missing")])), encoding="utf-8")
    payload = _run(tmp_path, reg)
    assert payload["per_scene"][0]["status"] == "BLOCKED"


def test_missing_required_file_is_fail(tmp_path: Path):
    (tmp_path / "scenes/a").mkdir(parents=True)
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([_entry("a", "scenes/a")])), encoding="utf-8")
    payload = _run(tmp_path, reg)
    assert payload["per_scene"][0]["status"] == "FAIL"
    assert "missing_required_file: environment.yaml" in payload["per_scene"][0]["blockers"]


def test_experimental_included_with_flag(tmp_path: Path):
    _write_scene(tmp_path / "scenes/exp", "exp")
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([_entry("exp", "scenes/exp", support_level="experimental")])), encoding="utf-8")
    payload = _run(tmp_path, reg, ["--include-experimental"])
    assert payload["per_scene"][0]["status"] in {"PASS", "PASS_WITH_WARNINGS"}


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


def test_catalog_blocked_scene_reports_known_blocker_without_guided_validator(tmp_path: Path):
    _write_scene(tmp_path / "scenes/blocked_scene", "blocked_scene")
    reg = tmp_path / "registry.yaml"
    blocker = "tracked upstream MoveIt config issue"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry("blocked_scene", "scenes/blocked_scene", status="blocked", known_blocker=blocker),
    ])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]
    assert row["status"] == "BLOCKED"
    assert row["validation_result"] == "BLOCKED"
    assert row["blocker"] == blocker
    assert row["known_blocker"] == blocker
    assert row["blockers"][0] == blocker
    assert row["static_validation"] == {"status": "PASS", "missing_files": []}
    assert row["guided_build_launch_readiness"] == {"status": "BLOCKED", "blockers": [blocker]}
    assert row["commands_run"] == []


def test_catalog_blocked_scene_keeps_static_missing_file_details(tmp_path: Path):
    (tmp_path / "scenes/blocked_missing_files").mkdir(parents=True)
    reg = tmp_path / "registry.yaml"
    blocker = "intentionally parked until generated assets are restored"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry("blocked_missing_files", "scenes/blocked_missing_files", status="blocked", known_blocker=blocker),
    ])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]
    assert row["status"] == "BLOCKED"
    assert row["validation_result"] == "BLOCKED"
    assert row["blocker"] == blocker
    assert row["known_blocker"] == blocker
    assert row["static_validation"]["status"] == "FAIL"
    assert "environment.yaml" in row["static_validation"]["missing_files"]
    assert "missing_required_file: environment.yaml" in row["blockers"]
    assert row["commands_run"] == []


def test_supported_scene_with_stale_known_blocker_is_catalog_error(tmp_path: Path):
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry("stale_blocker", "scenes/stale_blocker", status="supported", known_blocker="old fixed issue"),
    ])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    assert payload["summary"]["blocked"] == 1
    assert "stale_blocker: known_blocker must be empty when status is supported" in payload["catalog_errors"]


def test_blocked_scene_without_known_blocker_is_catalog_error(tmp_path: Path):
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry("blocked_without_reason", "scenes/blocked_without_reason", status="blocked", known_blocker=""),
    ])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    assert payload["summary"]["blocked"] == 1
    assert "blocked_without_reason: known_blocker must be non-empty when status is blocked" in payload["catalog_errors"]


def test_unknown_catalog_status_and_support_level_are_catalog_errors(tmp_path: Path):
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry("bad_catalog_values", "scenes/bad_catalog_values", status="parked", support_level="beta"),
    ])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    assert payload["summary"]["blocked"] == 1
    assert "bad_catalog_values: unknown status 'parked'; accepted values: blocked, disabled, supported" in payload["catalog_errors"]
    assert "bad_catalog_values: unknown support_level 'beta'; accepted values: experimental, supported" in payload["catalog_errors"]
