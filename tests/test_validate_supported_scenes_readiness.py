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


def test_missing_mesh_index_reports_mesh_validation_issue(tmp_path: Path):
    scene = tmp_path / "scenes/missing_mesh"
    _write_scene(scene, "missing_mesh")
    (scene / "generated/scene_visual_mesh_index.json").unlink()
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([_entry("missing_mesh", "scenes/missing_mesh")])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["status"] == "FAIL"
    assert row["mesh_index_validation"]["status"] == "FAIL"
    assert row["mesh_index_validation"]["issue_code"] == "missing_file"
    assert "mesh_index_validation: missing_file" in row["blockers"]


def test_malformed_mesh_index_item_entries_block_supported_scene(tmp_path: Path):
    scene = tmp_path / "scenes/malformed_mesh"
    _write_scene(scene, "malformed_mesh")
    (scene / "generated/scene_visual_mesh_index.json").write_text(
        json.dumps({"visual_items": [{"render_expected": True}, "not-an-object"]}),
        encoding="utf-8",
    )
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([_entry("malformed_mesh", "scenes/malformed_mesh")])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["status"] == "FAIL"
    assert row["mesh_index_validation"]["status"] == "FAIL"
    assert row["mesh_index_validation"]["issue_code"] == "malformed_item_entries"
    assert row["mesh_index_validation"]["malformed_item_count"] == 1
    assert "mesh_index_validation: malformed_item_entries" in row["blockers"]


def test_zero_renderable_mesh_index_blocks_supported_scene(tmp_path: Path):
    scene = tmp_path / "scenes/zero_mesh"
    _write_scene(scene, "zero_mesh")
    (scene / "generated/scene_visual_mesh_index.json").write_text(
        json.dumps({"visual_items": [{"render_expected": False}]}),
        encoding="utf-8",
    )
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([_entry("zero_mesh", "scenes/zero_mesh")])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["status"] == "FAIL"
    assert row["mesh_index_validation"]["status"] == "FAIL"
    assert row["mesh_index_validation"]["issue_code"] == "no_renderable_items"
    assert row["mesh_index_validation"]["renderable_item_count"] == 0
    assert "mesh_index_validation: no_renderable_items" in row["blockers"]


def test_blocked_catalog_scene_keeps_known_blocker_and_reports_mesh_issue(tmp_path: Path):
    scene = tmp_path / "scenes/blocked_mesh"
    _write_scene(scene, "blocked_mesh")
    (scene / "generated/scene_visual_mesh_index.json").write_text(
        json.dumps({"visual_items": []}),
        encoding="utf-8",
    )
    reg = tmp_path / "registry.yaml"
    reg.write_text(yaml.safe_dump(_catalog([
        _entry(
            "blocked_mesh",
            "scenes/blocked_mesh",
            status="blocked",
            known_blocker="waiting on upstream asset export",
        )
    ])), encoding="utf-8")

    payload = _run(tmp_path, reg)
    row = payload["per_scene"][0]

    assert row["known_blocker"] == "waiting on upstream asset export"
    assert row["mesh_index_validation"]["issue_code"] == "no_renderable_items"
    assert "mesh_index_validation: no_renderable_items" in row["warnings"]
    assert "mesh_index_validation: no_renderable_items" not in row["blockers"]
