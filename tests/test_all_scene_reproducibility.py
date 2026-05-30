from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

def test_validator_emits_per_scene_report_and_contract_keys():
    repo_root = Path(__file__).resolve().parents[1]
    script = repo_root / "scripts" / "validate_all_workcell_studio_scenes.py"
    report = repo_root / "build" / "workcell_studio" / "all_scene_reproducibility_report.json"

    if report.exists():
        report.unlink()

    proc = subprocess.run([sys.executable, str(script)], cwd=repo_root, capture_output=True, text=True)

    assert report.exists(), proc.stdout + "\n" + proc.stderr
    payload = json.loads(report.read_text(encoding="utf-8"))
    scenes = payload.get("scenes")
    assert isinstance(scenes, list) and scenes

    import yaml

    catalog = yaml.safe_load((repo_root / "scenes" / "supported_scenes.yaml").read_text(encoding="utf-8"))
    expected_names = {entry["scene_name"] for entry in catalog["scenes"] if entry.get("enabled", True)}
    names = {s.get("scene_name") for s in scenes}
    assert expected_names == names
    assert payload.get("supported_scene_catalog", "").endswith("scenes/supported_scenes.yaml")

    expected_keys = {
        "scene_name",
        "support_level",
        "catalog_status",
        "known_blocker",
        "status",
        "files",
        "optional_files",
        "generated_mesh_index_present",
        "mesh_index_regeneration_status",
        "mesh_index_renderable_items",
        "preview_readiness_status",
        "generated_artifacts_present",
        "fake_hardware_smoke_command_available",
        "fake_hardware_smoke_command",
        "blockers",
        "warnings",
        "warning_groups",
    }
    for scene in scenes:
        assert expected_keys.issubset(scene.keys())
        assert scene["status"] in {"PASS", "WARN", "FAIL", "SKIP", "BLOCKED"}
        assert isinstance(scene["support_level"], str) and scene["support_level"]
        assert isinstance(scene["catalog_status"], str) and scene["catalog_status"]
        assert isinstance(scene["known_blocker"], str)
        assert isinstance(scene["blockers"], list)
        assert isinstance(scene["warnings"], list)

    assert "ModuleNotFoundError: No module named 'yaml'" not in (proc.stdout + proc.stderr)


def test_scene_warnings_are_grouped_and_non_ambiguous():
    repo_root = Path(__file__).resolve().parents[1]
    script = repo_root / "scripts" / "validate_all_workcell_studio_scenes.py"
    report = repo_root / "build" / "workcell_studio" / "all_scene_reproducibility_report.json"

    subprocess.run([sys.executable, str(script)], cwd=repo_root, check=True, capture_output=True, text=True)
    payload = json.loads(report.read_text(encoding="utf-8"))

    for scene in payload.get("scenes", []):
        groups = scene.get("warning_groups")
        assert isinstance(groups, dict)
        assert set(groups.keys()) == {"metadata", "preview", "generation", "launch_simulation", "runtime_smoke"}
        for key, items in groups.items():
            assert isinstance(items, list), f"{scene.get('scene_name')}:{key}"
            for msg in items:
                assert isinstance(msg, str) and msg.strip(), f"{scene.get('scene_name')}:{key}"
        for warning in scene.get("warnings", []):
            assert warning.strip()
            assert any(token in warning.lower() for token in ("missing", "issue", "degraded", "failed", "unreadable", "not in known-scenes", "skipped"))


def test_moveit_launch_readiness_uses_synthetic_launch_report_statuses(tmp_path):
    from scripts.validate_all_workcell_studio_scenes import audit_scene

    def _mk_scene(name: str, report_status: str):
        scene = tmp_path / name
        (scene / "layout").mkdir(parents=True)
        (scene / "generated").mkdir(parents=True)
        (scene / "launch").mkdir(parents=True)
        (scene / "urdf").mkdir(parents=True)
        (scene / "scene_manifest.yaml").write_text("schema_version: x\n", encoding="utf-8")
        (scene / "environment.yaml").write_text("schema_version: y\nsupport_surfaces: []\ntask_zones: []\n", encoding="utf-8")
        (scene / "layout/workcell_studio_layout.yaml").write_text("schema_version: workcell_studio_layout/v1\nitems: [{id: i1, type: marker}]\n", encoding="utf-8")
        (scene / "launch/demo.launch.py").write_text("# launch\n", encoding="utf-8")
        (scene / "urdf/scene.urdf").write_text("<robot/>\n", encoding="utf-8")
        (scene / "generated/scene_visual_mesh_index.json").write_text(json.dumps({"items": [{"render_expected": True}]}), encoding="utf-8")
        (scene / "generated/fake_hardware_smoke_launch_report.json").write_text(
            json.dumps({"schema": "fake_hardware_smoke_launch_report/v1", "result": {"status": report_status, "warnings": [], "errors": []}}),
            encoding="utf-8",
        )
        return scene

    assert audit_scene(repo_root=Path(__file__).resolve().parents[1], scene_dir=_mk_scene("s1", "PASS")).readiness["moveit_launch_readiness"].status == "PASS"
    assert audit_scene(repo_root=Path(__file__).resolve().parents[1], scene_dir=_mk_scene("s2", "WARN")).readiness["moveit_launch_readiness"].status == "WARN"
    assert audit_scene(repo_root=Path(__file__).resolve().parents[1], scene_dir=_mk_scene("s3", "FAIL")).readiness["moveit_launch_readiness"].status == "FAIL"



def _catalog_entry(name: str, scene_path: str, *, support_level: str = "supported", status: str = "supported", known_blocker: str = ""):
    from scripts.supported_scene_catalog import SupportedSceneEntry

    return SupportedSceneEntry(
        scene_name=name,
        package_name=name,
        scene_path=scene_path,
        support_level=support_level,
        status=status,
        known_blocker=known_blocker,
        authoring_files=("environment.yaml", "layout/workcell_studio_layout.yaml"),
        generated_files=("scene_manifest.yaml", "urdf/scene.urdf.xacro", "launch/demo.launch.py"),
        validation_command=f"python3 scripts/validate_builder_generated_scene.py scenes/{name} --json",
        build_package_name=name,
        build_command=f"colcon build --symlink-install --packages-select {name}",
        fake_hardware_launch_command=f"ros2 launch {name} demo.launch.py use_fake_hardware:=true launch_rviz:=true",
        enabled=True,
        raw={},
    )


def test_catalog_blocked_scene_uses_known_blocker_without_filesystem_audit(tmp_path):
    from scripts.validate_all_workcell_studio_scenes import audit_catalog_entry

    entry = _catalog_entry(
        "blocked_scene",
        "scenes/blocked_scene",
        status="blocked",
        known_blocker="missing vendor MoveIt config",
    )

    audit = audit_catalog_entry(
        tmp_path,
        entry,
        include_experimental=False,
        regenerate_missing_mesh_indexes=False,
    )

    assert audit.scene_name == "blocked_scene"
    assert audit.support_level == "supported"
    assert audit.catalog_status == "blocked"
    assert audit.known_blocker == "missing vendor MoveIt config"
    assert audit.status == "BLOCKED"
    assert audit.blockers == ["missing vendor MoveIt config"]
    assert audit.mesh_index_regeneration_status == "not_attempted_catalog_only"
    assert audit.readiness["artifact_readiness"].status == "BLOCKED"


def test_experimental_scene_skips_by_default_and_audits_with_opt_in(tmp_path):
    from scripts.validate_all_workcell_studio_scenes import audit_catalog_entry

    scene = tmp_path / "scenes" / "experimental_scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir(parents=True)
    (scene / "launch").mkdir(parents=True)
    (scene / "urdf").mkdir(parents=True)
    (scene / "scene_manifest.yaml").write_text("schema_version: x\n", encoding="utf-8")
    (scene / "environment.yaml").write_text("schema_version: y\n", encoding="utf-8")
    (scene / "layout/workcell_studio_layout.yaml").write_text(
        "schema_version: workcell_studio_layout/v1\nitems: [{id: i1, type: marker}]\n",
        encoding="utf-8",
    )
    (scene / "launch/demo.launch.py").write_text("# launch\n", encoding="utf-8")
    (scene / "urdf/scene.urdf").write_text("<robot/>\n", encoding="utf-8")
    (scene / "generated/scene_visual_mesh_index.json").write_text(
        json.dumps({"items": [{"render_expected": True}]}),
        encoding="utf-8",
    )
    entry = _catalog_entry("experimental_scene", "scenes/experimental_scene", support_level="experimental")

    skipped = audit_catalog_entry(
        tmp_path,
        entry,
        include_experimental=False,
        regenerate_missing_mesh_indexes=False,
    )
    audited = audit_catalog_entry(
        tmp_path,
        entry,
        include_experimental=True,
        regenerate_missing_mesh_indexes=False,
    )

    assert skipped.status == "SKIP"
    assert skipped.support_level == "experimental"
    assert "--include-experimental" in skipped.warnings[0]
    assert audited.status in {"PASS", "WARN"}
    assert audited.support_level == "experimental"


def test_missing_mesh_index_regeneration_is_opt_in(tmp_path, monkeypatch):
    from scripts import validate_all_workcell_studio_scenes as validator

    scene = tmp_path / "scene_without_index"
    (scene / "layout").mkdir(parents=True)
    (scene / "launch").mkdir(parents=True)
    (scene / "urdf").mkdir(parents=True)
    (scene / "scene_manifest.yaml").write_text("schema_version: x\n", encoding="utf-8")
    (scene / "environment.yaml").write_text("schema_version: y\n", encoding="utf-8")
    (scene / "layout/workcell_studio_layout.yaml").write_text(
        "schema_version: workcell_studio_layout/v1\nitems: [{id: i1, type: marker}]\n",
        encoding="utf-8",
    )
    (scene / "launch/demo.launch.py").write_text("# launch\n", encoding="utf-8")
    (scene / "urdf/scene.urdf").write_text("<robot/>\n", encoding="utf-8")

    def _unexpected_regeneration(*_args, **_kwargs):
        raise AssertionError("mesh index regeneration should require explicit opt-in")

    monkeypatch.setattr(validator, "_run_extract_for_scene", _unexpected_regeneration)

    audit = validator.audit_scene(repo_root=tmp_path, scene_dir=scene)

    assert audit.mesh_index_regeneration_status == "skipped_missing_regeneration_disabled"
    assert any("--regenerate-missing-mesh-indexes" in blocker for blocker in audit.blockers)
