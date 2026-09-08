from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    return mod


validator = _load("validate_builder_generated_scene", REPO_ROOT / "scripts" / "validate_builder_generated_scene.py")
acceptance_validator = _load(
    "validate_workcell_studio_generated_scene_test",
    REPO_ROOT / "scripts" / "validate_workcell_studio_generated_scene.py",
)


def _write_required_scene_files(scene_root: Path) -> None:
    (scene_root / "package.xml").write_text("<package/>", encoding="utf-8")
    (scene_root / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.5)", encoding="utf-8")
    (scene_root / "environment.yaml").write_text(
        "robot: {name: ur5}\nend_effector: {name: robotiq}\nobjects: {}\n",
        encoding="utf-8",
    )
    (scene_root / "layout").mkdir(exist_ok=True)
    (scene_root / "layout" / "workcell_studio_layout.yaml").write_text(
        "schema_version: workcell_studio_layout/v1\nitems: []\n",
        encoding="utf-8",
    )


def test_before_export_artifacts_are_warn_not_fail(tmp_path: Path) -> None:
    _write_required_scene_files(tmp_path)
    (tmp_path / "workcell_builder_metadata.yaml").write_text("{}", encoding="utf-8")

    report = validator.validate_scene(tmp_path)

    assert report["ok"] is True
    assert report["readiness_classification"] == "physical_scene_only"
    assert any("generated/cell_definition.yaml missing; run ./generated/export_workcell_studio_sources.sh" in w for w in report["warnings"])
    assert all("cell_definition.yaml" not in e for e in report["errors"])
    assert all("environment_layout.yaml" not in e for e in report["errors"])


def test_require_generated_fails_when_cell_definition_handoff_is_missing(tmp_path: Path) -> None:
    _write_required_scene_files(tmp_path)
    generated = tmp_path / "generated"
    generated.mkdir()

    report = validator.validate_scene(tmp_path, require_generated=True)

    assert report["ok"] is False
    assert any("generated/cell_definition.yaml missing" in error for error in report["errors"])
    checks = {check["check"]: check for check in report["checks"]}
    assert checks["generated/cell_definition.yaml present"].get("optional") is False
    assert checks["generated/environment_layout.yaml legacy export present"].get("optional") is True


@pytest.mark.parametrize(
    ("filename", "invalid_content", "expected_error"),
    [
        ("cell_definition.yaml", "schema_version: wrong\n", "generated/cell_definition.yaml validation failed"),
        ("canonical_layout", "schema_version: wrong\nitems: []\n", "Invalid layout/workcell_studio_layout.yaml"),
    ],
)
def test_require_generated_fails_when_canonical_handoff_is_invalid(
    tmp_path: Path, filename: str, invalid_content: str, expected_error: str
) -> None:
    _write_required_scene_files(tmp_path)
    generated = tmp_path / "generated"
    generated.mkdir()
    (generated / "cell_definition.yaml").write_text("schema_version: cell_definition/v1\n", encoding="utf-8")
    if filename == "canonical_layout":
        (tmp_path / "layout" / "workcell_studio_layout.yaml").write_text(invalid_content, encoding="utf-8")
    else:
        (generated / filename).write_text(invalid_content, encoding="utf-8")

    report = validator.validate_scene(tmp_path, require_generated=True)

    assert report["ok"] is False
    assert any(expected_error in error for error in report["errors"])


def test_missing_legacy_export_does_not_fail_modern_scene(tmp_path: Path) -> None:
    _write_required_scene_files(tmp_path)
    (tmp_path / "workcell_builder_metadata.yaml").write_text("{}", encoding="utf-8")
    report = validator.validate_scene(tmp_path)

    checks = {c["check"]: c for c in report["checks"]}
    assert checks["generated/cell_definition.yaml present"]["ok"] is False
    assert checks["generated/environment_layout.yaml legacy export present"]["ok"] is False
    assert report["ok"] is True


def test_missing_required_package_xml_fails(tmp_path: Path) -> None:
    (tmp_path / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.5)", encoding="utf-8")
    (tmp_path / "environment.yaml").write_text("robot: {name: ur5}\n", encoding="utf-8")

    report = validator.validate_scene(tmp_path)

    assert report["ok"] is False
    assert any("Missing required file: package.xml" in e for e in report["errors"])


def test_missing_task_intent_is_warn_and_physical_scene_only(tmp_path: Path) -> None:
    _write_required_scene_files(tmp_path)
    (tmp_path / "workcell_builder_metadata.yaml").write_text("{}", encoding="utf-8")

    report = validator.validate_scene(tmp_path)

    assert report["ok"] is True
    assert report["readiness"] == "physical_scene_only"
    assert any("Task intent missing: physical scene only." in w for w in report["warnings"])


def test_successful_cli_gate_persists_existing_workflow_readiness_artifact(tmp_path: Path) -> None:
    report = {
        "ok": True,
        "readiness": "task_recipe_generated",
        "runtime_readiness": "runtime_possible",
        "warnings": [],
        "errors": [],
        "checks": [{"check": "package.xml exists", "ok": True}],
        "acceptance": {"status": "PASS"},
    }

    artifact = validator._sync_validation_artifact(tmp_path, report)

    assert artifact == tmp_path / "validation" / "readiness_report.json"
    payload = json.loads(artifact.read_text(encoding="utf-8"))
    assert payload["schema"] == "workcell_builder_generated_scene_validation/v1"
    assert payload["status"] == "PASS"
    assert payload["ok"] is True
    assert payload["acceptance_status"] == "PASS"


def test_failed_cli_gate_removes_stale_workflow_success_artifact(tmp_path: Path) -> None:
    artifact = tmp_path / "validation" / "readiness_report.json"
    artifact.parent.mkdir(parents=True)
    artifact.write_text('{"status":"PASS"}\n', encoding="utf-8")

    returned = validator._sync_validation_artifact(
        tmp_path, {"ok": False, "errors": ["broken scene"]}
    )

    assert returned == artifact
    assert not artifact.exists()


def test_acceptance_sync_refreshes_home_contract(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    acceptance = {
        "scene_name": "ur5_2f_test",
        "status": "PASS",
        "blockers": [],
        "authored_input_fingerprint": "abc123",
    }

    def fake_run(args, capture_output, text, check):
        assert args[1].endswith("validate_workcell_studio_generated_scene.py")
        assert args[-1] == "--json"
        artifact = tmp_path / "acceptance" / "generated_scene_acceptance.json"
        artifact.parent.mkdir(parents=True, exist_ok=True)
        artifact.write_text(json.dumps(acceptance), encoding="utf-8")
        return subprocess.CompletedProcess(args, 0, json.dumps(acceptance), "")

    monkeypatch.setattr(validator.subprocess, "run", fake_run)

    report, error = validator._sync_acceptance_artifact(tmp_path)

    assert error is None
    assert report["status"] == "PASS"
    assert (tmp_path / "acceptance" / "generated_scene_acceptance.json").is_file()


def test_acceptance_failure_clears_stale_home_pass(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    artifact = tmp_path / "acceptance" / "generated_scene_acceptance.json"
    artifact.parent.mkdir(parents=True)
    artifact.write_text('{"status":"PASS"}\n', encoding="utf-8")
    blocked = {"status": "BLOCKED", "blockers": ["unsafe"]}

    monkeypatch.setattr(
        validator.subprocess,
        "run",
        lambda args, capture_output, text, check: subprocess.CompletedProcess(
            args, 1, json.dumps(blocked), ""
        ),
    )

    report, error = validator._sync_acceptance_artifact(tmp_path)

    assert report["status"] == "BLOCKED"
    assert "unsafe" in (error or "")
    assert not artifact.exists()


def test_generated_asset_metadata_does_not_stale_authored_acceptance_fingerprint(tmp_path: Path) -> None:
    _write_required_scene_files(tmp_path)
    (tmp_path / "scene_manifest.yaml").write_text("scene: {package: sample_scene}\n", encoding="utf-8")
    (tmp_path / "urdf").mkdir()
    authored_urdf = tmp_path / "urdf" / "scene.urdf.xacro"
    authored_urdf.write_text("<robot name='sample'/>\n", encoding="utf-8")

    before = acceptance_validator.authored_input_fingerprint(tmp_path)
    derived = tmp_path / "urdf" / "generated_asset_metadata.yaml"
    derived.write_text("schema: generated_asset_metadata/v1\nrevision: 1\n", encoding="utf-8")
    after_create = acceptance_validator.authored_input_fingerprint(tmp_path)
    derived.write_text("schema: generated_asset_metadata/v1\nrevision: 2\n", encoding="utf-8")
    after_refresh = acceptance_validator.authored_input_fingerprint(tmp_path)

    assert before == after_create == after_refresh

    authored_urdf.write_text("<robot name='sample_changed'/>\n", encoding="utf-8")
    assert acceptance_validator.authored_input_fingerprint(tmp_path) != before


def test_home_browser_uses_the_same_generator_owned_fingerprint_exclusion():
    source = (
        REPO_ROOT
        / "workcell_builder"
        / "workcell_builder"
        / "src_workcell_studio_scene_browser.cpp"
    ).read_text(encoding="utf-8")

    assert 'relative.generic_string() == "urdf/generated_asset_metadata.yaml"' in source
    assert "is_generator_owned_derived_input(candidate)" in source
