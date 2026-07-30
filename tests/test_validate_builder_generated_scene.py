from __future__ import annotations

import importlib.util
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


def _write_required_scene_files(scene_root: Path) -> None:
    (scene_root / "package.xml").write_text("<package/>", encoding="utf-8")
    (scene_root / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.5)", encoding="utf-8")
    (scene_root / "environment.yaml").write_text(
        "robot: {name: ur5}\nend_effector: {name: robotiq}\nobjects: {}\n",
        encoding="utf-8",
    )


def test_before_export_artifacts_are_warn_not_fail(tmp_path: Path) -> None:
    _write_required_scene_files(tmp_path)
    (tmp_path / "workcell_builder_metadata.yaml").write_text("{}", encoding="utf-8")

    report = validator.validate_scene(tmp_path)

    assert report["ok"] is True
    assert report["readiness_classification"] == "physical_scene_only"
    assert any("generated/cell_definition.yaml missing; run ./generated/export_workcell_studio_sources.sh" in w for w in report["warnings"])
    assert any("generated/environment_layout.yaml missing; run ./generated/export_workcell_studio_sources.sh" in w for w in report["warnings"])
    assert all("cell_definition.yaml" not in e for e in report["errors"])
    assert all("environment_layout.yaml" not in e for e in report["errors"])


@pytest.mark.parametrize(
    ("present_filename", "missing_filename"),
    [
        ("cell_definition.yaml", "environment_layout.yaml"),
        ("environment_layout.yaml", "cell_definition.yaml"),
    ],
)
def test_require_generated_fails_when_either_canonical_handoff_is_missing(
    tmp_path: Path, present_filename: str, missing_filename: str
) -> None:
    _write_required_scene_files(tmp_path)
    generated = tmp_path / "generated"
    generated.mkdir()
    schema = "cell_definition/v1" if present_filename == "cell_definition.yaml" else "environment_layout/v1"
    (generated / present_filename).write_text(f"schema_version: {schema}\n", encoding="utf-8")

    report = validator.validate_scene(tmp_path, require_generated=True)

    assert report["ok"] is False
    assert any(f"generated/{missing_filename} missing" in error for error in report["errors"])
    checks = {check["check"]: check for check in report["checks"]}
    assert checks[f"generated/{missing_filename} present"].get("optional") is False


@pytest.mark.parametrize(
    ("filename", "invalid_content", "expected_error"),
    [
        ("cell_definition.yaml", "schema_version: wrong\n", "generated/cell_definition.yaml validation failed"),
        ("environment_layout.yaml", "schema_version: wrong\n", "generated/environment_layout.yaml validation failed"),
    ],
)
def test_require_generated_fails_when_canonical_handoff_is_invalid(
    tmp_path: Path, filename: str, invalid_content: str, expected_error: str
) -> None:
    _write_required_scene_files(tmp_path)
    generated = tmp_path / "generated"
    generated.mkdir()
    (generated / "cell_definition.yaml").write_text("schema_version: cell_definition/v1\n", encoding="utf-8")
    (generated / "environment_layout.yaml").write_text("schema_version: environment_layout/v1\n", encoding="utf-8")
    (generated / filename).write_text(invalid_content, encoding="utf-8")

    report = validator.validate_scene(tmp_path, require_generated=True)

    assert report["ok"] is False
    assert any(expected_error in error for error in report["errors"])


def test_after_export_artifacts_present_are_pass(tmp_path: Path) -> None:
    _write_required_scene_files(tmp_path)
    (tmp_path / "workcell_builder_metadata.yaml").write_text("{}", encoding="utf-8")
    generated = tmp_path / "generated"
    generated.mkdir()
    (generated / "cell_definition.yaml").write_text("schema_version: cell_definition/v1\n", encoding="utf-8")
    (generated / "environment_layout.yaml").write_text("schema_version: environment_layout/v1\n", encoding="utf-8")

    report = validator.validate_scene(tmp_path)

    checks = {c["check"]: c for c in report["checks"]}
    assert checks["generated/cell_definition.yaml present"]["ok"] is True
    assert checks["generated/environment_layout.yaml present"]["ok"] is True


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
