from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE = REPO_ROOT / "scenes" / "ur5_2f_test"
SCRIPTS = REPO_ROOT / "scripts"

sys.path.insert(0, str(SCRIPTS))
from workcell_studio_layout_source import inspect_saved_layout  # noqa: E402


def test_golden_scene_has_one_canonical_authored_layout_and_task_intent() -> None:
    assert (SCENE / "layout" / "workcell_studio_layout.yaml").is_file()
    assert not (SCENE / "environment_layout.yaml").exists()
    assert (SCENE / "config" / "workcell_builder_task_intent.yaml").is_file()
    cell = yaml.safe_load((SCENE / "cell_definition.yaml").read_text(encoding="utf-8"))
    assert cell["environment"]["layout"] == "layout/workcell_studio_layout.yaml"
    assert inspect_saved_layout(SCENE)["source"] == "canonical"


def test_modern_file_output_and_state_audits_accept_golden_scene(tmp_path: Path) -> None:
    file_report = tmp_path / "file_output.json"
    state_report = tmp_path / "state.json"
    subprocess.run(
        [sys.executable, str(SCRIPTS / "audit_new_cell_file_outputs.py"), "--scene-dir", str(SCENE),
         "--scene-name", "ur5_2f_test", "--json-out", str(file_report)],
        check=True, capture_output=True, text=True,
    )
    subprocess.run(
        [sys.executable, str(SCRIPTS / "audit_new_cell_state_transitions.py"), "--scene-dir", str(SCENE),
         "--scene-name", "ur5_2f_test", "--json-out", str(state_report)],
        check=True, capture_output=True, text=True,
    )
    file_payload = json.loads(file_report.read_text(encoding="utf-8"))
    state_payload = json.loads(state_report.read_text(encoding="utf-8"))
    assert file_payload["file_output_status"] == "PASS"
    assert file_payload["saved_layout"]["source"] == "canonical"
    assert "LAYOUT_SAVED" in state_payload["completed_states"]
    assert state_payload["state_conditions"]["saved_layout_source"] == "canonical"


def test_legacy_only_layout_remains_an_explicit_compatibility_fallback(tmp_path: Path) -> None:
    (tmp_path / "environment_layout.yaml").write_text(
        "schema_version: environment_layout/v1\nassets: []\n", encoding="utf-8"
    )
    result = inspect_saved_layout(tmp_path)
    assert result["saved"] is True
    assert result["source"] == "legacy_fallback"
    assert result["legacy_fallback"] is True


def test_modern_validation_does_not_recreate_legacy_authored_layout(tmp_path: Path) -> None:
    scene_copy = tmp_path / "ur5_2f_test"
    shutil.copytree(SCENE, scene_copy)
    legacy = scene_copy / "environment_layout.yaml"
    assert not legacy.exists()
    subprocess.run(
        [sys.executable, str(SCRIPTS / "validate_builder_generated_scene.py"), str(scene_copy), "--json"],
        check=True, capture_output=True, text=True,
    )
    assert not legacy.exists()


def test_readiness_helpers_resolve_the_canonical_layout_first() -> None:
    readiness = (SCRIPTS / "generate_workcell_studio_readiness_pack.py").read_text(encoding="utf-8")
    intent_validator = (SCRIPTS / "validate_builder_task_intent.py").read_text(encoding="utf-8")
    assert "resolve_saved_layout_path(scene)" in readiness
    assert "str(readiness_layout_path)" in readiness
    assert "CANONICAL_LAYOUT_REL" in intent_validator
    assert 'for rel in [CANONICAL_LAYOUT_REL, "environment.yaml"' in intent_validator
