from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "run_cross_scene_product_view_save_roundtrip_acceptance.py"


def _run(tmp_path: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(SCRIPT), "--output", str(tmp_path / "report.json"), *args],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def test_required_cross_scene_roundtrip_acceptance_passes_and_reports_machine_readable_results(tmp_path: Path) -> None:
    result = _run(tmp_path)
    assert result.returncode == 0, result.stderr
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert report["schema_version"] == "workcell_studio_product_view_save_roundtrip_acceptance/v1"
    statuses = {row["scene"]: row["status"] for row in report["results"]}
    assert set(statuses) == {"ur5_2f_test", "ur5_3f_test", "suction_test"}
    assert set(statuses.values()) <= {"PASS", "NOT_APPLICABLE"}
    assert "supported_scenes.yaml" in report["supported_scene_catalog"]
    for row in report["results"]:
        if row["status"] == "PASS":
            check_statuses = {check["name"]: check["status"] for check in row["checks"]}
            for required in [
                "generate_temp_scene",
                "validate_edit_patch",
                "save_via_existing_edit_patch_workflow",
                "regenerate_reload_edited_pose",
                "stable_identity_preserved",
                "repository_source_scene_unchanged",
                "reject_locked_or_generated_edit",
                "reject_stale_scene_edit",
                "dirty_after_edit",
                "undo_clears_dirty",
                "redo_restores_dirty",
                "save_clears_dirty",
            ]:
                assert check_statuses[required] == "PASS", (row["scene"], required, row)
        else:
            assert row["status"] == "NOT_APPLICABLE"
            assert "no editable" in row["reason"]


def test_missing_registry_scene_is_blocked_not_false_pass(tmp_path: Path) -> None:
    result = _run(tmp_path, "--scene", "does_not_exist")
    assert result.returncode == 1
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert report["results"] == [
        {"scene": "does_not_exist", "status": "BLOCKED", "reason": "scene not present in supported-scene registry"}
    ]
