from __future__ import annotations

from pathlib import Path

import yaml

from scripts import run_workcell_studio_scene_readiness_matrix as matrix


def test_missing_optional_scene3d_manifest_reference_does_not_fail(tmp_path: Path):
    scene = tmp_path / "scene"
    scene.mkdir()
    (scene / "environment.yaml").write_text("frame: world\n", encoding="utf-8")
    (scene / "scene_manifest.yaml").write_text(
        yaml.safe_dump(
            {
                "files": {
                    "environment": "environment.yaml",
                    "scene3d_gui_smoke": "generated/scene3d_gui_smoke.json",
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    result = matrix._check_manifest_refs(scene)

    assert result["status"] == matrix.PASS
    assert result["optional_scene3d_debug_evidence"] is True
    assert result["optional_debug_references"][0]["field"] == "files.scene3d_gui_smoke"


def test_required_manifest_reference_still_fails_when_scene3d_debug_is_also_missing(tmp_path: Path):
    scene = tmp_path / "scene"
    scene.mkdir()
    (scene / "scene_manifest.yaml").write_text(
        yaml.safe_dump(
            {
                "files": {
                    "environment": "missing_environment.yaml",
                    "scene3d_gui_smoke": "generated/scene3d_gui_smoke.json",
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    result = matrix._check_manifest_refs(scene)

    assert result["status"] == matrix.FAIL
    assert any(item["field"] == "files.environment" for item in result["missing"])


def test_scene3d_failures_are_preserved_as_optional_diagnostics(monkeypatch, tmp_path: Path):
    visual = {
        "status": matrix.FAIL,
        "message": "legacy topology mismatch",
        "blockers": ["scene3d_rendered_mesh_adjacency_failed"],
    }
    physical = {
        "status": matrix.FAIL,
        "message": "smoke evidence missing",
        "blockers": ["screenshot_missing"],
    }
    monkeypatch.setattr(matrix, "_ORIGINAL_CHECK_SCENE3D", lambda _name, _dir: (visual, physical))

    visual_result, physical_result = matrix._check_scene3d("fixture", tmp_path)

    for result in (visual_result, physical_result):
        assert result["status"] == matrix.PASS
        assert result["debug_status"] == matrix.FAIL
        assert result["production_readiness_gate"] is False
        assert result["optional_debug_evidence"] is True
        assert result["blockers"] == []
        assert result["diagnostic_blockers"]
