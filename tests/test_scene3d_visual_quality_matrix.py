from __future__ import annotations

import json
import subprocess
from pathlib import Path

import yaml

import scripts.run_scene3d_visual_quality_screenshots as screenshots
import scripts.validate_scene3d_visual_quality_matrix as matrix

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "validate_scene3d_visual_quality_matrix.py"


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _write_catalog(repo: Path, entry: dict) -> Path:
    catalog = repo / "scenes" / "supported_scenes.yaml"
    catalog.parent.mkdir(parents=True, exist_ok=True)
    catalog.write_text(
        yaml.safe_dump({"schema_version": "workcell_studio_supported_scenes/v1", "scenes": [entry]}),
        encoding="utf-8",
    )
    return catalog


def _write_scene(repo: Path, name: str, mesh_index: dict, smoke: dict) -> Path:
    scene_dir = repo / "scenes" / name
    _write_json(scene_dir / "generated" / "scene_visual_mesh_index.json", mesh_index)
    _write_json(scene_dir / "generated" / "scene3d_gui_smoke.json", smoke)
    return scene_dir


def _mesh_and_primitive_index() -> dict:
    return {
        "safe_for_preview": True,
        "visual_items": [
            {"id": "mesh_item", "geometry": {"mesh": {"filename": "package://demo/meshes/tool.stl"}}},
            {"id": "primitive_item", "geometry": {"box": {"size": [1.0, 1.0, 0.1]}}},
        ],
    }


def _passing_smoke() -> dict:
    return {
        "schema": "workcell_studio_scene3d_gui_smoke/v1",
        "status": "PASS",
        "render_debug_counters": {
            "rendered_count": 2,
            "mesh_rendered_count": 1,
            "primitive_rendered_count": 1,
            "placeholder_count": 0,
            "wireframe_fallback_count": 0,
        },
    }


def test_visual_quality_matrix_emits_required_fields_and_synthetic_fixture_passes(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    scene_dir = _write_scene(repo, "demo_scene", _mesh_and_primitive_index(), _passing_smoke())
    catalog = _write_catalog(
        repo,
        {
            "scene_name": "demo_scene",
            "package_name": "demo_scene",
            "scene_path": "scenes/demo_scene",
            "support_level": "supported",
            "status": "supported",
            "enabled": True,
        },
    )
    fixture = _write_scene(repo, "synthetic_visual_quality_fixture", _mesh_and_primitive_index(), _passing_smoke())

    payload = matrix.build_matrix(
        repo_root=repo,
        supported_scenes=catalog,
        synthetic_fixture=fixture,
    )

    assert payload["pass"] is True
    assert {scene["scene_name"] for scene in payload["scenes"]} == {"demo_scene", "synthetic_visual_quality_fixture"}
    for scene in payload["scenes"]:
        for field in (
            "scene_name",
            "total_payload_count",
            "mesh_source_count",
            "mesh_rendered_count",
            "primitive_source_count",
            "primitive_rendered_count",
            "physical_rendered_count",
            "placeholder_count",
            "raw_generated_bounds_count",
            "missing_geometry_box_count",
            "diagnostic_fallback_count",
            "missing_geometry_count",
            "wireframe_fallback_count",
            "visual_quality_status",
            "warnings",
            "blocker_reasons",
        ):
            assert field in scene
        assert scene["mesh_source_count"] == 1
        assert scene["mesh_rendered_count"] == 1
        assert scene["primitive_source_count"] == 1
        assert scene["primitive_rendered_count"] == 1
        assert scene["physical_rendered_count"] == 2
        assert scene["placeholder_count"] == 0
        assert scene["diagnostic_fallback_count"] == 0
        assert scene["visual_quality_status"] == "PASS"
        assert scene["blocker_reasons"] == []


def test_visual_quality_matrix_rejects_rendered_count_only_and_missing_primitive_render_counter(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    _write_scene(
        repo,
        "bad_scene",
        _mesh_and_primitive_index(),
        {
            "schema": "workcell_studio_scene3d_gui_smoke/v1",
            "status": "PASS",
            "counters": {
                "rendered_count": 2,
                "mesh_rendered_count": 1,
                "placeholder_count": 0,
            },
        },
    )
    catalog = _write_catalog(
        repo,
        {
            "scene_name": "bad_scene",
            "package_name": "bad_scene",
            "scene_path": "scenes/bad_scene",
            "support_level": "supported",
            "status": "supported",
            "enabled": True,
        },
    )
    fixture = _write_scene(repo, "synthetic_visual_quality_fixture", _mesh_and_primitive_index(), _passing_smoke())

    payload = matrix.build_matrix(repo_root=repo, supported_scenes=catalog, synthetic_fixture=fixture)
    bad_scene = next(scene for scene in payload["scenes"] if scene["scene_name"] == "bad_scene")

    assert payload["pass"] is False
    assert bad_scene["rendered_count"] == bad_scene["total_payload_count"]
    assert bad_scene["visual_quality_status"] == "FAIL"
    assert "primitive_source_count > 0 requires primitive_rendered_count > 0" in bad_scene["blockers"]
    assert any("rendered_count equals total_payload_count" in warning for warning in bad_scene["warnings"])


def test_visual_quality_matrix_rejects_primitive_placeholders_and_wireframe_dominance(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    _write_scene(
        repo,
        "wireframe_scene",
        _mesh_and_primitive_index(),
        {
            "schema": "workcell_studio_scene3d_gui_smoke/v1",
            "status": "PASS",
            "render_debug_counters": {
                "rendered_count": 4,
                "mesh_rendered_count": 1,
                "primitive_rendered_count": 1,
                "placeholder_count": 1,
                "wireframe_fallback_count": 4,
            },
        },
    )
    catalog = _write_catalog(
        repo,
        {
            "scene_name": "wireframe_scene",
            "package_name": "wireframe_scene",
            "scene_path": "scenes/wireframe_scene",
            "support_level": "supported",
            "status": "supported",
            "enabled": True,
        },
    )
    fixture = _write_scene(repo, "synthetic_visual_quality_fixture", _mesh_and_primitive_index(), _passing_smoke())

    payload = matrix.build_matrix(repo_root=repo, supported_scenes=catalog, synthetic_fixture=fixture)
    scene = next(scene for scene in payload["scenes"] if scene["scene_name"] == "wireframe_scene")

    assert scene["visual_quality_status"] == "FAIL"
    assert "valid URDF primitives must not increment placeholder_count" in scene["blockers"]
    assert "wireframe_fallback_count dominates visible physical items" in scene["blockers"]



def test_visual_quality_matrix_rejects_raw_generated_bounds_only_visible_evidence(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    scene_dir = _write_scene(
        repo,
        "raw_bounds_only_scene",
        _mesh_and_primitive_index(),
        {
            "schema": "workcell_studio_scene3d_gui_smoke/v1",
            "status": "PASS",
            "render_debug_counters": {
                "rendered_count": 2,
                "mesh_rendered_count": 0,
                "primitive_rendered_count": 0,
                "generated_fallback_count": 2,
                "placeholder_count": 0,
                "wireframe_fallback_count": 0,
            },
        },
    )
    mesh_index = scene_dir / "generated" / "scene_visual_mesh_index.json"
    smoke_json = scene_dir / "generated" / "scene3d_gui_smoke.json"

    result = matrix.evaluate_scene(
        scene_name="raw_bounds_only_scene",
        scene_dir=scene_dir,
        mesh_index_path=mesh_index,
        smoke_json_path=smoke_json,
    )

    assert result["visual_quality_status"] == "FAIL"
    assert result["physical_rendered_count"] == 0
    assert result["raw_generated_bounds_count"] == 2
    assert result["diagnostic_fallback_count"] == 2
    assert "raw/generated fallback bounds are the only visible evidence despite mesh or URDF primitive sources" in result["blockers"]


def test_visual_quality_matrix_rejects_mesh_scene_with_excessive_raw_generated_fallback_bounds(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    scene_dir = _write_scene(
        repo,
        "mesh_with_excessive_raw_bounds_scene",
        _mesh_and_primitive_index(),
        {
            "schema": "workcell_studio_scene3d_gui_smoke/v1",
            "status": "PASS",
            "render_debug_counters": {
                "rendered_count": 4,
                "mesh_rendered_count": 1,
                "primitive_rendered_count": 1,
                "raw_generated_bounds_count": 2,
                "placeholder_count": 0,
                "wireframe_fallback_count": 0,
            },
        },
    )
    mesh_index = scene_dir / "generated" / "scene_visual_mesh_index.json"
    smoke_json = scene_dir / "generated" / "scene3d_gui_smoke.json"

    result = matrix.evaluate_scene(
        scene_name="mesh_with_excessive_raw_bounds_scene",
        scene_dir=scene_dir,
        mesh_index_path=mesh_index,
        smoke_json_path=smoke_json,
    )

    assert result["visual_quality_status"] == "FAIL"
    assert result["physical_rendered_count"] == 2
    assert result["raw_generated_bounds_count"] == 2
    assert result["diagnostic_fallback_count"] == 2
    assert "raw/generated fallback bounds dominate physical render evidence despite mesh or URDF primitive sources" in result["blockers"]
    assert "diagnostic fallback evidence dominates physical render evidence despite mesh or URDF primitive sources" in result["blockers"]


def test_visual_quality_matrix_allows_valid_urdf_primitive_render_without_placeholder_inflation(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    primitive_index = {
        "safe_for_preview": True,
        "visual_items": [
            {"id": "primitive_item", "geometry": {"cylinder": {"radius": 0.2, "length": 0.8}}},
        ],
    }
    scene_dir = _write_scene(
        repo,
        "primitive_scene",
        primitive_index,
        {
            "schema": "workcell_studio_scene3d_gui_smoke/v1",
            "status": "PASS",
            "render_debug_counters": {
                "rendered_count": 1,
                "mesh_rendered_count": 0,
                "urdf_primitive_rendered_count": 1,
                "primitive_fallback_count": 1,
                "placeholder_count": 0,
                "wireframe_fallback_count": 0,
                "generated_fallback_count": 0,
            },
        },
    )
    mesh_index = scene_dir / "generated" / "scene_visual_mesh_index.json"
    smoke_json = scene_dir / "generated" / "scene3d_gui_smoke.json"

    result = matrix.evaluate_scene(
        scene_name="primitive_scene",
        scene_dir=scene_dir,
        mesh_index_path=mesh_index,
        smoke_json_path=smoke_json,
    )

    assert result["visual_quality_status"] == "PASS"
    assert result["mesh_source_count"] == 0
    assert result["primitive_source_count"] == 1
    assert result["primitive_rendered_count"] == 1
    assert result["physical_rendered_count"] == 1
    assert result["placeholder_count"] == 0
    assert result["diagnostic_fallback_count"] == 0
    assert result["blockers"] == []

def test_evaluate_scene_blocks_missing_smoke_json_with_reason_code(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    scene_dir = repo / "scenes" / "missing_smoke_scene"
    mesh_index = scene_dir / "generated" / "scene_visual_mesh_index.json"
    _write_json(mesh_index, _mesh_and_primitive_index())
    missing_smoke = scene_dir / "generated" / "scene3d_gui_smoke.json"

    result = matrix.evaluate_scene(
        scene_name="missing_smoke_scene",
        scene_dir=scene_dir,
        mesh_index_path=mesh_index,
        smoke_json_path=missing_smoke,
    )

    assert result["visual_quality_status"] == "FAIL"
    assert "smoke_json_missing" in result["blocker_reasons"]
    assert any(blocker.startswith("smoke_json_missing:") for blocker in result["blockers"])
    assert not any("smoke JSON not found" in warning for warning in result["warnings"])


def test_evaluate_scene_blocks_unreadable_smoke_json_with_reason_code(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    scene_dir = repo / "scenes" / "unreadable_smoke_scene"
    mesh_index = scene_dir / "generated" / "scene_visual_mesh_index.json"
    smoke_json = scene_dir / "generated" / "scene3d_gui_smoke.json"
    _write_json(mesh_index, _mesh_and_primitive_index())
    smoke_json.parent.mkdir(parents=True, exist_ok=True)
    smoke_json.write_text("{not valid json", encoding="utf-8")

    result = matrix.evaluate_scene(
        scene_name="unreadable_smoke_scene",
        scene_dir=scene_dir,
        mesh_index_path=mesh_index,
        smoke_json_path=smoke_json,
    )

    assert result["visual_quality_status"] == "FAIL"
    assert "smoke_json_unreadable" in result["blocker_reasons"]
    assert any(blocker.startswith("smoke_json_unreadable:") for blocker in result["blockers"])


def test_evaluate_scene_blocks_missing_screenshot_with_reason_code(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    scene_dir = _write_scene(repo, "missing_screenshot_scene", _mesh_and_primitive_index(), _passing_smoke())
    mesh_index = scene_dir / "generated" / "scene_visual_mesh_index.json"
    smoke_json = scene_dir / "generated" / "scene3d_gui_smoke.json"
    screenshot = scene_dir / "generated" / "scene3d_gui_smoke.png"

    result = matrix.evaluate_scene(
        scene_name="missing_screenshot_scene",
        scene_dir=scene_dir,
        mesh_index_path=mesh_index,
        smoke_json_path=smoke_json,
        screenshot_path=screenshot,
    )

    assert result["visual_quality_status"] == "FAIL"
    assert "screenshot_missing" in result["blocker_reasons"]
    assert any(blocker.startswith("screenshot_missing:") for blocker in result["blockers"])
    assert not any("screenshot not found" in warning for warning in result["warnings"])



def test_screenshot_runner_marks_visual_quality_blocker_reasons_blocked(tmp_path: Path, monkeypatch) -> None:
    repo = tmp_path / "repo"
    output_dir = tmp_path / "out"
    scene_dir = _write_scene(repo, "runner_scene", _mesh_and_primitive_index(), _passing_smoke())
    smoke_json = scene_dir / "generated" / "scene3d_gui_smoke.json"
    missing_screenshot = output_dir / "scene3d_gui_smoke_runner_scene.png"

    def fake_run_smoke_for_target(**kwargs):
        return 0, smoke_json, missing_screenshot, _passing_smoke(), [], "fake smoke command"

    monkeypatch.setattr(screenshots, "_run_smoke_for_target", fake_run_smoke_for_target)

    result = screenshots.build_result_for_target(
        repo_root=repo,
        workspace_root=repo,
        executable=None,
        target={
            "target_kind": "explicit_scene",
            "scene_name": "runner_scene",
            "scene_path": str(scene_dir),
        },
        output_dir=output_dir,
        timeout_sec=1.0,
        xvfb=False,
    )

    assert result["status"] == "BLOCKED"
    assert "missing_screenshot" in result["blocker_reasons"]
    assert result["visual_quality_evaluation"]["blocker_reasons"] == ["screenshot_missing"]

def test_visual_quality_matrix_cli_writes_json(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    _write_scene(repo, "demo_scene", _mesh_and_primitive_index(), _passing_smoke())
    catalog = _write_catalog(
        repo,
        {
            "scene_name": "demo_scene",
            "package_name": "demo_scene",
            "scene_path": "scenes/demo_scene",
            "support_level": "supported",
            "status": "supported",
            "enabled": True,
        },
    )
    fixture = _write_scene(repo, "synthetic_visual_quality_fixture", _mesh_and_primitive_index(), _passing_smoke())
    out = tmp_path / "matrix.json"

    proc = subprocess.run(
        [
            "python3",
            str(SCRIPT),
            "--repo-root",
            str(repo),
            "--supported-scenes",
            str(catalog),
            "--synthetic-fixture",
            str(fixture),
            "--json",
            str(out),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert proc.returncode == 0
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["schema"] == "workcell_studio_scene3d_visual_quality_matrix/v1"
    assert payload["pass"] is True
