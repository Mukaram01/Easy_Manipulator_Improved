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
            "placeholder_count",
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
        assert scene["placeholder_count"] == 0
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
    assert "screenshot_missing" in result["blocker_reasons"]
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


def _mesh_only_index(*, resolved: bool = True, reason_code: str | None = None) -> dict:
    item = {"id": "healthy_mesh", "geometry": {"mesh": {"filename": "package://demo/meshes/healthy.stl"}}, "resolved": resolved}
    if reason_code:
        item["reason_code"] = reason_code
    return {"safe_for_preview": resolved, "visual_items": [item]}


def _primitive_only_index() -> dict:
    return {"safe_for_preview": True, "visual_items": [{"id": "healthy_primitive", "geometry": {"box": {"size": [1.0, 1.0, 0.1]}}}]}


def _smoke_with_counts(**counts: int) -> dict:
    merged = {
        "rendered_count": 0,
        "mesh_rendered_count": 0,
        "primitive_rendered_count": 0,
        "placeholder_count": 0,
        "wireframe_fallback_count": 0,
    }
    merged.update(counts)
    return {"schema": "workcell_studio_scene3d_gui_smoke/v1", "status": "PASS", "render_debug_counters": merged}


def test_visual_quality_matrix_mesh_backed_fixture_requires_mesh_source_and_render_evidence(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    healthy = _write_scene(repo, "healthy_mesh_scene", _mesh_only_index(), _smoke_with_counts(rendered_count=1, mesh_rendered_count=1))
    missing_render = _write_scene(repo, "mesh_without_render_scene", _mesh_only_index(), _smoke_with_counts(rendered_count=1, mesh_rendered_count=0))

    healthy_result = matrix.evaluate_scene(
        scene_name="healthy_mesh_scene",
        scene_dir=healthy,
        mesh_index_path=healthy / "generated" / "scene_visual_mesh_index.json",
        smoke_json_path=healthy / "generated" / "scene3d_gui_smoke.json",
    )
    missing_render_result = matrix.evaluate_scene(
        scene_name="mesh_without_render_scene",
        scene_dir=missing_render,
        mesh_index_path=missing_render / "generated" / "scene_visual_mesh_index.json",
        smoke_json_path=missing_render / "generated" / "scene3d_gui_smoke.json",
    )

    assert healthy_result["visual_quality_status"] == "PASS"
    assert healthy_result["mesh_source_count"] > 0
    assert healthy_result["mesh_rendered_count"] > 0
    assert missing_render_result["visual_quality_status"] == "FAIL"
    assert "mesh_source_count > 0 requires mesh_rendered_count > 0" in missing_render_result["blockers"]


def test_visual_quality_matrix_urdf_primitive_fixture_requires_source_and_render_evidence(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    healthy = _write_scene(repo, "healthy_primitive_scene", _primitive_only_index(), _smoke_with_counts(rendered_count=1, primitive_rendered_count=1))
    missing_render = _write_scene(repo, "primitive_without_render_scene", _primitive_only_index(), _smoke_with_counts(rendered_count=1, primitive_rendered_count=0))

    healthy_result = matrix.evaluate_scene(
        scene_name="healthy_primitive_scene",
        scene_dir=healthy,
        mesh_index_path=healthy / "generated" / "scene_visual_mesh_index.json",
        smoke_json_path=healthy / "generated" / "scene3d_gui_smoke.json",
    )
    missing_render_result = matrix.evaluate_scene(
        scene_name="primitive_without_render_scene",
        scene_dir=missing_render,
        mesh_index_path=missing_render / "generated" / "scene_visual_mesh_index.json",
        smoke_json_path=missing_render / "generated" / "scene3d_gui_smoke.json",
    )

    assert healthy_result["visual_quality_status"] == "PASS"
    assert healthy_result["primitive_source_count"] > 0
    assert healthy_result["primitive_rendered_count"] > 0
    assert missing_render_result["visual_quality_status"] == "FAIL"
    assert "primitive_source_count > 0 requires primitive_rendered_count > 0" in missing_render_result["blockers"]


def test_visual_quality_matrix_reports_missing_mesh_reason_code(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    scene_dir = _write_scene(repo, "missing_mesh_scene", _mesh_only_index(resolved=False, reason_code="mesh_missing_on_disk"), _smoke_with_counts(rendered_count=0))
    catalog = _write_catalog(
        repo,
        {
            "scene_name": "missing_mesh_scene",
            "package_name": "missing_mesh_scene",
            "scene_path": "scenes/missing_mesh_scene",
            "support_level": "supported",
            "status": "supported",
            "enabled": True,
        },
    )

    fixture = _write_scene(repo, "synthetic_visual_quality_fixture", _mesh_and_primitive_index(), _passing_smoke())
    payload = matrix.build_matrix(repo_root=repo, supported_scenes=catalog, synthetic_fixture=fixture)
    result = next(scene for scene in payload["scenes"] if scene["scene_name"] == "missing_mesh_scene")

    assert payload["pass"] is False
    assert result["mesh_failure_summary_by_reason_code"]["by_reason_code"] == {"mesh_missing_on_disk": 1}
    assert "mesh_missing_on_disk" in result["blocker_reasons"]
    assert any(blocker.startswith("mesh_missing_on_disk:") for blocker in result["blockers"])


def test_visual_quality_matrix_rejects_overlay_only_render_counters_without_physical_evidence(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    scene_dir = _write_scene(
        repo,
        "overlay_only_scene",
        {"safe_for_preview": True, "visual_items": [{"id": "pick_zone_overlay", "source_layer": "overlay", "helper": True, "renderable": False}]},
        _smoke_with_counts(rendered_count=3, overlay_helper_count=3),
    )

    result = matrix.evaluate_scene(
        scene_name="overlay_only_scene",
        scene_dir=scene_dir,
        mesh_index_path=scene_dir / "generated" / "scene_visual_mesh_index.json",
        smoke_json_path=scene_dir / "generated" / "scene3d_gui_smoke.json",
    )

    assert result["visual_quality_status"] == "FAIL"
    assert result["mesh_source_count"] == 0
    assert result["primitive_source_count"] == 0
    assert result["mesh_rendered_count"] == 0
    assert result["primitive_rendered_count"] == 0
    assert "source geometry classification missing; rendered_count alone cannot prove visual quality" in result["blockers"]


def test_visual_quality_matrix_rejects_raw_generated_bounds_only_fallback_boxes(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    scene_dir = _write_scene(
        repo,
        "raw_bounds_only_scene",
        {"safe_for_preview": True, "visual_items": [{"id": "raw_generated_bounds", "raw_generated_bounds": [0, 0, 0, 1, 1, 1]}]},
        _smoke_with_counts(rendered_count=1, placeholder_count=1),
    )

    result = matrix.evaluate_scene(
        scene_name="raw_bounds_only_scene",
        scene_dir=scene_dir,
        mesh_index_path=scene_dir / "generated" / "scene_visual_mesh_index.json",
        smoke_json_path=scene_dir / "generated" / "scene3d_gui_smoke.json",
    )

    assert result["visual_quality_status"] == "FAIL"
    assert result["missing_geometry_count"] == 1
    assert result["placeholder_count"] == 1
    assert result["mesh_rendered_count"] + result["primitive_rendered_count"] == 0
    assert "source geometry classification missing; rendered_count alone cannot prove visual quality" in result["blockers"]
