import importlib.util
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "check_workcell_web_scene_mesh_contract.py"
spec = importlib.util.spec_from_file_location("check_workcell_web_scene_mesh_contract", SCRIPT)
checker = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(checker)


def _scene(tmp_path, mesh_url="assets/scene/robot.dae", *, fallback_count=0):
    (tmp_path / "assets" / "scene").mkdir(parents=True)
    (tmp_path / "assets" / "scene" / "robot.dae").write_text("mesh", encoding="utf-8")
    web_scene = tmp_path / "scene.web_scene.json"
    payload = {
        "metadata": {"mesh_contract": {"fallback_primitive_count": fallback_count}},
        "robots": [
            {
                "id": "robot_link",
                "mesh_load_required": True,
                "mesh_url": mesh_url,
                "mesh_staging_status": "staged",
                "geometry_type": "mesh",
            }
        ],
    }
    return web_scene, payload


def test_passes_when_required_mesh_url_is_staged_under_output_dir(tmp_path):
    web_scene, payload = _scene(tmp_path)

    summary, errors = checker.check(payload, web_scene)

    assert errors == []
    assert summary["required_mesh_count"] == 1
    assert summary["staged_mesh_count"] == 1
    assert summary["contract_status"] == "passed"


def test_fails_when_required_mesh_file_is_missing(tmp_path):
    web_scene, payload = _scene(tmp_path, mesh_url="assets/scene/missing.dae")

    summary, errors = checker.check(payload, web_scene)

    assert summary["required_mesh_count"] == 1
    assert summary["staged_mesh_count"] == 0
    assert summary["missing_required_meshes"] == ["robots:robot_link"]
    assert "required mesh item(s) are missing browser-loadable staged files" in errors


def test_fails_when_contract_reports_core_primitive_fallback(tmp_path):
    web_scene, payload = _scene(tmp_path, fallback_count=1)

    summary, errors = checker.check(payload, web_scene)

    assert summary["contract_status"] == "failed"
    assert "core mesh item(s) are primitive-fallback-only" in errors
