import json
import subprocess
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
WORKFLOW = REPO_ROOT / "scripts" / "run_workcell_studio_web_edit_workflow.py"


def _transform(x=0.0):
    return {"pose": {"xyz": {"x": x, "y": 0.1, "z": 0.2}, "rpy": {"x": 0.0, "y": 0.0, "z": 0.3}}}


def _patch(scene_id="tiny_scene", item_id="layout_bin", new_x=0.4):
    return {
        "schema_version": "workcell_studio_web_scene_edit_patch/v1",
        "scene_id": scene_id,
        "source_scene_schema_version": "workcell_studio_web_scene/v1",
        "created_at": "2026-06-29T00:00:00Z",
        "created_by": "static_web_viewer",
        "provenance": {"viewer_version": "test"},
        "edits": [{
            "item_id": item_id,
            "label": item_id,
            "source": "user_authored",
            "type": "box",
            "editable_required": True,
            "locked_required": False,
            "operation": "update_transform",
            "old_transform": _transform(0.0),
            "new_transform": _transform(new_x),
        }],
    }


def _scene(tmp_path):
    scene = tmp_path / "scenes" / "tiny_scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()
    (scene / "scene_manifest.yaml").write_text(yaml.safe_dump({"scene": {"id": "tiny_scene", "name": "tiny_scene"}}, sort_keys=False), encoding="utf-8")
    (scene / "cell_definition.yaml").write_text(yaml.safe_dump({"cell": {"id": "tiny_scene", "name": "tiny_scene"}}, sort_keys=False), encoding="utf-8")
    (scene / "layout" / "workcell_studio_layout.yaml").write_text(yaml.safe_dump({
        "schema_version": "workcell_studio_layout/v1",
        "items": [
            {"id": "layout_bin", "type": "bin", "display_name": "Bin", "pose": {"xyz": [0.0, 0.1, 0.2], "rpy": [0.0, 0.0, 0.3]}, "editable": True, "locked": False},
        ],
    }, sort_keys=False), encoding="utf-8")
    (scene / "environment.yaml").write_text(yaml.safe_dump({"schema_version": "workcell_scene/v1", "scene": {"id": "tiny_scene", "name": "tiny_scene"}, "environment": {"assets": []}}, sort_keys=False), encoding="utf-8")
    (scene / "generated" / "scene_visual_mesh_index.json").write_text(json.dumps({"items": [{"id": "ur5_visual", "role": "robot", "type": "robot", "pose": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}}]}), encoding="utf-8")
    return scene


def _write_patch(tmp_path, patch):
    path = tmp_path / "patch.json"
    path.write_text(json.dumps(patch, indent=2), encoding="utf-8")
    return path


def _run(scene, patch=None, output_dir=None, *extra):
    cmd = [sys.executable, str(WORKFLOW), "--scene", str(scene)]
    if patch is not None:
        cmd += ["--patch", str(patch)]
    if output_dir is not None:
        cmd += ["--output-dir", str(output_dir)]
    cmd += list(extra)
    return subprocess.run(cmd, cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def test_default_mode_does_not_mutate_source_files_and_creates_before(tmp_path):
    scene = _scene(tmp_path)
    patch = _write_patch(tmp_path, _patch())
    out = tmp_path / "out"
    layout = scene / "layout" / "workcell_studio_layout.yaml"
    before_text = layout.read_text(encoding="utf-8")
    result = _run(scene, patch, out)
    assert result.returncode == 0, result.stderr
    assert "dry-run result: PASS" in result.stdout
    assert "write/apply result: SKIPPED" in result.stdout
    assert (out / "tiny_scene.before.web_scene.json").exists()
    assert not (out / "tiny_scene.after.web_scene.json").exists()
    assert layout.read_text(encoding="utf-8") == before_text


def test_write_applies_reexports_and_verifies_persistence(tmp_path):
    scene = _scene(tmp_path)
    patch = _write_patch(tmp_path, _patch(new_x=0.7))
    out = tmp_path / "out"
    result = _run(scene, patch, out, "--write")
    assert result.returncode == 0, result.stderr
    assert "write/apply result: PASS" in result.stdout
    assert "persistence verification result: PASS" in result.stdout
    assert (out / "tiny_scene.before.web_scene.json").exists()
    assert (out / "tiny_scene.after.web_scene.json").exists()
    data = yaml.safe_load((scene / "layout" / "workcell_studio_layout.yaml").read_text(encoding="utf-8"))
    assert data["items"][0]["pose"]["xyz"] == [0.7, 0.1, 0.2]


def test_invalid_patch_fails_before_write(tmp_path):
    scene = _scene(tmp_path)
    bad = _patch()
    bad["schema_version"] = "wrong"
    patch = _write_patch(tmp_path, bad)
    layout = scene / "layout" / "workcell_studio_layout.yaml"
    before_text = layout.read_text(encoding="utf-8")
    result = _run(scene, patch, tmp_path / "out", "--write")
    assert result.returncode == 1
    assert "patch validation result: FAIL" in result.stderr
    assert "write/apply result: SKIPPED" in result.stdout + result.stderr
    assert layout.read_text(encoding="utf-8") == before_text


def test_scene_id_mismatch_fails_before_write(tmp_path):
    scene = _scene(tmp_path)
    patch = _write_patch(tmp_path, _patch(scene_id="wrong_scene"))
    result = _run(scene, patch, tmp_path / "out", "--write")
    assert result.returncode == 1
    assert "scene_id mismatch" in result.stderr


def test_locked_generated_edit_fails_before_write(tmp_path):
    scene = _scene(tmp_path)
    patch = _write_patch(tmp_path, _patch(item_id="ur5_visual"))
    result = _run(scene, patch, tmp_path / "out", "--write")
    assert result.returncode == 1
    assert "locked=true items cannot be edited" in result.stderr
    assert "generated robot/tool preview items cannot be edited" in result.stderr


def test_missing_patch_gives_useful_error(tmp_path):
    scene = _scene(tmp_path)
    result = _run(scene, tmp_path / "missing.json", tmp_path / "out")
    assert result.returncode == 2
    assert "patch missing" in result.stderr


def test_export_only_only_exports_and_does_not_require_patch(tmp_path):
    scene = _scene(tmp_path)
    out = tmp_path / "out"
    result = _run(scene, None, out, "--export-only")
    assert result.returncode == 0, result.stderr
    assert "export-only result: PASS" in result.stdout
    assert (out / "tiny_scene.before.web_scene.json").exists()


def test_validate_only_validates_and_does_not_write(tmp_path):
    scene = _scene(tmp_path)
    patch = _write_patch(tmp_path, _patch(new_x=0.8))
    layout = scene / "layout" / "workcell_studio_layout.yaml"
    before_text = layout.read_text(encoding="utf-8")
    result = _run(scene, patch, tmp_path / "out", "--validate-only")
    assert result.returncode == 0, result.stderr
    assert "validate-only result: PASS" in result.stdout
    assert "dry-run apply" not in result.stdout
    assert layout.read_text(encoding="utf-8") == before_text


def test_run_readiness_is_optional_for_core_flow(tmp_path):
    scene = _scene(tmp_path)
    patch = _write_patch(tmp_path, _patch())
    result = _run(scene, patch, tmp_path / "out")
    assert result.returncode == 0, result.stderr
    assert "readiness matrix" not in result.stdout


def test_generate_without_patch_uses_selected_scene_flow(tmp_path):
    scene = _scene(tmp_path)
    result = _run(scene, None, tmp_path / "out", "--generate")
    assert result.returncode == 0, result.stderr
    assert "patch applied/skipped: SKIPPED (no --patch provided)" in result.stdout
    assert "scene generation" in result.stdout
    assert "generation command/result:" in result.stdout
    assert "validation command/result: SKIPPED" in result.stdout
    assert (scene / "package.xml").exists()
    assert (scene / "CMakeLists.txt").exists()


def test_validate_without_patch_runs_selected_scene_validator(tmp_path):
    scene = _scene(tmp_path)
    # The selected-scene validator expects package files; generate them through the
    # same helper that --generate uses so this test focuses on the no-patch mode.
    gen = _run(scene, None, tmp_path / "gen", "--generate")
    assert gen.returncode == 0, gen.stderr
    result = _run(scene, None, tmp_path / "out", "--validate")
    assert result.returncode == 0, result.stderr
    assert "patch applied/skipped: SKIPPED (no --patch provided)" in result.stdout
    assert "selected-scene validation" in result.stdout
    assert "validation command/result:" in result.stdout
    assert "Workflow summary: PASS" in result.stdout


def test_generate_and_validate_after_write_preserves_safe_order(tmp_path):
    scene = _scene(tmp_path)
    patch = _write_patch(tmp_path, _patch(new_x=0.9))
    result = _run(scene, patch, tmp_path / "out", "--write", "--generate-and-validate")
    assert result.returncode == 0, result.stderr
    ordered_markers = [
        "patch validation result: PASS",
        "== dry-run apply ==",
        "== write apply ==",
        "re-export after result: PASS",
        "== persistence verification ==",
        "== scene generation ==",
        "== selected-scene validation ==",
    ]
    positions = [result.stdout.index(marker) for marker in ordered_markers]
    assert positions == sorted(positions)
    assert "patch applied/skipped: APPLIED" in result.stdout
    assert "generation command/result:" in result.stdout
    assert "validation command/result:" in result.stdout
