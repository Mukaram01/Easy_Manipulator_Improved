import json
import math
import subprocess
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
APPLICATOR = REPO_ROOT / "scripts" / "apply_workcell_studio_web_scene_edit_patch.py"
EXPORTER = REPO_ROOT / "scripts" / "export_workcell_studio_web_scene.py"


def _transform(x=0.0, scale=1.0):
    return {"pose": {"xyz": {"x": x, "y": 0.1, "z": 0.2}, "rpy": {"x": 0.0, "y": 0.0, "z": 0.3}}, "scale": {"x": scale, "y": scale, "z": scale}}


def _web_scene(source="layout/workcell_studio_layout.yaml", editable=True, locked=False, source_kind="user_authored"):
    return {
        "schema_version": "workcell_studio_web_scene/v1",
        "scene": {"id": "tiny_scene"},
        "robots": [{"id": "ur5_visual", "label": "UR5", "type": "robot", "source_kind": "generated_preview", "source_layer": "generated_preview", "editable": False, "locked": True}],
        "tools": [],
        "assets": [{"id": "layout_bin", "label": "Bin", "type": "bin", "source_kind": source_kind, "editable": editable, "locked": locked, "provenance": {"pose": source, "id": source}}],
        "sensors": [],
        "zones": [],
        "warnings": [],
        "backend_actions": [],
    }


def _patch(item_id="layout_bin", new_x=0.4, scale=1.0):
    return {
        "schema_version": "workcell_studio_web_scene_edit_patch/v1",
        "scene_id": "tiny_scene",
        "source_scene_schema_version": "workcell_studio_web_scene/v1",
        "created_at": "2026-06-29T00:00:00Z",
        "created_by": "static_web_viewer",
        "provenance": {"viewer_version": "test"},
        "edits": [{
            "item_id": item_id,
            "label": "Bin",
            "source": "user_authored",
            "type": "bin",
            "editable_required": True,
            "locked_required": False,
            "operation": "update_transform",
            "old_transform": _transform(0.0),
            "new_transform": _transform(new_x, scale),
        }],
    }


def _scene(tmp_path):
    scene = tmp_path / "scenes" / "tiny_scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "layout" / "workcell_studio_layout.yaml").write_text(yaml.safe_dump({
        "schema_version": "workcell_studio_layout/v1",
        "items": [{"id": "layout_bin", "type": "bin", "display_name": "Bin", "pose": {"xyz": [0.0, 0.1, 0.2], "rpy": [0.0, 0.0, 0.3]}, "editable": True, "locked": False}],
    }, sort_keys=False), encoding="utf-8")
    (scene / "environment.yaml").write_text(yaml.safe_dump({
        "schema_version": "workcell_scene/v1",
        "scene": {"id": "tiny_scene", "name": "tiny_scene"},
        "environment": {"assets": [{"id": "env_box", "type": "box", "pose_xyz": [0.0, 0.0, 0.0], "pose_rpy": [0.0, 0.0, 0.0]}]},
    }, sort_keys=False), encoding="utf-8")
    return scene


def _run(tmp_path, scene, web_scene, patch, *extra):
    web = tmp_path / "scene.web_scene.json"
    patch_path = tmp_path / "scene.edit_patch.json"
    web.write_text(json.dumps(web_scene), encoding="utf-8")
    patch_path.write_text(json.dumps(patch), encoding="utf-8")
    return subprocess.run([sys.executable, str(APPLICATOR), "--scene", str(scene), "--web-scene", str(web), "--patch", str(patch_path), *extra], cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def test_dry_run_does_not_mutate_files(tmp_path):
    scene = _scene(tmp_path)
    layout = scene / "layout" / "workcell_studio_layout.yaml"
    before = layout.read_text(encoding="utf-8")
    result = _run(tmp_path, scene, _web_scene(), _patch())
    assert result.returncode == 0, result.stderr
    assert "Dry-run only" in result.stdout
    assert "write would occur: False" in result.stdout
    assert layout.read_text(encoding="utf-8") == before


def test_write_updates_editable_layout_item_pose(tmp_path):
    scene = _scene(tmp_path)
    result = _run(tmp_path, scene, _web_scene(), _patch(new_x=0.7), "--write")
    assert result.returncode == 0, result.stderr
    assert "updated item count: 1" in result.stdout
    data = yaml.safe_load((scene / "layout" / "workcell_studio_layout.yaml").read_text(encoding="utf-8"))
    assert data["items"][0]["pose"]["xyz"] == [0.7, 0.1, 0.2]


def test_locked_generated_edit_is_rejected(tmp_path):
    scene = _scene(tmp_path)
    result = _run(tmp_path, scene, _web_scene(), _patch("ur5_visual"), "--write")
    assert result.returncode == 1
    assert "locked=true items cannot be edited" in result.stderr
    assert "generated robot/tool preview items cannot be edited" in result.stderr


def test_ambiguous_source_mapping_is_rejected(tmp_path):
    scene = _scene(tmp_path)
    web_scene = _web_scene(source="unknown.yaml")
    result = _run(tmp_path, scene, web_scene, _patch(), "--write")
    assert result.returncode == 1
    assert "ambiguous source mapping" in result.stderr


def test_missing_item_id_is_rejected(tmp_path):
    scene = _scene(tmp_path)
    result = _run(tmp_path, scene, _web_scene(), _patch("missing"))
    assert result.returncode == 1
    assert "item not found" in result.stderr


def test_non_finite_transform_rejected_via_validator(tmp_path):
    scene = _scene(tmp_path)
    patch = _patch()
    patch["edits"][0]["new_transform"]["pose"]["xyz"]["x"] = math.inf
    result = _run(tmp_path, scene, _web_scene(), patch)
    assert result.returncode == 1
    assert "finite number" in result.stderr or "unable to load JSON" in result.stderr


def test_generated_files_are_never_written(tmp_path):
    scene = _scene(tmp_path)
    (scene / "generated").mkdir()
    generated = scene / "generated" / "scene_visual_mesh_index.json"
    generated.write_text('{"items": []}', encoding="utf-8")
    before = generated.read_text(encoding="utf-8")
    web_scene = _web_scene(source="generated/scene_visual_mesh_index.json")
    result = _run(tmp_path, scene, web_scene, _patch(), "--write")
    assert result.returncode == 1
    assert generated.read_text(encoding="utf-8") == before


def test_reexport_after_apply_reflects_updated_pose(tmp_path):
    scene = _scene(tmp_path)
    result = _run(tmp_path, scene, _web_scene(), _patch(new_x=0.9), "--write")
    assert result.returncode == 0, result.stderr
    out = tmp_path / "reexport.web_scene.json"
    export = subprocess.run([sys.executable, str(EXPORTER), "--scene", str(scene), "--output", str(out)], cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert export.returncode == 0, export.stderr
    payload = json.loads(out.read_text(encoding="utf-8"))
    item = next(i for i in payload["assets"] if i["id"] == "layout_bin")
    assert item["pose"]["xyz"] == [0.9, 0.1, 0.2]
