import json
import subprocess
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
APPLICATOR = REPO_ROOT / "scripts" / "apply_workcell_studio_web_scene_edit_patch.py"
EXPORTER = REPO_ROOT / "scripts" / "export_workcell_studio_web_scene.py"
VERIFIER = REPO_ROOT / "scripts" / "verify_workcell_studio_web_scene_edit_persistence.py"


def _transform(x=0.0, y=0.1, z=0.2, yaw=0.3, scale=None):
    data = {"pose": {"xyz": {"x": x, "y": y, "z": z}, "rpy": {"x": 0.0, "y": 0.0, "z": yaw}}}
    if scale is not None:
        data["scale"] = {"x": scale, "y": scale, "z": scale}
    return data


def _patch(item_id="layout_bin", new_x=0.4, old_x=0.0, scale=None, old_scale=None):
    return {
        "schema_version": "workcell_studio_web_scene_edit_patch/v1",
        "scene_id": "tiny_scene",
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
            "old_transform": _transform(old_x, scale=old_scale),
            "new_transform": _transform(new_x, scale=scale),
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
            {"id": "other_box", "type": "box", "display_name": "Other", "pose": {"xyz": [1.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}, "editable": True, "locked": False},
            {"id": "scaled_box", "type": "box", "display_name": "Scaled", "pose": {"xyz": [0.0, 0.1, 0.2], "rpy": [0.0, 0.0, 0.3]}, "scale": [1.0, 1.0, 1.0], "editable": True, "locked": False},
        ],
    }, sort_keys=False), encoding="utf-8")
    (scene / "environment.yaml").write_text(yaml.safe_dump({"schema_version": "workcell_scene/v1", "scene": {"id": "tiny_scene", "name": "tiny_scene"}, "environment": {"assets": []}}, sort_keys=False), encoding="utf-8")
    (scene / "generated" / "scene_visual_mesh_index.json").write_text(json.dumps({"items": [{"id": "ur5_visual", "role": "robot", "type": "robot", "pose": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}}]}), encoding="utf-8")
    return scene


def _run(cmd, **kwargs):
    return subprocess.run([sys.executable, *map(str, cmd)], cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, **kwargs)


def _export(scene, output):
    result = _run([EXPORTER, "--scene", scene, "--output", output])
    assert result.returncode == 0, result.stderr
    return json.loads(Path(output).read_text(encoding="utf-8"))


def _write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")


def _full_loop(tmp_path, patch):
    scene = _scene(tmp_path)
    before_path = tmp_path / "before.json"
    patch_path = tmp_path / "patch.json"
    after_path = tmp_path / "after.json"
    before = _export(scene, before_path)
    _write_json(patch_path, patch)
    dry = _run([APPLICATOR, "--scene", scene, "--web-scene", before_path, "--patch", patch_path])
    assert dry.returncode == 0, dry.stderr
    write = _run([APPLICATOR, "--scene", scene, "--web-scene", before_path, "--patch", patch_path, "--write"])
    assert write.returncode == 0, write.stderr
    after = _export(scene, after_path)
    verify = _run([VERIFIER, "--scene", scene, "--web-scene-before", before_path, "--patch", patch_path, "--web-scene-after", after_path])
    return scene, before, after, verify, before_path, patch_path, after_path


def test_edited_layout_pose_persists_after_apply_and_reexport(tmp_path):
    _, _, after, verify, *_ = _full_loop(tmp_path, _patch(new_x=0.7))
    assert verify.returncode == 0, verify.stderr
    assert "PASS edited item layout_bin" in verify.stdout
    item = next(i for i in after["assets"] if i["id"] == "layout_bin")
    assert item["pose"]["xyz"] == [0.7, 0.1, 0.2]


def test_scale_persists_when_supported_by_source_mapping(tmp_path):
    _, _, after, verify, *_ = _full_loop(tmp_path, _patch("scaled_box", new_x=0.2, scale=1.5, old_scale=1.0))
    assert verify.returncode == 0, verify.stderr
    item = next(i for i in after["assets"] if i["id"] == "scaled_box")
    assert item["scale"] == [1.5, 1.5, 1.5]


def test_unrelated_editable_and_locked_generated_items_remain_unchanged(tmp_path):
    _, before, after, verify, *_ = _full_loop(tmp_path, _patch(new_x=0.8))
    assert verify.returncode == 0, verify.stderr
    assert next(i for i in before["assets"] if i["id"] == "other_box") == next(i for i in after["assets"] if i["id"] == "other_box")
    assert next(i for i in before["robots"] if i["id"] == "ur5_visual") == next(i for i in after["robots"] if i["id"] == "ur5_visual")


def test_dry_run_still_writes_nothing(tmp_path):
    scene = _scene(tmp_path)
    before_path = tmp_path / "before.json"
    patch_path = tmp_path / "patch.json"
    _export(scene, before_path)
    _write_json(patch_path, _patch(new_x=0.9))
    layout = scene / "layout" / "workcell_studio_layout.yaml"
    before_text = layout.read_text(encoding="utf-8")
    result = _run([APPLICATOR, "--scene", scene, "--web-scene", before_path, "--patch", patch_path])
    assert result.returncode == 0, result.stderr
    assert layout.read_text(encoding="utf-8") == before_text


def test_verifier_rejects_after_scene_where_edited_item_did_not_change(tmp_path):
    scene = _scene(tmp_path)
    before_path = tmp_path / "before.json"
    patch_path = tmp_path / "patch.json"
    _export(scene, before_path)
    _write_json(patch_path, _patch(new_x=0.6))
    result = _run([VERIFIER, "--scene", scene, "--web-scene-before", before_path, "--patch", patch_path, "--web-scene-after", before_path])
    assert result.returncode == 1
    assert "new_transform" in result.stderr


def test_verifier_rejects_after_scene_where_unrelated_item_changed(tmp_path):
    scene, _, after, _, before_path, patch_path, after_path = _full_loop(tmp_path, _patch(new_x=0.6))
    for item in after["assets"]:
        if item["id"] == "other_box":
            item["pose"]["xyz"] = [9, 9, 9]
    _write_json(after_path, after)
    result = _run([VERIFIER, "--scene", scene, "--web-scene-before", before_path, "--patch", patch_path, "--web-scene-after", after_path])
    assert result.returncode == 1
    assert "unrelated item changed unexpectedly" in result.stderr


def test_verifier_rejects_scene_id_mismatch(tmp_path):
    scene, _, after, _, before_path, patch_path, after_path = _full_loop(tmp_path, _patch(new_x=0.6))
    after["scene"]["id"] = "wrong_scene"
    _write_json(after_path, after)
    result = _run([VERIFIER, "--scene", scene, "--web-scene-before", before_path, "--patch", patch_path, "--web-scene-after", after_path])
    assert result.returncode == 1
    assert "scene_id mismatch" in result.stderr


def test_verifier_prints_useful_failure_messages(tmp_path):
    scene = _scene(tmp_path)
    before_path = tmp_path / "before.json"
    patch_path = tmp_path / "patch.json"
    _export(scene, before_path)
    bad_patch = _patch("missing_item", new_x=0.6)
    _write_json(patch_path, bad_patch)
    result = _run([VERIFIER, "--scene", scene, "--web-scene-before", before_path, "--patch", patch_path, "--web-scene-after", before_path])
    assert result.returncode == 1
    assert "missing_item" in result.stderr
    assert "edited item missing" in result.stderr or "item not found" in result.stderr
