import json
import math
import subprocess
import sys
from pathlib import Path

from jsonschema import Draft202012Validator

REPO_ROOT = Path(__file__).resolve().parents[1]
SCHEMA = REPO_ROOT / "schemas" / "workcell_studio_web_scene_edit_patch_v1.schema.json"
VALIDATOR = REPO_ROOT / "scripts" / "validate_workcell_studio_web_scene_edit_patch.py"
VIEWER = REPO_ROOT / "workcell_studio_web" / "viewer" / "viewer.js"
INDEX = REPO_ROOT / "workcell_studio_web" / "viewer" / "index.html"


def _transform(x=0.0):
    return {"pose": {"xyz": {"x": x, "y": 0.0, "z": 0.0}, "rpy": {"x": 0.0, "y": 0.0, "z": 0.0}}, "scale": {"x": 1.0, "y": 1.0, "z": 1.0}}


def _web_scene():
    return {
        "schema_version": "workcell_studio_web_scene/v1",
        "scene": {"id": "tiny_scene"},
        "robots": [{"id": "ur5_visual", "label": "UR5", "type": "robot", "source_kind": "generated_preview", "editable": False, "locked": True}],
        "tools": [{"id": "tool_visual", "label": "Tool", "type": "gripper", "source_kind": "generated_preview", "editable": False, "locked": True}],
        "assets": [{"id": "layout_bin", "label": "Bin", "type": "bin", "source_kind": "user_authored", "editable": True, "locked": False}],
        "sensors": [],
        "zones": [],
        "warnings": [],
        "backend_actions": [],
    }


def _patch(item_id="layout_bin", new_x=0.2):
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
            "new_transform": _transform(new_x),
        }],
    }


def _run_validator(tmp_path, web_scene, patch):
    web = tmp_path / "scene.web_scene.json"
    patch_path = tmp_path / "scene.edit_patch.json"
    web.write_text(json.dumps(web_scene), encoding="utf-8")
    patch_path.write_text(json.dumps(patch), encoding="utf-8")
    return subprocess.run([sys.executable, str(VALIDATOR), "--web-scene", str(web), "--patch", str(patch_path)], cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def test_edit_patch_schema_exists_and_is_valid_json():
    assert SCHEMA.exists()
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    assert schema["properties"]["schema_version"]["const"] == "workcell_studio_web_scene_edit_patch/v1"
    Draft202012Validator.check_schema(schema)


def test_validator_accepts_valid_editable_item_transform_patch(tmp_path):
    result = _run_validator(tmp_path, _web_scene(), _patch())
    assert result.returncode == 0, result.stderr
    assert "No source scene files were modified" in result.stdout


def test_validator_rejects_locked_generated_item_patch(tmp_path):
    result = _run_validator(tmp_path, _web_scene(), _patch("ur5_visual"))
    assert result.returncode == 1
    assert "locked=true items cannot be edited" in result.stderr
    assert "generated robot/tool preview items cannot be edited" in result.stderr


def test_validator_rejects_missing_item_id(tmp_path):
    result = _run_validator(tmp_path, _web_scene(), _patch("missing_item"))
    assert result.returncode == 1
    assert "item not found" in result.stderr


def test_validator_rejects_non_finite_transform_values(tmp_path):
    patch = _patch()
    patch["edits"][0]["new_transform"]["pose"]["xyz"]["x"] = math.inf
    patch_path = tmp_path / "bad.edit_patch.json"
    # JSON cannot portably encode infinity for strict parsers, so write the token explicitly.
    text = json.dumps(patch).replace("Infinity", "1e999")
    web = tmp_path / "scene.web_scene.json"
    web.write_text(json.dumps(_web_scene()), encoding="utf-8")
    patch_path.write_text(text, encoding="utf-8")
    result = subprocess.run([sys.executable, str(VALIDATOR), "--web-scene", str(web), "--patch", str(patch_path)], cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert result.returncode != 0
    assert "finite number" in result.stderr or "unable to load JSON" in result.stderr


def test_viewer_files_contain_export_edit_patch_and_locked_editable_guards():
    viewer = VIEWER.read_text(encoding="utf-8")
    index = INDEX.read_text(encoding="utf-8")
    assert "Export Edit Patch" in index
    assert "Clear Preview Edits" in index
    assert "Locked/generated preview item; edit source layout/environment instead." in viewer
    assert "editable !== true" in viewer
    assert "source.includes('generated')" in viewer
    assert "schema_version: EDIT_PATCH_SCHEMA_VERSION" in viewer


def test_no_generated_patch_json_committed():
    tracked = subprocess.run(["git", "ls-files", "*.edit_patch.json", "*edit_patch.json"], cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, check=True).stdout.splitlines()
    assert not tracked
