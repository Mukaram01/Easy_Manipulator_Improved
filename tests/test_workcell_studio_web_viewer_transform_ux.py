import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
VIEWER = REPO_ROOT / "workcell_studio_web" / "viewer" / "viewer.js"
INDEX = REPO_ROOT / "workcell_studio_web" / "viewer" / "index.html"


def _viewer_text():
    return VIEWER.read_text(encoding="utf-8")


def _index_text():
    return INDEX.read_text(encoding="utf-8")


def test_viewer_has_edit_mode_guard_for_editable_and_unlocked_items():
    viewer = _viewer_text()
    assert "function canEditItem" in viewer
    assert "item?.locked" in viewer
    assert "item?.editable !== true" in viewer
    assert "source.includes('generated')" in viewer
    assert "sourceIdentity" in viewer
    assert "active_visual_source" in viewer
    assert "Edit mode active for editable/unlocked item" in viewer


def test_locked_generated_edit_message_exists():
    assert "Locked/generated preview item; edit source layout/environment instead." in _viewer_text()


def test_transform_gizmo_hook_is_loaded_without_build_system():
    viewer = _viewer_text()
    assert "TransformControls" in viewer
    assert "three/addons/controls/TransformControls.js" in viewer
    assert "attachTransformGizmo" in viewer
    assert "gizmo.attach(rendered.object3d)" in viewer


def test_snap_controls_exist():
    index = _index_text()
    viewer = _viewer_text()
    assert 'id="snap-toggle"' in index
    assert 'id="translation-snap"' in index
    assert 'id="rotation-snap"' in index
    assert "setTranslationSnap" in viewer
    assert "setRotationSnap" in viewer
    assert "snapTransform" in viewer


def test_undo_redo_clear_preview_edit_controls_exist():
    index = _index_text()
    viewer = _viewer_text()
    assert "Undo" in index
    assert "Redo" in index
    assert "Clear Preview Edits" in index
    assert "undoPreviewEdit" in viewer
    assert "redoPreviewEdit" in viewer
    assert "clearPreviewEdits" in viewer
    assert "undoStack" in viewer
    assert "redoStack" in viewer


def test_patch_export_still_exists_and_remains_preview_only():
    index = _index_text()
    viewer = _viewer_text()
    assert "Export Edit Patch" in index
    assert "buildEditPatch" in viewer
    assert "schema_version: EDIT_PATCH_SCHEMA_VERSION" in viewer
    assert "Preview-only browser transform edit. Source scene files were not modified." in viewer


def test_no_direct_yaml_write_or_browser_apply_logic_added():
    combined = (_viewer_text() + "\n" + _index_text()).lower()
    forbidden = ["environment.yaml", "workcell_studio_layout.yaml", "yaml.safe_dump", "js-yaml", "apply_workcell_studio_web_scene_edit_patch"]
    for token in forbidden:
        assert token not in combined


def test_no_generated_scene_json_committed():
    tracked = subprocess.run(
        ["git", "ls-files", "*.web_scene.json", "*web_scene.json", "*.edit_patch.json", "*edit_patch.json"],
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        check=True,
    ).stdout.splitlines()
    assert not tracked
