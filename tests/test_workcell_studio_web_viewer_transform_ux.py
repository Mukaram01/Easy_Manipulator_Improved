import subprocess
import re
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
    assert "canEditItem" in viewer
    assert "item?.editable !== true" in viewer
    assert "item?.locked" in viewer


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


def test_status_summary_reports_loaded_fallback_and_failed_meshes():
    index = _index_text()
    viewer = _viewer_text()
    assert "meshLoadedCount" in viewer
    assert "fallbackCount" in viewer
    assert "meshFailedCount" in viewer
    assert "render_status === 'mesh_loaded'" in viewer
    assert "isRuntimeFallbackStatus" in viewer
    assert "isMissingOrFailedMeshStatus" in viewer
    assert 'data-summary-field="mesh-loaded-count"' in index
    assert 'data-summary-field="fallback-count"' in index
    assert 'data-summary-field="mesh-failed-count"' in index


def test_camera_fit_constants_cover_ur5_2f_sized_workcell_without_clipping():
    viewer = _viewer_text()
    min_radius = float(re.search(r"const MIN_FRAME_RADIUS = ([0-9.]+);", viewer).group(1))
    distance_multiplier = float(re.search(r"const FRAME_DISTANCE_MULTIPLIER = ([0-9.]+);", viewer).group(1))
    near_formula = re.search(r"camera\.near = Math\.max\(0\.01, radius / ([0-9.]+)\);", viewer)
    far_formula = re.search(r"camera\.far = Math\.max\(100, distance \+ radius \* ([0-9.]+)\);", viewer)
    assert near_formula
    assert far_formula

    # Roughly ur5_2f_test-sized bounds: workbench, robot, camera, and bins fit in
    # about a 2.0 m x 1.6 m x 0.85 m envelope. The fitted near/far values should
    # have a wide margin around the workcell rather than clipping table/camera.
    span = (2.0, 1.6, 0.85)
    scene_radius = max((sum((axis / 2.0) ** 2 for axis in span)) ** 0.5, min_radius)
    distance = max(scene_radius * distance_multiplier, min_radius * distance_multiplier)
    near = max(0.01, scene_radius / float(near_formula.group(1)))
    far = max(100.0, distance + scene_radius * float(far_formula.group(1)))

    assert near <= 0.02
    assert far >= 100.0
    assert far > distance + scene_radius
    assert far / near >= 5000
