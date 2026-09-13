import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / "workcell_builder/workcell_builder/gui/embedded_web_edit_save_controller.hpp"


def _section(source: str, start: str, end: str) -> str:
    return source.split(start, 1)[1].split(end, 1)[0]


def _persisted_rebase_script(source: str, patch: dict) -> str:
    function = _section(
        source,
        "static QString persistedPatchRebaseScript(const QJsonObject & patch)",
        "void logPatchSummary",
    )
    raw = function.split('R"JS(', 1)[1].split(')JS"', 1)[0]
    return raw.replace("%1", json.dumps(patch, separators=(",", ":")), 1)


def _transform(x: float, y: float, yaw: float = 0.0) -> dict:
    return {
        "pose": {
            "xyz": {"x": x, "y": y, "z": 0.06},
            "rpy": {"x": 0.0, "y": 0.0, "z": yaw},
        },
        "scale": {"x": 1.0, "y": 1.0, "z": 1.0},
    }


def _run_rebase_harness(current_transform: dict, *, generated_canonical_owner: bool = False) -> dict:
    source = CONTROLLER.read_text(encoding="utf-8")
    original = _transform(0.55, 0.0)
    persisted = _transform(-0.32, -0.15, -2.5307274)
    second_edit = _transform(-0.22, -0.16, -2.5307274)
    patch = {
        "schema_version": "workcell_studio_web_scene_edit_patch/v1",
        "scene_id": "ur5_2f_test",
        "created_by": "static_web_viewer",
        "edits": [
            {
                "item_id": "support_surface_table",
                "operation": "update_transform",
                "old_transform": original,
                "new_transform": persisted,
            }
        ],
    }
    rebase_script = _persisted_rebase_script(source, patch)
    viewer = (ROOT / "workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")
    rebase_function = "function rebasePersistedPatch(patch)" + _section(
        viewer, "function rebasePersistedPatch(patch)", "window.__WORKCELL_EDITOR_API_V1__ ="
    )
    selection_functions = "function selectionOwnerRenderedById(id)" + _section(
        viewer, "function selectionOwnerRenderedById(id)", "function selectedRenderIdentity()"
    )
    harness = f"""
const assert = require('assert');
const cloneTransform = value => JSON.parse(JSON.stringify(value));
const finite = value => typeof value === 'number' && Number.isFinite(value);
const isFiniteTransform = transform => [
  transform?.pose?.xyz?.x, transform?.pose?.xyz?.y, transform?.pose?.xyz?.z,
  transform?.pose?.rpy?.x, transform?.pose?.rpy?.y, transform?.pose?.rpy?.z,
  transform?.scale?.x, transform?.scale?.y, transform?.scale?.z,
].every(finite);
const sameTransform = (left, right) => JSON.stringify(left) === JSON.stringify(right);
const original = {json.dumps(original)};
const persisted = {json.dumps(persisted)};
const current = {json.dumps(current_transform)};
const secondEdit = {json.dumps(second_edit)};
const rendered = {{
  item: {{id:'support_surface_table', editable:true, locked:false, pose:{{xyz:[0.55,0,0.06],rpy:[0,0,0]}}, scale:[1,1,1]}},
  object3d: {{transform:cloneTransform(current)}},
  originalTransform: cloneTransform(original),
}};
const state = {{
  objects:[rendered],
  pickRecords:[],
  selectionIdentityIndex:{{selectionOwners:[]}},
  dirtyTransforms:new Map([['support_surface_table', {{oldTransform:cloneTransform(original), newTransform:cloneTransform(current)}}]]),
  undoStack:[{{before:original,after:current}}],
  redoStack:[{{before:current,after:original}}],
  selected:'support_surface_table',
}};
const renderedById = id => state.objects.find(record => record.item.id === id) || null;
const generatedVisual = {{item:{{id:'generated_urdf::camera_visual',source_kind:'generated_preview',editable:false,locked:true}},object3d:{{transform:cloneTransform(original)}}}};
const visualBefore = JSON.stringify(generatedVisual);
const canonicalTransformOwner = record => {str(generated_canonical_owner).lower()} ? generatedVisual : record;
const isGeneratedUrdfItem = item => item?.source_kind === 'generated_preview';
{selection_functions}
const canEditItem = item => item?.editable === true && item?.locked !== true;
const transformFromObject = object => cloneTransform(object.transform);
const applyTransformToObject = (object, transform) => {{ object.transform = cloneTransform(transform); return true; }};
const updateDirtyState = () => {{}};
const updateLabels = () => {{}};
const emitDirtyChanged = () => {{}};
const syncInspectorTransformFields = () => {{}};
const populateInspector = () => {{}};
const pushEditorEvent = () => {{}};
const buildPatch = () => ({{
  schema_version:'workcell_studio_web_scene_edit_patch/v1',
  scene_id:'ur5_2f_test',
  edits:Array.from(state.dirtyTransforms, ([itemId, dirty]) => ({{
    item_id:itemId,
    operation:'update_transform',
    old_transform:cloneTransform(dirty.oldTransform),
    new_transform:cloneTransform(dirty.newTransform),
  }})),
}});
const window = {{__WORKCELL_EDITOR_API_V1__:{{
  getState:() => ({{sceneId:'ur5_2f_test', dirty:state.dirtyTransforms.size > 0, dirtyCount:state.dirtyTransforms.size}}),
  getEditPatch:() => buildPatch(),
}}}};
{rebase_function}
window.__WORKCELL_EDITOR_API_V1__.rebasePersistedPatch = rebasePersistedPatch;
const result = {rebase_script};
assert.strictEqual(result.ok, true, JSON.stringify(result));
assert.deepStrictEqual(rendered.originalTransform, persisted);
assert.strictEqual(state.undoStack.length, 0);
assert.strictEqual(state.redoStack.length, 0);
const afterRebasePatch = buildPatch();
if (sameTransform(current, persisted)) {{
  assert.strictEqual(result.clearedCount, 1);
  assert.strictEqual(result.preservedCount, 0);
  assert.strictEqual(state.dirtyTransforms.size, 0);
  assert.deepStrictEqual(rendered.object3d.transform, persisted);
  state.dirtyTransforms.set('support_surface_table', {{oldTransform:cloneTransform(rendered.originalTransform), newTransform:cloneTransform(secondEdit)}});
  const secondPatch = buildPatch();
  assert.deepStrictEqual(secondPatch.edits[0].old_transform, persisted);
  assert.deepStrictEqual(secondPatch.edits[0].new_transform, secondEdit);
}} else {{
  assert.strictEqual(result.clearedCount, 0);
  assert.strictEqual(result.preservedCount, 1);
  assert.strictEqual(state.dirtyTransforms.size, 1);
  assert.deepStrictEqual(afterRebasePatch.edits[0].old_transform, persisted);
  assert.deepStrictEqual(afterRebasePatch.edits[0].new_transform, current);
  assert.deepStrictEqual(rendered.object3d.transform, current);
}}
assert.strictEqual(JSON.stringify(generatedVisual), visualBefore, 'rebase must not mutate generated visual identity or local transform');
// Exact authored identity remains mandatory, even when a different physical
// visual is the canonical render owner. Generated/locked/unknown IDs must fail.
const savedItem = cloneTransform(rendered.item);
for (const invalid of [{{locked:true}}, {{editable:false}}, {{source_kind:'generated_preview'}}]) {{
  rendered.item = {{...savedItem,...invalid}};
  const rejected = window.__WORKCELL_EDITOR_API_V1__.rebasePersistedPatch({json.dumps(patch)});
  assert.strictEqual(rejected.ok, false);
  assert.strictEqual(rejected.error, 'persisted_owner_unavailable');
}}
rendered.item = savedItem;
const unknownPatch = {json.dumps(patch)};
unknownPatch.edits[0].item_id = 'unrelated_missing_owner';
assert.strictEqual(window.__WORKCELL_EDITOR_API_V1__.rebasePersistedPatch(unknownPatch).ok, false);
assert.strictEqual(JSON.stringify(generatedVisual), visualBefore);
console.log(JSON.stringify({{result, afterRebasePatch}}));
"""
    completed = subprocess.run(
        ["node", "-e", harness], cwd=ROOT, capture_output=True, text=True
    )
    assert completed.returncode == 0, completed.stderr
    return json.loads(completed.stdout)


def test_exact_persisted_edit_clears_dirty_and_second_save_uses_saved_baseline():
    result = _run_rebase_harness(_transform(-0.32, -0.15, -2.5307274))
    assert result["result"]["dirty"] is False
    assert result["result"]["dirtyCount"] == 0


def test_newer_browser_edit_is_preserved_with_persisted_old_transform():
    newer = _transform(-0.22, -0.16, -2.5307274)
    result = _run_rebase_harness(newer)
    assert result["result"]["dirty"] is True
    assert result["result"]["dirtyCount"] == 1
    edit = result["afterRebasePatch"]["edits"][0]
    assert edit["old_transform"] == _transform(-0.32, -0.15, -2.5307274)
    assert edit["new_transform"] == newer


def test_successful_write_rebases_before_forced_canonical_refresh():
    source = CONTROLLER.read_text(encoding="utf-8")
    callback = _section(
        source,
        "connect(process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished)",
        "process_->start();",
    )
    write_success = callback.split("if (!saveTargetContextIsActive()) {", 1)[1]
    assert 'logPhase(QStringLiteral("saved"))' in write_success
    assert "rebaseBrowserAfterPersistedWrite();" in write_success
    assert "request_post_save_product_view_refresh()" not in write_success

    rebase = _section(
        source,
        "void rebaseBrowserAfterPersistedWrite()",
        "bool resolveSaveContext",
    )
    assert "persistedPatchRebaseScript(active_patch_)" in rebase
    callback = rebase.split("view_->page()->runJavaScript", 1)[1]
    assert callback.index("persistedPatchRebaseScript(active_patch_)") < callback.index(
        "requestPostSaveProductViewRefresh();"
    )


def test_patch_transaction_is_captured_before_native_save_and_fail_safe_blocks_stale_save():
    source = CONTROLLER.read_text(encoding="utf-8")
    request = _section(source, "void requestSave()", "void startWorkflow")
    assert request.index("active_patch_ = patch;") < request.index(
        "native_save_(patch, &native_error)"
    )
    assert "if (reload_required_after_save_)" in request
    assert "Reload required before another save" in request

    poll = _section(source, "void pollEditorState()", "QPointer<ScenePreviewWidget>")
    assert "!reload_required_after_save_" in poll
    assert "last saved browser baseline could not be verified" in poll


def test_error_page_load_cannot_silently_clear_stale_save_guard():
    source = CONTROLLER.read_text(encoding="utf-8")
    constructor = _section(source, "EmbeddedWebEditSaveController(", "bool installed() const")
    assert "loadFinished" in constructor
    load_callback = constructor.split("connect(view_, &QWebEngineView::loadFinished", 1)[1]
    assert "reload_required_after_save_ = false" not in load_callback


def test_two_consecutive_saves_use_public_rebase_and_update_both_baselines():
    viewer = (ROOT / "workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")
    controller = CONTROLLER.read_text(encoding="utf-8")
    assert "function rebasePersistedPatch(patch)" in viewer
    assert "rebasePersistedPatch: patch => rebasePersistedPatch(patch)" in viewer
    assert "rendered.authoredBaselineTransform = cloneTransform(persisted);" in viewer
    rebase = _section(controller, "static QString persistedPatchRebaseScript", "void logPatchSummary")
    assert "api.rebasePersistedPatch(patch)" in rebase
    assert "typeof state === 'object'" not in rebase


def test_rebase_logical_owner_with_distinct_generated_canonical_owner():
    result = _run_rebase_harness(
        _transform(-0.32, -0.15, -2.5307274), generated_canonical_owner=True
    )
    assert result["result"]["ok"] is True
    assert result["result"]["clearedCount"] == 1
    assert result["afterRebasePatch"]["edits"] == []


def test_rebase_generated_canonical_owner_keeps_newer_logical_patch_identity():
    result = _run_rebase_harness(
        _transform(-0.22, -0.16, -2.5307274), generated_canonical_owner=True
    )
    assert result["result"]["preservedCount"] == 1
    assert result["afterRebasePatch"]["edits"][0]["item_id"] == "support_surface_table"


def test_failed_unavailable_and_timed_out_rebase_still_refresh_saved_yaml():
    source = CONTROLLER.read_text(encoding="utf-8")
    rebase = _section(source, "void rebaseBrowserAfterPersistedWrite()", "bool resolveSaveContext")
    unavailable = rebase.split("const quint64 transaction", 1)[0]
    failed = rebase.split("} else {", 1)[1].split("QTimer::singleShot", 1)[0]
    timeout = rebase.split("QTimer::singleShot", 1)[1]
    for path in (unavailable, failed, timeout):
        assert "requestPostSaveProductViewRefresh();" in path
        assert "busy_ = false" not in path
        assert "active_patch_ = QJsonObject{}" not in path
    assert "browser_rebase_succeeded_ = result.value" in rebase
    assert "browser_rebase_succeeded_ = false" in timeout
    assert "browser_rebase_pending_ = false" in timeout  # late JS callback is ignored
    assert "no Product View regeneration was requested" not in rebase


def test_matching_refresh_success_recovers_even_when_browser_rebase_failed():
    source = CONTROLLER.read_text(encoding="utf-8")
    callback = _section(source, "connect(preview_, &ScenePreviewWidget::post_save_product_view_refresh_finished", "connect(view_, &QWebEngineView::loadFinished")
    assert "revision != saved_reload_revision_" in callback
    success = callback.split("reload_required_after_save_ = false;", 1)[1]
    assert "restoreSelectionAfterReload();" in success
    assert "if (browser_rebase_succeeded_)" not in success
    preview = (CONTROLLER.parent / "scene_preview_widget.cpp").read_text(encoding="utf-8")
    finish = _section(preview, "bool ScenePreviewWidget::finish_post_save_product_view_refresh(", "void ScenePreviewWidget::ensure_embedded_web_server_started")
    assert finish.index("identity.generation != post_save_refresh_generation_") < finish.index("if (success) persisted_product_view_stale_ = false;")
    assert finish.index("identity.payload_revision) != post_save_refresh_payload_revision_") < finish.index("if (success) persisted_product_view_stale_ = false;")
    assert "browser_rebase" not in finish
