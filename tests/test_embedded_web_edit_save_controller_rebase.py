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


def _run_rebase_harness(current_transform: dict) -> dict:
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
  dirtyTransforms:new Map([['support_surface_table', {{oldTransform:cloneTransform(original), newTransform:cloneTransform(current)}}]]),
  undoStack:[{{before:original,after:current}}],
  redoStack:[{{before:current,after:original}}],
  selected:'support_surface_table',
}};
const renderedById = id => state.objects.find(record => record.item.id === id) || null;
const canonicalTransformOwner = record => record;
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
    assert rebase.index("persistedPatchRebaseScript(active_patch_)") < rebase.index(
        "requestPostSaveProductViewRefresh();"
    )


def test_patch_transaction_is_captured_before_dry_run_and_fail_safe_blocks_stale_save():
    source = CONTROLLER.read_text(encoding="utf-8")
    request = _section(source, "void requestSave()", "void startWorkflow")
    assert request.index("active_patch_ = patch;") < request.index(
        "startWorkflow(WorkflowPhase::DryRun);"
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
    assert "reload_required_after_save_ = false" not in constructor
