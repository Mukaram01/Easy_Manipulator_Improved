from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
INDEX = ROOT / "workcell_studio_web/viewer/index.html"
GUARD = ROOT / "workcell_studio_web/viewer/live_duplicate_visibility_guard.js"


def test_product_view_loads_duplicate_visibility_guard_after_versioned_bundle():
    index = INDEX.read_text(encoding="utf-8")
    bundle = "import(`./dist/viewer.bundle.js?v=${viewerBuild}`)"
    guard = "import('./live_duplicate_visibility_guard.js')"
    assert bundle in index
    assert guard in index
    assert index.index(bundle) < index.index(guard)


def test_duplicate_visibility_guard_only_bridges_stale_mask_window():
    harness = r"""
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const source = fs.readFileSync(process.argv[1], 'utf8');
let lastVisible = [];
const calls = [];
const api = {
  duplicateItem(sourceId, item) { calls.push(['duplicate', sourceId, item.id]); return {ok:true}; },
  setVisibleItemIds(ids) { lastVisible = [...ids]; calls.push(['visible', ...ids]); return {ok:true}; },
  removeItem(id) { calls.push(['remove', id]); return {ok:true}; },
};
const window = {
  __WORKCELL_EDITOR_API_V1__: api,
  addEventListener() {},
};
vm.runInNewContext(source, {window, console});
assert.strictEqual(api.__WORKCELL_DUPLICATE_VISIBILITY_GUARD__, true);

api.duplicateItem('object_01', {id:'object_01_copy'});
api.setVisibleItemIds(['object_01', 'object_02']);
assert.deepStrictEqual(lastVisible, ['object_01', 'object_02', 'object_01_copy']);

// Once Qt catches up and explicitly includes the duplicate, the transient
// inheritance is retired. Future masks are authoritative again.
api.setVisibleItemIds(['object_01', 'object_01_copy', 'object_02']);
assert.deepStrictEqual(lastVisible, ['object_01', 'object_01_copy', 'object_02']);
api.setVisibleItemIds(['object_01', 'object_02']);
assert.deepStrictEqual(lastVisible, ['object_01', 'object_02']);

// If the source is genuinely hidden, do not force its duplicate visible.
api.duplicateItem('object_03', {id:'object_03_copy'});
api.setVisibleItemIds(['object_01']);
assert.deepStrictEqual(lastVisible, ['object_01']);
api.setVisibleItemIds(['object_01', 'object_03']);
assert.deepStrictEqual(lastVisible, ['object_01', 'object_03']);

// Removing a pending duplicate must not allow a later stale mask to revive it.
api.duplicateItem('object_04', {id:'object_04_copy'});
api.removeItem('object_04_copy');
api.setVisibleItemIds(['object_04']);
assert.deepStrictEqual(lastVisible, ['object_04']);
"""
    subprocess.run(
        ["node", "-e", harness, str(GUARD)],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
