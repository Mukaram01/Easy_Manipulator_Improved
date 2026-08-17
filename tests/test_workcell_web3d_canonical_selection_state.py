from __future__ import annotations

import json
import subprocess
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE = REPO_ROOT / "workcell_studio_web/viewer/canonical_selection_state.js"
POST_BUNDLE = REPO_ROOT / "workcell_studio_web/viewer/post_bundle_viewer_modules.js"
LAYOUT = REPO_ROOT / "scenes/ur5_2f_test/layout/workcell_studio_layout.yaml"
SAVE_CONTROLLER = REPO_ROOT / "workcell_builder/workcell_builder/gui/embedded_web_edit_save_controller.hpp"


def test_post_bundle_loader_imports_and_tracks_canonical_selection_state() -> None:
    source = POST_BUNDLE.read_text(encoding="utf-8")
    assert "import './canonical_selection_state.js';" in source
    assert source.index("canonical_selection_state.js") < source.index("canonical_helper_pick_proxies.js")
    assert "id: 'canonical-selection-state'" in source
    assert "global: '__WORKCELL_CANONICAL_SELECTION_STATE_V1__'" in source
    assert "canonicalSelectedOwnerId" in source
    assert "canonicalTransform" in source


def test_canonical_selection_module_uses_only_the_public_editor_contract() -> None:
    source = MODULE.read_text(encoding="utf-8")
    assert "__WORKCELL_EDITOR_API_V1__" in source
    assert "api.getState" in source
    assert "api.selectItem" in source
    assert "api.getEditPatch" in source
    assert "state.objects" not in source
    assert "state.pickRecords" not in source
    assert "registerPickRecord" not in source
    assert "canonicalSelectedOwnerId" in source
    assert "canonicalTransform" in source
    assert "retainedSelectionPending" in source
    assert "event?.type === 'selection_changed'" in source
    assert "itemId: eventOwner" in source
    assert "uiItemId: eventOwner" in source


def test_ur5_2f_editable_six_vs_layout_seven_is_intentional() -> None:
    payload = yaml.safe_load(LAYOUT.read_text(encoding="utf-8"))
    editable = [item for item in payload["items"] if item.get("editable") is True and item.get("locked") is not True]
    derived = [
        item
        for item in editable
        if item.get("role") == "place_zone" and item.get("target_ref") and item.get("transform_group")
    ]
    canonical = [item for item in editable if item not in derived]

    assert len(editable) == 7
    assert [item["id"] for item in derived] == ["place_zone_default"]
    assert len(canonical) == 6
    assert "target_bin_default" in {item["id"] for item in canonical}
    assert derived[0]["target_ref"] == "target_bin_default"
    assert derived[0]["transform_group"] == "default_drop_destination"


def test_save_logs_use_exported_patch_old_and_new_xyz_rpy() -> None:
    source = SAVE_CONTROLLER.read_text(encoding="utf-8")
    assert "logPatchSummary(patch)" in source
    assert "patch edit: item_id=%1 operation=%2 old_%3 new_%4 persistence_source=%5" in source
    assert "xyz=[%1,%2,%3] rpy=[%4,%5,%6]" in source
    assert "transformText(edit.value(QStringLiteral(\"old_transform\")))" in source
    assert "transformText(edit.value(QStringLiteral(\"new_transform\")))" in source


def test_canonical_selection_transform_acceptance_harness() -> None:
    module_uri = MODULE.resolve().as_uri()
    script = f"""
import assert from 'node:assert/strict';
const canonical = await import({json.dumps(module_uri)});

const clone = value => JSON.parse(JSON.stringify(value));
const first = {{
  pose: {{xyz: {{x: -0.32, y: -0.15, z: 0.06}}, rpy: {{x: 0, y: 0, z: -2.5307274}}}},
  scale: {{x: 1, y: 1, z: 1}},
}};
const second = {{
  pose: {{xyz: {{x: -0.22, y: -0.16, z: 0.06}}, rpy: {{x: 0, y: 0, z: -2.44346095}}}},
  scale: {{x: 1, y: 1, z: 1}},
}};

function storage() {{
  const values = new Map();
  return {{
    getItem(key) {{ return values.has(key) ? values.get(key) : null; }},
    setItem(key, value) {{ values.set(key, String(value)); }},
    removeItem(key) {{ values.delete(key); }},
    values,
  }};
}}

function inspector() {{
  const fields = Object.fromEntries(['x','y','z','roll','pitch','yaw','scale_x','scale_y','scale_z'].map(name => [name, {{value:'0'}}]));
  const rows = new Map();
  for (const label of ['pose xyz', 'pose rpy', 'scale']) {{
    const dd = {{textContent:''}};
    rows.set(label, {{textContent:label, nextElementSibling:dd, dd}});
  }}
  return {{
    fields,
    rows,
    querySelector(selector) {{
      const match = selector.match(/data-transform-field=\"([^\"]+)\"/);
      return match ? fields[match[1]] || null : null;
    }},
    querySelectorAll(selector) {{ return selector === 'dt' ? [...rows.values()] : []; }},
  }};
}}

function windowHarness(sharedStorage) {{
  const listeners = new Map();
  const timers = [];
  const dispatched = [];
  return {{
    sessionStorage: sharedStorage,
    timers,
    dispatched,
    CustomEvent: class CustomEvent {{ constructor(type, init) {{ this.type = type; this.detail = init?.detail || {{}}; }} }},
    addEventListener(type, callback) {{
      if (!listeners.has(type)) listeners.set(type, []);
      listeners.get(type).push(callback);
    }},
    dispatchEvent(event) {{
      dispatched.push(event);
      for (const callback of listeners.get(event.type) || []) callback(event);
    }},
    setTimeout(callback) {{ timers.push(callback); return timers.length; }},
    flushTimers(limit = 500) {{
      let count = 0;
      while (timers.length && count++ < limit) timers.shift()();
      assert.ok(count < limit, 'restore timer loop did not settle');
    }},
  }};
}}

const sharedStorage = storage();
const inspectorOne = inspector();
const windowOne = windowHarness(sharedStorage);
let coreState = {{
  ready: true,
  sceneId: 'ur5_2f_test',
  selectedItemId: 'generated_urdf::camera_link::visual_17',
  uiSelectionItemId: 'realsense_overhead',
  editOwnerItemId: 'realsense_overhead',
  selectedEditable: true,
  mode: 'move',
  dirty: true,
  dirtyCount: 1,
}};
let currentTransform = clone(second);
let events = [
  {{type:'selection_changed', itemId:''}},
  {{type:'selection_changed', itemId:'realsense_overhead'}},
  {{type:'selection_changed', itemId:''}},
  {{type:'transform_committed', itemId:'realsense_overhead', patchEntry:{{new_transform:clone(second)}}}},
];
let gizmoTarget = 'realsense_overhead';
const originalSelections = [];
const apiOne = {{
  getState() {{ return clone(coreState); }},
  selectItem(id) {{
    originalSelections.push(id);
    coreState = {{...coreState, selectedItemId:id === 'generated_urdf::camera_link::visual_17' ? id : 'realsense_overhead', uiSelectionItemId:'realsense_overhead', editOwnerItemId:'realsense_overhead', selectedEditable:true}};
    return clone(coreState);
  }},
  selectionDiagnostics() {{ return {{gizmoAttachedTargetId:gizmoTarget}}; }},
  clearSelection() {{ coreState = {{...coreState, selectedItemId:'', uiSelectionItemId:'', editOwnerItemId:'', selectedEditable:false}}; return clone(coreState); }},
  setMode(mode) {{ coreState = {{...coreState, mode}}; gizmoTarget = mode === 'select' ? '' : 'realsense_overhead'; return clone(coreState); }},
  undo() {{ currentTransform = clone(first); return clone(coreState); }},
  redo() {{ currentTransform = clone(second); return clone(coreState); }},
  getEditPatch() {{ return {{schema_version:'workcell_studio_web_scene_edit_patch/v1', scene_id:'ur5_2f_test', edits:[{{item_id:'realsense_overhead', old_transform:clone(first), new_transform:clone(currentTransform)}}]}}; }},
  drainEvents() {{ const drained = events; events = []; return drained; }},
}};
windowOne.__WORKCELL_EDITOR_API_V1__ = apiOne;
const documentOne = {{getElementById(id) {{ return id === 'inspector' ? inspectorOne : null; }}}};
assert.equal(canonical.installCanonicalSelectionState({{windowRef:windowOne, documentRef:documentOne, storage:sharedStorage, scheduleMicrotask:callback=>callback(), scheduleTimeout:callback=>windowOne.setTimeout(callback)}}), true);

let state = apiOne.getState();
assert.equal(state.canonicalSelectedOwnerId, 'realsense_overhead');
assert.deepEqual(state.canonicalTransform, second);
assert.equal(state.canonicalTransformSource, 'edit_patch');
assert.equal(inspectorOne.fields.x.value, '-0.220000');
assert.equal(inspectorOne.fields.yaw.value, '-2.443461');
assert.equal(inspectorOne.rows.get('pose xyz').dd.textContent, '-0.220, -0.160, 0.060');
assert.equal(inspectorOne.rows.get('pose rpy').dd.textContent, '0.000, 0.000, -2.443');

let diagnostics = apiOne.selectionDiagnostics();
assert.equal(diagnostics.canonicalSelectedOwnerId, 'realsense_overhead');
assert.equal(diagnostics.gizmoAttachedTargetId, 'realsense_overhead');
assert.equal(diagnostics.gizmoOwnerMatchesSelection, true);
assert.deepEqual(diagnostics.canonicalTransform, second);
assert.deepEqual(diagnostics.gizmoTransform, second);
assert.deepEqual(diagnostics.inspectorTransform, {{...second, pose:{{...second.pose, rpy:{{...second.pose.rpy, z:-2.443461}}}}}});
assert.deepEqual(diagnostics.patchTransform, second);
assert.deepEqual(apiOne.getEditPatch().edits[0].new_transform, second);

apiOne.setMode('rotate');
assert.equal(apiOne.getState().canonicalSelectedOwnerId, 'realsense_overhead');
assert.equal(apiOne.selectionDiagnostics().gizmoAttachedTargetId, 'realsense_overhead');

apiOne.undo();
assert.deepEqual(apiOne.getState().canonicalTransform, first);
assert.equal(inspectorOne.fields.x.value, '-0.320000');
assert.equal(inspectorOne.rows.get('pose xyz').dd.textContent, '-0.320, -0.150, 0.060');
apiOne.redo();
assert.deepEqual(apiOne.getState().canonicalTransform, second);
assert.equal(inspectorOne.fields.x.value, '-0.220000');

const drained = apiOne.drainEvents();
const selectionEvents = drained.filter(event => event.type === 'selection_changed');
assert.deepEqual(selectionEvents.map(event => event.itemId), ['', 'realsense_overhead']);
assert.equal(selectionEvents[0].uiItemId, '');
const committed = drained.find(event => event.type === 'transform_committed');
assert.equal(committed.canonicalOwnerItemId, 'realsense_overhead');
assert.deepEqual(committed.canonicalTransform, second);
assert.deepEqual(committed.savedXyz, second.pose.xyz);
assert.deepEqual(committed.savedRpy, second.pose.rpy);

const audit = canonical.auditEditableLayoutOwners([
  {{id:'support_surface_table', editable:true, locked:false}},
  {{id:'pick_zone_commissioning', editable:true, locked:false}},
  {{id:'place_zone_default', role:'place_zone', target_ref:'target_bin_default', transform_group:'default_drop_destination', editable:true, locked:false}},
  {{id:'target_bin_default', editable:true, locked:false}},
  {{id:'realsense_overhead', editable:true, locked:false}},
  {{id:'safety_zone_keepout', editable:true, locked:false}},
  {{id:'home_pose_safe', editable:true, locked:false}},
]);
assert.equal(audit.layoutEditableCount, 7);
assert.equal(audit.canonicalEditableOwnerCount, 6);
assert.deepEqual(audit.derivedEditableIds, ['place_zone_default']);

// Simulate the new page created by the canonical Product View reload. The
// stored owner remains visible to Qt polling until scene_ready restores it.
const inspectorTwo = inspector();
const windowTwo = windowHarness(sharedStorage);
let reloadState = {{
  ready: false,
  sceneId: 'ur5_2f_test',
  selectedItemId: '',
  uiSelectionItemId: '',
  editOwnerItemId: '',
  selectedEditable: false,
  mode: 'select',
  dirty: false,
  dirtyCount: 0,
}};
const restoreCalls = [];
const apiTwo = {{
  getState() {{ return clone(reloadState); }},
  selectItem(id) {{ restoreCalls.push(id); reloadState = {{...reloadState, selectedItemId:id, uiSelectionItemId:id, editOwnerItemId:id, selectedEditable:true}}; return clone(reloadState); }},
  selectionDiagnostics() {{ return {{gizmoAttachedTargetId:''}}; }},
  setMode(mode) {{ reloadState = {{...reloadState, mode}}; return clone(reloadState); }},
  undo() {{ return clone(reloadState); }},
  redo() {{ return clone(reloadState); }},
  getEditPatch() {{ return {{edits:[]}}; }},
  drainEvents() {{ return []; }},
}};
windowTwo.__WORKCELL_EDITOR_API_V1__ = apiTwo;
const documentTwo = {{getElementById(id) {{ return id === 'inspector' ? inspectorTwo : null; }}}};
assert.equal(canonical.installCanonicalSelectionState({{windowRef:windowTwo, documentRef:documentTwo, storage:sharedStorage, scheduleMicrotask:callback=>callback(), scheduleTimeout:callback=>windowTwo.setTimeout(callback)}}), true);
state = apiTwo.getState();
assert.equal(state.selectedItemId, 'realsense_overhead');
assert.equal(state.canonicalSelectedOwnerId, 'realsense_overhead');
assert.equal(state.retainedSelectionPending, true);
reloadState = {{...reloadState, ready:true}};
windowTwo.dispatchEvent(new windowTwo.CustomEvent('workcell:web3d_readiness', {{detail:{{state:'scene_ready'}}}}));
windowTwo.flushTimers();
assert.deepEqual(restoreCalls, ['realsense_overhead']);
state = apiTwo.getState();
assert.equal(state.selectedItemId, 'realsense_overhead');
assert.equal(state.retainedSelectionPending, false);
const reloadEvents = apiTwo.drainEvents().filter(event => event.type === 'selection_changed');
assert.deepEqual(reloadEvents.map(event => [event.itemId, event.uiItemId]), [['realsense_overhead', 'realsense_overhead']]);

console.log(JSON.stringify({{
  owner: state.canonicalSelectedOwnerId,
  originalSelections,
  restoreCalls,
  audit,
}}));
"""
    result = subprocess.run(
        ["node", "--input-type=module", "-e", script],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr or result.stdout
    payload = json.loads(result.stdout.strip().splitlines()[-1])
    assert payload["owner"] == "realsense_overhead"
    assert payload["restoreCalls"] == ["realsense_overhead"]
    assert payload["audit"]["canonicalEditableOwnerCount"] == 6


def test_explicit_blank_selection_cancels_retention_and_stale_restore_callbacks() -> None:
    module_uri = MODULE.resolve().as_uri()
    script = f"""
import assert from 'node:assert/strict';
const canonical = await import({json.dumps(module_uri)});
const key = 'workcell_studio.canonical_selection.ur5_2f_test';
const retained = JSON.stringify({{sceneId:'ur5_2f_test', ownerId:'object_01', editable:true, transform:null}});

function makeHarness({{clearEvent=false}}={{}}) {{
  const values = new Map([[key, retained]]);
  const storage = {{getItem:k=>values.get(k) ?? null, setItem:(k,v)=>values.set(k,String(v)), removeItem:k=>values.delete(k)}};
  const timers = [];
  const listeners = new Map();
  const windowRef = {{sessionStorage:storage, setTimeout:cb=>timers.push(cb), addEventListener(type,cb){{(listeners.get(type) || (listeners.set(type,[]), listeners.get(type))).push(cb);}}, dispatchEvent(event){{for(const cb of listeners.get(event.type)||[]) cb(event);}}, CustomEvent:class {{constructor(type,init){{this.type=type;this.detail=init?.detail||{{}};}}}}}};
  let state = {{ready:false,sceneId:'ur5_2f_test',selectedItemId:'',uiSelectionItemId:'',editOwnerItemId:'',selectedEditable:false}};
  let events = clearEvent ? [{{type:'selection_changed',itemId:'',uiItemId:''}}] : [];
  const selected = [];
  const api = {{
    getState:()=>({{...state}}),
    selectItem(id){{selected.push(id);state={{...state,selectedItemId:id,uiSelectionItemId:id,editOwnerItemId:id,selectedEditable:true}};return {{...state}};}},
    clearSelection(){{state={{...state,selectedItemId:'',uiSelectionItemId:'',editOwnerItemId:'',selectedEditable:false}};return {{...state}};}},
    getEditPatch:()=>({{edits:[]}}),
    drainEvents(){{const result=events;events=[];return result;}},
  }};
  windowRef.__WORKCELL_EDITOR_API_V1__=api;
  canonical.installCanonicalSelectionState({{windowRef,documentRef:{{getElementById:()=>null}},storage,scheduleMicrotask:cb=>cb(),scheduleTimeout:cb=>timers.push(cb)}});
  return {{api,windowRef,values,timers,selected,setReady(){{state={{...state,ready:true}};}}}};
}}

const explicit = makeHarness({{clearEvent:true}});
assert.equal(explicit.api.getState().retainedSelectionPending,true);
const staleExplicit = explicit.timers.shift();
const drained = explicit.api.drainEvents();
assert.deepEqual(drained.filter(e=>e.type==='selection_changed').map(e=>[e.itemId,e.uiItemId]),[['','']]);
assert.equal(explicit.api.getState().canonicalSelectedOwnerId,'');
assert.equal(explicit.api.getState().selectedItemId,'');
assert.equal(explicit.api.getState().retainedSelectionPending,false);
assert.equal(explicit.values.has(key),false);
explicit.setReady(); staleExplicit();
assert.deepEqual(explicit.selected,[]);
assert.equal(explicit.api.drainEvents().some(e=>e.itemId==='object_01'),false);

const publicClear = makeHarness();
const stalePublic = publicClear.timers.shift();
publicClear.api.clearSelection(); publicClear.setReady(); stalePublic();
assert.deepEqual(publicClear.selected,[]);
assert.equal(publicClear.values.has(key),false);
assert.equal(publicClear.api.getState().retainedSelectionPending,false);

for (const lifecycle of ['scene_ready','pageshow']) {{
  const reload = makeHarness(); reload.setReady();
  if (lifecycle === 'scene_ready') reload.windowRef.dispatchEvent(new reload.windowRef.CustomEvent('workcell:web3d_readiness',{{detail:{{state:'scene_ready'}}}}));
  else reload.windowRef.dispatchEvent(new reload.windowRef.CustomEvent('pageshow'));
  while(reload.timers.length) reload.timers.shift()();
  assert.deepEqual(reload.selected,['object_01']);
  assert.equal(reload.api.getState().canonicalSelectedOwnerId,'object_01');
}}
"""
    result = subprocess.run(["node", "--input-type=module", "-e", script], cwd=REPO_ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr or result.stdout
