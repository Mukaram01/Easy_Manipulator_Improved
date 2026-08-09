from __future__ import annotations

import json
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE = REPO_ROOT / "workcell_studio_web/viewer/canonical_helper_pick_proxies.js"
POST_BUNDLE = REPO_ROOT / "workcell_studio_web/viewer/post_bundle_viewer_modules.js"


def test_post_bundle_loader_imports_and_tracks_canonical_helper_pick_proxies() -> None:
    source = POST_BUNDLE.read_text(encoding="utf-8")
    assert "import './canonical_helper_pick_proxies.js';" in source
    assert "id: 'canonical-helper-pick-proxies'" in source
    assert "global: '__WORKCELL_CANONICAL_HELPER_PICK_PROXIES_V1__'" in source


def test_helper_proxy_module_does_not_create_editor_identity_records() -> None:
    source = MODULE.read_text(encoding="utf-8")
    assert "state.objects" not in source
    assert "state.pickRecords" not in source
    assert "registerPickRecord" not in source
    assert "__WORKCELL_EDITOR_API_V1__" in source
    assert "api.selectItem(ownerCandidate)" in source
    assert "window.__WORKCELL_CANONICAL_HELPER_PICK_PROXIES_V1__ = publicApi" in source


def test_canonical_helper_proxy_acceptance_harness() -> None:
    module_uri = MODULE.resolve().as_uri()
    script = f"""
import assert from 'node:assert/strict';
const proxy = await import({json.dumps(module_uri)});

const listeners = {{ capture: [], bubble: [] }};
const canvas = {{
  addEventListener(type, callback, options) {{
    if (type !== 'pointerdown') return;
    (options === true ? listeners.capture : listeners.bubble).push(callback);
  }},
}};

let state = {{
  selectedItemId: '',
  selectedEditable: false,
  lastCanvasPickReason: '',
  lastFailedCanvasPickDiagnostic: null,
  selectionDiagnostics: {{ gizmoAttachedTargetId: '' }},
}};
const forwarded = [];
const dispatched = [];
const canonicalOwners = new Map([
  ['target_bin_default', {{ id: 'target_bin_default', editable: true }}],
  ['support_surface_table', {{ id: 'support_surface_table', editable: true }}],
  ['urdf_visual_17_camera_link_visual_17_package_realsense2_description_meshes_d435_dae', {{ id: 'realsense_overhead', editable: true }}],
  ['ur5', {{ id: 'ur5', editable: false }}],
  ['robotiq_85_gripper', {{ id: 'robotiq_85_gripper', editable: false }}],
]);
const api = {{
  getState() {{ return state; }},
  selectItem(candidate) {{
    const owner = canonicalOwners.get(candidate) || {{ id: candidate, editable: false }};
    forwarded.push(candidate);
    state = {{
      ...state,
      selectedItemId: owner.id,
      selectedEditable: owner.editable,
      selectionDiagnostics: {{
        gizmoAttachedTargetId: owner.editable ? owner.id : '',
      }},
    }};
    return state;
  }},
  selectionDiagnostics() {{ return state.selectionDiagnostics; }},
}};
const windowRef = {{
  __WORKCELL_EDITOR_API_V1__: api,
  CustomEvent: class CustomEvent {{ constructor(type, init) {{ this.type = type; this.detail = init.detail; }} }},
  dispatchEvent(event) {{ dispatched.push(event); }},
}};
const documentRef = {{ getElementById(id) {{ return id === 'scene-canvas' ? canvas : null; }} }};
const microtasks = [];
assert.equal(proxy.installCanonicalHelperPickProxies({{
  windowRef,
  documentRef,
  scheduleMicrotask: callback => microtasks.push(callback),
}}), true);

function helperDiagnostic(name, selectionOwnerId = '') {{
  return {{
    hit_object_names: [name],
    hit_resolutions: [{{
      hit_node_name: name,
      selection_owner_id: selectionOwnerId,
      rejection_stage_reason: 'hit_node_intrinsically_excluded',
    }}],
  }};
}}
function clickHelper(name, selectionOwnerId = '') {{
  for (const callback of listeners.capture) callback({{}});
  state = {{
    ...state,
    selectedItemId: '',
    selectedEditable: false,
    lastCanvasPickReason: 'no_eligible_candidate',
    lastFailedCanvasPickDiagnostic: helperDiagnostic(name, selectionOwnerId),
    selectionDiagnostics: {{ gizmoAttachedTargetId: '' }},
  }};
  for (const callback of listeners.bubble) callback({{}});
  while (microtasks.length) microtasks.shift()();
  return state;
}}
function clickNormal(selectedItemId, selectedEditable) {{
  for (const callback of listeners.capture) callback({{}});
  state = {{
    ...state,
    selectedItemId,
    selectedEditable,
    lastCanvasPickReason: 'eligible_candidate',
    lastFailedCanvasPickDiagnostic: null,
    selectionDiagnostics: {{ gizmoAttachedTargetId: selectedEditable ? selectedItemId : '' }},
  }};
  for (const callback of listeners.bubble) callback({{}});
  while (microtasks.length) microtasks.shift()();
  return state;
}}

for (let i = 0; i < 20; ++i) {{
  const selected = clickHelper('target_bin_default_fallback_edges');
  assert.equal(selected.selectedItemId, 'target_bin_default');
  assert.equal(selected.selectedEditable, true);
  assert.equal(selected.selectionDiagnostics.gizmoAttachedTargetId, 'target_bin_default');
}}

const cameraVisual = 'urdf_visual_17_camera_link_visual_17_package_realsense2_description_meshes_d435_dae';
assert.equal(proxy.isCanonicalOwnerHelperName(`${{cameraVisual}}_fallback_sensor_frustum`), false);
for (let i = 0; i < 20; ++i) {{
  const forwardedBeforeFrustum = forwarded.length;
  let selected = clickHelper(`${{cameraVisual}}_fallback_sensor_frustum`, 'realsense_overhead');
  assert.equal(forwarded.length, forwardedBeforeFrustum);
  assert.equal(selected.selectedItemId, '');
  assert.equal(selected.selectedEditable, false);
  assert.equal(selected.selectionDiagnostics.gizmoAttachedTargetId, '');

  selected = clickNormal('realsense_overhead', true);
  assert.equal(selected.selectedItemId, 'realsense_overhead');
  assert.equal(selected.selectedEditable, true);
  assert.notEqual(selected.selectionDiagnostics.gizmoAttachedTargetId, `${{cameraVisual}}_fallback_sensor_frustum`);
}}

let selected = clickHelper('support_surface_table_fallback_edges');
assert.equal(selected.selectedItemId, 'support_surface_table');
assert.equal(selected.selectedEditable, true);
assert.equal(selected.selectionDiagnostics.gizmoAttachedTargetId, 'support_surface_table');

selected = clickHelper('ur5_fallback_edges');
assert.equal(selected.selectedItemId, 'ur5');
assert.equal(selected.selectedEditable, false);
assert.equal(selected.selectionDiagnostics.gizmoAttachedTargetId, '');

selected = clickHelper('robotiq_85_gripper_fallback_edges');
assert.equal(selected.selectedItemId, 'robotiq_85_gripper');
assert.equal(selected.selectedEditable, false);
assert.equal(selected.selectionDiagnostics.gizmoAttachedTargetId, '');

state = {{
  ...state,
  selectedItemId: 'target_bin_default',
  selectedEditable: true,
  lastCanvasPickReason: 'eligible_candidate',
  lastFailedCanvasPickDiagnostic: null,
  selectionDiagnostics: {{ gizmoAttachedTargetId: 'target_bin_default' }},
}};
selected = clickHelper('selection_subtle_bounds_highlight');
assert.equal(selected.selectedItemId, 'target_bin_default');
assert.equal(selected.selectionDiagnostics.gizmoAttachedTargetId, 'target_bin_default');

const forwardedBeforeOrdinaryFailure = forwarded.length;
clickHelper('unrelated_debug_helper');
assert.equal(forwarded.length, forwardedBeforeOrdinaryFailure);
assert.ok(forwarded.every(id => !proxy.isCanonicalOwnerHelperName(id)));
assert.equal(dispatched.length, 24);
assert.ok(dispatched.every(event => event.detail.gizmo_attached_to_helper === false));
console.log(JSON.stringify({{ forwardedCount: forwarded.length, dispatchedCount: dispatched.length }}));
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
    assert payload["forwardedCount"] == 24
