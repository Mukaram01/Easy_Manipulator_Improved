import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';

const PATCH_FLAG = Symbol.for('workcell-studio.collision-placement-validation.v1');
const CLEARANCE_M = 0.01;
const OVERLAP_EPSILON_M = 1e-5;
const VALID_COLOR = 0x2f8f5b;
const NEAR_COLOR = 0xb26b00;
const INVALID_COLOR = 0xc43434;
const SUPPORT_KINDS = new Set(['workbench_body', 'table_surface', 'tabletop']);

const runtime = {
  scene: null,
  canvas: null,
  gizmo: null,
  active: null,
  helpers: new Map(),
  statusNode: null,
  feedbackTimer: null,
  listenersInstalled: false,
  gizmosInstalled: new WeakSet(),
  replayingObjectChange: false,
  numericBefore: null,
  lastResult: null,
};

function editorApi() {
  return window.__WORKCELL_EDITOR_API_V1__ || null;
}

function editorState() {
  try {
    return editorApi()?.getState?.() || null;
  } catch (_) {
    return null;
  }
}

function itemOf(node) {
  return node?.userData?.item || null;
}

function itemId(item) {
  return String(item?.id || '').trim();
}

function itemLabel(item, fallback = 'object') {
  return item?.label || item?.display_name || item?.object_name || item?.name || item?.id || fallback;
}

function supportKind(item) {
  return String(item?.support_surface_kind || item?.supportSurfaceKind || '').trim().toLowerCase();
}

function rootForItemId(id) {
  if (!runtime.scene || !id) return null;
  let found = null;
  runtime.scene.traverse(node => {
    if (!found && itemId(itemOf(node)) === String(id)) found = node;
  });
  if (!found) return null;
  while (found.parent && found.parent !== runtime.scene && itemId(itemOf(found.parent)) === String(id)) {
    found = found.parent;
  }
  return found;
}

function worldVisible(node) {
  for (let current = node; current; current = current.parent) {
    if (current.visible === false) return false;
  }
  return true;
}

function identityFor(node) {
  const item = itemOf(node);
  return [
    node?.name,
    node?.userData?.role,
    node?.userData?.category,
    node?.userData?.source_layer,
    node?.userData?.render_status,
    item?.role,
    item?.category,
    item?.type,
    item?.source_layer,
    item?.active_visual_source,
  ].map(value => String(value || '').toLowerCase().replace(/[_-]+/g, ' ')).join(' ');
}

function excludedNode(node) {
  for (let current = node; current && current !== runtime.scene; current = current.parent) {
    const data = current.userData || {};
    const item = itemOf(current);
    if (
      current instanceof TransformControls ||
      current.isGridHelper ||
      current.isAxesHelper ||
      data.exclude_from_fit_bounds === true ||
      data.exclude_from_physical_bounds === true ||
      data.helper_overlay === true ||
      data.debug_only === true ||
      data.diagnostic_only === true ||
      item?.exclude_from_fit_bounds === true ||
      item?.exclude_from_physical_bounds === true ||
      item?.debug_overlay === true ||
      /\b(debug|diagnostic|overlay|helper|zone|reachability|field of view|fov|warning|gizmo)\b/.test(identityFor(current))
    ) return true;
  }
  return false;
}

function visibleBounds(root) {
  if (!root) return null;
  root.updateWorldMatrix?.(true, true);
  const box = new THREE.Box3();
  let count = 0;
  root.traverse(node => {
    if (!node?.isMesh || !worldVisible(node) || excludedNode(node) || !node.geometry) return;
    if (!node.geometry.boundingBox) node.geometry.computeBoundingBox?.();
    const local = node.geometry.boundingBox;
    if (!local || local.isEmpty()) return;
    const world = local.clone().applyMatrix4(node.matrixWorld);
    const values = [world.min.x, world.min.y, world.min.z, world.max.x, world.max.y, world.max.z];
    if (!values.every(Number.isFinite)) return;
    box.union(world);
    count += 1;
  });
  return count && !box.isEmpty() ? box : null;
}

function ancestorItemNode(node) {
  for (let current = node; current && current !== runtime.scene; current = current.parent) {
    if (itemOf(current)) return current;
  }
  return null;
}

function isDescendantOf(node, root) {
  for (let current = node; current; current = current.parent) {
    if (current === root) return true;
  }
  return false;
}

function collisionCandidates(selectedRoot, selectedId) {
  if (!runtime.scene) return [];
  const candidates = [];
  const seenRoots = new Set();

  runtime.scene.traverse(node => {
    const item = itemOf(node);
    const id = itemId(item);
    if (!id || id === String(selectedId) || SUPPORT_KINDS.has(supportKind(item)) || excludedNode(node)) return;
    const root = rootForItemId(id);
    if (!root || root === selectedRoot || seenRoots.has(root.uuid)) return;
    seenRoots.add(root.uuid);
    const box = visibleBounds(root);
    if (box) candidates.push({ root, item, box, label: itemLabel(item) });
  });

  runtime.scene.traverse(node => {
    if (!node?.isMesh || !worldVisible(node) || excludedNode(node) || isDescendantOf(node, selectedRoot)) return;
    if (ancestorItemNode(node)) return;
    const box = visibleBounds(node);
    if (!box) return;
    candidates.push({ root: node, item: null, box, label: node.name || node.parent?.name || 'fixed geometry' });
  });

  return candidates;
}

function penetrationDepth(a, b) {
  return {
    x: Math.min(a.max.x, b.max.x) - Math.max(a.min.x, b.min.x),
    y: Math.min(a.max.y, b.max.y) - Math.max(a.min.y, b.min.y),
    z: Math.min(a.max.z, b.max.z) - Math.max(a.min.z, b.min.z),
  };
}

function boxesOverlap(a, b) {
  const depth = penetrationDepth(a, b);
  return depth.x > OVERLAP_EPSILON_M && depth.y > OVERLAP_EPSILON_M && depth.z > OVERLAP_EPSILON_M;
}

function validateRoot(root, id, { feedback = false } = {}) {
  const objectBox = visibleBounds(root);
  if (!objectBox) return { valid: true, severity: 'valid', item_id: id, objectBox: null, collisions: [], nearby: [] };
  const collisions = [];
  const nearby = [];
  const clearanceBox = objectBox.clone().expandByScalar(CLEARANCE_M);

  for (const candidate of collisionCandidates(root, id)) {
    if (boxesOverlap(objectBox, candidate.box)) collisions.push(candidate);
    else if (clearanceBox.intersectsBox(candidate.box)) nearby.push(candidate);
  }

  const severity = collisions.length ? 'invalid' : (nearby.length ? 'warning' : 'valid');
  const result = {
    valid: collisions.length === 0,
    severity,
    item_id: String(id || ''),
    clearance_m: CLEARANCE_M,
    objectBox,
    collisions,
    nearby,
    reason: collisions.length
      ? `Invalid: overlaps ${collisions.slice(0, 3).map(entry => entry.label).join(', ')}`
      : nearby.length
        ? `Close to ${nearby.slice(0, 3).map(entry => entry.label).join(', ')} · less than 10 mm clearance`
        : 'Valid placement · 10 mm clearance',
  };
  runtime.lastResult = result;
  root.userData.collision_validation = {
    valid: result.valid,
    severity: result.severity,
    collisions: collisions.map(entry => entry.label),
    nearby: nearby.map(entry => entry.label),
    clearance_m: CLEARANCE_M,
  };
  if (feedback) showFeedback(result);
  publishResult(result);
  return result;
}

function snapshot(root) {
  return {
    position: root.position.clone(),
    quaternion: root.quaternion.clone(),
    scale: root.scale.clone(),
  };
}

function restore(root, saved) {
  if (!root || !saved) return;
  root.position.copy(saved.position);
  root.quaternion.copy(saved.quaternion);
  root.scale.copy(saved.scale);
  root.updateWorldMatrix?.(true, true);
}

function snapshotChanged(root, saved) {
  return Boolean(saved && (
    root.position.distanceToSquared(saved.position) > 1e-12 ||
    1 - Math.abs(root.quaternion.dot(saved.quaternion)) > 1e-10 ||
    root.scale.distanceToSquared(saved.scale) > 1e-12
  ));
}

function helperFor(name, box, color) {
  let helper = runtime.helpers.get(name);
  if (!helper) {
    helper = new THREE.Box3Helper(box.clone(), color);
    helper.name = `collision_validation_${name}`;
    helper.renderOrder = 1001;
    helper.userData.exclude_from_fit_bounds = true;
    helper.userData.exclude_from_physical_bounds = true;
    helper.userData.helper_overlay = true;
    helper.material.depthTest = false;
    helper.material.transparent = true;
    helper.material.opacity = 0.95;
    runtime.scene?.add(helper);
    runtime.helpers.set(name, helper);
  }
  helper.box.copy(box);
  helper.material.color.setHex(color);
  helper.visible = true;
  helper.updateMatrixWorld(true);
  return helper;
}

function ensureStatusNode() {
  if (runtime.statusNode?.isConnected) return runtime.statusNode;
  const host = document.querySelector('.viewport-panel');
  if (!host) return null;
  const node = document.createElement('div');
  node.id = 'collision-validation-status';
  node.setAttribute('role', 'status');
  node.setAttribute('aria-live', 'polite');
  node.hidden = true;
  node.style.cssText = 'position:absolute;left:50%;bottom:3.35rem;transform:translateX(-50%);z-index:6;max-width:min(38rem,calc(100% - 2rem));padding:.48rem .72rem;border:1px solid currentColor;border-radius:.45rem;box-shadow:0 .15rem .55rem rgba(23,32,42,.18);font:600 .82rem/1.3 Inter,ui-sans-serif,system-ui,sans-serif;text-align:center;pointer-events:none';
  host.appendChild(node);
  runtime.statusNode = node;
  return node;
}

function showFeedback(result) {
  for (const helper of runtime.helpers.values()) helper.visible = false;
  const color = result.severity === 'invalid' ? INVALID_COLOR : (result.severity === 'warning' ? NEAR_COLOR : VALID_COLOR);
  if (result.objectBox) helperFor('selected', result.objectBox, color);
  const other = result.collisions[0] || result.nearby[0];
  if (other?.box) helperFor('other', other.box, color);
  const node = ensureStatusNode();
  if (!node) return;
  node.hidden = false;
  node.textContent = result.reason;
  node.style.color = result.severity === 'invalid' ? '#8b1e1e' : (result.severity === 'warning' ? '#7a4600' : '#14532d');
  node.style.background = result.severity === 'invalid' ? 'rgba(255,238,240,.97)' : (result.severity === 'warning' ? 'rgba(255,246,221,.97)' : 'rgba(236,253,245,.97)');
}

function clearFeedback(delayMs = 0) {
  window.clearTimeout(runtime.feedbackTimer);
  const clear = () => {
    for (const helper of runtime.helpers.values()) {
      helper.parent?.remove(helper);
      helper.geometry?.dispose?.();
      helper.material?.dispose?.();
    }
    runtime.helpers.clear();
    if (runtime.statusNode) runtime.statusNode.hidden = true;
  };
  if (delayMs > 0) runtime.feedbackTimer = window.setTimeout(clear, delayMs);
  else clear();
}

function publishResult(result) {
  const detail = {
    valid: result.valid,
    severity: result.severity,
    item_id: result.item_id,
    clearance_m: CLEARANCE_M,
    collisions: result.collisions?.map(entry => entry.label) || [],
    nearby: result.nearby?.map(entry => entry.label) || [],
    reason: result.reason,
  };
  window.dispatchEvent?.(new CustomEvent('workcell:collision-placement', { detail }));
  window.parent?.postMessage?.({ type: 'workcell_collision_placement', ...detail }, '*');
}

function beginActive(source, pointerId = null) {
  const state = editorState();
  if (!state || !['move', 'rotate'].includes(state.mode) || state.selectedEditable !== true || !state.selectedItemId) return null;
  const root = rootForItemId(state.selectedItemId);
  if (!root) return null;
  const start = snapshot(root);
  runtime.active = {
    source,
    pointerId,
    mode: state.mode,
    itemId: state.selectedItemId,
    root,
    start,
    lastValid: start,
    invalid: false,
  };
  validateRoot(root, state.selectedItemId, { feedback: false });
  return runtime.active;
}

function evaluateActive({ blockGizmo = false } = {}) {
  const active = runtime.active;
  if (!active) return null;
  const attempted = validateRoot(active.root, active.itemId, { feedback: true });
  active.invalid = !attempted.valid;
  if (attempted.valid) {
    active.lastValid = snapshot(active.root);
    return attempted;
  }
  if (blockGizmo && active.lastValid) {
    restore(active.root, active.lastValid);
    if (runtime.gizmo && !runtime.replayingObjectChange) {
      runtime.replayingObjectChange = true;
      runtime.gizmo.dispatchEvent({ type: 'objectChange' });
      runtime.replayingObjectChange = false;
    }
    active.invalid = false;
  }
  return attempted;
}

function cancelEditorDrag(active, message = 'Invalid placement restored') {
  const api = editorApi();
  const mode = active?.mode || editorState()?.mode || 'move';
  api?.setMode?.('select');
  restore(active?.root, active?.start);
  api?.setMode?.(mode);
  const result = validateRoot(active?.root, active?.itemId, { feedback: false });
  showFeedback({ ...result, severity: 'invalid', valid: false, reason: message });
}

function onCanvasPointerDown(event) {
  beginActive('canvas', event.pointerId);
}

function onCanvasPointerMove(event) {
  const active = runtime.active;
  if (!active || active.source !== 'canvas' || active.pointerId !== event.pointerId) return;
  if (!snapshotChanged(active.root, active.lastValid)) return;
  evaluateActive();
}

function onCanvasPointerFinishCapture(event) {
  const active = runtime.active;
  if (!active || active.source !== 'canvas' || (event.pointerId !== undefined && active.pointerId !== event.pointerId)) return;
  const result = evaluateActive();
  if (result && !result.valid) cancelEditorDrag(active, `${result.reason} · previous pose restored`);
  runtime.active = null;
  clearFeedback(1000);
}

function installGizmoListeners(gizmo) {
  if (!gizmo || runtime.gizmosInstalled.has(gizmo)) return;
  runtime.gizmosInstalled.add(gizmo);
  gizmo.addEventListener('dragging-changed', event => {
    if (event.value) beginActive('gizmo');
    else {
      if (runtime.active?.source === 'gizmo') evaluateActive({ blockGizmo: true });
      runtime.active = null;
      clearFeedback(1000);
    }
  });
  gizmo.addEventListener('objectChange', () => {
    if (runtime.replayingObjectChange || runtime.active?.source !== 'gizmo') return;
    evaluateActive({ blockGizmo: true });
  });
}

function validateNumericEdit() {
  const before = runtime.numericBefore;
  runtime.numericBefore = null;
  if (!before?.root || !snapshotChanged(before.root, before.saved)) return;
  const result = validateRoot(before.root, before.itemId, { feedback: true });
  if (!result.valid) {
    editorApi()?.undo?.();
    showFeedback({ ...result, reason: `${result.reason} · numeric edit rejected` });
  }
  clearFeedback(1200);
}

function validatePatchBeforeExport(event) {
  const edits = editorApi()?.getEditPatch?.()?.edits || [];
  for (const edit of edits) {
    const root = rootForItemId(edit.item_id);
    if (!root) continue;
    const result = validateRoot(root, edit.item_id, { feedback: false });
    if (result.valid) continue;
    event.preventDefault();
    event.stopImmediatePropagation();
    editorApi()?.selectItem?.(edit.item_id);
    showFeedback({ ...result, reason: `${result.reason} · export blocked` });
    clearFeedback(1800);
    return false;
  }
  return true;
}

function installCanvasListeners() {
  if (runtime.listenersInstalled) return;
  const canvas = document.getElementById('scene-canvas');
  if (!canvas) return;
  runtime.canvas = canvas;
  runtime.listenersInstalled = true;
  canvas.addEventListener('pointerdown', onCanvasPointerDown);
  canvas.addEventListener('pointermove', onCanvasPointerMove);
  canvas.addEventListener('pointerup', onCanvasPointerFinishCapture, true);
  canvas.addEventListener('pointercancel', onCanvasPointerFinishCapture, true);
  window.addEventListener('keydown', event => {
    if (event.key !== 'Escape' || !runtime.active) return;
    const active = runtime.active;
    cancelEditorDrag(active, 'Placement cancelled · previous pose restored');
    runtime.active = null;
    clearFeedback(900);
  });
  document.addEventListener('beforeinput', event => {
    if (!event.target?.matches?.('[data-transform-field]')) return;
    const state = editorState();
    const root = rootForItemId(state?.selectedItemId);
    if (root) runtime.numericBefore = { root, itemId: state.selectedItemId, saved: snapshot(root) };
  }, true);
  document.addEventListener('input', event => {
    if (event.target?.matches?.('[data-transform-field]')) queueMicrotask(validateNumericEdit);
  });
  document.getElementById('export-edit-patch')?.addEventListener('click', validatePatchBeforeExport, true);
}

function findGizmo(scene) {
  let found = null;
  scene?.traverse?.(node => {
    if (!found && node instanceof TransformControls) found = node;
  });
  return found;
}

function install() {
  const prototype = THREE.WebGLRenderer?.prototype;
  if (!prototype || prototype[PATCH_FLAG]) return;
  const originalRender = prototype.render;
  prototype.render = function renderWithCollisionPlacementValidation(scene, camera) {
    if (runtime.scene && runtime.scene !== scene) {
      runtime.active = null;
      clearFeedback();
    }
    runtime.scene = scene;
    runtime.gizmo = runtime.gizmo?.parent ? runtime.gizmo : findGizmo(scene);
    installGizmoListeners(runtime.gizmo);
    installCanvasListeners();
    return originalRender.call(this, scene, camera);
  };
  prototype[PATCH_FLAG] = true;

  window.__WORKCELL_COLLISION_PLACEMENT_V1__ = Object.freeze({
    enabled: true,
    version: 1,
    clearance_m: CLEARANCE_M,
    getState: () => ({ active: Boolean(runtime.active), lastResult: runtime.lastResult }),
    validateItem: id => {
      const root = rootForItemId(String(id || ''));
      return root ? validateRoot(root, String(id || ''), { feedback: false }) : null;
    },
    clear: () => {
      runtime.active = null;
      clearFeedback();
    },
  });
}

install();
