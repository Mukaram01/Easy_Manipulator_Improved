import * as THREE from 'three';

const SUPPORT_KINDS = new Set(['workbench_body', 'table_surface', 'tabletop']);
const PATCH_FLAG = Symbol.for('workcell-studio.support-surface-placement.v1');
const EDGE_MARGIN_M = 0.01;
const MAX_ACQUIRE_DISTANCE_M = 0.15;
const VALID_COLOR = 0x2f8f5b;
const INVALID_COLOR = 0xc43434;
const EPSILON = 1e-5;

const runtime = {
  scene: null,
  camera: null,
  renderer: null,
  canvas: null,
  active: null,
  helpers: new Map(),
  statusNode: null,
  feedbackTimer: null,
  listenersInstalled: false,
  lastResult: null,
};

function editorState() {
  try {
    return window.__WORKCELL_EDITOR_API_V1__?.getState?.() || null;
  } catch (_) {
    return null;
  }
}

function itemOf(node) {
  return node?.userData?.item || null;
}

function supportKind(item) {
  return String(item?.support_surface_kind || item?.supportSurfaceKind || '').trim().toLowerCase();
}

function itemLabel(item) {
  return item?.label || item?.display_name || item?.object_name || item?.name || item?.id || 'support surface';
}

function rootForItemId(itemId) {
  if (!runtime.scene || !itemId) return null;
  let found = null;
  runtime.scene.traverse(node => {
    if (!found && String(itemOf(node)?.id || '') === String(itemId)) found = node;
  });
  if (!found) return null;
  while (
    found.parent &&
    found.parent !== runtime.scene &&
    String(itemOf(found.parent)?.id || '') === String(itemId)
  ) {
    found = found.parent;
  }
  return found;
}

function finiteBounds(object) {
  if (!object) return null;
  object.updateWorldMatrix?.(true, true);
  const box = new THREE.Box3().setFromObject(object);
  return [box.min.x, box.min.y, box.min.z, box.max.x, box.max.y, box.max.z]
    .every(Number.isFinite) && !box.isEmpty() ? box : null;
}

function topSurfaceZ(item, box) {
  const explicit = Number(item?.top_surface_z_m ?? item?.topSurfaceZM);
  return Number.isFinite(explicit) ? explicit : box.max.z;
}

function supportEntries(selectedId) {
  if (!runtime.scene) return [];
  const entries = [];
  const seen = new Set();
  runtime.scene.traverse(node => {
    const item = itemOf(node);
    if (!item || !SUPPORT_KINDS.has(supportKind(item))) return;
    if (String(item.id || '') === String(selectedId || '')) return;
    let root = node;
    while (
      root.parent &&
      root.parent !== runtime.scene &&
      String(itemOf(root.parent)?.id || '') === String(item.id || '')
    ) {
      root = root.parent;
    }
    const key = root.uuid || item.id;
    if (!key || seen.has(key)) return;
    seen.add(key);
    const box = finiteBounds(root);
    if (!box) return;
    entries.push({ root, item, box, top: topSurfaceZ(item, box) });
  });
  return entries;
}

function outsideDistance(value, min, max) {
  if (value < min) return min - value;
  if (value > max) return value - max;
  return 0;
}

function nearestSupport(object, selectedId, maxDistance = MAX_ACQUIRE_DISTANCE_M) {
  const objectBox = finiteBounds(object);
  if (!objectBox) return null;
  const center = objectBox.getCenter(new THREE.Vector3());
  let best = null;
  for (const support of supportEntries(selectedId)) {
    const dx = outsideDistance(center.x, support.box.min.x, support.box.max.x);
    const dy = outsideDistance(center.y, support.box.min.y, support.box.max.y);
    const horizontalDistance = Math.hypot(dx, dy);
    if (horizontalDistance > maxDistance) continue;
    const verticalDistance = Math.abs(objectBox.min.z - support.top);
    const score = horizontalDistance * 10 + verticalDistance;
    if (!best || score < best.score) best = { ...support, score };
  }
  return best;
}

function moveObjectInWorld(object, delta) {
  if (!object || !delta) return;
  const target = object.getWorldPosition(new THREE.Vector3()).add(delta);
  if (object.parent) object.parent.worldToLocal(target);
  object.position.copy(target);
  object.updateWorldMatrix?.(true, true);
}

function helperFor(name, box, color) {
  let helper = runtime.helpers.get(name);
  if (!helper) {
    helper = new THREE.Box3Helper(box.clone(), color);
    helper.name = `support_placement_${name}`;
    helper.renderOrder = 1000;
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
  node.id = 'support-placement-status';
  node.setAttribute('role', 'status');
  node.setAttribute('aria-live', 'polite');
  node.hidden = true;
  node.style.cssText = [
    'position:absolute',
    'left:50%',
    'bottom:0.85rem',
    'transform:translateX(-50%)',
    'z-index:5',
    'max-width:min(34rem,calc(100% - 2rem))',
    'padding:0.48rem 0.72rem',
    'border:1px solid currentColor',
    'border-radius:0.45rem',
    'box-shadow:0 0.15rem 0.55rem rgba(23,32,42,0.18)',
    'font:600 0.82rem/1.3 Inter,ui-sans-serif,system-ui,sans-serif',
    'text-align:center',
    'pointer-events:none',
  ].join(';');
  host.appendChild(node);
  runtime.statusNode = node;
  return node;
}

function showStatus(valid, message) {
  const node = ensureStatusNode();
  if (!node) return;
  node.hidden = false;
  node.textContent = message;
  node.style.color = valid ? '#14532d' : '#8b1e1e';
  node.style.background = valid ? 'rgba(236,253,245,0.96)' : 'rgba(255,238,240,0.96)';
}

function clearPlacementFeedback(delayMs = 0) {
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
  runtime.lastResult = result;
  window.dispatchEvent?.(new CustomEvent('workcell:support-placement', { detail: result }));
  window.parent?.postMessage?.({ type: 'workcell_support_placement', ...result }, '*');
}

function applySupportPlacement(active) {
  const { object, itemId } = active;
  let support = active.support;
  if (!support) support = nearestSupport(object, itemId);
  if (!support) {
    const objectBox = finiteBounds(object);
    if (objectBox) helperFor('object', objectBox, INVALID_COLOR);
    const result = {
      valid: false,
      item_id: itemId,
      support_surface_id: '',
      reason: 'No table or workbench under the object',
    };
    showStatus(false, result.reason);
    publishResult(result);
    return result;
  }

  support.box = finiteBounds(support.root) || support.box;
  support.top = topSurfaceZ(support.item, support.box);
  active.support = support;

  let objectBox = finiteBounds(object);
  if (!objectBox) return null;
  const objectSize = objectBox.getSize(new THREE.Vector3());
  const innerMinX = support.box.min.x + EDGE_MARGIN_M;
  const innerMaxX = support.box.max.x - EDGE_MARGIN_M;
  const innerMinY = support.box.min.y + EDGE_MARGIN_M;
  const innerMaxY = support.box.max.y - EDGE_MARGIN_M;
  const availableX = innerMaxX - innerMinX;
  const availableY = innerMaxY - innerMinY;

  if (objectSize.x > availableX + EPSILON || objectSize.y > availableY + EPSILON) {
    helperFor('support', support.box, INVALID_COLOR);
    helperFor('object', objectBox, INVALID_COLOR);
    const result = {
      valid: false,
      item_id: itemId,
      support_surface_id: support.item.id || '',
      reason: `${itemLabel(active.item)} is larger than ${itemLabel(support.item)}`,
    };
    showStatus(false, result.reason);
    publishResult(result);
    return result;
  }

  let correctionX = 0;
  let correctionY = 0;
  if (objectBox.min.x < innerMinX) correctionX = innerMinX - objectBox.min.x;
  else if (objectBox.max.x > innerMaxX) correctionX = innerMaxX - objectBox.max.x;
  if (objectBox.min.y < innerMinY) correctionY = innerMinY - objectBox.min.y;
  else if (objectBox.max.y > innerMaxY) correctionY = innerMaxY - objectBox.max.y;
  if (correctionX || correctionY) {
    moveObjectInWorld(object, new THREE.Vector3(correctionX, correctionY, 0));
    objectBox = finiteBounds(object);
  }

  const correctionZ = support.top - objectBox.min.z;
  if (Math.abs(correctionZ) > EPSILON) {
    moveObjectInWorld(object, new THREE.Vector3(0, 0, correctionZ));
    objectBox = finiteBounds(object);
  }

  const valid = Boolean(
    objectBox &&
    objectBox.min.x >= innerMinX - EPSILON &&
    objectBox.max.x <= innerMaxX + EPSILON &&
    objectBox.min.y >= innerMinY - EPSILON &&
    objectBox.max.y <= innerMaxY + EPSILON &&
    Math.abs(objectBox.min.z - support.top) <= EPSILON * 5
  );
  const color = valid ? VALID_COLOR : INVALID_COLOR;
  helperFor('support', support.box, color);
  helperFor('object', objectBox, color);

  object.userData.preview_support_surface_id = support.item.id || '';
  const result = {
    valid,
    item_id: itemId,
    support_surface_id: support.item.id || '',
    support_surface_label: itemLabel(support.item),
    top_surface_z_m: support.top,
    clamped_to_footprint: Boolean(correctionX || correctionY),
    reason: valid ? `Valid placement on ${itemLabel(support.item)}` : 'Placement remains outside the support surface',
  };
  showStatus(valid, result.reason);
  publishResult(result);
  return result;
}

function positionChanged(object, previous) {
  return previous && object.position.distanceToSquared(previous) > 1e-12;
}

function onPointerDown(event) {
  const state = editorState();
  if (!state || state.mode !== 'move' || state.selectedEditable !== true || !state.selectedItemId) return;
  const object = rootForItemId(state.selectedItemId);
  if (!object) return;
  clearPlacementFeedback();
  runtime.active = {
    pointerId: event.pointerId,
    itemId: state.selectedItemId,
    item: itemOf(object),
    object,
    lastPosition: object.position.clone(),
    support: nearestSupport(object, state.selectedItemId, 0.04),
    moved: false,
  };
}

function onPointerMove(event) {
  const active = runtime.active;
  if (!active || active.pointerId !== event.pointerId) return;
  const state = editorState();
  if (!state || state.mode !== 'move' || state.selectedItemId !== active.itemId) return;
  if (!positionChanged(active.object, active.lastPosition)) return;
  active.moved = true;
  applySupportPlacement(active);
  active.lastPosition.copy(active.object.position);
}

function finishPlacement(event) {
  const active = runtime.active;
  if (!active || (event?.pointerId !== undefined && active.pointerId !== event.pointerId)) return;
  runtime.active = null;
  if (active.moved) clearPlacementFeedback(900);
  else clearPlacementFeedback();
}

function installCanvasListeners() {
  if (runtime.listenersInstalled || !runtime.scene) return;
  const canvas = document.getElementById('scene-canvas');
  if (!canvas) return;
  runtime.canvas = canvas;
  runtime.listenersInstalled = true;
  canvas.addEventListener('pointerdown', onPointerDown);
  canvas.addEventListener('pointermove', onPointerMove);
  canvas.addEventListener('pointerup', finishPlacement);
  canvas.addEventListener('pointercancel', finishPlacement);
  window.addEventListener('keydown', event => {
    if (event.key === 'Escape') {
      runtime.active = null;
      clearPlacementFeedback();
    }
  });
}

function install() {
  const prototype = THREE.WebGLRenderer?.prototype;
  if (!prototype || prototype[PATCH_FLAG]) return;
  const originalRender = prototype.render;
  prototype.render = function renderWithSupportPlacement(scene, camera) {
    if (runtime.scene && runtime.scene !== scene) {
      runtime.active = null;
      clearPlacementFeedback();
    }
    runtime.scene = scene;
    runtime.camera = camera;
    runtime.renderer = this;
    installCanvasListeners();
    return originalRender.call(this, scene, camera);
  };
  prototype[PATCH_FLAG] = true;

  window.__WORKCELL_SUPPORT_PLACEMENT_V1__ = Object.freeze({
    enabled: true,
    version: 1,
    scope: 'Preview-only placement helper; final poses remain committed by the existing edit-patch workflow.',
    getState: () => ({ active: Boolean(runtime.active), lastResult: runtime.lastResult }),
    clear: () => {
      runtime.active = null;
      clearPlacementFeedback();
    },
  });
}

install();
