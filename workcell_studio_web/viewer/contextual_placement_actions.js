import * as THREE from 'three';

const PATCH_FLAG = Symbol.for('workcell-studio.contextual-placement-actions.v1');
const SUPPORT_KINDS = new Set(['workbench_body', 'table_surface', 'tabletop']);
const EDGE_MARGIN_M = 0.01;
const EPSILON = 1e-6;

const ACTIONS = Object.freeze([
  ['place-nearest', 'Place on nearest surface'],
  ['center', 'Centre on surface'],
  ['align-left', 'Align left'],
  ['align-right', 'Align right'],
  ['align-front', 'Align front'],
  ['align-back', 'Align back'],
  ['move-pick', 'Move to pick area'],
  ['move-place', 'Move to place area'],
]);

const runtime = {
  scene: null,
  observer: null,
  installed: false,
  frame: 0,
  message: '',
  severity: 'neutral',
  messageTimer: null,
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

function itemLabel(item, fallback = 'item') {
  return item?.label || item?.display_name || item?.object_name || item?.name || item?.id || fallback;
}

function supportKind(item) {
  return String(item?.support_surface_kind || item?.supportSurfaceKind || '').trim().toLowerCase();
}

function identity(item, node = null) {
  return [
    item?.id,
    item?.label,
    item?.display_name,
    item?.object_name,
    item?.name,
    item?.role,
    item?.type,
    item?.category,
    item?.zone_kind,
    item?.zone_type,
    item?.task_zone,
    item?.task_zone_name,
    item?.source_layer,
    node?.name,
  ].map(value => String(value || '').toLowerCase().replace(/[_-]+/g, ' ')).join(' ');
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

function finiteBounds(root) {
  if (!root) return null;
  root.updateWorldMatrix?.(true, true);
  const box = new THREE.Box3().setFromObject(root);
  const values = [box.min.x, box.min.y, box.min.z, box.max.x, box.max.y, box.max.z];
  return values.every(Number.isFinite) && !box.isEmpty() ? box : null;
}

function topSurfaceZ(item, box) {
  const explicit = Number(item?.top_surface_z_m ?? item?.topSurfaceZM);
  return Number.isFinite(explicit) ? explicit : box.max.z;
}

function uniqueSceneEntries(predicate) {
  if (!runtime.scene) return [];
  const entries = [];
  const seen = new Set();
  runtime.scene.traverse(node => {
    const item = itemOf(node);
    if (!item || !predicate(item, node)) return;
    const id = itemId(item);
    const root = id ? rootForItemId(id) : node;
    const key = root?.uuid || id;
    if (!root || !key || seen.has(key)) return;
    seen.add(key);
    const box = finiteBounds(root);
    if (box) entries.push({ root, item, box });
  });
  return entries;
}

function supportEntries() {
  return uniqueSceneEntries(item => SUPPORT_KINDS.has(supportKind(item))).map(entry => ({
    ...entry,
    top: topSurfaceZ(entry.item, entry.box),
  }));
}

function zoneKind(item, node) {
  const text = identity(item, node);
  const zoneLike = /\b(zone|area|region|task)\b/.test(text) || item?.task_zone || item?.zone_kind || item?.zone_type;
  if (!zoneLike) return '';
  if (/\bpick(?:ing)?\b/.test(text)) return 'pick';
  if (/\bplace(?:ment)?\b/.test(text)) return 'place';
  return '';
}

function zoneEntries(kind) {
  return uniqueSceneEntries((item, node) => zoneKind(item, node) === kind);
}

function boxCenter(box) {
  return box.getCenter(new THREE.Vector3());
}

function outsideDistance(value, min, max) {
  if (value < min) return min - value;
  if (value > max) return value - max;
  return 0;
}

function supportDistanceToPoint(support, point) {
  return Math.hypot(
    outsideDistance(point.x, support.box.min.x, support.box.max.x),
    outsideDistance(point.y, support.box.min.y, support.box.max.y),
  );
}

function nearestSupport(point) {
  let best = null;
  for (const support of supportEntries()) {
    const distance = supportDistanceToPoint(support, point);
    if (!best || distance < best.distance) best = { ...support, distance };
  }
  return best;
}

function containingOrNearestSupport(point) {
  const supports = supportEntries();
  const containing = supports.find(support => (
    point.x >= support.box.min.x - EPSILON && point.x <= support.box.max.x + EPSILON &&
    point.y >= support.box.min.y - EPSILON && point.y <= support.box.max.y + EPSILON
  ));
  return containing ? { ...containing, distance: 0 } : nearestSupport(point);
}

function availableInnerBounds(support, objectBox) {
  const size = objectBox.getSize(new THREE.Vector3());
  const inner = {
    minX: support.box.min.x + EDGE_MARGIN_M,
    maxX: support.box.max.x - EDGE_MARGIN_M,
    minY: support.box.min.y + EDGE_MARGIN_M,
    maxY: support.box.max.y - EDGE_MARGIN_M,
    size,
  };
  if (size.x > inner.maxX - inner.minX + EPSILON || size.y > inner.maxY - inner.minY + EPSILON) return null;
  return inner;
}

function clampCenterToSupport(center, inner) {
  return new THREE.Vector3(
    THREE.MathUtils.clamp(center.x, inner.minX + inner.size.x / 2, inner.maxX - inner.size.x / 2),
    THREE.MathUtils.clamp(center.y, inner.minY + inner.size.y / 2, inner.maxY - inner.size.y / 2),
    center.z,
  );
}

function targetDelta(action, objectBox, support, zone = null) {
  const inner = availableInnerBounds(support, objectBox);
  if (!inner) return { error: `Selected item is too large for ${itemLabel(support.item, 'the surface')}` };

  const current = boxCenter(objectBox);
  let target = current.clone();
  if (action === 'center') {
    target.x = (inner.minX + inner.maxX) / 2;
    target.y = (inner.minY + inner.maxY) / 2;
  } else if (action === 'align-left') {
    target.x = inner.minX + inner.size.x / 2;
    target.y = clampCenterToSupport(target, inner).y;
  } else if (action === 'align-right') {
    target.x = inner.maxX - inner.size.x / 2;
    target.y = clampCenterToSupport(target, inner).y;
  } else if (action === 'align-front') {
    target.y = inner.minY + inner.size.y / 2;
    target.x = clampCenterToSupport(target, inner).x;
  } else if (action === 'align-back') {
    target.y = inner.maxY - inner.size.y / 2;
    target.x = clampCenterToSupport(target, inner).x;
  } else if ((action === 'move-pick' || action === 'move-place') && zone) {
    target.copy(clampCenterToSupport(boxCenter(zone.box), inner));
  } else {
    target.copy(clampCenterToSupport(target, inner));
  }

  return {
    delta: new THREE.Vector3(
      target.x - current.x,
      target.y - current.y,
      support.top - objectBox.min.z,
    ),
  };
}

function localPositionAfterWorldDelta(root, delta) {
  const world = root.getWorldPosition(new THREE.Vector3()).add(delta);
  if (root.parent) root.parent.worldToLocal(world);
  return world;
}

function transformInputs() {
  const inspector = document.getElementById('inspector');
  if (!inspector) return null;
  const input = name => inspector.querySelector(`[data-transform-field="${name}"]`);
  const fields = { x: input('x'), y: input('y'), z: input('z') };
  return fields.x && fields.y && fields.z ? fields : null;
}

function commitLocalPosition(position) {
  const fields = transformInputs();
  if (!fields) return false;
  fields.x.value = Number(position.x).toFixed(6);
  fields.y.value = Number(position.y).toFixed(6);
  fields.z.value = Number(position.z).toFixed(6);
  fields.x.dispatchEvent(new Event('input', { bubbles: true }));
  return true;
}

function collisionResult(id) {
  return window.__WORKCELL_COLLISION_PLACEMENT_V1__?.validateItem?.(id) || null;
}

function publish(action, state, extra = {}) {
  const detail = { action, item_id: state?.selectedItemId || '', ...extra };
  window.dispatchEvent?.(new CustomEvent('workcell:contextual-placement-action', { detail }));
  window.parent?.postMessage?.({ type: 'workcell_contextual_placement_action', ...detail }, '*');
}

function showMessage(message, severity = 'neutral', timeoutMs = 1800) {
  runtime.message = message;
  runtime.severity = severity;
  window.clearTimeout(runtime.messageTimer);
  refreshActions();
  if (timeoutMs > 0) {
    runtime.messageTimer = window.setTimeout(() => {
      runtime.message = '';
      runtime.severity = 'neutral';
      refreshActions();
    }, timeoutMs);
  }
}

function runAction(action) {
  const state = editorState();
  if (!state?.selectedItemId || state.selectedEditable !== true) return false;
  const root = rootForItemId(state.selectedItemId);
  const objectBox = finiteBounds(root);
  if (!root || !objectBox) {
    showMessage('Selected item has no usable rendered bounds.', 'error');
    return false;
  }

  let zone = null;
  let support = null;
  if (action === 'move-pick' || action === 'move-place') {
    const kind = action === 'move-pick' ? 'pick' : 'place';
    zone = zoneEntries(kind)[0] || null;
    if (!zone) {
      showMessage(`No ${kind} area is defined in this scene.`, 'error');
      return false;
    }
    support = containingOrNearestSupport(boxCenter(zone.box));
  } else {
    support = containingOrNearestSupport(boxCenter(objectBox));
  }
  if (!support) {
    showMessage('No table or workbench is available.', 'error');
    return false;
  }

  const target = targetDelta(action, objectBox, support, zone);
  if (target.error) {
    showMessage(target.error, 'error');
    return false;
  }

  const local = localPositionAfterWorldDelta(root, target.delta);
  if (!commitLocalPosition(local)) {
    showMessage('Placement controls are unavailable for this item.', 'error');
    return false;
  }

  const collision = collisionResult(state.selectedItemId);
  if (collision && !collision.valid) {
    editorApi()?.undo?.();
    showMessage(`${collision.reason || 'Placement overlaps another object'} · action reverted`, 'error', 2400);
    publish(action, state, { valid: false, reverted: true, reason: collision.reason || 'collision' });
    return false;
  }

  const label = ACTIONS.find(([name]) => name === action)?.[1] || action;
  const severity = collision?.severity === 'warning' ? 'warning' : 'success';
  showMessage(`${label} · ${itemLabel(support.item, 'surface')}`, severity);
  publish(action, state, {
    valid: true,
    support_surface_id: itemId(support.item),
    zone_id: itemId(zone?.item),
    collision_severity: collision?.severity || 'valid',
  });
  return true;
}

function actionAvailability(state) {
  const editable = Boolean(state?.selectedItemId && state.selectedEditable === true);
  if (!editable) return Object.fromEntries(ACTIONS.map(([name]) => [name, false]));
  const root = rootForItemId(state.selectedItemId);
  const bounds = finiteBounds(root);
  const hasSurface = Boolean(bounds && containingOrNearestSupport(boxCenter(bounds)));
  return {
    'place-nearest': hasSurface,
    center: hasSurface,
    'align-left': hasSurface,
    'align-right': hasSurface,
    'align-front': hasSurface,
    'align-back': hasSurface,
    'move-pick': hasSurface && zoneEntries('pick').length > 0,
    'move-place': hasSurface && zoneEntries('place').length > 0,
  };
}

function sectionSignature(state, availability) {
  return JSON.stringify({
    item: state.selectedItemId,
    availability,
    message: runtime.message,
    severity: runtime.severity,
  });
}

function createActionsSection(state, availability = actionAvailability(state)) {
  const section = document.createElement('section');
  section.className = 'contextual-placement-actions';
  section.dataset.itemId = state.selectedItemId;
  section.dataset.signature = sectionSignature(state, availability);

  const title = document.createElement('h3');
  title.textContent = 'Quick placement';
  const note = document.createElement('p');
  note.className = 'contextual-placement-note';
  note.textContent = 'Use the current surface, edge or task area. Collision validation still applies.';
  const grid = document.createElement('div');
  grid.className = 'contextual-placement-grid';

  for (const [name, label] of ACTIONS) {
    const button = document.createElement('button');
    button.type = 'button';
    button.dataset.placementAction = name;
    button.textContent = label;
    button.disabled = !availability[name];
    if ((name === 'move-pick' || name === 'move-place') && !availability[name]) {
      button.title = `${name === 'move-pick' ? 'Pick' : 'Place'} area is not available in this scene`;
    }
    button.addEventListener('click', () => runAction(name));
    grid.appendChild(button);
  }

  const status = document.createElement('p');
  status.className = `contextual-placement-message ${runtime.severity}`;
  status.setAttribute('role', 'status');
  status.setAttribute('aria-live', 'polite');
  status.hidden = !runtime.message;
  status.textContent = runtime.message;
  section.append(title, note, grid, status);
  return section;
}

function refreshActions() {
  const inspector = document.getElementById('inspector');
  if (!inspector) return;
  const state = editorState();
  const existing = inspector.querySelector(':scope > .contextual-placement-actions');
  const editable = Boolean(state?.selectedItemId && state.selectedEditable === true && rootForItemId(state.selectedItemId));
  if (!editable) {
    existing?.remove();
    return;
  }

  const availability = actionAvailability(state);
  const signature = sectionSignature(state, availability);
  if (existing?.dataset.signature === signature) return;
  const replacement = createActionsSection(state, availability);
  if (existing) existing.replaceWith(replacement);
  else {
    const editor = inspector.querySelector(':scope > .transform-editor');
    inspector.insertBefore(replacement, editor || null);
  }
}

function installObserver() {
  if (runtime.observer) return;
  const inspector = document.getElementById('inspector');
  if (!inspector) return;
  runtime.observer = new MutationObserver(() => queueMicrotask(refreshActions));
  runtime.observer.observe(inspector, { childList: true, subtree: true });
}

function install() {
  if (runtime.installed) return;
  runtime.installed = true;
  installObserver();

  const prototype = THREE.WebGLRenderer?.prototype;
  if (!prototype || prototype[PATCH_FLAG]) return;
  const originalRender = prototype.render;
  prototype.render = function renderWithContextualPlacementActions(scene, camera) {
    runtime.scene = scene;
    runtime.frame = (runtime.frame + 1) % 30;
    if (runtime.frame === 1) refreshActions();
    return originalRender.call(this, scene, camera);
  };
  prototype[PATCH_FLAG] = true;

  window.__WORKCELL_CONTEXTUAL_PLACEMENT_ACTIONS_V1__ = Object.freeze({
    enabled: true,
    version: 1,
    structural_actions: 'deferred_until_persistent_add_remove_patch_support',
    availableActions: () => actionAvailability(editorState()),
    run: action => runAction(String(action || '')),
    refresh: refreshActions,
  });
}

install();
