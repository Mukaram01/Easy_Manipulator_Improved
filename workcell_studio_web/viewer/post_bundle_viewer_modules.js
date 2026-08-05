import './canonical_helper_pick_proxies.js';

const VERSION = 2;

const MODULES = Object.freeze([
  Object.freeze({
    id: 'rviz-light-baseline',
    file: 'rviz_light_baseline.js',
    global: '__WORKCELL_RVIZ_LIGHT_BASELINE__',
    purpose: 'rendering baseline',
  }),
  Object.freeze({
    id: 'support-surface-placement',
    file: 'support_surface_placement.js',
    global: '__WORKCELL_SUPPORT_PLACEMENT_V1__',
    purpose: 'support-aware dragging',
  }),
  Object.freeze({
    id: 'task-gizmo',
    file: 'workcell_task_gizmo.js',
    global: '__WORKCELL_TASK_GIZMO_V1__',
    purpose: 'task-focused transform controls',
  }),
  Object.freeze({
    id: 'collision-placement-validation',
    file: 'collision_placement_validation.js',
    global: '__WORKCELL_COLLISION_PLACEMENT_V1__',
    purpose: 'layout collision feedback',
  }),
  Object.freeze({
    id: 'simple-product-ui',
    file: 'simple_product_ui.js',
    global: '__WORKCELL_SIMPLE_PRODUCT_UI_V1__',
    purpose: 'progressive disclosure',
  }),
  Object.freeze({
    id: 'contextual-placement-actions',
    file: 'contextual_placement_actions.js',
    global: '__WORKCELL_CONTEXTUAL_PLACEMENT_ACTIONS_V1__',
    purpose: 'one-click placement actions',
  }),
  Object.freeze({
    id: 'canonical-helper-pick-proxies',
    file: 'canonical_helper_pick_proxies.js',
    global: '__WORKCELL_CANONICAL_HELPER_PICK_PROXIES_V1__',
    purpose: 'canonical owner forwarding for helper-only canvas hits',
  }),
]);

let lastSceneId = '';
let lastAnnouncement = '';

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

function moduleApi(module) {
  return window[module.global] || null;
}

function moduleStatus(module) {
  const api = moduleApi(module);
  return Object.freeze({
    id: module.id,
    file: module.file,
    purpose: module.purpose,
    global: module.global,
    ready: Boolean(api?.enabled),
    version: Number(api?.version || 0),
  });
}

function getStatus() {
  const modules = MODULES.map(moduleStatus);
  const missing = modules.filter(module => !module.ready).map(module => module.id);
  const state = editorState();
  return Object.freeze({
    enabled: true,
    version: VERSION,
    ready: missing.length === 0,
    sceneId: String(state?.sceneId || ''),
    selectedItemId: String(state?.selectedItemId || ''),
    dirty: Boolean(state?.dirty),
    modules: Object.freeze(modules),
    missing: Object.freeze(missing),
  });
}

function clearTransientFeedback() {
  window.__WORKCELL_SUPPORT_PLACEMENT_V1__?.clear?.();
  window.__WORKCELL_COLLISION_PLACEMENT_V1__?.clear?.();
}

function refreshUi() {
  window.__WORKCELL_SIMPLE_PRODUCT_UI_V1__?.refresh?.();
  window.__WORKCELL_CONTEXTUAL_PLACEMENT_ACTIONS_V1__?.refresh?.();
  return getStatus();
}

function validateSelected() {
  const id = String(editorState()?.selectedItemId || '');
  if (!id) return null;
  return window.__WORKCELL_COLLISION_PLACEMENT_V1__?.validateItem?.(id) || null;
}

function setFreeHeight(enabled) {
  window.__WORKCELL_TASK_GIZMO_V1__?.setFreeHeight?.(Boolean(enabled));
  return window.__WORKCELL_TASK_GIZMO_V1__?.getState?.() || null;
}

function runPlacementAction(action) {
  return Boolean(window.__WORKCELL_CONTEXTUAL_PLACEMENT_ACTIONS_V1__?.run?.(String(action || '')));
}

function syncSceneIdentity() {
  const sceneId = String(editorState()?.sceneId || '');
  if (lastSceneId && sceneId && sceneId !== lastSceneId) clearTransientFeedback();
  if (sceneId) lastSceneId = sceneId;
  return sceneId;
}

function announceStatus() {
  syncSceneIdentity();
  const status = getStatus();
  const signature = JSON.stringify({
    ready: status.ready,
    sceneId: status.sceneId,
    missing: status.missing,
  });
  if (signature === lastAnnouncement) return status;
  lastAnnouncement = signature;
  window.dispatchEvent?.(new CustomEvent('workcell:post-bundle-viewer-status', { detail: status }));
  window.parent?.postMessage?.({
    type: 'workcell_post_bundle_viewer_status',
    ready: status.ready,
    scene_id: status.sceneId,
    missing: [...status.missing],
  }, '*');
  return status;
}

const api = Object.freeze({
  enabled: true,
  version: VERSION,
  modules: MODULES,
  getStatus,
  getEditorState: editorState,
  clearTransientFeedback,
  refreshUi,
  validateSelected,
  setFreeHeight,
  runPlacementAction,
  syncSceneIdentity,
});

window.__WORKCELL_POST_BUNDLE_VIEWER_V1__ = api;

for (const eventName of [
  'workcell:support-placement',
  'workcell:gizmo-task-mode',
  'workcell:collision-placement',
  'workcell:contextual-placement-action',
  'workcell:editor-state',
  'workcell:scene-loaded',
]) {
  window.addEventListener(eventName, announceStatus);
}
window.addEventListener('pageshow', announceStatus);
window.addEventListener('beforeunload', clearTransientFeedback, { once: true });
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) announceStatus();
});

queueMicrotask(() => {
  refreshUi();
  announceStatus();
});
