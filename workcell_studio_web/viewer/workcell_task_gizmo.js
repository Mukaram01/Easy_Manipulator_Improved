import { THREE, TransformControls } from './dist/viewer.bundle.js';

const PATCH_FLAG = Symbol.for('workcell-studio.task-gizmo.v1');
const FINE_TRANSLATION_M = 0.001;
const FINE_ROTATION_DEG = 1;

const runtime = {
  gizmo: null,
  freeHeight: false,
  fineMode: false,
  selectedItemId: '',
  toggle: null,
  labelText: null,
  listenersInstalled: false,
};

function editorState() {
  try {
    return window.__WORKCELL_EDITOR_API_V1__?.getState?.() || null;
  } catch (_) {
    return null;
  }
}

function isTypingTarget(target) {
  const tag = String(target?.tagName || '').toLowerCase();
  return tag === 'input' || tag === 'textarea' || tag === 'select' || target?.isContentEditable === true;
}

function numericInputValue(id, fallback) {
  const value = Number(document.getElementById(id)?.value);
  return Number.isFinite(value) && value > 0 ? value : fallback;
}

function snapEnabled() {
  return Boolean(document.getElementById('snap-toggle')?.checked);
}

function findGizmo(scene) {
  let found = null;
  scene?.traverse?.(node => {
    if (!found && node instanceof TransformControls) found = node;
  });
  return found;
}

function clearSupportPlacement() {
  window.__WORKCELL_SUPPORT_PLACEMENT_V1__?.clear?.();
}

function publishMode() {
  const detail = {
    free_height: runtime.freeHeight,
    fine_mode: runtime.fineMode,
    move_axes: runtime.freeHeight ? ['x', 'y', 'z'] : ['x', 'y'],
    rotation_axes: ['z'],
  };
  window.dispatchEvent?.(new CustomEvent('workcell:gizmo-task-mode', { detail }));
  window.parent?.postMessage?.({ type: 'workcell_gizmo_task_mode', ...detail }, '*');
}

function updateLabel() {
  if (!runtime.toggle || !runtime.labelText) return;
  runtime.toggle.checked = runtime.freeHeight;
  runtime.labelText.textContent = runtime.freeHeight ? 'Free height (XYZ)' : 'Surface move (XY)';
  runtime.toggle.parentElement.title = runtime.freeHeight
    ? 'Z translation is enabled and table-surface snapping is paused. Turn this off to return to safe XY placement.'
    : 'Default workcell placement: move in XY while support-surface placement controls Z. Hold Shift for 1 mm fine snap.';
}

function setFreeHeight(enabled, announce = true) {
  runtime.freeHeight = Boolean(enabled);
  clearSupportPlacement();
  updateLabel();
  applyTaskGizmo();
  if (announce) publishMode();
}

function ensureToolbarControl() {
  if (runtime.toggle?.isConnected) return;
  const toolbar = document.querySelector('.toolbar');
  if (!toolbar) return;

  const label = document.createElement('label');
  label.className = 'toolbar-toggle';
  label.title = 'Default workcell placement: move in XY while support-surface placement controls Z. Hold Shift for 1 mm fine snap.';

  const input = document.createElement('input');
  input.id = 'free-height-toggle';
  input.type = 'checkbox';
  input.disabled = true;
  input.addEventListener('change', () => setFreeHeight(input.checked));

  const text = document.createElement('span');
  text.id = 'workcell-gizmo-mode-label';
  text.textContent = 'Surface move (XY)';

  label.append(input, text);
  toolbar.appendChild(label);
  runtime.toggle = input;
  runtime.labelText = text;
}

function snapDiffers(current, desired) {
  if (current === desired) return false;
  if (current == null || desired == null) return true;
  return Math.abs(Number(current) - Number(desired)) > 1e-12;
}

function applySnap(gizmo, mode) {
  const enabled = snapEnabled();
  const translation = enabled
    ? (runtime.fineMode ? FINE_TRANSLATION_M : numericInputValue('translation-snap', 0.01))
    : null;
  const rotation = enabled
    ? THREE.MathUtils.degToRad(runtime.fineMode ? FINE_ROTATION_DEG : numericInputValue('rotation-snap', 5))
    : null;
  const desiredTranslation = mode === 'move' ? translation : (enabled ? numericInputValue('translation-snap', 0.01) : null);

  if (snapDiffers(gizmo.translationSnap, desiredTranslation)) gizmo.setTranslationSnap(desiredTranslation);
  if (snapDiffers(gizmo.rotationSnap, rotation)) gizmo.setRotationSnap(rotation);
}

function configureAxes(gizmo, mode, showX, showY, showZ) {
  if (gizmo.mode !== mode) gizmo.setMode(mode);
  if (gizmo.space !== 'world') gizmo.setSpace('world');
  if (gizmo.showX !== showX) gizmo.showX = showX;
  if (gizmo.showY !== showY) gizmo.showY = showY;
  if (gizmo.showZ !== showZ) gizmo.showZ = showZ;
}

function applyTaskGizmo() {
  const state = editorState();
  const gizmo = runtime.gizmo;
  if (!state || !gizmo) return;

  const editableMove = state.mode === 'move' && state.selectedEditable === true && Boolean(state.selectedItemId);
  if (runtime.toggle) runtime.toggle.disabled = !editableMove;

  if (runtime.selectedItemId && runtime.selectedItemId !== state.selectedItemId && runtime.freeHeight) {
    runtime.freeHeight = false;
    clearSupportPlacement();
    updateLabel();
  }
  runtime.selectedItemId = state.selectedItemId || '';

  if (state.mode !== 'move' && runtime.freeHeight) {
    runtime.freeHeight = false;
    clearSupportPlacement();
    updateLabel();
  }

  if (state.mode === 'move') {
    configureAxes(gizmo, 'translate', true, true, runtime.freeHeight);
    applySnap(gizmo, 'move');
  } else if (state.mode === 'rotate') {
    configureAxes(gizmo, 'rotate', false, false, true);
    applySnap(gizmo, 'rotate');
  }
}

function setFineMode(enabled) {
  const next = Boolean(enabled);
  if (runtime.fineMode === next) return;
  runtime.fineMode = next;
  applyTaskGizmo();
  publishMode();
}

function installInputListeners() {
  if (runtime.listenersInstalled) return;
  runtime.listenersInstalled = true;

  window.addEventListener('keydown', event => {
    if (event.key === 'Shift' && !isTypingTarget(event.target)) setFineMode(true);
    if (event.key === 'Escape') setFineMode(false);
  });
  window.addEventListener('keyup', event => {
    if (event.key === 'Shift') setFineMode(false);
  });
  window.addEventListener('blur', () => setFineMode(false));

  const canvas = document.getElementById('scene-canvas');
  canvas?.addEventListener('pointerdown', () => {
    if (runtime.freeHeight) queueMicrotask(clearSupportPlacement);
  });
}

function install() {
  const prototype = THREE.WebGLRenderer?.prototype;
  if (!prototype || prototype[PATCH_FLAG]) return;
  const originalRender = prototype.render;

  prototype.render = function renderWithTaskGizmo(scene, camera) {
    runtime.gizmo = runtime.gizmo && runtime.gizmo.parent ? runtime.gizmo : findGizmo(scene);
    ensureToolbarControl();
    applyTaskGizmo();
    const result = originalRender.call(this, scene, camera);
    installInputListeners();
    return result;
  };
  prototype[PATCH_FLAG] = true;

  window.__WORKCELL_TASK_GIZMO_V1__ = Object.freeze({
    enabled: true,
    version: 1,
    fine_translation_m: FINE_TRANSLATION_M,
    fine_rotation_deg: FINE_ROTATION_DEG,
    getState: () => ({
      freeHeight: runtime.freeHeight,
      fineMode: runtime.fineMode,
      selectedItemId: runtime.selectedItemId,
    }),
    setFreeHeight: enabled => setFreeHeight(enabled),
  });
}

install();
