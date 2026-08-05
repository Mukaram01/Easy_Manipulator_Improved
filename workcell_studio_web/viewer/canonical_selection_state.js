const VERSION = 1;
const INSTALL_FLAG = '__WORKCELL_CANONICAL_SELECTION_STATE_INSTALLED__';
const STORAGE_PREFIX = 'workcell_studio.canonical_selection.';
const MAX_INSTALL_ATTEMPTS = 200;
const INSTALL_RETRY_MS = 50;
const MAX_RESTORE_ATTEMPTS = 80;
const RESTORE_RETRY_MS = 50;

function stringValue(value) {
  return typeof value === 'string' ? value.trim() : '';
}

function clone(value) {
  if (value === undefined) return undefined;
  return JSON.parse(JSON.stringify(value));
}

function finiteNumber(value) {
  return typeof value === 'number' && Number.isFinite(value);
}

export function isFiniteCanonicalTransform(transform) {
  return Boolean(transform) && [
    transform?.pose?.xyz?.x,
    transform?.pose?.xyz?.y,
    transform?.pose?.xyz?.z,
    transform?.pose?.rpy?.x,
    transform?.pose?.rpy?.y,
    transform?.pose?.rpy?.z,
    transform?.scale?.x,
    transform?.scale?.y,
    transform?.scale?.z,
  ].every(finiteNumber);
}

function canonicalOwnerIdFromState(state) {
  return stringValue(
    state?.editOwnerItemId ||
    state?.canonicalSelectedOwnerId ||
    state?.uiSelectionItemId ||
    state?.selectedItemId
  );
}

function patchEditForOwner(patch, ownerId) {
  const edits = Array.isArray(patch?.edits) ? patch.edits : [];
  return edits.find(edit => stringValue(edit?.item_id) === ownerId) || null;
}

function transformFromPatch(patch, ownerId) {
  const edit = patchEditForOwner(patch, ownerId);
  return isFiniteCanonicalTransform(edit?.new_transform) ? clone(edit.new_transform) : null;
}

function transformFromInspector(documentRef) {
  const inspector = documentRef?.getElementById?.('inspector');
  if (!inspector?.querySelector) return null;
  const read = field => Number(inspector.querySelector(`[data-transform-field="${field}"]`)?.value);
  const transform = {
    pose: {
      xyz: { x: read('x'), y: read('y'), z: read('z') },
      rpy: { x: read('roll'), y: read('pitch'), z: read('yaw') },
    },
    scale: { x: read('scale_x'), y: read('scale_y'), z: read('scale_z') },
  };
  return isFiniteCanonicalTransform(transform) ? transform : null;
}

function writeInspectorSummary(inspector, label, value) {
  for (const key of inspector?.querySelectorAll?.('dt') || []) {
    if (stringValue(key.textContent).toLowerCase() !== label) continue;
    if (key.nextElementSibling) key.nextElementSibling.textContent = value;
  }
}

function writeInspectorTransform(documentRef, transform) {
  if (!isFiniteCanonicalTransform(transform)) return false;
  const inspector = documentRef?.getElementById?.('inspector');
  if (!inspector?.querySelector) return false;
  const values = {
    x: transform.pose.xyz.x,
    y: transform.pose.xyz.y,
    z: transform.pose.xyz.z,
    roll: transform.pose.rpy.x,
    pitch: transform.pose.rpy.y,
    yaw: transform.pose.rpy.z,
    scale_x: transform.scale.x,
    scale_y: transform.scale.y,
    scale_z: transform.scale.z,
  };
  for (const [field, value] of Object.entries(values)) {
    const input = inspector.querySelector(`[data-transform-field="${field}"]`);
    if (input) input.value = Number(value).toFixed(6);
  }
  writeInspectorSummary(inspector, 'pose xyz', [transform.pose.xyz.x, transform.pose.xyz.y, transform.pose.xyz.z].map(value => Number(value).toFixed(3)).join(', '));
  writeInspectorSummary(inspector, 'pose rpy', [transform.pose.rpy.x, transform.pose.rpy.y, transform.pose.rpy.z].map(value => Number(value).toFixed(3)).join(', '));
  writeInspectorSummary(inspector, 'scale', JSON.stringify([transform.scale.x, transform.scale.y, transform.scale.z]));
  return true;
}

function storageKey(sceneId) {
  return `${STORAGE_PREFIX}${sceneId}`;
}

function readStoredSelection(storage, sceneId) {
  if (!storage || !sceneId) return null;
  try {
    const parsed = JSON.parse(storage.getItem(storageKey(sceneId)) || 'null');
    if (!parsed || stringValue(parsed.sceneId) !== sceneId || !stringValue(parsed.ownerId)) return null;
    return {
      sceneId,
      ownerId: stringValue(parsed.ownerId),
      editable: Boolean(parsed.editable),
      transform: isFiniteCanonicalTransform(parsed.transform) ? clone(parsed.transform) : null,
    };
  } catch (_) {
    return null;
  }
}

function writeStoredSelection(storage, record) {
  if (!storage || !record?.sceneId || !record?.ownerId) return;
  try {
    storage.setItem(storageKey(record.sceneId), JSON.stringify({
      sceneId: record.sceneId,
      ownerId: record.ownerId,
      editable: Boolean(record.editable),
      transform: isFiniteCanonicalTransform(record.transform) ? record.transform : null,
    }));
  } catch (_) {
    // Selection retention is optional and must never interrupt authoring.
  }
}

function clearStoredSelection(storage, sceneId) {
  if (!storage || !sceneId) return;
  try {
    storage.removeItem(storageKey(sceneId));
  } catch (_) {
    // Ignore unavailable session storage.
  }
}

export function auditEditableLayoutOwners(items) {
  const editable = (Array.isArray(items) ? items : []).filter(item => item?.editable === true && item?.locked !== true);
  const derived = editable.filter(item => {
    const role = stringValue(item?.role || item?.type).toLowerCase();
    return Boolean(item?.transform_group && item?.target_ref && role === 'place_zone');
  });
  const derivedIds = new Set(derived.map(item => stringValue(item?.id)).filter(Boolean));
  const canonical = editable.filter(item => !derivedIds.has(stringValue(item?.id)));
  return Object.freeze({
    layoutEditableCount: editable.length,
    canonicalEditableOwnerCount: canonical.length,
    derivedEditableCount: derived.length,
    layoutEditableIds: Object.freeze(editable.map(item => stringValue(item?.id)).filter(Boolean)),
    canonicalEditableOwnerIds: Object.freeze(canonical.map(item => stringValue(item?.id)).filter(Boolean)),
    derivedEditableIds: Object.freeze([...derivedIds]),
  });
}

function dispatchState(windowRef, detail) {
  try {
    if (typeof windowRef?.CustomEvent === 'function' && typeof windowRef?.dispatchEvent === 'function') {
      windowRef.dispatchEvent(new windowRef.CustomEvent('workcell:canonical_selection_state', { detail }));
    }
  } catch (_) {
    // Optional diagnostics cannot break editor state.
  }
}

export function installCanonicalSelectionState({
  windowRef = globalThis.window,
  documentRef = globalThis.document,
  storage = windowRef?.sessionStorage,
  scheduleMicrotask = globalThis.queueMicrotask || (callback => Promise.resolve().then(callback)),
  scheduleTimeout = (callback, delay) => windowRef?.setTimeout?.(callback, delay),
} = {}) {
  const api = windowRef?.__WORKCELL_EDITOR_API_V1__;
  if (!api?.getState || !api?.selectItem || !api?.getEditPatch) return false;
  if (api[INSTALL_FLAG]) return true;
  api[INSTALL_FLAG] = true;

  const original = {
    getState: api.getState.bind(api),
    selectItem: api.selectItem.bind(api),
    selectionDiagnostics: api.selectionDiagnostics?.bind(api),
    clearSelection: api.clearSelection?.bind(api),
    setMode: api.setMode?.bind(api),
    undo: api.undo?.bind(api),
    redo: api.redo?.bind(api),
    getEditPatch: api.getEditPatch.bind(api),
    drainEvents: api.drainEvents?.bind(api),
  };

  let canonicalRecord = {
    sceneId: '',
    ownerId: '',
    editable: false,
    transform: null,
    transformSource: 'none',
    sequence: 0,
  };
  let restorePending = false;
  let restoreGeneration = 0;
  let lastStateSignature = '';
  let lastDrainedSelectionOwner = '';

  const rawState = () => original.getState() || {};
  const rawPatch = () => original.getEditPatch() || { edits: [] };

  function retainedRecordFor(state) {
    return readStoredSelection(storage, stringValue(state?.sceneId));
  }

  function resolveCanonicalTransform(state, ownerId) {
    const patchTransform = transformFromPatch(rawPatch(), ownerId);
    if (patchTransform) return { transform: patchTransform, source: 'edit_patch' };
    const inspectorTransform = transformFromInspector(documentRef);
    if (inspectorTransform) return { transform: inspectorTransform, source: 'inspector' };
    if (canonicalRecord.ownerId === ownerId && isFiniteCanonicalTransform(canonicalRecord.transform)) {
      return { transform: clone(canonicalRecord.transform), source: canonicalRecord.transformSource || 'retained' };
    }
    const retained = retainedRecordFor(state);
    if (retained?.ownerId === ownerId && isFiniteCanonicalTransform(retained.transform)) {
      return { transform: clone(retained.transform), source: 'session_retained' };
    }
    return { transform: null, source: 'none' };
  }

  function synchronize(reason = 'poll') {
    const state = rawState();
    const sceneId = stringValue(state.sceneId);
    let ownerId = canonicalOwnerIdFromState(state);
    const retained = retainedRecordFor(state);
    if (!ownerId && restorePending && retained?.ownerId) ownerId = retained.ownerId;
    const resolved = ownerId ? resolveCanonicalTransform(state, ownerId) : { transform: null, source: 'none' };
    const next = {
      sceneId,
      ownerId,
      editable: ownerId === canonicalOwnerIdFromState(state) ? Boolean(state.selectedEditable) : Boolean(retained?.editable),
      transform: resolved.transform,
      transformSource: resolved.source,
      sequence: canonicalRecord.sequence,
    };
    const signature = JSON.stringify({
      sceneId: next.sceneId,
      ownerId: next.ownerId,
      editable: next.editable,
      transform: next.transform,
      source: next.transformSource,
    });
    if (signature !== lastStateSignature) {
      next.sequence += 1;
      lastStateSignature = signature;
      canonicalRecord = next;
      dispatchState(windowRef, {
        reason,
        scene_id: next.sceneId,
        sceneId: next.sceneId,
        canonical_selected_owner_id: next.ownerId,
        canonicalSelectedOwnerId: next.ownerId,
        canonical_transform: clone(next.transform),
        canonicalTransform: clone(next.transform),
        transform_source: next.transformSource,
        transformSource: next.transformSource,
        sequence: next.sequence,
      });
    } else {
      canonicalRecord = { ...next, sequence: canonicalRecord.sequence };
    }
    if (canonicalRecord.ownerId) {
      writeStoredSelection(storage, canonicalRecord);
      writeInspectorTransform(documentRef, canonicalRecord.transform);
    }
    return canonicalRecord;
  }

  function publicState() {
    const state = rawState();
    const record = synchronize('get_state');
    const retained = !canonicalOwnerIdFromState(state) && record.ownerId;
    return {
      ...state,
      selectedItemId: retained ? record.ownerId : state.selectedItemId,
      uiSelectionItemId: retained ? record.ownerId : state.uiSelectionItemId,
      editOwnerItemId: retained ? record.ownerId : state.editOwnerItemId,
      selectedEditable: retained ? record.editable : state.selectedEditable,
      canonicalSelectedOwnerId: record.ownerId,
      canonicalTransform: clone(record.transform),
      canonicalTransformSource: record.transformSource,
      canonicalSelectionSequence: record.sequence,
      retainedSelectionPending: Boolean(retained && restorePending),
    };
  }

  function restoreSelection(attempt = 0, generation = restoreGeneration) {
    if (generation !== restoreGeneration) return false;
    const state = rawState();
    const sceneId = stringValue(state.sceneId);
    const currentOwner = canonicalOwnerIdFromState(state);
    if (currentOwner) {
      restorePending = false;
      synchronize('selection_already_current');
      return true;
    }
    const retained = readStoredSelection(storage, sceneId);
    if (state.ready && retained?.ownerId) {
      const restored = original.selectItem(retained.ownerId) || rawState();
      if (canonicalOwnerIdFromState(restored) === retained.ownerId) {
        restorePending = false;
        synchronize('selection_restored');
        return true;
      }
    }
    if (attempt >= MAX_RESTORE_ATTEMPTS) {
      restorePending = false;
      return false;
    }
    scheduleTimeout?.(() => restoreSelection(attempt + 1, generation), RESTORE_RETRY_MS);
    return false;
  }

  function beginRestore() {
    restorePending = true;
    restoreGeneration += 1;
    restoreSelection(0, restoreGeneration);
  }

  api.getState = () => publicState();
  api.selectItem = id => {
    const selected = original.selectItem(stringValue(id)) || rawState();
    const ownerId = canonicalOwnerIdFromState(selected);
    if (ownerId && stringValue(selected.selectedItemId) !== ownerId) original.selectItem(ownerId);
    restorePending = false;
    synchronize('select_item');
    return publicState();
  };
  api.selectionDiagnostics = () => {
    const diagnostics = original.selectionDiagnostics?.() || {};
    const record = synchronize('selection_diagnostics');
    const patchTransform = record.ownerId ? transformFromPatch(rawPatch(), record.ownerId) : null;
    const inspectorTransform = transformFromInspector(documentRef);
    return {
      ...diagnostics,
      canonicalSelectedOwnerId: record.ownerId,
      canonicalTransform: clone(record.transform),
      canonicalTransformSource: record.transformSource,
      inspectorTransform: clone(inspectorTransform),
      patchTransform: clone(patchTransform),
      gizmoTransform: clone(record.transform),
      gizmoOwnerMatchesSelection: !stringValue(diagnostics.gizmoAttachedTargetId) || stringValue(diagnostics.gizmoAttachedTargetId) === record.ownerId,
    };
  };
  if (original.clearSelection) {
    api.clearSelection = () => {
      const sceneId = stringValue(rawState().sceneId);
      restorePending = false;
      clearStoredSelection(storage, sceneId);
      const result = original.clearSelection();
      canonicalRecord = { sceneId, ownerId: '', editable: false, transform: null, transformSource: 'none', sequence: canonicalRecord.sequence + 1 };
      lastStateSignature = '';
      return result;
    };
  }
  if (original.setMode) {
    api.setMode = mode => {
      const record = synchronize('before_mode_change');
      if (record.ownerId && canonicalOwnerIdFromState(rawState()) !== record.ownerId) original.selectItem(record.ownerId);
      original.setMode(mode);
      synchronize('mode_change');
      return publicState();
    };
  }
  if (original.undo) {
    api.undo = () => {
      original.undo();
      synchronize('undo');
      scheduleMicrotask(() => synchronize('undo_settled'));
      return publicState();
    };
  }
  if (original.redo) {
    api.redo = () => {
      original.redo();
      synchronize('redo');
      scheduleMicrotask(() => synchronize('redo_settled'));
      return publicState();
    };
  }
  api.getEditPatch = () => original.getEditPatch();
  if (original.drainEvents) {
    api.drainEvents = () => {
      const events = original.drainEvents() || [];
      const record = synchronize('drain_events');
      const normalized = events.filter(event => event?.type !== 'selection_changed').map(event => {
        if (event?.type !== 'transform_committed') return event;
        const transform = isFiniteCanonicalTransform(event?.patchEntry?.new_transform)
          ? clone(event.patchEntry.new_transform)
          : clone(record.transform);
        return {
          ...event,
          canonicalOwnerItemId: record.ownerId,
          canonicalTransform: transform,
          savedXyz: clone(transform?.pose?.xyz),
          savedRpy: clone(transform?.pose?.rpy),
        };
      });
      if (record.ownerId !== lastDrainedSelectionOwner) {
        lastDrainedSelectionOwner = record.ownerId;
        normalized.push({
          type: 'selection_changed',
          itemId: record.ownerId,
          uiItemId: record.ownerId,
          editable: record.editable,
          canonicalOwnerItemId: record.ownerId,
          canonicalTransform: clone(record.transform),
        });
      }
      if (!record.ownerId && !restorePending) clearStoredSelection(storage, record.sceneId);
      return normalized;
    };
  }

  const onReadiness = event => {
    const readiness = stringValue(event?.detail?.state || event?.detail?.lifecycle_state || event?.detail?.lifecycleState);
    if (readiness === 'scene_ready') beginRestore();
  };
  windowRef?.addEventListener?.('workcell:web3d_readiness', onReadiness);
  windowRef?.addEventListener?.('workcell:scene-loaded', beginRestore);
  windowRef?.addEventListener?.('pageshow', beginRestore);

  if (typeof windowRef?.MutationObserver === 'function') {
    const inspector = documentRef?.getElementById?.('inspector');
    if (inspector) {
      const observer = new windowRef.MutationObserver(() => scheduleMicrotask(() => synchronize('inspector_rebuilt')));
      observer.observe(inspector, { childList: true, subtree: true });
    }
  }

  synchronize('install');
  beginRestore();
  return true;
}

const publicApi = Object.freeze({
  enabled: true,
  version: VERSION,
  install: installCanonicalSelectionState,
  auditEditableLayoutOwners,
  isFiniteCanonicalTransform,
});

if (typeof window !== 'undefined') {
  window.__WORKCELL_CANONICAL_SELECTION_STATE_V1__ = publicApi;
}

function installWhenReady(attempt = 0) {
  if (typeof window === 'undefined' || typeof document === 'undefined') return;
  if (installCanonicalSelectionState()) return;
  if (attempt >= MAX_INSTALL_ATTEMPTS) {
    console.warn?.('Canonical selection state was not installed because the Product View editor API did not become ready.');
    return;
  }
  window.setTimeout?.(() => installWhenReady(attempt + 1), INSTALL_RETRY_MS);
}

installWhenReady();
