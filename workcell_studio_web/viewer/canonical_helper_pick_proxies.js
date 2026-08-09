const VERSION = 1;
const INSTALL_FLAG = '__WORKCELL_CANONICAL_HELPER_PICK_PROXIES_V1__';
const MAX_INSTALL_ATTEMPTS = 200;
const INSTALL_RETRY_MS = 50;

const OWNER_SUFFIXES = Object.freeze([
  '_fallback_edges',
  '_selection_outline',
  '_visual_outline',
  '_owner_outline',
  '_bounds_highlight',
]);

const OWNERLESS_SELECTION_HELPERS = Object.freeze(new Set([
  'selection_subtle_bounds_highlight',
  'selection_bounds_highlight',
  'selection_outline',
]));

function stringValue(value) {
  return typeof value === 'string' ? value.trim() : '';
}

export function isCanonicalOwnerHelperName(value) {
  const name = stringValue(value).toLowerCase();
  if (!name || /transformcontrols|transform_controls|gizmo|transient_gizmo/.test(name)) return false;
  if (OWNERLESS_SELECTION_HELPERS.has(name)) return true;
  return OWNER_SUFFIXES.some(suffix => name.endsWith(suffix));
}

export function ownerIdFromHelperName(value, previousSelectedItemId = '') {
  const originalName = stringValue(value);
  const name = originalName.toLowerCase();
  if (!isCanonicalOwnerHelperName(name)) return '';
  if (OWNERLESS_SELECTION_HELPERS.has(name)) return stringValue(previousSelectedItemId);
  const suffix = OWNER_SUFFIXES.find(candidate => name.endsWith(candidate));
  return suffix ? originalName.slice(0, originalName.length - suffix.length) : '';
}

function diagnosticHitResolutions(diagnostic) {
  const values = diagnostic?.hit_resolutions || diagnostic?.hitResolutions;
  return Array.isArray(values) ? values : [];
}

function diagnosticHitObjectNames(diagnostic) {
  const values = diagnostic?.hit_object_names || diagnostic?.hitObjectNames;
  return Array.isArray(values) ? values.map(stringValue).filter(Boolean) : [];
}

function resolutionNodeName(resolution) {
  return stringValue(resolution?.hit_node_name || resolution?.hitNodeName);
}

function resolutionSelectionOwnerId(resolution) {
  return stringValue(
    resolution?.selection_owner_id ||
    resolution?.selectionOwnerId ||
    resolution?.edit_owner_id ||
    resolution?.editOwnerId
  );
}

function resolutionRegisteredRecordId(resolution) {
  return stringValue(resolution?.registered_record_id || resolution?.registeredRecordId);
}

export function helperProxyOwnerCandidate(diagnostic, previousSelectedItemId = '') {
  if (!diagnostic || typeof diagnostic !== 'object') return '';

  for (const resolution of diagnosticHitResolutions(diagnostic)) {
    const nodeName = resolutionNodeName(resolution);
    if (!isCanonicalOwnerHelperName(nodeName)) continue;
    const explicitOwner = resolutionSelectionOwnerId(resolution);
    if (explicitOwner) return explicitOwner;
    const registeredOwner = resolutionRegisteredRecordId(resolution);
    if (registeredOwner) return registeredOwner;
    const inferredOwner = ownerIdFromHelperName(nodeName, previousSelectedItemId);
    if (inferredOwner) return inferredOwner;
  }

  for (const nodeName of diagnosticHitObjectNames(diagnostic)) {
    const inferredOwner = ownerIdFromHelperName(nodeName, previousSelectedItemId);
    if (inferredOwner) return inferredOwner;
  }
  return '';
}

export function shouldForwardCanonicalHelperPick(editorState) {
  return Boolean(
    editorState &&
    editorState.lastCanvasPickReason === 'no_eligible_candidate' &&
    editorState.lastFailedCanvasPickDiagnostic
  );
}

function dispatchForwardedEvent(windowRef, detail) {
  try {
    const EventConstructor = windowRef?.CustomEvent;
    if (typeof EventConstructor === 'function' && typeof windowRef.dispatchEvent === 'function') {
      windowRef.dispatchEvent(new EventConstructor('workcell:canonical_helper_pick_forwarded', { detail }));
    }
  } catch (_) {
    // Selection forwarding must not fail because optional diagnostics are unavailable.
  }
}

export function installCanonicalHelperPickProxies({
  windowRef = globalThis.window,
  documentRef = globalThis.document,
  scheduleMicrotask = globalThis.queueMicrotask || (callback => Promise.resolve().then(callback)),
} = {}) {
  const canvas = documentRef?.getElementById?.('scene-canvas');
  const api = windowRef?.__WORKCELL_EDITOR_API_V1__;
  if (!canvas || typeof canvas.addEventListener !== 'function' || !api?.getState || !api?.selectItem) return false;
  if (canvas[INSTALL_FLAG]) return true;
  canvas[INSTALL_FLAG] = true;

  let pointerContext = { previousSelectedItemId: '' };
  canvas.addEventListener('pointerdown', () => {
    const before = api.getState?.() || {};
    pointerContext = { previousSelectedItemId: stringValue(before.selectedItemId) };
  }, true);

  canvas.addEventListener('pointerdown', () => {
    const context = pointerContext;
    scheduleMicrotask(() => {
      const afterPick = api.getState?.() || {};
      if (!shouldForwardCanonicalHelperPick(afterPick)) return;
      const ownerCandidate = helperProxyOwnerCandidate(
        afterPick.lastFailedCanvasPickDiagnostic,
        context.previousSelectedItemId,
      );
      if (!ownerCandidate) return;

      const forwardedState = api.selectItem(ownerCandidate) || api.getState?.() || {};
      const selectedItemId = stringValue(forwardedState.selectedItemId);
      if (!selectedItemId) return;
      const selectionDiagnostics = api.selectionDiagnostics?.() || forwardedState.selectionDiagnostics || {};
      const gizmoTargetId = stringValue(selectionDiagnostics.gizmoAttachedTargetId);
      const helperNodeNames = diagnosticHitObjectNames(afterPick.lastFailedCanvasPickDiagnostic);
      dispatchForwardedEvent(windowRef, {
        helper_node_names: helperNodeNames,
        helperNodeNames,
        forwarded_candidate_id: ownerCandidate,
        forwardedCandidateId: ownerCandidate,
        selected_item_id: selectedItemId,
        selectedItemId,
        selected_editable: Boolean(forwardedState.selectedEditable),
        selectedEditable: Boolean(forwardedState.selectedEditable),
        gizmo_attached_target_id: gizmoTargetId,
        gizmoAttachedTargetId: gizmoTargetId,
        gizmo_attached_to_helper: isCanonicalOwnerHelperName(gizmoTargetId),
        gizmoAttachedToHelper: isCanonicalOwnerHelperName(gizmoTargetId),
      });
    });
  });
  return true;
}

const publicApi = Object.freeze({
  enabled: true,
  version: VERSION,
  install: installCanonicalHelperPickProxies,
  isCanonicalOwnerHelperName,
  ownerIdFromHelperName,
  helperProxyOwnerCandidate,
});

if (typeof window !== 'undefined') {
  window.__WORKCELL_CANONICAL_HELPER_PICK_PROXIES_V1__ = publicApi;
}

function installWhenReady(attempt = 0) {
  if (typeof window === 'undefined' || typeof document === 'undefined') return;
  if (installCanonicalHelperPickProxies()) return;
  if (attempt >= MAX_INSTALL_ATTEMPTS) {
    console.warn?.('Canonical helper pick proxies were not installed because the Product View editor API did not become ready.');
    return;
  }
  window.setTimeout?.(() => installWhenReady(attempt + 1), INSTALL_RETRY_MS);
}

installWhenReady();
