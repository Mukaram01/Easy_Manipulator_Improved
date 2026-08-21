let THREE;
let OrbitControls;
let STLLoader;
let ColladaLoader;
let OBJLoader;
let TransformControls;
let loadRobotPreview;
let applyRobotJointPreview;

const PRODUCT_VIEW_LIGHT_PALETTE = Object.freeze({
  workspaceBackground: 0xeef1f4,
  rendererClearColor: 0xeef1f4,
  gridMajor: 0x7f8b98,
  gridMinor: 0xb6c0ca,
  ambientSky: 0xffffff,
  ambientGround: 0xcbd3dc,
  keyLight: 0xffffff,
  fillLight: 0xdcefff,
  labelText: 0x123040,
  labelSurface: 0xf8fafc,
  overlaySurface: 0xfff6dd,
  errorSurface: 0xffeef0,
  errorAccent: 0xc43434,
});

const SUPPORTED_SCHEMA_VERSION = 'workcell_studio_web_scene/v1';
const EDIT_PATCH_SCHEMA_VERSION = 'workcell_studio_web_scene_edit_patch/v1';
const TRANSFORM_CLIPBOARD_SCHEMA_VERSION = 'workcell_studio_transform/v1';
const VIEWER_VERSION = 'static_web_viewer_edit_patch_v1';
const READINESS_CONTRACT_VERSION = 1;
const PHYSICAL_MESH_LOAD_TIMEOUT_MS = 30000;
const REQUIRED_LOAD_DEADLINE_MS = 40000;
const VALID_SCENE_LIFECYCLE_STATES = Object.freeze(['booting', 'scene_loading', 'scene_ready', 'scene_failed']);
const LOCKED_EDIT_REASON = 'Locked/generated preview item; edit source layout/environment instead.';
const MIN_FRAME_RADIUS = 1.2;
const EMPTY_SCENE_MESSAGE = 'Scene contains no renderable robots, tools, assets, sensors, zones, items, or objects.';
const FRAME_DISTANCE_MULTIPLIER = 2.35;
const CAMERA_PRESET_DIRECTIONS = Object.freeze({
  isometric: [1.35, -1.65, 1.05],
  front: [0, -1, 0.28],
  top: [0, -0.001, 1],
  robot: [1.1, -1.25, 0.72],
});
const state = { sceneJson: null, sourceWebSceneFile: '', diagnosticKeys: new Set(), frameLookup: new Map(), resolvedFramePoses: new Map(), objects: [], pickRecords: [], pickIdentityByObject: new WeakMap(), selectionIdentityIndex: null, physicalEditBindings: new Map(), assemblyRoots: [], robotAssemblyDiagnostics: [], robotAssemblyRenderDiagnostics: {}, robotUrdfPreviewDiagnostics: {}, physicalAssemblyBounds: null, finalPhysicalFitBounds: null, selected: null, selectedRenderIdentityId: '', three: {}, animationId: null, lastFrameBounds: null, initialCameraFit: { sceneKey: '', done: false, attempts: 0, pendingRetry: null, userControlled: false }, runtimeWarnings: [], labelsVisible: false, debugOverlaysVisible: false, dirtyTransforms: new Map(), undoStack: [], redoStack: [], transformClipboard: null, transformClipboardStatus: '', gizmoDragStart: null, gizmoPivot: null, gizmoPivotDragStart: null, cancellingTransformOperation: false, gizmoAttachmentDiagnostic: { targetId: '', reason: 'not_evaluated' }, directMoveDrag: null, directRotateDrag: null, editorMode: 'select', transformSpace: 'world', editorEvents: [], editorError: '', healthAutoOpenedNavigationKey: '', placement: { armed: false, persistent: false, previewRoot: null, asset: null, orientationPreset: 'upright', orientationQuaternion: null, yaw: 0, rawPoint: null, proposedPoint: null, supportOwnerId: '', supportValid: false, collision: false, collidingOwnerIds: [], valid: null }, robotPreviewResult: null, lastRaycastHitCount: 0, lastRaycastCandidateIds: [], lastCanvasSelectedItemId: '', lastCanvasPickReason: '', lastCanvasPickDiagnostic: null, lastFailedCanvasPickDiagnostic: null, initialPosePreview: { active: false, robotId: '', sceneKey: '' }, web3dReadiness: { state: 'booting', terminal: false, terminalState: '', terminalNavigationKey: '', terminalEmissionCount: 0, statusSequence: 0, required: {}, pending: new Set(), failed: false, failure: null }, builderRevision: '', sceneJsonLoaded: false, activeNavigationKey: '' };
const PLACEMENT_COLLISION_EPSILON = 1e-5;
const PLACEMENT_CONTACT_EPSILON = 1e-5;
const PLACEMENT_ORIENTATION_PRESETS = Object.freeze({
  Digit1: Object.freeze({ id: 'upright', label: 'Upright', roll: 0, pitch: 0 }),
  Digit2: Object.freeze({ id: 'positive_x_side', label: '+X side', roll: Math.PI / 2, pitch: 0 }),
  Digit3: Object.freeze({ id: 'negative_x_side', label: '-X side', roll: -Math.PI / 2, pitch: 0 }),
  Digit4: Object.freeze({ id: 'positive_y_side', label: '+Y side', roll: 0, pitch: Math.PI / 2 }),
  Digit5: Object.freeze({ id: 'negative_y_side', label: '-Y side', roll: 0, pitch: -Math.PI / 2 }),
  Digit6: Object.freeze({ id: 'upside_down', label: 'Upside-down', roll: Math.PI, pitch: 0 }),
});
let robotPreviewLoadToken = 0;
let physicalLoadToken = 0;
const RESET_VIEW_TITLE = 'Fit Scene / Reset View: recomputes and reapplies the fitted workcell overview from renderable bounds.';
const STAGED_MESH_ROOTS = [
  'build/workcell_studio_web_scene/assets/',
  'workcell_studio_web/',
  'assets/',
  'scenes/',
];
const EXPECTED_GENERATED_URDF_DIAGNOSTIC_LINKS = Object.freeze([]);

const WEB3D_REQUIRED_CATEGORIES = ['robot_arm', 'attached_tool_gripper', 'workbench_support_surface', 'configured_camera'];
function web3dNavigationKey() { return `${state.sourceWebSceneFile || ''}#${state.builderRevision || ''}`; }
function emitWeb3dReadinessState(readinessState, detail = {}) {
  state.web3dReadiness = state.web3dReadiness || { state: 'booting', terminal: false, terminalState: '', terminalNavigationKey: '', terminalEmissionCount: 0, statusSequence: 0, required: {}, pending: new Set(), failed: false, failure: null };
  if (!VALID_SCENE_LIFECYCLE_STATES.includes(readinessState)) readinessState = 'scene_loading';
  const navigationKey = detail.navigation_key || detail.navigationKey || web3dNavigationKey();
  if (state.web3dReadiness.terminal) {
    if (state.web3dReadiness.terminalNavigationKey && navigationKey && navigationKey !== state.web3dReadiness.terminalNavigationKey) return window.__WORKCELL_VIEWER_STATUS__;
    if (readinessState !== state.web3dReadiness.terminalState) return updateViewerStatus();
    return window.__WORKCELL_VIEWER_STATUS__;
  }
  const terminalTransition = readinessState === 'scene_ready' || readinessState === 'scene_failed';
  state.web3dReadiness.state = readinessState;
  state.web3dReadiness.statusSequence = Number(state.web3dReadiness.statusSequence || 0) + 1;
  if (terminalTransition) {
    state.web3dReadiness.terminal = true;
    state.web3dReadiness.terminalState = readinessState;
    state.web3dReadiness.terminalNavigationKey = navigationKey;
    state.web3dReadiness.terminalEmissionCount = Number(state.web3dReadiness.terminalEmissionCount || 0) + 1;
  }
  const structured = structuredWeb3dReadinessFields(readinessState);
  const eventDetail = { ...structured, ...detail, final_lifecycle_state: readinessState, finalLifecycleState: readinessState, pending_required_loads: pendingRequiredLoads() };
  if (readinessState === 'scene_failed') {
    state.web3dReadiness.failed = true;
    state.web3dReadiness.failure = eventDetail;
  }
  const status = updateViewerStatus();
  if (readinessState === 'scene_ready') triggerInitialCameraFitAfterSceneReady();
  window.dispatchEvent?.(new CustomEvent('workcell:web3d_readiness', { detail: { state: readinessState, ...eventDetail, status } }));
  window.parent?.postMessage?.({ type: 'workcell_web3d_readiness', state: readinessState, ...eventDetail, status }, '*');
  return status;
}
function itemRenderPolicy(item) { return String(item?.render_policy || item?.renderPolicy || 'legacy_primary').toLowerCase(); }
function itemRenderOwner(item) { return String(item?.render_owner || item?.renderOwner || '').toLowerCase(); }
function isPrimaryRenderableItem(item) {
  const policy = itemRenderPolicy(item);
  return policy === 'primary' || policy === 'legacy_primary';
}
function isDiagnosticOnlyItem(item) { return itemRenderPolicy(item) === 'diagnostic_only'; }
function isOverlayPolicyItem(item) { return itemRenderPolicy(item) === 'overlay'; }
function isTaskOnlyHelperItem(item) {
  const identity = [
    item?.source_layer,
    item?.active_visual_source,
    item?.render_owner,
    item?.role,
    item?.category,
    item?.type,
    item?.id,
    item?.display_name,
  ].map(value => String(value || '').toLowerCase().replace(/[_-]+/g, ' ')).join(' ');
  return item?.task_only === true || item?.non_blocking_pick === true ||
    /\b(task helper|task marker|source helper|commissioning object)\b/.test(identity);
}
function isPrimaryAuthoredPhysicalMesh(item) {
  if (itemRenderPolicy(item) !== 'primary') return false;
  const contractCategory = String(item?.mesh_contract_category || item?.meshContractCategory || '').toLowerCase();
  const identity = viewerGroupIdentity(item);
  const authoredMesh = Boolean(displayMeshUri(item)) && /\b(authored|editable|layout|imported)\b/.test(identity);
  return (contractCategory === 'object' && (truthyFlag(item?.mesh_load_required) || authoredMesh)) ||
    /\b(target bin|target container|destination bin)\b/.test(identity);
}
function readinessCategoryForItem(item) {
  if (!item || !isPrimaryRenderableItem(item) || isDebugOverlayItem(item)) return '';
  if (item?.readiness_category || item?.readinessCategory) return String(item.readiness_category || item.readinessCategory);
  if (isPrimaryAuthoredPhysicalMesh(item)) return 'authored_physical_mesh';
  const category = meshContractCategoryOf(item);
  const identity = viewerGroupIdentity(item);
  if (category === 'camera' || isSensor(item)) return 'configured_camera';
  if (category === 'table' || supportSurfaceDisplayType(item) || /\b(workbench|support surface|tabletop|table)\b/.test(identity)) return 'workbench_support_surface';
  if (isGeneratedToolOrGripperItem(item) || category === 'tool') return 'attached_tool_gripper';
  if (isGeneratedRobotItem(item) || category === 'robot') return 'robot_arm';
  if (itemRequiresMeshBackedVisual(item) && (category === 'object' || viewerGroupFor(item) === 'environment/layout')) return 'authored_physical_mesh';
  return '';
}
function configuredCameraRelationshipIds(item) {
  const fields = [
    'configured_camera_id', 'configuredCameraId', 'camera_id', 'cameraId',
    'canonical_scene_item_id', 'canonicalSceneItemId', 'canonical_item_id', 'canonicalItemId',
    'authored_item_id', 'authoredItemId', 'authored_id', 'authoredId',
    'layout_item_ref', 'layoutItemRef', 'scene_item_id', 'sceneItemId',
    'selection_owner_id', 'selectionOwnerId', 'owner_id', 'ownerId',
    'object_ref', 'objectRef', 'source_item_id', 'sourceItemId',
  ];
  return fields.map(field => String(item?.[field] || '').trim()).filter(Boolean);
}
function canonicalAuthoredCameraId(item) {
  const relationshipIds = new Set(configuredCameraRelationshipIds(item));
  const ownId = String(item?.id || '').trim();
  if (ownId) relationshipIds.add(ownId);
  const authoredRows = collectItems(state.sceneJson || {}).filter(candidate => {
    if (candidate === item || readinessCategoryForItem(candidate) !== 'configured_camera') return false;
    const candidateId = String(candidate?.id || '').trim();
    if (!candidateId || isGeneratedUrdfItem(candidate)) return false;
    const candidateRefs = configuredCameraRelationshipIds(candidate);
    return relationshipIds.has(candidateId) || candidateRefs.some(value => relationshipIds.has(value));
  });
  const canonical = authoredRows.find(candidate => isPrimaryRenderableItem(candidate) && (candidate?.editable === true || String(candidate?.source_layer || '').includes('authored')))
    || authoredRows.find(candidate => isPrimaryRenderableItem(candidate))
    || authoredRows[0];
  return String(canonical?.id || '').trim();
}
function readinessIdentityForItem(category, item) {
  if (category === 'configured_camera') {
    const authoredId = canonicalAuthoredCameraId(item);
    if (authoredId) return authoredId;
    const relationshipId = configuredCameraRelationshipIds(item)[0];
    if (relationshipId) return relationshipId;
  }
  return String(item?.id || item?.link || itemLabel(item || {}));
}
function readinessKey(category, item) { return `${category}:${readinessIdentityForItem(category, item)}`; }
function pendingRequiredLoads() { return Array.from(state.web3dReadiness?.pending || []); }

function physicalReadinessItems() {
  return collectItems(state.sceneJson || {}).filter(item => isPrimaryRenderableItem(item) && !isDebugOverlayItem(item) && readinessCategoryForItem(item));
}
function renderedPhysicalItemCount() {
  const assemblyCount = collectPhysicalAssemblyBounds?.()?.count || 0;
  return statusCountedRenderables().length + assemblyCount;
}
function expectedPhysicalItemCount() {
  const items = physicalReadinessItems();
  const hasExpanded = isExpandedUrdfRobotPreview(state.sceneJson?.robot_preview);
  return items.length + (hasExpanded && !items.some(item => readinessCategoryForItem(item) === 'robot_arm') ? 1 : 0);
}
function failedRequiredItemCount() {
  const renderFailures = statusCountedRenderables().filter(isRequiredMeshFailureStatus).length;
  return renderFailures + (state.web3dReadiness?.state === 'scene_failed' && renderFailures === 0 ? 1 : 0);
}
function readinessCategoryStatus(category) {
  const readiness = state.web3dReadiness || {};
  if (!readiness.required?.[category]) return 'missing';
  const pending = pendingRequiredLoads().some(key => String(key).startsWith(`${category}:`));
  if (readiness.state === 'scene_failed' && (readiness.failure?.required_category === category || pending)) return 'failed';
  if (pending) return 'pending';
  return readiness.state === 'scene_ready' ? 'ready' : 'loading';
}
function structuredWeb3dReadinessFields(lifecycleState) {
  const finalState = lifecycleState || state.web3dReadiness?.state || 'booting';
  const pendingLoads = pendingRequiredLoads();
  return {
    scene_id: sceneId(),
    sceneId: sceneId(),
    expected_physical_item_count: expectedPhysicalItemCount(),
    expectedPhysicalItemCount: expectedPhysicalItemCount(),
    rendered_physical_item_count: renderedPhysicalItemCount(),
    renderedPhysicalItemCount: renderedPhysicalItemCount(),
    failed_required_item_count: failedRequiredItemCount(),
    failedRequiredItemCount: failedRequiredItemCount(),
    robot_status: readinessCategoryStatus('robot_arm'),
    robotStatus: readinessCategoryStatus('robot_arm'),
    tool_status: readinessCategoryStatus('attached_tool_gripper'),
    toolStatus: readinessCategoryStatus('attached_tool_gripper'),
    end_effector_status: readinessCategoryStatus('attached_tool_gripper'),
    endEffectorStatus: readinessCategoryStatus('attached_tool_gripper'),
    environment_status: readinessCategoryStatus('workbench_support_surface'),
    environmentStatus: readinessCategoryStatus('workbench_support_surface'),
    camera_status: readinessCategoryStatus('configured_camera'),
    cameraStatus: readinessCategoryStatus('configured_camera'),
    pending_required_loads: pendingLoads,
    pendingRequiredLoads: pendingLoads,
    readiness_contract_version: READINESS_CONTRACT_VERSION,
    readinessContractVersion: READINESS_CONTRACT_VERSION,
    lifecycle_state: finalState,
    lifecycleState: finalState,
    terminal: Boolean(state.web3dReadiness?.terminal),
    status_sequence: Number(state.web3dReadiness?.statusSequence || 0),
    statusSequence: Number(state.web3dReadiness?.statusSequence || 0),
    source_web_scene_file: state.sourceWebSceneFile || '',
    sourceWebSceneFile: state.sourceWebSceneFile || '',
    builder_revision: state.builderRevision || '',
    builderRevision: state.builderRevision || '',
    final_lifecycle_state: finalState,
    finalLifecycleState: finalState,
  };
}
function beginWeb3dSceneReadiness(items) {
  const required = Object.fromEntries(WEB3D_REQUIRED_CATEGORIES.map(category => [category, false]));
  for (const item of items || []) {
    const category = readinessCategoryForItem(item);
    if (category) required[category] = true;
  }
  if (isExpandedUrdfRobotPreview(state.sceneJson?.robot_preview)) {
    required.robot_arm = true;
    required.attached_tool_gripper = true;
  }
  state.web3dReadiness = { state: 'scene_loading', terminal: false, terminalState: '', terminalNavigationKey: web3dNavigationKey(), terminalEmissionCount: 0, statusSequence: 0, required, pending: new Set(), failed: false, failure: null };
  emitWeb3dReadinessState('scene_loading', { required_categories: required });
}
function registerReadinessOperation(keys, options = {}) {
  const pendingKeys = new Set(asArray(keys).map(key => String(key || '').trim()).filter(Boolean));
  const operation = {
    id: String(options.operationId || `required_load:${Array.from(pendingKeys).join('|')}`),
    pendingKeys,
    physicalLoadToken,
    navigationKey: web3dNavigationKey(),
    robotPreviewLoadToken: options.robotPreviewLoadToken,
    completed: false,
  };
  for (const key of operation.pendingKeys) state.web3dReadiness?.pending?.add(key);
  return operation;
}
function readinessOperationIsCurrent(operation) {
  return Boolean(operation) && operation.physicalLoadToken === physicalLoadToken && operation.navigationKey === web3dNavigationKey()
    && (operation.robotPreviewLoadToken === undefined || operation.robotPreviewLoadToken === robotPreviewLoadToken);
}
function completeReadinessOperation(operation) {
  if (!readinessOperationIsCurrent(operation) || operation.completed) return false;
  operation.completed = true;
  for (const key of operation.pendingKeys) state.web3dReadiness?.pending?.delete(key);
  maybeEmitSceneReady();
  return true;
}
function readinessOperationDiagnostic(operation, outcome, extra = {}) {
  return {
    operation_id: operation?.id || 'unknown_required_load',
    operation_pending_keys: Array.from(operation?.pendingKeys || []),
    operation_outcome: outcome,
    pending_required_loads: pendingRequiredLoads(),
    ...extra,
  };
}
function requiredReadinessCompleteForItem(item) {
  const category = readinessCategoryForItem(item);
  if (!category) return;
  state.web3dReadiness?.pending?.delete(readinessKey(category, item));
  maybeEmitSceneReady();
}
function physicalMeshAttempt(item, operation) {
  const category = readinessCategoryForItem(item);
  const identity = category ? readinessIdentityForItem(category, item) : '';
  return {
    operation: operation || registerReadinessOperation(category ? [`${category}:${identity}`] : []),
    token: physicalLoadToken,
    navigationKey: web3dNavigationKey(),
    category,
    identity,
    readinessKey: category ? `${category}:${identity}` : '',
    deadlineAt: Date.now() + PHYSICAL_MESH_LOAD_TIMEOUT_MS,
  };
}
function physicalMeshAttemptIsCurrent(attempt) {
  return readinessOperationIsCurrent(attempt?.operation) && attempt.token === physicalLoadToken && attempt.navigationKey === web3dNavigationKey();
}
function completePhysicalMeshAttempt(attempt) {
  if (!physicalMeshAttemptIsCurrent(attempt)) return false;
  return completeReadinessOperation(attempt.operation);
}
function failPhysicalMeshAttempt(attempt, item, url, reason, extra = {}) {
  if (!physicalMeshAttemptIsCurrent(attempt)) return false;
  emitWeb3dReadinessState('scene_failed', {
    required_category: attempt.category || 'required_physical_item',
    readiness_identity: attempt.identity,
    readiness_key: attempt.readinessKey,
    item_id: item?.id || '',
    link: item?.link || item?.link_name || item?.object_name || '',
    url: url || displayMeshUri(item),
    reason: reason || 'required mesh failed',
    ...extra,
    loader: extra.loader || extra.loader_type || '',
    loader_type: extra.loader_type || extra.loader || '',
    pending_required_loads: pendingRequiredLoads(),
  });
  return true;
}
function physicalMeshBoundsFailurePayload(item, loadUrl, loader) {
  const expected = expectedDimensionsOf(item);
  const localBounds = item?.loaded_mesh_local_bounds || item?.loaded_mesh_bounds || null;
  const worldBounds = item?.loaded_mesh_world_bounds || null;
  return {
    scene_id: sceneId(),
    item_id: item?.id || '',
    item_display_name: item?.display_name || item?.label || item?.name || item?.object_name || item?.link || item?.id || '',
    category: meshContractCategoryOf(item),
    mesh_uri: displayMeshUri(item),
    mesh_load_url: loadUrl || '',
    loader: loader || '',
    expected_dimensions: expected ? { x: expected.x, y: expected.y, z: expected.z } : null,
    loaded_local_dimensions: localBounds?.dimensions || null,
    loaded_local_bounds: localBounds,
    loaded_local_bounds_coordinate_space: item?.loaded_mesh_bounds_coordinate_space?.validation_bounds || 'authored_visual_local_after_unit_correction',
    loaded_world_dimensions: worldBounds?.dimensions || null,
    loaded_world_bounds: worldBounds,
    loaded_world_bounds_coordinate_space: item?.loaded_mesh_bounds_coordinate_space?.diagnostic_world_bounds || 'scene_world_after_visual_origin_and_item_pose',
    mesh_bounds_coordinate_space_contract: item?.loaded_mesh_bounds_coordinate_space || MESH_BOUNDS_COORDINATE_SPACE_CONTRACT,
    axis_ratios: item?.loaded_mesh_axis_ratios || null,
    maximum_ratio: item?.loaded_mesh_maximum_ratio ?? null,
    uniform_ratio: item?.loaded_mesh_uniform_ratio ?? null,
    applied_mesh_scale: item?.mesh_unit_correction?.scale ?? 1.0,
    unit_correction_decision: item?.mesh_unit_correction?.confidence || 'not_requested_or_not_applicable',
    bounds_reason_code: item?.loaded_mesh_bounds_reason_code || 'loaded_mesh_bounds_invalid',
  };
}
function failWeb3dSceneReadiness(item, url, reason, extra = {}) {
  const category = readinessCategoryForItem(item) || extra.category || 'required_physical_item';
  const readinessIdentity = readinessIdentityForItem(category, item);
  emitWeb3dReadinessState('scene_failed', {
    required_category: category,
    readiness_identity: readinessIdentity,
    readiness_key: `${category}:${readinessIdentity}`,
    item_id: item?.id || '',
    link: item?.link || item?.link_name || item?.object_name || '',
    url: url || displayMeshUri(item),
    reason: reason || 'required mesh failed',
    ...extra,
    pending_required_loads: pendingRequiredLoads(),
  });
}
function completeExpandedUrdfReadiness(operation) { return completeReadinessOperation(operation); }
function failExpandedUrdfReadiness(operation, err, diagnostics = {}, detail = {}) {
  if (!readinessOperationIsCurrent(operation) || operation.completed) return false;
  const link = String(detail.link || detail.link_name || '').trim();
  const expectedTool = new Set(asArray(state.sceneJson?.robot_preview?.expected_tool_visual_links || state.sceneJson?.robot_preview?.expectedToolVisualLinks).map(value => String(value || '').trim()).filter(Boolean));
  emitWeb3dReadinessState('scene_failed', {
    required_category: expectedTool.has(link) ? 'attached_tool_gripper' : 'robot_arm',
    item_id: 'expanded_urdf_loader',
    link,
    url: detail.url || detail.uri || diagnostics.robot_urdf_url || '',
    reason: err?.message || String(err || 'expanded URDF required mesh failed'),
    robot_preview_lifecycle_state: diagnostics.robot_preview_lifecycle_state || diagnostics.robotPreviewLifecycleState || '',
    robot_loaded_visual_count: Number(diagnostics.robot_loaded_visual_count ?? diagnostics.robotLoadedVisualCount ?? 0) || 0,
    robot_expected_visual_count: Number(diagnostics.robot_expected_visual_count ?? diagnostics.robotExpectedVisualCount ?? 0) || 0,
    robot_failed_visual_count: Number(diagnostics.robot_failed_visual_count ?? diagnostics.robotFailedVisualCount ?? 0) || 0,
    robot_missing_meshes: diagnostics.robot_missing_meshes || [],
    ...detail,
    pending_required_loads: pendingRequiredLoads(),
  });
  return true;
}
function expandedUrdfTerminalFailure(rendererDiagnostics = {}) {
  const list = (snakeName, camelName) => asArray(rendererDiagnostics[snakeName] || rendererDiagnostics[camelName])
    .map(value => String(value || '').trim()).filter(Boolean);
  const lifecycle = String(rendererDiagnostics.robot_preview_lifecycle_state || rendererDiagnostics.robotPreviewLifecycleState || '').trim();
  const previewLoaded = rendererDiagnostics.robot_preview_loaded === true || rendererDiagnostics.robotPreviewLoaded === true;
  const previewExplicitlyNotLoaded = rendererDiagnostics.robot_preview_loaded === false || rendererDiagnostics.robotPreviewLoaded === false;
  const expectedVisualCount = Number(rendererDiagnostics.robot_expected_visual_count ?? rendererDiagnostics.robotExpectedVisualCount ?? 0) || 0;
  const loadedVisualCount = Number(rendererDiagnostics.robot_loaded_visual_count ?? rendererDiagnostics.robotLoadedVisualCount ?? 0) || 0;
  const completedVisualCount = Number(rendererDiagnostics.robot_completed_visual_count ?? rendererDiagnostics.robotCompletedVisualCount ?? 0) || 0;
  const failedVisualCount = Number(rendererDiagnostics.robot_failed_visual_count ?? rendererDiagnostics.robotFailedVisualCount ?? 0) || 0;
  const missingToolLinks = list('robot_missing_required_tool_visual_links', 'robotMissingRequiredToolVisualLinks');
  const missingRobotLinks = list('robot_missing_required_robot_visual_links', 'robotMissingRequiredRobotVisualLinks');
  const missingHierarchyLinks = list('robot_hierarchy_missing_links', 'robotHierarchyMissingLinks');
  const loadingManagerComplete = rendererDiagnostics.robot_loading_manager_complete === true || rendererDiagnostics.robotLoadingManagerComplete === true;
  const loadingManagerExplicitlyIncomplete = rendererDiagnostics.robot_loading_manager_complete === false || rendererDiagnostics.robotLoadingManagerComplete === false;
  const meshCallbacksComplete = rendererDiagnostics.robot_mesh_callbacks_complete === true || rendererDiagnostics.robotMeshCallbacksComplete === true;
  const meshCallbacksExplicitlyIncomplete = rendererDiagnostics.robot_mesh_callbacks_complete === false || rendererDiagnostics.robotMeshCallbacksComplete === false;

  let requiredCategory = '';
  let reason = '';
  if (missingToolLinks.length) {
    requiredCategory = 'attached_tool_gripper';
    reason = `Expanded URDF tool preview is missing required visual links: ${missingToolLinks.join(', ')}. Check the tool URDF mesh paths and regenerate the scene.`;
  } else if (missingRobotLinks.length) {
    requiredCategory = 'robot_arm';
    reason = `Expanded URDF robot preview is missing required visual links: ${missingRobotLinks.join(', ')}. Check the robot URDF mesh paths and regenerate the scene.`;
  } else if (missingHierarchyLinks.length) {
    requiredCategory = 'robot_arm';
    reason = `Expanded URDF robot hierarchy is missing required links: ${missingHierarchyLinks.join(', ')}. Check the generated URDF link/joint hierarchy and regenerate the scene.`;
  } else if (meshCallbacksExplicitlyIncomplete) {
    requiredCategory = 'robot_arm';
    reason = `Expanded URDF mesh callbacks did not complete (${completedVisualCount}/${expectedVisualCount} completed). Check failed mesh requests and the browser console, then regenerate or reload the scene.`;
  } else if (lifecycle === 'failed' && previewExplicitlyNotLoaded) {
    requiredCategory = 'robot_arm';
    reason = String(rendererDiagnostics.robot_preview_failure_reason || rendererDiagnostics.robotPreviewFailureReason || '').trim()
      || 'Expanded URDF preview entered the failed lifecycle without loading. Check the URDF and mesh request diagnostics, then regenerate or reload the scene.';
  } else {
    return null;
  }

  return {
    required_category: requiredCategory,
    item_id: 'expanded_urdf_loader',
    reason,
    robot_preview_lifecycle_state: lifecycle,
    robot_preview_loaded: previewLoaded,
    robot_expected_visual_count: expectedVisualCount,
    robot_loaded_visual_count: loadedVisualCount,
    robot_completed_visual_count: completedVisualCount,
    robot_failed_visual_count: failedVisualCount,
    robot_missing_required_tool_visual_links: missingToolLinks,
    robot_missing_required_robot_visual_links: missingRobotLinks,
    robot_hierarchy_missing_links: missingHierarchyLinks,
    robot_loading_manager_complete: loadingManagerComplete,
    robot_loading_manager_completion_state: loadingManagerComplete ? 'complete' : (loadingManagerExplicitlyIncomplete ? 'incomplete' : 'unknown'),
    robot_mesh_callbacks_complete: meshCallbacksComplete,
    robot_mesh_callback_completion_state: meshCallbacksComplete ? 'complete' : (meshCallbacksExplicitlyIncomplete ? 'incomplete' : 'unknown'),
  };
}
function maybeEmitSceneReady() {
  if (isExpandedUrdfRobotPreview(state.sceneJson?.robot_preview)) {
    const diagnostics = state.robotUrdfPreviewDiagnostics || {};
    const lifecycle = String(diagnostics.robot_preview_lifecycle_state || diagnostics.robotPreviewLifecycleState || '');
    const terminalFailure = expandedUrdfTerminalFailure(diagnostics);
    if (terminalFailure) {
      emitWeb3dReadinessState('scene_failed', terminalFailure);
      return;
    }
    if (lifecycle === 'failed') {
      const failureReason = String(diagnostics.robot_preview_failure_reason || diagnostics.robotPreviewFailureReason || '').trim();
      const missingMeshes = asArray(diagnostics.robot_missing_meshes || diagnostics.robotMissingMeshes);
      const failedVisualCount = Number(diagnostics.robot_failed_visual_count ?? diagnostics.robotFailedVisualCount ?? 0) || 0;
      if (!failureReason && missingMeshes.length === 0 && failedVisualCount === 0) return;
      return;
    }
    if (lifecycle !== 'ready') return;
  }
  if (failIfExpandedUrdfExpectedVisualSetInvalid()) return;
  const readiness = state.web3dReadiness;
  if (!readiness || readiness.failed || readiness.state === 'scene_failed') return;
  const missing = WEB3D_REQUIRED_CATEGORIES.filter(category => !readiness.required?.[category]);
  if (missing.length) {
    emitWeb3dReadinessState('scene_failed', { reason: `missing required physical categories: ${missing.join(', ')}`, missing_required_categories: missing });
    return;
  }
  if (readiness.pending?.size === 0) emitWeb3dReadinessState('scene_ready', { required_categories: readiness.required });
}


const el = {
  file: document.getElementById('scene-file'),
  resetView: document.getElementById('reset-view'),
  cameraPreset: document.getElementById('camera-preset'),
  undoEdit: document.getElementById('undo-edit'),
  redoEdit: document.getElementById('redo-edit'),
  transformSpace: document.getElementById('transform-space'),
  clearEdits: document.getElementById('clear-edits'),
  exportEditPatch: document.getElementById('export-edit-patch'),
  dirty: document.getElementById('dirty-state'),
  snapToggle: document.getElementById('snap-toggle'),
  translationSnap: document.getElementById('translation-snap'),
  rotationSnap: document.getElementById('rotation-snap'),
  labelsToggle: document.getElementById('labels-toggle'),
  debugOverlaysToggle: document.getElementById('debug-overlays-toggle'),
  showInitialPose: document.getElementById('show-initial-pose'),
  initialPoseStatus: document.getElementById('initial-pose-status'),
  placementStatus: document.getElementById('placement-status'),
  canvas: document.getElementById('scene-canvas'),
  labelLayer: document.getElementById('label-layer'),
  empty: document.getElementById('empty-state'),
  error: document.getElementById('error-state'),
  list: document.getElementById('object-list'),
  inspector: document.getElementById('inspector'),
  warnings: document.getElementById('warnings'),
  summary: document.getElementById('scene-summary'),
  sceneHealth: document.getElementById('scene-health'),
  sceneHealthTitle: document.getElementById('scene-health-title'),
  sceneHealthSummary: document.getElementById('scene-health-summary'),
  sceneHealthCount: document.getElementById('scene-health-count'),
  sceneHealthItems: document.getElementById('scene-health-items'),
  warningsDetails: document.getElementById('warnings-details'),
  warningCount: document.getElementById('warning-count'),
};



function robotRecords() {
  const records = asArray(state.sceneJson?.robots).concat(asArray(state.sceneJson?.robot_preview?.robots));
  if (state.sceneJson?.robot_preview && !records.length) records.push(state.sceneJson.robot_preview);
  return records.filter(r => r && (r.urdf_url || r.joint_values || r.initial_joint_values || r.initial_positions || r.configured_initial_positions));
}
function robotRecordId(robot, index = 0) { return String(robot?.id || robot?.robot_id || robot?.robot_instance_id || robot?.name || `robot_${index}`).trim(); }
function selectedRobotRecord() {
  const robots = robotRecords();
  if (robots.length === 1) return { robot: robots[0], id: robotRecordId(robots[0], 0), automatic: true };
  const selected = String(state.selected || '');
  const found = robots.find((robot, index) => robotRecordId(robot, index) === selected);
  return found ? { robot: found, id: selected, automatic: false } : null;
}
function configuredInitialJointValues(robot) {
  return robot?.initial_joint_values || robot?.configured_initial_positions || robot?.initial_positions || robot?.joint_initial_positions || null;
}
function normalJointValues(robot) { return robot?.joint_values || {}; }
function validateInitialJointMap(robot, result) {
  const values = configuredInitialJointValues(robot);
  if (!values || Array.isArray(values) || typeof values !== 'object') throw new Error('Initial pose is not configured');
  const joints = Array.from((result?.joints || new Map()).entries());
  if (!joints.length) throw new Error('Product View is not ready');
  const movable = joints.filter(([, joint]) => !['fixed'].includes(String(joint?.jointType || '').toLowerCase()) && !joint?.isURDFMimicJoint && !joint?.mimicJoint);
  const names = new Set();
  for (const [name] of joints) { if (names.has(name)) throw new Error(`Initial pose has duplicate joint: ${name}`); names.add(name); }
  for (const name of Object.keys(values)) {
    if (!names.has(name)) throw new Error(`Initial pose has unknown joint: ${name}`);
    if (!Number.isFinite(Number(values[name]))) throw new Error('Initial pose contains an invalid value');
  }
  for (const [name] of movable) if (!(name in values)) throw new Error(`Initial pose is missing joint: ${name}`);
  return Object.fromEntries(Object.entries(values).map(([name, value]) => [name, Number(value)]));
}
function setInitialPosePreviewUi(active, message = '') {
  if (el.initialPoseStatus) { el.initialPoseStatus.hidden = !active; el.initialPoseStatus.textContent = active ? 'Initial pose preview' : ''; }
  if (message) pushEditorEvent('status', { message });
}
function refreshInitialPoseActionState() {
  if (!el.showInitialPose) return;
  const robots = robotRecords();
  el.showInitialPose.disabled = robots.length === 0 || (robots.length > 1 && !selectedRobotRecord());
}
function clearInitialPosePreview({ message = '' } = {}) {
  if (!state.initialPosePreview.active) { refreshInitialPoseActionState(); return; }
  const selected = selectedRobotRecord();
  const result = state.robotPreviewResult;
  if (selected && result) applyRobotJointPreview?.(result, normalJointValues(selected.robot));
  state.initialPosePreview = { active: false, robotId: '', sceneKey: '' };
  if (el.showInitialPose) el.showInitialPose.checked = false;
  setInitialPosePreviewUi(false, message || 'Initial pose preview ended');
  refreshWarnings();
}
function toggleInitialPosePreview(checked) {
  if (!checked) { clearInitialPosePreview(); return; }
  try {
    if (!state.robotPreviewResult?.root) throw new Error('Product View is not ready');
    const selected = selectedRobotRecord();
    if (!selected) throw new Error(robotRecords().length > 1 ? 'Select a robot to preview its initial pose' : 'Initial pose is not configured');
    const jointMap = validateInitialJointMap(selected.robot, state.robotPreviewResult);
    applyRobotJointPreview(state.robotPreviewResult, jointMap);
    state.initialPosePreview = { active: true, robotId: selected.id, sceneKey: sceneId() };
    setInitialPosePreviewUi(true, `Showing initial pose for ${selected.robot.name || selected.id}`);
  } catch (err) {
    if (el.showInitialPose) el.showInitialPose.checked = false;
    state.initialPosePreview = { active: false, robotId: '', sceneKey: '' };
    setInitialPosePreviewUi(false);
    showError(err.message || String(err));
  }
}

function sceneDisplayName() { return state.sceneJson?.scene?.id || state.sceneJson?.scene_id || state.sourceWebSceneFile || 'No scene loaded'; }
function isGeneratedOrLockedItem(item) {
  const sourceIdentity = [item?.source_kind, item?.source_layer, item?.active_visual_source, item?.role, item?.category, item?.id, itemLabel(item || {})]
    .map(value => String(value || '').toLowerCase())
    .join(' ');
  return Boolean(item?.locked || sourceIdentity.includes('generated') || sourceIdentity.includes('urdf') || sourceIdentity.includes('moveit'));
}
function isRuntimeFallbackStatus(status) { return /fallback/.test(String(status || '').toLowerCase()); }
function isMissingOrFailedMeshStatus(status) {
  return ['missing_file', 'unresolved_package_uri', 'unsafe_path', 'unsupported_format', 'load_error', 'url_not_served', 'file_access_blocked', 'loader_failure']
    .includes(String(status || '').toLowerCase());
}
function statusCountedRenderables() {
  return (state.objects || []).filter(obj => isPrimaryRenderableItem(obj.item) && !isDebugOverlayItem(obj.item));
}
function isRequiredMeshFailureStatus(obj) {
  return obj?.renderInfo?.render_status === 'required_mesh_failed_debug_fallback' || obj?.item?.renderInfo?.render_status === 'required_mesh_failed_debug_fallback';
}
function computeSceneSummary() {
  const rendered = state.objects || [];
  const statusRendered = statusCountedRenderables();
  return {
    sceneName: sceneDisplayName(),
    renderableCount: rendered.length,
    meshLoadedCount: statusRendered.filter(obj => obj.renderInfo?.render_status === 'mesh_loaded').length,
    expectedMeshLoadedCount: statusRendered.filter(obj => itemRequiresMeshBackedVisual(obj.item)).length,
    meshVisuallyInvalidCount: statusRendered.filter(obj => obj.item?.visual_bounds_status && !['valid', 'corrected_by_local_unit_scale'].includes(obj.item.visual_bounds_status)).length,
    meshUnitScaleCorrectedCount: statusRendered.filter(obj => obj.item?.visual_bounds_status === 'corrected_by_local_unit_scale').length,
    fallbackCount: statusRendered.filter(obj => isRuntimeFallbackStatus(obj.renderInfo?.render_status || obj.item?.renderInfo?.render_status)).length,
    meshFailedCount: statusRendered.filter(obj => isMissingOrFailedMeshStatus(obj.item?.mesh_status)).length,
    generatedLockedCount: rendered.filter(obj => isGeneratedOrLockedItem(obj.item)).length,
    editableCount: rendered.filter(obj => canEditItem(obj.item)).length,
    robotPreviewMode: robotPreviewSummaryMode(state.sceneJson?.robot_preview),
  };
}
function isExpectedMeshlessTool0Frame(item) {
  const link = linkNameOfItem(item);
  if (link !== 'tool0') return false;
  if (displayMeshUri(item)) return false;
  const geometry = String(item?.geometry_type || item?.primitive_geometry_type || item?.type || '').toLowerCase();
  return !geometry || ['frame', 'anchor', 'none', 'link_frame', 'meshless_frame'].includes(geometry) || item?.render_expected === false;
}
function collectAssemblyRenderDiagnostics() {
  const diagnostics = state.robotAssemblyRenderDiagnostics || {};
  const assemblyBounds = collectPhysicalAssemblyBounds();
  let effectiveFinalFitBox = state.finalPhysicalFitBounds ? state.finalPhysicalFitBounds.clone() : null;
  if (assemblyBounds.bounds) {
    effectiveFinalFitBox = effectiveFinalFitBox ? effectiveFinalFitBox.union(assemblyBounds.bounds) : assemblyBounds.bounds.clone();
    state.finalPhysicalFitBounds = effectiveFinalFitBox.clone();
  }
  const finalFitBounds = effectiveFinalFitBox ? box3ToJson(effectiveFinalFitBox) : null;
  const rendered = state.objects || [];
  const independentGenerated = rendered.filter(obj => {
    const item = obj?.item || {};
    return isGeneratedUrdfMeshVisualItem(item)
      && usesAssembledUrdfHierarchy(item)
      && !obj?.object3d?.userData?.assembled_urdf_hierarchy;
  });
  const visibleTool0Fallback = rendered.filter(obj => {
    const item = obj?.item || {};
    return linkNameOfItem(item) === 'tool0' && Boolean(obj?.fallback?.visible);
  });
  const distances = (state.robotAssemblyDiagnostics || []).flatMap(d => {
    const adjacency = d.assembled_link_adjacency_distances_m || {};
    return [adjacency['tool0 -> gripper_base_link'], adjacency['wrist_3_link -> gripper_base_link']];
  }).filter(value => typeof value === 'number' && Number.isFinite(value));
  return {
    skipped_flattened_urdf_visual_count: Number(diagnostics.skipped_flattened_urdf_visual_count || 0),
    skippedFlattenedUrdfVisualCount: Number(diagnostics.skipped_flattened_urdf_visual_count || 0),
    assembled_hierarchy_rendered_mesh_count: Number(diagnostics.assembled_hierarchy_rendered_mesh_count || 0),
    assembledHierarchyRenderedMeshCount: Number(diagnostics.assembled_hierarchy_rendered_mesh_count || 0),
    rendered_fk_visual_count: Number(diagnostics.rendered_fk_visual_count || 0),
    renderedFkVisualCount: Number(diagnostics.rendered_fk_visual_count || 0),
    skipped_legacy_generated_urdf_count: Number(diagnostics.skipped_legacy_generated_urdf_count || 0),
    skippedLegacyGeneratedUrdfCount: Number(diagnostics.skipped_legacy_generated_urdf_count || 0),
    physical_assembly_root_count: assemblyBounds.count,
    physicalAssemblyRootCount: assemblyBounds.count,
    physical_assembly_bounds: assemblyBounds.bounds_json,
    physicalAssemblyBounds: assemblyBounds.bounds_json,
    physical_fit_included_robot_preview: Boolean(assemblyBounds.count && assemblyBounds.bounds_json),
    physicalFitIncludedRobotPreview: Boolean(assemblyBounds.count && assemblyBounds.bounds_json),
    final_physical_fit_bounds: finalFitBounds,
    finalPhysicalFitBounds: finalFitBounds,
    physical_renderable_count: rendered.length + assemblyBounds.count,
    physicalRenderableCount: rendered.length + assemblyBounds.count,
    visible_duplicate_generated_urdf_count: independentGenerated.length,
    visibleDuplicateGeneratedUrdfCount: independentGenerated.length,
    visible_tool0_fallback_count: visibleTool0Fallback.length,
    visibleTool0FallbackCount: visibleTool0Fallback.length,
    detached_robot_mesh_clusters: Number(diagnostics.detached_robot_mesh_clusters || 0),
    detached_robot_mesh_clusters_count: Number(diagnostics.detached_robot_mesh_clusters || 0),
    detachedRobotMeshClusters: Number(diagnostics.detached_robot_mesh_clusters || 0),
    max_wrist_tool0_to_gripper_base_distance_m: distances.length ? Math.max(...distances) : null,
    maxWristTool0ToGripperBaseDistanceM: distances.length ? Math.max(...distances) : null,
  };
}
function isUserFacingWarning(w) {
  if (!w) return false;
  if (w.code === 'camera_framing_blocker_excluded') return false;
  if (w.mesh_load_error || w.mesh_load_warning) return true;
  if (Array.isArray(w.missing_files) && w.missing_files.length) return true;
  if (w.missing_file) return true;
  const fields = [
    w.code, w.status, w.source, w.reason, w.message, w.mesh_status, w.mesh_load_status, w.render_status,
    w.visual_bounds_status, w.fallback_or_skip_reason,
  ];
  const text = fields.map(value => Array.isArray(value) ? value.join(' ') : String(value || '')).join(' ').toLowerCase();
  const realFailureTokens = [
    'mesh_load_error', 'mesh load error', 'mesh_load_warning', 'mesh load warning',
    'required_mesh_failed', 'required mesh failed', 'required_mesh_failed_debug_fallback',
    'invalid_dimension', 'invalid_dimensions', 'invalid dimensions', 'invalid_renderable_transform', 'invalid_mesh_local_transform',
    'loaded_mesh_bounds_invalid', 'loaded_mesh_collapsed', 'loaded_mesh_oversized',
    'visual_contract', 'visual bounds', 'visual_bounds_status',
    'unsafe_path', 'unsafe path', 'unsupported_format', 'unsupported format',
    'missing_file', 'missing file', 'missing files', 'not found',
    'loader_failure', 'loader failure', 'mesh_loader_failure', 'load_error', 'load error',
    'url_not_served', 'file_access_blocked', 'unresolved_package_uri',
  ];
  return realFailureTokens.some(token => text.includes(token));
}

const PRODUCT_DIAGNOSTIC_DEFINITIONS = Object.freeze({
  scene_failed: Object.freeze({ severity: 'error', title: 'Scene could not finish loading', action: 'Repair the first failed component, then reload Product View.' }),
  missing_mesh: Object.freeze({ severity: 'error', title: 'Mesh file is missing', action: 'Reimport the asset or restore its staged mesh file, then reload Product View.' }),
  unsupported_format: Object.freeze({ severity: 'error', title: 'Mesh format is not supported', action: 'Convert the model to STL, OBJ, or DAE, then import it again.' }),
  unresolved_package_uri: Object.freeze({ severity: 'error', title: 'ROS mesh path was not staged', action: 'Generate or refresh the scene so the package:// mesh is copied into the portable web assets.' }),
  invalid_scale: Object.freeze({ severity: 'error', title: 'Mesh scale or bounds are invalid', action: 'Check the asset units and authored scale. Use a positive finite scale, then reload the scene.' }),
  mesh_load_failed: Object.freeze({ severity: 'error', title: 'Physical asset failed to load', action: 'Check the staged file and loader details, then reimport or regenerate the asset.' }),
  collision: Object.freeze({ severity: 'warning', title: 'Placement collides with another object', action: 'Move the preview until it turns green before placing the asset.' }),
  generated_stale: Object.freeze({ severity: 'warning', title: 'Generated scene is out of date', action: 'Save the authored layout, then use Generate/Refresh Product View.' }),
});

function diagnosticSearchText(raw = {}) {
  return [
    raw.code, raw.status, raw.source, raw.reason, raw.message, raw.mesh_status,
    raw.mesh_load_status, raw.render_status, raw.visual_bounds_status,
    raw.fallback_reason, raw.fallback_or_skip_reason,
  ].map(value => Array.isArray(value) ? value.join(' ') : String(value || '')).join(' ').toLowerCase();
}
function productDiagnosticKind(raw = {}) {
  const code = String(raw.code || '').toLowerCase();
  const text = diagnosticSearchText(raw);
  if (code === 'optional_file_missing' || text.includes('optional input file is missing')) return '';
  if (/\bcollision\b/.test(text)) return 'collision';
  if (/generated.{0,30}(stale|out of date)|stale.{0,30}generated/.test(text)) return 'generated_stale';
  if (/unsupported[_ ]format|unsupported mesh format/.test(text)) return 'unsupported_format';
  if (/unresolved[_ ]package[_ ]uri|package uri (was not staged|must be resolved)|package:\/\//.test(text)) return 'unresolved_package_uri';
  if (/loaded[_ ]mesh[_ ](bounds[_ ])?(invalid|collapsed|oversized)|invalid[_ ](dimension|dimensions|scale|mesh[_ ]local[_ ]transform|renderable[_ ]transform)|visual[_ ]bounds/.test(text)) return 'invalid_scale';
  if (/missing[_ ]file|mesh file.{0,20}(missing|not found)|missing mesh/.test(text)) return 'missing_mesh';
  if (/required[_ ]mesh[_ ]failed|loader[_ ]failure|mesh[_ ]loader[_ ]failure|load[_ ]error|url[_ ]not[_ ]served|file[_ ]access[_ ]blocked|physical asset failed to load/.test(text)) return 'mesh_load_failed';
  if (/scene[_ ]failed|failed to load scene/.test(text)) return 'scene_failed';
  return '';
}
function productDiagnosticDetail(raw = {}, definition = {}) {
  const detail = String(raw.message || raw.reason || raw.mesh_load_error || raw.fallback_reason || definition.title || '').trim();
  return detail || definition.title || 'Product View reported a scene issue.';
}
function normalizeProductDiagnostic(raw = {}, overrideKind = '') {
  const kind = overrideKind || productDiagnosticKind(raw);
  const definition = PRODUCT_DIAGNOSTIC_DEFINITIONS[kind];
  if (!definition) return null;
  const itemId = String(raw.object_id || raw.item_id || raw.id || raw.link || raw.object_name || '').trim();
  const meshUri = String(raw.mesh_uri || raw.original_mesh_uri || raw.url || raw.mesh_load_url || '').trim();
  return {
    kind,
    severity: definition.severity,
    title: definition.title,
    detail: productDiagnosticDetail(raw, definition),
    action: definition.action,
    itemId,
    meshUri,
    code: String(raw.code || raw.status || raw.mesh_status || kind),
  };
}
function invalidAuthoredScale(item = {}) {
  const scale = item?.mesh_local_transform?.scale || item?.mesh_scale || item?.scale;
  return Array.isArray(scale) && (scale.length !== 3 || scale.some(value => !Number.isFinite(Number(value)) || Number(value) <= 0));
}
function collectProductDiagnostics() {
  const diagnostics = [];
  const add = diagnostic => { if (diagnostic) diagnostics.push(diagnostic); };
  const readinessFailure = state.web3dReadiness?.failure;
  if (state.web3dReadiness?.state === 'scene_failed' || state.editorError) {
    add(normalizeProductDiagnostic({
      code: 'scene_failed',
      reason: state.editorError || readinessFailure?.reason || 'A required scene component did not load.',
      item_id: readinessFailure?.item_id,
      url: readinessFailure?.url,
    }, 'scene_failed'));
  }
  const rawWarnings = asArray(state.sceneJson?.warnings).concat(asArray(state.sceneJson?.notes_warnings), asArray(state.runtimeWarnings));
  for (const warning of rawWarnings) {
    const kind = productDiagnosticKind(warning);
    if (kind && (isUserFacingWarning(warning) || kind === 'collision' || kind === 'generated_stale')) add(normalizeProductDiagnostic(warning, kind));
  }
  for (const rendered of statusCountedRenderables()) {
    const item = rendered?.item || {};
    const renderInfo = rendered?.renderInfo || item.renderInfo || {};
    const raw = {
      code: item.mesh_status || renderInfo.render_status,
      status: item.mesh_status || renderInfo.render_status,
      render_status: renderInfo.render_status,
      reason: item.mesh_load_error || renderInfo.fallback_reason,
      object_id: item.id,
      mesh_uri: renderInfo.mesh_uri || displayMeshUri(item),
      visual_bounds_status: item.visual_bounds_status,
    };
    if (isRequiredMeshFailureStatus(rendered) || isMissingOrFailedMeshStatus(item.mesh_status)) add(normalizeProductDiagnostic(raw));
    if ((item.visual_bounds_status && !['valid', 'corrected_by_local_unit_scale'].includes(item.visual_bounds_status)) || invalidAuthoredScale(item)) {
      add(normalizeProductDiagnostic({ ...raw, code: 'invalid_scale', reason: item.mesh_load_error || `Invalid visual bounds or scale (${item.visual_bounds_status || 'non-positive/non-finite scale'}).` }, 'invalid_scale'));
    }
  }
  if (state.placement?.armed && state.placement?.collision) {
    const colliders = asArray(state.placement.collidingOwnerIds);
    add(normalizeProductDiagnostic({
      code: 'collision',
      object_id: state.placement.asset?.id || state.placement.asset?.catalog_id || 'placement preview',
      reason: `Placement overlaps ${colliders.length ? colliders.join(', ') : 'another physical object'}.`,
    }, 'collision'));
  }
  const deduped = [];
  const keys = new Set();
  for (const diagnostic of diagnostics) {
    const key = [diagnostic.kind, diagnostic.itemId || diagnostic.meshUri || diagnostic.detail].join('|');
    if (keys.has(key)) continue;
    keys.add(key);
    deduped.push(diagnostic);
  }
  return deduped;
}
function sceneHealthModel() {
  const diagnostics = collectProductDiagnostics();
  const lifecycle = state.editorError ? 'scene_failed' : (state.web3dReadiness?.state || 'booting');
  const pending = pendingRequiredLoads();
  const errorCount = diagnostics.filter(item => item.severity === 'error').length;
  const warningCount = diagnostics.filter(item => item.severity === 'warning').length;
  if (lifecycle === 'scene_failed' || errorCount) {
    return { state: 'failed', title: 'Scene needs attention', summary: `${errorCount || 1} blocking issue${(errorCount || 1) === 1 ? '' : 's'} · open for the fix`, diagnostics, errorCount, warningCount };
  }
  if (lifecycle === 'booting' || lifecycle === 'scene_loading') {
    const waiting = pending.length ? `${pending.length} required component${pending.length === 1 ? '' : 's'} remaining` : 'Checking robot, tool, environment, and camera';
    return { state: 'loading', title: state.sceneJsonLoaded ? 'Loading physical scene' : 'Opening scene', summary: waiting, diagnostics, errorCount, warningCount };
  }
  if (warningCount) return { state: 'warning', title: 'Scene ready with warnings', summary: `${warningCount} item${warningCount === 1 ? '' : 's'} to review`, diagnostics, errorCount, warningCount };
  const physicalCount = renderedPhysicalItemCount();
  return { state: 'ready', title: 'Scene ready', summary: `${physicalCount} physical item${physicalCount === 1 ? '' : 's'} loaded · safe to edit`, diagnostics, errorCount, warningCount };
}
function renderSceneHealth(model = sceneHealthModel()) {
  if (!el.sceneHealth) return model;
  el.sceneHealth.classList?.remove?.('health-loading', 'health-ready', 'health-warning', 'health-failed');
  el.sceneHealth.classList?.add?.(`health-${model.state}`);
  if (el.sceneHealthTitle) el.sceneHealthTitle.textContent = model.title;
  if (el.sceneHealthSummary) el.sceneHealthSummary.textContent = model.summary;
  const issueCount = model.diagnostics.length;
  if (el.sceneHealthCount) {
    el.sceneHealthCount.hidden = issueCount === 0;
    el.sceneHealthCount.textContent = String(issueCount);
  }
  if (el.sceneHealthItems) {
    el.sceneHealthItems.innerHTML = issueCount ? model.diagnostics.map(diagnostic => {
      const context = [diagnostic.itemId ? `Item: ${diagnostic.itemId}` : '', diagnostic.meshUri ? `Mesh: ${diagnostic.meshUri}` : ''].filter(Boolean).join(' · ');
      return `<article class="health-item health-item-${escapeHtml(diagnostic.severity)}"><strong>${escapeHtml(diagnostic.title)}</strong><span>${escapeHtml(diagnostic.detail)}</span><small>${context ? `${escapeHtml(context)}<br>` : ''}<b>Next:</b> ${escapeHtml(diagnostic.action)}</small></article>`;
    }).join('') : `<div class="health-empty">${model.state === 'ready' ? 'All required physical components loaded. No action is needed.' : 'Product View is checking required physical components.'}</div>`;
  }
  const navigationKey = web3dNavigationKey();
  if (model.state === 'failed' && state.healthAutoOpenedNavigationKey !== navigationKey) {
    el.sceneHealth.open = true;
    state.healthAutoOpenedNavigationKey = navigationKey;
  }
  return model;
}

function vector3ToDiagnostics(value) {
  const xyz = finiteXyzArrayFromVector(value);
  return xyz ? { x: xyz[0], y: xyz[1], z: xyz[2] } : null;
}
function quaternionToDiagnostics(q) { return q ? { x: q.x, y: q.y, z: q.z, w: q.w } : null; }
function eulerToDiagnostics(e) { return e ? { x: e.x, y: e.y, z: e.z, order: e.order || 'XYZ' } : null; }
function worldUpDiagnosticsForObject(object) {
  if (!object || !THREE?.Vector3) return null;
  object.updateMatrixWorld(true);
  const up = new THREE.Vector3(0, 0, 1).transformDirection(object.matrixWorld).normalize();
  return vector3ToDiagnostics(up);
}
function box3DiagnosticsForObject(object) {
  if (!object || !THREE?.Box3) return { center: null, size: null };
  object.updateMatrixWorld(true);
  const box = finiteBox3(new THREE.Box3().setFromObject(object));
  if (!box) return { center: null, size: null };
  const center = new THREE.Vector3();
  const size = new THREE.Vector3();
  box.getCenter(center);
  box.getSize(size);
  return {
    center: vector3ToDiagnostics(center),
    size: vector3ToDiagnostics(size),
  };
}

function expandedUrdfExpectedVisualSet(json = state.sceneJson || {}) {
  const preview = json?.robot_preview || {};
  if (!isExpandedUrdfRobotPreview(preview)) return null;
  const robotVisuals = asArray(preview.expected_robot_visual_links || preview.expectedRobotVisualLinks).map(link => String(link || '').trim()).filter(Boolean);
  const toolVisuals = asArray(preview.expected_tool_visual_links || preview.expectedToolVisualLinks).map(link => String(link || '').trim()).filter(Boolean);
  return {
    scene_id: String(json?.scene?.id || json?.scene_id || '').trim(),
    sceneId: String(json?.scene?.id || json?.scene_id || '').trim(),
    robot_visuals: robotVisuals,
    robotVisuals,
    tool_visuals: toolVisuals,
    toolVisuals,
    table_visuals: ['workbench_support_surface'],
    tableVisuals: ['workbench_support_surface'],
    camera_visuals: ['configured_camera'],
    cameraVisuals: ['configured_camera'],
  };
}
function countBy(values) {
  const counts = {};
  for (const value of values || []) counts[value] = (counts[value] || 0) + 1;
  return counts;
}
function expandedUrdfVisualIdentity(sceneId, robotInstanceId, visual) {
  const link = String(visual?.link_name || visual?.linkName || '').trim();
  if (!link) return '';
  const visualName = String(visual?.visual_name || visual?.visualName || visual?.object_name || visual?.objectName || '').trim();
  const visualIndex = visual?.visual_index ?? visual?.visualIndex;
  const visualKey = visualIndex !== undefined && visualIndex !== null && String(visualIndex).trim() !== ''
    ? `index:${String(visualIndex).trim()}`
    : `name:${visualName || 'visual'}`;
  return [sceneId || '<scene>', robotInstanceId || '<robot>', link, visualKey].join('|');
}
function renderedPhysicalVisualIdentity(sceneId, entry) {
  const category = readinessCategoryForItem(entry);
  const link = String(entry?.link_name || entry?.link || entry?.frame || entry?.id || category || '').trim();
  if (!link) return '';
  const visualName = String(entry?.visual_name || entry?.visualName || entry?.object_name || entry?.objectName || entry?.display_name || entry?.id || '').trim();
  const visualIndex = entry?.visual_index ?? entry?.visualIndex;
  const visualKey = visualIndex !== undefined && visualIndex !== null && String(visualIndex).trim() !== ''
    ? `index:${String(visualIndex).trim()}`
    : `name:${visualName || 'physical'}`;
  return [sceneId || '<scene>', String(entry?.robot_instance_id || entry?.robotInstanceId || entry?.source_instance || entry?.sourceInstance || 'physical').trim() || 'physical', link, visualKey].join('|');
}
function dedupeByStableIdentity(records, identityFn) {
  const byIdentity = new Map();
  const duplicates = [];
  for (const record of records || []) {
    const identity = identityFn(record);
    if (!identity) continue;
    if (byIdentity.has(identity)) duplicates.push(identity);
    else byIdentity.set(identity, record);
  }
  return { records: Array.from(byIdentity.values()), duplicateIdentities: Array.from(new Set(duplicates)) };
}
function isSuccessfulPhysicalVisualDiagnostic(entry) {
  if (!entry || entry.debug_overlay || entry.debugOverlay) return false;
  if (entry.mesh_loaded !== true && entry.meshLoaded !== true && entry.render_status !== 'mesh_loaded' && entry.renderStatus !== 'mesh_loaded') return false;
  return Boolean(readinessCategoryForItem(entry));
}
function failedRequiredMeshUrlFromEntry(entry) {
  const url = entry?.mesh_uri || entry?.meshUri || entry?.url || entry?.source_url || entry?.sourceUrl || '';
  return String(url || '').trim();
}
function expandedUrdfVisualReadinessDiagnostics() {
  const required = expandedUrdfExpectedVisualSet();
  if (!required) return null;
  const rendererDiagnostics = state.robotUrdfPreviewDiagnostics || {};
  const sceneId = required.scene_id || required.sceneId || '';
  const preview = state.sceneJson?.robot_preview || {};
  const robotInstanceId = String(preview.robot_instance_id || preview.robotInstanceId || preview.instance_id || preview.instanceId || preview.id || 'expanded_urdf_robot').trim();
  const rawUrdfVisuals = asArray(rendererDiagnostics.robot_visual_wrapper_world_matrices);
  const urdfDedupe = dedupeByStableIdentity(rawUrdfVisuals, visual => expandedUrdfVisualIdentity(sceneId, robotInstanceId, visual));
  const urdfVisuals = urdfDedupe.records;
  const urdfLinksWithLoadedVisuals = new Set(urdfVisuals.map(visual => String(visual?.link_name || visual?.linkName || '').trim()).filter(Boolean));
  const sceneDiagnostics = collectRenderedMeshDiagnostics();
  const physicalDiagnostics = dedupeByStableIdentity(sceneDiagnostics.filter(isSuccessfulPhysicalVisualDiagnostic), entry => renderedPhysicalVisualIdentity(sceneId, entry));
  const categoryCounts = countBy(physicalDiagnostics.records.map(item => readinessCategoryForItem(item)).filter(Boolean));
  const rendererLifecycle = String(rendererDiagnostics.robot_preview_lifecycle_state || rendererDiagnostics.robotPreviewLifecycleState || '');
  const rendererLoaded = rendererDiagnostics.robot_preview_loaded === true || rendererDiagnostics.robotPreviewLoaded === true;
  const rendererExpectedVisualCount = Number(rendererDiagnostics.robot_expected_visual_count ?? rendererDiagnostics.robotExpectedVisualCount ?? 0) || 0;
  const rendererCompletedVisualCount = Number(rendererDiagnostics.robot_completed_visual_count ?? rendererDiagnostics.robotCompletedVisualCount ?? 0) || 0;
  const rendererLoadedVisualCount = Number(rendererDiagnostics.robot_loaded_visual_count ?? rendererDiagnostics.robotLoadedVisualCount ?? 0) || 0;
  const rendererFailedVisualCount = Number(rendererDiagnostics.robot_failed_visual_count ?? rendererDiagnostics.robotFailedVisualCount ?? 0) || 0;
  const rendererMissingMeshes = asArray(rendererDiagnostics.robot_missing_meshes || rendererDiagnostics.robotMissingMeshes);
  const rendererMeshCallbacksComplete = rendererDiagnostics.robot_mesh_callbacks_complete === true || rendererDiagnostics.robotMeshCallbacksComplete === true;
  const rendererSuccessfulCallbackAccounting = rendererMeshCallbacksComplete
    && rendererExpectedVisualCount > 0
    && rendererCompletedVisualCount === rendererExpectedVisualCount
    && rendererLoadedVisualCount === rendererExpectedVisualCount
    && rendererFailedVisualCount === 0
    && rendererMissingMeshes.length === 0;
  const rendererReady = (rendererLifecycle === 'ready' && rendererLoaded && rendererFailedVisualCount === 0 && rendererMissingMeshes.length === 0)
    || rendererSuccessfulCallbackAccounting;
  const missingRobot = rendererReady
    ? []
    : asArray(rendererDiagnostics.robot_missing_required_robot_visual_links || rendererDiagnostics.robotMissingRequiredRobotVisualLinks).map(value => String(value || '').trim()).filter(Boolean);
  const missingTool = rendererReady
    ? []
    : asArray(rendererDiagnostics.robot_missing_required_tool_visual_links || rendererDiagnostics.robotMissingRequiredToolVisualLinks).map(value => String(value || '').trim()).filter(Boolean);
  if (!rendererReady && missingRobot.length === 0 && missingTool.length === 0 && !rendererLoaded) {
    for (const link of required.robot_visuals) {
      if (!urdfLinksWithLoadedVisuals.has(link)) missingRobot.push(link);
    }
    for (const link of required.tool_visuals) {
      if (!urdfLinksWithLoadedVisuals.has(link)) missingTool.push(link);
    }
  }
  const failedLinks = [];
  const failedMeshUrls = [];
  for (const detail of rendererMissingMeshes) {
    const text = String(detail || '').trim();
    if (text) failedMeshUrls.push(text.split(': ')[0] || text);
  }
  if (rendererFailedVisualCount > 0) failedLinks.push('expanded_urdf_loader');
  const missing = Array.from(new Set(missingRobot.concat(missingTool).filter(Boolean)));
  const failed = Array.from(new Set(failedLinks.filter(Boolean)));
  const duplicatePhysicalIdentities = Array.from(new Set(urdfDedupe.duplicateIdentities.concat(physicalDiagnostics.duplicateIdentities)));
  const requiredVisualReady = rendererReady || (missing.length === 0 && failed.length === 0);
  return {
    expanded_urdf_expected_visual_set: required,
    expandedUrdfExpectedVisualSet: required,
    expanded_urdf_required_visual_counts: { ...categoryCounts },
    expandedUrdfRequiredVisualCounts: { ...categoryCounts },
    expanded_urdf_loaded_visual_link_counts: countBy(urdfVisuals.map(visual => String(visual?.link_name || visual?.linkName || '').trim()).filter(Boolean)),
    expandedUrdfLoadedVisualLinkCounts: countBy(urdfVisuals.map(visual => String(visual?.link_name || visual?.linkName || '').trim()).filter(Boolean)),
    robot_preview_lifecycle_state: rendererLifecycle,
    robotPreviewLifecycleState: rendererLifecycle,
    robot_preview_loaded: rendererLoaded,
    robotPreviewLoaded: rendererLoaded,
    robot_expected_visual_count: rendererExpectedVisualCount,
    robotExpectedVisualCount: rendererExpectedVisualCount,
    robot_completed_visual_count: rendererCompletedVisualCount,
    robotCompletedVisualCount: rendererCompletedVisualCount,
    robot_loaded_visual_count: rendererLoadedVisualCount,
    robotLoadedVisualCount: rendererLoadedVisualCount,
    robot_failed_visual_count: rendererFailedVisualCount,
    robotFailedVisualCount: rendererFailedVisualCount,
    robot_mesh_callbacks_complete: rendererMeshCallbacksComplete,
    robotMeshCallbacksComplete: rendererMeshCallbacksComplete,
    robot_successful_callback_accounting: rendererSuccessfulCallbackAccounting,
    robotSuccessfulCallbackAccounting: rendererSuccessfulCallbackAccounting,
    robot_missing_meshes: rendererMissingMeshes,
    robotMissingMeshes: rendererMissingMeshes,
    robot_missing_required_robot_visual_links: missingRobot,
    robotMissingRequiredRobotVisualLinks: missingRobot,
    robot_missing_required_tool_visual_links: missingTool,
    robotMissingRequiredToolVisualLinks: missingTool,
    missing_required_robot_visuals: missingRobot,
    missingRequiredRobotVisuals: missingRobot,
    missing_required_tool_visuals: missingTool,
    missingRequiredToolVisuals: missingTool,
    missing_required_visuals: missing,
    missingRequiredVisuals: missing,
    duplicate_required_visuals: duplicatePhysicalIdentities,
    duplicateRequiredVisuals: duplicatePhysicalIdentities,
    duplicate_physical_visual_identities: duplicatePhysicalIdentities,
    duplicatePhysicalVisualIdentities: duplicatePhysicalIdentities,
    failed_required_visuals: failed,
    failedRequiredVisuals: failed,
    failed_required_links: failed,
    failedRequiredLinks: failed,
    failed_mesh_urls: Array.from(new Set(failedMeshUrls)),
    failedMeshUrls: Array.from(new Set(failedMeshUrls)),
    required_visual_ready: requiredVisualReady,
    requiredVisualReady: requiredVisualReady,
  };
}
function failIfExpandedUrdfExpectedVisualSetInvalid() {
  const lifecycle = String(state.robotUrdfPreviewDiagnostics?.robot_preview_lifecycle_state || state.robotUrdfPreviewDiagnostics?.robotPreviewLifecycleState || '');
  if (expandedUrdfExpectedVisualSet() && lifecycle !== 'ready' && lifecycle !== 'failed') return false;
  const diagnostics = expandedUrdfVisualReadinessDiagnostics();
  if (!diagnostics || diagnostics.required_visual_ready) return false;
  const requiredCategory = diagnostics.robot_missing_required_tool_visual_links?.length ? 'attached_tool_gripper' : 'robot_arm';
  const reason = diagnostics.robot_preview_loaded
    ? 'expanded URDF renderer reported required robot/tool visual failure'
    : 'expanded URDF expected robot/tool visuals are missing or failed';
  emitWeb3dReadinessState('scene_failed', {
    required_category: requiredCategory,
    reason,
    ...diagnostics,
  });
  return true;
}

function collectRenderedMeshDiagnostics() {
  if (!THREE?.Vector3) return [];
  state.three?.scene?.updateMatrixWorld?.(true);
  const diagnostics = [];
  for (const rendered of state.objects || []) {
    const item = rendered?.item || {};
    const category = meshContractCategoryOf(item);
    if (!rendered?.object3d || (!isGeneratedUrdfItem(item) && !['table', 'environment', 'camera', 'tool', 'robot'].includes(category))) continue;
    if (typeof rendered.object3d.updateMatrixWorld !== 'function') continue;
    if (typeof rendered.object3d.getWorldPosition !== 'function') continue;
    if (typeof rendered.object3d.getWorldQuaternion !== 'function') continue;
    rendered.object3d.updateMatrixWorld(true);
    rendered.meshObject?.updateMatrixWorld?.(true);
    rendered.loadedMeshObject?.updateMatrixWorld?.(true);

    const linkFrameWorldPosition = new THREE.Vector3();
    rendered.object3d.getWorldPosition(linkFrameWorldPosition);

    let visualWrapperWorldPosition = null;
    if (rendered.meshObject?.getWorldPosition) {
      const visualWrapperWorld = new THREE.Vector3();
      rendered.meshObject.getWorldPosition(visualWrapperWorld);
      visualWrapperWorldPosition = vector3ToDiagnostics(visualWrapperWorld);
    }

    const boundsSource = rendered.loadedMeshObject || rendered.meshObject || rendered.object3d;
    const bounds = box3DiagnosticsForObject(boundsSource);
    const itemBounds = box3DiagnosticsForObject(rendered.object3d);
    const worldQuaternion = new THREE.Quaternion();
    rendered.object3d.getWorldQuaternion(worldQuaternion);
    const worldEuler = new THREE.Euler().setFromQuaternion(worldQuaternion, 'XYZ');
    const inferredUpAxis = worldUpDiagnosticsForObject(rendered.object3d);
    const expectedDimensions = expectedDimensionsOf(item);
    const meshLocal = meshLocalTransformOf(item);
    const renderStatus = rendered.renderInfo?.render_status || item.renderInfo?.render_status || item.mesh_status || '';
    diagnostics.push({
      id: item.id || '',
      object_id: item.id || item.object_name || item.name || '',
      object_name: item.object_name || item.name || item.display_name || item.label || item.id || '',
      category,
      link: item.link || item.object_name || item.visual || '',
      link_name: item.link_name || item.link || item.object_name || '',
      frame: item.frame || item.frame_id || item.link || item.link_name || '',
      display_name: item.display_name || item.label || item.name || item.object_name || item.link || item.id || '',
      source_layer: item.source_layer || '',
      active_visual_source: item.active_visual_source || '',
      role: item.role || '',
      status: item.status || '',
      support_surface_display_type: supportSurfaceDisplayType(item),
      supportSurfaceDisplayType: supportSurfaceDisplayType(item),
      ...supportSurfaceMetadata(item),
      render_status: renderStatus,
      renderStatus,
      mesh_loaded: renderStatus === 'mesh_loaded',
      meshLoaded: renderStatus === 'mesh_loaded',
      fallback_visible: Boolean(rendered.fallback?.visible),
      fallbackVisible: Boolean(rendered.fallback?.visible),
      exclude_from_fit_bounds: Boolean(item.exclude_from_fit_bounds || rendered.object3d.userData?.exclude_from_fit_bounds),
      excludeFromFitBounds: Boolean(item.exclude_from_fit_bounds || rendered.object3d.userData?.exclude_from_fit_bounds),
      debug_overlay: Boolean(isDebugOverlayItem(item)),
      debugOverlay: Boolean(isDebugOverlayItem(item)),
      visual_bounds_status: item.visual_bounds_status || '',
      visualBoundsStatus: item.visual_bounds_status || '',
      workcell_web_render_pose_mode: effectiveWorkcellWebRenderPoseMode(item),
      workcellWebRenderPoseMode: effectiveWorkcellWebRenderPoseMode(item),
      exported_workcell_web_render_pose_mode: item.workcell_web_render_pose_mode || '',
      exportedWorkcellWebRenderPoseMode: item.workcell_web_render_pose_mode || '',
      baked_world_visual_pose: item.baked_world_visual_pose || null,
      bakedWorldVisualPose: item.baked_world_visual_pose || null,
      expected_visual_pose: item.expected_visual_pose || null,
      expectedVisualPose: item.expected_visual_pose || null,
      final_transform: item.final_transform || null,
      finalTransform: item.final_transform || null,
      world_from_visual: item.world_from_visual || null,
      worldFromVisual: item.world_from_visual || null,
      mesh_unit_correction: item.mesh_unit_correction || null,
      meshUnitCorrection: item.mesh_unit_correction || null,
      expected_dimensions_m: expectedDimensions ? vector3ToDiagnostics(expectedDimensions) : null,
      expectedDimensionsM: expectedDimensions ? vector3ToDiagnostics(expectedDimensions) : null,
      mesh_local_scale: vector3ToDiagnostics(meshLocal.scale),
      meshLocalScale: vector3ToDiagnostics(meshLocal.scale),
      mesh_uri: rendered.renderInfo?.mesh_uri || displayMeshUri(item),
      fallback_reason: rendered.renderInfo?.fallback_reason || '',
      linkFrameWorldPosition: vector3ToDiagnostics(linkFrameWorldPosition),
      link_frame_world_position: vector3ToDiagnostics(linkFrameWorldPosition),
      visualWrapperWorldPosition,
      visual_wrapper_world_position: visualWrapperWorldPosition,
      loadedMeshBoundingBoxCenter: bounds.center,
      loaded_mesh_bounding_box_center: bounds.center,
      loadedMeshBoundingBoxSize: bounds.size,
      loaded_mesh_bounding_box_size: bounds.size,
      boundingBoxCenter: bounds.center,
      bounding_box_center: bounds.center,
      boundingBoxSize: bounds.size,
      bounding_box_size: bounds.size,
      itemBoundingBoxCenter: itemBounds.center,
      item_bounding_box_center: itemBounds.center,
      itemBoundingBoxSize: itemBounds.size,
      item_bounding_box_size: itemBounds.size,
      worldQuaternion: quaternionToDiagnostics(worldQuaternion),
      world_quaternion: quaternionToDiagnostics(worldQuaternion),
      worldEuler: eulerToDiagnostics(worldEuler),
      world_euler: eulerToDiagnostics(worldEuler),
      inferredUpAxis: inferredUpAxis,
      inferred_up_axis: inferredUpAxis,
      inferredNormal: inferredUpAxis,
      inferred_normal: inferredUpAxis,
    });
  }
  const expectedOrder = new Map(EXPECTED_GENERATED_URDF_DIAGNOSTIC_LINKS.map((name, index) => [name, index]));
  diagnostics.sort((a, b) => {
    const aKey = a.link_name || a.link || a.frame || a.id;
    const bKey = b.link_name || b.link || b.frame || b.id;
    const aIndex = expectedOrder.has(aKey) ? expectedOrder.get(aKey) : Number.MAX_SAFE_INTEGER;
    const bIndex = expectedOrder.has(bKey) ? expectedOrder.get(bKey) : Number.MAX_SAFE_INTEGER;
    if (aIndex !== bIndex) return aIndex - bIndex;
    return String(aKey).localeCompare(String(bKey));
  });
  return diagnostics;
}

function updateViewerStatus() {
  const summary = computeSceneSummary();
  const productHealth = sceneHealthModel();
  const selectionDiagnostics = currentSelectionDiagnostics();
  const warnings = asArray(state.sceneJson?.warnings).concat(asArray(state.sceneJson?.notes_warnings), asArray(state.runtimeWarnings));
  const resolvedFrameStatus = buildResolvedFrameStatus();
  const renderedMeshDiagnostics = collectRenderedMeshDiagnostics();
  const frameDiagnostics = collectFrameDiagnostics();
  const renderedObjectStatuses = statusCountedRenderables().map(obj => {
    const item = obj?.item || {};
    const renderStatus = obj?.renderInfo?.render_status || item?.renderInfo?.render_status || item?.mesh_status || '';
    return {
      id: item.id || '',
      display_name: item.display_name || item.label || item.name || item.object_name || item.link || item.id || '',
      category: meshContractCategoryOf(item),
      render_status: renderStatus,
      renderStatus,
      mesh_loaded: renderStatus === 'mesh_loaded',
      meshLoaded: renderStatus === 'mesh_loaded',
      fallback_visible: Boolean(obj?.fallback?.visible),
      fallbackVisible: Boolean(obj?.fallback?.visible),
      mesh_uri: obj?.renderInfo?.mesh_uri || displayMeshUri(item),
      fallback_reason: obj?.renderInfo?.fallback_reason || '',
      visual_bounds_status: item.visual_bounds_status || '',
      visualBoundsStatus: item.visual_bounds_status || '',
      support_surface_kind: item.support_surface_kind || '',
      supportSurfaceKind: item.supportSurfaceKind || item.support_surface_kind || '',
      support_surface_display_type: supportSurfaceDisplayType(item),
      supportSurfaceDisplayType: supportSurfaceDisplayType(item),
      top_surface_z_m: item.top_surface_z_m ?? null,
      topSurfaceZM: item.topSurfaceZM ?? item.top_surface_z_m ?? null,
      support_surface_height_m: item.support_surface_height_m ?? null,
      supportSurfaceHeightM: item.supportSurfaceHeightM ?? item.support_surface_height_m ?? null,
      expected_support_footprint_m: item.expected_support_footprint_m || null,
      expectedSupportFootprintM: item.expectedSupportFootprintM || item.expected_support_footprint_m || null,
    };
  });
  const assemblyRenderDiagnostics = collectAssemblyRenderDiagnostics();
  const expandedUrdfVisualDiagnostics = expandedUrdfVisualReadinessDiagnostics() || {};
  window.__WORKCELL_VIEWER_STATUS__ = {
    viewer_boot_state: state.web3dReadiness?.state || 'booting',
    viewerBootState: state.web3dReadiness?.state || 'booting',
    web3d_readiness_state: state.web3dReadiness?.state || 'booting',
    web3dReadinessState: state.web3dReadiness?.state || 'booting',
    failed_stage: state.web3dReadiness?.state === 'scene_failed' ? 'scene_failed' : '',
    failedStage: state.web3dReadiness?.state === 'scene_failed' ? 'scene_failed' : '',
    fatal_error: state.web3dReadiness?.failure?.reason || '',
    fatalError: state.web3dReadiness?.failure?.reason || '',
    fatal_stack: '',
    fatalStack: '',
    source_web_scene_file: state.sourceWebSceneFile || '',
    sourceWebSceneFile: state.sourceWebSceneFile || '',
    scene_json_loaded: Boolean(state.sceneJsonLoaded),
    sceneJsonLoaded: Boolean(state.sceneJsonLoaded),
    scene_name: summary.sceneName,
    sceneName: summary.sceneName,
    selectionOwnerRegistryCount: selectionDiagnostics.selectionOwnerRegistryCount,
    selectionOwnerIds: selectionDiagnostics.selectionOwnerIds,
    selectedItemId: selectionDiagnostics.selectedItemId,
    uiSelectionItemId: selectionDiagnostics.uiSelectionItemId,
    selectedLinkName: selectionDiagnostics.selectedLinkName,
    uiSelectionResolution: selectionDiagnostics.uiSelectionResolution,
    pickRecordSource: selectionDiagnostics.pickRecordSource,
    last_failed_canvas_pick_diagnostic: state.lastFailedCanvasPickDiagnostic,
    lastFailedCanvasPickDiagnostic: state.lastFailedCanvasPickDiagnostic,
    renderable_count: summary.renderableCount,
    renderableCount: summary.renderableCount,
    mesh_loaded_count: summary.meshLoadedCount,
    meshLoadedCount: summary.meshLoadedCount,
    expected_scene_mesh_loaded_count: summary.expectedMeshLoadedCount,
    expectedSceneMeshLoadedCount: summary.expectedMeshLoadedCount,
    required_mesh_failed_count: statusCountedRenderables().filter(isRequiredMeshFailureStatus).length,
    requiredMeshFailedCount: statusCountedRenderables().filter(isRequiredMeshFailureStatus).length,
    fallback_count: summary.fallbackCount,
    fallbackCount: summary.fallbackCount,
    runtime_warnings: warnings,
    runtimeWarnings: warnings,
    scene_health_state: productHealth.state,
    sceneHealthState: productHealth.state,
    product_diagnostics: productHealth.diagnostics,
    productDiagnostics: productHealth.diagnostics,
    product_diagnostic_error_count: productHealth.errorCount,
    productDiagnosticErrorCount: productHealth.errorCount,
    product_diagnostic_warning_count: productHealth.warningCount,
    productDiagnosticWarningCount: productHealth.warningCount,
    resolvedFramePositions: resolvedFrameStatus.resolvedFramePositions,
    resolved_frame_positions: resolvedFrameStatus.resolved_frame_positions,
    viewer_resolved_distances_m: resolvedFrameStatus.viewer_resolved_distances_m,
    resolvedFrameDistancesM: resolvedFrameStatus.resolvedFrameDistancesM,
    renderedMeshDiagnostics,
    rendered_mesh_diagnostics: renderedMeshDiagnostics,
    frameDiagnostics,
    frame_diagnostics: frameDiagnostics,
    renderedObjectStatuses,
    rendered_object_statuses: renderedObjectStatuses,
    robot_transform_source: state.robotAssemblyDiagnostics?.some(d => d.robot_transform_source === 'ros_tf_verified_urdf_fk') ? 'ros_tf_verified_urdf_fk' : (state.robotAssemblyDiagnostics?.some(d => d.robot_transform_source === 'expanded_urdf_joint_tree') ? 'expanded_urdf_joint_tree' : ''),
    robotTransformSource: state.robotAssemblyDiagnostics?.some(d => d.robot_transform_source === 'ros_tf_verified_urdf_fk') ? 'ros_tf_verified_urdf_fk' : (state.robotAssemblyDiagnostics?.some(d => d.robot_transform_source === 'expanded_urdf_joint_tree') ? 'expanded_urdf_joint_tree' : ''),
    robot_render_mode: state.robotUrdfPreviewDiagnostics?.robot_render_mode || (state.robotAssemblyDiagnostics?.some(d => d.robot_render_mode === 'verified_urdf_fk_visual_world_pose') ? 'verified_urdf_fk_visual_world_pose' : (state.robotAssemblyDiagnostics?.some(d => d.robot_render_mode === 'urdf_fk_visual_world_pose') ? 'urdf_fk_visual_world_pose' : (state.robotAssemblyDiagnostics?.length ? 'assembled_urdf_hierarchy' : ''))),
    robotRenderMode: state.robotUrdfPreviewDiagnostics?.robot_render_mode || (state.robotAssemblyDiagnostics?.some(d => d.robot_render_mode === 'verified_urdf_fk_visual_world_pose') ? 'verified_urdf_fk_visual_world_pose' : (state.robotAssemblyDiagnostics?.some(d => d.robot_render_mode === 'urdf_fk_visual_world_pose') ? 'urdf_fk_visual_world_pose' : (state.robotAssemblyDiagnostics?.length ? 'assembled_urdf_hierarchy' : ''))),
    ...state.robotUrdfPreviewDiagnostics,
    robot_hierarchy_diagnostics: state.robotAssemblyDiagnostics || [],
    robotHierarchyDiagnostics: state.robotAssemblyDiagnostics || [],
    robot_hierarchy_links: state.robotUrdfPreviewDiagnostics?.robot_hierarchy_links || Array.from(new Set((state.robotAssemblyDiagnostics || []).flatMap(d => d.robot_hierarchy_links || []))),
    robotHierarchyLinks: state.robotUrdfPreviewDiagnostics?.robot_hierarchy_links || Array.from(new Set((state.robotAssemblyDiagnostics || []).flatMap(d => d.robot_hierarchy_links || []))),
    robot_hierarchy_missing_links: state.robotUrdfPreviewDiagnostics?.robot_hierarchy_missing_links || Array.from(new Set((state.robotAssemblyDiagnostics || []).flatMap(d => d.robot_hierarchy_missing_links || []))),
    robot_hierarchy_missing_parents: Array.from(new Set((state.robotAssemblyDiagnostics || []).flatMap(d => d.robot_hierarchy_missing_parents || []))),
    robot_hierarchy_mesh_count: (state.robotAssemblyDiagnostics || []).reduce((total, d) => total + Number(d.robot_hierarchy_mesh_count || 0), 0),
    robotHierarchyMeshCount: (state.robotAssemblyDiagnostics || []).reduce((total, d) => total + Number(d.robot_hierarchy_mesh_count || 0), 0),
    ...assemblyRenderDiagnostics,
    ...expandedUrdfVisualDiagnostics,
    required_physical_categories: state.web3dReadiness?.required || {},
    requiredPhysicalCategories: state.web3dReadiness?.required || {},
    pending_required_loads: pendingRequiredLoads(),
    pendingRequiredLoads: pendingRequiredLoads(),
    ...structuredWeb3dReadinessFields(state.web3dReadiness?.state || 'booting'),
    final_failed_url: state.web3dReadiness?.failure?.url || state.web3dReadiness?.failure?.final_failed_url || '',
    finalFailedUrl: state.web3dReadiness?.failure?.url || state.web3dReadiness?.failure?.finalFailedUrl || '',
    final_failed_link: state.web3dReadiness?.failure?.link || state.web3dReadiness?.failure?.link_name || '',
    finalFailedLink: state.web3dReadiness?.failure?.link || state.web3dReadiness?.failure?.linkName || '',
    readiness_failure: state.web3dReadiness?.failure || null,
    readinessFailure: state.web3dReadiness?.failure || null,
  };
  renderSceneHealth(productHealth);
  return window.__WORKCELL_VIEWER_STATUS__;
}
function renderSceneSummary() {
  const summary = updateViewerStatus();
  if (!el.summary) return;
  el.summary.classList.toggle('empty', !state.sceneJson);
  const fields = {
    'scene-name': summary.sceneName,
    'renderable-count': summary.renderableCount,
    'mesh-loaded-count': `${summary.meshLoadedCount}${summary.meshVisuallyInvalidCount ? ` (${summary.meshVisuallyInvalidCount} visually invalid)` : ''}${summary.meshUnitScaleCorrectedCount ? `, ${summary.meshUnitScaleCorrectedCount} unit-corrected` : ''}`,
    'fallback-count': summary.fallbackCount,
    'mesh-failed-count': summary.meshFailedCount,
    'generated-locked-count': summary.generatedLockedCount,
    'editable-count': summary.editableCount,
    'robot-preview-mode': summary.robotPreviewMode,
  };
  for (const [name, value] of Object.entries(fields)) {
    const node = el.summary.querySelector(`[data-summary-field="${name}"]`);
    if (node) node.textContent = String(value);
  }
}

function pushEditorEvent(type, payload = {}) {
  state.editorEvents.push({ type, timestamp: new Date().toISOString(), ...payload });
  if (state.editorEvents.length > 100) state.editorEvents.splice(0, state.editorEvents.length - 100);
}
const UI_SELECTION_REFERENCE_FIELDS = Object.freeze([
  'canonical_scene_item_id',
  'canonical_item_id',
  'layout_item_ref',
  'authored_item_id',
  'scene_item_id',
  'object_ref',
  'support_surface_ref',
  'camera_id',
]);
const SELECTION_LINK_IDENTITY_FIELDS = Object.freeze([
  'link_name', 'link', 'canonical_link_name', 'object_name', 'final_render_link',
]);
function exactSelectionLinkName(item) {
  for (const field of SELECTION_LINK_IDENTITY_FIELDS) {
    const value = String(item?.[field] || '').trim();
    if (value) return value;
  }
  return '';
}
function rebuildSelectionIdentityIndex(sceneJson = state.sceneJson || {}) {
  const items = collectItems(sceneJson);
  const selectionOwners = asArray(sceneJson.ui_selection_owners).filter(owner => owner && typeof owner === 'object');
  const itemById = new Map(items.concat(selectionOwners).map(item => [String(item?.id || '').trim(), item]).filter(([id]) => id));
  const recordsByLink = new Map();
  for (const item of items) {
    const link = exactSelectionLinkName(item);
    if (!link) continue;
    if (!recordsByLink.has(link)) recordsByLink.set(link, []);
    recordsByLink.get(link).push(item);
  }
  const explicitUiIdByLink = new Map();
  for (const [link, records] of recordsByLink) {
    const candidates = new Set();
    for (const record of records) {
      for (const field of UI_SELECTION_REFERENCE_FIELDS) {
        const candidate = String(record?.[field] || '').trim();
        if (candidate && itemById.has(candidate)) candidates.add(candidate);
      }
    }
    if (candidates.size === 1) explicitUiIdByLink.set(link, [...candidates][0]);
  }
  state.selectionIdentityIndex = { itemById, recordsByLink, explicitUiIdByLink, selectionOwners };
  return state.selectionIdentityIndex;
}
function uiSelectionIdentity(rendered) {
  const exactId = String(rendered?.item?.id || '');
  if (!exactId) return { id: '', resolution: 'exact_identity_fallback', linkName: '', pickRecordSource: '' };
  const index = state.selectionIdentityIndex || rebuildSelectionIdentityIndex();
  for (const field of UI_SELECTION_REFERENCE_FIELDS) {
    const candidate = String(rendered.item?.[field] || '').trim();
    if (candidate && index.itemById.has(candidate)) return { id: candidate, resolution: 'direct_explicit_ref', linkName: exactSelectionLinkName(rendered.item), pickRecordSource: rendered.pickRecordSource || 'payload_item' };
  }
  const linkName = exactSelectionLinkName(rendered.item);
  const linkedId = linkName ? index.explicitUiIdByLink.get(linkName) : '';
  if (linkedId) return { id: linkedId, resolution: 'exact_link_explicit_ref', linkName, pickRecordSource: rendered.pickRecordSource || 'payload_item' };
  if (rendered.uiSelectionOwnerId && index.itemById.has(rendered.uiSelectionOwnerId)) {
    return { id: rendered.uiSelectionOwnerId, resolution: rendered.uiSelectionResolution || 'exact_identity_fallback', linkName, pickRecordSource: rendered.pickRecordSource || 'expanded_urdf_inspection' };
  }
  return { id: exactId, resolution: 'exact_identity_fallback', linkName, pickRecordSource: rendered.pickRecordSource || 'payload_item' };
}
function explicitUiSelectionItemId(rendered) {
  return expandedUrdfPhysicalOwnerId(rendered) || uiSelectionIdentity(rendered).id;
}
function expandedUrdfPhysicalOwnerId(rendered) {
  if (rendered?.authoritativePhysicalPick !== true) return '';
  const directOwnerId = String(rendered.uiSelectionOwnerId || '').trim();
  const index = state.selectionIdentityIndex || rebuildSelectionIdentityIndex();
  if (directOwnerId && (selectionOwnerRenderedById(directOwnerId) || index.itemById.has(directOwnerId))) return directOwnerId;
  const preview = state.sceneJson?.robot_preview || {};
  const diagnostics = state.robotUrdfPreviewDiagnostics || {};
  const linkName = exactSelectionLinkName(rendered.item);
  const expectedLinks = fields => new Set(fields.flatMap(field => asArray(preview?.[field])).map(value => String(value || '').trim()).filter(Boolean));
  const toolLinks = expectedLinks(['expected_tool_visual_links', 'expectedToolVisualLinks']);
  const robotLinks = expectedLinks(['expected_robot_visual_links', 'expectedRobotVisualLinks']);
  const ownerFromContract = fields => {
    for (const source of [preview, diagnostics]) {
      for (const field of fields) {
        const id = String(source?.[field] || '').trim();
        if (id && (selectionOwnerRenderedById(id) || index.itemById.has(id))) return id;
      }
    }
    return '';
  };
  if (toolLinks.has(linkName)) return ownerFromContract(['selection_tool_owner_id', 'selectionToolOwnerId']);
  if (robotLinks.has(linkName)) return ownerFromContract(['selection_robot_owner_id', 'selectionRobotOwnerId']);
  return '';
}
function currentSelectionDiagnostics() {
  const id = String(state.selected || '');
  const rendered = selectedRenderIdentity();
  const editOwner = canonicalTransformOwner(rendered);
  const hasExplicitTransformOwner = explicitUiSelectionItemId(rendered) !== rendered?.item?.id;
  const binding = resolveCanonicalPhysicalEditBinding(rendered || id);
  const item = rendered?.item || null;
  const selectionIdentity = uiSelectionIdentity(rendered);
  const selectionOwners = state.selectionIdentityIndex?.selectionOwners || [];
  const attachedObject = state.three?.transformControls?.object || null;
  const canonicalOwnerId = editOwner?.item?.id || '';
  const hasCanonicalGizmoAttachment = Boolean(
    canonicalOwnerId && state.selected === canonicalOwnerId && selectionIsEditable(editOwner) &&
    (attachedObject === editOwner.object3d ||
      (attachedObject === state.gizmoPivot?.group && state.gizmoPivot?.owner === editOwner))
  );
  return {
    sceneId: sceneId(),
    selectedItemId: id,
    uiSelectionItemId: selectionIdentity.id,
    editOwnerItemId: canonicalOwnerId,
    canonicalOwnerId,
    canonicalSelectedId: id,
    editAuthoritySource: editAuthoritySource(editOwner?.item),
    gizmoAttachedTargetId: hasCanonicalGizmoAttachment ? canonicalOwnerId : '',
    gizmoAttachedObjectName: attachedObject?.name || '',
    gizmoVisible: Boolean(state.three?.transformControls?.visible),
    gizmoEnabled: Boolean(state.three?.transformControls?.enabled),
    gizmoAttachmentReason: state.gizmoAttachmentDiagnostic?.reason || 'not_evaluated',
    physicalEditBinding: binding ? {
      canonicalOwnerId: binding.ownerId,
      ownerRecordSource: binding.ownerRecordSource,
      physicalVisualId: binding.visual?.item?.id || '',
      pivotWorldCentre: state.gizmoAttachmentDiagnostic?.pivotWorldCentre || null,
      attachedObjectWorldPosition: state.gizmoAttachmentDiagnostic?.attachedObjectWorldPosition || null,
      distance: Number.isFinite(state.gizmoAttachmentDiagnostic?.distance) ? state.gizmoAttachmentDiagnostic.distance : null,
      attachmentReason: state.gizmoAttachmentDiagnostic?.reason || 'not_evaluated',
    } : null,
    renderIdentity: rendered?.item?.id || '',
    selectedItemType: rendered ? itemType(item) : '',
    selectable: Boolean(rendered && isCanvasSelectableRendered(rendered)),
    editable: Boolean(selectionIsEditable(rendered) ||
      ((hasExplicitTransformOwner || rendered?.authoritativePhysicalPick === true) && selectionIsEditable(editOwner))),
    locked: Boolean(item?.locked),
    sourceLayer: String(item?.source_layer || ''),
    activeVisualSource: String(item?.active_visual_source || ''),
    diagnosticOnly: Boolean(item && isDiagnosticOnlyItem(item)),
    helperOrOverlay: Boolean(item && isDebugOverlayItem(item)),
    objectPresent: Boolean(rendered),
    selectedLinkName: selectionIdentity.linkName,
    uiSelectionResolution: selectionIdentity.resolution,
    pickRecordSource: selectionIdentity.pickRecordSource,
    selectionOwnerRegistryCount: selectionOwners.length,
    selectionOwnerIds: selectionOwners.map(owner => String(owner.id || '')).filter(Boolean),
    lastRaycastHitCount: state.lastRaycastHitCount,
    lastRaycastCandidateIds: [...state.lastRaycastCandidateIds],
    lastCanvasSelectedItemId: state.lastCanvasSelectedItemId,
    lastCanvasPickReason: state.lastCanvasPickReason,
    lastCanvasPickDiagnostic: state.lastCanvasPickDiagnostic,
    lastFailedCanvasPickDiagnostic: state.lastFailedCanvasPickDiagnostic,
  };
}
function editorState() {
  const rendered = selectedRenderIdentity();
  const editOwner = canonicalTransformOwner(rendered);
  const hasExplicitTransformOwner = explicitUiSelectionItemId(rendered) !== rendered?.item?.id;
  return { ready: Boolean(state.sceneJson && state.three?.scene), sceneId: sceneId(), selectedItemId: state.selected || '', uiSelectionItemId: state.selected || '', editOwnerItemId: editOwner?.item?.id || '', selectedItemType: rendered ? itemType(rendered.item) : '', selectedEditable: selectionIsEditable(rendered) || ((hasExplicitTransformOwner || rendered?.authoritativePhysicalPick === true) && selectionIsEditable(editOwner)), selectionDiagnostics: currentSelectionDiagnostics(), lastRaycastHitCount: state.lastRaycastHitCount, lastRaycastCandidateIds: [...state.lastRaycastCandidateIds], lastCanvasSelectedItemId: state.lastCanvasSelectedItemId, lastCanvasPickReason: state.lastCanvasPickReason, lastCanvasPickDiagnostic: state.lastCanvasPickDiagnostic, lastFailedCanvasPickDiagnostic: state.lastFailedCanvasPickDiagnostic, dirty: state.dirtyTransforms.size > 0, dirtyCount: state.dirtyTransforms.size, canUndo: state.undoStack.length > 0, canRedo: state.redoStack.length > 0, mode: state.editorMode, transformSpace: state.transformSpace, error: state.editorError || '' };
}
function emitDirtyChanged() { pushEditorEvent('dirty_changed', { dirty: state.dirtyTransforms.size > 0, dirtyCount: state.dirtyTransforms.size, canUndo: state.undoStack.length > 0, canRedo: state.redoStack.length > 0 }); }
function showError(message) {
  const text = message || 'Unknown viewer error';
  state.editorError = text;
  pushEditorEvent('editor_error', { message: text });
  el.error.textContent = text;
  el.error.hidden = false;
  window.__WORKCELL_VIEWER_STATUS__ = {
    ...(window.__WORKCELL_VIEWER_STATUS__ || {}),
    readiness_contract_version: READINESS_CONTRACT_VERSION,
    readinessContractVersion: READINESS_CONTRACT_VERSION,
    lifecycle_state: 'scene_failed',
    lifecycleState: 'scene_failed',
    terminal: true,
    viewer_boot_state: 'scene_failed',
    viewerBootState: 'scene_failed',
    web3d_readiness_state: 'scene_failed',
    web3dReadinessState: 'scene_failed',
    failed_stage: 'viewer_runtime',
    failedStage: 'viewer_runtime',
    fatal_error: text,
    fatalError: text,
    fatal_stack: (new Error(text).stack || '').split('\n').slice(0, 6).join('\n'),
    fatalStack: (new Error(text).stack || '').split('\n').slice(0, 6).join('\n'),
  };
  renderSceneHealth();
}
function clearError() { state.editorError = ''; el.error.hidden = true; el.error.textContent = ''; }
function valueOrDash(value) { return value === undefined || value === null || value === '' ? '—' : value; }
function asArray(value) { return Array.isArray(value) ? value : []; }
function escapeHtml(value) { return String(valueOrDash(value)).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c])); }
function vector3(value, fallback = [0, 0, 0]) {
  const arr = Array.isArray(value) ? value : fallback;
  return new THREE.Vector3(Number(arr[0] || 0), Number(arr[1] || 0), Number(arr[2] || 0));
}
function isGeneratedUrdfItem(item) {
  const identity = viewerGroupIdentity(item);
  const source = String(item?.source || item?.source_kind || item?.source_layer || item?.active_visual_source || '').toLowerCase();
  return Boolean(
    item?.source === 'urdf_flattened' ||
    source.includes('urdf') ||
    (isGeneratedPreviewIdentity(item) && /\b(link|visual|robot|tool|gripper|urdf|moveit)\b/.test(identity)) ||
    String(item?.baked_world_visual_transform_source || item?.transform_source || '').toLowerCase().includes('urdf')
  );
}
function poseBlockOf(source, fallback = {}) {
  const pose = source && typeof source === 'object' ? source : fallback;
  const xyz = pose.xyz || pose.position || pose.translation || (Array.isArray(pose) ? pose.slice(0, 3) : fallback.xyz || [0, 0, 0]);
  const rpy = pose.rpy || pose.rotation_rpy || (Array.isArray(pose) ? pose.slice(3, 6) : fallback.rpy || [0, 0, 0]);
  return { xyz: vector3(xyz), rpy: vector3(rpy) };
}
function hasFinitePoseBlock(source) {
  if (!source || typeof source !== 'object') return false;
  const pose = poseBlockOf(source);
  return finiteVector(pose.xyz) && finiteVector(pose.rpy);
}
function bakedVisibleWorldPoseSource(item) {
  return item?.baked_world_visual_pose || item?.expected_visual_pose || item?.final_transform || item?.world_from_visual || null;
}
function hasMeshBackedVisualContract(item) {
  return Boolean(displayMeshUri(item) || item?.mesh_loaded || item?.meshLoaded || item?.mesh_status === 'loaded' || item?.renderInfo?.render_status === 'mesh_loaded');
}
function isExpandedUrdfRobotPreview(preview) {
  const mode = String(preview?.mode || '').trim();
  return mode === 'expanded_urdf_loader' || mode === 'expanded_urdf_robot_subtree';
}
function robotPreviewSummaryMode(preview) {
  return isExpandedUrdfRobotPreview(preview) ? 'Robot preview: expanded URDF loader' : (preview?.mode || 'mesh rows');
}
function isGeneratedUrdfMeshVisualItem(item) {
  if (!isGeneratedUrdfItem(item)) return false;
  if (!hasMeshBackedVisualContract(item)) return false;
  const identity = viewerGroupIdentity(item);
  return Boolean(
    item?.visual !== undefined ||
    item?.visual_name !== undefined ||
    item?.visual_index !== undefined ||
    item?.mesh_uri || item?.package_uri || item?.mesh_path || item?.source_path ||
    /\b(mesh|visual|link|robot|tool|gripper|camera|table|workbench)\b/.test(identity)
  );
}
function isRobotToolGeneratedUrdfMeshVisualItem(item) {
  return Boolean(isGeneratedUrdfMeshVisualItem(item) && (isGeneratedRobotItem(item) || isGeneratedToolOrGripperItem(item)));
}
function itemAssemblyGroup(item) { return String(item?.assembly_group || item?.robot_instance_id || '').trim(); }
function isAssemblyCandidateItem(item) {
  if (!isGeneratedUrdfItem(item)) return false;
  if (itemAssemblyGroup(item)) return true;
  const link = String(item?.link_name || item?.link || item?.frame || item?.object_name || '');
  const parent = String(item?.parent_link || item?.joint_parent_link || item?.immediate_parent_link || '');
  return Boolean(link && (parent || link === 'base_link' || link === 'base_link_inertia') && (isGeneratedRobotItem(item) || isGeneratedToolOrGripperItem(item)));
}
function usesUrdfFkVisualWorldPose(item) { return Boolean(item?.urdf_fk_verified_against_ros_tf === true && item?.urdf_fk_source === 'expanded_urdf_joint_tree' && hasFinitePoseBlock(item?.urdf_fk_visual_world_pose)); }
function usesAssembledUrdfHierarchy(item) { return Boolean(!usesUrdfFkVisualWorldPose(item) && (item?.robot_render_mode === 'assembled_urdf_hierarchy' || item?.workcell_web_render_pose_mode === 'assembled_urdf_hierarchy' || itemAssemblyGroup(item))); }
function usesBakedVisibleWorldPose(item) {
  if (usesAssembledUrdfHierarchy(item)) return false;
  if (!hasFinitePoseBlock(bakedVisibleWorldPoseSource(item))) return false;
  if (item?.workcell_web_render_pose_mode === 'baked_visible_world_pose') return true;
  return isGeneratedUrdfMeshVisualItem(item);
}
function effectiveWorkcellWebRenderPoseMode(item) {
  return usesUrdfFkVisualWorldPose(item) ? 'urdf_fk_visual_world_pose' : (usesAssembledUrdfHierarchy(item) ? 'assembled_urdf_hierarchy' : (usesBakedVisibleWorldPose(item) ? 'baked_visible_world_pose' : (item?.workcell_web_render_pose_mode || '')));
}
function generatedUrdfFramePoseSource(item) {
  if (item?.frame_world_pose) return item.frame_world_pose;
  if (item?.link_world_pose) return item.link_world_pose;
  warnMissingGeneratedUrdfFramePose(item);
  return {};
}
function framePoseOf(item) {
  return poseBlockOf(generatedUrdfFramePoseSource(item));
}
function visualOriginOf(item) {
  return poseBlockOf(item?.visual_origin || item?.visual_local_transform || {});
}
function canonicalFinalPose(item) {
  if (isGeneratedUrdfItem(item)) {
    if (usesUrdfFkVisualWorldPose(item)) return item.urdf_fk_visual_world_pose || {};
    if (usesBakedVisibleWorldPose(item)) {
      return item.baked_world_visual_pose || item.expected_visual_pose || item.final_transform || item.world_from_visual || {};
    }
    return generatedUrdfFramePoseSource(item);
  }
  return item.final_transform || item.world_from_visual || item.baked_world_visual_pose || item.pose || item.world_pose || {};
}
function poseOf(item) {
  const pose = canonicalFinalPose(item);
  if (isGeneratedUrdfItem(item)) return poseBlockOf(pose);
  const xyz = item.final_transform || item.world_from_visual || item.baked_world_visual_pose
    ? (pose.xyz || pose.position || pose.translation || (Array.isArray(pose) ? pose.slice(0, 3) : [0, 0, 0]))
    : (item.pose_xyz || pose.xyz || pose.position || pose.translation || (Array.isArray(pose) ? pose.slice(0, 3) : [0, 0, 0]));
  const rpy = item.final_transform || item.world_from_visual || item.baked_world_visual_pose
    ? (pose.rpy || pose.rotation_rpy || (Array.isArray(pose) ? pose.slice(3, 6) : [0, 0, 0]))
    : (item.pose_rpy || pose.rpy || pose.rotation_rpy || (Array.isArray(pose) ? pose.slice(3, 6) : [0, 0, 0]));
  return { xyz: vector3(xyz), rpy: vector3(rpy) };
}
function scaleOf(item) {
  // Generated URDF item roots are link/frame nodes. URDF mesh scale is
  // applied only to the loaded mesh/local wrapper, never to the link root.
  // Authored mesh conversion metadata has the same ownership rule: its
  // scale belongs to the visual-origin child, even when a compatibility
  // payload also mirrors that value into item.scale. An item with only an
  // object-level scale keeps the ordinary authored root-scale contract.
  const meshLocalScale = item?.mesh_local_transform?.scale || item?.mesh_scale;
  const meshScaleOwnedByVisual = Boolean(
    !isGeneratedUrdfItem(item) && hasMeshBackedVisualContract(item) && Array.isArray(meshLocalScale)
  );
  const scale = isGeneratedUrdfItem(item) || meshScaleOwnedByVisual ? [1, 1, 1] : (item.scale || [1, 1, 1]);
  return vector3(scale, [1, 1, 1]);
}
function transformOf(item) {
  const pose = poseOf(item);
  const scale = scaleOf(item);
  return {
    pose: { xyz: { x: pose.xyz.x, y: pose.xyz.y, z: pose.xyz.z }, rpy: { x: pose.rpy.x, y: pose.rpy.y, z: pose.rpy.z } },
    scale: { x: scale.x, y: scale.y, z: scale.z },
  };
}
function meshLocalVector(value, fallback, fieldName, reasons) {
  const arr = Array.isArray(value) ? value : fallback;
  if (value !== undefined && !Array.isArray(value)) reasons.push(`${fieldName} must be an array`);
  const out = fallback.map((fallbackValue, index) => {
    const raw = arr[index];
    const number = raw === undefined || raw === null || raw === '' ? fallbackValue : Number(raw);
    if (!Number.isFinite(number)) {
      reasons.push(`${fieldName}[${index}] must be finite`);
      return fallbackValue;
    }
    return number;
  });
  return new THREE.Vector3(out[0], out[1], out[2]);
}
function meshLocalTransformOf(item) {
  const source = item?.mesh_local_transform;
  const reasons = [];
  if (source !== undefined && (!source || typeof source !== 'object' || Array.isArray(source))) {
    reasons.push('mesh_local_transform must be an object');
  }
  const transform = source && typeof source === 'object' && !Array.isArray(source) ? source : {};
  const scaleSource = transform.scale || item?.mesh_scale || (isGeneratedUrdfItem(item) ? item?.scale : null) || [1, 1, 1];
  return {
    pose: {
      xyz: meshLocalVector(transform.xyz || transform.origin || transform.position, [0, 0, 0], 'mesh_local_transform.xyz', reasons),
      rpy: meshLocalVector(transform.rpy || transform.rotation_rpy, [0, 0, 0], 'mesh_local_transform.rpy', reasons),
    },
    scale: meshLocalVector(scaleSource, [1, 1, 1], 'mesh_local_transform.scale/mesh_scale', reasons),
    valid: reasons.length === 0,
    reason: reasons.join('; '),
    raw: source,
  };
}
function cloneTransform(transform) { return JSON.parse(JSON.stringify(transform)); }
function sameTransform(a, b) { return JSON.stringify(a) === JSON.stringify(b); }
function renderedById(id) { return state.objects.find(obj => obj.item.id === id) || state.pickRecords.find(obj => obj.item.id === id); }
function resolveCanonicalPhysicalEditBinding(value = state.selected) {
  const record = value?.item ? value : null;
  const candidateIds = [
    record?.physicalEditOwner?.item?.id,
    record?.item?.id,
    record ? explicitUiSelectionItemId(record) : value,
    state.selected,
  ].map(id => String(id || '').trim()).filter(Boolean);
  for (const ownerId of candidateIds) {
    const binding = state.physicalEditBindings.get(ownerId);
    if (binding) return binding;
  }
  return null;
}
function canonicalTransformOwner(value = state.selected) {
  const binding = resolveCanonicalPhysicalEditBinding(value);
  if (binding) return binding.owner;
  const rendered = value?.item ? value : renderedById(value);
  return canonicalEditOwnerRendered(rendered);
}
function selectionOwnerRenderedById(id) {
  return resolveSelectionOwner(id).record;
}
function resolveSelectionOwner(id) {
  const ownerId = String(id || '').trim();
  if (!ownerId) return { record: null, source: '' };
  const objectMatches = state.objects.filter(record => record?.item?.id === ownerId);
  const rendered = objectMatches.find(record => canEditItem(record.item) && !isGeneratedUrdfItem(record.item)) ||
    objectMatches.find(record => !isGeneratedUrdfItem(record.item)) || objectMatches[0] ||
    state.pickRecords.find(record => record?.item?.id === ownerId);
  if (rendered) return { record: rendered, source: 'state.objects' };
  const index = state.selectionIdentityIndex || rebuildSelectionIdentityIndex();
  const item = index.selectionOwners.find(owner => String(owner?.id || '').trim() === ownerId);
  return item ? {
    record: { item, object3d: null, pickRoot: null, readOnlyPick: true, virtualSelectionOwner: true },
    source: 'ui_selection_owners',
  } : { record: null, source: '' };
}
function selectedRenderIdentity() {
  return state.pickRecords.find(record => record.authoritativePhysicalPick === true && record.item?.id === state.selectedRenderIdentityId) ||
    renderedById(state.selectedRenderIdentityId) || renderedById(state.selected);
}
function registerPickRecord(item, object3d, root = object3d, options = {}) {
  if (!item?.id || !object3d) return null;
  const record = { item, object3d, pickRoot: root, readOnlyPick: true, ...options };
  state.pickRecords.push(record);
  state.pickIdentityByObject.set(object3d, record);
  return record;
}
function registerCanonicalPhysicalPick(rendered, source = 'rendered_physical') {
  if (!rendered?.item?.id || !rendered.object3d ||
      !isPhysicalSemanticItem(rendered.item) || !isNormalSelectableRendered(rendered)) return null;
  const ownerId = String(explicitUiSelectionItemId(rendered) || rendered.item.id).trim();
  if (!ownerId) return null;
  let record = rendered.canonicalPickRecord;
  if (!record || !state.pickRecords.includes(record)) {
    record = registerPickRecord(rendered.item, rendered.object3d, rendered.object3d, {
      // The pick record describes the physical hit identity; edit authority
      // remains with the one authored state.objects record for this stable ID.
      readOnlyPick: true,
      pickRecordSource: source,
      authoritativePhysicalPick: true,
      uiSelectionOwnerId: ownerId,
      uiSelectionResolution: ownerId === rendered.item.id ? 'authored_stable_id' : 'explicit_selection_owner',
      canonicalRenderedRecord: rendered,
    });
    rendered.canonicalPickRecord = record;
  } else {
    record.item = rendered.item;
    record.object3d = rendered.object3d;
    record.pickRoot = rendered.object3d;
    record.pickRecordSource = source;
    record.uiSelectionOwnerId = ownerId;
  }
  let boundNodeCount = 0;
  rendered.object3d.traverse?.(node => {
    state.pickIdentityByObject.set(node, record);
    node.userData = node.userData || {};
    node.userData.canonical_physical_owner_id = ownerId;
    node.userData.authoritative_physical_pick = true;
    boundNodeCount += 1;
  });
  record.canonicalPickBoundNodeCount = boundNodeCount;
  return record;
}
function bindExpandedUrdfPickRecordToSubtree(linkRoot, record, urdfLinkRoots) {
  if (!linkRoot || !record) return 0;
  let boundNodeCount = 0;
  const pending = [linkRoot];
  while (pending.length) {
    const node = pending.pop();
    if (node !== linkRoot && urdfLinkRoots?.has(node)) continue;
    state.pickIdentityByObject.set(node, record);
    // Keep the canonical record as the authority, but mirror the two fields
    // needed to recover ownership while inspecting a real URDFLoader subtree.
    // WeakMap is intentionally not iterable, so without this marker a link
    // whose record shape differs from the flattened payload cannot be found
    // from its mesh descendants when building selection bounds.
    node.userData = node.userData || {};
    node.userData.expanded_urdf_physical_owner_id = String(record.uiSelectionOwnerId || '');
    node.userData.expanded_urdf_authoritative_physical_pick = record.authoritativePhysicalPick === true;
    if (node !== linkRoot) boundNodeCount += 1;
    for (const child of node.children || []) pending.push(child);
  }
  return boundNodeCount;
}
function derivedTransformTargetId(item) {
  const targetId = String(item?.target_ref || '').trim();
  const identity = [item?.role, item?.semantic_role, item?.type, item?.category, item?.id]
    .map(value => String(value || '').toLowerCase().replaceAll('_', ' ')).join(' ');
  return targetId && /\bplace zone\b/.test(identity) ? targetId : '';
}
function isDerivedTransformDependent(item) { return Boolean(derivedTransformTargetId(item)); }
function inspectionSelectionRendered(rendered) {
  return rendered?.item?.id ? rendered : null;
}
function canonicalEditOwnerRendered(rendered) {
  const selectionOwner = selectionOwnerRenderedById(explicitUiSelectionItemId(rendered));
  if (selectionOwner && selectionOwner !== rendered && canEditItem(selectionOwner.item)) return selectionOwner;
  const derivedTarget = renderedById(derivedTransformTargetId(rendered?.item));
  return derivedTarget && canEditItem(derivedTarget.item) ? derivedTarget : inspectionSelectionRendered(rendered);
}
function exportedPhysicalEditOwnerId(rendered) {
  const item = rendered?.item;
  if (!item || !isGeneratedUrdfItem(item) || item.editable === true || item.locked !== true) return '';
  const category = String(item.mesh_contract_category || item.meshContractCategory || '').toLowerCase();
  if (category !== 'camera' && category !== 'table') return '';
  const ownerId = uiSelectionIdentity(rendered).id;
  const ownerType = String((state.selectionIdentityIndex || rebuildSelectionIdentityIndex()).itemById.get(ownerId)?.type || '').toLowerCase();
  return (category === 'camera' && ownerType === 'camera') || (category === 'table' && ownerType === 'support_surface') ? ownerId : '';
}
function applyExportedOwnerRelativeVisualTransform(rendered, owner) {
  const relative = rendered?.item?.owner_relative_visual_transform;
  if (!owner?.object3d || !rendered?.object3d || !hasFinitePoseBlock(relative)) return false;
  if (rendered.object3d.parent !== owner.object3d) owner.object3d.add(rendered.object3d);
  const pose = poseBlockOf(relative);
  rendered.object3d.position.copy(pose.xyz);
  rendered.object3d.rotation.set(pose.rpy.x, pose.rpy.y, pose.rpy.z, 'XYZ');
  // The rendered root follows the item's root-scale contract.  In particular,
  // generated URDF roots are unit scale; their mesh/unit conversion is applied
  // exactly once below this root by applyLoadedMeshScaleHandling().  Ignoring
  // legacy relative.scale also makes older exported payloads safe to reopen.
  rendered.object3d.scale.copy(scaleOf(rendered.item));
  rendered.object3d.updateMatrixWorld(true);
  return true;
}
function bindExportedPhysicalTransformOwnership() {
  state.physicalEditBindings.clear();
  const bindings = [];
  for (const rendered of state.objects) {
    const ownerId = exportedPhysicalEditOwnerId(rendered);
    if (!ownerId) continue;
    let owner = selectionOwnerRenderedById(ownerId);
    if (!owner?.object3d) {
      const exportedOwner = (state.selectionIdentityIndex || rebuildSelectionIdentityIndex()).itemById.get(ownerId) || {};
      const authoredTransform = transformOf(exportedOwner);
      const object3d = new THREE.Group();
      object3d.name = `${ownerId}_physical_edit_root`;
      object3d.up.copy(THREE.Object3D.DEFAULT_UP);
      applyTransformToObject(object3d, authoredTransform);
      const item = { ...exportedOwner, id: ownerId, editable: true, locked: false, source_layer: 'editable_authored_physical', render_policy: 'primary' };
      assignItemUserData(object3d, item);
      state.three.scene.add(object3d);
      owner = { item, object3d, fallback: null, labelEl: createLabelElement(item), originalTransform: cloneTransform(authoredTransform), authoredBaselineTransform: cloneTransform(authoredTransform), physicalEditRoot: true };
      state.objects.push(owner);
    }
    if (!owner?.object3d || owner === rendered) continue;
    // Reconstruct the generated visual from its exported stable owner-local
    // transform. Never preserve a stale generated world matrix during reparent.
    if (!applyExportedOwnerRelativeVisualTransform(rendered, owner)) continue;
    rendered.physicalEditOwner = owner;
    owner.ownedPhysicalVisual = rendered;
    const binding = { ownerId, owner, visual: rendered, ownerRecordSource: owner.physicalEditRoot ? 'synthetic_authored_selection_owner' : 'state.objects_authored_owner' };
    state.physicalEditBindings.set(ownerId, binding);
    bindings.push(binding);
  }
  return bindings;
}
function suppressOwnedAuthoredFallback(rendered) {
  const binding = resolveCanonicalPhysicalEditBinding(rendered);
  const owner = binding?.owner || rendered?.physicalEditOwner;
  if (!owner || !rendered.meshObject || rendered.item?.mesh_status !== 'loaded') return false;
  // Mesh loaders complete asynchronously. Reassert the same stable local
  // relationship so completion cannot restore the generated default world pose.
  applyExportedOwnerRelativeVisualTransform(rendered, owner);
  if (owner.fallback) owner.fallback.visible = false;
  owner.authoritativePhysicalVisualLoaded = true;
  if (state.selected === owner.item?.id && state.editorMode !== 'select') attachTransformGizmo(owner, 'asynchronous_physical_mesh_completion');
  return true;
}
function detachTransientPivotForFailedPhysicalVisual(rendered) {
  if (resolveCanonicalPhysicalEditBinding(rendered)?.owner === state.gizmoPivot?.owner) detachTransformGizmo('authoritative_physical_mesh_failed');
}
// canonicalSelectionRendered intentionally retired: inspection identity must
// never be rewritten to the transform owner.
function selectionIsEditable(rendered) {
  return Boolean(rendered && !rendered.readOnlyPick && !isTaskOnlyHelperItem(rendered.item) && !isDebugOverlayItem(rendered.item) && canonicalEditOwnerRendered(rendered) === rendered && canEditItem(rendered.item));
}
function transformFromObject(object) {
  return { pose: { xyz: { x: object.position.x, y: object.position.y, z: object.position.z }, rpy: { x: object.rotation.x, y: object.rotation.y, z: object.rotation.z } }, scale: { x: object.scale.x, y: object.scale.y, z: object.scale.z } };
}
function translationSnapValue() { const v = Number(el.translationSnap?.value || 0); return Number.isFinite(v) && v > 0 ? v : null; }
function rotationSnapRadians() { const v = Number(el.rotationSnap?.value || 0); return Number.isFinite(v) && v > 0 ? THREE.MathUtils.degToRad(v) : null; }
function snapTransform(transform, { translationAxes = ['x', 'y', 'z'], rotationAxes = ['x', 'y', 'z'] } = {}) {
  if (!el.snapToggle?.checked) return transform;
  const out = cloneTransform(transform);
  const t = translationSnapValue();
  if (t) for (const axis of translationAxes) out.pose.xyz[axis] = Math.round(out.pose.xyz[axis] / t) * t;
  const r = rotationSnapRadians();
  if (r) for (const axis of rotationAxes) out.pose.rpy[axis] = Math.round(out.pose.rpy[axis] / r) * r;
  return out;
}
function isFiniteTransform(transform) {
  const values = [transform?.pose?.xyz?.x, transform?.pose?.xyz?.y, transform?.pose?.xyz?.z, transform?.pose?.rpy?.x, transform?.pose?.rpy?.y, transform?.pose?.rpy?.z, transform?.scale?.x, transform?.scale?.y, transform?.scale?.z];
  return values.every(value => Number.isFinite(Number(value)));
}
function applyTransformToObject(object, transform) {
  if (!isFiniteTransform(transform)) return false;
  object.position.set(transform.pose.xyz.x, transform.pose.xyz.y, transform.pose.xyz.z);
  object.rotation.set(transform.pose.rpy.x, transform.pose.rpy.y, transform.pose.rpy.z, 'XYZ');
  object.scale.set(transform.scale.x, transform.scale.y, transform.scale.z);
  return true;
}
function editableTransformGroupMembers(rendered) {
  const group = String(rendered?.item?.transform_group || '').trim();
  if (!group || !canEditItem(rendered.item)) return rendered ? [rendered] : [];
  return state.objects.filter(candidate => canEditItem(candidate.item) && String(candidate.item.transform_group || '').trim() === group);
}
function captureTransformGroup(rendered) {
  return new Map(editableTransformGroupMembers(rendered).map(member => [member.item.id, cloneTransform(state.dirtyTransforms.get(member.item.id)?.newTransform || transformFromObject(member.object3d))]));
}
function linkedTransformChanges(rendered, before, after, memberStarts = null) {
  const yawDelta = after.pose.rpy.z - before.pose.rpy.z;
  const cosine = Math.cos(yawDelta);
  const sine = Math.sin(yawDelta);
  return editableTransformGroupMembers(rendered).map(member => {
    const memberBefore = member === rendered
      ? cloneTransform(before)
      : cloneTransform(memberStarts?.get(member.item.id) || state.dirtyTransforms.get(member.item.id)?.newTransform || transformFromObject(member.object3d));
    if (member === rendered) return { rendered: member, before: memberBefore, after: cloneTransform(after) };
    const relativeX = memberBefore.pose.xyz.x - before.pose.xyz.x;
    const relativeY = memberBefore.pose.xyz.y - before.pose.xyz.y;
    const relativeZ = memberBefore.pose.xyz.z - before.pose.xyz.z;
    const memberAfter = cloneTransform(memberBefore);
    memberAfter.pose.xyz.x = after.pose.xyz.x + cosine * relativeX - sine * relativeY;
    memberAfter.pose.xyz.y = after.pose.xyz.y + sine * relativeX + cosine * relativeY;
    memberAfter.pose.xyz.z = after.pose.xyz.z + relativeZ;
    memberAfter.pose.rpy.z += yawDelta;
    return { rendered: member, before: memberBefore, after: memberAfter };
  });
}
function applyTransformChanges(changes, { updateDirty = false } = {}) {
  if (!changes.every(change => isFiniteTransform(change.after))) return false;
  for (const change of changes) {
    applyTransformToObject(change.rendered.object3d, change.after);
    if (!updateDirty || isDerivedTransformDependent(change.rendered.item)) continue;
    if (sameTransform(change.rendered.originalTransform, change.after)) state.dirtyTransforms.delete(change.rendered.item.id);
    else state.dirtyTransforms.set(change.rendered.item.id, { oldTransform: cloneTransform(change.rendered.originalTransform), newTransform: cloneTransform(change.after) });
    syncInspectorTransformFields(change.rendered);
  }
  updateLabels();
  return true;
}
function markDirtyTransform(rendered, next, { pushHistory = true, oldTransform = null, snapOptions = undefined, memberStarts = null } = {}) {
  if (!rendered || !canEditItem(rendered.item)) return false;
  const owner = canonicalEditOwnerRendered(rendered);
  if (owner && owner !== rendered) {
    const dependentBefore = oldTransform || state.dirtyTransforms.get(rendered.item.id)?.newTransform || transformFromObject(rendered.object3d);
    const ownerBefore = state.dirtyTransforms.get(owner.item.id)?.newTransform || transformFromObject(owner.object3d);
    const ownerNext = cloneTransform(ownerBefore);
    ownerNext.pose.xyz.x += next.pose.xyz.x - dependentBefore.pose.xyz.x;
    ownerNext.pose.xyz.y += next.pose.xyz.y - dependentBefore.pose.xyz.y;
    ownerNext.pose.xyz.z += next.pose.xyz.z - dependentBefore.pose.xyz.z;
    ownerNext.pose.rpy.z += next.pose.rpy.z - dependentBefore.pose.rpy.z;
    ownerNext.scale = cloneTransform(next).scale;
    return markDirtyTransform(owner, ownerNext, { pushHistory, oldTransform: ownerBefore, snapOptions, memberStarts: memberStarts || captureTransformGroup(owner) });
  }
  const previous = oldTransform || state.dirtyTransforms.get(rendered.item.id)?.newTransform || transformFromObject(rendered.object3d);
  const snapped = snapOptions === null ? cloneTransform(next) : snapTransform(next, snapOptions);
  const changes = linkedTransformChanges(rendered, previous, snapped, memberStarts);
  if (pushHistory && !sameTransform(previous, snapped)) {
    state.undoStack.push({ changes: changes.map(change => ({ itemId: change.rendered.item.id, before: cloneTransform(change.before), after: cloneTransform(change.after) })) });
    state.redoStack = [];
  }
  if (!applyTransformChanges(changes, { updateDirty: true })) return false;
  updateDirtyState();
  emitDirtyChanged();
  return true;
}
function editPatchEntryFor(rendered) {
  return buildEditPatch().edits.find(edit => edit.item_id === rendered?.item?.id) || null;
}
function emitTransformCommitted(rendered) {
  if (!rendered) return;
  const transform = state.dirtyTransforms.get(rendered.item.id)?.newTransform || transformFromObject(rendered.object3d);
  pushEditorEvent('transform_committed', { itemId: rendered.item.id, itemType: itemType(rendered.item), editable: canEditItem(rendered.item), pose: cloneTransform(transform).pose, scale: cloneTransform(transform).scale, patchEntry: editPatchEntryFor(rendered) });
}

function finiteNumber(value) {
  return typeof value === 'number' && Number.isFinite(value);
}
function finiteVector(value) {
  return value && finiteNumber(value.x) && finiteNumber(value.y) && finiteNumber(value.z);
}
function positiveFiniteVector(value) {
  return finiteVector(value) && value.x > 0 && value.y > 0 && value.z > 0;
}
function rawPoseDiagnostics(item) {
  return {
    pose: item?.pose,
    world_pose: item?.world_pose,
    baked_world_visual_pose: item?.baked_world_visual_pose,
    expected_visual_pose: item?.expected_visual_pose,
    final_transform: item?.final_transform,
    world_from_visual: item?.world_from_visual,
    frame_world_pose: item?.frame_world_pose,
    link_world_pose: item?.link_world_pose,
    visual_origin: item?.visual_origin,
  };
}
function viewerFinalPoseDiagnostics(pose, scale) {
  return {
    xyz: { x: pose?.xyz?.x, y: pose?.xyz?.y, z: pose?.xyz?.z },
    rpy: { x: pose?.rpy?.x, y: pose?.rpy?.y, z: pose?.rpy?.z },
    scale: { x: scale?.x, y: scale?.y, z: scale?.z },
  };
}
function warnMissingGeneratedUrdfFramePose(item) {
  const key = `${item?.id || itemLabel(item || {})}:missing_generated_urdf_frame_pose`;
  state._generatedUrdfFramePoseWarnings = state._generatedUrdfFramePoseWarnings || new Set();
  if (state._generatedUrdfFramePoseWarnings.has(key)) return;
  state._generatedUrdfFramePoseWarnings.add(key);
  const meshUri = displayMeshUri(item);
  appendViewerDiagnosticWarning(item, 'missing_generated_urdf_frame_pose', `generated URDF item lacks both frame_world_pose and link_world_pose; placing generated URDF link at identity and reporting legacy visual-world fields only for diagnostics (mesh_uri=${meshUri || 'none'})`, {
    mesh_uri: meshUri,
    legacy_fallback_pose: item?.final_transform || item?.world_from_visual || item?.baked_world_visual_pose || item?.pose || item?.world_pose || null,
  });
}
function diagnosticResourceKey(item, meshUri = '') {
  return String(meshUri || item?.mesh_uri || item?.original_mesh_uri || item?.package_uri || item?.source_path || item?.mesh_path || item?.id || itemLabel(item || {}) || '').trim();
}
function appendDedupedRuntimeDiagnostic(warning) {
  const key = [sceneId(), warning?.code || warning?.source || 'diagnostic', diagnosticResourceKey(warning, warning?.mesh_uri || warning?.original_mesh_uri)].join('\n');
  if (state.diagnosticKeys.has(key)) return null;
  state.diagnosticKeys.add(key);
  state.runtimeWarnings.push(warning);
  refreshWarnings();
  return warning;
}
function appendViewerDiagnosticWarning(item, code, reason, extra = {}) {
  const warning = {
    source: 'runtime_viewer',
    code,
    object_id: item?.id || itemLabel(item || {}),
    id: item?.id || '',
    link: item?.link || item?.object_name || item?.visual || '',
    object_name: item?.object_name || item?.link || itemLabel(item || {}),
    parent_link: item?.parent_link || '',
    immediate_parent_link: item?.immediate_parent_link || '',
    pose: item?.pose,
    world_pose: item?.world_pose,
    baked_world_visual_pose: item?.baked_world_visual_pose,
    visual_origin: item?.visual_origin,
    scale: item?.scale,
    mesh_scale: item?.mesh_scale,
    reason: reason || 'viewer diagnostic warning',
    message: `${code}: ${item?.id || itemLabel(item || {})} (${item?.link || item?.object_name || itemLabel(item || {})}): ${reason || 'viewer diagnostic warning'}`,
    ...extra,
  };
  return appendDedupedRuntimeDiagnostic(warning);
}
function validateRenderableTransform(item) {
  const pose = poseOf(item);
  const scale = scaleOf(item);
  const finalPose = viewerFinalPoseDiagnostics(pose, scale);
  const reasons = [];
  if (!finiteVector(pose.xyz)) reasons.push('final xyz contains non-finite values');
  if (!finiteVector(pose.rpy)) reasons.push('final rpy contains non-finite values');
  if (!finiteVector(scale)) reasons.push('scale contains non-finite values');
  else if (!positiveFiniteVector(scale)) reasons.push('scale must be positive on every axis');
  if (reasons.length) {
    appendViewerDiagnosticWarning(item, 'invalid_renderable_transform', reasons.join('; '), {
      final_pose: finalPose,
      final_scale: finalPose.scale,
      raw_pose_fields: rawPoseDiagnostics(item),
      fallback_or_skip_reason: 'renderable skipped before applyPose because transform validation failed',
    });
    return { valid: false, pose, scale, reason: reasons.join('; ') };
  }
  return { valid: true, pose, scale, reason: '' };
}
function truthyFlag(value) { return value === true || String(value).toLowerCase() === 'true'; }
function itemRequiresMeshBackedVisual(item) {
  const explicit = item?.mesh_load_required || item?.requires_mesh || item?.mesh_required || item?.required_mesh || item?.mesh_backed_required || item?.requires_mesh_backed_visual;
  if (truthyFlag(explicit)) return true;
  const source = String(item?.source_kind || item?.source_layer || item?.active_visual_source || '').toLowerCase();
  const role = String(item?.role || item?.category || item?.type || '').toLowerCase();
  const hasMeshContract = Boolean(displayMeshUri(item) || item?.original_mesh_uri || item?.mesh_path || item?.source_path || item?.package_uri);
  return hasMeshContract && (source.includes('generated') || role.includes('robot') || role.includes('tool') || role.includes('gripper'));
}
function warnRequiredMeshFallback(item, meshUri, reason, extra = {}) {
  const transform = transformOf(item);
  const itemId = item?.id || itemLabel(item || {});
  const url = meshUri || displayMeshUri(item);
  appendViewerDiagnosticWarning(item, 'required_mesh_failed', `Required mesh failed: ${itemId} ${url}`, {
    mesh_uri: url,
    original_mesh_uri: item?.original_mesh_uri || item?.package_uri || item?.source_path || item?.mesh_path || '',
    final_pose: transform.pose,
    final_scale: transform.scale,
    raw_pose_fields: rawPoseDiagnostics(item),
    fallback_or_skip_reason: reason || 'required mesh failed; normal primitive fallback suppressed',
    mesh_load_error: reason || '',
    message: `Required mesh failed: ${itemId} ${url}`,
    ...extra,
  });
}


function trackMeshLoadAttempt(item, status, meshUrl, error = '') {
  item.mesh_load_required = truthyFlag(item?.mesh_load_required) || itemRequiresMeshBackedVisual(item);
  item.mesh_load_url = meshUrl || displayMeshUri(item);
  item.mesh_load_status = status;
  if (error) item.mesh_load_error = error;
}
function logRequiredMeshFailure(item, meshUrl, error) {
  if (!itemRequiresMeshBackedVisual(item)) return;
  // Required mesh failures must be visible in browser diagnostics with the exact URL attempted.
  console.error('Required mesh failed:', item?.id || itemLabel(item || {}), meshUrl || displayMeshUri(item), error || 'mesh load failed');
}
function failedMeshDebugMaterial() {
  return new THREE.MeshBasicMaterial({
    color: PRODUCT_VIEW_LIGHT_PALETTE.errorAccent,
    wireframe: true,
    transparent: true,
    opacity: 0.82,
    side: THREE.DoubleSide,
    depthWrite: false,
  });
}
function failedMeshDebugEdgeMaterial() {
  return new THREE.LineBasicMaterial({ color: PRODUCT_VIEW_LIGHT_PALETTE.errorAccent, transparent: true, opacity: 1 });
}
function styleFailedMeshDebugFallback(fallback, item, reason) {
  if (!fallback) return;
  fallback.visible = true;
  fallback.name = `${item.id || itemLabel(item)}_FAILED_REQUIRED_MESH_DEBUG_FALLBACK`;
  fallback.traverse?.(child => {
    if (child.isMesh) child.material = failedMeshDebugMaterial();
    else if (child.isLine || child.isLineSegments) child.material = failedMeshDebugEdgeMaterial();
    child.userData.render_status = 'required_mesh_failed_debug_fallback';
    child.userData.renderInfo = {
      render_status: 'required_mesh_failed_debug_fallback',
      mesh_uri: displayMeshUri(item),
      fallback_reason: reason || 'required mesh failed; debug fallback is not product geometry',
    };
  });
}

function primitiveOf(item) {
  return item.primitive || item.primitive_details || item.dimensions || item.geometry || item.primitive_geometry || null;
}
function displayMeshUri(item) {
  return item.mesh_uri || item.package_uri || item.mesh_path || item.source_path || '';
}
function setRenderInfo(rendered, renderStatus, meshUri, fallbackReason) {
  if (!rendered) return null;
  const info = {
    render_status: renderStatus,
    mesh_uri: meshUri || '',
    fallback_reason: fallbackReason || '',
  };
  rendered.renderInfo = info;
  if (rendered.item) rendered.item.renderInfo = info;
  rendered.object3d?.traverse?.(child => {
    child.userData.renderInfo = info;
    child.userData.render_status = renderStatus;
  });
  return info;
}

function refreshMeshLoadUi(rendered) {
  // Mesh loads complete asynchronously after the hierarchy is first drawn. Refresh
  // derived UI only; do not touch selection, transform gizmos, or dirty edit state.
  populateObjectList();
  renderSceneSummary();
  if (state.selected === rendered?.item?.id) populateInspector(rendered);
}
function meshStatusLabel(rendered) {
  const status = rendered?.renderInfo?.render_status || rendered?.item?.renderInfo?.render_status || rendered?.item?.mesh_status || 'unknown';
  if (status === 'mesh_loaded') {
    const boundsStatus = rendered?.item?.visual_bounds_status || '';
    if (boundsStatus === 'corrected_by_local_unit_scale') return 'mesh loaded (unit scale corrected)';
    if (boundsStatus && boundsStatus !== 'valid') return 'mesh loaded (visually invalid)';
    return 'mesh loaded';
  }
  if (isRuntimeFallbackStatus(status)) return 'fallback';
  if (isMissingOrFailedMeshStatus(rendered?.item?.mesh_status)) return 'mesh error';
  return String(status || 'unknown').replace(/_/g, ' ');
}
function appendRuntimeWarning(item, meshUri, reason, code = 'mesh_primitive_fallback', extra = {}) {
  return appendDedupedRuntimeDiagnostic({
    source: 'runtime_mesh',
    code,
    object_id: item?.id || itemLabel(item || {}),
    link: item?.link || item?.object_name || item?.visual || '',
    object_name: item?.object_name || item?.link || itemLabel(item || {}),
    original_mesh_uri: item?.original_mesh_uri || item?.package_uri || item?.source_path || item?.mesh_path || meshUri || '',
    mesh_uri: meshUri || '',
    parent_link: item?.parent_link || '',
    immediate_parent_link: item?.immediate_parent_link || '',
    pose: item?.pose,
    world_pose: item?.world_pose,
    baked_world_visual_pose: item?.baked_world_visual_pose,
    visual_origin: item?.visual_origin,
    final_pose: transformOf(item).pose,
    final_scale: transformOf(item).scale,
    scale: item?.scale,
    mesh_scale: item?.mesh_scale,
    fallback_or_skip_reason: reason || 'mesh loading skipped',
    reason: reason || 'mesh loading skipped',
    message: `${code}: Primitive fallback for ${item?.id || itemLabel(item || {})} (${item?.link || item?.object_name || itemLabel(item || {})}): ${reason || 'mesh loading skipped'}`,
    ...extra,
  });
}

function meshUriDiagnostic(item) {
  const raw = item?.mesh_uri;
  const requested = displayMeshUri(item);
  if (typeof raw !== 'string' || !raw.trim()) {
    if (requested && String(requested).startsWith('package://')) {
      return { uri: null, status: 'unresolved_package_uri', reason: `package URI was not staged for browser loading: ${requested}` };
    }
    return { uri: null, status: 'missing_file', reason: requested ? `mesh_uri was not staged for browser loading: ${requested}` : 'no mesh_uri provided' };
  }
  const uri = raw.trim();
  const lower = uri.toLowerCase();
  if (lower.startsWith('package://')) return { uri: null, status: 'unresolved_package_uri', reason: `package URI must be resolved and staged before browser loading: ${uri}` };
  const stagedHttpUrl = (() => {
    if (!lower.startsWith('http://') && !lower.startsWith('https://')) return null;
    try {
      const parsed = new URL(uri);
      const currentOrigin = window.location?.origin || '';
      const stagedPath = parsed.pathname.replace(/^\/+/, '');
      if (currentOrigin && currentOrigin !== 'null' && parsed.origin === currentOrigin && STAGED_MESH_ROOTS.some(root => stagedPath.startsWith(root))) return stagedPath + parsed.search + parsed.hash;
    } catch (_) { /* fall through to unsafe_path below */ }
    return null;
  })();
  if (stagedHttpUrl) return stagedHttpUrl;
  if (
    lower.startsWith('http://') ||
    lower.startsWith('https://') ||
    lower.startsWith('file://') ||
    lower.startsWith('data:') ||
    lower.startsWith('//') ||
    uri.startsWith('/') ||
    uri.startsWith('\\') ||
    /^[a-zA-Z]:[\\/]/.test(uri) ||
    /^[a-zA-Z][a-zA-Z\d+.-]*:/.test(uri) ||
    uri.includes('\\')
  ) return { uri: null, status: 'unsafe_path', reason: `unsafe mesh_uri rejected by viewer policy: ${uri}` };
  const pathOnly = uri.split(/[?#]/, 1)[0];
  const parts = pathOnly.split('/');
  if (parts.some(part => part === '..')) return { uri: null, status: 'unsafe_path', reason: `mesh_uri path traversal rejected: ${uri}` };
  if (!STAGED_MESH_ROOTS.some(root => pathOnly.startsWith(root))) {
    return { uri: null, status: 'unsafe_path', reason: `mesh_uri must be staged under build/workcell_studio_web_scene/assets/, workcell_studio_web/, or assets/: ${uri}` };
  }
  const ext = pathOnly.includes('.') ? pathOnly.slice(pathOnly.lastIndexOf('.') + 1).toLowerCase() : '';
  if (!['stl', 'dae', 'obj'].includes(ext)) return { uri: null, status: 'unsupported_format', reason: `unsupported mesh format .${ext || 'unknown'} for ${uri}` };
  const stagingStatus = String(item?.mesh_staging_status || '').toLowerCase();
  if (stagingStatus && !['staged', 'copied', 'available', 'loaded'].includes(stagingStatus)) {
    const status = stagingStatus === 'unsupported_format' ? 'unsupported_format'
      : stagingStatus === 'unsafe_path' || stagingStatus === 'unsafe_destination' || stagingStatus === 'unsupported_scheme' ? 'unsafe_path'
      : stagingStatus === 'no_mesh_uri' ? 'missing_file'
      : stagingStatus === 'resolve_failed' ? 'missing_file'
      : stagingStatus === 'unresolved_package_uri' ? 'unresolved_package_uri'
      : 'missing_file';
    return { uri: null, status, reason: item?.mesh_resolve_warning || `mesh staging did not produce a loadable file (mesh_staging_status=${stagingStatus})` };
  }
  return { uri, status: 'loaded', reason: '' };
}
function safeMeshUri(item) {
  return meshUriDiagnostic(item).uri;
}
function repoRootRelativeUrl(uri) {
  const origin = window.location?.origin;
  const base = origin && origin !== 'null' ? `${origin}/` : (document.baseURI || window.location?.href || '');
  return new URL(uri, base).href;
}

function meshLoaderNameForExtension(ext) {
  if (ext === 'stl') return 'STLLoader';
  if (ext === 'dae') return 'ColladaLoader';
  if (ext === 'obj') return 'OBJLoader';
  return 'unknown_loader';
}
function meshExtensionFromUri(uri) {
  const pathOnly = String(uri || '').split(/[?#]/, 1)[0];
  return pathOnly.includes('.') ? pathOnly.slice(pathOnly.lastIndexOf('.') + 1).toLowerCase() : '';
}
async function preflightMeshUrl(uri, loadUrl) {
  if (typeof fetch !== 'function') {
    return { ok: true, status: 'preflight_unavailable', reason: 'fetch is unavailable; loader will report any mesh access failure' };
  }
  const url = loadUrl || repoRootRelativeUrl(uri);
  const fileLike = String(window.location?.protocol || '').toLowerCase() === 'file:' || String(url).toLowerCase().startsWith('file:');
  const describeFailure = err => {
    const status = fileLike ? 'file_access_blocked' : 'url_not_served';
    const code = fileLike ? 'mesh_file_access_blocked' : 'mesh_url_not_served';
    return { ok: false, status, code, reason: `${status}: browser could not preflight mesh URL ${url} (${err?.message || err || 'fetch failed'})`, url };
  };
  try {
    let response = await fetch(url, { method: 'HEAD' });
    if (response.status === 405 || response.status === 501) {
      response = await fetch(url, { method: 'GET', headers: { Range: 'bytes=0-0' } });
    }
    if (!response.ok) {
      return {
        ok: false,
        status: 'url_not_served',
        code: 'mesh_url_not_served',
        reason: `url_not_served: mesh URL returned HTTP ${response.status} ${response.statusText || ''} for ${url}`.trim(),
        http_status: response.status,
        url,
      };
    }
    return { ok: true, status: 'url_served', reason: `mesh URL preflight succeeded for ${url}`, http_status: response.status, url };
  } catch (err) {
    return describeFailure(err);
  }
}
function supportSurfaceKindOf(item) {
  return String(item?.support_surface_kind || item?.supportSurfaceKind || '').trim().toLowerCase();
}
function supportSurfaceDisplayType(item) {
  const kind = supportSurfaceKindOf(item);
  if (kind === 'workbench_body') return 'Workbench / support surface';
  if (kind === 'table_surface' || kind === 'tabletop') return 'Tabletop / support surface';
  return '';
}

function supportSurfaceMetadata(item) {
  return {
    support_surface_kind: item?.support_surface_kind || '',
    supportSurfaceKind: item?.supportSurfaceKind || item?.support_surface_kind || '',
    top_surface_z_m: item?.top_surface_z_m ?? null,
    topSurfaceZM: item?.topSurfaceZM ?? item?.top_surface_z_m ?? null,
    support_surface_height_m: item?.support_surface_height_m ?? null,
    supportSurfaceHeightM: item?.supportSurfaceHeightM ?? item?.support_surface_height_m ?? null,
    expected_support_footprint_m: item?.expected_support_footprint_m || null,
    expectedSupportFootprintM: item?.expectedSupportFootprintM || item?.expected_support_footprint_m || null,
  };
}
function supportSurfaceWarningKey(item, code) {
  return `${item?.id || itemLabel(item || {})}:${code}`;
}
function warnInconsistentSupportSurfaceSemantics(item, code, reason, extra = {}) {
  state._supportSurfaceSemanticWarnings = state._supportSurfaceSemanticWarnings || new Set();
  const key = supportSurfaceWarningKey(item, code);
  if (state._supportSurfaceSemanticWarnings.has(key)) return;
  state._supportSurfaceSemanticWarnings.add(key);
  appendViewerDiagnosticWarning(item, code, reason, {
    support_surface_display_type: supportSurfaceDisplayType(item),
    ...supportSurfaceMetadata(item),
    ...extra,
  });
}
function maybeWarnSupportSurfaceSemantics(item, dims = null) {
  const kind = supportSurfaceKindOf(item);
  if (!kind) return;
  const topSurface = item?.top_surface_z_m ?? item?.topSurfaceZM;
  const supportHeight = item?.support_surface_height_m ?? item?.supportSurfaceHeightM;
  if (kind === 'workbench_body' && (!Number.isFinite(Number(topSurface)) || !Number.isFinite(Number(supportHeight)))) {
    warnInconsistentSupportSurfaceSemantics(item, 'support_surface_semantics_missing_height', 'workbench_body support surface is missing top/support height metadata');
  }
  if ((kind === 'tabletop' || kind === 'table_surface') && dims) {
    const thickness = Number(dims.z);
    const footprint = Math.max(Number(dims.x) || 0, Number(dims.y) || 0);
    if (Number.isFinite(thickness) && Number.isFinite(footprint) && footprint > 0 && thickness > Math.max(0.12, footprint * 0.12)) {
      warnInconsistentSupportSurfaceSemantics(item, 'support_surface_semantics_non_thin_tabletop', 'tabletop/table_surface support surface has non-thin loaded bounds', {
        loaded_dimensions: { x: dims.x, y: dims.y, z: dims.z },
        thickness_to_footprint_ratio: thickness / footprint,
      });
    }
  }
}

function itemType(item) {
  const supportSurfaceType = supportSurfaceDisplayType(item);
  if (supportSurfaceType) return supportSurfaceType;
  if (isGeneratedToolOrGripperItem(item)) return item.type || item.category || item.role || 'tool/gripper';
  if (isGeneratedRobotItem(item)) return item.type || item.category || item.role || 'robot';
  return item.type || item.category || item.role || item.source_kind || 'asset';
}
function itemLabel(item) {
  const supportSurfaceType = supportSurfaceDisplayType(item);
  if (supportSurfaceType) return `${item.label || item.display_name || item.object_name || item.name || item.link || item.id || 'unnamed'} (${supportSurfaceType})`;
  return item.label || item.display_name || item.object_name || item.name || item.link || item.id || 'unnamed';
}
function viewerGroupIdentity(item) {
  return [
    item?.source_kind,
    item?.source_layer,
    item?.active_visual_source,
    item?.renderer_owner,
    item?.renderer_ownership,
    item?.renderer,
    item?.robot_instance_id,
    item?.link_name,
    item?.visual_index,
    item?.type,
    item?.category,
    item?.role,
    item?.id,
    item?.label,
    item?.display_name,
    item?.object_name,
    item?.link,
    itemLabel(item || {}),
  ].map(value => String(value || '').toLowerCase().replace(/[_-]+/g, ' ')).join(' ');
}
function isGeneratedPreviewIdentity(item) {
  const identity = viewerGroupIdentity(item);
  return Boolean(item?.locked || /\b(generated|generated preview|urdf|moveit)\b/.test(identity));
}
function isGeneratedToolOrGripperItem(item) {
  const identity = viewerGroupIdentity(item);
  if (!isGeneratedPreviewIdentity(item)) return false;
  return /\b(tool|gripper|end effector|eef|suction|vacuum|robotiq|airpick|finger|coupler|flange|tcp|tool0)\b/.test(identity);
}
function isGeneratedRobotItem(item) {
  const identity = viewerGroupIdentity(item);
  if (!isGeneratedPreviewIdentity(item) || isGeneratedToolOrGripperItem(item)) return false;
  return /\b(robot|arm|manipulator|ur3|ur5|ur10|universal robot|base link|shoulder|upper arm|forearm|wrist|elbow|base)\b/.test(identity);
}
function viewerGroupFor(item) {
  const identity = viewerGroupIdentity(item);
  if (isPrimaryAuthoredPhysicalMesh(item)) return 'environment/layout';
  if (/\b(zone|pick zone|place zone|observation zone|spawn zone|safety zone|work envelope|reachability|collision)\b/.test(identity)) return 'zones';
  if (/\b(camera|sensor|realsense|depth camera|rgbd|lidar|vision)\b/.test(identity)) return 'sensors';
  if (isGeneratedToolOrGripperItem(item)) return 'tool/gripper';
  if (isGeneratedRobotItem(item)) return 'robot';
  if (/\b(environment|layout|asset|object|item|table|workbench|fixture|bin|tray|conveyor|shelf|rack|pallet|floor|wall|part|product)\b/.test(identity)) return 'environment/layout';
  if (/\b(robot|arm|manipulator|ur3|ur5|ur10|tool|gripper|end effector|eef|suction|vacuum|robotiq|airpick|generated|generated preview|urdf|moveit)\b/.test(identity)) return 'robot/tool/generated';
  return 'unknown';
}
const DEBUG_OVERLAY_TOKEN_RE = /\b(overlay|helper|debug|diagnostic|safety zone|pick zone|place zone|robot reach|transform anchor|warning marker|warning anchor|warning badge|camera fov|fov|pick coverage|reachability|collision|work envelope|task route|home pose|approach retreat|epd detection|detection label|bounds box|bounding box)\b/;
const PHYSICAL_SEMANTIC_TOKEN_RE = /\b(robot|arm|manipulator|tool|gripper|end effector|eef|camera body|configured camera|sensor body|conveyor|object|workpiece|part|product|bin|tray|support surface|table|tabletop|workbench|fixture|pallet|physical safety barrier|safety barrier|guard fence|fence)\b/;
function isZone(item) { return viewerGroupFor(item) === 'zones'; }
function isPhysicalSemanticItem(item) {
  if (isGeneratedToolOrGripperItem(item) || isGeneratedRobotItem(item) || supportSurfaceDisplayType(item) || isSensor(item)) return true;
  const identity = viewerGroupIdentity(item);
  return PHYSICAL_SEMANTIC_TOKEN_RE.test(identity);
}
function hasDimensionBackedPhysicalPrimitive(item) {
  return isPhysicalSemanticItem(item) && Boolean(dimensionsFromPrimitive(primitiveOf(item)));
}
function isDebugOverlayItem(item) {
  if (isOverlayPolicyItem(item)) return true;
  if (item?.debug_overlay === true || item?.exclude_from_fit_bounds === true || item?.source_layer === 'debug_overlay') return true;
  if (isPrimaryAuthoredPhysicalMesh(item)) return false;
  const identity = [
    item?.source_layer,
    item?.active_visual_source,
    item?.role,
    item?.category,
    item?.type,
    item?.status,
    item?.id,
    item?.display_name,
    item?.source_path,
    item?.mesh_path,
    item?.mesh_uri,
    item?.mesh_load_warning,
    ...(Array.isArray(item?.warnings) ? item.warnings : []),
    itemLabel(item || {}),
  ].map(value => String(value || '').toLowerCase().replace(/[_-]+/g, ' ')).join(' ');
  if (isPhysicalSemanticItem(item) && !/\b(safety zone|pick zone|place zone|observation zone|spawn zone|work envelope|robot reach|camera fov|fov|home pose|transform anchor|warning marker|warning anchor|warning badge)\b/.test(identity)) return false;
  if (viewerGroupFor(item) === 'zones') return true;
  return DEBUG_OVERLAY_TOKEN_RE.test(identity);
}
function isSensor(item) { return viewerGroupFor(item) === 'sensors'; }
function shouldLabelItem(item) { return !isDebugOverlayItem(item); }
function materialFor(item) {
  if (isZone(item)) return new THREE.MeshBasicMaterial({ color: 0xffc857, transparent: true, opacity: 0.1, side: THREE.DoubleSide, wireframe: true, depthWrite: false });
  if (isSensor(item)) return new THREE.MeshStandardMaterial({ color: 0x5f7485, roughness: 0.62, metalness: 0.08 });
  if (item.locked || item.source_kind === 'generated_preview') return new THREE.MeshStandardMaterial({ color: 0x8b96a6, roughness: 0.76, metalness: 0.04 });
  const authoredColor = item?.material?.color || item?.material?.rgba || item?.material_color || item?.color;
  // Imported STL files do not carry a material and a number of catalog STLs
  // have inconsistent triangle winding.  Fresh-placement previews used a
  // two-sided material, while the persisted mesh-loader path used Three's
  // front-side default and could therefore load successfully but render no
  // surface.  Keep this scoped to authored physical meshes; generated URDF
  // visuals retain their authored material/culling contract.
  const material = new THREE.MeshStandardMaterial({
    color: 0x8aa38d,
    roughness: 0.72,
    metalness: 0.02,
    side: isPrimaryAuthoredPhysicalMesh(item) ? THREE.DoubleSide : THREE.FrontSide,
  });
  if (Array.isArray(authoredColor) && authoredColor.length >= 3) {
    material.color.setRGB(Number(authoredColor[0]), Number(authoredColor[1]), Number(authoredColor[2]));
    if (authoredColor.length > 3) {
      material.opacity = Number(authoredColor[3]);
      material.transparent = material.opacity < 1;
    }
  }
  return material;
}
function materialHasUsableAppearance(material) {
  if (!material) return false;
  const materials = Array.isArray(material) ? material : [material];
  return materials.some(entry => entry && (entry.map || entry.color || entry.vertexColors || entry.emissive || entry.metalness !== undefined || entry.roughness !== undefined));
}
function applyNeutralFallbackToUnmaterialedMeshes(object, item) {
  object?.traverse?.(child => {
    if (child?.isMesh && !materialHasUsableAppearance(child.material)) child.material = materialFor(item);
  });
}
function fallbackMaterialFor(item) {
  const isZoneFallback = isZone(item);
  return new THREE.MeshStandardMaterial({
    color: isZoneFallback ? 0xffc857 : 0xff9f1c,
    emissive: isZoneFallback ? 0x3a2600 : 0x4a2600,
    emissiveIntensity: 0.12,
    roughness: 0.82,
    metalness: 0.02,
    transparent: true,
    opacity: isZoneFallback ? 0.08 : 0.46,
    side: THREE.DoubleSide,
    wireframe: isZoneFallback,
    depthWrite: false,
  });
}
function fallbackEdgeMaterialFor(item) {
  return new THREE.LineBasicMaterial({
    color: isZone(item) ? 0xffd36a : 0xffb347,
    transparent: true,
    opacity: 0.9,
  });
}
function applyFallbackRenderMetadata(object, item, status = 'fallback_geometry') {
  object.userData.render_status = status;
  object.userData.renderInfo = {
    render_status: status,
    mesh_uri: displayMeshUri(item),
    fallback_reason: 'fallback geometry rendered while a real mesh is unavailable',
  };
  object.traverse?.(child => {
    child.userData.render_status = status;
    child.userData.renderInfo = object.userData.renderInfo;
  });
}
function dimensionsFromPrimitive(primitive) {
  if (!primitive) return null;
  const dims = Array.isArray(primitive)
    ? primitive.slice(0, 3)
    : (primitive.size || primitive.dimensions || primitive.extents || [primitive.x || primitive.width, primitive.y || primitive.depth, primitive.z || primitive.height]);
  if (!Array.isArray(dims) || dims.length < 3) return null;
  const numeric = dims.slice(0, 3).map(Number);
  return numeric.every(value => Number.isFinite(value) && value > 0) ? numeric : null;
}
function makePrimitiveMesh(item) {
  const primitive = primitiveOf(item);
  const kind = String(item.geometry_type || item.primitive_geometry_type || primitive?.type || primitive?.shape || '').toLowerCase();
  let geometry;
  if (kind.includes('sphere')) geometry = new THREE.SphereGeometry(Number(primitive?.radius || 0.12), 24, 16);
  else if (kind.includes('cylinder')) geometry = new THREE.CylinderGeometry(Number(primitive?.radius || 0.08), Number(primitive?.radius || 0.08), Number(primitive?.height || 0.25), 24);
  else {
    const dims = dimensionsFromPrimitive(primitive);
    if (!dims) return null;
    // Zones are visual editing context, not destination volumes.  Keep place
    // zones as a thin floor footprint so the linked physical bin remains
    // visible and receives normal product picking.
    if (isZone(item) && /\bplace zone\b/.test(viewerGroupIdentity(item))) dims[2] = Math.min(dims[2], 0.01);
    geometry = new THREE.BoxGeometry(dims[0], dims[1], dims[2]);
  }
  const group = new THREE.Group();
  group.name = `${item.id || itemLabel(item)}_fallback_primitive`;
  const mesh = new THREE.Mesh(geometry, fallbackMaterialFor(item));
  mesh.name = `${item.id || itemLabel(item)}_fallback_solid`;
  group.add(mesh);
  const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geometry), fallbackEdgeMaterialFor(item));
  edges.name = `${item.id || itemLabel(item)}_fallback_edges`;
  group.add(edges);
  applyFallbackRenderMetadata(group, item, 'primitive_fallback');
  return group;
}
function makeSensorMarker(item) {
  const group = new THREE.Group();
  group.name = `${item.id || itemLabel(item)}_fallback_sensor`;
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.08, 0.08), fallbackMaterialFor(item));
  body.name = `${item.id || itemLabel(item)}_fallback_sensor_body`;
  body.userData.fallback_sensor_body = true;
  body.userData.selection_owner_id = String(item.id || '');
  group.add(body);
  const frustum = new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0,0,0), new THREE.Vector3(0.35,0.22,-0.18), new THREE.Vector3(0,0,0), new THREE.Vector3(0.35,-0.22,-0.18),
      new THREE.Vector3(0,0,0), new THREE.Vector3(0.35,0.22,0.18), new THREE.Vector3(0,0,0), new THREE.Vector3(0.35,-0.22,0.18),
      new THREE.Vector3(0.35,0.22,-0.18), new THREE.Vector3(0.35,-0.22,-0.18), new THREE.Vector3(0.35,-0.22,-0.18), new THREE.Vector3(0.35,-0.22,0.18),
      new THREE.Vector3(0.35,-0.22,0.18), new THREE.Vector3(0.35,0.22,0.18), new THREE.Vector3(0.35,0.22,0.18), new THREE.Vector3(0.35,0.22,-0.18),
    ]),
    fallbackEdgeMaterialFor(item)
  );
  frustum.name = `${item.id || itemLabel(item)}_fallback_sensor_frustum`;
  frustum.userData.fallback_sensor_frustum = true;
  frustum.userData.non_selectable = true;
  group.add(frustum);
  applyFallbackRenderMetadata(group, item, 'sensor_fallback');
  return group;
}
function applyRosRpy(object, rpy) {
  object.rotation.set(rpy.x, rpy.y, rpy.z, 'ZYX');
}
function applyPose(object, item) {
  const validation = validateRenderableTransform(item);
  if (!validation.valid) return false;
  const pose = validation.pose;
  object.position.copy(pose.xyz);
  object.rotation.set(pose.rpy.x, pose.rpy.y, pose.rpy.z, 'XYZ');
  const s = validation.scale;
  object.scale.set(s.x, s.y, s.z);
  return true;
}

function assignItemUserData(object, item) {
  object.userData.item = item;
  object.traverse?.(child => { child.userData.item = item; });
}
function applyLoadedMeshOverlayMaterial(object, item) {
  if (!isZone(item) || !object?.traverse) return;
  object.traverse(child => {
    if (child?.isMesh) child.material = materialFor(item);
  });
}
function materializeLoadedMesh(item, uri, loaded) {
  const ext = uri.split(/[?#]/, 1)[0].slice(uri.split(/[?#]/, 1)[0].lastIndexOf('.') + 1).toLowerCase();
  let object;
  if (ext === 'stl') object = new THREE.Mesh(loaded, materialFor(item));
  else if (ext === 'dae') object = loaded.scene;
  else object = loaded;
  applyNeutralFallbackToUnmaterialedMeshes(object, item);
  object.name = `${item.id || itemLabel(item)}_mesh`;
  applyLoadedMeshOverlayMaterial(object, item);
  assignItemUserData(object, item);
  return object;
}
function makeMeshVisualRoot(item, meshObject) {
  const visualRoot = new THREE.Group();
  visualRoot.name = `${item.id || itemLabel(item)}_visual_origin`;
  visualRoot.up.copy(THREE.Object3D.DEFAULT_UP);
  applyMeshLocalTransform(visualRoot, item);
  // Keep loader-provided scene hierarchies, especially Collada .dae internal
  // transforms, intact below the visual-origin root. Generated URDF visual
  // world/baked transforms belong to the posed link root, and mesh scale/unit
  // corrections stay on the raw loader output instead of the visual wrapper.
  applyLoadedMeshScaleHandling(meshObject, item);
  visualRoot.add(meshObject);
  assignItemUserData(visualRoot, item);
  return visualRoot;
}
function applyMeshLocalTransform(meshObject, item) {
  const transform = meshLocalTransformOf(item);
  const generatedUrdf = isGeneratedUrdfItem(item);
  // Legacy branch retained: ? (usesBakedVisibleWorldPose(item) ? poseBlockOf({ xyz: [0, 0, 0], rpy: [0, 0, 0] }) : visualOriginOf(item))
  const visualOrigin = generatedUrdf
    ? (usesUrdfFkVisualWorldPose(item) ? poseBlockOf({ xyz: [0, 0, 0], rpy: [0, 0, 0] }) : (usesBakedVisibleWorldPose(item) ? poseBlockOf({ xyz: [0, 0, 0], rpy: [0, 0, 0] }) : visualOriginOf(item)))
    : transform.pose;
  meshObject.position.copy(visualOrigin.xyz);
  applyRosRpy(meshObject, visualOrigin.rpy);
  if (generatedUrdf) meshObject.scale.set(1, 1, 1);
  else meshObject.scale.set(transform.scale.x, transform.scale.y, transform.scale.z);
  if (!transform.valid) {
    appendViewerDiagnosticWarning(item, 'invalid_mesh_local_transform', transform.reason, {
      mesh_local_transform: item?.mesh_local_transform,
      visual_local_transform: item?.visual_local_transform,
      visual_origin: item?.visual_origin,
      applied_mesh_local_transform: {
        xyz: { x: visualOrigin.xyz.x, y: visualOrigin.xyz.y, z: visualOrigin.xyz.z },
        rpy: { x: visualOrigin.rpy.x, y: visualOrigin.rpy.y, z: visualOrigin.rpy.z },
        scale: { x: transform.scale.x, y: transform.scale.y, z: transform.scale.z },
      },
      fallback_or_skip_reason: 'malformed mesh-local scale/origin metadata was defaulted before adding the mesh under the posed item object',
    });
  }
  return transform.valid;
}
function applyLoadedMeshScaleHandling(meshObject, item) {
  if (!isGeneratedUrdfItem(item)) return;
  const transform = meshLocalTransformOf(item);
  meshObject.scale.set(transform.scale.x, transform.scale.y, transform.scale.z);
}
function promiseWithDeadline(start, timeoutMs = PHYSICAL_MESH_LOAD_TIMEOUT_MS) {
  let timeoutId;
  let settled = false;
  return new Promise((resolve, reject) => {
    const finish = (handler, value) => {
      if (settled) return;
      settled = true;
      clearTimeout(timeoutId);
      handler(value);
    };
    timeoutId = setTimeout(() => {
      const error = new Error(`mesh load timed out after ${timeoutMs}ms`);
      error.code = 'mesh_load_timeout';
      error.timeoutMs = timeoutMs;
      finish(reject, error);
    }, timeoutMs);
    try {
      start(value => finish(resolve, value), error => finish(reject, error));
    } catch (error) {
      finish(reject, error);
    }
  });
}
function loadMeshWithDeadline(loader, url, timeoutMs = PHYSICAL_MESH_LOAD_TIMEOUT_MS) {
  return promiseWithDeadline((resolve, reject) => {
    if (typeof loader?.loadAsync === 'function') Promise.resolve(loader.loadAsync(url)).then(resolve, reject);
    else if (typeof loader?.load === 'function') loader.load(url, resolve, undefined, reject);
    else reject(new Error('mesh loader provides neither loadAsync() nor load()'));
  }, timeoutMs);
}
function emitAuthoredMeshDiagnosticOnce(item, detail = {}) {
  if (!isPrimaryAuthoredPhysicalMesh(item) || item.__authoredMeshDiagnosticEmitted) return;
  item.__authoredMeshDiagnosticEmitted = true;
  // QWebEngine forwards warning output to the Workcell Studio terminal. Keep
  // this one-shot: it is acceptance evidence, not a per-frame trace.
  console.warn?.(`Product View authored_mesh: ${JSON.stringify({ id: String(item?.id || ''), ...detail })}`);
}
function sceneAttachedThroughVisibleParents(object, expectedParent) {
  if (!object || object.parent !== expectedParent) return false;
  let node = object;
  while (node) {
    if (node.visible === false) return false;
    if (node === state.three?.scene) return true;
    node = node.parent;
  }
  return false;
}
function renderableDescendantCount(object) {
  let count = 0;
  object?.traverse?.(node => { if (node?.isMesh || node?.isLine || node?.isLineSegments || node?.isPoints || node?.isSprite) count += 1; });
  return count;
}
async function tryLoadMesh(item, rendered, fallback, readinessOperation) {
  const attempt = physicalMeshAttempt(item, readinessOperation);
  const diagnostic = meshUriDiagnostic(item);
  const uri = diagnostic.uri;
  const requestedUri = displayMeshUri(item);
  item.mesh_status = uri ? 'loading' : diagnostic.status;
  if (!uri) {
    const required = itemRequiresMeshBackedVisual(item);
    trackMeshLoadAttempt(item, diagnostic.status, requestedUri, diagnostic.reason);
    if (required) {
      logRequiredMeshFailure(item, requestedUri, diagnostic.reason);
      styleFailedMeshDebugFallback(fallback, item, diagnostic.reason);
      setRenderInfo(rendered, 'required_mesh_failed_debug_fallback', requestedUri, diagnostic.reason);
      warnRequiredMeshFallback(item, requestedUri, diagnostic.reason, { mesh_url: requestedUri, mesh_status: diagnostic.status });
    } else {
      setRenderInfo(rendered, rendered.renderInfo?.render_status || 'box_fallback', requestedUri, diagnostic.reason);
      if (requestedUri) appendRuntimeWarning(item, requestedUri, diagnostic.reason);
    }
    refreshMeshLoadUi(rendered);
    emitAuthoredMeshDiagnosticOnce(item, { requested_mesh_uri: requestedUri, resolved_url: '', loader: '', http_load_success: false, load_failure: diagnostic.reason, mesh_object_created: false, visual_root_attached: false, render_status: rendered?.renderInfo?.render_status || '' });
    detachTransientPivotForFailedPhysicalVisual(rendered);
    if (required) failWeb3dSceneReadiness(item, requestedUri, diagnostic.reason, { mesh_status: diagnostic.status });
    else requiredReadinessCompleteForItem(item);
    return;
  }
  const ext = meshExtensionFromUri(uri);
  const loaderName = meshLoaderNameForExtension(ext);
  const loadUrl = repoRootRelativeUrl(uri);
  const remainingPreflightMs = Math.max(1, attempt.deadlineAt - Date.now());
  const preflight = await promiseWithDeadline(
    (resolve, reject) => Promise.resolve(preflightMeshUrl(uri, loadUrl)).then(resolve, reject),
    remainingPreflightMs,
  ).catch(error => ({
    ok: false,
    status: 'loader_failure',
    reason: error?.message || String(error),
    url: loadUrl,
    timeout_ms: error?.code === 'mesh_load_timeout' ? PHYSICAL_MESH_LOAD_TIMEOUT_MS : undefined,
  }));
  if (!physicalMeshAttemptIsCurrent(attempt)) return;
  if (!preflight.ok) {
    const required = itemRequiresMeshBackedVisual(item);
    item.mesh_status = preflight.status || 'url_not_served';
    item.mesh_load_error = `${preflight.reason}; url=${preflight.url || loadUrl}`;
    trackMeshLoadAttempt(item, item.mesh_status, preflight.url || loadUrl, item.mesh_load_error);
    if (required) {
      logRequiredMeshFailure(item, preflight.url || loadUrl, item.mesh_load_error);
      styleFailedMeshDebugFallback(fallback, item, item.mesh_load_error);
      setRenderInfo(rendered, 'required_mesh_failed_debug_fallback', uri, item.mesh_load_error);
      warnRequiredMeshFallback(item, preflight.url || loadUrl, item.mesh_load_error, { mesh_url: preflight.url || loadUrl, http_status: preflight.http_status || null });
    } else {
      if (fallback) fallback.visible = true;
      setRenderInfo(rendered, rendered.renderInfo?.render_status || 'box_fallback', uri, item.mesh_load_error);
      appendRuntimeWarning(item, uri, item.mesh_load_error, preflight.code || 'mesh_url_not_served', { mesh_url: preflight.url || loadUrl, http_status: preflight.http_status || null });
    }
    refreshMeshLoadUi(rendered);
    emitAuthoredMeshDiagnosticOnce(item, { requested_mesh_uri: requestedUri, resolved_url: preflight.url || loadUrl, loader: loaderName, http_load_success: false, load_failure: item.mesh_load_error, mesh_object_created: false, visual_root_attached: false, render_status: rendered?.renderInfo?.render_status || '' });
    detachTransientPivotForFailedPhysicalVisual(rendered);
    if (required) failPhysicalMeshAttempt(attempt, item, preflight.url || loadUrl, item.mesh_load_error, { http_status: preflight.http_status || null, mesh_status: item.mesh_status, loader: loaderName, ...(preflight.timeout_ms ? { timeout_ms: preflight.timeout_ms } : {}) });
    else requiredReadinessCompleteForItem(item);
    return;
  }
  try {
    const loader = ext === 'stl' ? new STLLoader() : (ext === 'dae' ? new ColladaLoader() : new OBJLoader());
    const loaded = await loadMeshWithDeadline(loader, loadUrl, Math.max(1, attempt.deadlineAt - Date.now()));
    if (!physicalMeshAttemptIsCurrent(attempt)) return;
    const meshObject = materializeLoadedMesh(item, uri, loaded);
    // Legacy flow was applyMeshLocalTransform(meshObject, item) followed by
    // rendered.object3d.add(meshObject); generated URDF now applies that
    // local transform to the visual-origin wrapper so the loaded hierarchy is
    // preserved under the posed link root.
    const visualRoot = makeMeshVisualRoot(item, meshObject);
    const rawAuthoredLocalBounds = measureLoadedMeshBoundsInAuthoredLocalSpace(visualRoot);
    maybeApplyMeshUnitAutoscale(item, meshObject, visualRoot, rawAuthoredLocalBounds, uri);
    const validationLocalBounds = measureLoadedMeshBoundsInAuthoredLocalSpace(visualRoot);
    if (fallback) fallback.visible = false;
    rendered.object3d.add(visualRoot);
    rendered.meshObject = visualRoot;
    rendered.loadedMeshObject = meshObject;
    registerCanonicalPhysicalPick(rendered, 'loaded_physical_mesh');
    item.mesh_status = 'loaded';
    suppressOwnedAuthoredFallback(rendered);
    item.mesh_load_error = '';
    trackMeshLoadAttempt(item, 'loaded', loadUrl, '');
    setRenderInfo(rendered, 'mesh_loaded', uri, '');
    const boundsValid = diagnoseLoadedMeshBounds(item, visualRoot, rendered, validationLocalBounds);
    if (isPrimaryAuthoredPhysicalMesh(item)) {
      rendered.object3d.updateWorldMatrix?.(true, true);
      const authoredBounds = finiteBox3(new THREE.Box3().setFromObject(visualRoot));
      const worldScale = new THREE.Vector3();
      const worldPosition = new THREE.Vector3();
      visualRoot.getWorldScale?.(worldScale);
      visualRoot.getWorldPosition?.(worldPosition);
      const attachedVisible = sceneAttachedThroughVisibleParents(visualRoot, rendered.object3d);
      emitAuthoredMeshDiagnosticOnce(item, {
        requested_mesh_uri: requestedUri, resolved_url: loadUrl, loader: loaderName,
        http_load_success: true, load_failure: '', mesh_object_created: Boolean(meshObject),
        visual_root_attached: attachedVisible, parent_name: String(visualRoot.parent?.name || ''),
        object_visible: visualRoot.visible !== false, parent_visible: rendered.object3d.visible !== false,
        local_position: [visualRoot.position.x, visualRoot.position.y, visualRoot.position.z],
        local_quaternion: [visualRoot.quaternion.x, visualRoot.quaternion.y, visualRoot.quaternion.z, visualRoot.quaternion.w],
        local_scale: [visualRoot.scale.x, visualRoot.scale.y, visualRoot.scale.z],
        world_position: [worldPosition.x, worldPosition.y, worldPosition.z],
        world_scale: [worldScale.x, worldScale.y, worldScale.z], finite_world_bounds: Boolean(authoredBounds),
        bounds_min: authoredBounds ? [authoredBounds.min.x, authoredBounds.min.y, authoredBounds.min.z] : null,
        bounds_max: authoredBounds ? [authoredBounds.max.x, authoredBounds.max.y, authoredBounds.max.z] : null,
        material_side: meshObject?.material?.side ?? null,
        child_renderable_count: renderableDescendantCount(visualRoot),
        render_status: rendered?.renderInfo?.render_status || '',
      });
      if (!attachedVisible || !renderableDescendantCount(visualRoot)) {
        item.visual_bounds_status = !attachedVisible ? 'detached_or_hidden' : 'no_renderable_descendants';
      }
    }
    refreshMeshLoadUi(rendered);
    const usableAttachedVisual = !isPrimaryAuthoredPhysicalMesh(item) ||
      (sceneAttachedThroughVisibleParents(visualRoot, rendered.object3d) && renderableDescendantCount(visualRoot) > 0);
    if ((!boundsValid || !usableAttachedVisual) && itemRequiresMeshBackedVisual(item)) {
      detachTransientPivotForFailedPhysicalVisual(rendered);
      failPhysicalMeshAttempt(attempt, item, loadUrl, `loaded mesh bounds validation failed (${item.visual_bounds_status || 'invalid'})`, {
        mesh_status: item.mesh_status,
        ...physicalMeshBoundsFailurePayload(item, loadUrl, loaderName),
      });
      return;
    }
    completePhysicalMeshAttempt(attempt);
  } catch (err) {
    if (!physicalMeshAttemptIsCurrent(attempt)) return;
    const required = itemRequiresMeshBackedVisual(item);
    item.mesh_status = 'loader_failure';
    item.mesh_load_error = err?.message || String(err);
    const reason = `loader_failure: ${loaderName} failed for .${ext || 'unknown'} after fetchable URL ${loadUrl}: ${item.mesh_load_error}`;
    trackMeshLoadAttempt(item, 'loader_failure', loadUrl, reason);
    if (required) {
      logRequiredMeshFailure(item, loadUrl, reason);
      styleFailedMeshDebugFallback(fallback, item, reason);
      setRenderInfo(rendered, 'required_mesh_failed_debug_fallback', uri, reason);
      warnRequiredMeshFallback(item, loadUrl, reason, { extension: ext, loader: loaderName, mesh_url: loadUrl });
    } else {
      if (fallback) fallback.visible = true;
      setRenderInfo(rendered, rendered.renderInfo?.render_status || 'box_fallback', uri, reason);
      appendRuntimeWarning(item, uri, reason, 'mesh_loader_failure', { extension: ext, loader: loaderName, mesh_url: loadUrl });
    }
    refreshMeshLoadUi(rendered);
    emitAuthoredMeshDiagnosticOnce(item, { requested_mesh_uri: requestedUri, resolved_url: loadUrl, loader: loaderName, http_load_success: true, load_failure: reason, mesh_object_created: false, visual_root_attached: false, render_status: rendered?.renderInfo?.render_status || '' });
    detachTransientPivotForFailedPhysicalVisual(rendered);
    if (required) failPhysicalMeshAttempt(attempt, item, loadUrl, reason, { extension: ext, loader: loaderName, mesh_status: item.mesh_status, ...(err?.code === 'mesh_load_timeout' ? { timeout_ms: PHYSICAL_MESH_LOAD_TIMEOUT_MS } : {}) });
    else requiredReadinessCompleteForItem(item);
  }
}
function collectItems(sceneJson) {
  const buckets = ['robots', 'tools', 'assets', 'sensors', 'zones', 'items', 'objects', 'frames'];
  const byId = new Map();
  for (const bucket of buckets) for (const item of asArray(sceneJson[bucket])) if (item && typeof item === 'object') byId.set(item.id || `${bucket}_${byId.size}`, { ...item, id: item.id || `${bucket}_${byId.size}` });
  return Array.from(byId.values());
}
function frameNameOf(frame, fallback = '') {
  return String(frame?.name || frame?.frame || frame?.frame_id || frame?.link || frame?.link_name || frame?.id || fallback || '');
}
function frameParentNameOf(frame) {
  return String(frame?.parent || frame?.parent_frame || frame?.parent_frame_id || frame?.parent_link || frame?.parent_link_name || '');
}
function parseSceneFrames(sceneJson) {
  const lookup = new Map();
  const source = sceneJson?.frames || sceneJson?.scene?.frames;
  if (Array.isArray(source)) {
    source.forEach((frame, index) => {
      if (!frame || typeof frame !== 'object') return;
      const name = frameNameOf(frame, `frame_${index}`);
      if (name) lookup.set(name, { ...frame, name });
    });
  } else if (source && typeof source === 'object') {
    for (const [key, value] of Object.entries(source)) {
      if (!value || typeof value !== 'object') continue;
      const name = frameNameOf(value, key);
      if (name) lookup.set(name, { ...value, name });
    }
  }
  return lookup;
}
function matrixFromPoseBlock(source) {
  const pose = poseBlockOf(source || {});
  return new THREE.Matrix4().compose(
    pose.xyz,
    new THREE.Quaternion().setFromEuler(new THREE.Euler(pose.rpy.x, pose.rpy.y, pose.rpy.z, 'XYZ')),
    new THREE.Vector3(1, 1, 1),
  );
}
function poseBlockFromMatrix(matrix) {
  const xyz = new THREE.Vector3();
  const quaternion = new THREE.Quaternion();
  const scale = new THREE.Vector3();
  matrix.decompose(xyz, quaternion, scale);
  const euler = new THREE.Euler().setFromQuaternion(quaternion, 'XYZ');
  return { xyz, rpy: new THREE.Vector3(euler.x, euler.y, euler.z), matrix: matrix.clone() };
}
function localFramePoseSource(frame) {
  return frame?.world_pose || frame?.world_transform || frame?.pose || frame?.transform || frame?.origin || frame;
}
function resolveFrameWorldPose(name, lookup = state.frameLookup, stack = new Set()) {
  if (!name || !lookup?.has(name)) return null;
  const cached = state.resolvedFramePoses.get(name);
  if (cached) return cached;
  if (stack.has(name)) return null;
  stack.add(name);
  const frame = lookup.get(name);
  const localMatrix = matrixFromPoseBlock(localFramePoseSource(frame));
  const parentName = frameParentNameOf(frame);
  let worldMatrix = localMatrix;
  if (parentName && lookup.has(parentName) && !frame?.world_pose && !frame?.world_transform) {
    const parentPose = resolveFrameWorldPose(parentName, lookup, stack);
    if (parentPose?.matrix) worldMatrix = parentPose.matrix.clone().multiply(localMatrix);
  }
  const pose = poseBlockFromMatrix(worldMatrix);
  state.resolvedFramePoses.set(name, pose);
  stack.delete(name);
  return pose;
}

function finiteXyzArrayFromVector(value) {
  const xyz = value && typeof value === 'object'
    ? [Number(value.x), Number(value.y), Number(value.z)]
    : [];
  return xyz.length === 3 && xyz.every(Number.isFinite) ? xyz : null;
}
function finiteXyzArrayFromPoseSource(source) {
  if (!source || typeof source !== 'object' || !THREE?.Vector3) return null;
  return finiteXyzArrayFromVector(poseBlockOf(source).xyz);
}
function renderedObjectFrameNames(item, object3d) {
  const names = [
    item?.frame, item?.frame_id, item?.link, item?.link_name, item?.name, item?.id, item?.label, item?.display_name,
    object3d?.name,
  ];
  return new Set(names.filter(value => value !== undefined && value !== null && value !== '').map(value => String(value)));
}
function renderedObjectWorldXyzForFrame(name) {
  if (!name || !THREE?.Vector3) return null;
  for (const rendered of state.objects || []) {
    if (!rendered?.object3d || !isGeneratedUrdfItem(rendered.item)) continue;
    if (!renderedObjectFrameNames(rendered.item, rendered.object3d).has(String(name))) continue;
    rendered.object3d.updateWorldMatrix(true, true);
    const world = new THREE.Vector3();
    rendered.object3d.getWorldPosition(world);
    const xyz = finiteXyzArrayFromVector(world);
    if (xyz) return xyz;
  }
  return null;
}
function resolvedFrameAnchorXyz(name) {
  if (!name || !THREE?.Vector3) return null;
  let pose = state.resolvedFramePoses.get(name);
  if (!pose) pose = resolveFrameWorldPose(name);
  return finiteXyzArrayFromVector(pose?.xyz);
}
function metadataWorldXyzForFrame(name) {
  if (!name) return null;
  for (const rendered of state.objects || []) {
    const item = rendered?.item;
    if (!item || !renderedObjectFrameNames(item, rendered?.object3d).has(String(name))) continue;
    const xyz = finiteXyzArrayFromPoseSource(item.frame_world_pose) || finiteXyzArrayFromPoseSource(item.link_world_pose);
    if (xyz) return xyz;
  }
  return null;
}
function resolveWorldXyzForFrame(name) {
  return renderedObjectWorldXyzForFrame(name)
    || resolvedFrameAnchorXyz(name)
    || metadataWorldXyzForFrame(name);
}
function distanceMetersBetweenXyz(a, b) {
  if (!a || !b) return null;
  const distance = Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  return Number.isFinite(distance) ? distance : null;
}

function renderedObjectForFrame(name) {
  if (!name) return null;
  for (const rendered of state.objects || []) {
    const item = rendered?.item;
    if (!item || !renderedObjectFrameNames(item, rendered?.object3d).has(String(name))) continue;
    return rendered;
  }
  return null;
}
function collectFrameDiagnostics() {
  const names = new Set();
  for (const name of state.frameLookup?.keys?.() || []) names.add(String(name));
  for (const name of state.resolvedFramePoses?.keys?.() || []) names.add(String(name));
  for (const rendered of state.objects || []) {
    if (isGeneratedUrdfItem(rendered?.item)) {
      const link = linkNameOfItem(rendered.item);
      if (link) names.add(link);
    }
  }
  const diagnostics = [];
  for (const name of names) {
    const frame = state.frameLookup?.get?.(name) || {};
    const pose = state.resolvedFramePoses.get(name) || resolveFrameWorldPose(name);
    const rendered = renderedObjectForFrame(name);
    const item = rendered?.item || {};
    const renderStatus = rendered?.renderInfo?.render_status || item?.renderInfo?.render_status || item?.mesh_status || '';
    const sourceLayer = frame.source_layer || item.source_layer || '';
    const provenance = frame.provenance || item.provenance || null;
    const worldPose = frame.world_pose || frame.world_transform || null;
    const xyz = finiteXyzArrayFromVector(pose?.xyz);
    diagnostics.push({
      id: frame.id || item.id || name,
      name: frame.name || name,
      frame: frame.frame || frame.frame_id || item.frame || item.frame_id || name,
      link: frame.link || frame.link_name || item.link || item.link_name || name,
      parent: frame.parent || frame.parent_frame || frame.parent_frame_id || item.parent || item.parent_frame || '',
      parent_link: frame.parent_link || frame.parent_link_name || item.parent_link || item.parent_link_name || frameParentNameOf(frame) || '',
      role: frame.role || item.role || '',
      type: frame.type || item.type || '',
      render_expected: Boolean((frame.render_expected ?? item.render_expected) || displayMeshUri(frame) || displayMeshUri(item) || rendered),
      mesh_available: Boolean(renderStatus === 'mesh_loaded' || displayMeshUri(frame) || displayMeshUri(item)),
      source_layer: sourceLayer,
      provenance,
      world_pose: worldPose,
      xyz: xyz,
      rpy: pose?.rpy ? finiteXyzArrayFromVector(pose.rpy) : null,
      resolved_world_position: xyz,
    });
  }
  diagnostics.sort((a, b) => String(a.name || a.frame || a.id).localeCompare(String(b.name || b.frame || b.id)));
  return diagnostics;
}

function buildResolvedFrameStatus() {
  const frameNames = ['wrist_3_link', 'tool0', 'gripper_base_link'];
  const pairs = [
    ['wrist_3_link', 'tool0'],
    ['tool0', 'gripper_base_link'],
    ['wrist_3_link', 'gripper_base_link'],
  ];
  const positions = {};
  for (const name of frameNames) {
    const xyz = resolveWorldXyzForFrame(name);
    if (xyz) positions[name] = xyz;
  }
  const distances = {};
  for (const [a, b] of pairs) {
    const distance = distanceMetersBetweenXyz(positions[a], positions[b]);
    if (distance !== null) distances[`${a} -> ${b}`] = distance;
  }
  return {
    resolvedFramePositions: positions,
    resolved_frame_positions: positions,
    viewer_resolved_distances_m: distances,
    resolvedFrameDistancesM: distances,
  };
}

function diagnosticDebugItem(id, label) {
  return {
    id,
    label,
    display_name: label,
    type: 'debug_overlay',
    category: 'diagnostic',
    role: 'frame_diagnostic',
    source_layer: 'debug_overlay',
    active_visual_source: 'debug_overlay',
    debug_overlay: true,
    exclude_from_fit_bounds: true,
    locked: true,
  };
}
function addDebugRenderedObject(object3d, item) {
  object3d.visible = state.debugOverlaysVisible;
  object3d.userData.exclude_from_fit_bounds = true;
  assignItemUserData(object3d, item);
  state.three.scene.add(object3d);
  state.objects.push({ item, object3d, fallback: null, labelEl: null, renderInfo: { render_status: 'debug_overlay' }, originalTransform: transformOf(item) });
}
function renderFrameDebugOverlays() {
  const targets = ['wrist_3_link', 'tool0', 'gripper_base_link'];
  const resolved = new Map();
  state.resolvedFramePoses.clear();
  for (const name of targets) {
    const pose = resolveFrameWorldPose(name);
    if (!pose) continue;
    resolved.set(name, pose);
    const axes = new THREE.AxesHelper(0.12);
    axes.name = `${name}_debug_frame_axes`;
    axes.position.copy(pose.xyz);
    axes.rotation.set(pose.rpy.x, pose.rpy.y, pose.rpy.z, 'XYZ');
    addDebugRenderedObject(axes, diagnosticDebugItem(`debug_frame_axes_${name}`, `${name} frame axes`));
  }
  for (const [parent, child] of [['wrist_3_link', 'tool0'], ['tool0', 'gripper_base_link']]) {
    const parentPose = resolved.get(parent);
    const childPose = resolved.get(child);
    if (!parentPose || !childPose) continue;
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([parentPose.xyz, childPose.xyz]),
      new THREE.LineBasicMaterial({ color: 0x8b5cf6, transparent: true, opacity: 0.85 }),
    );
    line.name = `${parent}_to_${child}_debug_connector`;
    addDebugRenderedObject(line, diagnosticDebugItem(`debug_frame_connector_${parent}_to_${child}`, `${parent} → ${child} connector`));
  }
}
function validateSceneJson(json) {
  if (!json || typeof json !== 'object' || Array.isArray(json)) throw new Error('Invalid web_scene.json: expected a JSON object.');
  if (json.schema_version !== SUPPORTED_SCHEMA_VERSION) throw new Error(`Unsupported schema_version "${valueOrDash(json.schema_version)}". Expected ${SUPPORTED_SCHEMA_VERSION}.`);
  const items = collectItems(json);
  return items;
}
function installProductViewLights(scene) {
  for (const child of [...scene.children]) {
    if (child?.userData?.product_view_light === true) scene.remove(child);
  }
  const ambient = new THREE.HemisphereLight(PRODUCT_VIEW_LIGHT_PALETTE.ambientSky, PRODUCT_VIEW_LIGHT_PALETTE.ambientGround, 1.18);
  ambient.name = 'product_view_balanced_ambient_light';
  ambient.userData.product_view_light = true;
  scene.add(ambient);

  const key = new THREE.DirectionalLight(PRODUCT_VIEW_LIGHT_PALETTE.keyLight, 1.28);
  key.name = 'product_view_key_light';
  key.position.set(3.4, -4.2, 5.6);
  key.castShadow = true;
  key.shadow.mapSize.set(1024, 1024);
  Object.assign(key.shadow.camera, { near: 0.1, far: 12, left: -4, right: 4, top: 4, bottom: -4 });
  key.shadow.camera.updateProjectionMatrix();
  key.shadow.bias = -0.0002;
  key.shadow.normalBias = 0.015;
  key.userData.product_view_light = true;
  scene.add(key);

  const fill = new THREE.DirectionalLight(PRODUCT_VIEW_LIGHT_PALETTE.fillLight, 0.42);
  fill.name = 'product_view_fill_light';
  fill.position.set(-3.2, 3.8, 2.8);
  fill.userData.product_view_light = true;
  scene.add(fill);
}

function initThree() {
  try {
    if (!THREE?.Scene || !OrbitControls || !STLLoader || !ColladaLoader || !OBJLoader || !TransformControls) throw new Error('Three.js modules were not available.');
    const ROS_Z_UP = new THREE.Vector3(0, 0, 1);
    THREE.Object3D.DEFAULT_UP.copy(ROS_Z_UP);
    const renderer = new THREE.WebGLRenderer({ canvas: el.canvas, antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    const scene = new THREE.Scene();
    scene.up.copy(ROS_Z_UP);
    scene.background = new THREE.Color(PRODUCT_VIEW_LIGHT_PALETTE.workspaceBackground);
    renderer.setClearColor(PRODUCT_VIEW_LIGHT_PALETTE.rendererClearColor, 1);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.02;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    const camera = new THREE.PerspectiveCamera(55, 1, 0.01, 100);
    camera.up.copy(ROS_Z_UP);
    camera.position.set(2.4, -2.8, 1.8);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.object.up.copy(ROS_Z_UP);
    controls.enableDamping = true;
    controls.enableZoom = true;
    controls.mouseButtons.LEFT = null;
    controls.mouseButtons.MIDDLE = THREE.MOUSE.PAN;
    controls.mouseButtons.RIGHT = THREE.MOUSE.ROTATE;
    const grid = new THREE.GridHelper(5, 20, PRODUCT_VIEW_LIGHT_PALETTE.gridMajor, PRODUCT_VIEW_LIGHT_PALETTE.gridMinor);
    grid.name = 'ros_xy_ground_grid';
    for (const material of Array.isArray(grid.material) ? grid.material : [grid.material]) {
      if (!material) continue;
      material.transparent = true;
      material.opacity = 0.34;
      material.depthWrite = false;
    }
    grid.userData.exclude_from_fit_bounds = true;
    grid.userData.exclude_from_physical_bounds = true;
    grid.up.copy(ROS_Z_UP);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);
    scene.add(new THREE.AxesHelper(0.75));
    installProductViewLights(scene);
    const transformControls = new TransformControls(camera, renderer.domElement);
    transformControls.setMode('translate');
    transformControls.setSpace(state.transformSpace);
    transformControls.addEventListener('dragging-changed', event => {
      syncOrbitControlsForEditorMode();
      if (state.cancellingTransformOperation) return;
      const rendered = canonicalTransformOwner(state.selected);
      const attachedToOwner = transformControls.object === rendered?.object3d || (state.gizmoPivot?.owner === rendered && transformControls.object === state.gizmoPivot.group);
      if (!rendered || !canEditItem(rendered.item) || !attachedToOwner) return;
      if (state.gizmoPivot?.owner === rendered && transformControls.object === state.gizmoPivot.group) {
        if (event.value) beginTransientPivotDrag(rendered);
        else finishTransientPivotDrag(rendered);
        return;
      }
      if (event.value) { state.gizmoDragStart = cloneTransform(state.dirtyTransforms.get(rendered.item.id)?.newTransform || transformFromObject(rendered.object3d)); state.gizmoDragGroupStart = captureTransformGroup(rendered); }
      else { const finalTransform = transformFromObject(rendered.object3d); if (state.gizmoDragStart && !sameTransform(state.gizmoDragStart, finalTransform)) { const committed = markDirtyTransform(rendered, finalTransform, { pushHistory: true, oldTransform: state.gizmoDragStart, snapOptions: null, memberStarts: state.gizmoDragGroupStart }); if (committed) emitTransformCommitted(rendered); } state.gizmoDragStart = null; state.gizmoDragGroupStart = null; }
    });
    transformControls.addEventListener('objectChange', () => {
      if (state.cancellingTransformOperation) return;
      const rendered = canonicalTransformOwner(state.selected);
      if (!rendered || !canEditItem(rendered.item)) return;
      if (state.gizmoPivot?.owner === rendered && transformControls.object === state.gizmoPivot.group) { previewTransientPivotDrag(rendered); return; }
      if (!state.gizmoDragStart || transformControls.object !== rendered.object3d) return;
      const preview = transformFromObject(rendered.object3d);
      applyTransformChanges(linkedTransformChanges(rendered, state.gizmoDragStart || preview, preview, state.gizmoDragGroupStart));
      syncInspectorTransformFields(rendered);
      updateLabels();
    });
    controls.addEventListener('start', markCameraUserControlled);
    scene.add(transformControls);
    state.three = { renderer, scene, camera, controls, transformControls, raycaster: new THREE.Raycaster(), pointer: new THREE.Vector2() };
    syncOrbitControlsForEditorMode();
    resize();
    window.addEventListener('resize', resize);
    el.canvas.addEventListener('pointerdown', onCanvasPointerDown);
    el.canvas.addEventListener('pointermove', onCanvasPointerMove);
    el.canvas.addEventListener('pointerup', onCanvasPointerUp);
    el.canvas.addEventListener('pointercancel', onCanvasPointerCancel);
    el.canvas.addEventListener('contextmenu', onCanvasContextMenu);
    window.addEventListener('keydown', onEditorKeyDown);
    animate();
  } catch (err) {
    showError(`Bundled Three.js module load failure: ${err.message || err}`);
  }
}
function resize() {
  const { renderer, camera } = state.three;
  if (!renderer || !camera) return;
  const rect = el.canvas.getBoundingClientRect();
  renderer.setSize(rect.width, rect.height, false);
  camera.aspect = rect.width / Math.max(rect.height, 1);
  camera.updateProjectionMatrix();
}
function animate() {
  const { renderer, scene, camera, controls } = state.three;
  if (!renderer) return;
  state.animationId = requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
  updateLabels();
}


function physicalBoundsIdentityFor(object) {
  const item = object?.userData?.item || {};
  return [
    object?.userData?.source_layer,
    object?.userData?.active_visual_source,
    object?.userData?.role,
    object?.userData?.category,
    object?.userData?.id,
    object?.userData?.display_name,
    object?.userData?.status,
    object?.userData?.mesh_load_warning,
    item?.source_layer,
    item?.active_visual_source,
    item?.role,
    item?.category,
    item?.type,
    item?.id,
    item?.display_name,
    item?.status,
    item?.mesh_load_warning,
    ...(Array.isArray(item?.warnings) ? item.warnings : []),
    itemLabel(item || {}),
  ].map(value => String(value || '').toLowerCase().replace(/[_-]+/g, ' ')).join(' ');
}
function physicalBoundsNodeIdentityFor(object) {
  const data = object?.userData || {};
  return [
    object?.name,
    object?.type,
    data.source_layer,
    data.active_visual_source,
    data.role,
    data.category,
    data.type,
    data.id,
    data.display_name,
    data.object_name,
    data.link_name,
    data.visual_name,
    data.render_owner,
    data.renderer_owner,
    data.render_policy,
  ].map(value => String(value || '').toLowerCase().replace(/[_-]+/g, ' ')).join(' ');
}
function physicalBoundsItemFor(object, nearestItem = null) {
  return object?.userData?.item || nearestItem || object?.userData || {};
}
function isInitialFitPhysicalGeometryItem(item, identity = '') {
  const category = meshContractCategoryOf(item);
  if (['robot', 'tool', 'table', 'camera', 'object'].includes(category)) return true;
  return /\b(robot|arm|manipulator|ur3|ur5|ur10|universal robot|base link|shoulder|wrist|tool|gripper|end effector|eef|suction|robotiq|airpick|camera body|configured camera|sensor body|conveyor|object|workpiece|part|product|bin|tray|support surface|table|tabletop|workbench|fixture|pallet)\b/.test(identity);
}
function isPhysicalBoundsHelperObject(object, item, identity) {
  if (!object || object.visible === false) return true;
  if (object.isGridHelper || object.isAxesHelper) return true;
  if (object.userData?.selection_outline === true || object.userData?.selection_highlight === true) return true;
  if (object.userData?.fallback_sensor_frustum === true) return true;
  if (object.userData?.exclude_from_physical_bounds === true || object.userData?.exclude_from_fit_bounds === true) return true;
  if (item?.exclude_from_physical_bounds === true || item?.exclude_from_fit_bounds === true) return true;
  if (DEBUG_OVERLAY_TOKEN_RE.test(identity)) return true;
  return !isInitialFitPhysicalGeometryItem(item, identity);
}
function physicalBoundsHelperReasons(object, item, identity, options = {}) {
  if (!object) return [];
  const reasons = [];
  const add = reason => { if (!reasons.includes(reason)) reasons.push(reason); };
  const data = object.userData || {};
  if (data.selection_outline === true) add('selection_outline');
  if (data.selection_highlight === true) add('selection_highlight');
  if (data.fallback_sensor_frustum === true) add('fallback_sensor_frustum');
  if (data.exclude_from_physical_bounds === true) add('node_exclude_from_physical_bounds');
  if (data.exclude_from_fit_bounds === true) add('node_exclude_from_fit_bounds');
  // An expanded-URDF pick record describes ownership, not the nature of each
  // descendant. Its broad item exclusions must not hide real link meshes.
  if (options.authoritativePhysical !== true) {
    if (item?.exclude_from_physical_bounds === true) add('item_exclude_from_physical_bounds');
    if (item?.exclude_from_fit_bounds === true) add('item_exclude_from_fit_bounds');
  }
  if (object.isGridHelper) add('grid_helper');
  if (object.isAxesHelper) add('axes_helper');
  // Helper classification is node-local. The owning item can legitimately
  // carry loader diagnostics and warning text without making its loaded mesh
  // a helper render node.
  const localIdentity = physicalBoundsNodeIdentityFor(object);
  if (/\btransform\s*controls?\b/.test(localIdentity)) add('transform_controls');
  if (DEBUG_OVERLAY_TOKEN_RE.test(localIdentity)) add('debug_helper');
  const renderStatus = String(data.render_status || data.renderInfo?.render_status || '').toLowerCase();
  const localFallbackIdentity = `${renderStatus} ${localIdentity}`;
  if (data.fallback_geometry === true || data.isFallback === true || /fallback/.test(localFallbackIdentity)) add('fallback_geometry');
  return reasons;
}
function isRobotPrimitiveFallbackObject(object, item, identity) {
  const visualSource = String(item?.active_visual_source || object?.userData?.active_visual_source || '').toLowerCase();
  const sourceLayer = String(item?.source_layer || object?.userData?.source_layer || '').toLowerCase();
  return /primitive fallback|fallback/.test(identity) && /robot|arm|manipulator|ur3|ur5|ur10|universal robot|base link|shoulder|wrist/.test(identity)
    && (visualSource.includes('primitive_fallback') || visualSource.includes('fallback') || sourceLayer.includes('primitive_fallback'));
}
function collectPhysicalVisibleBounds(root, options = {}) {
  if (!root || !THREE?.Box3) return { count: 0, bounds: null, bounds_json: null };
  root.updateWorldMatrix?.(true, true);
  const bounds = new THREE.Box3();
  const candidates = [];
  const diagnostics = options.diagnostics || null;
  const visitedNodes = options.visitedNodes || new Set();
  const selectionOwnerId = String(options.selectionOwnerId || '').trim();
  let hasGeneratedRobotMesh = false;
  const visit = (node, nearestItem = null) => {
    if (!node || visitedNodes.has(node)) return;
    if (node.visible === false) {
      if (diagnostics) diagnostics.exclusion_reasons.push('hidden_renderable_or_subtree');
      return;
    }
    const identityRecord = state.pickIdentityByObject?.get?.(node);
    if (selectionOwnerId && identityRecord?.authoritativePhysicalPick === true &&
        explicitUiSelectionItemId(identityRecord) !== selectionOwnerId) {
      if (diagnostics) diagnostics.exclusion_reasons.push('different_authoritative_owner');
      return;
    }
    visitedNodes.add(node);
    const item = identityRecord?.item || physicalBoundsItemFor(node, nearestItem);
    const identity = identityRecord?.item
      ? `${physicalBoundsIdentityFor(node)} ${viewerGroupIdentity(identityRecord.item)}`
      : physicalBoundsIdentityFor(node);
    const nextNearestItem = identityRecord?.item || node?.userData?.item || nearestItem;
    const isRenderable = node.isMesh || node.isLine || node.isLineSegments || node.isPoints || node.isSprite;
    if (isRenderable && diagnostics) diagnostics.visible_renderable_count += 1;
    const renderStatus = String(node?.userData?.render_status || node?.userData?.renderInfo?.render_status || '').toLowerCase();
    // Authoritative expanded-URDF records and successfully loaded authored
    // meshes are physical by construction.  Do not run their descriptive
    // payload (which may contain diagnostic warning text) through the broad
    // helper-token classifier.  Explicit helper/fallback flags still win.
    const authoritativePhysical = identityRecord?.authoritativePhysicalPick === true;
    const helperReasons = physicalBoundsHelperReasons(node, item, identity, { authoritativePhysical });
    const loadedAuthoredPhysical = renderStatus === 'mesh_loaded' && isPrimaryAuthoredPhysicalMesh(item);
    const rejected = helperReasons.length > 0 ||
      (!authoritativePhysical && !loadedAuthoredPhysical && isPhysicalBoundsHelperObject(node, item, identity));
    if (isRenderable && !rejected) {
      const visualSource = String(item?.active_visual_source || node?.userData?.active_visual_source || '').toLowerCase();
      const isGenerated = /generated|urdf/.test(identity);
      const isRobot = /robot|arm|manipulator|ur3|ur5|ur10|universal robot|base link|shoulder|wrist/.test(identity);
      if (isGenerated && isRobot && /mesh|generated urdf visual|mesh preview/.test(visualSource.replace(/_/g, ' '))) hasGeneratedRobotMesh = true;
      candidates.push({ node, item, identity });
      if (diagnostics) diagnostics.accepted_renderable_count += 1;
    } else if (isRenderable && diagnostics) {
      diagnostics.excluded_renderable_count += 1;
      const reasons = helperReasons.length ? helperReasons : ['non_physical_identity'];
      diagnostics.exclusion_reasons.push(...reasons);
      diagnostics.explicit_helper_reasons = diagnostics.explicit_helper_reasons || {};
      for (const reason of helperReasons) diagnostics.explicit_helper_reasons[reason] = (diagnostics.explicit_helper_reasons[reason] || 0) + 1;
    }
    for (const child of node.children || []) visit(child, nextNearestItem);
  };
  visit(root);
  let count = 0;
  for (const candidate of candidates) {
    if (hasGeneratedRobotMesh && isRobotPrimitiveFallbackObject(candidate.node, candidate.item, candidate.identity)) continue;
    const nodeBounds = finiteBox3(new THREE.Box3().setFromObject(candidate.node));
    if (!nodeBounds) continue;
    bounds.union(nodeBounds);
    count += 1;
  }
  const finite = count ? finiteBox3(bounds) : null;
  return { count: finite ? count : 0, bounds: finite, bounds_json: box3ToJson(finite) };
}

function selectionPhysicalBoundsRecords(rendered) {
  const ownerId = String(explicitUiSelectionItemId(rendered) || rendered?.item?.id || '').trim();
  if (!ownerId) return rendered ? [rendered] : [];
  const records = [...state.objects, ...state.pickRecords].filter(record => {
    if (!record?.object3d) return false;
    // Expanded URDF records already carry the canonical owner assigned from
    // robot_preview.  Compare it directly before attempting payload-derived
    // identity resolution; production inspection records do not necessarily
    // have the same item fields as flattened/generated rows.
    const directOwner = String(record.uiSelectionOwnerId || record.object3d?.userData?.expanded_urdf_physical_owner_id || '').trim();
    return directOwner === ownerId || explicitUiSelectionItemId(record) === ownerId;
  });
  const binding = resolveCanonicalPhysicalEditBinding(ownerId);
  for (const record of [rendered, binding?.owner, binding?.visual]) {
    if (record?.object3d && !records.includes(record)) records.push(record);
  }
  return records;
}
function collectSelectionPhysicalBounds(rendered) {
  if (!rendered) return { count: 0, bounds: null, bounds_json: null };
  const ownerId = String(explicitUiSelectionItemId(rendered) || rendered?.item?.id || '').trim();
  const bounds = new THREE.Box3();
  const visitedNodes = new Set();
  let count = 0;
  const records = selectionPhysicalBoundsRecords(rendered);
  const diagnostic = {
    owner_id: ownerId,
    selected_record_id: String(rendered?.item?.id || ''),
    candidate_record_count: records.length,
    authoritative_pick_record_count: records.filter(record => record.authoritativePhysicalPick === true).length,
    candidate_root_names: records.map(record => String(record.object3d?.name || '')).filter(Boolean),
    candidate_root_visible_states: records.map(record => record.object3d?.visible !== false),
    state_objects_matching_owner_count: state.objects.filter(record => String(record?.uiSelectionOwnerId || '') === ownerId || explicitUiSelectionItemId(record) === ownerId).length,
    state_pick_records_matching_owner_count: state.pickRecords.filter(record => String(record?.uiSelectionOwnerId || '') === ownerId || explicitUiSelectionItemId(record) === ownerId).length,
    descendant_registered_identity_count: 0,
    visible_renderable_count: 0,
    accepted_renderable_count: 0,
    excluded_renderable_count: 0,
    exclusion_reasons: [],
    explicit_helper_reasons: {},
  };
  diagnostic.records = records.map(record => {
    let childMeshCount = 0;
    let registeredIdentityCount = 0;
    record.object3d?.traverse?.(node => {
      if (node?.isMesh) childMeshCount += 1;
      if (state.pickIdentityByObject?.has?.(node)) registeredIdentityCount += 1;
    });
    diagnostic.descendant_registered_identity_count += registeredIdentityCount;
    return {
      record_id: String(record.item?.id || ''), link_name: exactSelectionLinkName(record.item),
      authoritativePhysicalPick: record.authoritativePhysicalPick === true,
      uiSelectionOwnerId: String(record.uiSelectionOwnerId || ''),
      object3d_name: String(record.object3d?.name || ''), object3d_visible: record.object3d?.visible !== false,
      child_mesh_count: childMeshCount,
    };
  });
  for (const record of records) {
    const physical = collectPhysicalVisibleBounds(record.object3d, { selectionOwnerId: ownerId, visitedNodes, selectionBounds: true, diagnostics: diagnostic });
    if (!physical.bounds) continue;
    bounds.union(physical.bounds);
    count += physical.count;
  }
  const finite = count ? finiteBox3(bounds) : null;
  diagnostic.exclusion_reasons = [...new Set(diagnostic.exclusion_reasons)];
  diagnostic.finite_bounds = Boolean(finite);
  diagnostic.bounds_min = finite ? [finite.min.x, finite.min.y, finite.min.z] : null;
  diagnostic.bounds_max = finite ? [finite.max.x, finite.max.y, finite.max.z] : null;
  const diagnosticKey = JSON.stringify(diagnostic);
  state.selectionBoundsDiagnosticKeys = state.selectionBoundsDiagnosticKeys || new Map();
  if (ownerId && state.selectionBoundsDiagnosticKeys.get(ownerId) !== diagnosticKey) {
    state.selectionBoundsDiagnosticKeys.set(ownerId, diagnosticKey);
    console.warn?.(`Product View selection_bounds: ${diagnosticKey}`);
  }
  return { count: finite ? count : 0, bounds: finite, bounds_json: box3ToJson(finite) };
}

function visibleRenderableBounds(object) {
  if (!object || object.visible === false) return null;
  object.updateWorldMatrix(true, true);
  const bounds = new THREE.Box3();
  let hasVisible = false;
  const visit = node => {
    if (!node || node.visible === false) return;
    if (node.isMesh || node.isLine || node.isLineSegments || node.isPoints || node.isSprite) {
      const nodeBounds = finiteBox3(new THREE.Box3().setFromObject(node));
      if (nodeBounds) {
        bounds.union(nodeBounds);
        hasVisible = true;
      }
    }
    for (const child of node.children || []) visit(child);
  };
  visit(object);
  return hasVisible ? finiteBox3(bounds) : null;
}
function objectHasVisibleRenderable(object) {
  return Boolean(visibleRenderableBounds(object));
}
function assemblyRootHiddenForFit(root) {
  const data = root?.userData || {};
  return root?.visible === false
    || data.exclude_from_fit_bounds === true
    || data.debug_only === true
    || data.diagnostic_only === true
    || data.helper_overlay === true;
}
function collectPhysicalAssemblyBounds() {
  if (!(state.assemblyRoots || []).length || !THREE?.Box3) return { count: 0, bounds: null, bounds_json: null, roots: [] };
  const bounds = new THREE.Box3();
  let count = 0;
  const roots = [];
  for (const root of state.assemblyRoots || []) {
    if (!root || assemblyRootHiddenForFit(root)) continue;
    const rootBounds = visibleRenderableBounds(root);
    if (!rootBounds) continue;
    bounds.union(rootBounds);
    count += 1;
    roots.push({ name: root.name || '', robot_render_mode: root.userData?.robot_render_mode || '', bounds: box3ToJson(rootBounds) });
  }
  const finite = count ? finiteBox3(bounds) : null;
  return { count: finite ? count : 0, bounds: finite, bounds_json: box3ToJson(finite), roots };
}
function itemHiddenForFit(item) {
  return item?.visible === false || item?.hidden === true || item?.rendered === false || item?.enabled === false;
}
function renderedStatusForFit(rendered) {
  return String(rendered?.renderInfo?.render_status || rendered?.item?.renderInfo?.render_status || rendered?.item?.mesh_status || '').toLowerCase();
}
function isRequiredMeshDebugFallback(rendered) {
  const status = renderedStatusForFit(rendered);
  if (status === 'required_mesh_failed_debug_fallback') return true;
  let debug = false;
  rendered?.object3d?.traverse?.(child => {
    if (debug) return;
    const childStatus = String(child?.userData?.render_status || child?.userData?.renderInfo?.render_status || '').toLowerCase();
    if (childStatus === 'required_mesh_failed_debug_fallback') debug = true;
  });
  return debug;
}
function isLoadingPlaceholderHidden(rendered) {
  return renderedStatusForFit(rendered) === 'mesh_loading_required' && rendered?.fallback?.visible === false && !rendered?.meshObject;
}
function warnFitBlockerExcluded(rendered, reason, bounds = null, extra = {}) {
  const item = rendered?.item || {};
  const key = `${item.id || itemLabel(item)}:${reason}`;
  state._fitBlockerWarnings = state._fitBlockerWarnings || new Set();
  if (state._fitBlockerWarnings.has(key)) return;
  state._fitBlockerWarnings.add(key);
  appendViewerDiagnosticWarning(item, 'camera_framing_blocker_excluded', reason, {
    render_status: renderedStatusForFit(rendered),
    visual_bounds_status: item?.visual_bounds_status || '',
    excluded_fit_bounds: box3ToJson(bounds),
    ...extra,
  });
}
function computeFitBounds({ includeDebugFallbacks = false } = {}) {
  const normalBounds = new THREE.Box3();
  const fallbackBounds = new THREE.Box3();
  let hasNormal = false;
  let hasFallback = false;
  const deferredWarnings = [];
  const assemblyBounds = collectPhysicalAssemblyBounds();
  if (assemblyBounds.bounds) {
    normalBounds.union(assemblyBounds.bounds);
    hasNormal = true;
    state.physicalAssemblyBounds = assemblyBounds.bounds.clone();
  } else {
    state.physicalAssemblyBounds = null;
  }
  for (const rendered of state.objects) {
    if (!rendered?.object3d) continue;
    if (rendered.item?.exclude_from_fit_bounds === true || rendered.object3d.userData?.exclude_from_fit_bounds === true) {
      deferredWarnings.push([rendered, 'debug diagnostic excluded from camera fit bounds', null]);
      continue;
    }
    if ((!state.debugOverlaysVisible && isDebugOverlayItem(rendered.item)) || itemHiddenForFit(rendered.item) || rendered.object3d.visible === false) {
      deferredWarnings.push([rendered, 'hidden renderable excluded from camera fit bounds', null]);
      continue;
    }
    if (isLoadingPlaceholderHidden(rendered) || !objectHasVisibleRenderable(rendered.object3d)) {
      deferredWarnings.push([rendered, 'hidden loading placeholder excluded from camera fit bounds', null]);
      continue;
    }
    const itemBounds = visibleRenderableBounds(rendered.object3d);
    if (!itemBounds) continue;
    const isDebugFallback = isRequiredMeshDebugFallback(rendered);
    const isOversized = rendered.item?.visual_bounds_status === 'oversized';
    if (isDebugFallback || isOversized) {
      fallbackBounds.union(itemBounds);
      hasFallback = true;
      deferredWarnings.push([
        rendered,
        isDebugFallback
          ? 'required_mesh_failed_debug_fallback excluded from normal camera fit bounds'
          : 'oversized visual bounds excluded from normal camera fit bounds',
        itemBounds,
      ]);
      continue;
    }
    normalBounds.union(itemBounds);
    hasNormal = true;
  }
  const finiteNormal = hasNormal ? finiteBox3(normalBounds) : null;
  if (finiteNormal) {
    for (const warning of deferredWarnings) warnFitBlockerExcluded(...warning);
    maybeWarnSceneBoundsExceedWorkspace(finiteNormal);
    state.finalPhysicalFitBounds = finiteNormal.clone();
    return finiteNormal;
  }
  const finiteFallback = (includeDebugFallbacks || hasFallback) ? finiteBox3(fallbackBounds) : null;
  if (finiteFallback) {
    for (const warning of deferredWarnings) warnFitBlockerExcluded(...warning);
    const fallbackKey = `${sceneId() || sceneDisplayName()}:fallback_fit_bounds`;
    state._fitBlockerWarnings = state._fitBlockerWarnings || new Set();
    if (!state._fitBlockerWarnings.has(fallbackKey)) {
      state._fitBlockerWarnings.add(fallbackKey);
      appendViewerDiagnosticWarning({ id: sceneId(), label: sceneDisplayName() }, 'camera_framing_blocker_excluded', 'camera fit fell back to debug or oversized geometry because no normal physical bounds were available', {
        fallback_fit_bounds: box3ToJson(finiteFallback),
      });
    }
    maybeWarnSceneBoundsExceedWorkspace(finiteFallback);
    state.finalPhysicalFitBounds = finiteFallback.clone();
    return finiteFallback;
  }
  state.finalPhysicalFitBounds = null;
  return null;
}
function computeRenderedBounds() {
  return computeFitBounds({ includeDebugFallbacks: true });
}
function box3ToJson(box) {
  if (!box || box.isEmpty() || !finiteBox3(box)) return null;
  const size = box3Dimensions(box);
  const center = new THREE.Vector3();
  box.getCenter(center);
  return {
    min: { x: box.min.x, y: box.min.y, z: box.min.z },
    max: { x: box.max.x, y: box.max.y, z: box.max.z },
    center: { x: center.x, y: center.y, z: center.z },
    dimensions: { x: size.x, y: size.y, z: size.z },
  };
}
function box3Dimensions(box) {
  const size = new THREE.Vector3();
  if (box && !box.isEmpty()) box.getSize(size);
  return size;
}
function finiteBox3(box) {
  if (!box || box.isEmpty()) return null;
  const values = [box.min.x, box.min.y, box.min.z, box.max.x, box.max.y, box.max.z];
  if (!values.every(Number.isFinite)) return null;
  return box;
}
function dimensionsVectorFrom(value) {
  if (!value) return null;
  let raw = value;
  if (!Array.isArray(raw) && typeof raw === 'object') raw = raw.dimensions || raw.size || raw.extents || [raw.x || raw.width, raw.y || raw.depth, raw.z || raw.height];
  if (!Array.isArray(raw)) return null;
  const dims = raw.slice(0, 3).map(Number);
  return dims.every(n => Number.isFinite(n) && n > 0) ? new THREE.Vector3(dims[0], dims[1], dims[2]) : null;
}
function expectedDimensionsOf(item) {
  return dimensionsVectorFrom(item?.expected_dimensions_m || item?.expected_dimensions || item?.dimensions_m || primitiveOf(item));
}
function meshContractCategoryOf(item) {
  const identity = [
    item?.source_layer,
    item?.active_visual_source,
    item?.role,
    item?.category,
    item?.type,
    item?.id,
    itemLabel(item || {}),
  ].map(value => String(value || '').toLowerCase()).join(' ');
  if (/\b(table|workbench|bench)\b/.test(identity)) return 'table';
  if (/\b(camera|sensor|realsense|rgbd|vision)\b/.test(identity)) return 'camera';
  if (/\b(object|part|product|item)\b/.test(identity)) return 'object';
  if (/\b(environment|asset|fixture)\b/.test(identity)) return 'environment';
  if (/\b(robot|arm|manipulator|ur3|ur5|ur10|ur_|universal_robot)\b/.test(identity)) return 'robot';
  if (/\b(tool|gripper|eef|suction|robotiq|airpick)\b/.test(identity)) return 'tool';
  if (/\b(zone|overlay|helper|diagnostic|fov|bounds|reachability|collision)\b/.test(identity)) return 'helper';
  return 'core';
}

function meshUnitAutoscaleAllowed(item) {
  if (!item || item.allow_mesh_unit_autoscale !== true || !item.expected_dimensions_m) return false;
  if (item.source_kind === 'generated_preview' || item.active_visual_source === 'generated_preview') return false;
  const category = meshContractCategoryOf(item);
  if (!['table', 'camera', 'object', 'environment'].includes(category)) return false;
  const identity = [
    item.source_layer,
    item.active_visual_source,
    item.role,
    item.category,
    item.type,
    item.id,
    item.link,
    item.object_name,
    item.display_name,
    item.source_path,
    item.mesh_path,
    item.package_uri,
    item.original_mesh_uri,
    itemLabel(item || {}),
  ].map(value => String(value || '').toLowerCase()).join(' ');
  return !/\b(robot|arm|manipulator|ur3|ur5|ur10|ur_|universal_robot|robotiq|gripper|suction|airpick|tool|eef|end_effector)\b/.test(identity);
}
const MESH_BOUNDS_COORDINATE_SPACE_CONTRACT = Object.freeze({
  version: 1,
  raw_mesh_geometry: 'mesh_local_before_visual_origin_or_world_transform',
  unit_correction: 'uniform_scale_applied_once_at_loaded_mesh_object',
  validation_bounds: 'authored_visual_local_after_unit_correction',
  diagnostic_world_bounds: 'scene_world_after_visual_origin_and_item_pose',
});
const MESH_OVERSIZED_RATIO_THRESHOLD = 3;

// Coordinate-space contract shared by maybeApplyMeshUnitAutoscale() and
// diagnoseLoadedMeshBounds(): geometry is measured in the authored visual
// local axes.  Visual-origin translation/rotation and every parent/world pose
// are removed, while the explicitly authored visual scale and loader-provided
// child hierarchy are retained.  This prevents a rotated AABB in world axes
// from being mistaken for a mesh-unit defect.
function measureLoadedMeshBoundsInAuthoredLocalSpace(visualRoot) {
  if (!visualRoot || !THREE?.Box3 || !THREE?.Matrix4) return null;
  visualRoot.updateWorldMatrix(true, true);
  const inverseVisualRootWorld = visualRoot.matrixWorld.clone().invert();
  const visualScale = new THREE.Matrix4().makeScale(
    Number(visualRoot.scale?.x ?? 1),
    Number(visualRoot.scale?.y ?? 1),
    Number(visualRoot.scale?.z ?? 1),
  );
  const bounds = new THREE.Box3();
  let foundGeometry = false;
  visualRoot.traverse(node => {
    const geometry = node?.geometry;
    if (!geometry) return;
    if (!geometry.boundingBox) geometry.computeBoundingBox?.();
    const geometryBounds = finiteBox3(geometry.boundingBox?.clone?.());
    if (!geometryBounds) return;
    const relativeToVisualRoot = inverseVisualRootWorld.clone().multiply(node.matrixWorld);
    const authoredLocalMatrix = visualScale.clone().multiply(relativeToVisualRoot);
    const transformed = geometryBounds.clone().applyMatrix4(authoredLocalMatrix);
    if (!finiteBox3(transformed)) return;
    bounds.union(transformed);
    foundGeometry = true;
  });
  return foundGeometry ? finiteBox3(bounds) : null;
}
function measureLoadedMeshBoundsInWorldSpace(visualRoot) {
  if (!visualRoot || !THREE?.Box3) return null;
  visualRoot.updateWorldMatrix(true, true);
  return finiteBox3(new THREE.Box3().setFromObject(visualRoot));
}
function meshDimensionComparison(expected, dimensions) {
  if (!expected || !dimensions) return null;
  const axes = ['x', 'y', 'z'];
  if (axes.some(axis => !Number.isFinite(dimensions[axis]) || dimensions[axis] <= 1e-9 || !Number.isFinite(expected[axis]) || expected[axis] <= 1e-9)) return null;
  const axisRatios = Object.fromEntries(axes.map(axis => [axis, dimensions[axis] / expected[axis]]));
  const ratios = Object.values(axisRatios);
  const maxRatio = Math.max(...ratios);
  const minRatio = Math.min(...ratios);
  const uniformRatio = minRatio > 0 ? maxRatio / minRatio : Infinity;
  return {
    axisRatios,
    maxRatio,
    minRatio,
    uniformRatio,
    oversized: maxRatio > MESH_OVERSIZED_RATIO_THRESHOLD,
  };
}
function meshUnitCorrectionPayload(source, confidence, rawLocalBounds, correctedLocalBounds, scale, axisRatios, targetRatio, applied = false) {
  return {
    source,
    confidence,
    coordinate_space_contract: MESH_BOUNDS_COORDINATE_SPACE_CONTRACT,
    raw_mesh_local_bounds: box3ToJson(rawLocalBounds),
    corrected_mesh_local_bounds: box3ToJson(correctedLocalBounds),
    // Compatibility aliases retained for existing status consumers.
    native_bounds: box3ToJson(rawLocalBounds),
    corrected_bounds: box3ToJson(correctedLocalBounds),
    scale,
    axis_ratios: axisRatios,
    target_ratio: targetRatio,
    applied,
    applied_once_at: applied ? 'loaded_mesh_object.scale' : null,
  };
}
function maybeApplyMeshUnitAutoscale(
  item,
  meshLocalNode,
  visualRootOrRawBounds,
  rawLocalBoundsOrMeshUri,
  meshUriMaybe,
) {
  // Compatibility: the legacy helper form already supplied authored-local bounds.
  // Production loading supplies the visual root and raw authored-local bounds separately.
  const legacyAuthoredLocalBoundsCall =
    meshUriMaybe === undefined && typeof rawLocalBoundsOrMeshUri === 'string';
  const meshObject = meshLocalNode;
  const visualRoot = legacyAuthoredLocalBoundsCall ? meshObject : visualRootOrRawBounds;
  const rawLocalBounds = legacyAuthoredLocalBoundsCall
    ? visualRootOrRawBounds
    : rawLocalBoundsOrMeshUri;
  const meshUri = legacyAuthoredLocalBoundsCall ? rawLocalBoundsOrMeshUri : meshUriMaybe;
  if (!meshUnitAutoscaleAllowed(item)) return false;
  // A successful decision is immutable for this loaded visual.  This makes an
  // accidental second call harmless instead of multiplying the correction.
  if (item.mesh_unit_correction?.applied === true) return false;
  const expected = expectedDimensionsOf(item);
  const finiteRaw = finiteBox3(rawLocalBounds || measureLoadedMeshBoundsInAuthoredLocalSpace(visualRoot));
  const dims = finiteRaw ? box3Dimensions(finiteRaw) : null;
  const comparison = meshDimensionComparison(expected, dims);
  if (!comparison) return false;
  const { axisRatios, maxRatio, minRatio, uniformRatio } = comparison;
  const targetRatio = [1000, 100].find(target =>
    Math.abs(maxRatio - target) / target <= 0.2 &&
    Math.abs(minRatio - target) / target <= 0.2 &&
    uniformRatio <= 1.25);
  if (!targetRatio) {
    item.mesh_unit_correction = meshUnitCorrectionPayload(
      'viewer_expected_dimensions_m',
      'rejected_non_uniform_or_unclear_ratio',
      finiteRaw,
      finiteRaw,
      1.0,
      axisRatios,
      null,
      false,
    );
    appendRuntimeWarning(item, meshUri, `mesh unit autoscale rejected: authored-local bounds do not have a clear uniform 100x or 1000x ratio to expected_dimensions_m (uniform_ratio=${uniformRatio.toFixed(3)})`, 'mesh_unit_autoscale_rejected', item.mesh_unit_correction);
    return false;
  }
  const scale = targetRatio === 1000 ? 0.001 : 0.01;
  meshObject.scale.multiplyScalar(scale);
  meshObject.updateMatrix?.();
  visualRoot.updateWorldMatrix?.(true, true);
  visualRoot.updateMatrixWorld?.(true);
  let correctedLocalBounds;
  if (legacyAuthoredLocalBoundsCall) {
    correctedLocalBounds = new THREE.Box3();
    correctedLocalBounds.min.x = finiteRaw.min.x * scale;
    correctedLocalBounds.min.y = finiteRaw.min.y * scale;
    correctedLocalBounds.min.z = finiteRaw.min.z * scale;
    correctedLocalBounds.max.x = finiteRaw.max.x * scale;
    correctedLocalBounds.max.y = finiteRaw.max.y * scale;
    correctedLocalBounds.max.z = finiteRaw.max.z * scale;
  } else {
    correctedLocalBounds = measureLoadedMeshBoundsInAuthoredLocalSpace(visualRoot);
  }
  const confidence = targetRatio === 1000 ? 'auto_detected_mm_to_m' : 'auto_detected_cm_to_m';
  item.mesh_unit_correction = meshUnitCorrectionPayload(
    'viewer_expected_dimensions_m',
    confidence,
    finiteRaw,
    correctedLocalBounds,
    scale,
    axisRatios,
    targetRatio,
    true,
  );
  item.visual_bounds_status = 'corrected_by_local_unit_scale';
  appendRuntimeWarning(item, meshUri, `mesh unit autoscale applied once at the loaded mesh node: authored-local bounds matched a clear ${targetRatio}x ratio to expected_dimensions_m`, 'mesh_unit_autoscale_applied', item.mesh_unit_correction);
  return true;
}

function isCoreMeshContractItem(item) {
  return !['helper'].includes(meshContractCategoryOf(item)) && !isZone(item);
}
function warnLoadedMeshBounds(item, code, reason, extra = {}) {
  item.visual_bounds_status = code === 'loaded_mesh_oversized' ? 'oversized' : code === 'loaded_mesh_collapsed' ? 'collapsed' : 'invalid';
  item.loaded_mesh_bounds_reason_code = code;
  appendViewerDiagnosticWarning(item, code, reason, {
    mesh_uri: displayMeshUri(item),
    loaded_mesh_local_bounds: item.loaded_mesh_local_bounds,
    loaded_mesh_bounds: item.loaded_mesh_bounds,
    loaded_mesh_world_bounds: item.loaded_mesh_world_bounds,
    loaded_mesh_bounds_coordinate_space: item.loaded_mesh_bounds_coordinate_space,
    expected_dimensions_m: item.expected_dimensions_m,
    mesh_contract_category: meshContractCategoryOf(item),
    ...extra,
  });
}
function diagnoseLoadedMeshBounds(item, visualRoot, rendered, authoredLocalBounds = null) {
  const localBounds = finiteBox3(authoredLocalBounds || measureLoadedMeshBoundsInAuthoredLocalSpace(visualRoot));
  rendered.object3d.updateWorldMatrix(true, true);
  const worldBounds = measureLoadedMeshBoundsInWorldSpace(visualRoot);
  item.loaded_mesh_local_bounds = box3ToJson(localBounds);
  // Compatibility alias: loaded_mesh_bounds now explicitly means authored-local.
  item.loaded_mesh_bounds = item.loaded_mesh_local_bounds;
  item.loaded_mesh_world_bounds = box3ToJson(worldBounds);
  item.loaded_mesh_bounds_coordinate_space = MESH_BOUNDS_COORDINATE_SPACE_CONTRACT;
  item.visual_bounds_status = item.mesh_unit_correction?.scale && item.mesh_unit_correction.scale !== 1.0 ? 'corrected_by_local_unit_scale' : 'valid';
  const expected = expectedDimensionsOf(item);
  const dims = localBounds ? box3Dimensions(localBounds) : null;
  const worldDims = worldBounds ? box3Dimensions(worldBounds) : null;
  const tiny = 1e-9;
  if (!localBounds || !worldBounds || !dims || !worldDims) {
    if (isCoreMeshContractItem(item)) warnLoadedMeshBounds(item, 'loaded_mesh_bounds_invalid', 'loaded mesh produced empty or non-finite bounds');
    return !isCoreMeshContractItem(item);
  }
  maybeWarnSupportSurfaceSemantics(item, dims);
  const collapsedAxes = ['x', 'y', 'z'].filter(axis => dims[axis] <= tiny || worldDims[axis] <= tiny);
  if (collapsedAxes.length) {
    if (isCoreMeshContractItem(item)) warnLoadedMeshBounds(item, 'loaded_mesh_collapsed', `loaded mesh bounds are zero-volume or collapsed on ${collapsedAxes.join(', ')}`, { collapsed_axes: collapsedAxes });
    return !isCoreMeshContractItem(item);
  }
  if (!expected) return true;
  const comparison = meshDimensionComparison(expected, dims);
  if (!comparison) return true;
  const { axisRatios, maxRatio, uniformRatio, oversized } = comparison;
  item.loaded_mesh_axis_ratios = axisRatios;
  item.loaded_mesh_maximum_ratio = maxRatio;
  item.loaded_mesh_uniform_ratio = uniformRatio;
  const category = meshContractCategoryOf(item);
  if (oversized) {
    if (['table', 'camera', 'object', 'environment'].includes(category)) warnLoadedMeshBounds(item, 'loaded_mesh_oversized', `loaded mesh authored-local dimensions exceed expected_dimensions_m by the strict ${MESH_OVERSIZED_RATIO_THRESHOLD}x contract (max_axis_ratio=${maxRatio.toFixed(3)}, uniform_ratio=${uniformRatio.toFixed(3)})`, {
      expected_dimensions: { x: expected.x, y: expected.y, z: expected.z },
      loaded_local_dimensions: { x: dims.x, y: dims.y, z: dims.z },
      loaded_world_dimensions: { x: worldDims.x, y: worldDims.y, z: worldDims.z },
      axis_ratios: axisRatios,
      max_axis_ratio: maxRatio,
      uniform_ratio: uniformRatio,
      oversized_threshold: MESH_OVERSIZED_RATIO_THRESHOLD,
    });
    else if (Math.abs(maxRatio - 1000) < 100 || Math.abs(maxRatio - 0.001) < 0.001) item.visual_bounds_status = 'corrected_by_local_unit_scale';
  }
  return item.visual_bounds_status !== 'oversized';
}

function workspaceDimensionsOf(sceneJson) {
  return dimensionsVectorFrom(sceneJson?.workspace?.dimensions_m || sceneJson?.workspace?.size_m || sceneJson?.workspace_dimensions_m || sceneJson?.scene?.workspace_dimensions_m);
}
function maybeWarnSceneBoundsExceedWorkspace(bounds) {
  const workspace = workspaceDimensionsOf(state.sceneJson);
  if (!workspace || state._sceneBoundsExceededWarned) return;
  const dims = box3Dimensions(bounds);
  if (dims.x > workspace.x || dims.y > workspace.y || dims.z > workspace.z) {
    state._sceneBoundsExceededWarned = true;
    appendViewerDiagnosticWarning({ id: sceneId(), label: sceneDisplayName() }, 'scene_bounds_exceed_workspace', 'rendered scene bounds exceed configured workspace dimensions', {
      scene_bounds: box3ToJson(bounds),
      workspace_dimensions_m: { x: workspace.x, y: workspace.y, z: workspace.z },
    });
  }
}
function cameraDirectionForPreset(preset = 'isometric') {
  const raw = CAMERA_PRESET_DIRECTIONS[String(preset || 'isometric').toLowerCase()] || CAMERA_PRESET_DIRECTIONS.isometric;
  return new THREE.Vector3(raw[0], raw[1], raw[2]).normalize();
}
function applyCameraClipping(camera, radius, distance) {
  const safeRadius = Math.max(Number(radius) || 0, 0.01);
  const safeDistance = Math.max(Number(distance) || 0, safeRadius * 2);
  camera.near = Math.max(0.001, Math.min(safeRadius / 200, safeDistance / 50));
  camera.far = Math.max(camera.near + 1, safeDistance + safeRadius * 8, safeRadius * 20, 100);
  camera.updateProjectionMatrix();
}
function frameScene(bounds, { preset = 'isometric' } = {}) {
  const { camera, controls } = state.three;
  const finiteBounds = finiteBox3(bounds);
  if (!camera || !controls || !finiteBounds) return false;
  const center = new THREE.Vector3();
  const sphere = new THREE.Sphere();
  finiteBounds.getCenter(center);
  finiteBounds.getBoundingSphere(sphere);
  if (![center.x, center.y, center.z, sphere.radius].every(Number.isFinite)) return false;
  const radius = Math.max(sphere.radius, MIN_FRAME_RADIUS);
  const direction = cameraDirectionForPreset(preset);
  const multiplier = preset === 'top' ? 2.15 : (preset === 'robot' ? 2.05 : FRAME_DISTANCE_MULTIPLIER);
  const distance = Math.max(radius * multiplier, MIN_FRAME_RADIUS * multiplier);
  camera.position.copy(center).addScaledVector(direction, distance);
  applyCameraClipping(camera, radius, distance);
  if (![camera.position.x, camera.position.y, camera.position.z, camera.near, camera.far].every(Number.isFinite)) return false;
  controls.target.copy(center);
  controls.update();
  state.lastFrameBounds = finiteBounds.clone();
  if (el.resetView) el.resetView.disabled = false;
  if (el.cameraPreset) el.cameraPreset.disabled = false;
  return true;
}
function stableSceneCameraKey() {
  return [
    state.sceneJson?.scene?.root,
    state.sceneJson?.scene?.canonical_root,
    state.sceneJson?.scene?.id,
    state.sceneJson?.scene_id,
    state.sceneJson?.scene?.generation_version,
    state.sceneJson?.scene_generation_token,
    state.sourceWebSceneFile,
    state.sceneJson?.schema_version,
  ].map(value => String(value || '')).filter(Boolean).join('|');
}
function cancelInitialCameraFitRetry() {
  if (state.initialCameraFit?.pendingRetry) clearTimeout(state.initialCameraFit.pendingRetry);
  state.initialCameraFit.pendingRetry = null;
}
function beginInitialCameraFitForCurrentScene() {
  const sceneKey = stableSceneCameraKey();
  if (state.initialCameraFit?.sceneKey === sceneKey) return;
  cancelInitialCameraFitRetry();
  state.initialCameraFit = { sceneKey, done: false, attempts: 0, pendingRetry: null, userControlled: false };
}
function markCameraUserControlled() {
  if (!state.initialCameraFit) return;
  state.initialCameraFit.userControlled = true;
  cancelInitialCameraFitRetry();
}
function reportFitCellNoGeometry(message = 'No visible physical geometry to frame') {
  state.editorError = message;
  pushEditorEvent('fit_cell_unavailable', { message });
  if (el.error) { el.error.textContent = message; el.error.hidden = false; }
}
function resetView({ userInitiated = true, preset = 'isometric' } = {}) {
  if (userInitiated) markCameraUserControlled();
  const physical = collectPhysicalVisibleBounds(state.three.scene);
  if (!physical.bounds || !frameScene(physical.bounds, { preset })) { reportFitCellNoGeometry(); return false; }
  if (state.editorError === 'No visible physical geometry to frame') clearError();
  return true;
}
function reportFitSelectionFallback(message) {
  state.editorError = message;
  pushEditorEvent('fit_selection_fallback', { message, itemId: state.selected || '' });
  if (el.error) { el.error.textContent = message; el.error.hidden = false; }
}
function fitSelection() {
  markCameraUserControlled();
  const selectedId = String(state.selected || '').trim();
  const rendered = selectedId ? renderedById(selectedId) : null;
  const fallback = message => { reportFitSelectionFallback(message); resetView({ userInitiated: false }); return false; };
  if (!selectedId) return fallback('No physical item selected; fitting the workcell');
  if (!rendered?.object3d) return fallback('Selected item has no visible physical geometry; fitting the workcell');
  const physical = collectPhysicalVisibleBounds(rendered.object3d);
  if (!physical.bounds || !frameScene(physical.bounds)) return fallback('Selected item has no visible physical geometry; fitting the workcell');
  if (state.editorError === 'No physical item selected; fitting the workcell' || state.editorError === 'Selected item has no visible physical geometry; fitting the workcell') clearError();
  pushEditorEvent('fit_selection', { itemId: selectedId, physicalRenderableCount: physical.count, bounds: physical.bounds_json });
  return true;
}
function attemptInitialCameraFit({ allowRetry = true } = {}) {
  const fit = state.initialCameraFit;
  if (!fit || fit.done || fit.userControlled || fit.sceneKey !== stableSceneCameraKey()) return false;
  cancelInitialCameraFitRetry();
  fit.attempts += 1;
  const physical = collectPhysicalVisibleBounds(state.three.scene);
  if (physical.bounds && frameScene(physical.bounds)) {
    fit.done = true;
    return true;
  }
  if (allowRetry && fit.attempts < 2) {
    fit.pendingRetry = setTimeout(() => attemptInitialCameraFit({ allowRetry: false }), 250);
  } else {
    reportFitCellNoGeometry('No visible physical geometry available for initial framing');
  }
  return false;
}
function triggerInitialCameraFitAfterSceneReady() {
  return attemptInitialCameraFit({ allowRetry: false });
}
function applyCameraPreset(preset) {
  const selectedPreset = String(preset || '').toLowerCase();
  if (!CAMERA_PRESET_DIRECTIONS[selectedPreset]) return false;
  return resetView({ userInitiated: true, preset: selectedPreset });
}
function scheduleInitialCameraFitRetry() {
  const fit = state.initialCameraFit;
  if (!fit || fit.done || fit.userControlled || fit.pendingRetry || fit.attempts >= 2) return;
  fit.pendingRetry = setTimeout(() => attemptInitialCameraFit({ allowRetry: false }), 0);
}
if (el.resetView) {
  el.resetView.title = RESET_VIEW_TITLE;
  el.resetView.setAttribute('aria-label', 'Fit Scene / Reset View');
}

function clearLabels() {
  if (el.labelLayer) el.labelLayer.innerHTML = '';
}
function disposeOwnedObject3d(object3d, seen = new Set()) {
  if (!object3d || seen.has(object3d)) return;
  seen.add(object3d);
  if (Array.isArray(object3d.children)) {
    for (const child of [...object3d.children]) disposeOwnedObject3d(child, seen);
  }
  object3d.geometry?.dispose?.();
  const material = object3d.material;
  if (Array.isArray(material)) {
    for (const entry of material) entry?.dispose?.();
  } else {
    material?.dispose?.();
  }
}
function resetSceneLifecycleState() {
  cancelPlacement();
  robotPreviewLoadToken += 1;
  physicalLoadToken += 1;
  cancelInitialCameraFitRetry();
  state.objects = [];
  state.pickRecords = [];
  state.pickIdentityByObject = new WeakMap();
  state.selectionIdentityIndex = null;
  state.physicalEditBindings.clear();
  state.lastRaycastHitCount = 0;
  state.lastRaycastCandidateIds = [];
  state.lastCanvasSelectedItemId = '';
  state.lastCanvasPickReason = '';
  state.lastCanvasPickDiagnostic = null;
  state.lastFailedCanvasPickDiagnostic = null;
  state.assemblyRoots = [];
  state.robotAssemblyDiagnostics = [];
  state.robotAssemblyRenderDiagnostics = {};
  state.robotUrdfPreviewDiagnostics = {};
  state.physicalAssemblyBounds = null;
  state.finalPhysicalFitBounds = null;
  state.resolvedFramePoses.clear();
  state.frameLookup = new Map();
  state.lastFrameBounds = null;
  removeSelectionHighlight();
  state._sceneBoundsExceededWarned = false;
  state.diagnosticKeys = new Set();
  state._fitBlockerWarnings = new Set();
  state._generatedUrdfFramePoseWarnings = new Set();
  state._supportSurfaceSemanticWarnings = new Set();
  state.ignoredSelectionKeys = new Set();
  state.robotPreviewResult = null;
  state.initialPosePreview = { active: false, robotId: '', sceneKey: '' };
  state.web3dReadiness = { state: 'scene_loading', terminal: false, terminalState: '', terminalNavigationKey: web3dNavigationKey(), terminalEmissionCount: 0, statusSequence: 0, required: {}, pending: new Set(), failed: false, failure: null };
}
function clearSceneObjects() {
  const scene = state.three.scene;
  const previousAssemblyRoots = [...(state.assemblyRoots || [])];
  const previousObjects = [...(state.objects || [])];
  resetSceneLifecycleState();
  if (scene) {
    const disposed = new Set();
    for (const root of previousAssemblyRoots) {
      scene.remove(root);
      disposeOwnedObject3d(root, disposed);
    }
    for (const rendered of previousObjects) {
      scene.remove(rendered.object3d);
      disposeOwnedObject3d(rendered.object3d, disposed);
    }
  }
  clearLabels();
  if (el.showInitialPose) el.showInitialPose.checked = false;
  setInitialPosePreviewUi(false);
  if (el.resetView) el.resetView.disabled = true;
  renderSceneSummary();
}
function renderScene(items) {
  clearSceneObjects();
  const selectionIndex = rebuildSelectionIdentityIndex(state.sceneJson || {});
  const ownersByType = type => selectionIndex.selectionOwners.filter(owner => owner.type === type).map(owner => owner.id);
  const preview = state.sceneJson?.robot_preview || {};
  console.info?.(`Product View selection owners: count=${selectionIndex.selectionOwners.length} robot=${preview.selection_robot_owner_id || 'missing'} tool=${preview.selection_tool_owner_id || 'missing'} camera=${ownersByType('camera').join(',') || 'missing'} support_surface=${ownersByType('support_surface').join(',') || 'missing'}`);
  state.frameLookup = parseSceneFrames(state.sceneJson || {});
  state.resolvedFramePoses.clear();
  beginWeb3dSceneReadiness(items);
  state.dirtyTransforms.clear();
  state.undoStack = [];
  state.redoStack = [];
  detachTransformGizmo();
  updateDirtyState();
  const scene = state.three.scene;
  state.robotUrdfPreviewDiagnostics = {};
  const urdfPreviewActive = isExpandedUrdfRobotPreview(state.sceneJson?.robot_preview);
  const diagnosticOnlyItems = items.filter(isDiagnosticOnlyItem);
  const overlayPolicyItems = items.filter(isOverlayPolicyItem);
  const primaryItems = items.filter(isPrimaryRenderableItem);
  const robotToolGeneratedUrdfItems = items.filter(isRobotToolGeneratedUrdfMeshVisualItem);
  const policyEligibleRobotToolGeneratedUrdfItems = robotToolGeneratedUrdfItems.filter(item => isDiagnosticOnlyItem(item) || isPrimaryRenderableItem(item));
  // Legacy static guard: handled: new Set(robotToolGeneratedUrdfItems)
  // Legacy static guard: skipped_legacy_generated_urdf_visual_count: robotToolGeneratedUrdfItems.length
  if (urdfPreviewActive) {
    for (const item of policyEligibleRobotToolGeneratedUrdfItems) {
      item.workcell_web_render_pose_mode = 'expanded_urdf_loader_skip_flattened_row';
      item.expanded_urdf_loader_skip_reason = 'expanded URDF mode renders robot/tool only through loadRobotPreview and URDFLoader; flattened row transforms are diagnostics only';
      item.baked_world_visual_pose_diagnostic_only = Boolean(item.baked_world_visual_pose || item.expected_visual_pose || item.baked_world_visual_matrix || item.visual_origin || item.robot_world_pose);
    }
  }
  const assemblyBuild = urdfPreviewActive ? { handled: new Set(policyEligibleRobotToolGeneratedUrdfItems), assemblies: [], renderDiagnostics: { skipped_flattened_urdf_visual_count: 0, diagnostic_only_record_count: diagnosticOnlyItems.length, overlay_record_count: overlayPolicyItems.length, primary_physical_record_count: primaryItems.length, assembled_hierarchy_rendered_mesh_count: 0, rendered_fk_visual_count: 0, skipped_legacy_generated_urdf_count: policyEligibleRobotToolGeneratedUrdfItems.length, skipped_legacy_generated_urdf_visual_count: policyEligibleRobotToolGeneratedUrdfItems.length, visible_duplicate_generated_urdf_count: 0, visible_tool0_fallback_count: 0, detached_robot_mesh_clusters: 0 } } : buildRobotAssemblies(primaryItems.concat(overlayPolicyItems));
  state.robotAssemblyDiagnostics = assemblyBuild.assemblies;
  state.robotAssemblyRenderDiagnostics = assemblyBuild.renderDiagnostics || {};
  if (urdfPreviewActive) loadExpandedUrdfRobotPreview(state.sceneJson.robot_preview);
  for (const item of items) {
    if (isDiagnosticOnlyItem(item)) continue;
    if (assemblyBuild.handled.has(item)) continue;
    if (usesAssembledUrdfHierarchy(item)) {
      state.robotAssemblyRenderDiagnostics.skipped_flattened_urdf_visual_count = Number(state.robotAssemblyRenderDiagnostics.skipped_flattened_urdf_visual_count || 0) + 1;
      continue;
    }
    const object3d = new THREE.Group();
    object3d.name = isGeneratedUrdfItem(item)
      ? `${item.id || itemLabel(item)}_link_frame_root`
      : `${item.id || itemLabel(item)}_object_root`;
    object3d.up.copy(THREE.Object3D.DEFAULT_UP);
    const primitive = primitiveOf(item);
    const fallback = isSensor(item) ? makeSensorMarker(item) : makePrimitiveMesh(item);
    if (fallback) {
      fallback.name = `${item.id || itemLabel(item)}_fallback`;
      assignItemUserData(fallback, item);
      object3d.add(fallback);
    }
    if (!applyPose(object3d, item)) continue;
    assignItemUserData(object3d, item);
    object3d.visible = state.debugOverlaysVisible || !isDebugOverlayItem(item);
    scene.add(object3d);
    const rendered = { item, object3d, fallback, labelEl: createLabelElement(item), originalTransform: transformOf(item), authoredBaselineTransform: cloneTransform(transformOf(item)) };
    const requiredMesh = itemRequiresMeshBackedVisual(item);
    const fallbackStatus = requiredMesh ? 'mesh_loading_required' : (fallback ? 'primitive_fallback' : 'no_physical_dimensions');
    const fallbackReason = requiredMesh ? 'required mesh is loading; primitive fallback hidden unless mesh load fails as debug geometry' : (fallback ? 'primitive geometry rendered while mesh loads or is unavailable' : 'no mesh or physical primitive dimensions were provided; Product View box fallback suppressed');
    if (fallback) fallback.visible = !requiredMesh;
    setRenderInfo(rendered, fallbackStatus, displayMeshUri(item), fallbackReason);
    state.objects.push(rendered);
    registerCanonicalPhysicalPick(rendered, fallback ? 'initial_physical_fallback' : 'initial_physical_root');
    maybeWarnSupportSurfaceSemantics(item);
    const category = readinessCategoryForItem(item);
    const operation = registerReadinessOperation(category ? [readinessKey(category, item)] : []);
    tryLoadMesh(item, rendered, fallback, operation);
  }
  bindExportedPhysicalTransformOwnership();
  renderFrameDebugOverlays();
  populateObjectList();
  updateLabels();
  renderSceneSummary();
  maybeEmitSceneReady();
}



function loadExpandedUrdfRobotPreview(preview) {
  const loadToken = ++robotPreviewLoadToken;
  const loadSceneId = sceneId();
  const readinessOperation = registerReadinessOperation([
    'robot_arm:expanded_urdf_loader',
    'attached_tool_gripper:expanded_urdf_loader',
  ], { robotPreviewLoadToken: loadToken, operationId: `expanded_urdf:${loadSceneId}:${loadToken}` });
  let staleCallbackLogged = false;
  const callbackIsCurrent = () => readinessOperationIsCurrent(readinessOperation) && loadSceneId === sceneId();
  const ignoreStaleCallback = (source = 'callback') => {
    if (!staleCallbackLogged) {
      console.debug?.('Ignored stale robot preview completion.', readinessOperationDiagnostic(readinessOperation, 'stale_replacement', {
        completion_source: source,
        callback_scene: loadSceneId,
        active_scene: sceneId(),
      }));
      staleCallbackLogged = true;
    }
    return true;
  };
  let requiredLoadDeadline = null;
  const finalizeRequiredLoad = (outcome, detail = {}) => {
    if (readinessOperation.completed) return false;
    if (!callbackIsCurrent()) return ignoreStaleCallback(detail.completion_source || outcome);
    readinessOperation.completed = true;
    if (requiredLoadDeadline !== null) clearTimeout(requiredLoadDeadline);
    for (const key of readinessOperation.pendingKeys) state.web3dReadiness?.pending?.delete(key);
    const terminalDetail = readinessOperationDiagnostic(readinessOperation, outcome, detail);
    if (outcome === 'success') {
      maybeEmitSceneReady();
      return true;
    }
    const err = detail.error instanceof Error ? detail.error : new Error(detail.reason || 'Expanded URDF required load failed');
    failExpandedUrdfReadiness({ ...readinessOperation, completed: false }, err, state.robotUrdfPreviewDiagnostics, terminalDetail);
    return true;
  };
  requiredLoadDeadline = setTimeout(() => {
    finalizeRequiredLoad('timeout', {
      completion_source: 'required_load_deadline',
      timeout_ms: REQUIRED_LOAD_DEADLINE_MS,
      reason: `Expanded URDF required load timed out after ${REQUIRED_LOAD_DEADLINE_MS}ms. Check URDF and mesh requests, then reload the scene.`,
    });
  }, REQUIRED_LOAD_DEADLINE_MS);
  const diagnostics = state.robotUrdfPreviewDiagnostics = {
    robot_render_mode: 'expanded_urdf_loader',
    robot_preview_loaded: false,
    robot_urdf_url: preview?.urdf_url || '',
    robot_loaded_link_count: 0,
    robot_loaded_joint_count: 0,
    robot_loaded_visual_count: 0,
    robot_missing_meshes: [],
    robot_joint_values_applied: preview?.joint_values || {},
    robot_expected_visual_count: 0,
    robotExpectedVisualCount: 0,
    robot_completed_visual_count: 0,
    robotCompletedVisualCount: 0,
    robot_failed_visual_count: 0,
    robotFailedVisualCount: 0,
    robot_root_link_count: 0,
    robotRootLinkCount: 0,
    robot_root_links: [],
    robotRootLinks: [],
    robot_disconnected_links: [],
    robotDisconnectedLinks: [],
    robot_duplicate_links: [],
    robotDuplicateLinks: [],
    robot_preview_lifecycle_state: 'idle',
    robotPreviewLifecycleState: 'idle',
    robot_preview_canonical_fallback_used: false,
    robotPreviewCanonicalFallbackUsed: false,
    skipped_legacy_generated_urdf_visual_count: state.robotAssemblyRenderDiagnostics?.skipped_legacy_generated_urdf_visual_count || state.robotAssemblyRenderDiagnostics?.skipped_legacy_generated_urdf_count || 0,
  };
  const mergeMissingDiagnosticsInPlace = (target, additions) => {
    if (!target || typeof target !== 'object') return target;
    for (const [key, value] of Object.entries(additions || {})) {
      if (target[key] === undefined) target[key] = value;
    }
    return target;
  };
  const rendererReady = rendererDiagnostics => {
    const lifecycle = String(rendererDiagnostics?.robot_preview_lifecycle_state || rendererDiagnostics?.robotPreviewLifecycleState || '');
    const loaded = rendererDiagnostics?.robot_preview_loaded === true || rendererDiagnostics?.robotPreviewLoaded === true;
    return lifecycle === 'ready' && loaded;
  };
  if (typeof loadRobotPreview !== 'function') {
    diagnostics.robot_preview_lifecycle_state = 'failed';
    diagnostics.robotPreviewLifecycleState = 'failed';
    diagnostics.robot_failed_visual_count = 1;
    diagnostics.robotFailedVisualCount = 1;
    diagnostics.robot_missing_meshes.push('urdf_robot_renderer module was not loaded');
    appendRuntimeWarning({}, preview?.urdf_url || '', 'expanded_urdf_loader failed: urdf_robot_renderer module was not loaded', 'expanded_urdf_loader_failed');
    finalizeRequiredLoad('failure', { completion_source: 'module_preflight', reason: 'expanded_urdf_loader failed: urdf_robot_renderer module was not loaded' });
    refreshWarnings();
    return { root: null, links: new Map(), joints: new Map(), diagnostics, ready: Promise.resolve(null) };
  }
  const previewResult = loadRobotPreview(preview, {
    sceneId: loadSceneId,
    scene: { add: root => { if (callbackIsCurrent()) state.three.scene?.add?.(root); } },
    assemblyRoots: { push: root => { if (callbackIsCurrent()) state.assemblyRoots.push(root); } },
    repoRootRelativeUrl,
    meshUriDiagnostic,
    rootName: `${sceneDisplayName()}_expanded_urdf_loader_robot`,
    skippedLegacyGeneratedUrdfVisualCount: diagnostics.skipped_legacy_generated_urdf_visual_count,
    onRobotLoaded: result => {
      if (!callbackIsCurrent()) return ignoreStaleCallback();
      const itemsByLink = new Map();
      for (const item of collectItems(state.sceneJson || {})) {
        if (!isGeneratedUrdfItem(item)) continue;
        const explicitLink = exactSelectionLinkName(item);
        if (explicitLink && !itemsByLink.has(explicitLink)) itemsByLink.set(explicitLink, item);
      }
      const robotLinks = new Set(asArray(preview?.expected_robot_visual_links || preview?.expectedRobotVisualLinks).map(value => String(value || '').trim()).filter(Boolean));
      const toolLinks = new Set(asArray(preview?.expected_tool_visual_links || preview?.expectedToolVisualLinks).map(value => String(value || '').trim()).filter(Boolean));
      const index = state.selectionIdentityIndex || rebuildSelectionIdentityIndex();
      const ownerFromExplicitContract = fields => {
        for (const field of fields) {
          const id = String(preview?.[field] || '').trim();
          if (id && index.itemById.has(id)) return id;
        }
        return '';
      };
      const robotOwnerId = ownerFromExplicitContract(['selection_robot_owner_id', 'selectionRobotOwnerId', 'robot_instance_id', 'robotInstanceId', 'robot_id', 'robotId']);
      const toolOwnerId = ownerFromExplicitContract(['selection_tool_owner_id', 'selectionToolOwnerId', 'tool_instance_id', 'toolInstanceId', 'tool_id', 'toolId', 'end_effector_id', 'endEffectorId']);
      const robotInstance = String(preview?.robot_instance_id || preview?.robotInstanceId || preview?.robot_id || preview?.robotId || 'robot').trim();
      const urdfLinkRoots = new Set(result.links.values());
      const loadedPickRecords = [];
      let boundNodeCount = 0;
      for (const [linkName, linkObject] of result?.links || []) {
        const exactLink = String(linkName || '').trim();
        const fixtureOwnerId = index.explicitUiIdByLink.get(exactLink) || '';
        const eligible = toolLinks.has(exactLink) || robotLinks.has(exactLink) || Boolean(fixtureOwnerId);
        if (!eligible || !linkObject || linkObject.visible === false) continue;
        const payloadItem = itemsByLink.get(exactLink);
        const inspectionId = `${loadSceneId}::expanded_urdf_inspection::${robotInstance}::${exactLink}`;
        const item = payloadItem || { id: inspectionId, link_name: exactLink, locked: true, editable: false, selectable: true, source_layer: 'expanded_urdf_inspection' };
        let ownerId = fixtureOwnerId;
        let resolution = fixtureOwnerId ? 'exact_link_explicit_ref' : 'exact_identity_fallback';
        if (toolLinks.has(exactLink) && toolOwnerId) { ownerId = toolOwnerId; resolution = 'tool_owner'; }
        else if (robotLinks.has(exactLink) && robotOwnerId) { ownerId = robotOwnerId; resolution = 'robot_owner'; }
        if (!callbackIsCurrent()) return ignoreStaleCallback();
        const record = registerPickRecord(item, linkObject, result.root || linkObject, {
          pickRecordSource: payloadItem ? 'payload_item' : 'expanded_urdf_inspection',
          authoritativePhysicalPick: true,
          uiSelectionOwnerId: ownerId,
          uiSelectionResolution: resolution,
        });
        if (record) loadedPickRecords.push(record);
        if (callbackIsCurrent()) boundNodeCount += bindExpandedUrdfPickRecordToSubtree(linkObject, record, urdfLinkRoots);
      }
      const robotPickRecords = loadedPickRecords.filter(record => robotLinks.has(exactSelectionLinkName(record.item)));
      const toolPickRecords = loadedPickRecords.filter(record => toolLinks.has(exactSelectionLinkName(record.item)));
      const localDiagnostics = {
        robot_pick_record_count: loadedPickRecords.length,
        robotPickRecordCount: loadedPickRecords.length,
        expanded_urdf_pick_record_count: loadedPickRecords.length,
        expandedUrdfPickRecordCount: loadedPickRecords.length,
        expanded_urdf_unique_link_record_count: new Set(loadedPickRecords.map(record => exactSelectionLinkName(record.item)).filter(Boolean)).size,
        expandedUrdfUniqueLinkRecordCount: new Set(loadedPickRecords.map(record => exactSelectionLinkName(record.item)).filter(Boolean)).size,
        expanded_urdf_bound_node_count: boundNodeCount,
        expandedUrdfBoundNodeCount: boundNodeCount,
        robot_pick_bound_descendant_node_count: boundNodeCount,
        robotPickBoundDescendantNodeCount: boundNodeCount,
        expanded_urdf_robot_pick_record_count: robotPickRecords.length,
        expandedUrdfRobotPickRecordCount: robotPickRecords.length,
        robot_pick_robot_record_count: robotPickRecords.length,
        robotPickRobotRecordCount: robotPickRecords.length,
        expanded_urdf_tool_pick_record_count: toolPickRecords.length,
        expandedUrdfToolPickRecordCount: toolPickRecords.length,
        robot_pick_tool_record_count: toolPickRecords.length,
        robotPickToolRecordCount: toolPickRecords.length,
        expanded_urdf_robot_pick_record_links: robotPickRecords.map(record => exactSelectionLinkName(record.item)),
        expandedUrdfRobotPickRecordLinks: robotPickRecords.map(record => exactSelectionLinkName(record.item)),
        expanded_urdf_tool_pick_record_links: toolPickRecords.map(record => exactSelectionLinkName(record.item)),
        expandedUrdfToolPickRecordLinks: toolPickRecords.map(record => exactSelectionLinkName(record.item)),
        selection_robot_owner_id: robotOwnerId,
        selectionRobotOwnerId: robotOwnerId,
        selection_tool_owner_id: toolOwnerId,
        selectionToolOwnerId: toolOwnerId,
      };
      state.robotPreviewResult = result;
      const authoritativeDiagnostics = result.diagnostics || previewResult.diagnostics;
      mergeMissingDiagnosticsInPlace(authoritativeDiagnostics, diagnostics);
      Object.assign(authoritativeDiagnostics, localDiagnostics);
      state.robotUrdfPreviewDiagnostics = authoritativeDiagnostics;
      if (rendererReady(authoritativeDiagnostics) && !failIfExpandedUrdfExpectedVisualSetInvalid()) finalizeRequiredLoad('success', { completion_source: 'onRobotLoaded' });
      maybeEmitSceneReady();
      refreshInitialPoseActionState();
      renderSceneSummary();
    },
    onRobotMeshLoaded: () => {
      if (!callbackIsCurrent()) return ignoreStaleCallback();
      renderSceneSummary();
    },
    onRobotMeshLoadError: (err, uri, detail) => { if (!callbackIsCurrent()) return ignoreStaleCallback(); finalizeRequiredLoad('failure', { ...(detail || { uri }), error: err, completion_source: 'onRobotMeshLoadError' }); renderSceneSummary(); },
    onRobotError: (err, diagnostics) => {
      if (!callbackIsCurrent()) return ignoreStaleCallback();
      const authoritativeDiagnostics = diagnostics || previewResult.diagnostics || state.robotUrdfPreviewDiagnostics;
      mergeMissingDiagnosticsInPlace(authoritativeDiagnostics, state.robotUrdfPreviewDiagnostics);
      state.robotUrdfPreviewDiagnostics = authoritativeDiagnostics;
      state.robotUrdfPreviewDiagnostics.robot_preview_lifecycle_state = 'failed';
      state.robotUrdfPreviewDiagnostics.robotPreviewLifecycleState = 'failed';
      state.robotUrdfPreviewDiagnostics.robot_preview_failure_reason = err?.message || String(err || 'expanded URDF preview failed');
      state.robotUrdfPreviewDiagnostics.robotPreviewFailureReason = state.robotUrdfPreviewDiagnostics.robot_preview_failure_reason;
      finalizeRequiredLoad('failure', { error: err, completion_source: 'onRobotError' });
      maybeEmitSceneReady();
      appendRuntimeWarning({}, preview?.urdf_url || '', `expanded_urdf_loader failed: ${err?.message || err}`, 'expanded_urdf_loader_failed');
      refreshWarnings();
      renderSceneSummary();
    },
  });
  if (previewResult.diagnostics && typeof previewResult.diagnostics === 'object') {
    mergeMissingDiagnosticsInPlace(previewResult.diagnostics, diagnostics);
    state.robotUrdfPreviewDiagnostics = previewResult.diagnostics;
  }
  if (!previewResult.ready || typeof previewResult.ready.then !== 'function') {
    finalizeRequiredLoad('failure', {
      completion_source: 'previewResult.ready',
      reason: 'Expanded URDF renderer did not provide previewResult.ready. Reload the viewer bundle or repair the renderer contract.',
    });
  } else {
    previewResult.ready.then(readyResult => {
      if (!callbackIsCurrent()) return ignoreStaleCallback('previewResult.ready');
      const terminalResult = readyResult || previewResult;
      const terminalDiagnostics = terminalResult?.diagnostics || previewResult.diagnostics;
      state.robotPreviewResult = terminalResult;
      if (terminalDiagnostics && typeof terminalDiagnostics === 'object') {
        mergeMissingDiagnosticsInPlace(terminalDiagnostics, diagnostics);
        state.robotUrdfPreviewDiagnostics = terminalDiagnostics;
      }
      if (rendererReady(state.robotUrdfPreviewDiagnostics) && !failIfExpandedUrdfExpectedVisualSetInvalid()) {
        finalizeRequiredLoad('success', { completion_source: 'previewResult.ready' });
      } else {
        finalizeRequiredLoad('failure', {
          completion_source: 'previewResult.ready',
          reason: String(state.robotUrdfPreviewDiagnostics.robot_preview_failure_reason || state.robotUrdfPreviewDiagnostics.robotPreviewFailureReason || 'Expanded URDF previewResult.ready resolved without a ready preview. Check required robot/tool visual diagnostics.'),
        });
      }
      renderSceneSummary();
    }, err => finalizeRequiredLoad('failure', { error: err, completion_source: 'previewResult.ready_rejection' }));
  }
  return previewResult;
}

function linkNameOfItem(item) { return String(item?.link_name || item?.link || item?.frame || item?.object_name || item?.id || '').trim(); }
function parentLinkOfItem(item) { return String(item?.parent_link || item?.joint_parent_link || item?.immediate_parent_link || '').trim(); }
function jointOriginOfItem(item) { return item?.parent_to_child_pose || item?.parent_from_child || item?.joint_origin || item?.parent_joint_origin || { xyz: [0, 0, 0], rpy: [0, 0, 0] }; }
function derivedParentToChildPose(item, parentItem) {
  if (!THREE?.Matrix4 || !item || !parentItem) return null;
  const childPose = item?.link_world_pose || item?.frame_world_pose;
  const parentPose = parentItem?.link_world_pose || parentItem?.frame_world_pose;
  if (!hasFinitePoseBlock(childPose) || !hasFinitePoseBlock(parentPose)) return null;
  const parentWorld = matrixFromPoseBlock(parentPose);
  const childWorld = matrixFromPoseBlock(childPose);
  return poseBlockFromMatrix(parentWorld.clone().invert().multiply(childWorld));
}
function jointOriginForChildItem(item, parentItem) {
  const explicit = item?.parent_to_child_pose || item?.parent_from_child || item?.joint_origin || item?.parent_joint_origin;
  if (hasFinitePoseBlock(explicit)) return explicit;
  const derived = derivedParentToChildPose(item, parentItem);
  if (derived) {
    const pose = { xyz: finiteXyzArrayFromVector(derived.xyz) || [0, 0, 0], rpy: finiteXyzArrayFromVector(derived.rpy) || [0, 0, 0] };
    item.parent_to_child_pose = pose;
    item.parent_from_child = pose;
    item.joint_origin = pose;
    item.parent_joint_origin = pose;
    item.parent_to_child_pose_source = 'viewer_derived_parent_world_inverse_times_child_world';
    item.parent_from_child_source = 'viewer_derived_parent_world_inverse_times_child_world';
    return pose;
  }
  return { xyz: [0, 0, 0], rpy: [0, 0, 0] };
}
function applyPoseBlockToObject(object, poseSource) {
  const pose = poseBlockOf(poseSource || {});
  object.position.copy(pose.xyz);
  object.rotation.set(pose.rpy.x, pose.rpy.y, pose.rpy.z, 'XYZ');
  object.scale.set(1, 1, 1);
}
function assemblyGroupKey(item) {
  if (itemAssemblyGroup(item)) return itemAssemblyGroup(item);
  const scene = sceneId() || sceneDisplayName() || 'scene';
  const tool = isGeneratedToolOrGripperItem(item) ? 'tool' : 'robot';
  return `${scene}_${tool}_generated_urdf`;
}
function createAssemblyLinkNode(name) {
  const node = new THREE.Group();
  node.name = `${name}_assembled_link_node`;
  node.up.copy(THREE.Object3D.DEFAULT_UP);
  node.userData.assembled_urdf_hierarchy = true;
  node.userData.link_name = name;
  return node;
}
function buildRobotAssemblies(items) {
  const candidates = items.filter(isAssemblyCandidateItem);
  const renderDiagnostics = { skipped_flattened_urdf_visual_count: 0, assembled_hierarchy_rendered_mesh_count: 0, rendered_fk_visual_count: 0, skipped_legacy_generated_urdf_count: 0, visible_duplicate_generated_urdf_count: 0, visible_tool0_fallback_count: 0, detached_robot_mesh_clusters: 0 };
  if (!candidates.length) return { handled: new Set(), assemblies: [], renderDiagnostics };
  if (candidates.some(usesUrdfFkVisualWorldPose)) {
    const handled = new Set();
    const assemblies = [];
    const fkItems = candidates;
    const root = new THREE.Group();
    root.name = `${assemblyGroupKey(fkItems[0] || {})}_UrdfFkVisualWorldRoot`;
    root.up.copy(THREE.Object3D.DEFAULT_UP);
    root.userData.robot_render_mode = 'verified_urdf_fk_visual_world_pose';
    root.userData.robot_transform_source = 'ros_tf_verified_urdf_fk';
    const linkPositions = {};
    const visualPositions = {};
    const chainLinks = ['base_link_inertia','shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link','tool0','gripper_base_link'];
    for (const item of fkItems) {
      item.robot_render_mode = 'verified_urdf_fk_visual_world_pose';
      item.workcell_web_render_pose_mode = 'verified_urdf_fk_visual_world_pose';
      item.baked_world_visual_pose_diagnostic_only = Boolean(item.baked_world_visual_pose || item.expected_visual_pose);
      const object3d = new THREE.Group();
      object3d.name = `${item.id || itemLabel(item)}_urdf_fk_visual_world_pose_root`;
      object3d.up.copy(THREE.Object3D.DEFAULT_UP);
      applyPoseBlockToObject(object3d, item.urdf_fk_visual_world_pose || item.final_transform || item.world_from_visual || item.link_world_pose || item.frame_world_pose);
      root.add(object3d);
      assignItemUserData(object3d, item);
      const primitive = primitiveOf(item);
      const meshlessTool0Frame = isExpectedMeshlessTool0Frame(item);
      const fallback = meshlessTool0Frame ? null : (isSensor(item) ? makeSensorMarker(item) : makePrimitiveMesh(item));
      if (fallback) { fallback.name = `${item.id || itemLabel(item)}_fallback`; assignItemUserData(fallback, item); fallback.visible = false; object3d.add(fallback); }
      const rendered = { item, object3d, fallback, labelEl: createLabelElement(item), originalTransform: transformOf(item), authoredBaselineTransform: cloneTransform(transformOf(item)) };
      setRenderInfo(rendered, meshlessTool0Frame ? 'meshless_frame' : (itemRequiresMeshBackedVisual(item) ? 'mesh_loading_required' : (primitive ? 'primitive_fallback' : 'box_fallback')), displayMeshUri(item), meshlessTool0Frame ? 'tool0 is an expected meshless frame; no visible fallback is rendered' : 'rendered from ROS TF verified URDF FK visual world pose');
      state.objects.push(rendered);
      handled.add(item);
      if (!meshlessTool0Frame) { const loadMeshForUrdfFk = tryLoadMesh; loadMeshForUrdfFk(item, rendered, fallback); }
      const link = linkNameOfItem(item);
      const linkPoseSource = item.urdf_fk_link_world_pose || item.link_world_pose || item.frame_world_pose || item.final_transform;
      const visualPoseSource = item.urdf_fk_visual_world_pose || item.final_transform || item.world_from_visual || item.link_world_pose || item.frame_world_pose;
      if (link && !linkPositions[link] && hasFinitePoseBlock(linkPoseSource)) linkPositions[link] = finiteXyzArrayFromVector(poseBlockOf(linkPoseSource).xyz);
      if (link && !visualPositions[link] && hasFinitePoseBlock(visualPoseSource)) visualPositions[link] = finiteXyzArrayFromVector(poseBlockOf(visualPoseSource).xyz);
    }
    state.three.scene.add(root);
    state.assemblyRoots.push(root);
    const pairs = [['shoulder_link','upper_arm_link'],['upper_arm_link','forearm_link'],['forearm_link','wrist_1_link'],['wrist_3_link','tool0'],['tool0','gripper_base_link']];
    const distances = {};
    for (const [a,b] of pairs) distances[`${a} -> ${b}`] = distanceMetersBetweenXyz(linkPositions[a], linkPositions[b]);
    assemblies.push({ assembly_group: assemblyGroupKey(fkItems[0] || {}), robot_instance_id: fkItems.find(i => i.robot_instance_id)?.robot_instance_id || assemblyGroupKey(fkItems[0] || {}), robot_transform_source: 'ros_tf_verified_urdf_fk', robot_render_mode: 'verified_urdf_fk_visual_world_pose', robot_hierarchy_links: chainLinks.filter(l => linkPositions[l] || visualPositions[l]), robot_hierarchy_missing_links: chainLinks.filter(l => !linkPositions[l] && !visualPositions[l]), robot_hierarchy_missing_parents: [], robot_hierarchy_mesh_count: fkItems.filter(item => displayMeshUri(item)).length, assembled_hierarchy_rendered_mesh_count: fkItems.filter(item => displayMeshUri(item)).length, assembled_link_world_positions: linkPositions, assembled_visual_world_positions: visualPositions, assembled_link_adjacency_distances_m: distances, urdf_fk_debug_chain: chainLinks.map(link => { const item = fkItems.find(i => linkNameOfItem(i) === link) || {}; return { link, parent: item.urdf_joint_parent || parentLinkOfItem(item), joint_name: item.joint_name || item.parent_joint_name || '', joint_origin: item.urdf_joint_origin || item.joint_origin || item.parent_joint_origin || null, joint_value: item.joint_value ?? item.parent_joint_value ?? 0, world_xyz: linkPositions[link] || null, visual_world_xyz: visualPositions[link] || null }; }), urdf_fk_distances_m: distances });
    renderDiagnostics.assembled_hierarchy_rendered_mesh_count = fkItems.filter(item => displayMeshUri(item)).length;
    renderDiagnostics.rendered_fk_visual_count = fkItems.filter(item => displayMeshUri(item)).length;
    renderDiagnostics.skipped_legacy_generated_urdf_count = handled.size;
    renderDiagnostics.skipped_flattened_urdf_visual_count = handled.size;
    return { handled, assemblies, renderDiagnostics };
  }
  const groups = new Map();
  for (const item of candidates) {
    const key = assemblyGroupKey(item);
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(item);
  }
  const handled = new Set();
  const assemblies = [];
  for (const [groupKey, groupItems] of groups.entries()) {
    const root = new THREE.Group();
    root.name = `${groupKey}_RobotRoot`;
    root.up.copy(THREE.Object3D.DEFAULT_UP);
    root.userData.robot_render_mode = 'assembled_urdf_hierarchy';
    root.userData.assembly_group = groupKey;
    const byLink = new Map();
    for (const item of groupItems) {
      const link = linkNameOfItem(item);
      if (!link) continue;
      if (!byLink.has(link)) byLink.set(link, []);
      byLink.get(link).push(item);
    }
    for (const name of Array.from(byLink.keys())) {
      const parent = parentLinkOfItem(byLink.get(name)[0]);
      if (parent && !byLink.has(parent)) byLink.set(parent, []);
    }
    const nodes = new Map(Array.from(byLink.keys()).map(name => [name, createAssemblyLinkNode(name)]));
    const missingParents = [];
    for (const [link, linkItems] of byLink.entries()) {
      const node = nodes.get(link);
      const representative = linkItems[0] || { link_name: link };
      const parent = parentLinkOfItem(representative);
      if (parent && nodes.has(parent)) {
        // Prefer jointOriginForChildItem here; the legacy call shape
        // applyPoseBlockToObject(node, jointOriginOfItem(representative)) is
        // intentionally superseded because missing joint origins must be
        // derived from parent-world inverse * child-world instead of falling
        // back to baked visual/world poses.
        const parentItem = (byLink.get(parent) || [])[0] || null;
        applyPoseBlockToObject(node, jointOriginForChildItem(representative, parentItem));
        nodes.get(parent).add(node);
      } else {
        if (parent) missingParents.push(`${parent} -> ${link}`);
        applyPoseBlockToObject(node, generatedUrdfFramePoseSource(representative));
        root.add(node);
      }
    }
    for (const item of groupItems) {
      const link = linkNameOfItem(item);
      const node = nodes.get(link);
      if (!node) continue;
      item.robot_render_mode = 'assembled_urdf_hierarchy';
      item.workcell_web_render_pose_mode = 'assembled_urdf_hierarchy';
      item.baked_world_visual_pose_diagnostic_only = Boolean(item.baked_world_visual_pose || item.expected_visual_pose);
      const primitive = primitiveOf(item);
      const meshlessTool0Frame = isExpectedMeshlessTool0Frame(item);
      const fallback = meshlessTool0Frame ? null : (isSensor(item) ? makeSensorMarker(item) : makePrimitiveMesh(item));
      if (fallback) {
        fallback.name = `${item.id || itemLabel(item)}_fallback`;
        assignItemUserData(fallback, item);
        fallback.visible = false;
        node.add(fallback);
      }
      assignItemUserData(node, item);
      const rendered = { item, object3d: node, fallback, labelEl: createLabelElement(item), originalTransform: transformOf(item), authoredBaselineTransform: cloneTransform(transformOf(item)) };
      const requiredMesh = itemRequiresMeshBackedVisual(item);
      setRenderInfo(rendered, meshlessTool0Frame ? 'meshless_frame' : (requiredMesh ? 'mesh_loading_required' : (primitive ? 'primitive_fallback' : 'box_fallback')), displayMeshUri(item), meshlessTool0Frame ? 'tool0 is an expected meshless frame; no visible fallback is rendered' : (requiredMesh ? 'required mesh is loading under assembled URDF hierarchy' : 'assembled URDF hierarchy fallback'));
      state.objects.push(rendered);
      handled.add(item);
      if (!meshlessTool0Frame) tryLoadMesh(item, rendered, fallback);
    }
    state.three.scene.add(root);
    state.assemblyRoots.push(root);
    const requiredLinks = ['base_link_inertia','shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link','tool0','gripper_base_link'];
    root.updateMatrixWorld(true);
    const linkPositions = {};
    for (const [name, node] of nodes.entries()) {
      const pos = new THREE.Vector3();
      node.getWorldPosition(pos);
      linkPositions[name] = finiteXyzArrayFromVector(pos);
    }
    const adjacencyPairs = [['base_link_inertia','shoulder_link'],['shoulder_link','upper_arm_link'],['upper_arm_link','forearm_link'],['forearm_link','wrist_1_link'],['wrist_1_link','wrist_2_link'],['wrist_2_link','wrist_3_link'],['wrist_3_link','tool0'],['tool0','gripper_base_link']];
    const adjacency = {};
    for (const [a, b] of adjacencyPairs) adjacency[`${a} -> ${b}`] = distanceMetersBetweenXyz(linkPositions[a], linkPositions[b]);
    assemblies.push({
      assembly_group: groupKey,
      robot_instance_id: groupItems.find(i => i.robot_instance_id)?.robot_instance_id || groupKey,
      robot_render_mode: 'assembled_urdf_hierarchy',
      robot_hierarchy_links: Array.from(nodes.keys()),
      robot_hierarchy_missing_links: requiredLinks.filter(link => !nodes.has(link)),
      robot_hierarchy_missing_parents: Array.from(new Set(missingParents)),
      robot_hierarchy_mesh_count: groupItems.filter(item => displayMeshUri(item)).length,
      assembled_hierarchy_rendered_mesh_count: groupItems.filter(item => displayMeshUri(item)).length,
      assembled_link_world_positions: linkPositions,
      assembled_link_adjacency_distances_m: adjacency,
    });
    renderDiagnostics.assembled_hierarchy_rendered_mesh_count += groupItems.filter(item => displayMeshUri(item)).length;
  }
  renderDiagnostics.skipped_flattened_urdf_visual_count = handled.size;
  return { handled, assemblies, renderDiagnostics };
}

function createLabelElement(item) {
  if (!el.labelLayer || !shouldLabelItem(item)) return null;
  const label = document.createElement('div');
  label.className = 'object-label';
  label.textContent = itemLabel(item);
  label.title = `${item.id || itemLabel(item)} — ${itemType(item)}`;
  label.setAttribute('data-object-id', item.id || itemLabel(item));
  el.labelLayer.appendChild(label);
  return label;
}
function setLabelsVisible(visible) {
  state.labelsVisible = Boolean(visible);
  if (el.labelsToggle) el.labelsToggle.checked = state.labelsVisible;
  if (el.labelLayer) el.labelLayer.classList.toggle('labels-hidden', !state.labelsVisible);
  updateLabels();
}

function setDebugOverlaysVisible(visible) {
  state.debugOverlaysVisible = Boolean(visible);
  if (el.debugOverlaysToggle) el.debugOverlaysToggle.checked = state.debugOverlaysVisible;
  for (const rendered of state.objects) {
    if (isDebugOverlayItem(rendered.item)) rendered.object3d.visible = state.debugOverlaysVisible;
  }
  if (state.selected && !state.debugOverlaysVisible) {
    const selected = state.objects.find(obj => obj.item.id === state.selected);
    if (selected && isDebugOverlayItem(selected.item)) {
      state.selected = null;
      state.selectedRenderIdentityId = '';
      detachTransformGizmo();
      el.inspector.className = 'state empty';
      el.inspector.textContent = state.objects.length ? 'Select an object from the list or canvas.' : EMPTY_SCENE_MESSAGE;
    }
  }
  populateObjectList();
  updateLabels();
  renderSceneSummary();
}

function updateLabels() {
  const { camera } = state.three;
  if (!camera || !el.labelLayer) return;
  const rect = el.canvas.getBoundingClientRect();
  const center = new THREE.Vector3();
  for (const rendered of state.objects) {
    const label = rendered.labelEl;
    if (!label) continue;
    if (!state.labelsVisible || !rendered.object3d.visible) {
      label.hidden = true;
      continue;
    }
    rendered.object3d.updateWorldMatrix(true, true);
    const bounds = new THREE.Box3().setFromObject(rendered.object3d);
    if (bounds.isEmpty()) rendered.object3d.getWorldPosition(center);
    else bounds.getCenter(center);
    const screen = center.clone().project(camera);
    const inFront = screen.z >= -1 && screen.z <= 1;
    label.hidden = !inFront;
    if (!inFront) continue;
    const x = (screen.x * 0.5 + 0.5) * rect.width;
    const y = (-screen.y * 0.5 + 0.5) * rect.height;
    label.style.transform = `translate(-50%, -115%) translate(${x.toFixed(1)}px, ${y.toFixed(1)}px)`;
  }
}

function objectListMeshSummary(renderedItems) {
  const counts = new Map();
  for (const rendered of renderedItems) {
    const label = meshStatusLabel(rendered);
    counts.set(label, (counts.get(label) || 0) + 1);
  }
  return Array.from(counts.entries()).map(([label, count]) => `${count} ${label}`).join(', ') || 'no mesh status';
}
function appendObjectListRow(rendered, group) {
  const li = document.createElement('li');
  li.dataset.id = rendered.item.id;
  if (state.selected === rendered.item.id) li.classList.add('selected');

  const name = document.createElement('span');
  name.className = 'object-name';
  name.textContent = `${rendered.item.id} — ${itemLabel(rendered.item)}`;
  li.appendChild(name);

  const tag = document.createElement('span');
  tag.className = 'type-tag';
  tag.textContent = itemType(rendered.item);
  li.appendChild(tag);

  const status = document.createElement('span');
  status.className = `status-chip status-${String(rendered.item.mesh_status || rendered.renderInfo?.render_status || 'unknown').replace(/[^a-z0-9_-]/gi, '-').toLowerCase()}`;
  status.textContent = meshStatusLabel(rendered);
  li.appendChild(status);

  const meta = document.createElement('span');
  meta.className = 'meta';
  meta.textContent = `${group} · ${rendered.item.locked ? 'locked/generated' : 'editable/environment'} · ${supportSurfaceDisplayType(rendered.item) || meshStatusLabel(rendered)}${state.dirtyTransforms.has(rendered.item.id) ? ' · edited' : ''}`;
  li.appendChild(meta);
  li.addEventListener('click', () => selectObject(rendered.item.id));
  el.list.appendChild(li);
}
function appendObjectListGroupSummary(group, renderedItems, groupLabels) {
  const li = document.createElement('li');
  li.className = 'object-group-summary';

  const name = document.createElement('span');
  name.className = 'object-name';
  name.textContent = groupLabels[group] || group;
  li.appendChild(name);

  const tag = document.createElement('span');
  tag.className = 'type-tag';
  tag.textContent = `${renderedItems.length} links`;
  li.appendChild(tag);

  const status = document.createElement('span');
  status.className = 'status-chip';
  status.textContent = objectListMeshSummary(renderedItems);
  li.appendChild(status);

  const meta = document.createElement('span');
  meta.className = 'meta';
  const editableCount = renderedItems.filter(rendered => canEditItem(rendered.item)).length;
  meta.textContent = `${renderedItems.length} generated locked URDF link${renderedItems.length === 1 ? '' : 's'} · ${objectListMeshSummary(renderedItems)}${editableCount ? ` · ${editableCount} editable` : ''} · enable debug/details to inspect individual links`;
  li.appendChild(meta);
  el.list.appendChild(li);
}
function populateObjectList() {
  el.list.innerHTML = '';
  if (!state.objects.length) {
    const li = document.createElement('li');
    li.className = 'state empty';
    li.textContent = EMPTY_SCENE_MESSAGE;
    el.list.appendChild(li);
    return;
  }
  const groupLabels = {
    robot: 'Robot',
    'tool/gripper': 'Tool / Gripper',
    'robot/tool/generated': 'Robot, tool & generated',
    'environment/layout': 'Environment & layout',
    sensors: 'Sensors',
    zones: 'Zones',
    unknown: 'Unknown',
  };
  const grouped = new Map(Object.keys(groupLabels).map(group => [group, []]));
  for (const rendered of state.objects) {
    if (!state.debugOverlaysVisible && isDebugOverlayItem(rendered.item)) continue;
    const group = viewerGroupFor(rendered.item);
    if (!grouped.has(group)) grouped.set(group, []);
    grouped.get(group).push(rendered);
  }
  const visibleListCount = Array.from(grouped.values()).reduce((total, renderedItems) => total + renderedItems.length, 0);
  if (!visibleListCount) {
    const li = document.createElement('li');
    li.className = 'state empty';
    li.textContent = 'Only zones/debug overlays are present. Enable Show zones/debug overlays to inspect them.';
    el.list.appendChild(li);
    return;
  }
  for (const [group, renderedItems] of grouped.entries()) {
    if (!renderedItems.length) continue;
    const heading = document.createElement('li');
    heading.className = 'object-group-heading';
    heading.textContent = groupLabels[group] || group;
    el.list.appendChild(heading);

    const generatedGroup = group === 'robot' || group === 'tool/gripper' || group === 'robot/tool/generated';
    if (!state.debugOverlaysVisible && generatedGroup) {
      appendObjectListGroupSummary(group, renderedItems, groupLabels);
      continue;
    }
    for (const rendered of renderedItems) appendObjectListRow(rendered, group);
  }
}

function removeSelectionHighlight() {
  const highlight = state.three.selectionHighlight;
  if (!highlight) return;
  state.three.scene?.remove(highlight);
  disposeOwnedObject3d(highlight);
  state.three.selectionHighlight = null;
}
function refreshSelectionHighlight(rendered) {
  removeSelectionHighlight();
  if (!rendered || !state.three.scene || !THREE?.Box3Helper) return;
  const physical = collectSelectionPhysicalBounds(rendered);
  if (!physical.bounds) return;
  const helper = new THREE.Box3Helper(physical.bounds, 0x0078a8);
  helper.name = 'selection_subtle_bounds_highlight';
  helper.userData.selection_outline = true;
  helper.userData.selection_highlight = true;
  helper.userData.exclude_from_fit_bounds = true;
  helper.userData.exclude_from_physical_bounds = true;
  helper.material.transparent = true;
  helper.material.opacity = 0.62;
  helper.material.depthTest = false;
  helper.renderOrder = 10;
  state.three.selectionHighlight = helper;
  state.three.scene.add(helper);
}

function isNormalSelectableRendered(rendered) {
  const item = rendered?.item;
  const inspectionSelectable = state.debugOverlaysVisible && (isTaskOnlyHelperItem(item) || isOverlayPolicyItem(item) || isDebugOverlayItem(item));
  return Boolean(item?.id) && item.selectable !== false && !isDiagnosticOnlyItem(item) && (inspectionSelectable || (!isTaskOnlyHelperItem(item) && !isOverlayPolicyItem(item) && !isDebugOverlayItem(item)));
}
function isExpandedUrdfInspectionPick(rendered) {
  return Boolean(rendered?.authoritativePhysicalPick === true && state.pickRecords.includes(rendered) &&
    rendered?.item?.id && rendered.object3d?.visible !== false);
}
function isCanvasSelectableRendered(rendered) {
  return isExpandedUrdfInspectionPick(rendered) || isNormalSelectableRendered(rendered);
}
function intrinsicallyExcludedPickNode(node) {
  const data = node?.userData || {};
  const name = String(node?.name || '').toLowerCase();
  const explicitlyExcluded = node?.visible === false || data.selection_outline || data.selection_highlight ||
    data.hidden_overlay || data.helper_hidden ||
    /transformcontrols|transform_controls|gizmo|selection_.*highlight/.test(name);
  if (explicitlyExcluded) return true;
  if (data.fallback_sensor_body === true || /(?:^|[_-])fallback_sensor_body(?:$|[_-])/.test(name)) return false;
  return /(?:^|[_-])(?:edges?|frustum|helper)(?:$|[_-])/.test(name);
}
function passThroughPickNode(node) {
  const data = node?.userData || {};
  const name = String(node?.name || '').toLowerCase();
  return data.diagnostic_only || data.non_selectable || data.selectable === false;
}
function excludedPickNode(node) {
  return intrinsicallyExcludedPickNode(node) || passThroughPickNode(node);
}
function hiddenPickSubtree(node) {
  for (let current = node; current; current = current.parent) {
    if (current.visible === false) return current;
  }
  return null;
}
function intrinsicallyExcludedPickItem(item) {
  const sourceLayer = String(item?.source_layer || '').toLowerCase();
  const identity = [item?.role, item?.category, item?.id].map(value => String(value || '').toLowerCase()).join(' ');
  const explicitDebug = isDebugOverlayItem(item) && (item?.diagnostic_only === true || /debug|diagnostic/.test(`${sourceLayer} ${identity}`));
  return explicitDebug || (isTaskOnlyHelperItem(item) && /task|debug/.test(sourceLayer));
}
function resolveCanvasPickHit(hit) {
  let node = hit?.object || null;
  let candidate = null;
  let registeredRecord = null;
  let rejectionReason = 'no_registered_or_rendered_identity';
  const hiddenAncestor = hiddenPickSubtree(node);
  if (hiddenAncestor) {
    return { renderIdentity: null, selectionOwner: null, selectionOwnerSource: '', editOwner: null,
      eligible: false, rejectionReason: 'hit_hidden_subtree', exclusionNode: hiddenAncestor,
      exclusionFlag: 'visible=false', registeredRecord: null, hit };
  }
  while (node) {
    // Registration is authoritative for expanded-URDF inspection descendants.
    // Loader-provided Collada/URDF userData can contain stale diagnostic flags,
    // so consult the registry before interpreting those descendant flags.
    const registered = state.pickIdentityByObject.get(node);
    const nodeItem = node.userData?.item;
    const nodeName = String(node?.name || '').toLowerCase();
    const directlyHitPhysicalCamera = node === hit?.object && Boolean(nodeItem && isSensor(nodeItem) &&
      (node.userData?.fallback_sensor_body === true || (!/(?:edges?|frustum|helper|overlay|highlight|gizmo)/.test(nodeName) && (node.isMesh || node.type === 'Mesh'))));
    if (intrinsicallyExcludedPickNode(node)) {
      rejectionReason = 'hit_node_intrinsically_excluded';
      return { renderIdentity: null, selectionOwner: null, selectionOwnerSource: '', editOwner: null, eligible: false, rejectionReason, exclusionNode: node, exclusionFlag: 'intrinsic_node_identity', registeredRecord, hit };
    }
    if (isExpandedUrdfInspectionPick(registered)) {
      registeredRecord = registered;
      candidate = registered;
      break;
    }
    // A different registered root is a nested URDF-link ownership boundary.
    if (candidate && registered && registered !== candidate) break;
    // Edges, frustums, and other helpers are transparent to the ordered hit
    // list, not aliases for a physical ancestor. Reject this intersection and
    // let the caller evaluate the next raycast hit (for example the camera
    // body behind its frustum).
    if (passThroughPickNode(node) && !directlyHitPhysicalCamera) {
      rejectionReason = 'hit_node_non_selectable_metadata';
      const flag = node.userData?.diagnostic_only ? 'diagnostic_only' : (node.userData?.non_selectable ? 'non_selectable' : 'selectable=false');
      return { renderIdentity: null, selectionOwner: null, selectionOwnerSource: '', editOwner: null, eligible: false, rejectionReason, exclusionNode: node, exclusionFlag: flag, registeredRecord, hit };
    }
    if (!candidate) {
      if (nodeItem && intrinsicallyExcludedPickItem(nodeItem) && !directlyHitPhysicalCamera) {
        rejectionReason = 'render_item_intrinsically_excluded';
        return { renderIdentity: null, selectionOwner: null, selectionOwnerSource: '', editOwner: null, eligible: false, rejectionReason, exclusionNode: node, exclusionFlag: 'item_diagnostic_policy', registeredRecord, hit };
      }
      const item = nodeItem || registered?.item;
      if (item?.id) {
        const rendered = registered?.authoritativePhysicalPick === true ? registered : (renderedById(item.id) || registered);
        if (!rendered) break;
        candidate = rendered;
      }
    }
    // A directly intersected physical camera body owns this intersection.
    // Generated ancestors may be diagnostic containers, but cannot turn the
    // already identified body into a helper hit.
    if (directlyHitPhysicalCamera && candidate) break;
    node = node.parent;
  }
  const renderIdentity = candidate;
  const authoritativeOwnerId = expandedUrdfPhysicalOwnerId(renderIdentity);
  const ownerId = authoritativeOwnerId || (renderIdentity ? explicitUiSelectionItemId(renderIdentity) : '');
  const ownerResolution = resolveSelectionOwner(ownerId);
  const selectionOwner = ownerResolution.record;
  const editOwner = selectionOwner && canEditItem(selectionOwner.item) ? selectionOwner : null;
  const explicitlyPhysical = renderIdentity?.authoritativePhysicalPick === true;
  const eligible = Boolean(renderIdentity && (explicitlyPhysical ? ownerId : isCanvasSelectableRendered(selectionOwner || renderIdentity)));
  if (!eligible) rejectionReason = explicitlyPhysical && !ownerId ? 'authoritative_physical_owner_unresolved' : (renderIdentity ? 'resolved_owner_not_canvas_selectable' : rejectionReason);
  else rejectionReason = '';
  return { renderIdentity, selectionOwner, selectionOwnerSource: ownerResolution.source, editOwner, eligible, rejectionReason, exclusionNode: null, exclusionFlag: '', registeredRecord: registeredRecord || (explicitlyPhysical ? renderIdentity : null), hit };
}
function itemFromRaycastHit(hit) {
  const resolved = resolveCanvasPickHit(hit);
  return resolved.eligible ? resolved.renderIdentity : null;
}
function pickingPriority(rendered) {
  const item = rendered?.item;
  if (!item?.id) return Number.POSITIVE_INFINITY;
  if (isTaskOnlyHelperItem(item) || isOverlayPolicyItem(item) || isDebugOverlayItem(item)) return state.debugOverlaysVisible ? 4 : Number.POSITIVE_INFINITY;
  if (canEditItem(item) && isPrimaryRenderableItem(item) && isPhysicalSemanticItem(item)) return 1;
  if (canEditItem(item) && !isGeneratedUrdfItem(item)) return 2;
  if ((isGeneratedUrdfItem(item) || item.locked) && isPhysicalSemanticItem(item)) return 3;
  return 2;
}
function rankedPickingCandidates(hits) {
  const candidates = [];
  const seen = new Set();
  for (const hit of hits || []) {
    if (hiddenPickSubtree(hit?.object)) continue;
    const resolved = resolveCanvasPickHit(hit);
    const rendered = resolved.renderIdentity;
    const identityKey = `${rendered?.item?.id || ''}|${resolved.selectionOwner?.item?.id || ''}`;
    if (!rendered?.item?.id || seen.has(identityKey)) continue;
    seen.add(identityKey);
    candidates.push({ ...resolved, rendered, priority: pickingPriority(resolved.editOwner || resolved.selectionOwner || rendered) });
  }
  // Coincident transparent surfaces can report tiny ordering noise. Semantic
  // priority is only a tie-break within one millimetre; distance wins otherwise.
  const PICK_COINCIDENCE_TOLERANCE_M = 0.001;
  return candidates.sort((a, b) => {
    const distanceDelta = Number(a.hit?.distance || 0) - Number(b.hit?.distance || 0);
    return Math.abs(distanceDelta) <= PICK_COINCIDENCE_TOLERANCE_M ? a.priority - b.priority || distanceDelta : distanceDelta;
  });
}
function failedCanvasPickDiagnostic(hits) {
  const objectNames = [];
  let traversedRegisteredIdentity = false;
  let firstActionableRejectionReason = '';
  const hitResolutions = [];
  let nearestKnownUrdfLinkAncestor = '';
  const knownLinks = state.robotPreviewResult?.links instanceof Map ? state.robotPreviewResult.links : new Map();
  const knownLinkByNode = new Map(Array.from(knownLinks.entries()).map(([name, node]) => [node, String(name || '')]));
  for (const hit of hits || []) {
    const resolution = resolveCanvasPickHit(hit);
    const rawName = String(hit?.object?.name || '').trim();
    if (rawName && !objectNames.includes(rawName) && objectNames.length < 12) objectNames.push(rawName);
    let node = hit?.object || null;
    while (node) {
      const registered = state.pickIdentityByObject.get(node);
      if (registered) traversedRegisteredIdentity = true;
      if (!nearestKnownUrdfLinkAncestor) nearestKnownUrdfLinkAncestor = knownLinkByNode.get(node) || String(registered?.item?.link_name || registered?.item?.link || '').trim();
      node = node.parent;
    }
    if (!firstActionableRejectionReason && resolution.rejectionReason) firstActionableRejectionReason = resolution.rejectionReason;
    hitResolutions.push({
      hit_node_name: rawName,
      registered_record_id: resolution.registeredRecord?.item?.id || '',
      pick_source: resolution.registeredRecord?.pickRecordSource || '',
      authoritative_physical_pick: resolution.registeredRecord?.authoritativePhysicalPick === true,
      selection_owner_id: resolution.selectionOwner?.item?.id || '',
      selection_owner_source: resolution.selectionOwnerSource || '',
      edit_owner_id: resolution.editOwner?.item?.id || '',
      rejection_stage_reason: resolution.rejectionReason || '',
      exclusion_ancestor_node: resolution.exclusionNode?.name || '',
      exclusion_flag: resolution.exclusionFlag || '',
      candidate_priority: Number.isFinite(pickingPriority(resolution.editOwner || resolution.selectionOwner || resolution.renderIdentity)) ? pickingPriority(resolution.editOwner || resolution.selectionOwner || resolution.renderIdentity) : 'Infinity',
    });
  }
  if (!firstActionableRejectionReason) firstActionableRejectionReason = traversedRegisteredIdentity ? 'registered_identity_rejected_by_selection_policy' : 'no_registered_identity_in_hit_ancestry';
  return {
    raw_hit_count: (hits || []).length,
    rawHitCount: (hits || []).length,
    hit_object_names: objectNames,
    hitObjectNames: objectNames,
    traversed_registered_identity: traversedRegisteredIdentity,
    traversedRegisteredIdentity,
    first_actionable_rejection_reason: firstActionableRejectionReason,
    firstActionableRejectionReason,
    nearest_known_urdf_link_ancestor: nearestKnownUrdfLinkAncestor,
    nearestKnownUrdfLinkAncestor,
    hit_resolutions: hitResolutions,
    hitResolutions,
  };
}
function selectObject(id) { return selectObjectFromRender(id, null); }
function selectObjectFromRender(id, renderIdentity = null) {
  const requestedId = String(id || '');
  const rawRequested = requestedId ? renderedById(requestedId) : null;
  const requested = inspectionSelectionRendered(renderIdentity || rawRequested);
  const selectionId = requested ? explicitUiSelectionItemId(requested) : requestedId;
  const explicitlySelectable = requested && isCanvasSelectableRendered(requested);
  if (requestedId && !explicitlySelectable) {
    const reason = requested ? 'diagnostic_helper_or_non_selectable' : 'missing_render_identity';
    state.ignoredSelectionKeys ||= new Set();
    const warningKey = `${sceneId()}|${requestedId}|${reason}`;
    if (!state.ignoredSelectionKeys.has(warningKey)) {
      state.ignoredSelectionKeys.add(warningKey);
      pushEditorEvent('selection_ignored', { itemId: requestedId, reason, sceneId: sceneId() });
    }
    return '';
  }
  const wasInitialPreviewActive = state.initialPosePreview.active;
  if (selectionId !== state.selected) cancelActiveTransformOperation('Selection changed');
  id = selectionId;
  const previous = state.selected || '';
  state.selected = id;
  state.selectedRenderIdentityId = requested?.item?.id || '';
  document.querySelectorAll('.object-list li').forEach(li => li.classList.toggle('selected', li.dataset.id === id));
  for (const rendered of state.objects) {
    const selected = rendered.item.id === id;
    if (rendered.labelEl) rendered.labelEl.classList.toggle('selected', selected);
  }
  updateLabels();
  const rendered = requested || renderedById(id);
  const editOwner = canonicalTransformOwner(rendered);
  const selectionOwner = resolveSelectionOwner(selectionId).record;
  if (rendered) {
    populateInspector(editOwner || selectionOwner || rendered);
    if (state.editorMode === 'select') detachTransformGizmo('select_mode');
    else attachTransformGizmo(editOwner || selectionOwner || rendered, 'selection');
    refreshSelectionHighlight(rendered);
  } else { detachTransformGizmo(); removeSelectionHighlight(); }
  if (previous !== (id || '')) pushEditorEvent('selection_changed', { itemId: id || '', uiItemId: explicitUiSelectionItemId(rendered), itemType: (editOwner || selectionOwner || rendered) ? itemType((editOwner || selectionOwner || rendered).item) : '', editable: Boolean(editOwner && selectionIsEditable(editOwner)) });
}
function clearSelection() { selectObject(''); el.inspector.className = 'state empty'; el.inspector.textContent = state.objects.length ? 'Select an object from the list or canvas.' : EMPTY_SCENE_MESSAGE; }

function placementHitRendered(hit) {
  for (let node = hit?.object || null; node; node = node.parent) {
    if (intrinsicallyExcludedPickNode(node) || passThroughPickNode(node) || node.material?.wireframe === true) return null;
    const rendered = state.pickIdentityByObject.get(node);
    const item = node.userData?.item || rendered?.item;
    if (item?.id) return renderedById(item.id) || rendered || null;
  }
  return null;
}
function isEligiblePlacementSurfaceHit(hit) {
  const rendered = placementHitRendered(hit);
  const item = rendered?.item;
  return Boolean(item && rendered.object3d?.visible !== false && isPhysicalSemanticItem(item) &&
    !isZone(item) && !isTaskOnlyHelperItem(item) && !isOverlayPolicyItem(item) && !isDebugOverlayItem(item));
}
function finitePlacementPoint(point) {
  return point && [point.x, point.y, point.z].every(Number.isFinite)
    ? { x: Number(point.x), y: Number(point.y), z: Number(point.z) }
    : null;
}

function placementPointForPhysicalSurfaceItem(item, point) {
  const finitePoint = finitePlacementPoint(point);
  if (!finitePoint || !item) return finitePoint;

  const role = String(item?.role || '').toLowerCase().replace(/[_-]+/g, ' ');

  // Only normalize the actual support-surface physical visual. Other physical
  // items may reference a support surface but must keep their real mesh hit.
  if (role !== 'support surface') return finitePoint;

  const ownerId = String(item?.support_surface_ref || item?.id || '').trim();
  if (!ownerId) return finitePoint;

  const owners = Array.isArray(state.sceneJson?.ui_selection_owners)
    ? state.sceneJson.ui_selection_owners
    : [];

  const owner = owners.find(candidate =>
    String(candidate?.id || '').trim() === ownerId
  );
  if (!owner) return finitePoint;

  const xyz = owner?.pose?.xyz;
  const rpy = owner?.pose?.rpy;
  const dimensions = owner?.dimensions;

  if (!Array.isArray(xyz) || xyz.length < 3 ||
      !Array.isArray(rpy) || rpy.length < 3 ||
      !Array.isArray(dimensions) || dimensions.length < 3 ||
      ![...xyz.slice(0, 3), ...rpy.slice(0, 3), ...dimensions.slice(0, 3)].every(Number.isFinite) ||
      dimensions[2] <= 0) return finitePoint;

  // Tilted support surfaces keep the real physical raycast hit.
  if (Math.abs(rpy[0]) > 1e-6 || Math.abs(rpy[1]) > 1e-6) return finitePoint;

  return {
    ...finitePoint,
    z: Number(xyz[2] + dimensions[2] / 2),
  };
}

function placementPointOnAuthoredSupportSurface(raycaster) {
  if (!THREE?.Plane || !THREE?.Vector3 || !raycaster?.ray?.intersectPlane) return null;

  const owners = Array.isArray(state.sceneJson?.ui_selection_owners)
    ? state.sceneJson.ui_selection_owners
    : [];

  let bestPoint = null;
  let bestOwnerId = '';
  let bestDistance = Infinity;

  for (const item of owners) {
    const role = String(item?.role || '').toLowerCase().replace(/[_-]+/g, ' ');
    const type = String(item?.type || '').toLowerCase().replace(/[_-]+/g, ' ');
    const category = String(item?.category || '').toLowerCase().replace(/[_-]+/g, ' ');

    if (role !== 'support surface' &&
        type !== 'support surface' &&
        category !== 'work surface') continue;

    const xyz = item?.pose?.xyz;
    const rpy = item?.pose?.rpy;
    const dimensions = item?.dimensions;

    if (!Array.isArray(xyz) || xyz.length < 3 ||
        !Array.isArray(rpy) || rpy.length < 3 ||
        !Array.isArray(dimensions) || dimensions.length < 3 ||
        ![...xyz.slice(0, 3), ...rpy.slice(0, 3), ...dimensions.slice(0, 3)].every(Number.isFinite) ||
        dimensions[0] <= 0 || dimensions[1] <= 0 || dimensions[2] <= 0) continue;

    // Semantic support-surface fallback is intentionally limited to horizontal
    // authored surfaces. Tilted physical surfaces must be resolved by raycast.
    if (Math.abs(rpy[0]) > 1e-6 || Math.abs(rpy[1]) > 1e-6) continue;

    const topZ = xyz[2] + dimensions[2] / 2;
    const hit = new THREE.Vector3();
    const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -topZ);
    if (!raycaster.ray.intersectPlane(plane, hit)) continue;

    // Respect the authored rectangular footprint, including yaw.
    const dx = hit.x - xyz[0];
    const dy = hit.y - xyz[1];
    const cosYaw = Math.cos(rpy[2]);
    const sinYaw = Math.sin(rpy[2]);
    const localX = cosYaw * dx + sinYaw * dy;
    const localY = -sinYaw * dx + cosYaw * dy;

    if (Math.abs(localX) > dimensions[0] / 2 + 1e-6 ||
        Math.abs(localY) > dimensions[1] / 2 + 1e-6) continue;

    const origin = raycaster.ray.origin;
    const distance = origin &&
      [origin.x, origin.y, origin.z].every(Number.isFinite)
      ? Math.hypot(hit.x - origin.x, hit.y - origin.y, hit.z - origin.z)
      : 0;

    if (distance < bestDistance) {
      bestDistance = distance;
      bestPoint = finitePlacementPoint(hit);
      bestOwnerId = String(item?.support_surface_ref || item?.id || '').trim();
    }
  }

  if (bestPoint && state.placement.armed) state.placement.supportOwnerId = bestOwnerId;
  return bestPoint;
}
// Typed input contract: {clientX, clientY}, in browser client pixels. The point
// must lie inside the current canvas bounds; malformed and out-of-range input fails.
function placementPointFromViewport(position, options = {}) {
  if (state.placement.armed) state.placement.supportOwnerId = '';
  const clientX = position?.clientX;
  const clientY = position?.clientY;
  const rect = el.canvas?.getBoundingClientRect?.();
  const { raycaster, pointer, camera } = state.three;
  if (!Number.isFinite(clientX) || !Number.isFinite(clientY) || !rect || !Number.isFinite(rect.left) ||
      !Number.isFinite(rect.top) || !Number.isFinite(rect.width) || !Number.isFinite(rect.height) ||
      rect.width <= 0 || rect.height <= 0 || clientX < rect.left || clientY < rect.top ||
      clientX >= rect.left + rect.width || clientY >= rect.top + rect.height || !raycaster || !pointer || !camera) return null;
  pointer.x = ((clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
  const roots = [...state.objects.map(record => record.object3d), ...state.pickRecords.map(record => record.pickRoot || record.object3d)]
    .filter((root, index, all) => root?.visible !== false && all.indexOf(root) === index);
  for (const hit of raycaster.intersectObjects(roots, true) || []) {
    if (!isEligiblePlacementSurfaceHit(hit)) continue;
    const rendered = placementHitRendered(hit);
    const resolvedPoint = placementPointForPhysicalSurfaceItem(rendered?.item, hit.point);
    if (state.placement.armed) state.placement.supportOwnerId = String(explicitUiSelectionItemId(rendered) || rendered?.item?.support_surface_ref || rendered?.item?.id || '').trim();
    return resolvedPoint;
  }

  const supportSurfacePoint = placementPointOnAuthoredSupportSurface(raycaster);
  if (supportSurfacePoint) {
    return supportSurfacePoint;
  }

  if (options.allowGround === false) return null;
  if (!THREE?.Plane || !THREE?.Vector3 || !raycaster.ray?.intersectPlane) return null;
  const fallback = new THREE.Vector3();
  const groundPoint = raycaster.ray.intersectPlane(
    new THREE.Plane(new THREE.Vector3(0, 0, 1), 0),
    fallback
  ) ? finitePlacementPoint(fallback) : null;
  return groundPoint;
}
function markPlacementPreviewHelper(root) {
  root.name = 'placement_preview_helper';
  root.traverse?.(node => {
    node.userData.placement_preview = true;
    node.userData.exclude_from_physical_bounds = true;
    node.userData.exclude_from_fit_bounds = true;
    node.userData.selectable = false;
  });
  return root;
}
function markPlacementPreviewPhysicalVisual(root) {
  root?.traverse?.(node => {
    node.userData.placement_preview = true;
    node.userData.placement_preview_physical = true;
    node.userData.exclude_from_physical_bounds = false;
    node.userData.exclude_from_fit_bounds = true;
    node.userData.selectable = false;
  });
  return root;
}
function collectPlacementPreviewPhysicalBounds(root) {
  if (!root || !THREE?.Box3) return { count: 0, bounds: null, bounds_json: null };
  root.updateWorldMatrix?.(true, true);
  const bounds = new THREE.Box3();
  let count = 0;
  root.traverse?.(node => {
    if (node?.visible === false || node?.userData?.placement_preview_physical !== true) return;
    if (!node.isMesh) return;
    const helperReasons = physicalBoundsHelperReasons(
      node, node.userData?.item || {}, physicalBoundsNodeIdentityFor(node), { authoritativePhysical: true }
    ).filter(reason => reason !== 'node_exclude_from_fit_bounds');
    if (helperReasons.length) return;
    const nodeBounds = finiteBox3(new THREE.Box3().setFromObject(node));
    if (!nodeBounds) return;
    bounds.union(nodeBounds);
    count += 1;
  });
  const finite = count ? finiteBox3(bounds) : null;
  return { count: finite ? count : 0, bounds: finite, bounds_json: box3ToJson(finite) };
}
function stylePlacementPreview(root) {
  root?.traverse?.(node => {
    if (!node?.isMesh || !node.material) return;
    const originals = Array.isArray(node.material) ? node.material : [node.material];
    const previews = originals.map(material => {
      const preview = material.clone?.() || material;
      preview.transparent = true;
      preview.opacity = 0.5;
      preview.depthWrite = false;
      if (preview.color?.getHex) preview.userData = { ...(preview.userData || {}), placement_valid_color: preview.color.getHex() };
      return preview;
    });
    node.material = Array.isArray(node.material) ? previews : previews[0];
  });
}
function setPlacementCollisionStyle(collision) {
  state.placement.previewRoot?.traverse?.(node => {
    if (!node?.isMesh || !node.material) return;
    for (const material of (Array.isArray(node.material) ? node.material : [node.material])) {
      if (!material?.color?.setHex) continue;
      if (material.userData?.placement_valid_color === undefined && material.color.getHex) {
        material.userData = { ...(material.userData || {}), placement_valid_color: material.color.getHex() };
      }
      material.color.setHex(collision ? 0xd96b5f : material.userData.placement_valid_color);
    }
  });
}
function boxesPenetrate(a, b, epsilon = PLACEMENT_COLLISION_EPSILON) {
  return Boolean(a && b &&
    Math.min(a.max.x, b.max.x) - Math.max(a.min.x, b.min.x) > epsilon &&
    Math.min(a.max.y, b.max.y) - Math.max(a.min.y, b.min.y) > epsilon &&
    Math.min(a.max.z, b.max.z) - Math.max(a.min.z, b.min.z) > epsilon);
}
function placementPhysicalOwnerRecords() {
  const owners = new Map();
  for (const record of [...state.objects, ...state.pickRecords]) {
    if (!record?.object3d || record.object3d === state.placement.previewRoot || record.object3d.userData?.placement_preview === true) continue;
    const item = record.item;
    if (record.authoritativePhysicalPick !== true && !isPhysicalSemanticItem(item)) continue;
    const ownerId = String(explicitUiSelectionItemId(record) || item?.id || '').trim();
    if (ownerId && !owners.has(ownerId)) owners.set(ownerId, record);
  }
  return owners;
}
function evaluatePlacementCollision() {
  const placement = state.placement;
  placement.collision = false;
  placement.collidingOwnerIds = [];
  if (!placement.supportValid || !placement.previewRoot || !THREE?.Box3) {
    placement.valid = false;
    setPlacementCollisionStyle(false);
    return placement.valid;
  }
  placement.previewRoot.updateWorldMatrix?.(true, true);
  const ghostBounds = collectPlacementPreviewPhysicalBounds(placement.previewRoot).bounds;
  if (ghostBounds) {
    for (const [ownerId, record] of placementPhysicalOwnerRecords()) {
      if (ownerId === placement.supportOwnerId) continue;
      const physical = collectSelectionPhysicalBounds(record);
      if (boxesPenetrate(ghostBounds, physical.bounds)) placement.collidingOwnerIds.push(ownerId);
    }
  }
  placement.collidingOwnerIds.sort();
  placement.collision = placement.collidingOwnerIds.length > 0;
  placement.valid = placement.supportValid && Boolean(ghostBounds) && !placement.collision;
  setPlacementCollisionStyle(placement.collision);
  return placement.valid;
}
function removePlacementPreview() {
  const root = state.placement.previewRoot;
  if (!root) return;
  state.three.scene?.remove?.(root);
  disposeOwnedObject3d(root);
  state.placement.previewRoot = null;
}
function placementSnapValue() {
  return el.snapToggle?.checked ? translationSnapValue() : null;
}
function proposedPlacementPoint(rawPoint) {
  const point = finitePlacementPoint(rawPoint);
  if (!point) return null;
  const snap = placementSnapValue();
  return snap ? { x: Math.round(point.x / snap) * snap, y: Math.round(point.y / snap) * snap, z: point.z } : point;
}
function placementPresetDefinition(id = state.placement.orientationPreset) {
  return Object.values(PLACEMENT_ORIENTATION_PRESETS).find(preset => preset.id === id) || PLACEMENT_ORIENTATION_PRESETS.Digit1;
}
function composePlacementOrientation() {
  const preset = placementPresetDefinition();
  if (!THREE?.Quaternion || !THREE?.Euler || !THREE?.Vector3) {
    if (state.placement.previewRoot?.rotation) state.placement.previewRoot.rotation.z = state.placement.yaw;
    return null;
  }
  const presetQuaternion = new THREE.Quaternion().setFromEuler(new THREE.Euler(preset.roll, preset.pitch, 0, 'XYZ'));
  const yawQuaternion = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), state.placement.yaw);
  state.placement.orientationQuaternion = yawQuaternion.multiply(presetQuaternion);
  state.placement.previewRoot?.quaternion?.copy?.(state.placement.orientationQuaternion);
  return state.placement.orientationQuaternion;
}
function placementCanonicalRpy() {
  const quaternion = state.placement.orientationQuaternion || composePlacementOrientation();
  if (!quaternion || !THREE?.Euler) return { roll: 0, pitch: 0, yaw: state.placement.yaw };
  const euler = new THREE.Euler().setFromQuaternion(quaternion, 'XYZ');
  return { roll: euler.x, pitch: euler.y, yaw: euler.z };
}
function placementRequest(point, repeat) {
  return { ...point, ...placementCanonicalRpy(), repeat };
}
function contactCorrectPlacementPoint(supportPoint) {
  const placement = state.placement;
  const root = placement.previewRoot;
  if (!supportPoint || !root) return null;
  composePlacementOrientation();
  root.position.set(supportPoint.x, supportPoint.y, supportPoint.z);
  const initialBounds = collectPlacementPreviewPhysicalBounds(root).bounds;
  if (!initialBounds) {
    placement.invalidReason = 'placement preview has no finite physical bounds';
    return null;
  }
  root.position.z += supportPoint.z - initialBounds.min.z;
  const correctedBounds = collectPlacementPreviewPhysicalBounds(root).bounds;
  if (!correctedBounds || Math.abs(supportPoint.z - correctedBounds.min.z) > PLACEMENT_CONTACT_EPSILON) {
    placement.invalidReason = 'placement preview physical contact correction failed';
    return null;
  }
  return { x: root.position.x, y: root.position.y, z: root.position.z };
}
function updatePlacementStatus() {
  const status = el.placementStatus;
  if (!status) return;
  status.hidden = !state.placement.armed;
  status.className = `placement-status ${state.placement.collision ? 'collision' : (state.placement.valid === true ? 'valid' : 'invalid')}`;
  if (!state.placement.armed) {
    status.textContent = '';
    if (state.lastPlacementHealthSignature !== 'false|false|') {
      state.lastPlacementHealthSignature = 'false|false|';
      renderSceneHealth();
    }
    return;
  }
  const point = state.placement.proposedPoint;
  const snap = placementSnapValue();
  const collisions = state.placement.collidingOwnerIds || [];
  const orientation = placementPresetDefinition().label;
  status.title = collisions.join(', ');
  status.innerHTML = point
    ? state.placement.collision
      ? `<strong>COLLISION</strong> · ${collisions[0]}${collisions.length > 1 ? ` + ${collisions.length - 1} more` : ''}<br>Orientation: ${orientation} · Q/E yaw · 1–6 preset · Esc cancel`
      : `<strong>VALID</strong> · X ${point.x.toFixed(3)} · Y ${point.y.toFixed(3)} · Z ${point.z.toFixed(3)} m${snap ? ` · SNAP ${snap.toFixed(3)} m` : ' · SNAP OFF'}<br>Orientation: ${orientation} · Q/E yaw · 1–6 preset · Esc cancel`
    : `<strong>INVALID</strong> · ${state.placement.supportValid ? (state.placement.invalidReason || 'Placement preview unavailable') : 'No valid placement surface'}${snap ? ` · SNAP ${snap.toFixed(3)} m` : ' · SNAP OFF'}<br>${state.placement.supportValid ? 'Wait for physical geometry · Esc cancel' : 'Move over a physical support surface · Esc cancel'}`;
  const healthSignature = `${Boolean(state.placement.armed)}|${Boolean(state.placement.collision)}|${asArray(state.placement.collidingOwnerIds).join('|')}`;
  if (state.lastPlacementHealthSignature !== healthSignature) {
    state.lastPlacementHealthSignature = healthSignature;
    renderSceneHealth();
  }
}
function setPlacementPoint(rawPoint) {
  state.placement.invalidReason = '';
  state.placement.rawPoint = finitePlacementPoint(rawPoint);
  state.placement.supportPoint = proposedPlacementPoint(state.placement.rawPoint);
  state.placement.proposedPoint = contactCorrectPlacementPoint(state.placement.supportPoint);
  state.placement.supportValid = Boolean(state.placement.supportPoint);
  const root = state.placement.previewRoot;
  if (root) {
    root.visible = Boolean(state.placement.proposedPoint);
    const point = state.placement.proposedPoint;
    if (point) root.position.set(point.x, point.y, point.z);
  }
  evaluatePlacementCollision();
  updatePlacementStatus();
  return state.placement.proposedPoint;
}
async function createPlacementPreview(asset) {
  removePlacementPreview();
  if (!state.placement.armed || !state.three.scene || !THREE?.Group) return null;
  const root = markPlacementPreviewHelper(new THREE.Group());
  root.visible = false;
  state.placement.previewRoot = root;
  composePlacementOrientation();
  state.three.scene.add(root);

  const placeholder = new THREE.Mesh(
    new THREE.BoxGeometry(0.08, 0.08, 0.08),
    new THREE.MeshStandardMaterial({ color: 0x8aa38d, transparent: true, opacity: 0.5, depthWrite: false })
  );
  root.add(placeholder);
  markPlacementPreviewHelper(placeholder);

  const uri = safeMeshUri(asset);
  if (!uri) return root;
  try {
    const ext = meshExtensionFromUri(uri);
    const loader = ext === 'stl' ? new STLLoader() : (ext === 'dae' ? new ColladaLoader() : new OBJLoader());
    const loaded = await loadMeshWithDeadline(loader, repoRootRelativeUrl(uri));
    if (state.placement.previewRoot !== root || !state.placement.armed) return root;
    const mesh = materializeLoadedMesh(asset, uri, loaded);
    const visual = makeMeshVisualRoot(asset, mesh);
    stylePlacementPreview(visual);
    markPlacementPreviewHelper(visual);
    markPlacementPreviewPhysicalVisual(visual);
    root.remove(placeholder);
    disposeOwnedObject3d(placeholder);
    root.add(visual);
    setPlacementPoint(state.placement.rawPoint);
  } catch (_) {
    // Keep placement armed, but the non-physical loading placeholder cannot be committed.
  }
  return root;
}
function updatePlacementPreview(position) {
  if (!state.placement.armed) return null;
  return setPlacementPoint(placementPointFromViewport(position, { allowGround: false }));
}
function updatePlacementPointer(clientX, clientY) {
  if (!Number.isFinite(clientX) || !Number.isFinite(clientY)) return null;
  return updatePlacementPreview({ clientX, clientY });
}
function commitPlacementPointer(clientX, clientY) {
  const point = updatePlacementPointer(clientX, clientY);
  const valid = Boolean(point && state.placement.valid);
  if (valid) pushEditorEvent('placement_requested', placementRequest(point, false)); // repeat: false for drag/drop
  cancelPlacement();
  return valid;
}
function getPlacementState() { return { armed: state.placement.armed, persistent: state.placement.persistent, supportValid: state.placement.supportValid, collision: state.placement.collision, collidingOwnerIds: [...(state.placement.collidingOwnerIds || [])], valid: state.placement.valid, proposedPoint: state.placement.proposedPoint ? { ...state.placement.proposedPoint } : null }; }
function armPlacement(options = {}) {
  if (options === null || typeof options !== 'object' || Array.isArray(options)) return null;
  cancelPlacement();
  const asset = options.asset && typeof options.asset === 'object' && !Array.isArray(options.asset) ? { ...options.asset } : {};
  state.placement = { armed: true, persistent: options.persistent === true, previewRoot: null, asset, orientationPreset: 'upright', orientationQuaternion: null, yaw: 0, rawPoint: null, supportPoint: null, proposedPoint: null, supportOwnerId: '', supportValid: false, collision: false, collidingOwnerIds: [], valid: false };
  createPlacementPreview(asset);
  el.canvas?.classList?.add?.('placement-armed');
  if (el.canvas?.style) el.canvas.style.cursor = 'crosshair';
  el.canvas?.setAttribute?.('aria-label', 'Drop to place · Q/E rotate · Esc cancel');
  updatePlacementStatus();
  return getPlacementState();
}
function cancelPlacement() {
  removePlacementPreview();
  state.placement = { armed: false, persistent: false, previewRoot: null, asset: null, orientationPreset: 'upright', orientationQuaternion: null, yaw: 0, rawPoint: null, supportPoint: null, proposedPoint: null, supportOwnerId: '', supportValid: false, collision: false, collidingOwnerIds: [], valid: null };
  el.canvas?.classList?.remove?.('placement-armed');
  if (el.canvas?.style) el.canvas.style.cursor = '';
  el.canvas?.setAttribute?.('aria-label', 'Workcell 3D canvas');
  updatePlacementStatus();
  return getPlacementState();
}
function pickObject(event) {
  const rect = el.canvas.getBoundingClientRect();
  state.three.pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  state.three.pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  state.three.raycaster.setFromCamera(state.three.pointer, state.three.camera);
  const rootCandidates = [...state.objects.map(o => o.object3d), ...state.pickRecords.map(o => o.pickRoot || o.object3d)]
    .filter((root, index, all) => root?.visible !== false && !excludedPickNode(root) && all.indexOf(root) === index);
  const candidateSet = new Set(rootCandidates);
  const roots = rootCandidates.filter(root => {
    for (let ancestor = root.parent; ancestor; ancestor = ancestor.parent) if (candidateSet.has(ancestor)) return false;
    return true;
  });
  const rawHits = state.three.raycaster.intersectObjects(roots, true);
  const hits = rawHits.filter(hit => !hiddenPickSubtree(hit?.object));
  const candidates = rankedPickingCandidates(hits);
  state.lastRaycastHitCount = hits.length;
  state.lastRaycastCandidateIds = candidates.map(candidate => candidate.rendered.item.id);
  const selectedCandidate = candidates.find(candidate => Number.isFinite(candidate.priority) && candidate.eligible);
  if (!selectedCandidate) {
    state.lastCanvasSelectedItemId = '';
    state.lastCanvasPickReason = hits.length ? 'no_eligible_candidate' : 'empty_select_click';
    state.lastCanvasPickDiagnostic = null;
    state.lastFailedCanvasPickDiagnostic = hits.length ? failedCanvasPickDiagnostic(hits) : null;
    if (state.lastFailedCanvasPickDiagnostic) {
      const diagnostic = state.lastFailedCanvasPickDiagnostic;
      const signature = ['failed_canvas_pick', sceneId(), diagnostic.raw_hit_count, diagnostic.hit_object_names.join('|'), diagnostic.first_actionable_rejection_reason, diagnostic.nearest_known_urdf_link_ancestor].join('\n');
      if (!state.diagnosticKeys.has(signature)) {
        state.diagnosticKeys.add(signature);
        console.warn?.(`Product View canvas pick rejected: ${JSON.stringify(diagnostic)}`);
      }
    }
    clearSelection();
    return '';
  }
  const skippedHelper = candidates.find(candidate => candidate !== selectedCandidate && candidate.priority > selectedCandidate.priority && (isTaskOnlyHelperItem(candidate.rendered.item) || isDebugOverlayItem(candidate.rendered.item)));
  if (skippedHelper) {
    state.skippedHelperPickKeys ||= new Set();
    const warningKey = `${sceneId()}|${skippedHelper.rendered.item.id}|${selectedCandidate.rendered.item.id}`;
    if (!state.skippedHelperPickKeys.has(warningKey)) {
      state.skippedHelperPickKeys.add(warningKey);
      pushEditorEvent('helper_pick_skipped', { helperItemId: skippedHelper.rendered.item.id, selectedItemId: selectedCandidate.rendered.item.id, sceneId: sceneId() });
    }
  }
  const canonicalId = selectedCandidate.selectionOwner?.item?.id || explicitUiSelectionItemId(selectedCandidate.rendered);
  selectObjectFromRender(canonicalId, selectedCandidate.rendered);
  state.lastCanvasSelectedItemId = canonicalId;
  state.lastCanvasPickReason = 'eligible_candidate';
  state.lastCanvasPickDiagnostic = {
    pointer: { clientX: event.clientX, clientY: event.clientY },
    canvasRect: { left: rect.left, top: rect.top, width: rect.width, height: rect.height },
    ndc: { x: state.three.pointer.x, y: state.three.pointer.y },
    rawHitCount: hits.length,
    selectedItemId: canonicalId,
    selectedRenderIdentityId: selectedCandidate.rendered?.item?.id || '',
    candidates: candidates.slice(0, 16).map(candidate => ({
      renderIdentityId: candidate.rendered?.item?.id || '',
      selectionOwnerId: candidate.selectionOwner?.item?.id || '',
      hitNodeName: candidate.hit?.object?.name || '',
      distance: Number(candidate.hit?.distance),
      priority: candidate.priority,
      authoritativePhysicalPick: candidate.rendered?.authoritativePhysicalPick === true,
      pickRecordSource: candidate.rendered?.pickRecordSource || '',
    })),
  };
  state.lastFailedCanvasPickDiagnostic = null;
  return canonicalId;
}



function pointerToWorldPlane(event, z) { const rect = el.canvas.getBoundingClientRect(); if (!rect.width || !rect.height) return null; state.three.pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1; state.three.pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1; state.three.raycaster.setFromCamera(state.three.pointer, state.three.camera); const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -z); const hit = new THREE.Vector3(); if (!state.three.raycaster.ray.intersectPlane(plane, hit)) return null; return Number.isFinite(hit.x) && Number.isFinite(hit.y) && Number.isFinite(hit.z) ? hit : null; }
function snapHorizontalPreview(transform) { const snapped = snapTransform(transform, { translationAxes: ['x', 'y'], rotationAxes: [] }); snapped.pose.xyz.z = transform.pose.xyz.z; return snapped; }
function syncOrbitControlsForEditorMode() { if (state.three.controls) state.three.controls.enabled = !Boolean(state.three.transformControls?.dragging); }
function beginDirectMoveDrag(event, rendered) { if (state.editorMode !== 'move' || !selectionIsEditable(rendered)) return false; const start = cloneTransform(state.dirtyTransforms.get(rendered.item.id)?.newTransform || transformFromObject(rendered.object3d)); const hit = pointerToWorldPlane(event, start.pose.xyz.z); if (!hit) return false; state.directMoveDrag = { itemId: rendered.item.id, start, groupStart: captureTransformGroup(rendered), last: cloneTransform(start), offset: { x: start.pose.xyz.x - hit.x, y: start.pose.xyz.y - hit.y } }; syncOrbitControlsForEditorMode(); el.canvas.setPointerCapture?.(event.pointerId); el.canvas.classList.add('direct-move-dragging'); event.preventDefault(); event.stopPropagation(); return true; }
function updateDirectMoveDrag(event) { const drag = state.directMoveDrag; if (!drag) return false; const rendered = renderedById(drag.itemId); if (!rendered || state.selected !== drag.itemId || !selectionIsEditable(rendered)) return cancelDirectMoveDrag('Move cancelled'), true; const hit = pointerToWorldPlane(event, drag.start.pose.xyz.z); if (!hit) return true; const next = cloneTransform(drag.start); next.pose.xyz.x = hit.x + drag.offset.x; next.pose.xyz.y = hit.y + drag.offset.y; next.pose.xyz.z = drag.start.pose.xyz.z; const preview = snapHorizontalPreview(next); if (!isFiniteTransform(preview)) return true; drag.last = cloneTransform(preview); applyTransformChanges(linkedTransformChanges(rendered, drag.start, preview, drag.groupStart)); syncInspectorTransformFields(rendered); event.preventDefault(); event.stopPropagation(); return true; }
function finishDirectMoveDrag(event) { const drag = state.directMoveDrag; if (!drag) return false; const rendered = renderedById(drag.itemId); const finalTransform = cloneTransform(drag.last); endDirectMoveDrag(event); if (!selectionIsEditable(rendered) || !isFiniteTransform(finalTransform)) return false; if (sameTransform(drag.start, finalTransform)) { applyTransformChanges([...drag.groupStart].map(([itemId, after]) => ({ rendered: renderedById(itemId), after }))); syncInspectorTransformFields(rendered); return true; } const committed = markDirtyTransform(rendered, finalTransform, { pushHistory: true, oldTransform: drag.start, memberStarts: drag.groupStart }); if (!committed) { applyTransformChanges([...drag.groupStart].map(([itemId, after]) => ({ rendered: renderedById(itemId), after }))); showError(`Move failed for ${itemLabel(rendered.item)}: final transform was rejected by the editor bridge.`); return true; } emitTransformCommitted(rendered); pushEditorEvent('status', { message: `Moved ${itemLabel(rendered.item)}` }); return true; }
function endDirectMoveDrag(event) { state.directMoveDrag = null; syncOrbitControlsForEditorMode(); el.canvas.classList.remove('direct-move-dragging'); if (event?.pointerId !== undefined) el.canvas.releasePointerCapture?.(event.pointerId); }
function cancelActiveTransformOperation(reason = 'Transform cancelled') {
  if (state.cancellingTransformOperation) return false;
  const drag = state.directMoveDrag || state.directRotateDrag;
  const groupStart = drag?.groupStart || state.gizmoDragGroupStart;
  const owner = renderedById(drag?.itemId || '') || canonicalTransformOwner(state.selected) || state.gizmoPivot?.owner;
  const ownerStart = drag?.start || state.gizmoDragStart;
  const active = Boolean(drag || state.gizmoDragStart || state.gizmoDragGroupStart || state.gizmoPivotDragStart);
  if (!active) { syncOrbitControlsForEditorMode(); return false; }
  state.cancellingTransformOperation = true;
  try {
    // reset() can synchronously emit objectChange/dragging-changed; the guard above
    // prevents those events from previewing or committing the cancelled pose.
    state.three.transformControls?.reset?.();
    if (groupStart) applyTransformChanges([...groupStart].map(([itemId, after]) => ({ rendered: renderedById(itemId), after })));
    if (owner && ownerStart) applyTransformToObject(owner.object3d, ownerStart);
    state.gizmoDragStart = null;
    state.gizmoDragGroupStart = null;
    state.gizmoPivotDragStart = null;
    state.directMoveDrag = null;
    state.directRotateDrag = null;
    el.canvas.classList.remove('direct-move-dragging');
    if (owner && state.gizmoPivot?.owner === owner) refreshTransientGizmoPivot(owner, 'cancelled_transform_restored');
    updateLabels();
    if (owner) syncInspectorTransformFields(owner);
    pushEditorEvent('status', { message: reason || 'Transform cancelled' });
  } finally {
    state.cancellingTransformOperation = false;
    syncOrbitControlsForEditorMode();
  }
  return true;
}
function cancelDirectMoveDrag(message) { return cancelActiveTransformOperation(message || 'Move cancelled'); }
function transformControlsOwnsPointerDown(event, transformControls = state.three.transformControls) {
  if (!transformControls || state.editorMode === 'select') return false;
  // Once TransformControls has begun a drag it owns the pointer until release,
  // even if the pointer has left the handle or the active axis is reset.
  if (transformControls.dragging) return true;

  const clientX = event?.clientX;
  const clientY = event?.clientY;
  const rect = el.canvas?.getBoundingClientRect?.();
  const camera = state.three.camera;
  if (!Number.isFinite(clientX) || !Number.isFinite(clientY) || !rect ||
      !Number.isFinite(rect.left) || !Number.isFinite(rect.top) ||
      !Number.isFinite(rect.width) || !Number.isFinite(rect.height) ||
      rect.width <= 0 || rect.height <= 0 || !camera) return false;

  // TransformControls deliberately keeps its picker geometry separate from the
  // visible gizmo. Raycast that picker at this event's coordinates instead of
  // trusting `axis`, which is hover state from an earlier pointer position.
  const mode = transformControls.mode || (state.editorMode === 'rotate' ? 'rotate' : 'translate');
  const helper = transformControls.getHelper?.() || transformControls._gizmo;
  const picker = transformControls._gizmo?.picker?.[mode] || helper?.picker?.[mode];
  const raycaster = transformControls.getRaycaster?.() || transformControls._raycaster || state.three.raycaster;
  if (!picker || !raycaster?.setFromCamera || !raycaster?.intersectObject) return false;

  const pointer = { x: ((clientX - rect.left) / rect.width) * 2 - 1,
    y: -((clientY - rect.top) / rect.height) * 2 + 1 };
  raycaster.setFromCamera(pointer, camera);
  return (raycaster.intersectObject(picker, true) || []).some(hit => hit?.object);
}
function onCanvasPointerDown(event) {
  if (event.button === 0) {
    if (state.placement.armed) {
      const point = updatePlacementPreview({ clientX: event.clientX, clientY: event.clientY });
      if (!point || !state.placement.valid) return;
      pushEditorEvent('placement_requested', placementRequest(point, event.shiftKey === true));
      if (event.shiftKey !== true) cancelPlacement();
      event.preventDefault?.();
      event.stopPropagation?.();
      return;
    }
    // TransformControls owns pointer-down while dragging or when the current
    // pointer position actually intersects its picker geometry.
    // Do not raycast through the gizmo and accidentally replace the active
    // authored owner with another physical object behind the handle.
    const gizmoOwnsPointer = transformControlsOwnsPointerDown(event);
    if (gizmoOwnsPointer) return;

    pickObject(event);
  }
}
function onCanvasPointerMove(event) {
  if (state.placement.armed) updatePlacementPreview({ clientX: event.clientX, clientY: event.clientY });
}
function onCanvasPointerUp(event) {}
function onCanvasPointerCancel() { cancelActiveTransformOperation('Pointer cancelled'); }
function onCanvasContextMenu(event) {
  event.preventDefault();
  if (state.placement.armed) return;
  pickObject(event);
  pushEditorEvent('context_menu_requested', {
    clientX: Number(event.clientX),
    clientY: Number(event.clientY),
    itemId: state.selected || '',
  });
}
function keyboardEventTargetsEditorControl(event) {
  const target = event?.target;
  if (!target) return false;
  const tagName = String(target.tagName || '').toLowerCase();
  return Boolean(target.isContentEditable || ['input', 'textarea', 'select', 'button'].includes(tagName) || target.closest?.('input, textarea, select, button, [contenteditable="true"], [contenteditable=""]'));
}
function keyboardTranslationStep(event) {
  if (event.shiftKey) return 0.001;
  return el.snapToggle?.checked && translationSnapValue() ? translationSnapValue() : 0.01;
}
function keyboardRotationStepRadians(event) {
  if (event.shiftKey) return THREE.MathUtils.degToRad(1);
  return el.snapToggle?.checked && rotationSnapRadians() ? rotationSnapRadians() : THREE.MathUtils.degToRad(5);
}
function keyboardTransformCommand(event) {
  if (event.ctrlKey || event.metaKey || event.altKey) return null;
  const translations = { KeyW: ['y', 1], KeyS: ['y', -1], KeyA: ['x', 1], KeyD: ['x', -1], PageUp: ['z', 1], PageDown: ['z', -1] };
  const rotations = { KeyQ: ['z', -1], KeyE: ['z', 1], KeyR: ['y', 1], KeyF: ['y', -1], KeyZ: ['x', -1], KeyC: ['x', 1] };
  if (translations[event.code]) return { kind: 'translation', component: translations[event.code][0], direction: translations[event.code][1] };
  if (rotations[event.code]) return { kind: 'rotation', component: rotations[event.code][0], direction: rotations[event.code][1] };
  return null;
}
function applyKeyboardTransformStep(event, command) {
  if (!command || event.repeat || state.directMoveDrag || state.directRotateDrag || state.gizmoDragStart || state.gizmoDragGroupStart || state.gizmoPivotDragStart) return false;
  const owner = canonicalTransformOwner(state.selected);
  if (!selectionIsEditable(owner)) return false;
  const start = cloneTransform(state.dirtyTransforms.get(owner.item.id)?.newTransform || transformFromObject(owner.object3d));
  const next = cloneTransform(start);
  const step = command.kind === 'translation' ? keyboardTranslationStep(event) : keyboardRotationStepRadians(event);
  if (!Number.isFinite(step) || step <= 0) return false;
  if (command.kind === 'translation') next.pose.xyz[command.component] += command.direction * step;
  else next.pose.rpy[command.component] += command.direction * step;
  const committed = markDirtyTransform(owner, next, { pushHistory: true, oldTransform: start, snapOptions: null, memberStarts: captureTransformGroup(owner) });
  if (!committed) return false;
  emitTransformCommitted(owner);
  syncInspectorTransformFields(owner);
  updateLabels();
  if (state.gizmoPivot?.owner === owner) refreshTransientGizmoPivot(owner, 'keyboard_transform_committed');
  return true;
}
function onEditorKeyDown(event) {
  if (keyboardEventTargetsEditorControl(event)) return;
  if (event.key === 'Escape') {
    if (state.placement.armed) { cancelPlacement(); event.preventDefault?.(); return; }
    if (cancelActiveTransformOperation('Escape')) event.preventDefault?.();
    return;
  }
  const placementPreset = state.placement.armed && !event.ctrlKey && !event.metaKey && !event.altKey
    ? PLACEMENT_ORIENTATION_PRESETS[event.code] : null;
  if (placementPreset) {
    state.placement.orientationPreset = placementPreset.id;
    composePlacementOrientation();
    setPlacementPoint(state.placement.rawPoint);
    event.preventDefault?.();
    return;
  }
  if (state.placement.armed && !event.ctrlKey && !event.metaKey && !event.altKey &&
      (event.code === 'KeyQ' || event.code === 'KeyE')) {
    const direction = event.code === 'KeyQ' ? -1 : 1;
    state.placement.yaw += direction * (Math.PI / 12);
    composePlacementOrientation();
    setPlacementPoint(state.placement.rawPoint);
    event.preventDefault?.();
    return;
  }
  const undoShortcut = (event.ctrlKey || event.metaKey) && !event.altKey && event.code === 'KeyZ' && !event.shiftKey;
  const redoShortcut = (event.ctrlKey || event.metaKey) && !event.altKey && (event.code === 'KeyY' || (event.code === 'KeyZ' && event.shiftKey));
  const copyShortcut = (event.ctrlKey || event.metaKey) && !event.altKey && event.code === 'KeyC';
  const pasteShortcut = (event.ctrlKey || event.metaKey) && !event.altKey && event.code === 'KeyV';
  if (copyShortcut && selectionIsEditable(canonicalTransformOwner(state.selected))) { copySelectedTransformToClipboard(); event.preventDefault?.(); return; }
  if (pasteShortcut && selectionIsEditable(canonicalTransformOwner(state.selected))) { pasteSelectedTransformFromClipboard(); event.preventDefault?.(); return; }
  if (undoShortcut && state.undoStack.length) { undoPreviewEdit(); event.preventDefault?.(); return; }
  if (redoShortcut && state.redoStack.length) { redoPreviewEdit(); event.preventDefault?.(); return; }
  if (applyKeyboardTransformStep(event, keyboardTransformCommand(event))) event.preventDefault?.();
}

function cancelDirectRotateDrag(message) {
  return cancelActiveTransformOperation(message || 'Rotation cancelled');
}

function authoritativePhysicalMeshCentre(owner) {
  const visual = resolveCanonicalPhysicalEditBinding(owner)?.visual;
  if (!visual?.meshObject || visual.item?.mesh_status !== 'loaded') return null;
  visual.meshObject.updateWorldMatrix(true, true);
  const bounds = new THREE.Box3();
  visual.meshObject.traverse(node => {
    if (!node.visible || node.isMesh !== true || !node.geometry) return;
    const identity = `${node.name || ''} ${node.userData?.role || ''} ${node.userData?.category || ''}`.toLowerCase();
    if (node.userData?.fallback_geometry === true || node.userData?.isFallback === true ||
        node.userData?.selection_highlight === true || node.userData?.exclude_from_physical_bounds === true ||
        /helper|frustum|label|highlight|fallback/.test(identity)) return;
    node.updateWorldMatrix(true, false);
    if (!node.geometry.boundingBox) node.geometry.computeBoundingBox();
    if (node.geometry.boundingBox && !node.geometry.boundingBox.isEmpty()) bounds.union(node.geometry.boundingBox.clone().applyMatrix4(node.matrixWorld));
  });
  if (bounds.isEmpty()) return null;
  const centre = bounds.getCenter(new THREE.Vector3());
  return Number.isFinite(centre.x) && Number.isFinite(centre.y) && Number.isFinite(centre.z) ? centre : null;
}
function removeTransientGizmoPivot() {
  const pivot = state.gizmoPivot;
  if (pivot?.group?.parent) pivot.group.parent.remove(pivot.group);
  state.gizmoPivot = null;
  state.gizmoPivotDragStart = null;
}
function refreshTransientGizmoPivot(owner, attachmentReason = 'physical_binding') {
  const binding = resolveCanonicalPhysicalEditBinding(owner);
  owner = binding?.owner || owner;
  if (!owner || state.selected !== owner.item?.id) return false;
  const centre = authoritativePhysicalMeshCentre(owner);
  if (!centre) { detachTransformGizmo('authoritative_physical_mesh_unavailable'); return false; }
  let pivot = state.gizmoPivot;
  if (!pivot || pivot.owner !== owner) {
    removeTransientGizmoPivot();
    const group = new THREE.Group();
    group.name = `${owner.item.id}_transient_gizmo_pivot`;
    group.userData.transient_gizmo_pivot = true;
    state.three.scene?.add(group);
    pivot = state.gizmoPivot = { owner, group };
  }
  const parent = pivot.group.parent;
  const localCentre = parent ? parent.worldToLocal(centre.clone()) : centre.clone();
  pivot.group.position.copy(localCentre);
  owner.object3d.getWorldQuaternion(pivot.group.quaternion);
  if (parent) pivot.group.quaternion.premultiply(parent.getWorldQuaternion(new THREE.Quaternion()).invert());
  pivot.group.scale.set(1, 1, 1);
  pivot.group.updateMatrixWorld(true);
  const attachedWorld = new THREE.Vector3().setFromMatrixPosition(pivot.group.matrixWorld);
  const distance = attachedWorld.distanceTo(centre);
  state.gizmoAttachmentDiagnostic = {
    targetId: owner.item.id, reason: attachmentReason, canonicalOwnerId: binding?.ownerId || owner.item.id,
    ownerRecordSource: binding?.ownerRecordSource || editAuthoritySource(owner.item), physicalVisualId: binding?.visual?.item?.id || '',
    pivotWorldCentre: { x: centre.x, y: centre.y, z: centre.z }, attachedObjectWorldPosition: { x: attachedWorld.x, y: attachedWorld.y, z: attachedWorld.z }, distance,
  };
  if (distance >= 1e-6) { detachTransformGizmo('physical_pivot_world_position_mismatch'); return false; }
  if (state.three.transformControls?.object !== pivot.group) state.three.transformControls?.attach(pivot.group);
  return true;
}
function beginTransientPivotDrag(owner) {
  const pivot = state.gizmoPivot;
  if (!pivot || pivot.owner !== owner) return false;
  const axis = state.editorMode === 'rotate' ? state.three.transformControls?.axis : null;
  if (state.editorMode === 'rotate' && !['X', 'Y', 'Z'].includes(axis)) return false;
  owner.object3d.updateWorldMatrix(true, false);
  pivot.group.updateWorldMatrix(true, false);
  state.gizmoDragStart = cloneTransform(state.dirtyTransforms.get(owner.item.id)?.newTransform || transformFromObject(owner.object3d));
  state.gizmoDragGroupStart = captureTransformGroup(owner);
  state.gizmoPivotDragStart = { ownerWorld: owner.object3d.matrixWorld.clone(), pivotWorld: pivot.group.matrixWorld.clone(), axis };
  return true;
}
function previewTransientPivotDrag(owner) {
  const start = state.gizmoPivotDragStart;
  const pivot = state.gizmoPivot;
  if (!start || !pivot || pivot.owner !== owner) return false;
  pivot.group.updateWorldMatrix(true, false);
  const delta = pivot.group.matrixWorld.clone().multiply(start.pivotWorld.clone().invert());
  const ownerWorld = delta.multiply(start.ownerWorld);
  const ownerLocal = owner.object3d.parent ? owner.object3d.parent.matrixWorld.clone().invert().multiply(ownerWorld) : ownerWorld;
  ownerLocal.decompose(owner.object3d.position, owner.object3d.quaternion, owner.object3d.scale);
  owner.object3d.scale.set(state.gizmoDragStart.scale.x, state.gizmoDragStart.scale.y, state.gizmoDragStart.scale.z);
  owner.object3d.updateMatrixWorld(true);
  const next = transformFromObject(owner.object3d);
  applyTransformChanges(linkedTransformChanges(owner, state.gizmoDragStart, next, state.gizmoDragGroupStart));
  syncInspectorTransformFields(owner);
  updateLabels();
  return true;
}
function finishTransientPivotDrag(owner) {
  if (!state.gizmoPivotDragStart) return false;
  previewTransientPivotDrag(owner);
  const finalTransform = transformFromObject(owner.object3d);
  const committed = finalTransform && !sameTransform(state.gizmoDragStart, finalTransform) && markDirtyTransform(owner, finalTransform, { pushHistory: true, oldTransform: state.gizmoDragStart, snapOptions: null, memberStarts: state.gizmoDragGroupStart });
  if (committed) emitTransformCommitted(owner);
  state.gizmoDragStart = null;
  state.gizmoDragGroupStart = null;
  state.gizmoPivotDragStart = null;
  return committed;
}

function attachTransformGizmo(rendered, attachmentReason = 'mode_or_selection') {
  const gizmo = state.three.transformControls;
  if (!gizmo) { state.gizmoAttachmentDiagnostic = { targetId: '', reason: 'transform_controls_unavailable' }; return; }
  if (state.editorMode === 'select') { detachTransformGizmo('select_mode'); return; }
  const binding = resolveCanonicalPhysicalEditBinding(rendered);
  if (binding) rendered = binding.owner;
  if (selectionIsEditable(rendered)) {
    const usesPhysicalPivot = Boolean(binding);
    if (!usesPhysicalPivot || !refreshTransientGizmoPivot(rendered, attachmentReason === 'asynchronous_physical_mesh_completion' ? attachmentReason : 'attached_to_authoritative_physical_centre')) {
      if (usesPhysicalPivot) return;
      removeTransientGizmoPivot();
      gizmo.attach(rendered.object3d);
    }
    gizmo.visible = true;
    gizmo.enabled = true;
    if (state.editorMode === 'rotate') { gizmo.setMode('rotate'); gizmo.showX = true; gizmo.showY = true; gizmo.showZ = true; }
    else { gizmo.setMode('translate'); gizmo.showX = true; gizmo.showY = true; gizmo.showZ = true; }
    gizmo.setSpace(state.transformSpace);
    gizmo.enabled = state.editorMode !== 'select';
    gizmo.setTranslationSnap(el.snapToggle?.checked ? translationSnapValue() : null);
    gizmo.setRotationSnap(el.snapToggle?.checked ? rotationSnapRadians() : null);
    if (!usesPhysicalPivot) state.gizmoAttachmentDiagnostic = { targetId: rendered.item.id, reason: 'attached_to_canonical_edit_owner' };
  } else {
    const authority = editAuthorityForItem(rendered?.item);
    const refusal = rendered && (isTaskOnlyHelperItem(rendered.item) || isDebugOverlayItem(rendered.item))
      ? 'helper_or_overlay_record'
      : (rendered?.readOnlyPick ? 'read_only_render_identity' : (authority.eligible ? 'not_canonical_edit_owner' : authority.reason));
    detachTransformGizmo(refusal);
  }
}
function detachTransformGizmo(reason = 'detached', { skipCancel = false } = {}) {
  if (!skipCancel) cancelActiveTransformOperation(reason);
  const gizmo = state.three.transformControls;
  state.gizmoAttachmentDiagnostic = { targetId: '', reason };
  if (!gizmo) return;
  gizmo.detach();
  gizmo.visible = false;
  gizmo.enabled = false;
  removeTransientGizmoPivot();
}
function refreshGizmoSnap() {
  const gizmo = state.three.transformControls;
  if (!gizmo) return;
  gizmo.setTranslationSnap(el.snapToggle?.checked ? translationSnapValue() : null);
  gizmo.setRotationSnap(el.snapToggle?.checked ? rotationSnapRadians() : null);
}
function editAuthoritySource(item) {
  return String(item?.source_layer || item?.authoring_source_layer || item?.render_owner || item?.source_kind || '').trim();
}
function editAuthorityForItem(item) {
  if (!item) return { eligible: false, source: '', reason: 'missing_item' };
  if (item.locked === true) return { eligible: false, source: editAuthoritySource(item), reason: 'owner_locked' };
  if (item.editable !== true) return { eligible: false, source: editAuthoritySource(item), reason: 'owner_not_editable' };
  if (isDiagnosticOnlyItem(item)) return { eligible: false, source: editAuthoritySource(item), reason: 'diagnostic_record' };
  const source = editAuthoritySource(item);
  const normalizedSource = source.toLowerCase();
  const authoredSource = /(?:^|[_\s-])(?:editable|authored|authoring|layout)(?:$|[_\s-])/.test(normalizedSource);
  if (!authoredSource) return { eligible: false, source, reason: normalizedSource.includes('generated') ? 'generated_authority_layer' : 'non_authored_authority_layer' };
  // active_visual_source and renderer/mesh provenance deliberately do not
  // participate here: they describe how an authored owner is drawn, not who
  // owns its transform.
  return { eligible: true, source, reason: 'authored_edit_authority' };
}
function canEditItem(item) {
  return editAuthorityForItem(item).eligible;
}
const TRANSFORM_FIELD_SPECS = {
  x: { group: 'xyz', component: 'x', label: 'X', decimals: 4, step: '0.0001' },
  y: { group: 'xyz', component: 'y', label: 'Y', decimals: 4, step: '0.0001' },
  z: { group: 'xyz', component: 'z', label: 'Z', decimals: 4, step: '0.0001' },
  roll: { group: 'rpy', component: 'x', label: 'Roll', decimals: 2, step: '0.01' },
  pitch: { group: 'rpy', component: 'y', label: 'Pitch', decimals: 2, step: '0.01' },
  yaw: { group: 'rpy', component: 'z', label: 'Yaw', decimals: 2, step: '0.01' },
};
function canonicalTransformForRendered(rendered) {
  const dirty = state.dirtyTransforms.get(rendered.item.id)?.newTransform;
  if (dirty) return dirty;
  const object = rendered.object3d;
  return (object?.position && object?.rotation && object?.scale) || object?.t ? transformFromObject(object) : transformOf(rendered.item);
}
function transformFieldCanonicalValue(transform, name) {
  const spec = TRANSFORM_FIELD_SPECS[name];
  const value = transform.pose[spec.group][spec.component];
  return spec.group === 'rpy' ? value * 180 / Math.PI : value;
}
function formatTransformField(transform, name) {
  const spec = TRANSFORM_FIELD_SPECS[name];
  return Number(transformFieldCanonicalValue(transform, name)).toFixed(spec.decimals);
}
function strictFiniteDecimal(text) {
  const value = String(text ?? '').trim();
  if (!/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/.test(value)) return null;
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : null;
}
function commitCanonicalTransformEdit(rendered, nextTransform, reason = 'numeric_inspector') {
  const owner = canonicalTransformOwner(rendered);
  if (!owner || owner !== rendered || !selectionIsEditable(owner) || state.placement.armed) return false;
  const before = cloneTransform(canonicalTransformForRendered(owner));
  const after = cloneTransform(nextTransform);
  if (!isFiniteTransform(after) || sameTransform(before, after)) { syncInspectorTransformFields(owner, { force: true }); return false; }
  const committed = markDirtyTransform(owner, after, { pushHistory: true, oldTransform: before, snapOptions: null, memberStarts: captureTransformGroup(owner) });
  if (!committed) { syncInspectorTransformFields(owner, { force: true }); return false; }
  owner.object3d.updateMatrixWorld?.(true);
  syncInspectorTransformFields(owner, { force: true });
  updateLabels();
  if (state.gizmoPivot?.owner === owner) refreshTransientGizmoPivot(owner, `${reason}_committed`);
  else state.three.transformControls?.object?.updateMatrixWorld?.(true);
  emitTransformCommitted(owner);
  return true;
}
function transformClipboardPayload(rendered) {
  if (!selectionIsEditable(rendered)) return null;
  const transform = canonicalTransformForRendered(rendered);
  return {
    schema_version: TRANSFORM_CLIPBOARD_SCHEMA_VERSION,
    units: { position: 'metres', rotation: 'degrees' },
    source_item_id: String(rendered.item.id || ''),
    source_item_label: itemLabel(rendered.item),
    pose: {
      xyz: [transform.pose.xyz.x, transform.pose.xyz.y, transform.pose.xyz.z],
      rpy_deg: [transform.pose.rpy.x, transform.pose.rpy.y, transform.pose.rpy.z].map(value => value * 180 / Math.PI),
    },
  };
}
function parseTransformClipboardPayload(value) {
  let payload = value;
  if (typeof payload === 'string') {
    try { payload = JSON.parse(payload); } catch (_) { return null; }
  }
  if (!payload || payload.schema_version !== TRANSFORM_CLIPBOARD_SCHEMA_VERSION ||
      payload.units?.position !== 'metres' || payload.units?.rotation !== 'degrees') return null;
  const xyz = payload.pose?.xyz;
  const rpyDeg = payload.pose?.rpy_deg;
  if (!Array.isArray(xyz) || xyz.length !== 3 || !Array.isArray(rpyDeg) || rpyDeg.length !== 3) return null;
  const values = [...xyz, ...rpyDeg].map(Number);
  if (!values.every(Number.isFinite)) return null;
  return {
    schema_version: TRANSFORM_CLIPBOARD_SCHEMA_VERSION,
    units: { position: 'metres', rotation: 'degrees' },
    source_item_id: String(payload.source_item_id || ''),
    source_item_label: String(payload.source_item_label || payload.source_item_id || 'object'),
    pose: { xyz: values.slice(0, 3), rpy_deg: values.slice(3, 6) },
  };
}
function setTransformClipboardStatus(message) {
  state.transformClipboardStatus = String(message || '');
  const status = el.inspector?.querySelector?.('#transform-clipboard-status');
  if (status) { status.textContent = state.transformClipboardStatus; status.hidden = !state.transformClipboardStatus; }
  if (message) pushEditorEvent('status', { message: state.transformClipboardStatus });
}
async function copySelectedTransformToClipboard(rendered = canonicalTransformOwner(state.selected)) {
  const payload = transformClipboardPayload(rendered);
  if (!payload) { setTransformClipboardStatus('Select an editable object to copy its pose.'); return false; }
  state.transformClipboard = payload;
  let systemClipboard = false;
  try {
    if (globalThis.navigator?.clipboard?.writeText) {
      await globalThis.navigator.clipboard.writeText(JSON.stringify(payload, null, 2));
      systemClipboard = true;
    }
  } catch (_) { /* The in-app clipboard remains authoritative for this session. */ }
  setTransformClipboardStatus(`Copied pose from ${payload.source_item_label}${systemClipboard ? '' : ' (in-app clipboard)'}.`);
  return true;
}
function pasteTransformPayload(rendered, value) {
  if (!selectionIsEditable(rendered)) { setTransformClipboardStatus('Select an editable target object before pasting.'); return false; }
  const payload = parseTransformClipboardPayload(value);
  if (!payload) { setTransformClipboardStatus('Clipboard does not contain a valid Workcell Studio pose.'); return false; }
  const before = canonicalTransformForRendered(rendered);
  const next = cloneTransform(before);
  [next.pose.xyz.x, next.pose.xyz.y, next.pose.xyz.z] = payload.pose.xyz;
  [next.pose.rpy.x, next.pose.rpy.y, next.pose.rpy.z] = payload.pose.rpy_deg.map(value => value * Math.PI / 180);
  if (sameTransform(before, next)) {
    setTransformClipboardStatus(`${itemLabel(rendered.item)} already has this pose.`);
    syncInspectorTransformFields(rendered, { force: true });
    return false;
  }
  if (!commitCanonicalTransformEdit(rendered, next, 'paste_transform')) {
    setTransformClipboardStatus(`Could not paste the pose onto ${itemLabel(rendered.item)}.`);
    return false;
  }
  setTransformClipboardStatus(`Pasted pose from ${payload.source_item_label} onto ${itemLabel(rendered.item)}.`);
  return true;
}
async function pasteSelectedTransformFromClipboard(rendered = canonicalTransformOwner(state.selected)) {
  if (!selectionIsEditable(rendered)) { setTransformClipboardStatus('Select an editable target object before pasting.'); return false; }
  let payload = state.transformClipboard;
  try {
    if (globalThis.navigator?.clipboard?.readText) {
      const systemText = await globalThis.navigator.clipboard.readText();
      if (String(systemText || '').trim()) payload = systemText;
    }
  } catch (_) { /* Fall back to the in-app clipboard. */ }
  if (!payload) { setTransformClipboardStatus('Copy a pose first, then select the target object and paste.'); return false; }
  return pasteTransformPayload(rendered, payload);
}
function renderTransformInputs(rendered) {
  const editable = selectionIsEditable(rendered);
  const transform = canonicalTransformForRendered(rendered);
  const disabled = editable ? '' : 'disabled';
  const inputs = names => names.map(name => {
    const spec = TRANSFORM_FIELD_SPECS[name];
    return `<label>${spec.label}<input type="number" inputmode="decimal" step="${spec.step}" data-transform-field="${name}" value="${formatTransformField(transform, name)}" ${disabled}></label>`;
  }).join('');
  return `<section class="transform-editor"><h3>Transform</h3>${editable ? '<p class="edit-note edit-mode-active">Edit mode active for editable/unlocked item. Edit the exact authored pose: position is in metres and rotation is in degrees. Every change uses the same undo, redo, and Save Layout path as the gizmo.</p>' : `<p class="edit-lock-reason">${LOCKED_EDIT_REASON}</p>`}<h4>Position (m)</h4><div class="transform-grid">${inputs(['x', 'y', 'z'])}</div><h4>Rotation (deg)</h4><div class="transform-grid">${inputs(['roll', 'pitch', 'yaw'])}</div><div class="editor-actions transform-clipboard-actions"><button id="copy-transform" type="button" ${editable ? '' : 'disabled'} title="Copy XYZ in metres and RPY in degrees (Ctrl/Cmd+C)">Copy Pose</button><button id="paste-transform" type="button" ${editable ? '' : 'disabled'} title="Paste pose and preserve this object's scale (Ctrl/Cmd+V)">Paste Pose</button><button id="reset-selected" type="button" ${editable ? '' : 'disabled'}>Reset Pose</button></div><p id="transform-clipboard-status" class="transform-clipboard-status" role="status" aria-live="polite" ${state.transformClipboardStatus ? '' : 'hidden'}>${escapeHtml(state.transformClipboardStatus)}</p></section>`;
}
function syncInspectorTransformFields(rendered, { force = false } = {}) {
  if (state.selected !== rendered?.item?.id) return;
  const editor = el.inspector.querySelector('.transform-editor');
  if (!editor) return;
  const transform = canonicalTransformForRendered(rendered);
  for (const name of Object.keys(TRANSFORM_FIELD_SPECS)) {
    const input = editor.querySelector(`[data-transform-field="${name}"]`);
    if (!input || (!force && (input.dataset.transformDirty === 'true' || document.activeElement === input))) continue;
    input.value = formatTransformField(transform, name);
    input.dataset.transformDirty = 'false';
    input.setAttribute?.('aria-invalid', 'false');
  }
}
function commitTransformField(rendered, input) {
  if (!selectionIsEditable(rendered) || input.dataset.transformDirty !== 'true') return false;
  const name = input.dataset.transformField;
  const spec = TRANSFORM_FIELD_SPECS[name];
  const numeric = strictFiniteDecimal(input.value);
  input.dataset.transformDirty = 'false';
  if (!spec || numeric === null) { syncInspectorTransformFields(rendered, { force: true }); return false; }
  const next = cloneTransform(canonicalTransformForRendered(rendered));
  next.pose[spec.group][spec.component] = spec.group === 'rpy' ? numeric * Math.PI / 180 : numeric;
  const committed = commitCanonicalTransformEdit(rendered, next, `numeric_${name}`);
  syncInspectorTransformFields(rendered, { force: true });
  return committed;
}
function cancelTransformFieldEdit(rendered, input) {
  input.dataset.transformDirty = 'false';
  syncInspectorTransformFields(rendered, { force: true });
}
function wireTransformInputs(rendered) {
  const editor = el.inspector.querySelector('.transform-editor');
  if (!editor) return;
  editor.querySelectorAll('[data-transform-field]').forEach(input => {
    input.addEventListener('input', () => { if (selectionIsEditable(rendered)) input.dataset.transformDirty = 'true'; });
    input.addEventListener('keydown', event => {
      if (event.key === 'Enter') { commitTransformField(rendered, input); event.preventDefault(); event.stopPropagation(); }
      else if (event.key === 'Escape') { cancelTransformFieldEdit(rendered, input); event.preventDefault(); event.stopPropagation(); input.blur?.(); }
    });
    input.addEventListener('blur', () => commitTransformField(rendered, input));
  });
  const reset = el.inspector.querySelector('#reset-selected');
  if (reset) reset.addEventListener('click', () => resetSelectedTransform(rendered.item.id));
  const copy = el.inspector.querySelector('#copy-transform');
  if (copy) copy.addEventListener('click', () => copySelectedTransformToClipboard(rendered));
  const paste = el.inspector.querySelector('#paste-transform');
  if (paste) paste.addEventListener('click', () => pasteSelectedTransformFromClipboard(rendered));
}
function resetSelectedTransform(id = state.selected) {
  const rendered = state.objects.find(obj => obj.item.id === id);
  if (!selectionIsEditable(rendered)) return false;
  const baseline = rendered.authoredBaselineTransform || rendered.originalTransform;
  if (!baseline) return false;
  return commitCanonicalTransformEdit(rendered, cloneTransform(baseline), 'reset_pose');
}
function updateDirtyState() {
  const dirty = state.dirtyTransforms.size > 0;
  if (el.dirty) { el.dirty.hidden = !dirty; el.dirty.textContent = dirty ? `Unsaved preview edits (${state.dirtyTransforms.size})` : 'No preview edits'; }
  if (el.undoEdit) el.undoEdit.disabled = state.undoStack.length === 0;
  if (el.redoEdit) el.redoEdit.disabled = state.redoStack.length === 0;
  if (el.clearEdits) el.clearEdits.disabled = !dirty;
  if (el.exportEditPatch) el.exportEditPatch.disabled = !state.sceneJson;
  populateObjectList();
}
function clearPreviewEdits() {
  cancelDirectMoveDrag('Move cancelled');
  for (const rendered of state.objects) applyTransformToObject(rendered.object3d, rendered.originalTransform);
  state.dirtyTransforms.clear();
  state.undoStack = [];
  state.redoStack = [];
  updateDirtyState();
  if (state.selected) { const rendered = state.objects.find(obj => obj.item.id === state.selected); if (rendered) populateInspector(rendered); }
  updateLabels();
}
function applyHistoryEntry(entry, direction) {
  const records = entry.changes || [entry];
  const changes = records.map(record => ({ rendered: renderedById(record.itemId), after: cloneTransform(direction === 'undo' ? record.before : record.after) }))
    .filter(change => change.rendered && canEditItem(change.rendered.item));
  applyTransformChanges(changes, { updateDirty: true });
  const selected = renderedById(state.selected);
  if (selected) populateInspector(selected);
}
function undoPreviewEdit() {
  cancelDirectMoveDrag('Move cancelled');
  const entry = state.undoStack.pop();
  if (!entry) return;
  applyHistoryEntry(entry, 'undo');
  state.redoStack.push(entry);
  updateDirtyState();
  emitDirtyChanged();
}
function redoPreviewEdit() {
  cancelDirectMoveDrag('Move cancelled');
  const entry = state.redoStack.pop();
  if (!entry) return;
  applyHistoryEntry(entry, 'redo');
  state.undoStack.push(entry);
  updateDirtyState();
  emitDirtyChanged();
}
function sceneId() { return state.sceneJson?.scene?.id || state.sceneJson?.scene_id || state.sourceWebSceneFile || ''; }
function buildEditPatch() {
  const edits = [];
  for (const rendered of state.objects) {
    const dirty = state.dirtyTransforms.get(rendered.item.id);
    if (!dirty) continue;
    edits.push({
      item_id: rendered.item.id,
      label: itemLabel(rendered.item),
      source: rendered.item.source_kind || rendered.item.source || rendered.item.source_layer || '',
      type: itemType(rendered.item),
      editable_required: true,
      locked_required: false,
      operation: 'update_transform',
      persistence_source: rendered.item.provenance?.pose || '',
      old_transform: dirty.oldTransform,
      new_transform: dirty.newTransform,
      notes: ['Preview-only browser transform edit. Source scene files were not modified.'],
    });
  }
  return { schema_version: EDIT_PATCH_SCHEMA_VERSION, scene_id: sceneId(), source_scene_schema_version: state.sceneJson?.schema_version || '', created_at: new Date().toISOString(), created_by: 'static_web_viewer', provenance: { source_web_scene_file: state.sourceWebSceneFile || undefined, viewer_version: VIEWER_VERSION }, edits };
}
function exportEditPatch() {
  if (!state.sceneJson) { showError('Load a web_scene.json before exporting an edit patch.'); return; }
  if (state.dirtyTransforms.size === 0) showError('No changed items; exporting an empty edit patch.'); else clearError();
  const blob = new Blob([JSON.stringify(buildEditPatch(), null, 2) + '\n'], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `${sceneId() || 'workcell_studio'}.edit_patch.json`;
  document.body.appendChild(a); a.click(); a.remove(); URL.revokeObjectURL(a.href);
}

function firstPresent(item, keys) {
  for (const key of keys) {
    if (item?.[key] !== undefined && item?.[key] !== null && item?.[key] !== '') return item[key];
  }
  return undefined;
}
function populateInspector(renderedOrItem) {
  const rendered = renderedOrItem?.item ? renderedOrItem : state.objects.find(obj => obj.item.id === renderedOrItem?.id);
  const item = rendered?.item || renderedOrItem;
  const renderInfo = rendered?.renderInfo || item.renderInfo || {};
  const pose = poseOf(item);
  const rows = {
    id: item.id, label: itemLabel(item), type: itemType(item), details: supportSurfaceDisplayType(item), source: item.source_kind || item.source || valueOrDash(item.provenance && Object.values(item.provenance)[0]),
    'pose xyz': [pose.xyz.x, pose.xyz.y, pose.xyz.z].map(n => n.toFixed(3)).join(', '),
    'pose rpy': [pose.rpy.x, pose.rpy.y, pose.rpy.z].map(n => n.toFixed(3)).join(', '),
    scale: JSON.stringify(item.scale || item.mesh_scale || [1, 1, 1]), editable: String(Boolean(item.editable)), locked: String(Boolean(item.locked)),
    render_status: renderInfo.render_status, mesh_uri: renderInfo.mesh_uri || displayMeshUri(item), fallback_reason: renderInfo.fallback_reason,
    mesh_status: item.mesh_status, mesh_load_required: item.mesh_load_required, mesh_load_status: item.mesh_load_status, mesh_load_url: item.mesh_load_url, mesh_load_error: item.mesh_load_error,
    original_mesh_uri: item.original_mesh_uri, mesh_staging_status: item.mesh_staging_status,
    mesh_staged_path: item.mesh_staged_path, mesh_resolve_warning: item.mesh_resolve_warning,
    support_surface_kind: item.support_surface_kind, supportSurfaceKind: item.supportSurfaceKind,
    top_surface_z_m: item.top_surface_z_m, topSurfaceZM: item.topSurfaceZM,
    support_surface_height_m: item.support_surface_height_m, supportSurfaceHeightM: item.supportSurfaceHeightM,
    expected_support_footprint_m: JSON.stringify(item.expected_support_footprint_m || ''), expectedSupportFootprintM: JSON.stringify(item.expectedSupportFootprintM || ''),
    primitive: JSON.stringify(primitiveOf(item) || 'box fallback'),
  };
  if (isSensor(item)) {
    Object.assign(rows, {
      camera_id: firstPresent(item, ['camera_id', 'camera', 'sensor_id']),
      frame_id: firstPresent(item, ['frame_id', 'camera_frame', 'frame']),
      model: firstPresent(item, ['model', 'profile', 'camera_model', 'sensor_model']),
      optical_frame_id: firstPresent(item, ['optical_frame_id', 'optical_frame', 'color_optical_frame', 'rgb_optical_frame']),
      depth_frame_id: firstPresent(item, ['depth_frame_id', 'depth_frame']),
      rgb_topic: firstPresent(item, ['rgb_topic', 'color_topic', 'image_topic']),
      depth_topic: firstPresent(item, ['depth_topic']),
      camera_info_topic: firstPresent(item, ['camera_info_topic', 'info_topic']),
      pointcloud_topic: firstPresent(item, ['pointcloud_topic', 'points_topic']),
    });
  }
  el.inspector.className = '';
  el.inspector.innerHTML = `<table class="inspector-table"><tbody>${Object.entries(rows).map(([k,v]) => `<tr><th>${k}</th><td><code>${String(valueOrDash(v)).replace(/[&<>]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}</code></td></tr>`).join('')}</tbody></table>${rendered ? renderTransformInputs(rendered) : ''}`;
  if (rendered) wireTransformInputs(rendered);
}
function refreshWarnings(sceneJson = state.sceneJson) {
  const warnings = asArray(sceneJson?.warnings).concat(asArray(sceneJson?.notes_warnings), asArray(state.runtimeWarnings));
  const userFacingWarnings = warnings.filter(isUserFacingWarning);
  updateViewerStatus();
  if (el.warningsDetails) el.warningsDetails.hidden = userFacingWarnings.length === 0;
  if (el.warningCount) el.warningCount.textContent = String(userFacingWarnings.length);
  if (!userFacingWarnings.length) { el.warnings.className = 'warnings state empty'; el.warnings.textContent = 'No user-facing JSON or runtime mesh warnings.'; return; }
  el.warnings.className = 'warnings';
  el.warnings.innerHTML = userFacingWarnings.map(w => {
    const label = w.code || w.source || 'warning';
    const details = w.message || w.reason || JSON.stringify(w);
    const objectDetails = w.object_id || w.mesh_uri ? `<br><code>object=${escapeHtml(w.object_id)} link=${escapeHtml(w.link || w.object_name || '')} original=${escapeHtml(w.original_mesh_uri || '')} mesh=${escapeHtml(w.mesh_uri)}</code>` : '';
    return `<div class="warning-item"><strong>${escapeHtml(label)}</strong><br>${escapeHtml(details)}${objectDetails}</div>`;
  }).join('');
}
function populateWarnings(sceneJson) { refreshWarnings(sceneJson); }
async function loadFile(file) {
  clearError();
  cancelPlacement();
  try {
    const text = await file.text();
    let json;
    try { json = JSON.parse(text); } catch (err) { throw new Error(`Invalid JSON in ${file.name}: ${err.message}`); }
    state.sourceWebSceneFile = file.name || '';
    state.builderRevision = '';
    state.sceneJsonLoaded = false;
    emitWeb3dReadinessState('scene_loading');
    const items = validateSceneJson(json);
    state.sceneJson = json;
    state.sceneJsonLoaded = true;
    emitWeb3dReadinessState('scene_loading');
    beginInitialCameraFitForCurrentScene();
    state.runtimeWarnings = [];
    state.dirtyTransforms.clear();
    state.undoStack = [];
    state.redoStack = [];
    state.selected = null;
    state.selectedRenderIdentityId = '';
    cancelDirectMoveDrag('Move cancelled');
    cancelDirectRotateDrag('Rotation cancelled');
    detachTransformGizmo();
    el.empty.hidden = true;
    if (items.length) renderScene(items);
    else renderScene([]);
    syncOrbitControlsForEditorMode();
    refreshWarnings(json);
    renderSceneSummary();
    el.inspector.className = 'state empty';
    el.inspector.textContent = items.length ? 'Select an object from the list or canvas.' : EMPTY_SCENE_MESSAGE;
  } catch (err) {
    showError(err.message || String(err));
  }
}

function safeRelativeSceneUrl(raw) {
  if (typeof raw !== 'string' || !raw.trim()) throw new Error('Empty scene URL parameter.');
  const uri = raw.trim();
  const lower = uri.toLowerCase();
  const stagedHttpUrl = (() => {
    if (!lower.startsWith('http://') && !lower.startsWith('https://')) return null;
    try {
      const parsed = new URL(uri);
      const currentOrigin = window.location?.origin || '';
      const stagedPath = parsed.pathname.replace(/^\/+/, '');
      if (currentOrigin && currentOrigin !== 'null' && parsed.origin === currentOrigin && STAGED_MESH_ROOTS.some(root => stagedPath.startsWith(root))) return stagedPath + parsed.search + parsed.hash;
    } catch (_) { /* fall through to unsafe_path below */ }
    return null;
  })();
  if (stagedHttpUrl) return stagedHttpUrl;
  if (
    lower.startsWith('http://') ||
    lower.startsWith('https://') ||
    lower.startsWith('file://') ||
    lower.startsWith('data:') ||
    lower.startsWith('//') ||
    uri.startsWith('/') ||
    uri.startsWith('\\') ||
    /^[a-zA-Z]:[\\/]/.test(uri) ||
    /^[a-zA-Z][a-zA-Z\d+.-]*:/.test(uri) ||
    uri.includes('\\')
  ) throw new Error(`unsafe scene URL rejected by viewer policy: ${uri}`);
  const pathOnly = uri.split(/[?#]/, 1)[0];
  if (pathOnly.split('/').some(part => part === '..')) {
    throw new Error(`scene URL path traversal rejected: ${uri}`);
  }
  if (!pathOnly.startsWith('build/workcell_studio_web_scene/') || !pathOnly.endsWith('.web_scene.json')) {
    throw new Error(`scene URL must point under build/workcell_studio_web_scene/*.web_scene.json: ${uri}`);
  }
  return uri;
}

async function loadSceneUrl(rawUrl) {
  const sceneUrl = safeRelativeSceneUrl(rawUrl);
  cancelPlacement();
  try {
    state.sourceWebSceneFile = sceneUrl;
    state.sceneJsonLoaded = false;
    emitWeb3dReadinessState('scene_loading', { source_web_scene_file: sceneUrl });
    const response = await fetch(repoRootRelativeUrl(sceneUrl), { cache: 'no-store' });
    if (!response.ok) throw new Error(`HTTP ${response.status} ${response.statusText}`);
    const json = await response.json();
    const items = validateSceneJson(json);
    state.sceneJson = json;
    state.sceneJsonLoaded = true;
    emitWeb3dReadinessState('scene_loading');
    state.frameLookup = parseSceneFrames(json);
    state.resolvedFramePoses.clear();
    beginInitialCameraFitForCurrentScene();
    state.runtimeWarnings = [];
    state.dirtyTransforms.clear();
    state.undoStack = [];
    state.redoStack = [];
    state.selected = null;
    state.selectedRenderIdentityId = '';
    cancelDirectMoveDrag('Move cancelled');
    cancelDirectRotateDrag('Rotation cancelled');
    detachTransformGizmo();
    el.empty.hidden = true;
    renderScene(items);
    syncOrbitControlsForEditorMode();
    refreshWarnings(json);
    renderSceneSummary();
    el.inspector.className = 'state empty';
    el.inspector.textContent = items.length ? 'Select an object from the list or canvas.' : EMPTY_SCENE_MESSAGE;
  } catch (err) {
    showError(`Failed to load scene from ${sceneUrl}: ${err.message || err}`);
  }
}

if (el.resetView) el.resetView.addEventListener('click', resetView);
if (el.cameraPreset) el.cameraPreset.addEventListener('change', event => { applyCameraPreset(event.target.value); event.target.value = ''; });
if (el.labelsToggle) el.labelsToggle.addEventListener('change', event => setLabelsVisible(event.target.checked));
if (el.debugOverlaysToggle) el.debugOverlaysToggle.addEventListener('change', event => setDebugOverlaysVisible(event.target.checked));
if (el.showInitialPose) el.showInitialPose.addEventListener('change', event => toggleInitialPosePreview(event.target.checked));
if (el.undoEdit) el.undoEdit.addEventListener('click', undoPreviewEdit);
if (el.redoEdit) el.redoEdit.addEventListener('click', redoPreviewEdit);
if (el.clearEdits) el.clearEdits.addEventListener('click', clearPreviewEdits);
function refreshPlacementSnap() {
  if (state.placement.armed) setPlacementPoint(state.placement.rawPoint);
}
if (el.snapToggle) el.snapToggle.addEventListener('change', () => { refreshGizmoSnap(); refreshPlacementSnap(); });
if (el.translationSnap) el.translationSnap.addEventListener('input', () => { refreshGizmoSnap(); refreshPlacementSnap(); });
if (el.rotationSnap) el.rotationSnap.addEventListener('input', refreshGizmoSnap);
if (el.exportEditPatch) el.exportEditPatch.addEventListener('click', exportEditPatch);

function normalizeTransformSpace(space) { return String(space || '').toLowerCase() === 'local' ? 'local' : 'world'; }
function setTransformSpace(space) {
  const normalized = normalizeTransformSpace(space);
  if (state.transformSpace !== normalized) cancelActiveTransformOperation('Transform space changed');
  state.transformSpace = normalized;
  if (el.transformSpace) el.transformSpace.value = normalized;
  state.three.transformControls?.setSpace(normalized);
  if (state.gizmoPivot?.owner) refreshTransientGizmoPivot(state.gizmoPivot.owner, 'transform_space_change');
  return state.transformSpace;
}
if (el.transformSpace) el.transformSpace.addEventListener('change', event => setTransformSpace(event.target.value));

function setEditorMode(mode) {
  const normalized = mode === 'move' ? 'move' : (mode === 'rotate' ? 'rotate' : 'select');
  // Mode is the single authoring state.  Always clean up transient drag state,
  // even when a repeated Select command arrives while a pointer callback is
  // still in flight.
  if (state.editorMode !== normalized || normalized === 'select') cancelActiveTransformOperation('Mode changed');
  state.editorMode = normalized;
  syncOrbitControlsForEditorMode();
  const gizmo = state.three.transformControls;
  if (gizmo) {
    if (normalized === 'move') { gizmo.setMode('translate'); gizmo.setSpace(state.transformSpace); gizmo.showX = true; gizmo.showY = true; gizmo.showZ = true; gizmo.enabled = true; }
    else if (normalized === 'rotate') { gizmo.setMode('rotate'); gizmo.setSpace(state.transformSpace); gizmo.showX = true; gizmo.showY = true; gizmo.showZ = true; gizmo.enabled = true; }
    else {
      gizmo.reset?.();
      gizmo.detach();
      gizmo.visible = false;
      gizmo.enabled = false;
      state.gizmoDragStart = null;
      state.gizmoDragGroupStart = null;
    }
  }
  if (normalized !== 'select') attachTransformGizmo(canonicalTransformOwner(state.selected), 'mode_change');
  return state.editorMode;
}
function setEditorSnap(enabled, translationMeters, rotationDegrees) {
  if (el.snapToggle) el.snapToggle.checked = Boolean(enabled);
  if (el.translationSnap && Number.isFinite(Number(translationMeters))) el.translationSnap.value = String(Number(translationMeters));
  if (el.rotationSnap && Number.isFinite(Number(rotationDegrees))) el.rotationSnap.value = String(Number(rotationDegrees));
  refreshGizmoSnap();
  refreshPlacementSnap();
}
function setItemPoseFromBridge(id, x, y, z, roll, pitch, yaw, pushHistory = true) {
  const requested = renderedById(String(id || ''));
  const rendered = requested
    ? (canonicalEditOwnerRendered(requested) || requested)
    : null;

  if (!rendered || !selectionIsEditable(rendered)) return editorState();

  const values = [x, y, z, roll, pitch, yaw].map(Number);
  if (!values.every(Number.isFinite)) {
    showError(`Invalid transform received for ${String(id || '')}`);
    return editorState();
  }

  const before = cloneTransform(
    state.dirtyTransforms.get(rendered.item.id)?.newTransform ||
    transformFromObject(rendered.object3d)
  );
  const next = cloneTransform(before);

  next.pose.xyz.x = values[0];
  next.pose.xyz.y = values[1];
  next.pose.xyz.z = values[2];
  next.pose.rpy.x = values[3];
  next.pose.rpy.y = values[4];
  next.pose.rpy.z = values[5];

  const committed = markDirtyTransform(rendered, next, {
    pushHistory: Boolean(pushHistory),
    oldTransform: before,
    snapOptions: { translationAxes: [], rotationAxes: [] },
  });

  if (!committed) return editorState();

  if (state.selected === rendered.item.id) {
    populateInspector(rendered);
  }
  emitTransformCommitted(rendered);
  updateLabels();
  return editorState();
}

function setItemMetadataFromBridge(id, displayName, semanticRole) {
  const requested = renderedById(String(id || ''));
  const rendered = requested
    ? (canonicalEditOwnerRendered(requested) || requested)
    : null;
  if (!rendered || !selectionIsEditable(rendered)) return editorState();
  const name = String(displayName || '').trim();
  const role = String(semanticRole || '').trim();
  if (name) rendered.item.display_name = name;
  if (role) rendered.item.role = role;
  if (rendered.labelEl) rendered.labelEl.textContent = itemLabel(rendered.item);
  populateObjectList();
  updateLabels();
  if (state.selected === rendered.item.id) populateInspector(rendered);
  if (typeof pushEditorEvent === 'function') {
    pushEditorEvent('metadata_preview_updated', {
      itemId: rendered.item.id,
      displayName: rendered.item.display_name || '',
      semanticRole: rendered.item.role || ''
    });
  }
  return editorState();
}

function normalizedLiveAuthoringItem(value) {
  if (!value || typeof value !== 'object') return null;
  const item = { ...value };
  item.id = String(item.id || '').trim();
  if (!item.id || renderedById(item.id)) return null;
  item.display_name = String(item.display_name || item.id).trim() || item.id;
  item.role = String(item.role || 'object').trim() || 'object';
  item.category = String(item.category || item.type || 'object').trim() || 'object';
  item.type = String(item.type || item.category).trim() || 'object';
  item.source_layer = 'editable_layout';
  item.editable = true;
  item.locked = false;
  item.selectable = true;
  item.render_owner = 'editable_layout';
  item.render_policy = 'primary';
  return item;
}

function finishLiveAuthoringCrud(itemId, eventType) {
  state.selectionIdentityIndex = null;
  bindExportedPhysicalTransformOwnership();
  populateObjectList();
  updateLabels();
  renderSceneSummary();
  if (typeof pushEditorEvent === 'function') {
    pushEditorEvent(eventType, { itemId, liveSession: true });
  }
  return editorState();
}

function addItemFromBridge(value) {
  const item = normalizedLiveAuthoringItem(value);
  if (!item || !state.three?.scene) return editorState();
  const object3d = new THREE.Group();
  object3d.name = `${item.id}_object_root`;
  object3d.up.copy(THREE.Object3D.DEFAULT_UP);
  const fallback = isSensor(item) ? makeSensorMarker(item) : makePrimitiveMesh(item);
  if (fallback) {
    fallback.name = `${item.id}_fallback`;
    assignItemUserData(fallback, item);
    object3d.add(fallback);
  }
  if (!applyPose(object3d, item)) return editorState();
  assignItemUserData(object3d, item);
  state.three.scene.add(object3d);
  const rendered = {
    item,
    object3d,
    fallback,
    labelEl: createLabelElement(item),
    originalTransform: transformOf(item),
    authoredBaselineTransform: cloneTransform(transformOf(item)),
  };
  setRenderInfo(rendered, fallback ? 'primitive_fallback' : 'no_physical_dimensions',
    displayMeshUri(item), 'live authored-session item');
  state.objects.push(rendered);
  registerCanonicalPhysicalPick(rendered, 'live_item_added');
  tryLoadMesh(item, rendered, fallback);
  selectObject(item.id);
  return finishLiveAuthoringCrud(item.id, 'item_added');
}

function duplicateItemFromBridge(sourceId, value) {
  const source = renderedById(String(sourceId || ''));
  const item = normalizedLiveAuthoringItem(value);
  if (!source || !item || !state.three?.scene || !selectionIsEditable(source)) {
    return editorState();
  }
  const object3d = source.object3d.clone(true);
  object3d.name = `${item.id}_object_root`;
  if (!applyPose(object3d, item)) return editorState();
  object3d.traverse?.(child => assignItemUserData(child, item));
  state.three.scene.add(object3d);
  const rendered = {
    item,
    object3d,
    fallback: null,
    labelEl: createLabelElement(item),
    originalTransform: transformOf(item),
    authoredBaselineTransform: cloneTransform(transformOf(item)),
    renderInfo: { ...(source.renderInfo || {}), render_status: 'live_duplicate' },
  };
  state.objects.push(rendered);
  registerCanonicalPhysicalPick(rendered, 'live_item_duplicated');
  selectObject(item.id);
  return finishLiveAuthoringCrud(item.id, 'item_duplicated');
}

function removeItemFromBridge(id) {
  const stableId = String(id || '').trim();
  const rendered = renderedById(stableId);
  if (!rendered || !selectionIsEditable(rendered)) return editorState();
  if (state.selected === stableId) clearSelection();
  detachTransformGizmo();
  state.three.scene?.remove(rendered.object3d);
  rendered.labelEl?.remove?.();
  state.objects = state.objects.filter(record => record !== rendered);
  state.pickRecords = state.pickRecords.filter(record => record !== rendered && record.item?.id !== stableId);
  state.dirtyTransforms.delete(stableId);
  state.undoStack = state.undoStack.filter(entry => !(entry.changes || [entry]).some(change => change.itemId === stableId));
  state.redoStack = state.redoStack.filter(entry => !(entry.changes || [entry]).some(change => change.itemId === stableId));
  updateDirtyState();
  return finishLiveAuthoringCrud(stableId, 'item_removed');
}

function setVisibleItemIdsFromBridge(ids) {
  const visible = new Set(Array.isArray(ids) ? ids.map(id => String(id || '').trim()).filter(Boolean) : []);
  for (const rendered of [...state.objects, ...state.pickRecords]) {
    const id = String(rendered?.item?.id || '').trim();
    if (id && rendered?.object3d) rendered.object3d.visible = visible.has(id);
    if (rendered?.labelEl) rendered.labelEl.hidden = !visible.has(id);
  }
  updateLabels();
  renderSceneSummary();
  return editorState();
}

window.__WORKCELL_EDITOR_API_V1__ = {
  apiVersion: '1.1.0',
  getCapabilities: () => Object.freeze({
    schemaVersion: 'workcell_studio_live_authoring_capabilities/v1',
    apiVersion: '1.1.0',
    operations: Object.freeze([
      'getState', 'selectItem', 'setItemMetadata', 'setItemTransform',
      'addItem', 'duplicateItem', 'removeItem', 'setVisibleItemIds', 'undo', 'redo',
      'getEditPatch', 'drainEvents',
    ]),
  }),
  getState: () => {
    const base = editorState();
    const selectedRendered = state.selected ? renderedById(state.selected) : null;
    const transformOwner = selectedRendered
      ? (canonicalTransformOwner(selectedRendered) || selectedRendered)
      : null;
    const selectedTransform = transformOwner
      ? cloneTransform(
          state.dirtyTransforms.get(transformOwner.item.id)?.newTransform ||
          transformFromObject(transformOwner.object3d)
        )
      : null;
    return { ...base, selectedTransform };
  },
  selectItem: id => { selectObject(String(id || '')); return editorState(); },
  setItemPose: (id, x, y, z, roll, pitch, yaw) =>
    setItemPoseFromBridge(id, x, y, z, roll, pitch, yaw),
  setItemTransform: (id, x, y, z, roll, pitch, yaw) =>
    setItemPoseFromBridge(id, x, y, z, roll, pitch, yaw),
  syncItemTransform: (id, x, y, z, roll, pitch, yaw) =>
    setItemPoseFromBridge(id, x, y, z, roll, pitch, yaw, false),
  setItemMetadata: (id, displayName, semanticRole) =>
    setItemMetadataFromBridge(id, displayName, semanticRole),
  addItem: item => addItemFromBridge(item),
  removeItem: id => removeItemFromBridge(id),
  duplicateItem: (id, item) => duplicateItemFromBridge(id, item),
  setVisibleItemIds: ids => setVisibleItemIdsFromBridge(ids),
  selectionDiagnostics: () => currentSelectionDiagnostics(),
  clearSelection: () => { clearSelection(); return editorState(); },
  setMode: mode => { setEditorMode(mode); return editorState(); },
  setTransformSpace: space => { setTransformSpace(space); return editorState(); },
  setSnap: (enabled, translationMeters, rotationDegrees) => { setEditorSnap(enabled, translationMeters, rotationDegrees); return editorState(); },
  undo: () => { undoPreviewEdit(); return editorState(); },
  redo: () => { redoPreviewEdit(); return editorState(); },
  fitScene: () => { resetView(); return editorState(); },
  applyCameraPreset: preset => { applyCameraPreset(preset); return editorState(); },
  fitSelection: () => { fitSelection(); return editorState(); },
  placementPointFromViewport: position => placementPointFromViewport(position),
  updatePlacementPointer: (clientX, clientY) => updatePlacementPointer(clientX, clientY),
  commitPlacementPointer: (clientX, clientY) => commitPlacementPointer(clientX, clientY),
  armPlacement: options => armPlacement(options),
  cancelPlacement: () => cancelPlacement(),
  getPlacementState: () => getPlacementState(),
  getEditPatch: () => buildEditPatch(),
  drainEvents: () => { const events = state.editorEvents.slice(); state.editorEvents.length = 0; return events; },
};

const editorApiInstalledAt = new Date().toISOString();
const editorRuntimeViewerBuild = new URLSearchParams(window.location.search).get('viewerBuild') || '';
Object.defineProperty(window, '__WORKCELL_EDITOR_RUNTIME_V1__', {
  configurable: false,
  enumerable: false,
  writable: false,
  value: Object.freeze({
    schemaVersion: 'workcell_studio_editor_runtime/v1',
    buildId: 'workcell-editor-api-1.1.0-20260820',
    viewerBuild: editorRuntimeViewerBuild,
    apiVersion: window.__WORKCELL_EDITOR_API_V1__.apiVersion,
    installedAt: editorApiInstalledAt,
    locationHref: String(window.location.href || ''),
  }),
});
if (typeof window.CustomEvent === 'function' && typeof window.dispatchEvent === 'function') {
  window.dispatchEvent(new window.CustomEvent('workcell:editor-api-ready', {
    detail: window.__WORKCELL_EDITOR_RUNTIME_V1__,
  }));
}

el.file.addEventListener('change', event => {
  const file = event.target.files?.[0];
  if (file) loadFile(file);
});

async function boot() {
  try {
    const threeModule = await import('three');
    const controlsModule = await import('three/addons/controls/OrbitControls.js');
    const transformControlsModule = await import('three/addons/controls/TransformControls.js');
    const stlModule = await import('three/addons/loaders/STLLoader.js');
    const colladaModule = await import('three/addons/loaders/ColladaLoader.js');
    const objModule = await import('three/addons/loaders/OBJLoader.js');
    const urdfRobotRendererModule = await import('./urdf_robot_renderer.js');
    THREE = threeModule;
    OrbitControls = controlsModule.OrbitControls;
    TransformControls = transformControlsModule.TransformControls;
    STLLoader = stlModule.STLLoader;
    ColladaLoader = colladaModule.ColladaLoader;
    OBJLoader = objModule.OBJLoader;
    loadRobotPreview = urdfRobotRendererModule.loadRobotPreview;
    applyRobotJointPreview = urdfRobotRendererModule.applyRobotJointPreview;
    initThree();
    setLabelsVisible(el.labelsToggle?.checked || false);
    setDebugOverlaysVisible(el.debugOverlaysToggle?.checked || false);
    const params = new URLSearchParams(window.location.search);
    if (params.get('embedded') === '1') document.body.classList.add('embedded-mode');
    state.builderRevision = params.get('builderRevision') || '';
    const sceneParam = params.get('scene');
    if (sceneParam) {
      state.sourceWebSceneFile = safeRelativeSceneUrl(sceneParam);
      state.sceneJsonLoaded = false;
      emitWeb3dReadinessState('scene_loading');
      await loadSceneUrl(state.sourceWebSceneFile);
    } else {
      updateViewerStatus();
    }
  } catch (err) {
    showError(`Bundled Three.js module load failure: ${err.message || err}`);
  }
}

boot();
