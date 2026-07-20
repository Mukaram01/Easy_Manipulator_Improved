let THREE;
let OrbitControls;
let STLLoader;
let ColladaLoader;
let OBJLoader;
let TransformControls;
let loadRobotPreview;
let applyRobotJointPreview;

const PRODUCT_VIEW_LIGHT_PALETTE = Object.freeze({
  workspaceBackground: 0xe9edf1,
  rendererClearColor: 0xe9edf1,
  gridMajor: 0x8996a3,
  gridMinor: 0xc3cbd3,
  labelText: 0x123040,
  labelSurface: 0xf8fafc,
  overlaySurface: 0xfff6dd,
  errorSurface: 0xffeef0,
  errorAccent: 0xc43434,
});

const SUPPORTED_SCHEMA_VERSION = 'workcell_studio_web_scene/v1';
const EDIT_PATCH_SCHEMA_VERSION = 'workcell_studio_web_scene_edit_patch/v1';
const VIEWER_VERSION = 'static_web_viewer_edit_patch_v1';
const LOCKED_EDIT_REASON = 'Locked/generated preview item; edit source layout/environment instead.';
const MIN_FRAME_RADIUS = 1.2;
const EMPTY_SCENE_MESSAGE = 'Scene contains no renderable robots, tools, assets, sensors, zones, items, or objects.';
const FRAME_DISTANCE_MULTIPLIER = 2.7;
const state = { sceneJson: null, sourceWebSceneFile: '', frameLookup: new Map(), resolvedFramePoses: new Map(), objects: [], assemblyRoots: [], robotAssemblyDiagnostics: [], robotAssemblyRenderDiagnostics: {}, robotUrdfPreviewDiagnostics: {}, physicalAssemblyBounds: null, finalPhysicalFitBounds: null, selected: null, three: {}, animationId: null, lastFrameBounds: null, initialCameraFit: { sceneKey: '', done: false, attempts: 0, pendingRetry: null, userControlled: false }, runtimeWarnings: [], labelsVisible: false, debugOverlaysVisible: false, dirtyTransforms: new Map(), undoStack: [], redoStack: [], gizmoDragStart: null, directMoveDrag: null, directRotateDrag: null, editorMode: 'select', editorEvents: [], editorError: '', robotPreviewResult: null, initialPosePreview: { active: false, robotId: '', sceneKey: '' }, web3dReadiness: { state: 'server_ready', emittedSceneReady: false, required: {}, pending: new Set(), failed: false, failure: null } };
const RESET_VIEW_TITLE = 'Fit Scene / Reset View: recomputes and reapplies the fitted workcell overview from renderable bounds.';
const STAGED_MESH_ROOTS = [
  'build/workcell_studio_web_scene/assets/',
  'workcell_studio_web/',
  'assets/',
];
const CANONICAL_UR5_2F_REQUIRED_UR5_VISUALS = Object.freeze([
  'base_link_inertia',
  'shoulder_link',
  'upper_arm_link',
  'forearm_link',
  'wrist_1_link',
  'wrist_2_link',
  'wrist_3_link',
]);
const CANONICAL_UR5_2F_REQUIRED_ROBOTIQ_VISUALS = Object.freeze([
  'gripper_base_link',
  'left_outer_knuckle',
  'right_outer_knuckle',
  'left_outer_finger',
  'right_outer_finger',
  'left_inner_knuckle',
  'right_inner_knuckle',
  'left_inner_finger',
  'right_inner_finger',
]);
const CANONICAL_UR5_2F_REQUIRED_GENERATED_VISUAL_SET = Object.freeze({
  scene_id: 'ur5_2f_test',
  sceneId: 'ur5_2f_test',
  ur5_visuals: CANONICAL_UR5_2F_REQUIRED_UR5_VISUALS,
  ur5Visuals: CANONICAL_UR5_2F_REQUIRED_UR5_VISUALS,
  robotiq_visuals: CANONICAL_UR5_2F_REQUIRED_ROBOTIQ_VISUALS,
  robotiqVisuals: CANONICAL_UR5_2F_REQUIRED_ROBOTIQ_VISUALS,
  physical_scene_visuals: Object.freeze(['workbench_support_surface', 'configured_camera']),
  physicalSceneVisuals: Object.freeze(['workbench_support_surface', 'configured_camera']),
});
const EXPECTED_GENERATED_URDF_DIAGNOSTIC_LINKS = [
  ...CANONICAL_UR5_2F_REQUIRED_UR5_VISUALS,
  'tool0',
  ...CANONICAL_UR5_2F_REQUIRED_ROBOTIQ_VISUALS,
];

const WEB3D_REQUIRED_CATEGORIES = ['robot_arm', 'attached_tool_gripper', 'workbench_support_surface', 'configured_camera'];
function emitWeb3dReadinessState(readinessState, detail = {}) {
  state.web3dReadiness = state.web3dReadiness || { state: 'server_ready', emittedSceneReady: false, required: {}, pending: new Set(), failed: false, failure: null };
  if (readinessState === 'scene_ready') {
    if (state.web3dReadiness.emittedSceneReady) return window.__WORKCELL_VIEWER_STATUS__;
    state.web3dReadiness.emittedSceneReady = true;
  }
  state.web3dReadiness.state = readinessState;
  const structured = structuredWeb3dReadinessFields(readinessState);
  const eventDetail = { ...structured, ...detail, final_lifecycle_state: readinessState, finalLifecycleState: readinessState };
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
function readinessCategoryForItem(item) {
  if (!item || isDebugOverlayItem(item)) return '';
  const category = meshContractCategoryOf(item);
  const identity = viewerGroupIdentity(item);
  if (category === 'camera' || isSensor(item)) return 'configured_camera';
  if (category === 'table' || supportSurfaceDisplayType(item) || /\b(workbench|support surface|tabletop|table)\b/.test(identity)) return 'workbench_support_surface';
  if (isGeneratedToolOrGripperItem(item) || category === 'tool') return 'attached_tool_gripper';
  if (isGeneratedRobotItem(item) || category === 'robot') return 'robot_arm';
  return '';
}
function readinessKey(category, item) { return `${category}:${item?.id || item?.link || itemLabel(item || {})}`; }

function physicalReadinessItems() {
  return collectItems(state.sceneJson || {}).filter(item => !isDebugOverlayItem(item) && readinessCategoryForItem(item));
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
  const pending = Array.from(readiness.pending || []).some(key => String(key).startsWith(`${category}:`));
  if (readiness.state === 'scene_failed' && (readiness.failure?.required_category === category || pending)) return 'failed';
  if (pending) return 'pending';
  return readiness.state === 'scene_ready' ? 'ready' : 'loading';
}
function structuredWeb3dReadinessFields(lifecycleState) {
  const finalState = lifecycleState || state.web3dReadiness?.state || 'server_ready';
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
    final_lifecycle_state: finalState,
    finalLifecycleState: finalState,
  };
}
function beginWeb3dSceneReadiness(items) {
  const required = Object.fromEntries(WEB3D_REQUIRED_CATEGORIES.map(category => [category, false]));
  const pending = new Set();
  for (const item of items || []) {
    const category = readinessCategoryForItem(item);
    if (!category) continue;
    required[category] = true;
    if (itemRequiresMeshBackedVisual(item) || displayMeshUri(item)) pending.add(readinessKey(category, item));
  }
  if (isExpandedUrdfRobotPreview(state.sceneJson?.robot_preview)) {
    required.robot_arm = true;
    required.attached_tool_gripper = true;
    pending.add('robot_arm:expanded_urdf_loader');
    pending.add('attached_tool_gripper:expanded_urdf_loader');
  }
  state.web3dReadiness = { state: 'scene_loading', emittedSceneReady: false, required, pending, failed: false, failure: null };
  emitWeb3dReadinessState('scene_loading', { required_categories: required, pending_required_loads: Array.from(pending) });
}
function requiredReadinessCompleteForItem(item) {
  const category = readinessCategoryForItem(item);
  if (!category || !state.web3dReadiness?.pending) return;
  state.web3dReadiness.pending.delete(readinessKey(category, item));
  maybeEmitSceneReady();
}
function failWeb3dSceneReadiness(item, url, reason, extra = {}) {
  const category = readinessCategoryForItem(item) || extra.category || 'required_physical_item';
  emitWeb3dReadinessState('scene_failed', {
    required_category: category,
    item_id: item?.id || '',
    link: item?.link || item?.link_name || item?.object_name || '',
    url: url || displayMeshUri(item),
    reason: reason || 'required mesh failed',
    ...extra,
  });
}
function completeExpandedUrdfReadiness(result) {
  if (!state.web3dReadiness?.pending) return;
  state.web3dReadiness.pending.delete('robot_arm:expanded_urdf_loader');
  state.web3dReadiness.pending.delete('attached_tool_gripper:expanded_urdf_loader');
  maybeEmitSceneReady();
}
function failExpandedUrdfReadiness(err, diagnostics = {}, detail = {}) {
  emitWeb3dReadinessState('scene_failed', {
    required_category: 'robot_arm',
    item_id: 'expanded_urdf_loader',
    link: detail.link || detail.link_name || '',
    url: detail.url || detail.uri || diagnostics.robot_urdf_url || '',
    reason: err?.message || String(err || 'expanded URDF required mesh failed'),
    robot_missing_meshes: diagnostics.robot_missing_meshes || [],
    ...detail,
  });
}
function maybeEmitSceneReady() {
  if (failIfCanonicalRequiredVisualSetInvalid()) return;
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
  undoEdit: document.getElementById('undo-edit'),
  redoEdit: document.getElementById('redo-edit'),
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
  canvas: document.getElementById('scene-canvas'),
  labelLayer: document.getElementById('label-layer'),
  empty: document.getElementById('empty-state'),
  error: document.getElementById('error-state'),
  list: document.getElementById('object-list'),
  inspector: document.getElementById('inspector'),
  warnings: document.getElementById('warnings'),
  summary: document.getElementById('scene-summary'),
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
  return (state.objects || []).filter(obj => !isDebugOverlayItem(obj.item));
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

function canonicalRequiredGeneratedVisualSet(json = state.sceneJson || {}) {
  const id = String(json?.scene?.id || json?.scene_id || '').trim();
  if (id !== CANONICAL_UR5_2F_REQUIRED_GENERATED_VISUAL_SET.scene_id) return null;
  const preview = json?.robot_preview || {};
  if (!isExpandedUrdfRobotPreview(preview)) return null;
  const previewLinks = new Set(asArray(preview.expected_links).map(link => String(link || '').trim()).filter(Boolean));
  const requiredUr5 = CANONICAL_UR5_2F_REQUIRED_UR5_VISUALS.filter(link => !previewLinks.size || previewLinks.has(link));
  const requiredRobotiq = CANONICAL_UR5_2F_REQUIRED_ROBOTIQ_VISUALS.filter(link => !previewLinks.size || previewLinks.has(link));
  return {
    scene_id: id,
    sceneId: id,
    ur5_visuals: requiredUr5,
    ur5Visuals: requiredUr5,
    robotiq_visuals: requiredRobotiq,
    robotiqVisuals: requiredRobotiq,
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
function canonicalVisualReadinessDiagnostics() {
  const required = canonicalRequiredGeneratedVisualSet();
  if (!required) return null;
  const urdfVisualLinks = asArray(state.robotUrdfPreviewDiagnostics?.robot_visual_wrapper_world_matrices)
    .map(visual => String(visual?.link_name || visual?.linkName || '').trim())
    .filter(Boolean);
  const urdfCounts = countBy(urdfVisualLinks);
  const sceneDiagnostics = collectRenderedMeshDiagnostics();
  const categoryCounts = countBy(sceneDiagnostics.map(item => readinessCategoryForItem(item)).filter(Boolean));
  const missing = [];
  const duplicate = [];
  const failed = [];
  for (const link of required.ur5_visuals.concat(required.robotiq_visuals)) {
    const count = urdfCounts[link] || 0;
    if (count === 0) missing.push(link);
    if (count > 1) duplicate.push(link);
  }
  for (const category of required.table_visuals.concat(required.camera_visuals)) {
    const count = categoryCounts[category] || 0;
    if (count === 0) missing.push(category);
    if (count > 1) duplicate.push(category);
  }
  for (const entry of sceneDiagnostics) {
    if (isRequiredMeshFailureStatus({ item: entry, renderInfo: { render_status: entry.render_status } })) failed.push(entry.link_name || entry.id || entry.category);
  }
  if (Number(state.robotUrdfPreviewDiagnostics?.robot_failed_visual_count || 0) > 0) failed.push('expanded_urdf_loader');
  return {
    canonical_required_generated_visual_set: required,
    canonicalRequiredGeneratedVisualSet: required,
    canonical_required_visual_counts: { ...urdfCounts, ...categoryCounts },
    canonicalRequiredVisualCounts: { ...urdfCounts, ...categoryCounts },
    canonical_missing_required_visuals: missing,
    canonicalMissingRequiredVisuals: missing,
    canonical_duplicate_required_visuals: duplicate,
    canonicalDuplicateRequiredVisuals: duplicate,
    canonical_failed_required_visuals: Array.from(new Set(failed)),
    canonicalFailedRequiredVisuals: Array.from(new Set(failed)),
    canonical_required_visual_ready: missing.length === 0 && duplicate.length === 0 && failed.length === 0,
    canonicalRequiredVisualReady: missing.length === 0 && duplicate.length === 0 && failed.length === 0,
  };
}
function failIfCanonicalRequiredVisualSetInvalid() {
  const lifecycle = String(state.robotUrdfPreviewDiagnostics?.robot_preview_lifecycle_state || state.robotUrdfPreviewDiagnostics?.robotPreviewLifecycleState || '');
  if (canonicalRequiredGeneratedVisualSet() && lifecycle !== 'ready' && lifecycle !== 'failed') return false;
  const diagnostics = canonicalVisualReadinessDiagnostics();
  if (!diagnostics || diagnostics.canonical_required_visual_ready) return false;
  emitWeb3dReadinessState('scene_failed', {
    required_category: 'canonical_generated_visual_set',
    reason: 'canonical required generated visual set is missing, failed, or duplicated',
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
  const canonicalVisualDiagnostics = canonicalVisualReadinessDiagnostics() || {};
  window.__WORKCELL_VIEWER_STATUS__ = {
    viewer_boot_state: state.web3dReadiness?.state || 'server_ready',
    viewerBootState: state.web3dReadiness?.state || 'server_ready',
    web3d_readiness_state: state.web3dReadiness?.state || 'server_ready',
    web3dReadinessState: state.web3dReadiness?.state || 'server_ready',
    failed_stage: state.web3dReadiness?.state === 'scene_failed' ? 'scene_failed' : '',
    failedStage: state.web3dReadiness?.state === 'scene_failed' ? 'scene_failed' : '',
    fatal_error: state.web3dReadiness?.failure?.reason || '',
    fatalError: state.web3dReadiness?.failure?.reason || '',
    fatal_stack: '',
    fatalStack: '',
    source_web_scene_file: state.sourceWebSceneFile || '',
    sourceWebSceneFile: state.sourceWebSceneFile || '',
    scene_json_loaded: Boolean(state.sceneJson),
    sceneJsonLoaded: Boolean(state.sceneJson),
    scene_name: summary.sceneName,
    sceneName: summary.sceneName,
    renderable_count: summary.renderableCount,
    renderableCount: summary.renderableCount,
    mesh_loaded_count: summary.meshLoadedCount,
    meshLoadedCount: summary.meshLoadedCount,
    required_mesh_failed_count: statusCountedRenderables().filter(isRequiredMeshFailureStatus).length,
    requiredMeshFailedCount: statusCountedRenderables().filter(isRequiredMeshFailureStatus).length,
    fallback_count: summary.fallbackCount,
    fallbackCount: summary.fallbackCount,
    runtime_warnings: warnings,
    runtimeWarnings: warnings,
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
    ...canonicalVisualDiagnostics,
    required_physical_categories: state.web3dReadiness?.required || {},
    requiredPhysicalCategories: state.web3dReadiness?.required || {},
    pending_required_loads: Array.from(state.web3dReadiness?.pending || []),
    pendingRequiredLoads: Array.from(state.web3dReadiness?.pending || []),
    ...structuredWeb3dReadinessFields(state.web3dReadiness?.state || 'server_ready'),
    readiness_failure: state.web3dReadiness?.failure || null,
    readinessFailure: state.web3dReadiness?.failure || null,
  };
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
function editorState() {
  const rendered = renderedById(state.selected);
  return { ready: Boolean(state.sceneJson && state.three?.scene), sceneId: sceneId(), selectedItemId: state.selected || '', selectedItemType: rendered ? itemType(rendered.item) : '', selectedEditable: Boolean(rendered && rendered.item && canEditItem(rendered['item'])), dirty: state.dirtyTransforms.size > 0, dirtyCount: state.dirtyTransforms.size, canUndo: state.undoStack.length > 0, canRedo: state.redoStack.length > 0, mode: state.editorMode, error: state.editorError || '' };
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
  const scale = isGeneratedUrdfItem(item) ? [1, 1, 1] : (item.scale || item.mesh_scale || [1, 1, 1]);
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
  const scaleSource = transform.scale || item?.mesh_scale || item?.scale || [1, 1, 1];
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
function renderedById(id) { return state.objects.find(obj => obj.item.id === id); }
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
function markDirtyTransform(rendered, next, { pushHistory = true, oldTransform = null, snapOptions = undefined } = {}) {
  if (!rendered || !canEditItem(rendered.item)) return false;
  const previous = oldTransform || state.dirtyTransforms.get(rendered.item.id)?.newTransform || transformFromObject(rendered.object3d);
  const snapped = snapOptions === null ? cloneTransform(next) : snapTransform(next, snapOptions);
  if (pushHistory && !sameTransform(previous, snapped)) {
    state.undoStack.push({ itemId: rendered.item.id, before: cloneTransform(previous), after: cloneTransform(snapped) });
    state.redoStack = [];
  }
  if (!applyTransformToObject(rendered.object3d, snapped)) return false;
  if (sameTransform(rendered.originalTransform, snapped)) state.dirtyTransforms.delete(rendered.item.id);
  else state.dirtyTransforms.set(rendered.item.id, { oldTransform: cloneTransform(rendered.originalTransform), newTransform: cloneTransform(snapped) });
  syncInspectorTransformFields(rendered);
  updateDirtyState();
  updateLabels();
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
  state.runtimeWarnings.push(warning);
  refreshWarnings();
  return warning;
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
  state.runtimeWarnings.push({
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
  refreshWarnings();
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
  if (stagedHttpUrl) return meshUriDiagnostic({ ...item, mesh_uri: stagedHttpUrl });
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
  if (item?.debug_overlay === true || item?.exclude_from_fit_bounds === true || item?.source_layer === 'debug_overlay') return true;
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
  if (isZone(item)) return new THREE.MeshBasicMaterial({ color: 0xffc857, transparent: true, opacity: 0.08, side: THREE.DoubleSide, wireframe: true, depthWrite: false });
  if (isSensor(item)) return new THREE.MeshStandardMaterial({ color: 0x62d2ff, roughness: 0.65 });
  if (item.locked || item.source_kind === 'generated_preview') return new THREE.MeshStandardMaterial({ color: 0x8794aa, roughness: 0.78, metalness: 0.05 });
  return new THREE.MeshStandardMaterial({ color: 0x7bd88f, roughness: 0.72 });
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
  group.add(frustum);
  applyFallbackRenderMetadata(group, item, 'sensor_fallback');
  return group;
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
  meshObject.rotation.set(visualOrigin.rpy.x, visualOrigin.rpy.y, visualOrigin.rpy.z, 'XYZ');
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
async function tryLoadMesh(item, rendered, fallback) {
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
    if (required) failWeb3dSceneReadiness(item, requestedUri, diagnostic.reason, { mesh_status: diagnostic.status });
    else requiredReadinessCompleteForItem(item);
    return;
  }
  const ext = meshExtensionFromUri(uri);
  const loaderName = meshLoaderNameForExtension(ext);
  const loadUrl = repoRootRelativeUrl(uri);
  const preflight = await preflightMeshUrl(uri, loadUrl);
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
    if (required) failWeb3dSceneReadiness(item, preflight.url || loadUrl, item.mesh_load_error, { http_status: preflight.http_status || null, mesh_status: item.mesh_status });
    else requiredReadinessCompleteForItem(item);
    return;
  }
  try {
    let loaded;
    if (ext === 'stl') loaded = await new STLLoader().loadAsync(loadUrl);
    else if (ext === 'dae') loaded = await new ColladaLoader().loadAsync(loadUrl);
    else loaded = await new OBJLoader().loadAsync(loadUrl);
    const meshObject = materializeLoadedMesh(item, uri, loaded);
    // Legacy flow was applyMeshLocalTransform(meshObject, item) followed by
    // rendered.object3d.add(meshObject); generated URDF now applies that
    // local transform to the visual-origin wrapper so the loaded hierarchy is
    // preserved under the posed link root.
    const visualRoot = makeMeshVisualRoot(item, meshObject);
    visualRoot.updateMatrixWorld(true);
    const nativeBounds = new THREE.Box3().setFromObject(visualRoot);
    const autoscaled = maybeApplyMeshUnitAutoscale(item, meshObject, nativeBounds, uri);
    const validationBounds = autoscaled ? new THREE.Box3().setFromObject(visualRoot) : nativeBounds;
    if (fallback) fallback.visible = false;
    rendered.object3d.add(visualRoot);
    rendered.meshObject = visualRoot;
    rendered.loadedMeshObject = meshObject;
    item.mesh_status = 'loaded';
    item.mesh_load_error = '';
    trackMeshLoadAttempt(item, 'loaded', loadUrl, '');
    setRenderInfo(rendered, 'mesh_loaded', uri, '');
    diagnoseLoadedMeshBounds(item, visualRoot, rendered, validationBounds);
    refreshMeshLoadUi(rendered);
    requiredReadinessCompleteForItem(item);
  } catch (err) {
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
    if (required) failWeb3dSceneReadiness(item, loadUrl, reason, { extension: ext, loader: loaderName, mesh_status: item.mesh_status });
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
    const camera = new THREE.PerspectiveCamera(55, 1, 0.01, 100);
    camera.up.copy(ROS_Z_UP);
    camera.position.set(2.4, -2.8, 1.8);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.object.up.copy(ROS_Z_UP);
    controls.enableDamping = true;
    const grid = new THREE.GridHelper(5, 20, PRODUCT_VIEW_LIGHT_PALETTE.gridMajor, PRODUCT_VIEW_LIGHT_PALETTE.gridMinor);
    grid.name = 'ros_xy_ground_grid';
    grid.up.copy(ROS_Z_UP);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);
    scene.add(new THREE.AxesHelper(0.75));
    scene.add(new THREE.HemisphereLight(0xffffff, 0x8a96a3, 1.25));
    const light = new THREE.DirectionalLight(0xffffff, 1.35); light.position.set(2, -3, 4); scene.add(light);
    const transformControls = new TransformControls(camera, renderer.domElement);
    transformControls.setMode('translate');
    transformControls.setSpace('world');
    transformControls.addEventListener('dragging-changed', event => {
      if (state.editorMode !== 'rotate') controls.enabled = !event.value;
      const rendered = renderedById(state.selected);
      if (!rendered || !canEditItem(rendered.item)) return;
      if (state.editorMode === 'rotate') {
        if (event.value) beginDirectRotateDrag(rendered);
        else finishDirectRotateDrag(rendered);
        return;
      }
      if (event.value) state.gizmoDragStart = cloneTransform(state.dirtyTransforms.get(rendered.item.id)?.newTransform || transformFromObject(rendered.object3d));
      else { const committed = markDirtyTransform(rendered, transformFromObject(rendered.object3d), { pushHistory: true, oldTransform: state.gizmoDragStart }); if (committed) emitTransformCommitted(rendered); state.gizmoDragStart = null; }
    });
    transformControls.addEventListener('objectChange', () => {
      const rendered = renderedById(state.selected);
      if (!rendered || !canEditItem(rendered.item)) return;
      if (state.editorMode === 'rotate') { previewDirectRotateDrag(rendered); return; }
      const snapped = snapTransform(transformFromObject(rendered.object3d));
      applyTransformToObject(rendered.object3d, snapped);
      syncInspectorTransformFields(rendered);
      updateLabels();
    });
    controls.addEventListener('start', markCameraUserControlled);
    scene.add(transformControls);
    state.three = { renderer, scene, camera, controls, transformControls, raycaster: new THREE.Raycaster(), pointer: new THREE.Vector2() };
    resize();
    window.addEventListener('resize', resize);
    el.canvas.addEventListener('pointerdown', onCanvasPointerDown);
    el.canvas.addEventListener('pointermove', onCanvasPointerMove);
    el.canvas.addEventListener('pointerup', onCanvasPointerUp);
    el.canvas.addEventListener('pointercancel', () => { cancelDirectMoveDrag('Move cancelled'); cancelDirectRotateDrag('Rotation cancelled'); });
    window.addEventListener('keydown', event => { if (event.key === 'Escape') { cancelDirectMoveDrag('Move cancelled'); cancelDirectRotateDrag('Rotation cancelled'); } });
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
  if (object.userData?.exclude_from_physical_bounds === true || object.userData?.exclude_from_fit_bounds === true) return true;
  if (item?.exclude_from_physical_bounds === true || item?.exclude_from_fit_bounds === true) return true;
  if (DEBUG_OVERLAY_TOKEN_RE.test(identity)) return true;
  return !isInitialFitPhysicalGeometryItem(item, identity);
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
  let hasGeneratedRobotMesh = false;
  const visit = (node, nearestItem = null) => {
    if (!node || node.visible === false) return;
    const item = physicalBoundsItemFor(node, nearestItem);
    const identity = physicalBoundsIdentityFor(node);
    const nextNearestItem = node?.userData?.item || nearestItem;
    const isRenderable = node.isMesh || node.isLine || node.isLineSegments || node.isPoints || node.isSprite;
    if (isRenderable && !isPhysicalBoundsHelperObject(node, item, identity)) {
      const visualSource = String(item?.active_visual_source || node?.userData?.active_visual_source || '').toLowerCase();
      const isGenerated = /generated|urdf/.test(identity);
      const isRobot = /robot|arm|manipulator|ur3|ur5|ur10|universal robot|base link|shoulder|wrist/.test(identity);
      if (isGenerated && isRobot && /mesh|generated urdf visual|mesh preview/.test(visualSource.replace(/_/g, ' '))) hasGeneratedRobotMesh = true;
      candidates.push({ node, item, identity });
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
function meshUnitCorrectionPayload(source, confidence, nativeBounds, correctedBounds, scale, axisRatios, targetRatio) {
  return {
    source,
    confidence,
    native_bounds: box3ToJson(nativeBounds),
    corrected_bounds: box3ToJson(correctedBounds),
    scale,
    axis_ratios: axisRatios,
    target_ratio: targetRatio,
  };
}
function maybeApplyMeshUnitAutoscale(item, meshObject, nativeBounds, meshUri) {
  if (!meshUnitAutoscaleAllowed(item)) return false;
  const expected = expectedDimensionsOf(item);
  const finiteNative = finiteBox3(nativeBounds);
  const dims = finiteNative ? box3Dimensions(finiteNative) : null;
  const axes = ['x', 'y', 'z'];
  if (!expected || !dims || axes.some(axis => dims[axis] <= 1e-9 || expected[axis] <= 1e-9)) return false;
  const axisRatios = Object.fromEntries(axes.map(axis => [axis, dims[axis] / expected[axis]]));
  const ratioValues = Object.values(axisRatios);
  const maxRatio = Math.max(...ratioValues);
  const minRatio = Math.min(...ratioValues);
  const uniformRatio = minRatio > 0 ? maxRatio / minRatio : Infinity;
  const targetRatio = [1000, 100].find(target => Math.abs(maxRatio - target) / target <= 0.2 && Math.abs(minRatio - target) / target <= 0.2 && uniformRatio <= 1.25);
  if (!targetRatio) {
    item.mesh_unit_correction = meshUnitCorrectionPayload('viewer_expected_dimensions_m', 'rejected_non_uniform_or_unclear_ratio', finiteNative, finiteNative, 1.0, axisRatios, null);
    appendRuntimeWarning(item, meshUri, `mesh unit autoscale rejected: native bounds do not have a clear uniform 100x or 1000x ratio to expected_dimensions_m (uniform_ratio=${uniformRatio.toFixed(3)})`, 'mesh_unit_autoscale_rejected', item.mesh_unit_correction);
    return false;
  }
  const scale = targetRatio === 1000 ? 0.001 : 0.01;
  meshObject.scale.multiplyScalar(scale);
  meshObject.updateMatrixWorld(true);
  const correctedBounds = finiteBox3(new THREE.Box3().setFromObject(meshObject));
  const confidence = targetRatio === 1000 ? 'auto_detected_mm_to_m' : 'auto_detected_cm_to_m';
  item.mesh_unit_correction = meshUnitCorrectionPayload('viewer_expected_dimensions_m', confidence, finiteNative, correctedBounds, scale, axisRatios, targetRatio);
  item.visual_bounds_status = 'corrected_by_local_unit_scale';
  appendRuntimeWarning(item, meshUri, `mesh unit autoscale applied: native bounds matched a clear ${targetRatio}x ratio to expected_dimensions_m`, 'mesh_unit_autoscale_applied', item.mesh_unit_correction);
  return true;
}

function isCoreMeshContractItem(item) {
  return !['helper'].includes(meshContractCategoryOf(item)) && !isZone(item);
}
function warnLoadedMeshBounds(item, code, reason, extra = {}) {
  item.visual_bounds_status = code === 'loaded_mesh_oversized' ? 'oversized' : code === 'loaded_mesh_collapsed' ? 'collapsed' : 'invalid';
  appendViewerDiagnosticWarning(item, code, reason, {
    mesh_uri: displayMeshUri(item),
    loaded_mesh_bounds: item.loaded_mesh_bounds,
    loaded_mesh_world_bounds: item.loaded_mesh_world_bounds,
    expected_dimensions_m: item.expected_dimensions_m,
    mesh_contract_category: meshContractCategoryOf(item),
    ...extra,
  });
}
function diagnoseLoadedMeshBounds(item, meshObject, rendered, nativeBounds = null) {
  const localBounds = finiteBox3(nativeBounds || new THREE.Box3().setFromObject(meshObject));
  rendered.object3d.updateWorldMatrix(true, true);
  const worldBounds = finiteBox3(new THREE.Box3().setFromObject(meshObject));
  item.loaded_mesh_bounds = box3ToJson(localBounds);
  item.loaded_mesh_world_bounds = box3ToJson(worldBounds);
  item.visual_bounds_status = item.mesh_unit_correction?.scale && item.mesh_unit_correction.scale !== 1.0 ? 'corrected_by_local_unit_scale' : 'valid';
  const expected = expectedDimensionsOf(item);
  const dims = localBounds ? box3Dimensions(localBounds) : null;
  const worldDims = worldBounds ? box3Dimensions(worldBounds) : null;
  const tiny = 1e-9;
  if (!localBounds || !worldBounds || !dims || !worldDims) {
    if (isCoreMeshContractItem(item)) warnLoadedMeshBounds(item, 'loaded_mesh_bounds_invalid', 'loaded mesh produced empty or non-finite bounds');
    return;
  }
  maybeWarnSupportSurfaceSemantics(item, dims);
  const collapsedAxes = ['x', 'y', 'z'].filter(axis => dims[axis] <= tiny || worldDims[axis] <= tiny);
  if (collapsedAxes.length) {
    if (isCoreMeshContractItem(item)) warnLoadedMeshBounds(item, 'loaded_mesh_collapsed', `loaded mesh bounds are zero-volume or collapsed on ${collapsedAxes.join(', ')}`, { collapsed_axes: collapsedAxes });
    return;
  }
  if (!expected) return;
  const axes = ['x', 'y', 'z'];
  const axisRatios = Object.fromEntries(axes.map(axis => [axis, dims[axis] / expected[axis]]));
  const maxRatio = Math.max(...Object.values(axisRatios));
  const minRatio = Math.min(...Object.values(axisRatios));
  const uniformRatio = minRatio > 0 ? maxRatio / minRatio : Infinity;
  const category = meshContractCategoryOf(item);
  if (maxRatio > 3 || uniformRatio > 3) {
    if (['table', 'camera', 'object'].includes(category)) warnLoadedMeshBounds(item, 'loaded_mesh_oversized', `loaded mesh dimensions exceed expected_dimensions_m by more than 3x (max_axis_ratio=${maxRatio.toFixed(3)}, uniform_ratio=${uniformRatio.toFixed(3)})`, {
      expected_dimensions: { x: expected.x, y: expected.y, z: expected.z },
      loaded_dimensions: { x: dims.x, y: dims.y, z: dims.z },
      axis_ratios: axisRatios,
      max_axis_ratio: maxRatio,
      uniform_ratio: uniformRatio,
    });
    else if (Math.abs(maxRatio - 1000) < 100 || Math.abs(maxRatio - 0.001) < 0.001) item.visual_bounds_status = 'corrected_by_local_unit_scale';
  }
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
function frameScene(bounds) {
  const { camera, controls } = state.three;
  const finiteBounds = finiteBox3(bounds);
  if (!camera || !controls || !finiteBounds) return false;
  const center = new THREE.Vector3();
  const sphere = new THREE.Sphere();
  finiteBounds.getCenter(center);
  finiteBounds.getBoundingSphere(sphere);
  if (![center.x, center.y, center.z, sphere.radius].every(Number.isFinite)) return false;
  const radius = Math.max(sphere.radius, MIN_FRAME_RADIUS);
  const direction = new THREE.Vector3(1.35, -1.65, 1.05).normalize();
  const distance = Math.max(radius * FRAME_DISTANCE_MULTIPLIER, MIN_FRAME_RADIUS * FRAME_DISTANCE_MULTIPLIER);
  camera.position.copy(center).addScaledVector(direction, distance);
  camera.near = Math.max(0.01, Math.min(radius / 100, distance / 10));
  camera.far = Math.max(camera.near + 1, distance + radius * 6, 100);
  if (![camera.position.x, camera.position.y, camera.position.z, camera.near, camera.far].every(Number.isFinite)) return false;
  camera.updateProjectionMatrix();
  controls.target.copy(center);
  controls.update();
  state.lastFrameBounds = finiteBounds.clone();
  if (el.resetView) el.resetView.disabled = false;
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
function resetView({ userInitiated = true } = {}) {
  if (userInitiated) markCameraUserControlled();
  const physical = collectPhysicalVisibleBounds(state.three.scene);
  if (!physical.bounds || !frameScene(physical.bounds)) { reportFitCellNoGeometry(); return false; }
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
function clearSceneObjects() {
  const scene = state.three.scene;
  if (!scene) return;
  for (const root of state.assemblyRoots || []) scene.remove(root);
  for (const rendered of state.objects) scene.remove(rendered.object3d);
  clearLabels();
  state.objects = [];
  state.assemblyRoots = [];
  state.robotAssemblyDiagnostics = [];
  state.robotAssemblyRenderDiagnostics = {};
  state.physicalAssemblyBounds = null;
  state.finalPhysicalFitBounds = null;
  state.resolvedFramePoses.clear();
  state.lastFrameBounds = null;
  state._sceneBoundsExceededWarned = false;
  state._fitBlockerWarnings = new Set();
  state._generatedUrdfFramePoseWarnings = new Set();
  state._supportSurfaceSemanticWarnings = new Set();
  state.robotPreviewResult = null;
  state.initialPosePreview = { active: false, robotId: '', sceneKey: '' };
  if (el.showInitialPose) el.showInitialPose.checked = false;
  setInitialPosePreviewUi(false);
  if (el.resetView) el.resetView.disabled = true;
  renderSceneSummary();
}
function renderScene(items) {
  clearSceneObjects();
  beginWeb3dSceneReadiness(items);
  state.dirtyTransforms.clear();
  state.undoStack = [];
  state.redoStack = [];
  detachTransformGizmo();
  updateDirtyState();
  const scene = state.three.scene;
  state.robotUrdfPreviewDiagnostics = {};
  const urdfPreviewActive = isExpandedUrdfRobotPreview(state.sceneJson?.robot_preview);
  const robotToolGeneratedUrdfItems = items.filter(isRobotToolGeneratedUrdfMeshVisualItem);
  const assemblyBuild = urdfPreviewActive ? { handled: new Set(robotToolGeneratedUrdfItems), assemblies: [], renderDiagnostics: { skipped_flattened_urdf_visual_count: 0, assembled_hierarchy_rendered_mesh_count: 0, rendered_fk_visual_count: 0, skipped_legacy_generated_urdf_count: robotToolGeneratedUrdfItems.length, skipped_legacy_generated_urdf_visual_count: robotToolGeneratedUrdfItems.length, visible_duplicate_generated_urdf_count: 0, visible_tool0_fallback_count: 0, detached_robot_mesh_clusters: 0 } } : buildRobotAssemblies(items);
  state.robotAssemblyDiagnostics = assemblyBuild.assemblies;
  state.robotAssemblyRenderDiagnostics = assemblyBuild.renderDiagnostics || {};
  if (urdfPreviewActive) loadExpandedUrdfRobotPreview(state.sceneJson.robot_preview);
  for (const item of items) {
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
    const rendered = { item, object3d, fallback, labelEl: createLabelElement(item), originalTransform: transformOf(item) };
    const requiredMesh = itemRequiresMeshBackedVisual(item);
    const fallbackStatus = requiredMesh ? 'mesh_loading_required' : (fallback ? 'primitive_fallback' : 'no_physical_dimensions');
    const fallbackReason = requiredMesh ? 'required mesh is loading; primitive fallback hidden unless mesh load fails as debug geometry' : (fallback ? 'primitive geometry rendered while mesh loads or is unavailable' : 'no mesh or physical primitive dimensions were provided; Product View box fallback suppressed');
    if (fallback) fallback.visible = !requiredMesh;
    setRenderInfo(rendered, fallbackStatus, displayMeshUri(item), fallbackReason);
    state.objects.push(rendered);
    maybeWarnSupportSurfaceSemantics(item);
    tryLoadMesh(item, rendered, fallback);
  }
  renderFrameDebugOverlays();
  populateObjectList();
  updateLabels();
  renderSceneSummary();
  maybeEmitSceneReady();
}



function loadExpandedUrdfRobotPreview(preview) {
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
  if (typeof loadRobotPreview !== 'function') {
    diagnostics.robot_preview_lifecycle_state = 'failed';
    diagnostics.robotPreviewLifecycleState = 'failed';
    diagnostics.robot_failed_visual_count = 1;
    diagnostics.robotFailedVisualCount = 1;
    diagnostics.robot_missing_meshes.push('urdf_robot_renderer module was not loaded');
    appendRuntimeWarning({}, preview?.urdf_url || '', 'expanded_urdf_loader failed: urdf_robot_renderer module was not loaded', 'expanded_urdf_loader_failed');
    refreshWarnings();
    return { root: null, links: new Map(), joints: new Map(), diagnostics, ready: Promise.resolve(null) };
  }
  const previewResult = loadRobotPreview(preview, {
    sceneId: sceneId(),
    scene: state.three.scene,
    assemblyRoots: state.assemblyRoots,
    repoRootRelativeUrl,
    meshUriDiagnostic,
    rootName: `${sceneDisplayName()}_expanded_urdf_loader_robot`,
    skippedLegacyGeneratedUrdfVisualCount: diagnostics.skipped_legacy_generated_urdf_visual_count,
    onRobotLoaded: result => {
      state.robotPreviewResult = result;
      state.robotUrdfPreviewDiagnostics = result.diagnostics;
      if (!failIfCanonicalRequiredVisualSetInvalid()) completeExpandedUrdfReadiness(result);
      refreshInitialPoseActionState();
      renderSceneSummary();
    },
    onRobotMeshLoaded: () => {
      renderSceneSummary();
    },
    onRobotMeshLoadError: (err, uri, detail) => { failExpandedUrdfReadiness(err, state.robotUrdfPreviewDiagnostics, detail || { uri }); renderSceneSummary(); },
    onRobotError: (err, diagnostics) => {
      failExpandedUrdfReadiness(err, diagnostics || state.robotUrdfPreviewDiagnostics);
      appendRuntimeWarning({}, preview?.urdf_url || '', `expanded_urdf_loader failed: ${err?.message || err}`, 'expanded_urdf_loader_failed');
      refreshWarnings();
      renderSceneSummary();
    },
  });
  state.robotUrdfPreviewDiagnostics = previewResult.diagnostics;
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
      const rendered = { item, object3d, fallback, labelEl: createLabelElement(item), originalTransform: transformOf(item) };
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
      const rendered = { item, object3d: node, fallback, labelEl: createLabelElement(item), originalTransform: transformOf(item) };
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

function selectObject(id) {
  const wasInitialPreviewActive = state.initialPosePreview.active;
  if (state.directMoveDrag && state.directMoveDrag.itemId !== (id || '')) cancelDirectMoveDrag('Move cancelled');
  if (state.directRotateDrag && state.directRotateDrag.itemId !== (id || '')) cancelDirectRotateDrag('Rotation cancelled');
  const previous = state.selected || '';
  state.selected = id;
  document.querySelectorAll('.object-list li').forEach(li => li.classList.toggle('selected', li.dataset.id === id));
  for (const rendered of state.objects) {
    const selected = rendered.item.id === id;
    rendered.object3d.traverse(child => {
      if (child.material?.emissive) child.material.emissive.setHex(selected ? 0x1b6f8f : 0x000000);
    });
    if (rendered.labelEl) rendered.labelEl.classList.toggle('selected', selected);
  }
  updateLabels();
  const rendered = state.objects.find(obj => obj.item.id === id);
  if (rendered) { populateInspector(rendered); attachTransformGizmo(rendered); } else detachTransformGizmo();
  if (previous !== (id || '')) pushEditorEvent('selection_changed', { itemId: id || '', itemType: rendered ? itemType(rendered.item) : '', editable: Boolean(rendered && rendered.item && canEditItem(rendered['item'])) });
}
function clearSelection() { selectObject(''); el.inspector.className = 'state empty'; el.inspector.textContent = state.objects.length ? 'Select an object from the list or canvas.' : EMPTY_SCENE_MESSAGE; }
function pickObject(event) {
  const rect = el.canvas.getBoundingClientRect();
  state.three.pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  state.three.pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  state.three.raycaster.setFromCamera(state.three.pointer, state.three.camera);
  const hits = state.three.raycaster.intersectObjects(state.objects.map(o => o.object3d), true);
  const hit = hits.find(h => h.object?.parent || h.object);
  const item = hit?.object?.userData?.item || hit?.object?.parent?.userData?.item;
  if (item?.id) selectObject(item.id);
  return item?.id || '';
}



function pointerToWorldPlane(event, z) { const rect = el.canvas.getBoundingClientRect(); if (!rect.width || !rect.height) return null; state.three.pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1; state.three.pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1; state.three.raycaster.setFromCamera(state.three.pointer, state.three.camera); const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -z); const hit = new THREE.Vector3(); if (!state.three.raycaster.ray.intersectPlane(plane, hit)) return null; return Number.isFinite(hit.x) && Number.isFinite(hit.y) && Number.isFinite(hit.z) ? hit : null; }
function snapHorizontalPreview(transform) { const snapped = snapTransform(transform, { translationAxes: ['x', 'y'], rotationAxes: [] }); snapped.pose.xyz.z = transform.pose.xyz.z; return snapped; }
function beginDirectMoveDrag(event, rendered) { if (state.editorMode !== 'move' || !rendered || !canEditItem(rendered.item)) return false; const start = cloneTransform(state.dirtyTransforms.get(rendered.item.id)?.newTransform || transformFromObject(rendered.object3d)); const hit = pointerToWorldPlane(event, start.pose.xyz.z); if (!hit) return false; const controls = state.three.controls; state.directMoveDrag = { itemId: rendered.item.id, start, last: cloneTransform(start), offset: { x: start.pose.xyz.x - hit.x, y: start.pose.xyz.y - hit.y }, controlsWasEnabled: controls ? controls.enabled : true }; if (controls) controls.enabled = false; el.canvas.setPointerCapture?.(event.pointerId); el.canvas.classList.add('direct-move-dragging'); event.preventDefault(); event.stopPropagation(); return true; }
function updateDirectMoveDrag(event) { const drag = state.directMoveDrag; if (!drag) return false; const rendered = renderedById(drag.itemId); if (!rendered || state.selected !== drag.itemId || !canEditItem(rendered.item)) return cancelDirectMoveDrag('Move cancelled'), true; const hit = pointerToWorldPlane(event, drag.start.pose.xyz.z); if (!hit) return true; const next = cloneTransform(drag.start); next.pose.xyz.x = hit.x + drag.offset.x; next.pose.xyz.y = hit.y + drag.offset.y; next.pose.xyz.z = drag.start.pose.xyz.z; const preview = snapHorizontalPreview(next); if (!isFiniteTransform(preview)) return true; drag.last = cloneTransform(preview); applyTransformToObject(rendered.object3d, preview); syncInspectorTransformFields(rendered); updateLabels(); event.preventDefault(); event.stopPropagation(); return true; }
function finishDirectMoveDrag(event) { const drag = state.directMoveDrag; if (!drag) return false; const rendered = renderedById(drag.itemId); const finalTransform = cloneTransform(drag.last); endDirectMoveDrag(event); if (!rendered || !canEditItem(rendered.item) || !isFiniteTransform(finalTransform)) return false; if (sameTransform(drag.start, finalTransform)) { applyTransformToObject(rendered.object3d, drag.start); syncInspectorTransformFields(rendered); return true; } const committed = markDirtyTransform(rendered, finalTransform, { pushHistory: true, oldTransform: drag.start }); if (!committed) { applyTransformToObject(rendered.object3d, drag.start); showError(`Move failed for ${itemLabel(rendered.item)}: final transform was rejected by the editor bridge.`); return true; } emitTransformCommitted(rendered); pushEditorEvent('status', { message: `Moved ${itemLabel(rendered.item)}` }); return true; }
function endDirectMoveDrag(event) { const drag = state.directMoveDrag; state.directMoveDrag = null; if (state.three.controls) state.three.controls.enabled = drag ? drag.controlsWasEnabled : state.three.controls.enabled; el.canvas.classList.remove('direct-move-dragging'); if (event?.pointerId !== undefined) el.canvas.releasePointerCapture?.(event.pointerId); }
function cancelDirectMoveDrag(message) { const drag = state.directMoveDrag; if (!drag) return false; const rendered = renderedById(drag.itemId); if (rendered) { applyTransformToObject(rendered.object3d, drag.start); syncInspectorTransformFields(rendered); updateLabels(); } endDirectMoveDrag(); pushEditorEvent('status', { message: message || 'Move cancelled' }); return true; }
function onCanvasPointerDown(event) { const hitId = pickObject(event); const rendered = hitId && hitId === state.selected ? renderedById(state.selected) : null; if (beginDirectMoveDrag(event, rendered)) return; }
function onCanvasPointerMove(event) { updateDirectMoveDrag(event); }
function onCanvasPointerUp(event) { finishDirectMoveDrag(event); }

function directRotatePreviewTransform(rendered) {
  const drag = state.directRotateDrag;
  if (!drag || !rendered || rendered.item.id !== drag.itemId) return null;
  const next = cloneTransform(drag.start);
  next.pose.rpy.z = transformFromObject(rendered.object3d).pose.rpy.z;
  return snapTransform(next, { translationAxes: [], rotationAxes: ['z'] });
}
function beginDirectRotateDrag(rendered) {
  if (state.editorMode !== 'rotate' || !rendered || !canEditItem(rendered.item)) return false;
  const controls = state.three.controls;
  const start = cloneTransform(state.dirtyTransforms.get(rendered.item.id)?.newTransform || transformFromObject(rendered.object3d));
  state.directRotateDrag = { itemId: rendered.item.id, start, last: cloneTransform(start), controlsWasEnabled: controls ? controls.enabled : true };
  if (controls) controls.enabled = false;
  return true;
}
function previewDirectRotateDrag(rendered) {
  const preview = directRotatePreviewTransform(rendered);
  if (!preview || !isFiniteTransform(preview)) return false;
  state.directRotateDrag.last = cloneTransform(preview);
  applyTransformToObject(rendered.object3d, preview);
  syncInspectorTransformFields(rendered);
  updateLabels();
  return true;
}
function finishDirectRotateDrag(rendered) {
  const drag = state.directRotateDrag;
  if (!drag) return false;
  rendered = rendered || renderedById(drag.itemId);
  const finalTransform = cloneTransform(drag.last);
  endDirectRotateDrag();
  if (!rendered || !canEditItem(rendered.item) || !isFiniteTransform(finalTransform)) return false;
  if (sameTransform(drag.start, finalTransform)) { applyTransformToObject(rendered.object3d, drag.start); syncInspectorTransformFields(rendered); return true; }
  const committed = markDirtyTransform(rendered, finalTransform, { pushHistory: true, oldTransform: drag.start, snapOptions: { translationAxes: [], rotationAxes: ['z'] } });
  if (!committed) { applyTransformToObject(rendered.object3d, drag.start); showError(`Rotation failed for ${itemLabel(rendered.item)}: final transform was rejected by the editor bridge.`); return true; }
  emitTransformCommitted(rendered);
  pushEditorEvent('status', { message: `Rotated ${itemLabel(rendered.item)}` });
  return true;
}
function endDirectRotateDrag() {
  const drag = state.directRotateDrag; state.directRotateDrag = null;
  if (state.three.controls) state.three.controls.enabled = drag ? drag.controlsWasEnabled : state.three.controls.enabled;
}
function cancelDirectRotateDrag(message) {
  const drag = state.directRotateDrag; if (!drag) return false;
  const rendered = renderedById(drag.itemId);
  if (rendered) { applyTransformToObject(rendered.object3d, drag.start); syncInspectorTransformFields(rendered); updateLabels(); }
  endDirectRotateDrag();
  pushEditorEvent('status', { message: message || 'Rotation cancelled' });
  return true;
}

function attachTransformGizmo(rendered) {
  const gizmo = state.three.transformControls;
  if (!gizmo) return;
  if (rendered && canEditItem(rendered.item)) {
    gizmo.attach(rendered.object3d);
    gizmo.visible = true;
    gizmo.enabled = true;
    if (state.editorMode === 'rotate') { gizmo.setMode('rotate'); gizmo.setSpace('world'); gizmo.showX = false; gizmo.showY = false; gizmo.showZ = true; }
    else { gizmo.setMode('translate'); gizmo.setSpace('world'); gizmo.showX = true; gizmo.showY = true; gizmo.showZ = true; }
    gizmo.enabled = state.editorMode !== 'select';
    gizmo.setTranslationSnap(el.snapToggle?.checked ? translationSnapValue() : null);
    gizmo.setRotationSnap(el.snapToggle?.checked ? rotationSnapRadians() : null);
  } else detachTransformGizmo();
}
function detachTransformGizmo() {
  const gizmo = state.three.transformControls;
  if (!gizmo) return;
  gizmo.detach();
  gizmo.visible = false;
  gizmo.enabled = false;
}
function refreshGizmoSnap() {
  const gizmo = state.three.transformControls;
  if (!gizmo) return;
  gizmo.setTranslationSnap(el.snapToggle?.checked ? translationSnapValue() : null);
  gizmo.setRotationSnap(el.snapToggle?.checked ? rotationSnapRadians() : null);
}
function canEditItem(item) {
  const sourceIdentity = [item?.source_kind, item?.source_layer, item?.active_visual_source, item?.role, item?.category]
    .map(value => String(value || '').toLowerCase())
    .join(' ');
  const source = sourceIdentity;
  if (item?.locked || item?.editable !== true || source.includes('generated')) return false;
  return true;
}
function currentTransformFromInputs(container) {
  const get = name => Number(container.querySelector(`[data-transform-field="${name}"]`)?.value);
  return { pose: { xyz: { x: get('x'), y: get('y'), z: get('z') }, rpy: { x: get('roll'), y: get('pitch'), z: get('yaw') } }, scale: { x: get('scale_x'), y: get('scale_y'), z: get('scale_z') } };
}
function renderTransformInputs(rendered) {
  const item = rendered.item;
  const editable = canEditItem(item);
  const transform = state.dirtyTransforms.get(item.id)?.newTransform || transformOf(item);
  const fields = [
    ['x', 'X', transform.pose.xyz.x], ['y', 'Y', transform.pose.xyz.y], ['z', 'Z', transform.pose.xyz.z],
    ['roll', 'Roll', transform.pose.rpy.x], ['pitch', 'Pitch', transform.pose.rpy.y], ['yaw', 'Yaw', transform.pose.rpy.z],
    ['scale_x', 'Scale X', transform.scale.x], ['scale_y', 'Scale Y', transform.scale.y], ['scale_z', 'Scale Z', transform.scale.z],
  ];
  const disabled = editable ? '' : 'disabled';
  return `<section class="transform-editor"><h3>Preview transform editing</h3>${editable ? '<p class="edit-note edit-mode-active">Edit mode active for editable/unlocked item. Drag the translation gizmo or use numeric XYZ/RPY/scale fields; browser preview only. Export Edit Patch to save a JSON patch; source YAML is not modified.</p>' : `<p class="edit-lock-reason">${LOCKED_EDIT_REASON}</p>`}<div class="transform-grid">${fields.map(([name, label, value]) => `<label>${label}<input type="number" step="0.001" data-transform-field="${name}" value="${Number(value).toFixed(6)}" ${disabled}></label>`).join('')}</div><div class="editor-actions"><button id="reset-selected" type="button" ${editable ? '' : 'disabled'}>Reset Selected</button></div></section>`;
}
function syncInspectorTransformFields(rendered) {
  if (state.selected !== rendered?.item?.id) return;
  const editor = el.inspector.querySelector('.transform-editor');
  if (!editor) return;
  const transform = state.dirtyTransforms.get(rendered.item.id)?.newTransform || transformFromObject(rendered.object3d);
  const values = { x: transform.pose.xyz.x, y: transform.pose.xyz.y, z: transform.pose.xyz.z, roll: transform.pose.rpy.x, pitch: transform.pose.rpy.y, yaw: transform.pose.rpy.z, scale_x: transform.scale.x, scale_y: transform.scale.y, scale_z: transform.scale.z };
  for (const [name, value] of Object.entries(values)) { const input = editor.querySelector(`[data-transform-field="${name}"]`); if (input) input.value = Number(value).toFixed(6); }
}
function wireTransformInputs(rendered) {
  const editor = el.inspector.querySelector('.transform-editor');
  if (!editor) return;
  editor.querySelectorAll('[data-transform-field]').forEach(input => input.addEventListener('input', () => {
    if (!canEditItem(rendered.item)) return;
    const next = currentTransformFromInputs(editor);
    if (Object.values(next.pose.xyz).concat(Object.values(next.pose.rpy), Object.values(next.scale)).some(v => !Number.isFinite(v))) return;
    markDirtyTransform(rendered, next);
  }));
  const reset = el.inspector.querySelector('#reset-selected');
  if (reset) reset.addEventListener('click', () => resetSelectedTransform(rendered.item.id));
}
function resetSelectedTransform(id = state.selected) {
  const rendered = state.objects.find(obj => obj.item.id === id);
  if (!rendered || !canEditItem(rendered.item)) return;
  markDirtyTransform(rendered, rendered.originalTransform);
  populateInspector(rendered);
  updateLabels();
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
  const rendered = renderedById(entry.itemId);
  if (!rendered || !canEditItem(rendered.item)) return;
  const target = direction === 'undo' ? entry.before : entry.after;
  markDirtyTransform(rendered, target, { pushHistory: false });
  if (state.selected === rendered.item.id) populateInspector(rendered);
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
  try {
    const text = await file.text();
    let json;
    try { json = JSON.parse(text); } catch (err) { throw new Error(`Invalid JSON in ${file.name}: ${err.message}`); }
    emitWeb3dReadinessState('server_ready');
    const items = validateSceneJson(json);
    state.sceneJson = json;
    state.sourceWebSceneFile = file.name || '';
    beginInitialCameraFitForCurrentScene();
    state.runtimeWarnings = [];
    state.dirtyTransforms.clear();
    state.undoStack = [];
    state.redoStack = [];
    state.selected = null;
    cancelDirectMoveDrag('Move cancelled');
    cancelDirectRotateDrag('Rotation cancelled');
    detachTransformGizmo();
    el.empty.hidden = true;
    if (items.length) renderScene(items);
    else renderScene([]);
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
  if (stagedHttpUrl) return meshUriDiagnostic({ ...item, mesh_uri: stagedHttpUrl });
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
  try {
    const response = await fetch(repoRootRelativeUrl(sceneUrl), { cache: 'no-store' });
    if (!response.ok) throw new Error(`HTTP ${response.status} ${response.statusText}`);
    const json = await response.json();
    emitWeb3dReadinessState('server_ready');
    const items = validateSceneJson(json);
    state.sceneJson = json;
    state.frameLookup = parseSceneFrames(json);
    state.resolvedFramePoses.clear();
    state.sourceWebSceneFile = sceneUrl;
    beginInitialCameraFitForCurrentScene();
    state.runtimeWarnings = [];
    state.dirtyTransforms.clear();
    state.undoStack = [];
    state.redoStack = [];
    state.selected = null;
    cancelDirectMoveDrag('Move cancelled');
    cancelDirectRotateDrag('Rotation cancelled');
    detachTransformGizmo();
    el.empty.hidden = true;
    renderScene(items);
    refreshWarnings(json);
    renderSceneSummary();
    el.inspector.className = 'state empty';
    el.inspector.textContent = items.length ? 'Select an object from the list or canvas.' : EMPTY_SCENE_MESSAGE;
  } catch (err) {
    showError(`Failed to load scene from ${sceneUrl}: ${err.message || err}`);
  }
}

if (el.resetView) el.resetView.addEventListener('click', resetView);
if (el.labelsToggle) el.labelsToggle.addEventListener('change', event => setLabelsVisible(event.target.checked));
if (el.debugOverlaysToggle) el.debugOverlaysToggle.addEventListener('change', event => setDebugOverlaysVisible(event.target.checked));
if (el.showInitialPose) el.showInitialPose.addEventListener('change', event => toggleInitialPosePreview(event.target.checked));
if (el.undoEdit) el.undoEdit.addEventListener('click', undoPreviewEdit);
if (el.redoEdit) el.redoEdit.addEventListener('click', redoPreviewEdit);
if (el.clearEdits) el.clearEdits.addEventListener('click', clearPreviewEdits);
if (el.snapToggle) el.snapToggle.addEventListener('change', refreshGizmoSnap);
if (el.translationSnap) el.translationSnap.addEventListener('input', refreshGizmoSnap);
if (el.rotationSnap) el.rotationSnap.addEventListener('input', refreshGizmoSnap);
if (el.exportEditPatch) el.exportEditPatch.addEventListener('click', exportEditPatch);

function setEditorMode(mode) {
  const normalized = mode === 'move' ? 'move' : (mode === 'rotate' ? 'rotate' : 'select');
  if (state.editorMode !== normalized) { cancelDirectMoveDrag('Move cancelled'); cancelDirectRotateDrag('Rotation cancelled'); }
  state.editorMode = normalized;
  const gizmo = state.three.transformControls;
  if (gizmo) {
    if (normalized === 'move') { gizmo.setMode('translate'); gizmo.setSpace('world'); gizmo.showX = true; gizmo.showY = true; gizmo.showZ = true; gizmo.enabled = true; }
    else if (normalized === 'rotate') { gizmo.setMode('rotate'); gizmo.setSpace('world'); gizmo.showX = false; gizmo.showY = false; gizmo.showZ = true; gizmo.enabled = true; }
    else { gizmo.enabled = false; }
  }
}
function setEditorSnap(enabled, translationMeters, rotationDegrees) {
  if (el.snapToggle) el.snapToggle.checked = Boolean(enabled);
  if (el.translationSnap && Number.isFinite(Number(translationMeters))) el.translationSnap.value = String(Number(translationMeters));
  if (el.rotationSnap && Number.isFinite(Number(rotationDegrees))) el.rotationSnap.value = String(Number(rotationDegrees));
  refreshGizmoSnap();
}
window.__WORKCELL_EDITOR_API_V1__ = {
  getState: () => editorState(),
  selectItem: id => { selectObject(String(id || '')); return editorState(); },
  clearSelection: () => { clearSelection(); return editorState(); },
  setMode: mode => { setEditorMode(mode); return editorState(); },
  setSnap: (enabled, translationMeters, rotationDegrees) => { setEditorSnap(enabled, translationMeters, rotationDegrees); return editorState(); },
  undo: () => { undoPreviewEdit(); return editorState(); },
  redo: () => { redoPreviewEdit(); return editorState(); },
  fitScene: () => { resetView(); return editorState(); },
  fitSelection: () => { fitSelection(); return editorState(); },
  getEditPatch: () => buildEditPatch(),
  drainEvents: () => { const events = state.editorEvents.slice(); state.editorEvents.length = 0; return events; },
};

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
    const sceneParam = params.get('scene');
    if (sceneParam) await loadSceneUrl(sceneParam);
  } catch (err) {
    showError(`Bundled Three.js module load failure: ${err.message || err}`);
  }
}

boot();
