# Viewer target functions

## emitWeb3dReadinessState
```js
function emitWeb3dReadinessState(readinessState, detail = {}
```

## structuredWeb3dReadinessFields
```js
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
```

## failedCanvasPickDiagnostic
```js
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
```

## collectRenderedMeshDiagnostics
```js
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
```

## collectAssemblyRenderDiagnostics
```js
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
```

## expandedUrdfVisualReadinessDiagnostics
```js
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
```

## updateViewerStatus
```js
function updateViewerStatus() {
  const summary = computeSceneSummary();
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
  return window.__WORKCELL_VIEWER_STATUS__;
}
```

## showError
```js
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
}
```

# Viewer alias contexts
```text
68:   if (terminalTransition) {
69:     state.web3dReadiness.terminal = true;
70:     state.web3dReadiness.terminalState = readinessState;
71:     state.web3dReadiness.terminalNavigationKey = navigationKey;
72:     state.web3dReadiness.terminalEmissionCount = Number(state.web3dReadiness.terminalEmissionCount || 0) + 1;
73:   }
74:   const structured = structuredWeb3dReadinessFields(readinessState);
75:   const eventDetail = { ...structured, ...detail, final_lifecycle_state: readinessState, finalLifecycleState: readinessState, pending_required_loads: pendingRequiredLoads() };
76:   if (readinessState === 'scene_failed') {
77:     state.web3dReadiness.failed = true;
78:     state.web3dReadiness.failure = eventDetail;
79:   }
80:   const status = updateViewerStatus();
81:   if (readinessState === 'scene_ready') triggerInitialCameraFitAfterSceneReady();
82:   window.dispatchEvent?.(new CustomEvent('workcell:web3d_readiness', { detail: { state: readinessState, ...eventDetail, status } }));

---

157:     if (authoredId) return authoredId;
158:     const relationshipId = configuredCameraRelationshipIds(item)[0];
159:     if (relationshipId) return relationshipId;
160:   }
161:   return String(item?.id || item?.link || itemLabel(item || {}));
162: }
163: function readinessKey(category, item) { return `${category}:${readinessIdentityForItem(category, item)}`; }
164: function pendingRequiredLoads() { return Array.from(state.web3dReadiness?.pending || []); }
165: 
166: function physicalReadinessItems() {
167:   return collectItems(state.sceneJson || {}).filter(item => isPrimaryRenderableItem(item) && !isDebugOverlayItem(item) && readinessCategoryForItem(item));
168: }
169: function renderedPhysicalItemCount() {
170:   const assemblyCount = collectPhysicalAssemblyBounds?.()?.count || 0;
171:   return statusCountedRenderables().length + assemblyCount;

---

178: function failedRequiredItemCount() {
179:   const renderFailures = statusCountedRenderables().filter(isRequiredMeshFailureStatus).length;
180:   return renderFailures + (state.web3dReadiness?.state === 'scene_failed' && renderFailures === 0 ? 1 : 0);
181: }
182: function readinessCategoryStatus(category) {
183:   const readiness = state.web3dReadiness || {};
184:   if (!readiness.required?.[category]) return 'missing';
185:   const pending = pendingRequiredLoads().some(key => String(key).startsWith(`${category}:`));
186:   if (readiness.state === 'scene_failed' && (readiness.failure?.required_category === category || pending)) return 'failed';
187:   if (pending) return 'pending';
188:   return readiness.state === 'scene_ready' ? 'ready' : 'loading';
189: }
190: function structuredWeb3dReadinessFields(lifecycleState) {
191:   const finalState = lifecycleState || state.web3dReadiness?.state || 'booting';
192:   const pendingLoads = pendingRequiredLoads();

---

185:   const pending = pendingRequiredLoads().some(key => String(key).startsWith(`${category}:`));
186:   if (readiness.state === 'scene_failed' && (readiness.failure?.required_category === category || pending)) return 'failed';
187:   if (pending) return 'pending';
188:   return readiness.state === 'scene_ready' ? 'ready' : 'loading';
189: }
190: function structuredWeb3dReadinessFields(lifecycleState) {
191:   const finalState = lifecycleState || state.web3dReadiness?.state || 'booting';
192:   const pendingLoads = pendingRequiredLoads();
193:   return {
194:     scene_id: sceneId(),
195:     sceneId: sceneId(),
196:     expected_physical_item_count: expectedPhysicalItemCount(),
197:     expectedPhysicalItemCount: expectedPhysicalItemCount(),
198:     rendered_physical_item_count: renderedPhysicalItemCount(),
199:     renderedPhysicalItemCount: renderedPhysicalItemCount(),

---

206:     end_effector_status: readinessCategoryStatus('attached_tool_gripper'),
207:     endEffectorStatus: readinessCategoryStatus('attached_tool_gripper'),
208:     environment_status: readinessCategoryStatus('workbench_support_surface'),
209:     environmentStatus: readinessCategoryStatus('workbench_support_surface'),
210:     camera_status: readinessCategoryStatus('configured_camera'),
211:     cameraStatus: readinessCategoryStatus('configured_camera'),
212:     pending_required_loads: pendingLoads,
213:     pendingRequiredLoads: pendingLoads,
214:     readiness_contract_version: READINESS_CONTRACT_VERSION,
215:     readinessContractVersion: READINESS_CONTRACT_VERSION,
216:     lifecycle_state: finalState,
217:     lifecycleState: finalState,
218:     terminal: Boolean(state.web3dReadiness?.terminal),
219:     status_sequence: Number(state.web3dReadiness?.statusSequence || 0),
220:     statusSequence: Number(state.web3dReadiness?.statusSequence || 0),

---

219:     status_sequence: Number(state.web3dReadiness?.statusSequence || 0),
220:     statusSequence: Number(state.web3dReadiness?.statusSequence || 0),
221:     source_web_scene_file: state.sourceWebSceneFile || '',
222:     sourceWebSceneFile: state.sourceWebSceneFile || '',
223:     builder_revision: state.builderRevision || '',
224:     builderRevision: state.builderRevision || '',
225:     final_lifecycle_state: finalState,
226:     finalLifecycleState: finalState,
227:   };
228: }
229: function beginWeb3dSceneReadiness(items) {
230:   const required = Object.fromEntries(WEB3D_REQUIRED_CATEGORIES.map(category => [category, false]));
231:   for (const item of items || []) {
232:     const category = readinessCategoryForItem(item);
233:     if (category) required[category] = true;

---

264:   return true;
265: }
266: function readinessOperationDiagnostic(operation, outcome, extra = {}) {
267:   return {
268:     operation_id: operation?.id || 'unknown_required_load',
269:     operation_pending_keys: Array.from(operation?.pendingKeys || []),
270:     operation_outcome: outcome,
271:     pending_required_loads: pendingRequiredLoads(),
272:     ...extra,
273:   };
274: }
275: function requiredReadinessCompleteForItem(item) {
276:   const category = readinessCategoryForItem(item);
277:   if (!category) return;
278:   state.web3dReadiness?.pending?.delete(readinessKey(category, item));

---

307:     item_id: item?.id || '',
308:     link: item?.link || item?.link_name || item?.object_name || '',
309:     url: url || displayMeshUri(item),
310:     reason: reason || 'required mesh failed',
311:     ...extra,
312:     loader: extra.loader || extra.loader_type || '',
313:     loader_type: extra.loader_type || extra.loader || '',
314:     pending_required_loads: pendingRequiredLoads(),
315:   });
316:   return true;
317: }
318: function physicalMeshBoundsFailurePayload(item, loadUrl, loader) {
319:   const expected = expectedDimensionsOf(item);
320:   const localBounds = item?.loaded_mesh_local_bounds || item?.loaded_mesh_bounds || null;
321:   const worldBounds = item?.loaded_mesh_world_bounds || null;

---

351:     readiness_identity: readinessIdentity,
352:     readiness_key: `${category}:${readinessIdentity}`,
353:     item_id: item?.id || '',
354:     link: item?.link || item?.link_name || item?.object_name || '',
355:     url: url || displayMeshUri(item),
356:     reason: reason || 'required mesh failed',
357:     ...extra,
358:     pending_required_loads: pendingRequiredLoads(),
359:   });
360: }
361: function completeExpandedUrdfReadiness(operation) { return completeReadinessOperation(operation); }
362: function failExpandedUrdfReadiness(operation, err, diagnostics = {}, detail = {}) {
363:   if (!readinessOperationIsCurrent(operation) || operation.completed) return false;
364:   const link = String(detail.link || detail.link_name || '').trim();
365:   const expectedTool = new Set(asArray(state.sceneJson?.robot_preview?.expected_tool_visual_links || state.sceneJson?.robot_preview?.expectedToolVisualLinks).map(value => String(value || '').trim()).filter(Boolean));

---

365:   const expectedTool = new Set(asArray(state.sceneJson?.robot_preview?.expected_tool_visual_links || state.sceneJson?.robot_preview?.expectedToolVisualLinks).map(value => String(value || '').trim()).filter(Boolean));
366:   emitWeb3dReadinessState('scene_failed', {
367:     required_category: expectedTool.has(link) ? 'attached_tool_gripper' : 'robot_arm',
368:     item_id: 'expanded_urdf_loader',
369:     link,
370:     url: detail.url || detail.uri || diagnostics.robot_urdf_url || '',
371:     reason: err?.message || String(err || 'expanded URDF required mesh failed'),
372:     robot_preview_lifecycle_state: diagnostics.robot_preview_lifecycle_state || diagnostics.robotPreviewLifecycleState || '',
373:     robot_loaded_visual_count: Number(diagnostics.robot_loaded_visual_count ?? diagnostics.robotLoadedVisualCount ?? 0) || 0,
374:     robot_expected_visual_count: Number(diagnostics.robot_expected_visual_count ?? diagnostics.robotExpectedVisualCount ?? 0) || 0,
375:     robot_failed_visual_count: Number(diagnostics.robot_failed_visual_count ?? diagnostics.robotFailedVisualCount ?? 0) || 0,
376:     robot_missing_meshes: diagnostics.robot_missing_meshes || [],
377:     ...detail,
378:     pending_required_loads: pendingRequiredLoads(),
379:   });

---

371:     reason: err?.message || String(err || 'expanded URDF required mesh failed'),
372:     robot_preview_lifecycle_state: diagnostics.robot_preview_lifecycle_state || diagnostics.robotPreviewLifecycleState || '',
373:     robot_loaded_visual_count: Number(diagnostics.robot_loaded_visual_count ?? diagnostics.robotLoadedVisualCount ?? 0) || 0,
374:     robot_expected_visual_count: Number(diagnostics.robot_expected_visual_count ?? diagnostics.robotExpectedVisualCount ?? 0) || 0,
375:     robot_failed_visual_count: Number(diagnostics.robot_failed_visual_count ?? diagnostics.robotFailedVisualCount ?? 0) || 0,
376:     robot_missing_meshes: diagnostics.robot_missing_meshes || [],
377:     ...detail,
378:     pending_required_loads: pendingRequiredLoads(),
379:   });
380:   return true;
381: }
382: function expandedUrdfTerminalFailure(rendererDiagnostics = {}) {
383:   const list = (snakeName, camelName) => asArray(rendererDiagnostics[snakeName] || rendererDiagnostics[camelName])
384:     .map(value => String(value || '').trim()).filter(Boolean);
385:   const lifecycle = String(rendererDiagnostics.robot_preview_lifecycle_state || rendererDiagnostics.robotPreviewLifecycleState || '').trim();

---

378:     pending_required_loads: pendingRequiredLoads(),
379:   });
380:   return true;
381: }
382: function expandedUrdfTerminalFailure(rendererDiagnostics = {}) {
383:   const list = (snakeName, camelName) => asArray(rendererDiagnostics[snakeName] || rendererDiagnostics[camelName])
384:     .map(value => String(value || '').trim()).filter(Boolean);
385:   const lifecycle = String(rendererDiagnostics.robot_preview_lifecycle_state || rendererDiagnostics.robotPreviewLifecycleState || '').trim();
386:   const previewLoaded = rendererDiagnostics.robot_preview_loaded === true || rendererDiagnostics.robotPreviewLoaded === true;
387:   const previewExplicitlyNotLoaded = rendererDiagnostics.robot_preview_loaded === false || rendererDiagnostics.robotPreviewLoaded === false;
388:   const expectedVisualCount = Number(rendererDiagnostics.robot_expected_visual_count ?? rendererDiagnostics.robotExpectedVisualCount ?? 0) || 0;
389:   const loadedVisualCount = Number(rendererDiagnostics.robot_loaded_visual_count ?? rendererDiagnostics.robotLoadedVisualCount ?? 0) || 0;
390:   const completedVisualCount = Number(rendererDiagnostics.robot_completed_visual_count ?? rendererDiagnostics.robotCompletedVisualCount ?? 0) || 0;
391:   const failedVisualCount = Number(rendererDiagnostics.robot_failed_visual_count ?? rendererDiagnostics.robotFailedVisualCount ?? 0) || 0;
392:   const missingToolLinks = list('robot_missing_required_tool_visual_links', 'robotMissingRequiredToolVisualLinks');

---

409:     requiredCategory = 'robot_arm';
410:     reason = `Expanded URDF robot hierarchy is missing required links: ${missingHierarchyLinks.join(', ')}. Check the generated URDF link/joint hierarchy and regenerate the scene.`;
411:   } else if (meshCallbacksExplicitlyIncomplete) {
412:     requiredCategory = 'robot_arm';
413:     reason = `Expanded URDF mesh callbacks did not complete (${completedVisualCount}/${expectedVisualCount} completed). Check failed mesh requests and the browser console, then regenerate or reload the scene.`;
414:   } else if (lifecycle === 'failed' && previewExplicitlyNotLoaded) {
415:     requiredCategory = 'robot_arm';
416:     reason = String(rendererDiagnostics.robot_preview_failure_reason || rendererDiagnostics.robotPreviewFailureReason || '').trim()
417:       || 'Expanded URDF preview entered the failed lifecycle without loading. Check the URDF and mesh request diagnostics, then regenerate or reload the scene.';
418:   } else {
419:     return null;
420:   }
421: 
422:   return {
423:     required_category: requiredCategory,

---

437:     robot_mesh_callbacks_complete: meshCallbacksComplete,
438:     robot_mesh_callback_completion_state: meshCallbacksComplete ? 'complete' : (meshCallbacksExplicitlyIncomplete ? 'incomplete' : 'unknown'),
439:   };
440: }
441: function maybeEmitSceneReady() {
442:   if (isExpandedUrdfRobotPreview(state.sceneJson?.robot_preview)) {
443:     const diagnostics = state.robotUrdfPreviewDiagnostics || {};
444:     const lifecycle = String(diagnostics.robot_preview_lifecycle_state || diagnostics.robotPreviewLifecycleState || '');
445:     const terminalFailure = expandedUrdfTerminalFailure(diagnostics);
446:     if (terminalFailure) {
447:       emitWeb3dReadinessState('scene_failed', terminalFailure);
448:       return;
449:     }
450:     if (lifecycle === 'failed') {
451:       const failureReason = String(diagnostics.robot_preview_failure_reason || diagnostics.robotPreviewFailureReason || '').trim();

---

444:     const lifecycle = String(diagnostics.robot_preview_lifecycle_state || diagnostics.robotPreviewLifecycleState || '');
445:     const terminalFailure = expandedUrdfTerminalFailure(diagnostics);
446:     if (terminalFailure) {
447:       emitWeb3dReadinessState('scene_failed', terminalFailure);
448:       return;
449:     }
450:     if (lifecycle === 'failed') {
451:       const failureReason = String(diagnostics.robot_preview_failure_reason || diagnostics.robotPreviewFailureReason || '').trim();
452:       const missingMeshes = asArray(diagnostics.robot_missing_meshes || diagnostics.robotMissingMeshes);
453:       const failedVisualCount = Number(diagnostics.robot_failed_visual_count ?? diagnostics.robotFailedVisualCount ?? 0) || 0;
454:       if (!failureReason && missingMeshes.length === 0 && failedVisualCount === 0) return;
455:       return;
456:     }
457:     if (lifecycle !== 'ready') return;
458:   }

---

786:   const rawUrdfVisuals = asArray(rendererDiagnostics.robot_visual_wrapper_world_matrices);
787:   const urdfDedupe = dedupeByStableIdentity(rawUrdfVisuals, visual => expandedUrdfVisualIdentity(sceneId, robotInstanceId, visual));
788:   const urdfVisuals = urdfDedupe.records;
789:   const urdfLinksWithLoadedVisuals = new Set(urdfVisuals.map(visual => String(visual?.link_name || visual?.linkName || '').trim()).filter(Boolean));
790:   const sceneDiagnostics = collectRenderedMeshDiagnostics();
791:   const physicalDiagnostics = dedupeByStableIdentity(sceneDiagnostics.filter(isSuccessfulPhysicalVisualDiagnostic), entry => renderedPhysicalVisualIdentity(sceneId, entry));
792:   const categoryCounts = countBy(physicalDiagnostics.records.map(item => readinessCategoryForItem(item)).filter(Boolean));
793:   const rendererLifecycle = String(rendererDiagnostics.robot_preview_lifecycle_state || rendererDiagnostics.robotPreviewLifecycleState || '');
794:   const rendererLoaded = rendererDiagnostics.robot_preview_loaded === true || rendererDiagnostics.robotPreviewLoaded === true;
795:   const rendererExpectedVisualCount = Number(rendererDiagnostics.robot_expected_visual_count ?? rendererDiagnostics.robotExpectedVisualCount ?? 0) || 0;
796:   const rendererCompletedVisualCount = Number(rendererDiagnostics.robot_completed_visual_count ?? rendererDiagnostics.robotCompletedVisualCount ?? 0) || 0;
797:   const rendererLoadedVisualCount = Number(rendererDiagnostics.robot_loaded_visual_count ?? rendererDiagnostics.robotLoadedVisualCount ?? 0) || 0;
798:   const rendererFailedVisualCount = Number(rendererDiagnostics.robot_failed_visual_count ?? rendererDiagnostics.robotFailedVisualCount ?? 0) || 0;
799:   const rendererMissingMeshes = asArray(rendererDiagnostics.robot_missing_meshes || rendererDiagnostics.robotMissingMeshes);
800:   const rendererMeshCallbacksComplete = rendererDiagnostics.robot_mesh_callbacks_complete === true || rendererDiagnostics.robotMeshCallbacksComplete === true;

---

835:     expanded_urdf_expected_visual_set: required,
836:     expandedUrdfExpectedVisualSet: required,
837:     expanded_urdf_required_visual_counts: { ...categoryCounts },
838:     expandedUrdfRequiredVisualCounts: { ...categoryCounts },
839:     expanded_urdf_loaded_visual_link_counts: countBy(urdfVisuals.map(visual => String(visual?.link_name || visual?.linkName || '').trim()).filter(Boolean)),
840:     expandedUrdfLoadedVisualLinkCounts: countBy(urdfVisuals.map(visual => String(visual?.link_name || visual?.linkName || '').trim()).filter(Boolean)),
841:     robot_preview_lifecycle_state: rendererLifecycle,
842:     robotPreviewLifecycleState: rendererLifecycle,
843:     robot_preview_loaded: rendererLoaded,
844:     robotPreviewLoaded: rendererLoaded,
845:     robot_expected_visual_count: rendererExpectedVisualCount,
846:     robotExpectedVisualCount: rendererExpectedVisualCount,
847:     robot_completed_visual_count: rendererCompletedVisualCount,
848:     robotCompletedVisualCount: rendererCompletedVisualCount,
849:     robot_loaded_visual_count: rendererLoadedVisualCount,

---

877:     failed_mesh_urls: Array.from(new Set(failedMeshUrls)),
878:     failedMeshUrls: Array.from(new Set(failedMeshUrls)),
879:     required_visual_ready: requiredVisualReady,
880:     requiredVisualReady: requiredVisualReady,
881:   };
882: }
883: function failIfExpandedUrdfExpectedVisualSetInvalid() {
884:   const lifecycle = String(state.robotUrdfPreviewDiagnostics?.robot_preview_lifecycle_state || state.robotUrdfPreviewDiagnostics?.robotPreviewLifecycleState || '');
885:   if (expandedUrdfExpectedVisualSet() && lifecycle !== 'ready' && lifecycle !== 'failed') return false;
886:   const diagnostics = expandedUrdfVisualReadinessDiagnostics();
887:   if (!diagnostics || diagnostics.required_visual_ready) return false;
888:   const requiredCategory = diagnostics.robot_missing_required_tool_visual_links?.length ? 'attached_tool_gripper' : 'robot_arm';
889:   const reason = diagnostics.robot_preview_loaded
890:     ? 'expanded URDF renderer reported required robot/tool visual failure'
891:     : 'expanded URDF expected robot/tool visuals are missing or failed';

---

1115:     robot_hierarchy_missing_parents: Array.from(new Set((state.robotAssemblyDiagnostics || []).flatMap(d => d.robot_hierarchy_missing_parents || []))),
1116:     robot_hierarchy_mesh_count: (state.robotAssemblyDiagnostics || []).reduce((total, d) => total + Number(d.robot_hierarchy_mesh_count || 0), 0),
1117:     robotHierarchyMeshCount: (state.robotAssemblyDiagnostics || []).reduce((total, d) => total + Number(d.robot_hierarchy_mesh_count || 0), 0),
1118:     ...assemblyRenderDiagnostics,
1119:     ...expandedUrdfVisualDiagnostics,
1120:     required_physical_categories: state.web3dReadiness?.required || {},
1121:     requiredPhysicalCategories: state.web3dReadiness?.required || {},
1122:     pending_required_loads: pendingRequiredLoads(),
1123:     pendingRequiredLoads: pendingRequiredLoads(),
1124:     ...structuredWeb3dReadinessFields(state.web3dReadiness?.state || 'booting'),
1125:     final_failed_url: state.web3dReadiness?.failure?.url || state.web3dReadiness?.failure?.final_failed_url || '',
1126:     finalFailedUrl: state.web3dReadiness?.failure?.url || state.web3dReadiness?.failure?.finalFailedUrl || '',
1127:     final_failed_link: state.web3dReadiness?.failure?.link || state.web3dReadiness?.failure?.link_name || '',
1128:     finalFailedLink: state.web3dReadiness?.failure?.link || state.web3dReadiness?.failure?.linkName || '',
1129:     readiness_failure: state.web3dReadiness?.failure || null,

---

1116:     robot_hierarchy_mesh_count: (state.robotAssemblyDiagnostics || []).reduce((total, d) => total + Number(d.robot_hierarchy_mesh_count || 0), 0),
1117:     robotHierarchyMeshCount: (state.robotAssemblyDiagnostics || []).reduce((total, d) => total + Number(d.robot_hierarchy_mesh_count || 0), 0),
1118:     ...assemblyRenderDiagnostics,
1119:     ...expandedUrdfVisualDiagnostics,
1120:     required_physical_categories: state.web3dReadiness?.required || {},
1121:     requiredPhysicalCategories: state.web3dReadiness?.required || {},
1122:     pending_required_loads: pendingRequiredLoads(),
1123:     pendingRequiredLoads: pendingRequiredLoads(),
1124:     ...structuredWeb3dReadinessFields(state.web3dReadiness?.state || 'booting'),
1125:     final_failed_url: state.web3dReadiness?.failure?.url || state.web3dReadiness?.failure?.final_failed_url || '',
1126:     finalFailedUrl: state.web3dReadiness?.failure?.url || state.web3dReadiness?.failure?.finalFailedUrl || '',
1127:     final_failed_link: state.web3dReadiness?.failure?.link || state.web3dReadiness?.failure?.link_name || '',
1128:     finalFailedLink: state.web3dReadiness?.failure?.link || state.web3dReadiness?.failure?.linkName || '',
1129:     readiness_failure: state.web3dReadiness?.failure || null,
1130:     readinessFailure: state.web3dReadiness?.failure || null,

---

3845:     robot_root_links: [],
3846:     robotRootLinks: [],
3847:     robot_disconnected_links: [],
3848:     robotDisconnectedLinks: [],
3849:     robot_duplicate_links: [],
3850:     robotDuplicateLinks: [],
3851:     robot_preview_lifecycle_state: 'idle',
3852:     robotPreviewLifecycleState: 'idle',
3853:     robot_preview_canonical_fallback_used: false,
3854:     robotPreviewCanonicalFallbackUsed: false,
3855:     skipped_legacy_generated_urdf_visual_count: state.robotAssemblyRenderDiagnostics?.skipped_legacy_generated_urdf_visual_count || state.robotAssemblyRenderDiagnostics?.skipped_legacy_generated_urdf_count || 0,
3856:   };
3857:   const mergeMissingDiagnosticsInPlace = (target, additions) => {
3858:     if (!target || typeof target !== 'object') return target;
3859:     for (const [key, value] of Object.entries(additions || {})) {

---

3858:     if (!target || typeof target !== 'object') return target;
3859:     for (const [key, value] of Object.entries(additions || {})) {
3860:       if (target[key] === undefined) target[key] = value;
3861:     }
3862:     return target;
3863:   };
3864:   const rendererReady = rendererDiagnostics => {
3865:     const lifecycle = String(rendererDiagnostics?.robot_preview_lifecycle_state || rendererDiagnostics?.robotPreviewLifecycleState || '');
3866:     const loaded = rendererDiagnostics?.robot_preview_loaded === true || rendererDiagnostics?.robotPreviewLoaded === true;
3867:     return lifecycle === 'ready' && loaded;
3868:   };
3869:   if (typeof loadRobotPreview !== 'function') {
3870:     diagnostics.robot_preview_lifecycle_state = 'failed';
3871:     diagnostics.robotPreviewLifecycleState = 'failed';
3872:     diagnostics.robot_failed_visual_count = 1;

---

3864:   const rendererReady = rendererDiagnostics => {
3865:     const lifecycle = String(rendererDiagnostics?.robot_preview_lifecycle_state || rendererDiagnostics?.robotPreviewLifecycleState || '');
3866:     const loaded = rendererDiagnostics?.robot_preview_loaded === true || rendererDiagnostics?.robotPreviewLoaded === true;
3867:     return lifecycle === 'ready' && loaded;
3868:   };
3869:   if (typeof loadRobotPreview !== 'function') {
3870:     diagnostics.robot_preview_lifecycle_state = 'failed';
3871:     diagnostics.robotPreviewLifecycleState = 'failed';
3872:     diagnostics.robot_failed_visual_count = 1;
3873:     diagnostics.robotFailedVisualCount = 1;
3874:     diagnostics.robot_missing_meshes.push('urdf_robot_renderer module was not loaded');
3875:     appendRuntimeWarning({}, preview?.urdf_url || '', 'expanded_urdf_loader failed: urdf_robot_renderer module was not loaded', 'expanded_urdf_loader_failed');
3876:     finalizeRequiredLoad('failure', { completion_source: 'module_preflight', reason: 'expanded_urdf_loader failed: urdf_robot_renderer module was not loaded' });
3877:     refreshWarnings();
3878:     return { root: null, links: new Map(), joints: new Map(), diagnostics, ready: Promise.resolve(null) };

---

3978:     onRobotMeshLoadError: (err, uri, detail) => { if (!callbackIsCurrent()) return ignoreStaleCallback(); finalizeRequiredLoad('failure', { ...(detail || { uri }), error: err, completion_source: 'onRobotMeshLoadError' }); renderSceneSummary(); },
3979:     onRobotError: (err, diagnostics) => {
3980:       if (!callbackIsCurrent()) return ignoreStaleCallback();
3981:       const authoritativeDiagnostics = diagnostics || previewResult.diagnostics || state.robotUrdfPreviewDiagnostics;
3982:       mergeMissingDiagnosticsInPlace(authoritativeDiagnostics, state.robotUrdfPreviewDiagnostics);
3983:       state.robotUrdfPreviewDiagnostics = authoritativeDiagnostics;
3984:       state.robotUrdfPreviewDiagnostics.robot_preview_lifecycle_state = 'failed';
3985:       state.robotUrdfPreviewDiagnostics.robotPreviewLifecycleState = 'failed';
3986:       state.robotUrdfPreviewDiagnostics.robot_preview_failure_reason = err?.message || String(err || 'expanded URDF preview failed');
3987:       state.robotUrdfPreviewDiagnostics.robotPreviewFailureReason = state.robotUrdfPreviewDiagnostics.robot_preview_failure_reason;
3988:       finalizeRequiredLoad('failure', { error: err, completion_source: 'onRobotError' });
3989:       maybeEmitSceneReady();
3990:       appendRuntimeWarning({}, preview?.urdf_url || '', `expanded_urdf_loader failed: ${err?.message || err}`, 'expanded_urdf_loader_failed');
3991:       refreshWarnings();
3992:       renderSceneSummary();

---

3980:       if (!callbackIsCurrent()) return ignoreStaleCallback();
3981:       const authoritativeDiagnostics = diagnostics || previewResult.diagnostics || state.robotUrdfPreviewDiagnostics;
3982:       mergeMissingDiagnosticsInPlace(authoritativeDiagnostics, state.robotUrdfPreviewDiagnostics);
3983:       state.robotUrdfPreviewDiagnostics = authoritativeDiagnostics;
3984:       state.robotUrdfPreviewDiagnostics.robot_preview_lifecycle_state = 'failed';
3985:       state.robotUrdfPreviewDiagnostics.robotPreviewLifecycleState = 'failed';
3986:       state.robotUrdfPreviewDiagnostics.robot_preview_failure_reason = err?.message || String(err || 'expanded URDF preview failed');
3987:       state.robotUrdfPreviewDiagnostics.robotPreviewFailureReason = state.robotUrdfPreviewDiagnostics.robot_preview_failure_reason;
3988:       finalizeRequiredLoad('failure', { error: err, completion_source: 'onRobotError' });
3989:       maybeEmitSceneReady();
3990:       appendRuntimeWarning({}, preview?.urdf_url || '', `expanded_urdf_loader failed: ${err?.message || err}`, 'expanded_urdf_loader_failed');
3991:       refreshWarnings();
3992:       renderSceneSummary();
3993:     },
3994:   });

---

4012:         state.robotUrdfPreviewDiagnostics = terminalDiagnostics;
4013:       }
4014:       if (rendererReady(state.robotUrdfPreviewDiagnostics) && !failIfExpandedUrdfExpectedVisualSetInvalid()) {
4015:         finalizeRequiredLoad('success', { completion_source: 'previewResult.ready' });
4016:       } else {
4017:         finalizeRequiredLoad('failure', {
4018:           completion_source: 'previewResult.ready',
4019:           reason: String(state.robotUrdfPreviewDiagnostics.robot_preview_failure_reason || state.robotUrdfPreviewDiagnostics.robotPreviewFailureReason || 'Expanded URDF previewResult.ready resolved without a ready preview. Check required robot/tool visual diagnostics.'),
4020:         });
4021:       }
4022:       renderSceneSummary();
4023:     }, err => finalizeRequiredLoad('failure', { error: err, completion_source: 'previewResult.ready_rejection' }));
4024:   }
4025:   return previewResult;
4026: }

---

4559:     return Math.abs(distanceDelta) <= PICK_COINCIDENCE_TOLERANCE_M ? a.priority - b.priority || distanceDelta : distanceDelta;
4560:   });
4561: }
4562: function failedCanvasPickDiagnostic(hits) {
4563:   const objectNames = [];
4564:   let traversedRegisteredIdentity = false;
4565:   let firstActionableRejectionReason = '';
4566:   const hitResolutions = [];
4567:   let nearestKnownUrdfLinkAncestor = '';
4568:   const knownLinks = state.robotPreviewResult?.links instanceof Map ? state.robotPreviewResult.links : new Map();
4569:   const knownLinkByNode = new Map(Array.from(knownLinks.entries()).map(([name, node]) => [node, String(name || '')]));
4570:   for (const hit of hits || []) {
4571:     const resolution = resolveCanvasPickHit(hit);
4572:     const rawName = String(hit?.object?.name || '').trim();
4573:     if (rawName && !objectNames.includes(rawName) && objectNames.length < 12) objectNames.push(rawName);

---

4575:     while (node) {
4576:       const registered = state.pickIdentityByObject.get(node);
4577:       if (registered) traversedRegisteredIdentity = true;
4578:       if (!nearestKnownUrdfLinkAncestor) nearestKnownUrdfLinkAncestor = knownLinkByNode.get(node) || String(registered?.item?.link_name || registered?.item?.link || '').trim();
4579:       node = node.parent;
4580:     }
4581:     if (!firstActionableRejectionReason && resolution.rejectionReason) firstActionableRejectionReason = resolution.rejectionReason;
4582:     hitResolutions.push({
4583:       hit_node_name: rawName,
4584:       registered_record_id: resolution.registeredRecord?.item?.id || '',
4585:       pick_source: resolution.registeredRecord?.pickRecordSource || '',
4586:       authoritative_physical_pick: resolution.registeredRecord?.authoritativePhysicalPick === true,
4587:       selection_owner_id: resolution.selectionOwner?.item?.id || '',
4588:       selection_owner_source: resolution.selectionOwnerSource || '',
4589:       edit_owner_id: resolution.editOwner?.item?.id || '',

---

4592:       exclusion_flag: resolution.exclusionFlag || '',
4593:       candidate_priority: Number.isFinite(pickingPriority(resolution.editOwner || resolution.selectionOwner || resolution.renderIdentity)) ? pickingPriority(resolution.editOwner || resolution.selectionOwner || resolution.renderIdentity) : 'Infinity',
4594:     });
4595:   }
4596:   if (!firstActionableRejectionReason) firstActionableRejectionReason = traversedRegisteredIdentity ? 'registered_identity_rejected_by_selection_policy' : 'no_registered_identity_in_hit_ancestry';
4597:   return {
4598:     raw_hit_count: (hits || []).length,
4599:     rawHitCount: (hits || []).length,
4600:     hit_object_names: objectNames,
4601:     hitObjectNames: objectNames,
4602:     traversed_registered_identity: traversedRegisteredIdentity,
4603:     traversedRegisteredIdentity,
4604:     first_actionable_rejection_reason: firstActionableRejectionReason,
4605:     firstActionableRejectionReason,
4606:     nearest_known_urdf_link_ancestor: nearestKnownUrdfLinkAncestor,

---

4601:     hitObjectNames: objectNames,
4602:     traversed_registered_identity: traversedRegisteredIdentity,
4603:     traversedRegisteredIdentity,
4604:     first_actionable_rejection_reason: firstActionableRejectionReason,
4605:     firstActionableRejectionReason,
4606:     nearest_known_urdf_link_ancestor: nearestKnownUrdfLinkAncestor,
4607:     nearestKnownUrdfLinkAncestor,
4608:     hit_resolutions: hitResolutions,
4609:     hitResolutions,
4610:   };
4611: }
4612: function selectObject(id) { return selectObjectFromRender(id, null); }
4613: function selectObjectFromRender(id, renderIdentity = null) {
4614:   const requestedId = String(id || '');
4615:   const rawRequested = requestedId ? renderedById(requestedId) : null;

---

4602:     traversed_registered_identity: traversedRegisteredIdentity,
4603:     traversedRegisteredIdentity,
4604:     first_actionable_rejection_reason: firstActionableRejectionReason,
4605:     firstActionableRejectionReason,
4606:     nearest_known_urdf_link_ancestor: nearestKnownUrdfLinkAncestor,
4607:     nearestKnownUrdfLinkAncestor,
4608:     hit_resolutions: hitResolutions,
4609:     hitResolutions,
4610:   };
4611: }
4612: function selectObject(id) { return selectObjectFromRender(id, null); }
4613: function selectObjectFromRender(id, renderIdentity = null) {
4614:   const requestedId = String(id || '');
4615:   const rawRequested = requestedId ? renderedById(requestedId) : null;
4616:   const requested = inspectionSelectionRendered(renderIdentity || rawRequested);
```

# Renderer alias contexts
```text
7: const ROBOT_RENDER_MODE = 'expanded_urdf_loader';
8: 
9: const COLLADA_Z_UP_CONSOLE_MESSAGE = 'THREE.ColladaLoader: You are loading an asset with a Z-UP coordinate system. The loader just rotates the asset to transform it into Y-UP. The vertex data are not converted, see #24289.';
10: 
11: function loadColladaWithSceneScopedZUpDiagnostic(loader, url, onLoad, onProgress, onError, diagnostics) {
12:   const originalWarn = console.warn;
13:   console.warn = (...args) => {
14:     if (String(args?.[0] || '') === COLLADA_Z_UP_CONSOLE_MESSAGE) {
15:       diagnostics.collada_z_up_console_message_count = (diagnostics.collada_z_up_console_message_count || 0) + 1;
16:       diagnostics.colladaZUpConsoleMessageCount = diagnostics.collada_z_up_console_message_count;
17:       if (!diagnostics.collada_z_up_console_message_emitted) {
18:         diagnostics.collada_z_up_console_message_emitted = true;
19:         console.info(`Collada Z-UP loader notice collapsed for scene load: ${COLLADA_Z_UP_CONSOLE_MESSAGE}`);
20:       }
21:       return;
22:     }
23:     originalWarn.apply(console, args);
24:   };
25:   loader.load(url, dae => { console.warn = originalWarn; onLoad(dae); }, onProgress, err => { console.warn = originalWarn; onError(err); });

---

42: 
43: function safeDecodeUriSegment(segment) {
44:   try {
45:     return decodeURIComponent(segment);
46:   } catch (_) {
47:     return null;
48:   }
49: }
50: 
51: function rejectPackageMeshUri(reason, diagnostics, sourceUrl = '') {
52:   const detail = sourceUrl ? `${sourceUrl}: ${reason}` : reason;
53:   diagnostics.robot_missing_meshes.push(`URDF package mesh rejected: ${detail}`);
54:   diagnostics.robot_package_mesh_rejections = diagnostics.robot_package_mesh_rejections || [];
55:   diagnostics.robot_package_mesh_rejections.push({ source_url: sourceUrl || '', sourceUrl: sourceUrl || '', policy_reason: reason, policyReason: reason });
56:   return '';
57: }
58: 
59: 
60: function safeSceneAssetId(value) {

---

43: function safeDecodeUriSegment(segment) {
44:   try {
45:     return decodeURIComponent(segment);
46:   } catch (_) {
47:     return null;
48:   }
49: }
50: 
51: function rejectPackageMeshUri(reason, diagnostics, sourceUrl = '') {
52:   const detail = sourceUrl ? `${sourceUrl}: ${reason}` : reason;
53:   diagnostics.robot_missing_meshes.push(`URDF package mesh rejected: ${detail}`);
54:   diagnostics.robot_package_mesh_rejections = diagnostics.robot_package_mesh_rejections || [];
55:   diagnostics.robot_package_mesh_rejections.push({ source_url: sourceUrl || '', sourceUrl: sourceUrl || '', policy_reason: reason, policyReason: reason });
56:   return '';
57: }
58: 
59: 
60: function safeSceneAssetId(value) {
61:   const text = String(value || '').trim();

---

46:   } catch (_) {
47:     return null;
48:   }
49: }
50: 
51: function rejectPackageMeshUri(reason, diagnostics, sourceUrl = '') {
52:   const detail = sourceUrl ? `${sourceUrl}: ${reason}` : reason;
53:   diagnostics.robot_missing_meshes.push(`URDF package mesh rejected: ${detail}`);
54:   diagnostics.robot_package_mesh_rejections = diagnostics.robot_package_mesh_rejections || [];
55:   diagnostics.robot_package_mesh_rejections.push({ source_url: sourceUrl || '', sourceUrl: sourceUrl || '', policy_reason: reason, policyReason: reason });
56:   return '';
57: }
58: 
59: 
60: function safeSceneAssetId(value) {
61:   const text = String(value || '').trim();
62:   return /^[A-Za-z0-9][A-Za-z0-9_.-]*$/.test(text) && !text.includes('%') ? text : '';
63: }
64: 

---

63: }
64: 
65: function safePackageAssetId(value) {
66:   const text = String(value || '').trim();
67:   return /^[A-Za-z][A-Za-z0-9_]*$/.test(text) ? text : '';
68: }
69: 
70: function canonicalStagedMeshUrl(rawUrl, context, diagnostics) {
71:   const raw = String(rawUrl || '').trim();
72:   const sourceUrl = raw;
73:   if (!raw) return { uri: '', reason: 'empty URL', sourceUrl };
74:   if (/^(?:https?|file|data):/i.test(raw) || raw.startsWith('//')) {
75:     return { uri: '', reason: `remote, file, or data URL rejected by viewer policy: ${raw}`, sourceUrl };
76:   }
77:   if (raw.includes('\\')) return { uri: '', reason: `backslash path rejected by viewer policy: ${raw}`, sourceUrl };
78:   if (/^[A-Za-z]:[\\/]/.test(raw)) return { uri: '', reason: `absolute filesystem path rejected by viewer policy: ${raw}`, sourceUrl };
79:   const withoutQuery = raw.split(/[?#]/, 1)[0];
80:   if (/^\/(?!build\/workcell_studio_web_scene\/assets\/)/.test(withoutQuery)) {
81:     if (/^\/[A-Za-z][A-Za-z0-9_]*(?:\/|$)/.test(withoutQuery)) return { uri: '', reason: `bare package-root URL: ${raw}`, sourceUrl };

---

64: 
65: function safePackageAssetId(value) {
66:   const text = String(value || '').trim();
67:   return /^[A-Za-z][A-Za-z0-9_]*$/.test(text) ? text : '';
68: }
69: 
70: function canonicalStagedMeshUrl(rawUrl, context, diagnostics) {
71:   const raw = String(rawUrl || '').trim();
72:   const sourceUrl = raw;
73:   if (!raw) return { uri: '', reason: 'empty URL', sourceUrl };
74:   if (/^(?:https?|file|data):/i.test(raw) || raw.startsWith('//')) {
75:     return { uri: '', reason: `remote, file, or data URL rejected by viewer policy: ${raw}`, sourceUrl };
76:   }
77:   if (raw.includes('\\')) return { uri: '', reason: `backslash path rejected by viewer policy: ${raw}`, sourceUrl };
78:   if (/^[A-Za-z]:[\\/]/.test(raw)) return { uri: '', reason: `absolute filesystem path rejected by viewer policy: ${raw}`, sourceUrl };
79:   const withoutQuery = raw.split(/[?#]/, 1)[0];
80:   if (/^\/(?!build\/workcell_studio_web_scene\/assets\/)/.test(withoutQuery)) {
81:     if (/^\/[A-Za-z][A-Za-z0-9_]*(?:\/|$)/.test(withoutQuery)) return { uri: '', reason: `bare package-root URL: ${raw}`, sourceUrl };
82:     return { uri: '', reason: `absolute filesystem path rejected by viewer policy: ${raw}`, sourceUrl };

---

66:   const text = String(value || '').trim();
67:   return /^[A-Za-z][A-Za-z0-9_]*$/.test(text) ? text : '';
68: }
69: 
70: function canonicalStagedMeshUrl(rawUrl, context, diagnostics) {
71:   const raw = String(rawUrl || '').trim();
72:   const sourceUrl = raw;
73:   if (!raw) return { uri: '', reason: 'empty URL', sourceUrl };
74:   if (/^(?:https?|file|data):/i.test(raw) || raw.startsWith('//')) {
75:     return { uri: '', reason: `remote, file, or data URL rejected by viewer policy: ${raw}`, sourceUrl };
76:   }
77:   if (raw.includes('\\')) return { uri: '', reason: `backslash path rejected by viewer policy: ${raw}`, sourceUrl };
78:   if (/^[A-Za-z]:[\\/]/.test(raw)) return { uri: '', reason: `absolute filesystem path rejected by viewer policy: ${raw}`, sourceUrl };
79:   const withoutQuery = raw.split(/[?#]/, 1)[0];
80:   if (/^\/(?!build\/workcell_studio_web_scene\/assets\/)/.test(withoutQuery)) {
81:     if (/^\/[A-Za-z][A-Za-z0-9_]*(?:\/|$)/.test(withoutQuery)) return { uri: '', reason: `bare package-root URL: ${raw}`, sourceUrl };
82:     return { uri: '', reason: `absolute filesystem path rejected by viewer policy: ${raw}`, sourceUrl };
83:   }
84:   const normalized = raw.startsWith(STAGED_MESH_ASSET_ROOT) ? raw : raw.replace(/^\/+/, '');

---

68: }
69: 
70: function canonicalStagedMeshUrl(rawUrl, context, diagnostics) {
71:   const raw = String(rawUrl || '').trim();
72:   const sourceUrl = raw;
73:   if (!raw) return { uri: '', reason: 'empty URL', sourceUrl };
74:   if (/^(?:https?|file|data):/i.test(raw) || raw.startsWith('//')) {
75:     return { uri: '', reason: `remote, file, or data URL rejected by viewer policy: ${raw}`, sourceUrl };
76:   }
77:   if (raw.includes('\\')) return { uri: '', reason: `backslash path rejected by viewer policy: ${raw}`, sourceUrl };
78:   if (/^[A-Za-z]:[\\/]/.test(raw)) return { uri: '', reason: `absolute filesystem path rejected by viewer policy: ${raw}`, sourceUrl };
79:   const withoutQuery = raw.split(/[?#]/, 1)[0];
80:   if (/^\/(?!build\/workcell_studio_web_scene\/assets\/)/.test(withoutQuery)) {
81:     if (/^\/[A-Za-z][A-Za-z0-9_]*(?:\/|$)/.test(withoutQuery)) return { uri: '', reason: `bare package-root URL: ${raw}`, sourceUrl };
82:     return { uri: '', reason: `absolute filesystem path rejected by viewer policy: ${raw}`, sourceUrl };
83:   }
84:   const normalized = raw.startsWith(STAGED_MESH_ASSET_ROOT) ? raw : raw.replace(/^\/+/, '');
85:   const pathOnly = normalized.split(/[?#]/, 1)[0];
86:   const prefix = STAGED_MESH_ASSET_ROOT;

---

69: 
70: function canonicalStagedMeshUrl(rawUrl, context, diagnostics) {
71:   const raw = String(rawUrl || '').trim();
72:   const sourceUrl = raw;
73:   if (!raw) return { uri: '', reason: 'empty URL', sourceUrl };
74:   if (/^(?:https?|file|data):/i.test(raw) || raw.startsWith('//')) {
75:     return { uri: '', reason: `remote, file, or data URL rejected by viewer policy: ${raw}`, sourceUrl };
76:   }
77:   if (raw.includes('\\')) return { uri: '', reason: `backslash path rejected by viewer policy: ${raw}`, sourceUrl };
78:   if (/^[A-Za-z]:[\\/]/.test(raw)) return { uri: '', reason: `absolute filesystem path rejected by viewer policy: ${raw}`, sourceUrl };
79:   const withoutQuery = raw.split(/[?#]/, 1)[0];
80:   if (/^\/(?!build\/workcell_studio_web_scene\/assets\/)/.test(withoutQuery)) {
81:     if (/^\/[A-Za-z][A-Za-z0-9_]*(?:\/|$)/.test(withoutQuery)) return { uri: '', reason: `bare package-root URL: ${raw}`, sourceUrl };
82:     return { uri: '', reason: `absolute filesystem path rejected by viewer policy: ${raw}`, sourceUrl };
83:   }
84:   const normalized = raw.startsWith(STAGED_MESH_ASSET_ROOT) ? raw : raw.replace(/^\/+/, '');
85:   const pathOnly = normalized.split(/[?#]/, 1)[0];
86:   const prefix = STAGED_MESH_ASSET_ROOT;
87:   const legacyPrefix = LEGACY_STAGED_MESH_ASSET_ROOT;

---

72:   const sourceUrl = raw;
73:   if (!raw) return { uri: '', reason: 'empty URL', sourceUrl };
74:   if (/^(?:https?|file|data):/i.test(raw) || raw.startsWith('//')) {
75:     return { uri: '', reason: `remote, file, or data URL rejected by viewer policy: ${raw}`, sourceUrl };
76:   }
77:   if (raw.includes('\\')) return { uri: '', reason: `backslash path rejected by viewer policy: ${raw}`, sourceUrl };
78:   if (/^[A-Za-z]:[\\/]/.test(raw)) return { uri: '', reason: `absolute filesystem path rejected by viewer policy: ${raw}`, sourceUrl };
79:   const withoutQuery = raw.split(/[?#]/, 1)[0];
80:   if (/^\/(?!build\/workcell_studio_web_scene\/assets\/)/.test(withoutQuery)) {
81:     if (/^\/[A-Za-z][A-Za-z0-9_]*(?:\/|$)/.test(withoutQuery)) return { uri: '', reason: `bare package-root URL: ${raw}`, sourceUrl };
82:     return { uri: '', reason: `absolute filesystem path rejected by viewer policy: ${raw}`, sourceUrl };
83:   }
84:   const normalized = raw.startsWith(STAGED_MESH_ASSET_ROOT) ? raw : raw.replace(/^\/+/, '');
85:   const pathOnly = normalized.split(/[?#]/, 1)[0];
86:   const prefix = STAGED_MESH_ASSET_ROOT;
87:   const legacyPrefix = LEGACY_STAGED_MESH_ASSET_ROOT;
88:   const hasCanonicalPrefix = pathOnly.startsWith(prefix);
89:   const hasLegacyPrefix = pathOnly.startsWith(legacyPrefix);
90:   if (!hasCanonicalPrefix && !hasLegacyPrefix) return { uri: '', reason: `URL is not under canonical staged mesh root ${prefix}: ${raw}`, sourceUrl };

---

73:   if (!raw) return { uri: '', reason: 'empty URL', sourceUrl };
74:   if (/^(?:https?|file|data):/i.test(raw) || raw.startsWith('//')) {
75:     return { uri: '', reason: `remote, file, or data URL rejected by viewer policy: ${raw}`, sourceUrl };
76:   }
77:   if (raw.includes('\\')) return { uri: '', reason: `backslash path rejected by viewer policy: ${raw}`, sourceUrl };
78:   if (/^[A-Za-z]:[\\/]/.test(raw)) return { uri: '', reason: `absolute filesystem path rejected by viewer policy: ${raw}`, sourceUrl };
79:   const withoutQuery = raw.split(/[?#]/, 1)[0];
80:   if (/^\/(?!build\/workcell_studio_web_scene\/assets\/)/.test(withoutQuery)) {
81:     if (/^\/[A-Za-z][A-Za-z0-9_]*(?:\/|$)/.test(withoutQuery)) return { uri: '', reason: `bare package-root URL: ${raw}`, sourceUrl };
82:     return { uri: '', reason: `absolute filesystem path rejected by viewer policy: ${raw}`, sourceUrl };
83:   }
84:   const normalized = raw.startsWith(STAGED_MESH_ASSET_ROOT) ? raw : raw.replace(/^\/+/, '');
85:   const pathOnly = normalized.split(/[?#]/, 1)[0];
86:   const prefix = STAGED_MESH_ASSET_ROOT;
87:   const legacyPrefix = LEGACY_STAGED_MESH_ASSET_ROOT;
88:   const hasCanonicalPrefix = pathOnly.startsWith(prefix);
89:   const hasLegacyPrefix = pathOnly.startsWith(legacyPrefix);
90:   if (!hasCanonicalPrefix && !hasLegacyPrefix) return { uri: '', reason: `URL is not under canonical staged mesh root ${prefix}: ${raw}`, sourceUrl };
91:   const suffix = pathOnly.slice(hasCanonicalPrefix ? prefix.length : legacyPrefix.length);

---

81:     if (/^\/[A-Za-z][A-Za-z0-9_]*(?:\/|$)/.test(withoutQuery)) return { uri: '', reason: `bare package-root URL: ${raw}`, sourceUrl };
82:     return { uri: '', reason: `absolute filesystem path rejected by viewer policy: ${raw}`, sourceUrl };
83:   }
84:   const normalized = raw.startsWith(STAGED_MESH_ASSET_ROOT) ? raw : raw.replace(/^\/+/, '');
85:   const pathOnly = normalized.split(/[?#]/, 1)[0];
86:   const prefix = STAGED_MESH_ASSET_ROOT;
87:   const legacyPrefix = LEGACY_STAGED_MESH_ASSET_ROOT;
88:   const hasCanonicalPrefix = pathOnly.startsWith(prefix);
89:   const hasLegacyPrefix = pathOnly.startsWith(legacyPrefix);
90:   if (!hasCanonicalPrefix && !hasLegacyPrefix) return { uri: '', reason: `URL is not under canonical staged mesh root ${prefix}: ${raw}`, sourceUrl };
91:   const suffix = pathOnly.slice(hasCanonicalPrefix ? prefix.length : legacyPrefix.length);
92:   const parts = suffix.split('/');
93:   const scene = safeSceneAssetId(parts.shift());
94:   const packageName = safePackageAssetId(parts.shift());
95:   const expectedScene = safeSceneAssetId(context?.sceneId);
96:   if (!scene) return { uri: '', reason: `malformed staged scene ID in URL: ${raw}`, sourceUrl };
97:   if (expectedScene && scene !== expectedScene) return { uri: '', reason: `staged scene ID mismatch: expected ${expectedScene}, got ${scene}`, sourceUrl };
98:   if (!packageName) return { uri: '', reason: `malformed staged package ID in URL: ${raw}`, sourceUrl };
99:   if (!parts.length) return { uri: '', reason: `staged URL is missing a mesh path: ${raw}`, sourceUrl };

---

87:   const legacyPrefix = LEGACY_STAGED_MESH_ASSET_ROOT;
88:   const hasCanonicalPrefix = pathOnly.startsWith(prefix);
89:   const hasLegacyPrefix = pathOnly.startsWith(legacyPrefix);
90:   if (!hasCanonicalPrefix && !hasLegacyPrefix) return { uri: '', reason: `URL is not under canonical staged mesh root ${prefix}: ${raw}`, sourceUrl };
91:   const suffix = pathOnly.slice(hasCanonicalPrefix ? prefix.length : legacyPrefix.length);
92:   const parts = suffix.split('/');
93:   const scene = safeSceneAssetId(parts.shift());
94:   const packageName = safePackageAssetId(parts.shift());
95:   const expectedScene = safeSceneAssetId(context?.sceneId);
96:   if (!scene) return { uri: '', reason: `malformed staged scene ID in URL: ${raw}`, sourceUrl };
97:   if (expectedScene && scene !== expectedScene) return { uri: '', reason: `staged scene ID mismatch: expected ${expectedScene}, got ${scene}`, sourceUrl };
98:   if (!packageName) return { uri: '', reason: `malformed staged package ID in URL: ${raw}`, sourceUrl };
99:   if (!parts.length) return { uri: '', reason: `staged URL is missing a mesh path: ${raw}`, sourceUrl };
100:   const safeParts = [];
101:   for (const part of parts) {
102:     if (!part) return { uri: '', reason: `empty path segment rejected in staged URL: ${raw}`, sourceUrl };
103:     const decoded = safeDecodeUriSegment(part);
104:     if (!decoded || decoded === '.' || decoded === '..' || decoded.includes('\0') || decoded.includes('/') || decoded.includes('\\') || /^[A-Za-z]:[\\/]/.test(decoded) || /^[A-Za-z][A-Za-z0-9+.-]*:/.test(decoded)) {
105:       return { uri: '', reason: `unsafe path segment rejected in staged URL: ${raw}`, sourceUrl };

---

88:   const hasCanonicalPrefix = pathOnly.startsWith(prefix);
89:   const hasLegacyPrefix = pathOnly.startsWith(legacyPrefix);
90:   if (!hasCanonicalPrefix && !hasLegacyPrefix) return { uri: '', reason: `URL is not under canonical staged mesh root ${prefix}: ${raw}`, sourceUrl };
91:   const suffix = pathOnly.slice(hasCanonicalPrefix ? prefix.length : legacyPrefix.length);
92:   const parts = suffix.split('/');
93:   const scene = safeSceneAssetId(parts.shift());
94:   const packageName = safePackageAssetId(parts.shift());
95:   const expectedScene = safeSceneAssetId(context?.sceneId);
96:   if (!scene) return { uri: '', reason: `malformed staged scene ID in URL: ${raw}`, sourceUrl };
97:   if (expectedScene && scene !== expectedScene) return { uri: '', reason: `staged scene ID mismatch: expected ${expectedScene}, got ${scene}`, sourceUrl };
98:   if (!packageName) return { uri: '', reason: `malformed staged package ID in URL: ${raw}`, sourceUrl };
99:   if (!parts.length) return { uri: '', reason: `staged URL is missing a mesh path: ${raw}`, sourceUrl };
100:   const safeParts = [];
101:   for (const part of parts) {
102:     if (!part) return { uri: '', reason: `empty path segment rejected in staged URL: ${raw}`, sourceUrl };
103:     const decoded = safeDecodeUriSegment(part);
104:     if (!decoded || decoded === '.' || decoded === '..' || decoded.includes('\0') || decoded.includes('/') || decoded.includes('\\') || /^[A-Za-z]:[\\/]/.test(decoded) || /^[A-Za-z][A-Za-z0-9+.-]*:/.test(decoded)) {
105:       return { uri: '', reason: `unsafe path segment rejected in staged URL: ${raw}`, sourceUrl };
106:     }

---

89:   const hasLegacyPrefix = pathOnly.startsWith(legacyPrefix);
90:   if (!hasCanonicalPrefix && !hasLegacyPrefix) return { uri: '', reason: `URL is not under canonical staged mesh root ${prefix}: ${raw}`, sourceUrl };
91:   const suffix = pathOnly.slice(hasCanonicalPrefix ? prefix.length : legacyPrefix.length);
92:   const parts = suffix.split('/');
93:   const scene = safeSceneAssetId(parts.shift());
94:   const packageName = safePackageAssetId(parts.shift());
95:   const expectedScene = safeSceneAssetId(context?.sceneId);
96:   if (!scene) return { uri: '', reason: `malformed staged scene ID in URL: ${raw}`, sourceUrl };
97:   if (expectedScene && scene !== expectedScene) return { uri: '', reason: `staged scene ID mismatch: expected ${expectedScene}, got ${scene}`, sourceUrl };
98:   if (!packageName) return { uri: '', reason: `malformed staged package ID in URL: ${raw}`, sourceUrl };
99:   if (!parts.length) return { uri: '', reason: `staged URL is missing a mesh path: ${raw}`, sourceUrl };
100:   const safeParts = [];
101:   for (const part of parts) {
102:     if (!part) return { uri: '', reason: `empty path segment rejected in staged URL: ${raw}`, sourceUrl };
103:     const decoded = safeDecodeUriSegment(part);
104:     if (!decoded || decoded === '.' || decoded === '..' || decoded.includes('\0') || decoded.includes('/') || decoded.includes('\\') || /^[A-Za-z]:[\\/]/.test(decoded) || /^[A-Za-z][A-Za-z0-9+.-]*:/.test(decoded)) {
105:       return { uri: '', reason: `unsafe path segment rejected in staged URL: ${raw}`, sourceUrl };
106:     }
107:     if (decoded.includes('%')) {

---

90:   if (!hasCanonicalPrefix && !hasLegacyPrefix) return { uri: '', reason: `URL is not under canonical staged mesh root ${prefix}: ${raw}`, sourceUrl };
91:   const suffix = pathOnly.slice(hasCanonicalPrefix ? prefix.length : legacyPrefix.length);
92:   const parts = suffix.split('/');
93:   const scene = safeSceneAssetId(parts.shift());
94:   const packageName = safePackageAssetId(parts.shift());
95:   const expectedScene = safeSceneAssetId(context?.sceneId);
96:   if (!scene) return { uri: '', reason: `malformed staged scene ID in URL: ${raw}`, sourceUrl };
97:   if (expectedScene && scene !== expectedScene) return { uri: '', reason: `staged scene ID mismatch: expected ${expectedScene}, got ${scene}`, sourceUrl };
98:   if (!packageName) return { uri: '', reason: `malformed staged package ID in URL: ${raw}`, sourceUrl };
99:   if (!parts.length) return { uri: '', reason: `staged URL is missing a mesh path: ${raw}`, sourceUrl };
100:   const safeParts = [];
101:   for (const part of parts) {
102:     if (!part) return { uri: '', reason: `empty path segment rejected in staged URL: ${raw}`, sourceUrl };
103:     const decoded = safeDecodeUriSegment(part);
104:     if (!decoded || decoded === '.' || decoded === '..' || decoded.includes('\0') || decoded.includes('/') || decoded.includes('\\') || /^[A-Za-z]:[\\/]/.test(decoded) || /^[A-Za-z][A-Za-z0-9+.-]*:/.test(decoded)) {
105:       return { uri: '', reason: `unsafe path segment rejected in staged URL: ${raw}`, sourceUrl };
106:     }
107:     if (decoded.includes('%')) {
108:       const decodedAgain = safeDecodeUriSegment(decoded);

---

93:   const scene = safeSceneAssetId(parts.shift());
94:   const packageName = safePackageAssetId(parts.shift());
95:   const expectedScene = safeSceneAssetId(context?.sceneId);
96:   if (!scene) return { uri: '', reason: `malformed staged scene ID in URL: ${raw}`, sourceUrl };
97:   if (expectedScene && scene !== expectedScene) return { uri: '', reason: `staged scene ID mismatch: expected ${expectedScene}, got ${scene}`, sourceUrl };
98:   if (!packageName) return { uri: '', reason: `malformed staged package ID in URL: ${raw}`, sourceUrl };
99:   if (!parts.length) return { uri: '', reason: `staged URL is missing a mesh path: ${raw}`, sourceUrl };
100:   const safeParts = [];
101:   for (const part of parts) {
102:     if (!part) return { uri: '', reason: `empty path segment rejected in staged URL: ${raw}`, sourceUrl };
103:     const decoded = safeDecodeUriSegment(part);
104:     if (!decoded || decoded === '.' || decoded === '..' || decoded.includes('\0') || decoded.includes('/') || decoded.includes('\\') || /^[A-Za-z]:[\\/]/.test(decoded) || /^[A-Za-z][A-Za-z0-9+.-]*:/.test(decoded)) {
105:       return { uri: '', reason: `unsafe path segment rejected in staged URL: ${raw}`, sourceUrl };
106:     }
107:     if (decoded.includes('%')) {
108:       const decodedAgain = safeDecodeUriSegment(decoded);
109:       if (decodedAgain !== decoded) return { uri: '', reason: `encoded traversal rejected in staged URL: ${raw}`, sourceUrl };
110:     }
111:     safeParts.push(encodeURIComponent(decoded));

---

96:   if (!scene) return { uri: '', reason: `malformed staged scene ID in URL: ${raw}`, sourceUrl };
97:   if (expectedScene && scene !== expectedScene) return { uri: '', reason: `staged scene ID mismatch: expected ${expectedScene}, got ${scene}`, sourceUrl };
98:   if (!packageName) return { uri: '', reason: `malformed staged package ID in URL: ${raw}`, sourceUrl };
99:   if (!parts.length) return { uri: '', reason: `staged URL is missing a mesh path: ${raw}`, sourceUrl };
100:   const safeParts = [];
101:   for (const part of parts) {
102:     if (!part) return { uri: '', reason: `empty path segment rejected in staged URL: ${raw}`, sourceUrl };
103:     const decoded = safeDecodeUriSegment(part);
104:     if (!decoded || decoded === '.' || decoded === '..' || decoded.includes('\0') || decoded.includes('/') || decoded.includes('\\') || /^[A-Za-z]:[\\/]/.test(decoded) || /^[A-Za-z][A-Za-z0-9+.-]*:/.test(decoded)) {
105:       return { uri: '', reason: `unsafe path segment rejected in staged URL: ${raw}`, sourceUrl };
106:     }
107:     if (decoded.includes('%')) {
108:       const decodedAgain = safeDecodeUriSegment(decoded);
109:       if (decodedAgain !== decoded) return { uri: '', reason: `encoded traversal rejected in staged URL: ${raw}`, sourceUrl };
110:     }
111:     safeParts.push(encodeURIComponent(decoded));
112:   }
113:   return { uri: `${STAGED_MESH_ASSET_ROOT}${scene}/${packageName}/${safeParts.join('/')}`, reason: '', sourceUrl };
114: }

---

100:   const safeParts = [];
101:   for (const part of parts) {
102:     if (!part) return { uri: '', reason: `empty path segment rejected in staged URL: ${raw}`, sourceUrl };
103:     const decoded = safeDecodeUriSegment(part);
104:     if (!decoded || decoded === '.' || decoded === '..' || decoded.includes('\0') || decoded.includes('/') || decoded.includes('\\') || /^[A-Za-z]:[\\/]/.test(decoded) || /^[A-Za-z][A-Za-z0-9+.-]*:/.test(decoded)) {
105:       return { uri: '', reason: `unsafe path segment rejected in staged URL: ${raw}`, sourceUrl };
106:     }
107:     if (decoded.includes('%')) {
108:       const decodedAgain = safeDecodeUriSegment(decoded);
109:       if (decodedAgain !== decoded) return { uri: '', reason: `encoded traversal rejected in staged URL: ${raw}`, sourceUrl };
110:     }
111:     safeParts.push(encodeURIComponent(decoded));
112:   }
113:   return { uri: `${STAGED_MESH_ASSET_ROOT}${scene}/${packageName}/${safeParts.join('/')}`, reason: '', sourceUrl };
114: }
115: 
116: function resolvePackageMeshUri(uri, sceneId, diagnostics) {
117:   const raw = String(uri || '').trim();
118:   if (!raw.startsWith('package://')) return '';

---

104:     if (!decoded || decoded === '.' || decoded === '..' || decoded.includes('\0') || decoded.includes('/') || decoded.includes('\\') || /^[A-Za-z]:[\\/]/.test(decoded) || /^[A-Za-z][A-Za-z0-9+.-]*:/.test(decoded)) {
105:       return { uri: '', reason: `unsafe path segment rejected in staged URL: ${raw}`, sourceUrl };
106:     }
107:     if (decoded.includes('%')) {
108:       const decodedAgain = safeDecodeUriSegment(decoded);
109:       if (decodedAgain !== decoded) return { uri: '', reason: `encoded traversal rejected in staged URL: ${raw}`, sourceUrl };
110:     }
111:     safeParts.push(encodeURIComponent(decoded));
112:   }
113:   return { uri: `${STAGED_MESH_ASSET_ROOT}${scene}/${packageName}/${safeParts.join('/')}`, reason: '', sourceUrl };
114: }
115: 
116: function resolvePackageMeshUri(uri, sceneId, diagnostics) {
117:   const raw = String(uri || '').trim();
118:   if (!raw.startsWith('package://')) return '';
119:   const scene = String(sceneId || '').trim();
120:   if (!scene) return rejectPackageMeshUri('missing scene ID', diagnostics, raw);
121:   if (!safeSceneAssetId(scene)) return rejectPackageMeshUri(`malformed scene ID: ${scene}`, diagnostics, raw);
122:   const packagePath = raw.slice('package://'.length);

---

163:   if ('packages' in loader) loader.packages = resolver;
164:   else loader.packages = resolver;
165:   if (manager?.setURLModifier) {
166:     manager.setURLModifier(url => {
167:       const raw = String(url || '').trim();
168:       if (raw.startsWith('package://')) return resolvePackageMeshUri(raw, context?.sceneId, diagnostics);
169:       const staged = canonicalStagedMeshUrl(raw, context, diagnostics);
170:       if (staged.uri) return staged.uri;
171:       if (raw.startsWith('/') || /^(?:https?|file|data):/i.test(raw) || raw.startsWith('//') || raw.includes('\\') || /^[A-Za-z]:[\\/]/.test(raw)) {
172:         return rejectPackageMeshUri(staged.reason, diagnostics, staged.sourceUrl);
173:       }
174:       return url;
175:     });
176:   }
177: }
178: 
179: function normalizeMeshUri(path, context, diagnostics) {
180:   const raw = String(path || '').trim();
181:   if (!raw) return '';

---

179: function normalizeMeshUri(path, context, diagnostics) {
180:   const raw = String(path || '').trim();
181:   if (!raw) return '';
182:   if (raw.startsWith('package://')) {
183:     return resolvePackageMeshUri(raw, context?.sceneId, diagnostics);
184:   }
185:   const staged = canonicalStagedMeshUrl(raw, context, diagnostics);
186:   if (staged.uri) return staged.uri;
187:   if (raw.startsWith('/') || /^(?:https?|file|data):/i.test(raw) || raw.startsWith('//') || raw.includes('\\') || /^[A-Za-z]:[\\/]/.test(raw)) {
188:     return rejectPackageMeshUri(staged.reason, diagnostics, staged.sourceUrl);
189:   }
190:   const diagnostic = context?.meshUriDiagnostic?.({ mesh_uri: raw, mesh_staging_status: 'staged' });
191:   return diagnostic?.uri || raw;
192: }
193: 
194: function meshExtension(uri) {
195:   const path = String(uri || '').split(/[?#]/, 1)[0];
196:   return path.includes('.') ? path.slice(path.lastIndexOf('.') + 1).toLowerCase() : '';
197: }

---

200:   if (!material || !object?.traverse) return object;
201:   object.traverse(child => {
202:     if (child?.isMesh && !child.material) child.material = material;
203:   });
204:   return object;
205: }
206: 
207: function setLifecycleState(diagnostics, state) {
208:   diagnostics.robot_preview_lifecycle_state = state;
209:   diagnostics.robotPreviewLifecycleState = state;
210: }
211: 
212: function isIdentityTransform(object, eps = 1e-9) {
213:   if (!object) return true;
214:   return Math.abs(object.position?.x || 0) <= eps
215:     && Math.abs(object.position?.y || 0) <= eps
216:     && Math.abs(object.position?.z || 0) <= eps
217:     && Math.abs((object.quaternion?.x || 0)) <= eps
218:     && Math.abs((object.quaternion?.y || 0)) <= eps

---

287:   }
288: 
289:   scene.position.set(0, 0, 0);
290:   scene.quaternion.identity();
291:   scene.rotation.set(0, 0, 0);
292:   scene.scale.set(1, 1, 1);
293:   scene.updateMatrix();
294:   scene.updateMatrixWorld(true);
295:   diagnostics.robot_collada_root_normalization_count += 1;
296:   diagnostics.robotColladaRootNormalizationCount = diagnostics.robot_collada_root_normalization_count;
297:   const detail = {
298:     uri,
299:     up_axis: upAxis || null,
300:     unit_meter: Number.isFinite(unitMeter) ? unitMeter : null,
301:     descendant_mesh_count: meshCount,
302:     root_transform_normalized: true,
303:     root_transform_before: before,
304:     root_transform_after: objectLocalTransformDiagnostics(scene),
305:   };

---

313:   diagnostics.robot_expected_visual_count += 1;
314:   diagnostics.robotExpectedVisualCount = diagnostics.robot_expected_visual_count;
315:   if (!uri) {
316:     diagnostics.robot_failed_visual_count += 1;
317:     diagnostics.robotFailedVisualCount = diagnostics.robot_failed_visual_count;
318:     const policy = canonicalStagedMeshUrl(path, context, diagnostics);
319:     const reason = policy.reason || `unloadable URDF mesh URI: ${path}`;
320:     const err = new Error(`unloadable URDF mesh URI: link=${inferMeshLinkDetail(path).link || '<unknown>'} source_url=${path} policy_reason=${reason}`);
321:     done(null, err);
322:     context?.onRobotMeshLoadError?.(err, '', { path, source_url: path, sourceUrl: path, policy_reason: reason, policyReason: reason, ...inferMeshLinkDetail(path) });
323:     return;
324:   }
325:   const url = repoUrl(context, uri);
326:   const ext = meshExtension(uri);
327:   const onDone = object => {
328:     diagnostics.robot_loaded_visual_count += 1;
329:     diagnostics.robot_completed_visual_count = diagnostics.robot_loaded_visual_count;
330:     diagnostics.robotCompletedVisualCount = diagnostics.robot_completed_visual_count;
331:     done(applyFallbackMaterial(object, material));

---

330:     diagnostics.robotCompletedVisualCount = diagnostics.robot_completed_visual_count;
331:     done(applyFallbackMaterial(object, material));
332:     context?.onRobotMeshLoaded?.();
333:   };
334:   const onError = err => {
335:     diagnostics.robot_failed_visual_count += 1;
336:     diagnostics.robotFailedVisualCount = diagnostics.robot_failed_visual_count;
337:     diagnostics.robot_missing_meshes.push(`${url}: ${err?.message || err || 'load failed'}`);
338:     done(null, err);
339:     context?.onRobotMeshLoadError?.(err, uri, { url, uri, path, source_url: path, sourceUrl: path, policy_reason: err?.message || 'loader failure', policyReason: err?.message || 'loader failure', ...inferMeshLinkDetail(path) });
340:   };
341:   if (ext === 'stl') new STLLoader(manager).load(url, geom => onDone(new THREE.Mesh(geom, material || new THREE.MeshPhongMaterial())), undefined, onError);
342:   else if (ext === 'dae') loadColladaWithSceneScopedZUpDiagnostic(new ColladaLoader(manager), url, dae => onDone(normalizeRosColladaScene(dae, uri, diagnostics)), undefined, onError, diagnostics);
343:   else if (ext === 'obj') new OBJLoader(manager).load(url, obj => onDone(obj), undefined, onError);
344:   else onError(new Error(`unsupported mesh format .${ext || 'unknown'}`));
345: }
346: 
347: function inferMeshLinkDetail(path) {
348:   const text = String(path || '').toLowerCase();

---

463:   return { pass, tolerance, compared_count: comparisons.length, comparedCount: comparisons.length, comparisons };
464: }
465: 
466: function collectMatrixParityDiagnostics(previewConfig, diagnostics) {
467:   const tolerance = Number(previewConfig?.matrix_diagnostic_tolerance ?? previewConfig?.matrixDiagnosticTolerance ?? 1e-5);
468:   const linkExpected = expectedMatrixMap(
469:     previewConfig?.expected_robot_link_world_matrices,
470:     previewConfig?.expectedRobotLinkWorldMatrices,
471:     previewConfig?.robot_link_world_matrices_expected,
472:     previewConfig?.robotLinkWorldMatricesExpected
473:   );
474:   const visualExpected = expectedMatrixMap(
475:     previewConfig?.expected_robot_visual_wrapper_world_matrices,
476:     previewConfig?.expectedRobotVisualWrapperWorldMatrices,
477:     previewConfig?.robot_visual_wrapper_world_matrices_expected,
478:     previewConfig?.robotVisualWrapperWorldMatricesExpected
479:   );
480:   const linkParity = compareMatrixDiagnostics(diagnostics.robot_link_world_matrices || {}, linkExpected, tolerance);
481:   const visualActual = new Map((diagnostics.robot_visual_wrapper_world_matrices || []).map(entry => [String(entry?.link_name || entry?.linkName || '').trim(), entry]).filter(([key]) => key));

---

469:     previewConfig?.expected_robot_link_world_matrices,
470:     previewConfig?.expectedRobotLinkWorldMatrices,
471:     previewConfig?.robot_link_world_matrices_expected,
472:     previewConfig?.robotLinkWorldMatricesExpected
473:   );
474:   const visualExpected = expectedMatrixMap(
475:     previewConfig?.expected_robot_visual_wrapper_world_matrices,
476:     previewConfig?.expectedRobotVisualWrapperWorldMatrices,
477:     previewConfig?.robot_visual_wrapper_world_matrices_expected,
478:     previewConfig?.robotVisualWrapperWorldMatricesExpected
479:   );
480:   const linkParity = compareMatrixDiagnostics(diagnostics.robot_link_world_matrices || {}, linkExpected, tolerance);
481:   const visualActual = new Map((diagnostics.robot_visual_wrapper_world_matrices || []).map(entry => [String(entry?.link_name || entry?.linkName || '').trim(), entry]).filter(([key]) => key));
482:   const visualParity = compareMatrixDiagnostics(visualActual, visualExpected, tolerance);
483:   diagnostics.robot_matrix_world_parity = { tolerance, link: linkParity, visual_wrapper: visualParity, pass: linkParity.pass && visualParity.pass };
484:   diagnostics.robotMatrixWorldParity = diagnostics.robot_matrix_world_parity;
485:   return diagnostics.robot_matrix_world_parity;
486: }
487: 

---

696: 
697: export function applyRobotJointPreview(result, jointValues = {}) {
698:   const robot = result?.root;
699:   if (!robot?.setJointValues) throw new Error('Product View is not ready');
700:   robot.setJointValues(jointValues);
701:   robot.updateMatrixWorld?.(true);
702:   const links = Object.fromEntries(result.links || []);
703:   result.diagnostics.robot_joint_values_applied = { ...jointValues };
704:   result.diagnostics.robot_link_world_matrices = collectLinkMatrixDiagnostics(robot, links);
705:   result.diagnostics.robotLinkWorldMatrices = result.diagnostics.robot_link_world_matrices;
706:   result.diagnostics.robot_visual_wrapper_world_matrices = collectVisualWrapperMatrixDiagnostics(links);
707:   result.diagnostics.robotVisualWrapperWorldMatrices = result.diagnostics.robot_visual_wrapper_world_matrices;
708:   result.diagnostics.robot_descendant_render_mesh_diagnostics = collectDescendantRenderMeshDiagnostics(links);
709:   result.diagnostics.robotDescendantRenderMeshDiagnostics = result.diagnostics.robot_descendant_render_mesh_diagnostics;
710:   collectMatrixParityDiagnostics(result?.previewConfig || {}, result.diagnostics);
711:   return result.diagnostics;
712: }
713: 
714: function jointTypeCounts(joints) {

---

698:   const robot = result?.root;
699:   if (!robot?.setJointValues) throw new Error('Product View is not ready');
700:   robot.setJointValues(jointValues);
701:   robot.updateMatrixWorld?.(true);
702:   const links = Object.fromEntries(result.links || []);
703:   result.diagnostics.robot_joint_values_applied = { ...jointValues };
704:   result.diagnostics.robot_link_world_matrices = collectLinkMatrixDiagnostics(robot, links);
705:   result.diagnostics.robotLinkWorldMatrices = result.diagnostics.robot_link_world_matrices;
706:   result.diagnostics.robot_visual_wrapper_world_matrices = collectVisualWrapperMatrixDiagnostics(links);
707:   result.diagnostics.robotVisualWrapperWorldMatrices = result.diagnostics.robot_visual_wrapper_world_matrices;
708:   result.diagnostics.robot_descendant_render_mesh_diagnostics = collectDescendantRenderMeshDiagnostics(links);
709:   result.diagnostics.robotDescendantRenderMeshDiagnostics = result.diagnostics.robot_descendant_render_mesh_diagnostics;
710:   collectMatrixParityDiagnostics(result?.previewConfig || {}, result.diagnostics);
711:   return result.diagnostics;
712: }
713: 
714: function jointTypeCounts(joints) {
715:   const counts = { fixed: 0, revolute: 0, continuous: 0, prismatic: 0, mimic: 0, other: 0 };
716:   for (const joint of Object.values(joints || {})) {

---

744:     robot_root_link_count: 0,
745:     robotRootLinkCount: 0,
746:     robot_root_links: [],
747:     robotRootLinks: [],
748:     robot_disconnected_links: [],
749:     robotDisconnectedLinks: [],
750:     robot_duplicate_links: [],
751:     robotDuplicateLinks: [],
752:     robot_preview_lifecycle_state: 'idle',
753:     robotPreviewLifecycleState: 'idle',
754:     robot_preview_canonical_fallback_used: false,
755:     robotPreviewCanonicalFallbackUsed: false,
756:     robot_collada_root_normalization_count: 0,
757:     robotColladaRootNormalizationCount: 0,
758:     robot_collada_mesh_diagnostics: [],
759:     robotColladaMeshDiagnostics: [],
760:     robot_collada_summary: [],
761:     robotColladaSummary: [],
762:     robot_package_mesh_resolutions: [],

---

748:     robot_disconnected_links: [],
749:     robotDisconnectedLinks: [],
750:     robot_duplicate_links: [],
751:     robotDuplicateLinks: [],
752:     robot_preview_lifecycle_state: 'idle',
753:     robotPreviewLifecycleState: 'idle',
754:     robot_preview_canonical_fallback_used: false,
755:     robotPreviewCanonicalFallbackUsed: false,
756:     robot_collada_root_normalization_count: 0,
757:     robotColladaRootNormalizationCount: 0,
758:     robot_collada_mesh_diagnostics: [],
759:     robotColladaMeshDiagnostics: [],
760:     robot_collada_summary: [],
761:     robotColladaSummary: [],
762:     robot_package_mesh_resolutions: [],
763:     robot_package_mesh_rejections: [],
764:     robot_descendant_render_mesh_diagnostics: [],
765:     robotDescendantRenderMeshDiagnostics: [],
766:     skipped_legacy_generated_urdf_visual_count: rendererContext?.skippedLegacyGeneratedUrdfVisualCount || 0,

---

799: 
800:     result.root = robot;
801:     result.links = buildLookupMap(robot.links);
802:     result.joints = buildLookupMap(robot.joints);
803:     diagnostics.robot_loaded_link_count = result.links.size;
804:     diagnostics.robot_loaded_joint_count = result.joints.size;
805:     diagnostics.robot_joint_type_counts = jointTypeCounts(robot.joints);
806:     diagnostics.robot_hierarchy_links = Array.from(result.links.keys());
807:     diagnostics.robot_link_world_matrices = collectLinkMatrixDiagnostics(robot, robot.links);
808:     diagnostics.robotLinkWorldMatrices = diagnostics.robot_link_world_matrices;
809:     diagnostics.robot_visual_wrapper_world_matrices = collectVisualWrapperMatrixDiagnostics(robot.links);
810:     diagnostics.robotVisualWrapperWorldMatrices = diagnostics.robot_visual_wrapper_world_matrices;
811:     diagnostics.robot_descendant_render_mesh_diagnostics = collectDescendantRenderMeshDiagnostics(robot.links);
812:     diagnostics.robotDescendantRenderMeshDiagnostics = diagnostics.robot_descendant_render_mesh_diagnostics;
813:     collectMatrixParityDiagnostics(previewConfig, diagnostics);
814:     diagnostics.robot_collada_mesh_diagnostics = diagnostics.robot_collada_mesh_diagnostics || [];
815:     diagnostics.robotColladaMeshDiagnostics = diagnostics.robot_collada_mesh_diagnostics;
816:     diagnostics.robot_collada_summary = diagnostics.robot_collada_summary || [];
817:     diagnostics.robotColladaSummary = diagnostics.robot_collada_summary;

---

801:     result.links = buildLookupMap(robot.links);
802:     result.joints = buildLookupMap(robot.joints);
803:     diagnostics.robot_loaded_link_count = result.links.size;
804:     diagnostics.robot_loaded_joint_count = result.joints.size;
805:     diagnostics.robot_joint_type_counts = jointTypeCounts(robot.joints);
806:     diagnostics.robot_hierarchy_links = Array.from(result.links.keys());
807:     diagnostics.robot_link_world_matrices = collectLinkMatrixDiagnostics(robot, robot.links);
808:     diagnostics.robotLinkWorldMatrices = diagnostics.robot_link_world_matrices;
809:     diagnostics.robot_visual_wrapper_world_matrices = collectVisualWrapperMatrixDiagnostics(robot.links);
810:     diagnostics.robotVisualWrapperWorldMatrices = diagnostics.robot_visual_wrapper_world_matrices;
811:     diagnostics.robot_descendant_render_mesh_diagnostics = collectDescendantRenderMeshDiagnostics(robot.links);
812:     diagnostics.robotDescendantRenderMeshDiagnostics = diagnostics.robot_descendant_render_mesh_diagnostics;
813:     collectMatrixParityDiagnostics(previewConfig, diagnostics);
814:     diagnostics.robot_collada_mesh_diagnostics = diagnostics.robot_collada_mesh_diagnostics || [];
815:     diagnostics.robotColladaMeshDiagnostics = diagnostics.robot_collada_mesh_diagnostics;
816:     diagnostics.robot_collada_summary = diagnostics.robot_collada_summary || [];
817:     diagnostics.robotColladaSummary = diagnostics.robot_collada_summary;
818:     const rootDiagnostics = linkRootDiagnostics(robot, robot.links);
819:     diagnostics.robot_root_links = rootDiagnostics.roots;

---

834:     diagnostics.robot_missing_required_robot_visual_links = expectedRobotVisualLinks.filter(link => !loadedVisualLinks.has(link));
835:     diagnostics.robotMissingRequiredRobotVisualLinks = diagnostics.robot_missing_required_robot_visual_links;
836:     diagnostics.robot_missing_required_tool_visual_links = expectedToolVisualLinks.filter(link => !loadedVisualLinks.has(link));
837:     diagnostics.robotMissingRequiredToolVisualLinks = diagnostics.robot_missing_required_tool_visual_links;
838:     diagnostics.robot_expected_robot_visual_links = expectedRobotVisualLinks;
839:     diagnostics.robotExpectedRobotVisualLinks = expectedRobotVisualLinks;
840:     diagnostics.robot_expected_tool_visual_links = expectedToolVisualLinks;
841:     diagnostics.robotExpectedToolVisualLinks = expectedToolVisualLinks;
842:     diagnostics.robot_link_world_matrices = collectLinkMatrixDiagnostics(robot, robot.links);
843:     diagnostics.robotLinkWorldMatrices = diagnostics.robot_link_world_matrices;
844:     diagnostics.robot_visual_wrapper_world_matrices = collectVisualWrapperMatrixDiagnostics(robot.links);
845:     diagnostics.robotVisualWrapperWorldMatrices = diagnostics.robot_visual_wrapper_world_matrices;
846:     diagnostics.robot_descendant_render_mesh_diagnostics = collectDescendantRenderMeshDiagnostics(robot.links);
847:     diagnostics.robotDescendantRenderMeshDiagnostics = diagnostics.robot_descendant_render_mesh_diagnostics;
848:     diagnostics.robot_mesh_callbacks_complete = diagnostics.robot_expected_visual_count === (diagnostics.robot_loaded_visual_count + diagnostics.robot_failed_visual_count);
849:     diagnostics.robotMeshCallbacksComplete = diagnostics.robot_mesh_callbacks_complete;
850:     diagnostics.robot_preview_loaded = diagnostics.robot_failed_visual_count === 0 && diagnostics.robot_hierarchy_missing_links.length === 0 && diagnostics.robot_missing_required_robot_visual_links.length === 0 && diagnostics.robot_missing_required_tool_visual_links.length === 0 && diagnostics.robot_mesh_callbacks_complete;
851:     setLifecycleState(diagnostics, diagnostics.robot_preview_loaded ? 'ready' : 'failed');
852:     rendererContext?.scene?.add?.(robot);

---

836:     diagnostics.robot_missing_required_tool_visual_links = expectedToolVisualLinks.filter(link => !loadedVisualLinks.has(link));
837:     diagnostics.robotMissingRequiredToolVisualLinks = diagnostics.robot_missing_required_tool_visual_links;
838:     diagnostics.robot_expected_robot_visual_links = expectedRobotVisualLinks;
839:     diagnostics.robotExpectedRobotVisualLinks = expectedRobotVisualLinks;
840:     diagnostics.robot_expected_tool_visual_links = expectedToolVisualLinks;
841:     diagnostics.robotExpectedToolVisualLinks = expectedToolVisualLinks;
842:     diagnostics.robot_link_world_matrices = collectLinkMatrixDiagnostics(robot, robot.links);
843:     diagnostics.robotLinkWorldMatrices = diagnostics.robot_link_world_matrices;
844:     diagnostics.robot_visual_wrapper_world_matrices = collectVisualWrapperMatrixDiagnostics(robot.links);
845:     diagnostics.robotVisualWrapperWorldMatrices = diagnostics.robot_visual_wrapper_world_matrices;
846:     diagnostics.robot_descendant_render_mesh_diagnostics = collectDescendantRenderMeshDiagnostics(robot.links);
847:     diagnostics.robotDescendantRenderMeshDiagnostics = diagnostics.robot_descendant_render_mesh_diagnostics;
848:     diagnostics.robot_mesh_callbacks_complete = diagnostics.robot_expected_visual_count === (diagnostics.robot_loaded_visual_count + diagnostics.robot_failed_visual_count);
849:     diagnostics.robotMeshCallbacksComplete = diagnostics.robot_mesh_callbacks_complete;
850:     diagnostics.robot_preview_loaded = diagnostics.robot_failed_visual_count === 0 && diagnostics.robot_hierarchy_missing_links.length === 0 && diagnostics.robot_missing_required_robot_visual_links.length === 0 && diagnostics.robot_missing_required_tool_visual_links.length === 0 && diagnostics.robot_mesh_callbacks_complete;
851:     setLifecycleState(diagnostics, diagnostics.robot_preview_loaded ? 'ready' : 'failed');
852:     rendererContext?.scene?.add?.(robot);
853:     rendererContext?.assemblyRoots?.push?.(robot);
854:     rendererContext?.onRobotLoaded?.(result);
```

# Qt status-script compatibility contexts
```text
2183:     const QString detail = QStringLiteral(
2184:       "startup timed out after 45s for scene %1; viewer URL: %2; expected JSON: %3; pending_required_loads and last observed boot status: %4")
2185:       .arg(identity.scene_id, viewer_url, expected_json_path, embedded_web_last_boot_status_.isEmpty() ? QStringLiteral("unavailable") : embedded_web_last_boot_status_);
2186:     handle_embedded_web_runtime_failure(identity, navigation_token, detail);
2187:     emit studio_log_requested(QStringLiteral("Embedded Product View readiness timeout. %1").arg(detail));
2188:     return;
2189:   }
2190: 
2191:   static const char kStatusScript[] = R"JS(
2192: (() => {
2193:   const s = window.__WORKCELL_VIEWER_STATUS__ || {};
2194:   return {
2195:     readiness_contract_version: Number(s.readiness_contract_version ?? s.readinessContractVersion ?? 0),
2196:     lifecycle_state: s.lifecycle_state || s.lifecycleState || '',
2197:     terminal: Boolean(s.terminal),
2198:     status_sequence: Number(s.status_sequence ?? s.statusSequence ?? 0),
2199:     builder_revision: String(s.builder_revision ?? s.builderRevision ?? ''),
2200:     viewer_boot_state: s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || s.viewerBootState || '',
2201:     scene_name: s.scene_name || s.sceneName || '',
2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),

---

2184:       "startup timed out after 45s for scene %1; viewer URL: %2; expected JSON: %3; pending_required_loads and last observed boot status: %4")
2185:       .arg(identity.scene_id, viewer_url, expected_json_path, embedded_web_last_boot_status_.isEmpty() ? QStringLiteral("unavailable") : embedded_web_last_boot_status_);
2186:     handle_embedded_web_runtime_failure(identity, navigation_token, detail);
2187:     emit studio_log_requested(QStringLiteral("Embedded Product View readiness timeout. %1").arg(detail));
2188:     return;
2189:   }
2190: 
2191:   static const char kStatusScript[] = R"JS(
2192: (() => {
2193:   const s = window.__WORKCELL_VIEWER_STATUS__ || {};
2194:   return {
2195:     readiness_contract_version: Number(s.readiness_contract_version ?? s.readinessContractVersion ?? 0),
2196:     lifecycle_state: s.lifecycle_state || s.lifecycleState || '',
2197:     terminal: Boolean(s.terminal),
2198:     status_sequence: Number(s.status_sequence ?? s.statusSequence ?? 0),
2199:     builder_revision: String(s.builder_revision ?? s.builderRevision ?? ''),
2200:     viewer_boot_state: s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || s.viewerBootState || '',
2201:     scene_name: s.scene_name || s.sceneName || '',
2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),

---

2186:     handle_embedded_web_runtime_failure(identity, navigation_token, detail);
2187:     emit studio_log_requested(QStringLiteral("Embedded Product View readiness timeout. %1").arg(detail));
2188:     return;
2189:   }
2190: 
2191:   static const char kStatusScript[] = R"JS(
2192: (() => {
2193:   const s = window.__WORKCELL_VIEWER_STATUS__ || {};
2194:   return {
2195:     readiness_contract_version: Number(s.readiness_contract_version ?? s.readinessContractVersion ?? 0),
2196:     lifecycle_state: s.lifecycle_state || s.lifecycleState || '',
2197:     terminal: Boolean(s.terminal),
2198:     status_sequence: Number(s.status_sequence ?? s.statusSequence ?? 0),
2199:     builder_revision: String(s.builder_revision ?? s.builderRevision ?? ''),
2200:     viewer_boot_state: s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || s.viewerBootState || '',
2201:     scene_name: s.scene_name || s.sceneName || '',
2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',

---

2187:     emit studio_log_requested(QStringLiteral("Embedded Product View readiness timeout. %1").arg(detail));
2188:     return;
2189:   }
2190: 
2191:   static const char kStatusScript[] = R"JS(
2192: (() => {
2193:   const s = window.__WORKCELL_VIEWER_STATUS__ || {};
2194:   return {
2195:     readiness_contract_version: Number(s.readiness_contract_version ?? s.readinessContractVersion ?? 0),
2196:     lifecycle_state: s.lifecycle_state || s.lifecycleState || '',
2197:     terminal: Boolean(s.terminal),
2198:     status_sequence: Number(s.status_sequence ?? s.statusSequence ?? 0),
2199:     builder_revision: String(s.builder_revision ?? s.builderRevision ?? ''),
2200:     viewer_boot_state: s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || s.viewerBootState || '',
2201:     scene_name: s.scene_name || s.sceneName || '',
2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',

---

2188:     return;
2189:   }
2190: 
2191:   static const char kStatusScript[] = R"JS(
2192: (() => {
2193:   const s = window.__WORKCELL_VIEWER_STATUS__ || {};
2194:   return {
2195:     readiness_contract_version: Number(s.readiness_contract_version ?? s.readinessContractVersion ?? 0),
2196:     lifecycle_state: s.lifecycle_state || s.lifecycleState || '',
2197:     terminal: Boolean(s.terminal),
2198:     status_sequence: Number(s.status_sequence ?? s.statusSequence ?? 0),
2199:     builder_revision: String(s.builder_revision ?? s.builderRevision ?? ''),
2200:     viewer_boot_state: s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || s.viewerBootState || '',
2201:     scene_name: s.scene_name || s.sceneName || '',
2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',
2212:     camera_status: s.camera_status || s.cameraStatus || '',

---

2191:   static const char kStatusScript[] = R"JS(
2192: (() => {
2193:   const s = window.__WORKCELL_VIEWER_STATUS__ || {};
2194:   return {
2195:     readiness_contract_version: Number(s.readiness_contract_version ?? s.readinessContractVersion ?? 0),
2196:     lifecycle_state: s.lifecycle_state || s.lifecycleState || '',
2197:     terminal: Boolean(s.terminal),
2198:     status_sequence: Number(s.status_sequence ?? s.statusSequence ?? 0),
2199:     builder_revision: String(s.builder_revision ?? s.builderRevision ?? ''),
2200:     viewer_boot_state: s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || s.viewerBootState || '',
2201:     scene_name: s.scene_name || s.sceneName || '',
2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',
2212:     camera_status: s.camera_status || s.cameraStatus || '',
2213:     pending_required_loads: Array.isArray(s.pending_required_loads || s.pendingRequiredLoads) ? (s.pending_required_loads || s.pendingRequiredLoads) : [],
2214:     final_lifecycle_state: s.final_lifecycle_state || s.finalLifecycleState || s.lifecycle_state || s.lifecycleState || '',
2215:     readiness_failure: s.readiness_failure || s.readinessFailure || null,

---

2194:   return {
2195:     readiness_contract_version: Number(s.readiness_contract_version ?? s.readinessContractVersion ?? 0),
2196:     lifecycle_state: s.lifecycle_state || s.lifecycleState || '',
2197:     terminal: Boolean(s.terminal),
2198:     status_sequence: Number(s.status_sequence ?? s.statusSequence ?? 0),
2199:     builder_revision: String(s.builder_revision ?? s.builderRevision ?? ''),
2200:     viewer_boot_state: s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || s.viewerBootState || '',
2201:     scene_name: s.scene_name || s.sceneName || '',
2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',
2212:     camera_status: s.camera_status || s.cameraStatus || '',
2213:     pending_required_loads: Array.isArray(s.pending_required_loads || s.pendingRequiredLoads) ? (s.pending_required_loads || s.pendingRequiredLoads) : [],
2214:     final_lifecycle_state: s.final_lifecycle_state || s.finalLifecycleState || s.lifecycle_state || s.lifecycleState || '',
2215:     readiness_failure: s.readiness_failure || s.readinessFailure || null,
2216:     failed_stage: s.failed_stage || s.failedStage || '',
2217:     fatal_error: s.fatal_error || s.fatalError || '',
2218:     fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')

---

2195:     readiness_contract_version: Number(s.readiness_contract_version ?? s.readinessContractVersion ?? 0),
2196:     lifecycle_state: s.lifecycle_state || s.lifecycleState || '',
2197:     terminal: Boolean(s.terminal),
2198:     status_sequence: Number(s.status_sequence ?? s.statusSequence ?? 0),
2199:     builder_revision: String(s.builder_revision ?? s.builderRevision ?? ''),
2200:     viewer_boot_state: s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || s.viewerBootState || '',
2201:     scene_name: s.scene_name || s.sceneName || '',
2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',
2212:     camera_status: s.camera_status || s.cameraStatus || '',
2213:     pending_required_loads: Array.isArray(s.pending_required_loads || s.pendingRequiredLoads) ? (s.pending_required_loads || s.pendingRequiredLoads) : [],
2214:     final_lifecycle_state: s.final_lifecycle_state || s.finalLifecycleState || s.lifecycle_state || s.lifecycleState || '',
2215:     readiness_failure: s.readiness_failure || s.readinessFailure || null,
2216:     failed_stage: s.failed_stage || s.failedStage || '',
2217:     fatal_error: s.fatal_error || s.fatalError || '',
2218:     fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')
2219:   };

---

2196:     lifecycle_state: s.lifecycle_state || s.lifecycleState || '',
2197:     terminal: Boolean(s.terminal),
2198:     status_sequence: Number(s.status_sequence ?? s.statusSequence ?? 0),
2199:     builder_revision: String(s.builder_revision ?? s.builderRevision ?? ''),
2200:     viewer_boot_state: s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || s.viewerBootState || '',
2201:     scene_name: s.scene_name || s.sceneName || '',
2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',
2212:     camera_status: s.camera_status || s.cameraStatus || '',
2213:     pending_required_loads: Array.isArray(s.pending_required_loads || s.pendingRequiredLoads) ? (s.pending_required_loads || s.pendingRequiredLoads) : [],
2214:     final_lifecycle_state: s.final_lifecycle_state || s.finalLifecycleState || s.lifecycle_state || s.lifecycleState || '',
2215:     readiness_failure: s.readiness_failure || s.readinessFailure || null,
2216:     failed_stage: s.failed_stage || s.failedStage || '',
2217:     fatal_error: s.fatal_error || s.fatalError || '',
2218:     fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')
2219:   };
2220: })()

---

2197:     terminal: Boolean(s.terminal),
2198:     status_sequence: Number(s.status_sequence ?? s.statusSequence ?? 0),
2199:     builder_revision: String(s.builder_revision ?? s.builderRevision ?? ''),
2200:     viewer_boot_state: s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || s.viewerBootState || '',
2201:     scene_name: s.scene_name || s.sceneName || '',
2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',
2212:     camera_status: s.camera_status || s.cameraStatus || '',
2213:     pending_required_loads: Array.isArray(s.pending_required_loads || s.pendingRequiredLoads) ? (s.pending_required_loads || s.pendingRequiredLoads) : [],
2214:     final_lifecycle_state: s.final_lifecycle_state || s.finalLifecycleState || s.lifecycle_state || s.lifecycleState || '',
2215:     readiness_failure: s.readiness_failure || s.readinessFailure || null,
2216:     failed_stage: s.failed_stage || s.failedStage || '',
2217:     fatal_error: s.fatal_error || s.fatalError || '',
2218:     fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')
2219:   };
2220: })()
2221: )JS";

---

2198:     status_sequence: Number(s.status_sequence ?? s.statusSequence ?? 0),
2199:     builder_revision: String(s.builder_revision ?? s.builderRevision ?? ''),
2200:     viewer_boot_state: s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || s.viewerBootState || '',
2201:     scene_name: s.scene_name || s.sceneName || '',
2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',
2212:     camera_status: s.camera_status || s.cameraStatus || '',
2213:     pending_required_loads: Array.isArray(s.pending_required_loads || s.pendingRequiredLoads) ? (s.pending_required_loads || s.pendingRequiredLoads) : [],
2214:     final_lifecycle_state: s.final_lifecycle_state || s.finalLifecycleState || s.lifecycle_state || s.lifecycleState || '',
2215:     readiness_failure: s.readiness_failure || s.readinessFailure || null,
2216:     failed_stage: s.failed_stage || s.failedStage || '',
2217:     fatal_error: s.fatal_error || s.fatalError || '',
2218:     fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')
2219:   };
2220: })()
2221: )JS";
2222:   embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kStatusScript), [this, identity, navigation_token, readiness_token, expected_json_path, viewer_url](const QVariant & value) {

---

2199:     builder_revision: String(s.builder_revision ?? s.builderRevision ?? ''),
2200:     viewer_boot_state: s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || s.viewerBootState || '',
2201:     scene_name: s.scene_name || s.sceneName || '',
2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',
2212:     camera_status: s.camera_status || s.cameraStatus || '',
2213:     pending_required_loads: Array.isArray(s.pending_required_loads || s.pendingRequiredLoads) ? (s.pending_required_loads || s.pendingRequiredLoads) : [],
2214:     final_lifecycle_state: s.final_lifecycle_state || s.finalLifecycleState || s.lifecycle_state || s.lifecycleState || '',
2215:     readiness_failure: s.readiness_failure || s.readinessFailure || null,
2216:     failed_stage: s.failed_stage || s.failedStage || '',
2217:     fatal_error: s.fatal_error || s.fatalError || '',
2218:     fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')
2219:   };
2220: })()
2221: )JS";
2222:   embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kStatusScript), [this, identity, navigation_token, readiness_token, expected_json_path, viewer_url](const QVariant & value) {
2223:     if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_ ||

---

2200:     viewer_boot_state: s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || s.viewerBootState || '',
2201:     scene_name: s.scene_name || s.sceneName || '',
2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',
2212:     camera_status: s.camera_status || s.cameraStatus || '',
2213:     pending_required_loads: Array.isArray(s.pending_required_loads || s.pendingRequiredLoads) ? (s.pending_required_loads || s.pendingRequiredLoads) : [],
2214:     final_lifecycle_state: s.final_lifecycle_state || s.finalLifecycleState || s.lifecycle_state || s.lifecycleState || '',
2215:     readiness_failure: s.readiness_failure || s.readinessFailure || null,
2216:     failed_stage: s.failed_stage || s.failedStage || '',
2217:     fatal_error: s.fatal_error || s.fatalError || '',
2218:     fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')
2219:   };
2220: })()
2221: )JS";
2222:   embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kStatusScript), [this, identity, navigation_token, readiness_token, expected_json_path, viewer_url](const QVariant & value) {
2223:     if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_ ||
2224:         readiness_token != embedded_web_readiness_token_ ||

---

2201:     scene_name: s.scene_name || s.sceneName || '',
2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',
2212:     camera_status: s.camera_status || s.cameraStatus || '',
2213:     pending_required_loads: Array.isArray(s.pending_required_loads || s.pendingRequiredLoads) ? (s.pending_required_loads || s.pendingRequiredLoads) : [],
2214:     final_lifecycle_state: s.final_lifecycle_state || s.finalLifecycleState || s.lifecycle_state || s.lifecycleState || '',
2215:     readiness_failure: s.readiness_failure || s.readinessFailure || null,
2216:     failed_stage: s.failed_stage || s.failedStage || '',
2217:     fatal_error: s.fatal_error || s.fatalError || '',
2218:     fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')
2219:   };
2220: })()
2221: )JS";
2222:   embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kStatusScript), [this, identity, navigation_token, readiness_token, expected_json_path, viewer_url](const QVariant & value) {
2223:     if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_ ||
2224:         readiness_token != embedded_web_readiness_token_ ||
2225:         embedded_web_view_->url() != QUrl(viewer_url)) {

---

2202:     source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
2203:     scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',
2212:     camera_status: s.camera_status || s.cameraStatus || '',
2213:     pending_required_loads: Array.isArray(s.pending_required_loads || s.pendingRequiredLoads) ? (s.pending_required_loads || s.pendingRequiredLoads) : [],
2214:     final_lifecycle_state: s.final_lifecycle_state || s.finalLifecycleState || s.lifecycle_state || s.lifecycleState || '',
2215:     readiness_failure: s.readiness_failure || s.readinessFailure || null,
2216:     failed_stage: s.failed_stage || s.failedStage || '',
2217:     fatal_error: s.fatal_error || s.fatalError || '',
2218:     fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')
2219:   };
2220: })()
2221: )JS";
2222:   embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kStatusScript), [this, identity, navigation_token, readiness_token, expected_json_path, viewer_url](const QVariant & value) {
2223:     if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_ ||
2224:         readiness_token != embedded_web_readiness_token_ ||
2225:         embedded_web_view_->url() != QUrl(viewer_url)) {
2226:       emit studio_log_requested(QStringLiteral("Ignored stale Embedded Product View readiness result for scene %1 revision %2 navigation %3.")

---

2204:     robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',
2212:     camera_status: s.camera_status || s.cameraStatus || '',
2213:     pending_required_loads: Array.isArray(s.pending_required_loads || s.pendingRequiredLoads) ? (s.pending_required_loads || s.pendingRequiredLoads) : [],
2214:     final_lifecycle_state: s.final_lifecycle_state || s.finalLifecycleState || s.lifecycle_state || s.lifecycleState || '',
2215:     readiness_failure: s.readiness_failure || s.readinessFailure || null,
2216:     failed_stage: s.failed_stage || s.failedStage || '',
2217:     fatal_error: s.fatal_error || s.fatalError || '',
2218:     fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')
2219:   };
2220: })()
2221: )JS";
2222:   embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kStatusScript), [this, identity, navigation_token, readiness_token, expected_json_path, viewer_url](const QVariant & value) {
2223:     if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_ ||
2224:         readiness_token != embedded_web_readiness_token_ ||
2225:         embedded_web_view_->url() != QUrl(viewer_url)) {
2226:       emit studio_log_requested(QStringLiteral("Ignored stale Embedded Product View readiness result for scene %1 revision %2 navigation %3.")
2227:         .arg(identity.scene_id).arg(identity.generation).arg(navigation_token));
2228:       return;

---

2205:     scene_id: s.scene_id || s.sceneId || '',
2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',
2212:     camera_status: s.camera_status || s.cameraStatus || '',
2213:     pending_required_loads: Array.isArray(s.pending_required_loads || s.pendingRequiredLoads) ? (s.pending_required_loads || s.pendingRequiredLoads) : [],
2214:     final_lifecycle_state: s.final_lifecycle_state || s.finalLifecycleState || s.lifecycle_state || s.lifecycleState || '',
2215:     readiness_failure: s.readiness_failure || s.readinessFailure || null,
2216:     failed_stage: s.failed_stage || s.failedStage || '',
2217:     fatal_error: s.fatal_error || s.fatalError || '',
2218:     fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')
2219:   };
2220: })()
2221: )JS";
2222:   embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kStatusScript), [this, identity, navigation_token, readiness_token, expected_json_path, viewer_url](const QVariant & value) {
2223:     if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_ ||
2224:         readiness_token != embedded_web_readiness_token_ ||
2225:         embedded_web_view_->url() != QUrl(viewer_url)) {
2226:       emit studio_log_requested(QStringLiteral("Ignored stale Embedded Product View readiness result for scene %1 revision %2 navigation %3.")
2227:         .arg(identity.scene_id).arg(identity.generation).arg(navigation_token));
2228:       return;
2229:     }

---

2206:     expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
2207:     rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
2208:     failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
2209:     robot_status: s.robot_status || s.robotStatus || '',
2210:     tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
2211:     environment_status: s.environment_status || s.environmentStatus || '',
2212:     camera_status: s.camera_status || s.cameraStatus || '',
2213:     pending_required_loads: Array.isArray(s.pending_required_loads || s.pendingRequiredLoads) ? (s.pending_required_loads || s.pendingRequiredLoads) : [],
2214:     final_lifecycle_state: s.final_lifecycle_state || s.finalLifecycleState || s.lifecycle_state || s.lifecycleState || '',
2215:     readiness_failure: s.readiness_failure || s.readinessFailure || null,
2216:     failed_stage: s.failed_stage || s.failedStage || '',
2217:     fatal_error: s.fatal_error || s.fatalError || '',
2218:     fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')
2219:   };
2220: })()
2221: )JS";
2222:   embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kStatusScript), [this, identity, navigation_token, readiness_token, expected_json_path, viewer_url](const QVariant & value) {
2223:     if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_ ||
2224:         readiness_token != embedded_web_readiness_token_ ||
2225:         embedded_web_view_->url() != QUrl(viewer_url)) {
2226:       emit studio_log_requested(QStringLiteral("Ignored stale Embedded Product View readiness result for scene %1 revision %2 navigation %3.")
2227:         .arg(identity.scene_id).arg(identity.generation).arg(navigation_token));
2228:       return;
2229:     }
2230:     const QVariantMap status = value.toMap();

---

2349:   }
2350:   if (embedded_web_server_lifecycle_ != EmbeddedWebServerLifecycle::ServerReady ||
2351:       identity.absolute_repo_root.isEmpty() || identity.selected_server_port <= 0) return;
2352:   const QString web_scene_url_path = QStringLiteral("build/workcell_studio_web_scene/%1.web_scene.json").arg(identity.scene_id);
2353:   const QString builder_revision = QString::number(identity.payload_revision);
2354:   QUrl viewer_url;
2355:   viewer_url.setScheme(QStringLiteral("http"));
2356:   viewer_url.setHost(QStringLiteral("127.0.0.1"));
2357:   viewer_url.setPort(identity.selected_server_port);
2358:   viewer_url.setPath(QStringLiteral("/workcell_studio_web/viewer/index.html"));
2359:   QUrlQuery viewer_query;
2360:   viewer_query.addQueryItem(QStringLiteral("scene"), web_scene_url_path);
2361:   viewer_query.addQueryItem(QStringLiteral("builderRevision"), builder_revision);
2362:   viewer_query.addQueryItem(QStringLiteral("embedded"), QStringLiteral("1"));
2363:   viewer_url.setQuery(viewer_query);
2364: 
2365:   const QUrlQuery decoded_viewer_query(viewer_url);
2366:   const QString decoded_scene_path = decoded_viewer_query.queryItemValue(QStringLiteral("scene"), QUrl::FullyDecoded);
2367:   const QString decoded_builder_revision = decoded_viewer_query.queryItemValue(
2368:     QStringLiteral("builderRevision"), QUrl::FullyDecoded);
2369:   const bool valid_scene_path = decoded_scene_path == web_scene_url_path;
2370:   const bool valid_builder_revision = QRegularExpression(QStringLiteral("^[0-9]+$"))
2371:     .match(decoded_builder_revision).hasMatch();
2372:   if (!valid_scene_path || !valid_builder_revision) {
2373:     const QString detail = QStringLiteral("Embedded Product View URL validation failed; using native compatibility preview.");

---

2356:   viewer_url.setHost(QStringLiteral("127.0.0.1"));
2357:   viewer_url.setPort(identity.selected_server_port);
2358:   viewer_url.setPath(QStringLiteral("/workcell_studio_web/viewer/index.html"));
2359:   QUrlQuery viewer_query;
2360:   viewer_query.addQueryItem(QStringLiteral("scene"), web_scene_url_path);
2361:   viewer_query.addQueryItem(QStringLiteral("builderRevision"), builder_revision);
2362:   viewer_query.addQueryItem(QStringLiteral("embedded"), QStringLiteral("1"));
2363:   viewer_url.setQuery(viewer_query);
2364: 
2365:   const QUrlQuery decoded_viewer_query(viewer_url);
2366:   const QString decoded_scene_path = decoded_viewer_query.queryItemValue(QStringLiteral("scene"), QUrl::FullyDecoded);
2367:   const QString decoded_builder_revision = decoded_viewer_query.queryItemValue(
2368:     QStringLiteral("builderRevision"), QUrl::FullyDecoded);
2369:   const bool valid_scene_path = decoded_scene_path == web_scene_url_path;
2370:   const bool valid_builder_revision = QRegularExpression(QStringLiteral("^[0-9]+$"))
2371:     .match(decoded_builder_revision).hasMatch();
2372:   if (!valid_scene_path || !valid_builder_revision) {
2373:     const QString detail = QStringLiteral("Embedded Product View URL validation failed; using native compatibility preview.");
2374:     emit studio_log_requested(detail);
2375:     activate_native_compatibility_preview(detail);
2376:     return;
2377:   }
2378:   if (embedded_web_prepared_identity_.matches_effective_request(identity) ||
2379:       embedded_web_loading_identity_.matches_effective_request(identity)) {
2380:     ++embedded_web_duplicate_requests_coalesced_;
```

# Existing static-test contexts
```text
335:         check=True,
336:         capture_output=True,
337:         text=True,
338:     )
339: 
340: 
341: def test_configured_camera_readiness_identity_is_shared_across_authored_and_generated_rows():
342:     js_path = VIEWER / "viewer.js"
343:     harness = r"""
344: const fs = require('fs');
345: const vm = require('vm');
346: const assert = require('assert');
347: let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');

---

346: const assert = require('assert');
347: let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
348: const element = () => ({ hidden:false, checked:false, disabled:false, textContent:'', className:'', innerHTML:'', classList:{toggle(){}}, querySelector(){return null;}, appendChild(){}, addEventListener(){}, setAttribute(){} });
349: const context = { console, assert, window:{events:[], location:{search:''}, dispatchEvent(event){this.events.push(event.detail);}, parent:{postMessage(){}}}, document:{getElementById(){return element();},createElement(){return element();}}, URLSearchParams, CustomEvent:function(type, init){return {type, detail:init?.detail || {}};}, requestAnimationFrame(){return 0;}, setTimeout(){return 0;}, clearTimeout(){} };
350: vm.createContext(context);
351: vm.runInContext(source + `
352: const authoredCamera = { id:'realsense_overhead', category:'camera', readiness_category:'configured_camera', render_policy:'primary', mesh_uri:'camera.dae' };
353: const generatedCamera = { id:'generated_urdf::camera_link::visual_17', camera_id:'realsense_overhead', category:'camera', readiness_category:'configured_camera', render_policy:'primary', mesh_uri:'camera.dae' };
354: assert.strictEqual(readinessIdentityForItem('configured_camera', authoredCamera), 'realsense_overhead');
355: assert.strictEqual(readinessIdentityForItem('configured_camera', generatedCamera), 'realsense_overhead');
356: assert.strictEqual(readinessKey('configured_camera', authoredCamera), readinessKey('configured_camera', generatedCamera));
357: 
358: state.sceneJson = {scene:{id:'ur5_2f_test'}};

---

347: let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
348: const element = () => ({ hidden:false, checked:false, disabled:false, textContent:'', className:'', innerHTML:'', classList:{toggle(){}}, querySelector(){return null;}, appendChild(){}, addEventListener(){}, setAttribute(){} });
349: const context = { console, assert, window:{events:[], location:{search:''}, dispatchEvent(event){this.events.push(event.detail);}, parent:{postMessage(){}}}, document:{getElementById(){return element();},createElement(){return element();}}, URLSearchParams, CustomEvent:function(type, init){return {type, detail:init?.detail || {}};}, requestAnimationFrame(){return 0;}, setTimeout(){return 0;}, clearTimeout(){} };
350: vm.createContext(context);
351: vm.runInContext(source + `
352: const authoredCamera = { id:'realsense_overhead', category:'camera', readiness_category:'configured_camera', render_policy:'primary', mesh_uri:'camera.dae' };
353: const generatedCamera = { id:'generated_urdf::camera_link::visual_17', camera_id:'realsense_overhead', category:'camera', readiness_category:'configured_camera', render_policy:'primary', mesh_uri:'camera.dae' };
354: assert.strictEqual(readinessIdentityForItem('configured_camera', authoredCamera), 'realsense_overhead');
355: assert.strictEqual(readinessIdentityForItem('configured_camera', generatedCamera), 'realsense_overhead');
356: assert.strictEqual(readinessKey('configured_camera', authoredCamera), readinessKey('configured_camera', generatedCamera));
357: 
358: state.sceneJson = {scene:{id:'ur5_2f_test'}};
359: beginWeb3dSceneReadiness([authoredCamera, generatedCamera]);

---

348: const element = () => ({ hidden:false, checked:false, disabled:false, textContent:'', className:'', innerHTML:'', classList:{toggle(){}}, querySelector(){return null;}, appendChild(){}, addEventListener(){}, setAttribute(){} });
349: const context = { console, assert, window:{events:[], location:{search:''}, dispatchEvent(event){this.events.push(event.detail);}, parent:{postMessage(){}}}, document:{getElementById(){return element();},createElement(){return element();}}, URLSearchParams, CustomEvent:function(type, init){return {type, detail:init?.detail || {}};}, requestAnimationFrame(){return 0;}, setTimeout(){return 0;}, clearTimeout(){} };
350: vm.createContext(context);
351: vm.runInContext(source + `
352: const authoredCamera = { id:'realsense_overhead', category:'camera', readiness_category:'configured_camera', render_policy:'primary', mesh_uri:'camera.dae' };
353: const generatedCamera = { id:'generated_urdf::camera_link::visual_17', camera_id:'realsense_overhead', category:'camera', readiness_category:'configured_camera', render_policy:'primary', mesh_uri:'camera.dae' };
354: assert.strictEqual(readinessIdentityForItem('configured_camera', authoredCamera), 'realsense_overhead');
355: assert.strictEqual(readinessIdentityForItem('configured_camera', generatedCamera), 'realsense_overhead');
356: assert.strictEqual(readinessKey('configured_camera', authoredCamera), readinessKey('configured_camera', generatedCamera));
357: 
358: state.sceneJson = {scene:{id:'ur5_2f_test'}};
359: beginWeb3dSceneReadiness([authoredCamera, generatedCamera]);
360: assert.deepStrictEqual(Array.from(state.web3dReadiness.pending), []);

---

349: const context = { console, assert, window:{events:[], location:{search:''}, dispatchEvent(event){this.events.push(event.detail);}, parent:{postMessage(){}}}, document:{getElementById(){return element();},createElement(){return element();}}, URLSearchParams, CustomEvent:function(type, init){return {type, detail:init?.detail || {}};}, requestAnimationFrame(){return 0;}, setTimeout(){return 0;}, clearTimeout(){} };
350: vm.createContext(context);
351: vm.runInContext(source + `
352: const authoredCamera = { id:'realsense_overhead', category:'camera', readiness_category:'configured_camera', render_policy:'primary', mesh_uri:'camera.dae' };
353: const generatedCamera = { id:'generated_urdf::camera_link::visual_17', camera_id:'realsense_overhead', category:'camera', readiness_category:'configured_camera', render_policy:'primary', mesh_uri:'camera.dae' };
354: assert.strictEqual(readinessIdentityForItem('configured_camera', authoredCamera), 'realsense_overhead');
355: assert.strictEqual(readinessIdentityForItem('configured_camera', generatedCamera), 'realsense_overhead');
356: assert.strictEqual(readinessKey('configured_camera', authoredCamera), readinessKey('configured_camera', generatedCamera));
357: 
358: state.sceneJson = {scene:{id:'ur5_2f_test'}};
359: beginWeb3dSceneReadiness([authoredCamera, generatedCamera]);
360: assert.deepStrictEqual(Array.from(state.web3dReadiness.pending), []);
361: const cameraOperation = registerReadinessOperation([readinessKey('configured_camera', generatedCamera)]);

---

350: vm.createContext(context);
351: vm.runInContext(source + `
352: const authoredCamera = { id:'realsense_overhead', category:'camera', readiness_category:'configured_camera', render_policy:'primary', mesh_uri:'camera.dae' };
353: const generatedCamera = { id:'generated_urdf::camera_link::visual_17', camera_id:'realsense_overhead', category:'camera', readiness_category:'configured_camera', render_policy:'primary', mesh_uri:'camera.dae' };
354: assert.strictEqual(readinessIdentityForItem('configured_camera', authoredCamera), 'realsense_overhead');
355: assert.strictEqual(readinessIdentityForItem('configured_camera', generatedCamera), 'realsense_overhead');
356: assert.strictEqual(readinessKey('configured_camera', authoredCamera), readinessKey('configured_camera', generatedCamera));
357: 
358: state.sceneJson = {scene:{id:'ur5_2f_test'}};
359: beginWeb3dSceneReadiness([authoredCamera, generatedCamera]);
360: assert.deepStrictEqual(Array.from(state.web3dReadiness.pending), []);
361: const cameraOperation = registerReadinessOperation([readinessKey('configured_camera', generatedCamera)]);
362: assert.deepStrictEqual(Array.from(state.web3dReadiness.pending), ['configured_camera:realsense_overhead']);

---

355: assert.strictEqual(readinessIdentityForItem('configured_camera', generatedCamera), 'realsense_overhead');
356: assert.strictEqual(readinessKey('configured_camera', authoredCamera), readinessKey('configured_camera', generatedCamera));
357: 
358: state.sceneJson = {scene:{id:'ur5_2f_test'}};
359: beginWeb3dSceneReadiness([authoredCamera, generatedCamera]);
360: assert.deepStrictEqual(Array.from(state.web3dReadiness.pending), []);
361: const cameraOperation = registerReadinessOperation([readinessKey('configured_camera', generatedCamera)]);
362: assert.deepStrictEqual(Array.from(state.web3dReadiness.pending), ['configured_camera:realsense_overhead']);
363: completeReadinessOperation(cameraOperation);
364: assert.strictEqual(state.web3dReadiness.pending.size, 0);
365: 
366: state.web3dReadiness = {state:'scene_loading', terminal:false, terminalState:'', terminalNavigationKey:'', terminalEmissionCount:0, statusSequence:0, required:{configured_camera:true}, pending:new Set(['configured_camera:realsense_overhead']), failed:false, failure:null};
367: failWeb3dSceneReadiness(generatedCamera, 'camera.dae', 'camera mesh failed');

---

364: assert.strictEqual(state.web3dReadiness.pending.size, 0);
365: 
366: state.web3dReadiness = {state:'scene_loading', terminal:false, terminalState:'', terminalNavigationKey:'', terminalEmissionCount:0, statusSequence:0, required:{configured_camera:true}, pending:new Set(['configured_camera:realsense_overhead']), failed:false, failure:null};
367: failWeb3dSceneReadiness(generatedCamera, 'camera.dae', 'camera mesh failed');
368: const failure = window.events.at(-1);
369: assert.strictEqual(failure.required_category, 'configured_camera');
370: assert.strictEqual(failure.readiness_identity, 'realsense_overhead');
371: assert.strictEqual(failure.readiness_key, 'configured_camera:realsense_overhead');
372: assert.deepStrictEqual(failure.pending_required_loads, ['configured_camera:realsense_overhead']);
373: 
374: const unchanged = [
375:   ['robot_arm', {id:'ur5'}, 'robot_arm:ur5'],
376:   ['attached_tool_gripper', {id:'robotiq_85'}, 'attached_tool_gripper:robotiq_85'],

---

365: 
366: state.web3dReadiness = {state:'scene_loading', terminal:false, terminalState:'', terminalNavigationKey:'', terminalEmissionCount:0, statusSequence:0, required:{configured_camera:true}, pending:new Set(['configured_camera:realsense_overhead']), failed:false, failure:null};
367: failWeb3dSceneReadiness(generatedCamera, 'camera.dae', 'camera mesh failed');
368: const failure = window.events.at(-1);
369: assert.strictEqual(failure.required_category, 'configured_camera');
370: assert.strictEqual(failure.readiness_identity, 'realsense_overhead');
371: assert.strictEqual(failure.readiness_key, 'configured_camera:realsense_overhead');
372: assert.deepStrictEqual(failure.pending_required_loads, ['configured_camera:realsense_overhead']);
373: 
374: const unchanged = [
375:   ['robot_arm', {id:'ur5'}, 'robot_arm:ur5'],
376:   ['attached_tool_gripper', {id:'robotiq_85'}, 'attached_tool_gripper:robotiq_85'],
377:   ['workbench_support_surface', {id:'support_surface_table'}, 'workbench_support_surface:support_surface_table'],

---

374: const unchanged = [
375:   ['robot_arm', {id:'ur5'}, 'robot_arm:ur5'],
376:   ['attached_tool_gripper', {id:'robotiq_85'}, 'attached_tool_gripper:robotiq_85'],
377:   ['workbench_support_surface', {id:'support_surface_table'}, 'workbench_support_surface:support_surface_table'],
378:   ['authored_physical_mesh', {id:'target_bin'}, 'authored_physical_mesh:target_bin'],
379: ];
380: for (const [category, item, expected] of unchanged) assert.strictEqual(readinessKey(category, item), expected);
381: `, context);
382: """
383:     subprocess.run(
384:         ["node", "-e", harness, str(js_path)],
385:         cwd=ROOT,
386:         check=True,

---

418:   setRenderInfo = (rendered, status, uri, reason) => { rendered.renderInfo = {render_status:status,mesh_uri:uri,fallback_reason:reason}; };
419:   materializeLoadedMesh = (item, uri, loaded) => loaded;
420:   makeMeshVisualRoot = (item, loaded) => ({loaded,updateMatrixWorld(){}});
421:   maybeApplyMeshUnitAutoscale = () => false;
422:   diagnoseLoadedMeshBounds = () => true;
423:   THREE = {Box3:class {setFromObject(){return this;}}};
424:   const camera = {id:'generated_camera_visual',camera_id:'realsense_overhead',category:'camera',readiness_category:'configured_camera',render_policy:'primary',mesh_uri:'assets/camera.dae',mesh_load_required:true};
425:   const rendered = () => ({object3d:{add(value){this.added=value;}},renderInfo:{render_status:'mesh_loading_required'}});
426:   const fallback = () => ({visible:false});
427:   const readiness = pending => ({state:'scene_loading',terminal:false,terminalState:'',terminalNavigationKey:'',terminalEmissionCount:0,statusSequence:0,required:{robot_arm:true,attached_tool_gripper:true,workbench_support_surface:true,configured_camera:true},pending:new Set(pending),failed:false,failure:null});
428: 
429:   state.sceneJson={scene:{id:'timeout_scene'}}; state.sourceWebSceneFile='timeout.json'; resetSceneLifecycleState();
430:   state.web3dReadiness=readiness(['configured_camera:realsense_overhead']);

---

421:   maybeApplyMeshUnitAutoscale = () => false;
422:   diagnoseLoadedMeshBounds = () => true;
423:   THREE = {Box3:class {setFromObject(){return this;}}};
424:   const camera = {id:'generated_camera_visual',camera_id:'realsense_overhead',category:'camera',readiness_category:'configured_camera',render_policy:'primary',mesh_uri:'assets/camera.dae',mesh_load_required:true};
425:   const rendered = () => ({object3d:{add(value){this.added=value;}},renderInfo:{render_status:'mesh_loading_required'}});
426:   const fallback = () => ({visible:false});
427:   const readiness = pending => ({state:'scene_loading',terminal:false,terminalState:'',terminalNavigationKey:'',terminalEmissionCount:0,statusSequence:0,required:{robot_arm:true,attached_tool_gripper:true,workbench_support_surface:true,configured_camera:true},pending:new Set(pending),failed:false,failure:null});
428: 
429:   state.sceneJson={scene:{id:'timeout_scene'}}; state.sourceWebSceneFile='timeout.json'; resetSceneLifecycleState();
430:   state.web3dReadiness=readiness(['configured_camera:realsense_overhead']);
431:   ColladaLoader=class {loadAsync(){return new Promise(() => {});}};
432:   const started=Date.now(); await tryLoadMesh(camera,rendered(),fallback());
433:   assert(Date.now()-started < 45000);

---

424:   const camera = {id:'generated_camera_visual',camera_id:'realsense_overhead',category:'camera',readiness_category:'configured_camera',render_policy:'primary',mesh_uri:'assets/camera.dae',mesh_load_required:true};
425:   const rendered = () => ({object3d:{add(value){this.added=value;}},renderInfo:{render_status:'mesh_loading_required'}});
426:   const fallback = () => ({visible:false});
427:   const readiness = pending => ({state:'scene_loading',terminal:false,terminalState:'',terminalNavigationKey:'',terminalEmissionCount:0,statusSequence:0,required:{robot_arm:true,attached_tool_gripper:true,workbench_support_surface:true,configured_camera:true},pending:new Set(pending),failed:false,failure:null});
428: 
429:   state.sceneJson={scene:{id:'timeout_scene'}}; state.sourceWebSceneFile='timeout.json'; resetSceneLifecycleState();
430:   state.web3dReadiness=readiness(['configured_camera:realsense_overhead']);
431:   ColladaLoader=class {loadAsync(){return new Promise(() => {});}};
432:   const started=Date.now(); await tryLoadMesh(camera,rendered(),fallback());
433:   assert(Date.now()-started < 45000);
434:   const failure=events.at(-1);
435:   assert.strictEqual(failure.state,'scene_failed');
436:   assert.strictEqual(failure.required_category,'configured_camera');

---

431:   ColladaLoader=class {loadAsync(){return new Promise(() => {});}};
432:   const started=Date.now(); await tryLoadMesh(camera,rendered(),fallback());
433:   assert(Date.now()-started < 45000);
434:   const failure=events.at(-1);
435:   assert.strictEqual(failure.state,'scene_failed');
436:   assert.strictEqual(failure.required_category,'configured_camera');
437:   assert.strictEqual(failure.readiness_identity,'realsense_overhead');
438:   assert.strictEqual(failure.readiness_key,'configured_camera:realsense_overhead');
439:   assert.strictEqual(failure.url,'assets/camera.dae');
440:   assert.strictEqual(failure.loader,'ColladaLoader');
441:   assert.match(failure.reason,/timed out/);
442:   assert.strictEqual(failure.timeout_ms,20);
443:   assert.deepStrictEqual(failure.pending_required_loads,['configured_camera:realsense_overhead']);

---

432:   const started=Date.now(); await tryLoadMesh(camera,rendered(),fallback());
433:   assert(Date.now()-started < 45000);
434:   const failure=events.at(-1);
435:   assert.strictEqual(failure.state,'scene_failed');
436:   assert.strictEqual(failure.required_category,'configured_camera');
437:   assert.strictEqual(failure.readiness_identity,'realsense_overhead');
438:   assert.strictEqual(failure.readiness_key,'configured_camera:realsense_overhead');
439:   assert.strictEqual(failure.url,'assets/camera.dae');
440:   assert.strictEqual(failure.loader,'ColladaLoader');
441:   assert.match(failure.reason,/timed out/);
442:   assert.strictEqual(failure.timeout_ms,20);
443:   assert.deepStrictEqual(failure.pending_required_loads,['configured_camera:realsense_overhead']);
444: 

---

440:   assert.strictEqual(failure.loader,'ColladaLoader');
441:   assert.match(failure.reason,/timed out/);
442:   assert.strictEqual(failure.timeout_ms,20);
443:   assert.deepStrictEqual(failure.pending_required_loads,['configured_camera:realsense_overhead']);
444: 
445:   events.length=0; state.sceneJson={scene:{id:'success_scene'}}; state.sourceWebSceneFile='success.json'; resetSceneLifecycleState();
446:   state.web3dReadiness=readiness(['configured_camera:realsense_overhead']);
447:   ColladaLoader=class {loadAsync(){return Promise.resolve({mesh:true});}};
448:   await tryLoadMesh(camera,rendered(),fallback());
449:   assert.strictEqual(state.web3dReadiness.pending.size,0);
450:   await new Promise(resolve => setTimeout(resolve,35));
451:   assert.strictEqual(events.filter(event => event.state==='scene_failed').length,0);
452: 

---

448:   await tryLoadMesh(camera,rendered(),fallback());
449:   assert.strictEqual(state.web3dReadiness.pending.size,0);
450:   await new Promise(resolve => setTimeout(resolve,35));
451:   assert.strictEqual(events.filter(event => event.state==='scene_failed').length,0);
452: 
453:   let resolveA; events.length=0; state.sceneJson={scene:{id:'scene_a'}}; state.sourceWebSceneFile='a.json'; resetSceneLifecycleState();
454:   state.web3dReadiness=readiness(['configured_camera:realsense_overhead']);
455:   ColladaLoader=class {loadAsync(){return new Promise(resolve => {resolveA=resolve;});}};
456:   const staleSuccess=tryLoadMesh(camera,rendered(),fallback()); await Promise.resolve(); await Promise.resolve();
457:   state.sceneJson={scene:{id:'scene_b'}}; state.sourceWebSceneFile='b.json'; resetSceneLifecycleState();
458:   state.web3dReadiness=readiness(['configured_camera:replacement_camera']);
459:   resolveA({mesh:true}); await staleSuccess;
460:   assert.deepStrictEqual(Array.from(state.web3dReadiness.pending),['configured_camera:replacement_camera']);

---

452: 
453:   let resolveA; events.length=0; state.sceneJson={scene:{id:'scene_a'}}; state.sourceWebSceneFile='a.json'; resetSceneLifecycleState();
454:   state.web3dReadiness=readiness(['configured_camera:realsense_overhead']);
455:   ColladaLoader=class {loadAsync(){return new Promise(resolve => {resolveA=resolve;});}};
456:   const staleSuccess=tryLoadMesh(camera,rendered(),fallback()); await Promise.resolve(); await Promise.resolve();
457:   state.sceneJson={scene:{id:'scene_b'}}; state.sourceWebSceneFile='b.json'; resetSceneLifecycleState();
458:   state.web3dReadiness=readiness(['configured_camera:replacement_camera']);
459:   resolveA({mesh:true}); await staleSuccess;
460:   assert.deepStrictEqual(Array.from(state.web3dReadiness.pending),['configured_camera:replacement_camera']);
461: 
462:   let rejectA; events.length=0; state.sceneJson={scene:{id:'scene_a'}}; state.sourceWebSceneFile='a.json'; resetSceneLifecycleState();
463:   state.web3dReadiness=readiness(['configured_camera:realsense_overhead']);
464:   ColladaLoader=class {loadAsync(){return new Promise((resolve,reject) => {rejectA=reject;});}};

---

457:   state.sceneJson={scene:{id:'scene_b'}}; state.sourceWebSceneFile='b.json'; resetSceneLifecycleState();
458:   state.web3dReadiness=readiness(['configured_camera:replacement_camera']);
459:   resolveA({mesh:true}); await staleSuccess;
460:   assert.deepStrictEqual(Array.from(state.web3dReadiness.pending),['configured_camera:replacement_camera']);
461: 
462:   let rejectA; events.length=0; state.sceneJson={scene:{id:'scene_a'}}; state.sourceWebSceneFile='a.json'; resetSceneLifecycleState();
463:   state.web3dReadiness=readiness(['configured_camera:realsense_overhead']);
464:   ColladaLoader=class {loadAsync(){return new Promise((resolve,reject) => {rejectA=reject;});}};
465:   const staleError=tryLoadMesh(camera,rendered(),fallback()); await Promise.resolve(); await Promise.resolve();
466:   state.sceneJson={scene:{id:'scene_b'}}; state.sourceWebSceneFile='b.json'; resetSceneLifecycleState();
467:   state.web3dReadiness=readiness(['configured_camera:replacement_camera']);
468:   rejectA(new Error('scene A failed')); await staleError;
469:   assert.strictEqual(events.filter(event => event.state==='scene_failed').length,0);

---

461: 
462:   let rejectA; events.length=0; state.sceneJson={scene:{id:'scene_a'}}; state.sourceWebSceneFile='a.json'; resetSceneLifecycleState();
463:   state.web3dReadiness=readiness(['configured_camera:realsense_overhead']);
464:   ColladaLoader=class {loadAsync(){return new Promise((resolve,reject) => {rejectA=reject;});}};
465:   const staleError=tryLoadMesh(camera,rendered(),fallback()); await Promise.resolve(); await Promise.resolve();
466:   state.sceneJson={scene:{id:'scene_b'}}; state.sourceWebSceneFile='b.json'; resetSceneLifecycleState();
467:   state.web3dReadiness=readiness(['configured_camera:replacement_camera']);
468:   rejectA(new Error('scene A failed')); await staleError;
469:   assert.strictEqual(events.filter(event => event.state==='scene_failed').length,0);
470: 
471:   events.length=0; const optional={id:'optional_fixture',category:'fixture',render_policy:'primary',mesh_uri:'assets/fixture.obj'};
472:   state.sceneJson={scene:{id:'optional_scene'}}; state.sourceWebSceneFile='optional.json'; resetSceneLifecycleState();
473:   state.web3dReadiness=readiness([]); const optionalFallback=fallback();

---

467:   state.web3dReadiness=readiness(['configured_camera:replacement_camera']);
468:   rejectA(new Error('scene A failed')); await staleError;
469:   assert.strictEqual(events.filter(event => event.state==='scene_failed').length,0);
470: 
471:   events.length=0; const optional={id:'optional_fixture',category:'fixture',render_policy:'primary',mesh_uri:'assets/fixture.obj'};
472:   state.sceneJson={scene:{id:'optional_scene'}}; state.sourceWebSceneFile='optional.json'; resetSceneLifecycleState();
473:   state.web3dReadiness=readiness([]); const optionalFallback=fallback();
474:   OBJLoader=class {load(url,onLoad,onProgress,onError){}};
475:   await tryLoadMesh(optional,rendered(),optionalFallback);
476:   assert.strictEqual(optionalFallback.visible,true);
477:   assert.strictEqual(events.filter(event => event.state==='scene_failed').length,0);
478: })().catch(error => { console.error(error); process.exitCode=1; });
479: `, context);

---

706:     assert "state.objects = [];" in js
707:     assert "state.robotPreviewResult = null;" in js
708:     assert "state.robotUrdfPreviewDiagnostics = {};" in js
709: 
710:     assert "const loadToken = ++robotPreviewLoadToken;" in load_body
711:     assert "const loadSceneId = sceneId();" in load_body
712:     assert "readinessOperationIsCurrent(readinessOperation) && loadSceneId === sceneId()" in load_body
713:     assert "sceneId: loadSceneId" in load_body
714:     assert "if (callbackIsCurrent()) state.three.scene?.add?.(root);" in load_body
715:     assert "if (callbackIsCurrent()) state.assemblyRoots.push(root);" in load_body
716:     assert "Ignored stale robot preview completion." in load_body
717:     assert "readinessOperationDiagnostic(readinessOperation, 'stale_replacement'" in load_body
718: 

---

711:     assert "const loadSceneId = sceneId();" in load_body
712:     assert "readinessOperationIsCurrent(readinessOperation) && loadSceneId === sceneId()" in load_body
713:     assert "sceneId: loadSceneId" in load_body
714:     assert "if (callbackIsCurrent()) state.three.scene?.add?.(root);" in load_body
715:     assert "if (callbackIsCurrent()) state.assemblyRoots.push(root);" in load_body
716:     assert "Ignored stale robot preview completion." in load_body
717:     assert "readinessOperationDiagnostic(readinessOperation, 'stale_replacement'" in load_body
718: 
719:     guard_token = "if (!callbackIsCurrent()) return ignoreStaleCallback();"
720:     callback_mutations = {
721:         "onRobotLoaded: result => {": "state.robotPreviewResult = result;",
722:         "onRobotMeshLoaded: () => {": "renderSceneSummary();",
723:         "onRobotMeshLoadError: (err, uri, detail) => {": "finalizeRequiredLoad('failure'",

---

748: appendRuntimeWarning = () => {};
749: failIfCanonicalRequiredVisualSetInvalid = () => false;
750: let captured = [];
751: loadRobotPreview = (preview, rendererContext) => {
752:   const root = { name: preview.rootName || 'old_root', children: [], geometry: { dispose() {} }, material: { dispose() {} } };
753:   captured.push({ preview, rendererContext, root });
754:   return { root, links: new Map([['old_link', {}]]), joints: new Map([['old_joint', {}]]), diagnostics: { robot_preview_lifecycle_state: 'loading', robotPreviewLifecycleState: 'loading', scene: sceneId() }, ready: Promise.resolve(root) };
755: };
756: function makeSceneRecorder(id) { return { id, added: [], removed: [], add(root) { this.added.push(root); }, remove(root) { this.removed.push(root); } }; }
757: function beginScene(id, sceneExists = true) {
758:   state.sceneJson = { scene: { id }, robot_preview: { urdf_url: id + '.urdf', rootName: id + '_root' } };
759:   state.three.scene = sceneExists ? makeSceneRecorder(id) : null;
760:   state.web3dReadiness = { state: 'scene_loading', emittedSceneReady: false, required: { robot_arm: true, attached_tool_gripper: true, workbench_support_surface: true, configured_camera: true }, pending: new Set(['robot_arm:expanded_urdf_loader', 'attached_tool_gripper:expanded_urdf_loader']), failed: false, failure: null };

---

769:   state.assemblyRoots = [{ name: 'previous_assembly_root' }];
770:   state.objects = [{ object3d: { name: 'previous_flattened_fallback' }, item: { id: 'old' } }];
771: }
772: function fireOldCallbacks(capture) {
773:   capture.rendererContext.scene.add(capture.root);
774:   capture.rendererContext.assemblyRoots.push(capture.root);
775:   capture.rendererContext.onRobotLoaded({ root: capture.root, diagnostics: { robot_preview_lifecycle_state: 'loaded', robotPreviewLifecycleState: 'loaded' } });
776:   capture.rendererContext.onRobotMeshLoaded(capture.root);
777:   capture.rendererContext.onRobotMeshLoadError(new Error('old mesh failed'), 'old.dae', { uri: 'old.dae' });
778:   capture.rendererContext.onRobotError(new Error('old robot failed'), { robot_urdf_url: 'old.urdf' });
779: }
780: function assertOldCallbacksRejected(targetSceneId) {
781:   assert.strictEqual(state.robotPreviewResult, null);

---

822: `, context);
823: '''
824:     result = subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
825:     assert result.stderr == ""
826: 
827: 
828: def test_expanded_urdf_readiness_waits_for_current_terminal_loader_result():
829:     js_path = VIEWER / "viewer.js"
830:     harness = r'''
831: const fs = require('fs');
832: const vm = require('vm');
833: const assert = require('assert');
834: let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');

---

841: refreshWarnings = () => {};
842: appendRuntimeWarning = () => {};
843: rebuildSelectionIdentityIndex = () => ({ itemById:new Map(), explicitUiIdByLink:new Map() });
844: let callback, resolveReady;
845: loadRobotPreview = (preview, rendererContext) => {
846:   callback = rendererContext;
847:   const result = { diagnostics:{ robot_preview_lifecycle_state:'loading_urdf', robotPreviewLifecycleState:'loading_urdf' } };
848:   result.ready = new Promise(resolve => { resolveReady = resolve; });
849:   return result;
850: };
851: const required = { robot_arm:true, attached_tool_gripper:true, workbench_support_surface:true, configured_camera:true };
852: function beginExpanded() {
853:   window.dispatched.length = 0;

---

872: 
873: window.dispatched.length = 0;
874: state.sceneJson = { scene:{id:'legacy'}, robot_preview:{mode:'flattened_rows'} };
875: state.web3dReadiness = { state:'scene_loading', terminal:false, required, pending:new Set(), failed:false, failure:null };
876: state.robotUrdfPreviewDiagnostics = {robot_preview_lifecycle_state:'loading_urdf'};
877: maybeEmitSceneReady();
878: assert.strictEqual(window.dispatched.join(','), 'scene_ready', 'non-expanded scenes retain existing readiness behavior');
879: `, context);
880: '''
881:     result = subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
882:     assert result.stderr == ""
883: 
884: 

---

902: (async()=>{
903: renderSceneSummary=()=>{}; refreshInitialPoseActionState=()=>{}; refreshWarnings=()=>{}; appendRuntimeWarning=()=>{};
904: rebuildSelectionIdentityIndex=()=>({itemById:new Map(),explicitUiIdByLink:new Map()});
905: failIfExpandedUrdfExpectedVisualSetInvalid=()=>false;
906: state.sceneJson=payload; state.sourceWebSceneFile='ur5_2f_test.web_scene.json';
907: const rows=physicalReadinessItems();
908: const camera=rows.find(item=>readinessCategoryForItem(item)==='configured_camera');
909: const table=rows.find(item=>readinessCategoryForItem(item)==='workbench_support_surface');
910: assert(camera && table);
911: let controls=[];
912: loadRobotPreview=(_preview,callbacks)=>{let resolve,reject;const result={diagnostics:{robot_preview_lifecycle_state:'loading_urdf'},links:new Map()};result.ready=new Promise((ok,bad)=>{resolve=ok;reject=bad});controls.push({callbacks,result,resolve,reject});return result};
913: const required={robot_arm:true,attached_tool_gripper:true,workbench_support_surface:true,configured_camera:true};
914: function begin(){events.length=0;state.web3dReadiness={state:'scene_loading',terminal:false,terminalState:'',terminalNavigationKey:web3dNavigationKey(),terminalEmissionCount:0,statusSequence:0,required,pending:new Set(),failed:false,failure:null};const cameraOp=registerReadinessOperation([readinessKey('configured_camera',camera)]);const tableOp=registerReadinessOperation([readinessKey('workbench_support_surface',table)]);loadExpandedUrdfRobotPreview(payload.robot_preview);return {cameraOp,tableOp,control:controls.at(-1)}}

---

903: renderSceneSummary=()=>{}; refreshInitialPoseActionState=()=>{}; refreshWarnings=()=>{}; appendRuntimeWarning=()=>{};
904: rebuildSelectionIdentityIndex=()=>({itemById:new Map(),explicitUiIdByLink:new Map()});
905: failIfExpandedUrdfExpectedVisualSetInvalid=()=>false;
906: state.sceneJson=payload; state.sourceWebSceneFile='ur5_2f_test.web_scene.json';
907: const rows=physicalReadinessItems();
908: const camera=rows.find(item=>readinessCategoryForItem(item)==='configured_camera');
909: const table=rows.find(item=>readinessCategoryForItem(item)==='workbench_support_surface');
910: assert(camera && table);
911: let controls=[];
912: loadRobotPreview=(_preview,callbacks)=>{let resolve,reject;const result={diagnostics:{robot_preview_lifecycle_state:'loading_urdf'},links:new Map()};result.ready=new Promise((ok,bad)=>{resolve=ok;reject=bad});controls.push({callbacks,result,resolve,reject});return result};
913: const required={robot_arm:true,attached_tool_gripper:true,workbench_support_surface:true,configured_camera:true};
914: function begin(){events.length=0;state.web3dReadiness={state:'scene_loading',terminal:false,terminalState:'',terminalNavigationKey:web3dNavigationKey(),terminalEmissionCount:0,statusSequence:0,required,pending:new Set(),failed:false,failure:null};const cameraOp=registerReadinessOperation([readinessKey('configured_camera',camera)]);const tableOp=registerReadinessOperation([readinessKey('workbench_support_surface',table)]);loadExpandedUrdfRobotPreview(payload.robot_preview);return {cameraOp,tableOp,control:controls.at(-1)}}
915: let run=begin();

---

908: const camera=rows.find(item=>readinessCategoryForItem(item)==='configured_camera');
909: const table=rows.find(item=>readinessCategoryForItem(item)==='workbench_support_surface');
910: assert(camera && table);
911: let controls=[];
912: loadRobotPreview=(_preview,callbacks)=>{let resolve,reject;const result={diagnostics:{robot_preview_lifecycle_state:'loading_urdf'},links:new Map()};result.ready=new Promise((ok,bad)=>{resolve=ok;reject=bad});controls.push({callbacks,result,resolve,reject});return result};
913: const required={robot_arm:true,attached_tool_gripper:true,workbench_support_surface:true,configured_camera:true};
914: function begin(){events.length=0;state.web3dReadiness={state:'scene_loading',terminal:false,terminalState:'',terminalNavigationKey:web3dNavigationKey(),terminalEmissionCount:0,statusSequence:0,required,pending:new Set(),failed:false,failure:null};const cameraOp=registerReadinessOperation([readinessKey('configured_camera',camera)]);const tableOp=registerReadinessOperation([readinessKey('workbench_support_surface',table)]);loadExpandedUrdfRobotPreview(payload.robot_preview);return {cameraOp,tableOp,control:controls.at(-1)}}
915: let run=begin();
916: assert.deepStrictEqual(Array.from(run.cameraOp.pendingKeys),['configured_camera:realsense_overhead']);
917: const liveDiagnostics=run.control.result.diagnostics;
918: assert.strictEqual(state.robotUrdfPreviewDiagnostics,liveDiagnostics,'viewer must retain renderer diagnostics identity immediately after registration');
919: completeReadinessOperation(run.tableOp); completeReadinessOperation(run.cameraOp);
920: await Promise.resolve().then(()=>{liveDiagnostics.robot_preview_lifecycle_state='ready';liveDiagnostics.robotPreviewLifecycleState='ready';liveDiagnostics.robot_preview_loaded=true;liveDiagnostics.robot_expected_visual_count=2;liveDiagnostics.robot_loaded_visual_count=2;liveDiagnostics.robot_failed_visual_count=0;run.control.resolve(run.control.result)});await Promise.resolve();await Promise.resolve();

---

914: function begin(){events.length=0;state.web3dReadiness={state:'scene_loading',terminal:false,terminalState:'',terminalNavigationKey:web3dNavigationKey(),terminalEmissionCount:0,statusSequence:0,required,pending:new Set(),failed:false,failure:null};const cameraOp=registerReadinessOperation([readinessKey('configured_camera',camera)]);const tableOp=registerReadinessOperation([readinessKey('workbench_support_surface',table)]);loadExpandedUrdfRobotPreview(payload.robot_preview);return {cameraOp,tableOp,control:controls.at(-1)}}
915: let run=begin();
916: assert.deepStrictEqual(Array.from(run.cameraOp.pendingKeys),['configured_camera:realsense_overhead']);
917: const liveDiagnostics=run.control.result.diagnostics;
918: assert.strictEqual(state.robotUrdfPreviewDiagnostics,liveDiagnostics,'viewer must retain renderer diagnostics identity immediately after registration');
919: completeReadinessOperation(run.tableOp); completeReadinessOperation(run.cameraOp);
920: await Promise.resolve().then(()=>{liveDiagnostics.robot_preview_lifecycle_state='ready';liveDiagnostics.robotPreviewLifecycleState='ready';liveDiagnostics.robot_preview_loaded=true;liveDiagnostics.robot_expected_visual_count=2;liveDiagnostics.robot_loaded_visual_count=2;liveDiagnostics.robot_failed_visual_count=0;run.control.resolve(run.control.result)});await Promise.resolve();await Promise.resolve();
921: assert.strictEqual(run.control.result.diagnostics,liveDiagnostics);assert.strictEqual(state.robotUrdfPreviewDiagnostics,liveDiagnostics,'ready completion must still expose the renderer-owned diagnostics');
922: assert.deepStrictEqual(pendingRequiredLoads(),[]);assert.strictEqual(events.filter(e=>e.state==='scene_ready').length,1);assert.strictEqual(state.web3dReadiness.terminalEmissionCount,1);
923: run.control.callbacks.onRobotLoaded(run.control.result);
924: assert.strictEqual(events.filter(e=>e.state==='scene_ready').length,1,'duplicate callback cannot emit again');
925: 
926: run=begin();completeReadinessOperation(run.tableOp);completeReadinessOperation(run.cameraOp);run.control.reject(new Error('preview ready rejected'));await Promise.resolve();await Promise.resolve();

---

916: assert.deepStrictEqual(Array.from(run.cameraOp.pendingKeys),['configured_camera:realsense_overhead']);
917: const liveDiagnostics=run.control.result.diagnostics;
918: assert.strictEqual(state.robotUrdfPreviewDiagnostics,liveDiagnostics,'viewer must retain renderer diagnostics identity immediately after registration');
919: completeReadinessOperation(run.tableOp); completeReadinessOperation(run.cameraOp);
920: await Promise.resolve().then(()=>{liveDiagnostics.robot_preview_lifecycle_state='ready';liveDiagnostics.robotPreviewLifecycleState='ready';liveDiagnostics.robot_preview_loaded=true;liveDiagnostics.robot_expected_visual_count=2;liveDiagnostics.robot_loaded_visual_count=2;liveDiagnostics.robot_failed_visual_count=0;run.control.resolve(run.control.result)});await Promise.resolve();await Promise.resolve();
921: assert.strictEqual(run.control.result.diagnostics,liveDiagnostics);assert.strictEqual(state.robotUrdfPreviewDiagnostics,liveDiagnostics,'ready completion must still expose the renderer-owned diagnostics');
922: assert.deepStrictEqual(pendingRequiredLoads(),[]);assert.strictEqual(events.filter(e=>e.state==='scene_ready').length,1);assert.strictEqual(state.web3dReadiness.terminalEmissionCount,1);
923: run.control.callbacks.onRobotLoaded(run.control.result);
924: assert.strictEqual(events.filter(e=>e.state==='scene_ready').length,1,'duplicate callback cannot emit again');
925: 
926: run=begin();completeReadinessOperation(run.tableOp);completeReadinessOperation(run.cameraOp);run.control.reject(new Error('preview ready rejected'));await Promise.resolve();await Promise.resolve();
927: assert.strictEqual(state.web3dReadiness.state,'scene_failed');assert.deepStrictEqual(pendingRequiredLoads(),[]);assert.match(state.web3dReadiness.failure.reason,/preview ready rejected/);assert.match(state.web3dReadiness.failure.operation_id,/expanded_urdf:ur5_2f_test/);
928: assert.strictEqual(state.web3dReadiness.failure.robot_preview_lifecycle_state,'loading_urdf');assert.strictEqual(state.web3dReadiness.failure.robot_loaded_visual_count,0);assert.strictEqual(state.web3dReadiness.failure.robot_expected_visual_count,0);assert.strictEqual(state.web3dReadiness.failure.robot_failed_visual_count,0);assert.deepStrictEqual(state.web3dReadiness.failure.pending_required_loads,[]);

---

921: assert.strictEqual(run.control.result.diagnostics,liveDiagnostics);assert.strictEqual(state.robotUrdfPreviewDiagnostics,liveDiagnostics,'ready completion must still expose the renderer-owned diagnostics');
922: assert.deepStrictEqual(pendingRequiredLoads(),[]);assert.strictEqual(events.filter(e=>e.state==='scene_ready').length,1);assert.strictEqual(state.web3dReadiness.terminalEmissionCount,1);
923: run.control.callbacks.onRobotLoaded(run.control.result);
924: assert.strictEqual(events.filter(e=>e.state==='scene_ready').length,1,'duplicate callback cannot emit again');
925: 
926: run=begin();completeReadinessOperation(run.tableOp);completeReadinessOperation(run.cameraOp);run.control.reject(new Error('preview ready rejected'));await Promise.resolve();await Promise.resolve();
927: assert.strictEqual(state.web3dReadiness.state,'scene_failed');assert.deepStrictEqual(pendingRequiredLoads(),[]);assert.match(state.web3dReadiness.failure.reason,/preview ready rejected/);assert.match(state.web3dReadiness.failure.operation_id,/expanded_urdf:ur5_2f_test/);
928: assert.strictEqual(state.web3dReadiness.failure.robot_preview_lifecycle_state,'loading_urdf');assert.strictEqual(state.web3dReadiness.failure.robot_loaded_visual_count,0);assert.strictEqual(state.web3dReadiness.failure.robot_expected_visual_count,0);assert.strictEqual(state.web3dReadiness.failure.robot_failed_visual_count,0);assert.deepStrictEqual(state.web3dReadiness.failure.pending_required_loads,[]);
929: 
930: run=begin();completeReadinessOperation(run.tableOp);completeReadinessOperation(run.cameraOp);const deadline=timers.filter(t=>t.ms===REQUIRED_LOAD_DEADLINE_MS&&!t.cleared).at(-1);assert(deadline);deadline.fn();
931: assert.strictEqual(state.web3dReadiness.state,'scene_failed');assert.deepStrictEqual(pendingRequiredLoads(),[]);assert.strictEqual(state.web3dReadiness.failure.timeout_ms,REQUIRED_LOAD_DEADLINE_MS);assert.deepStrictEqual(state.web3dReadiness.failure.pending_required_loads,[]);
932: 
933: run=begin();const old=run.control;state.sceneJson={scene:{id:'replacement'}};resetSceneLifecycleState();state.web3dReadiness.required={...required};const replacementPending=['configured_camera:replacement'];state.web3dReadiness.pending=new Set(replacementPending);old.result.diagnostics.robot_preview_lifecycle_state='ready';old.result.diagnostics.robot_preview_loaded=true;old.resolve(old.result);await Promise.resolve();await Promise.resolve();old.callbacks.onRobotError(new Error('late old failure'),{});

---

925: 
926: run=begin();completeReadinessOperation(run.tableOp);completeReadinessOperation(run.cameraOp);run.control.reject(new Error('preview ready rejected'));await Promise.resolve();await Promise.resolve();
927: assert.strictEqual(state.web3dReadiness.state,'scene_failed');assert.deepStrictEqual(pendingRequiredLoads(),[]);assert.match(state.web3dReadiness.failure.reason,/preview ready rejected/);assert.match(state.web3dReadiness.failure.operation_id,/expanded_urdf:ur5_2f_test/);
928: assert.strictEqual(state.web3dReadiness.failure.robot_preview_lifecycle_state,'loading_urdf');assert.strictEqual(state.web3dReadiness.failure.robot_loaded_visual_count,0);assert.strictEqual(state.web3dReadiness.failure.robot_expected_visual_count,0);assert.strictEqual(state.web3dReadiness.failure.robot_failed_visual_count,0);assert.deepStrictEqual(state.web3dReadiness.failure.pending_required_loads,[]);
929: 
930: run=begin();completeReadinessOperation(run.tableOp);completeReadinessOperation(run.cameraOp);const deadline=timers.filter(t=>t.ms===REQUIRED_LOAD_DEADLINE_MS&&!t.cleared).at(-1);assert(deadline);deadline.fn();
931: assert.strictEqual(state.web3dReadiness.state,'scene_failed');assert.deepStrictEqual(pendingRequiredLoads(),[]);assert.strictEqual(state.web3dReadiness.failure.timeout_ms,REQUIRED_LOAD_DEADLINE_MS);assert.deepStrictEqual(state.web3dReadiness.failure.pending_required_loads,[]);
932: 
933: run=begin();const old=run.control;state.sceneJson={scene:{id:'replacement'}};resetSceneLifecycleState();state.web3dReadiness.required={...required};const replacementPending=['configured_camera:replacement'];state.web3dReadiness.pending=new Set(replacementPending);old.result.diagnostics.robot_preview_lifecycle_state='ready';old.result.diagnostics.robot_preview_loaded=true;old.resolve(old.result);await Promise.resolve();await Promise.resolve();old.callbacks.onRobotError(new Error('late old failure'),{});
934: assert.strictEqual(state.sceneJson.scene.id,'replacement');assert.deepStrictEqual(pendingRequiredLoads(),replacementPending);assert.strictEqual(state.web3dReadiness.state,'scene_loading');assert(debug.some(entry=>entry[1]?.operation_outcome==='stale_replacement'&&Array.isArray(entry[1]?.pending_required_loads)));
935: })().catch(err=>{console.error(err);process.exitCode=1});
936: `,context);
937: '''

---

928: assert.strictEqual(state.web3dReadiness.failure.robot_preview_lifecycle_state,'loading_urdf');assert.strictEqual(state.web3dReadiness.failure.robot_loaded_visual_count,0);assert.strictEqual(state.web3dReadiness.failure.robot_expected_visual_count,0);assert.strictEqual(state.web3dReadiness.failure.robot_failed_visual_count,0);assert.deepStrictEqual(state.web3dReadiness.failure.pending_required_loads,[]);
929: 
930: run=begin();completeReadinessOperation(run.tableOp);completeReadinessOperation(run.cameraOp);const deadline=timers.filter(t=>t.ms===REQUIRED_LOAD_DEADLINE_MS&&!t.cleared).at(-1);assert(deadline);deadline.fn();
931: assert.strictEqual(state.web3dReadiness.state,'scene_failed');assert.deepStrictEqual(pendingRequiredLoads(),[]);assert.strictEqual(state.web3dReadiness.failure.timeout_ms,REQUIRED_LOAD_DEADLINE_MS);assert.deepStrictEqual(state.web3dReadiness.failure.pending_required_loads,[]);
932: 
933: run=begin();const old=run.control;state.sceneJson={scene:{id:'replacement'}};resetSceneLifecycleState();state.web3dReadiness.required={...required};const replacementPending=['configured_camera:replacement'];state.web3dReadiness.pending=new Set(replacementPending);old.result.diagnostics.robot_preview_lifecycle_state='ready';old.result.diagnostics.robot_preview_loaded=true;old.resolve(old.result);await Promise.resolve();await Promise.resolve();old.callbacks.onRobotError(new Error('late old failure'),{});
934: assert.strictEqual(state.sceneJson.scene.id,'replacement');assert.deepStrictEqual(pendingRequiredLoads(),replacementPending);assert.strictEqual(state.web3dReadiness.state,'scene_loading');assert(debug.some(entry=>entry[1]?.operation_outcome==='stale_replacement'&&Array.isArray(entry[1]?.pending_required_loads)));
935: })().catch(err=>{console.error(err);process.exitCode=1});
936: `,context);
937: '''
938:     subprocess.run(
939:         ["node", "-e", harness, str(VIEWER / "viewer.js"), str(payload_path)],
940:         cwd=ROOT,

---

1299:     assert "sceneId()" not in init_body
1300: 
1301: 
1302: def test_viewer_initial_fit_waits_for_terminal_scene_ready_once():
1303:     js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
1304: 
1305:     readiness_body = js.split("function emitWeb3dReadinessState", 1)[1].split("function readinessCategoryForItem", 1)[0]
1306:     assert "readinessState === 'scene_ready' || readinessState === 'scene_failed'" in readiness_body
1307:     assert "if (state.web3dReadiness.terminal)" in readiness_body
1308:     assert "terminalEmissionCount" in readiness_body
1309:     assert "if (readinessState === 'scene_ready') triggerInitialCameraFitAfterSceneReady();" in readiness_body
1310: 
1311:     scene_ready_body = js.split("function maybeEmitSceneReady()", 1)[1].split("const el = {", 1)[0]

---

1300: 
1301: 
1302: def test_viewer_initial_fit_waits_for_terminal_scene_ready_once():
1303:     js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
1304: 
1305:     readiness_body = js.split("function emitWeb3dReadinessState", 1)[1].split("function readinessCategoryForItem", 1)[0]
1306:     assert "readinessState === 'scene_ready' || readinessState === 'scene_failed'" in readiness_body
1307:     assert "if (state.web3dReadiness.terminal)" in readiness_body
1308:     assert "terminalEmissionCount" in readiness_body
1309:     assert "if (readinessState === 'scene_ready') triggerInitialCameraFitAfterSceneReady();" in readiness_body
1310: 
1311:     scene_ready_body = js.split("function maybeEmitSceneReady()", 1)[1].split("const el = {", 1)[0]
1312:     assert "if (readiness.pending?.size === 0) emitWeb3dReadinessState('scene_ready'" in scene_ready_body

---

1301: 
1302: def test_viewer_initial_fit_waits_for_terminal_scene_ready_once():
1303:     js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
1304: 
1305:     readiness_body = js.split("function emitWeb3dReadinessState", 1)[1].split("function readinessCategoryForItem", 1)[0]
1306:     assert "readinessState === 'scene_ready' || readinessState === 'scene_failed'" in readiness_body
1307:     assert "if (state.web3dReadiness.terminal)" in readiness_body
1308:     assert "terminalEmissionCount" in readiness_body
1309:     assert "if (readinessState === 'scene_ready') triggerInitialCameraFitAfterSceneReady();" in readiness_body
1310: 
1311:     scene_ready_body = js.split("function maybeEmitSceneReady()", 1)[1].split("const el = {", 1)[0]
1312:     assert "if (readiness.pending?.size === 0) emitWeb3dReadinessState('scene_ready'" in scene_ready_body
1313: 

---

1302: def test_viewer_initial_fit_waits_for_terminal_scene_ready_once():
1303:     js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
1304: 
1305:     readiness_body = js.split("function emitWeb3dReadinessState", 1)[1].split("function readinessCategoryForItem", 1)[0]
1306:     assert "readinessState === 'scene_ready' || readinessState === 'scene_failed'" in readiness_body
1307:     assert "if (state.web3dReadiness.terminal)" in readiness_body
1308:     assert "terminalEmissionCount" in readiness_body
1309:     assert "if (readinessState === 'scene_ready') triggerInitialCameraFitAfterSceneReady();" in readiness_body
1310: 
1311:     scene_ready_body = js.split("function maybeEmitSceneReady()", 1)[1].split("const el = {", 1)[0]
1312:     assert "if (readiness.pending?.size === 0) emitWeb3dReadinessState('scene_ready'" in scene_ready_body
1313: 
1314:     trigger_body = js.split("function triggerInitialCameraFitAfterSceneReady()", 1)[1].split("function scheduleInitialCameraFitRetry", 1)[0]

---

1303:     js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
1304: 
1305:     readiness_body = js.split("function emitWeb3dReadinessState", 1)[1].split("function readinessCategoryForItem", 1)[0]
1306:     assert "readinessState === 'scene_ready' || readinessState === 'scene_failed'" in readiness_body
1307:     assert "if (state.web3dReadiness.terminal)" in readiness_body
1308:     assert "terminalEmissionCount" in readiness_body
1309:     assert "if (readinessState === 'scene_ready') triggerInitialCameraFitAfterSceneReady();" in readiness_body
1310: 
1311:     scene_ready_body = js.split("function maybeEmitSceneReady()", 1)[1].split("const el = {", 1)[0]
1312:     assert "if (readiness.pending?.size === 0) emitWeb3dReadinessState('scene_ready'" in scene_ready_body
1313: 
1314:     trigger_body = js.split("function triggerInitialCameraFitAfterSceneReady()", 1)[1].split("function scheduleInitialCameraFitRetry", 1)[0]
1315:     assert "return attemptInitialCameraFit({ allowRetry: false });" in trigger_body

---

1306:     assert "readinessState === 'scene_ready' || readinessState === 'scene_failed'" in readiness_body
1307:     assert "if (state.web3dReadiness.terminal)" in readiness_body
1308:     assert "terminalEmissionCount" in readiness_body
1309:     assert "if (readinessState === 'scene_ready') triggerInitialCameraFitAfterSceneReady();" in readiness_body
1310: 
1311:     scene_ready_body = js.split("function maybeEmitSceneReady()", 1)[1].split("const el = {", 1)[0]
1312:     assert "if (readiness.pending?.size === 0) emitWeb3dReadinessState('scene_ready'" in scene_ready_body
1313: 
1314:     trigger_body = js.split("function triggerInitialCameraFitAfterSceneReady()", 1)[1].split("function scheduleInitialCameraFitRetry", 1)[0]
1315:     assert "return attemptInitialCameraFit({ allowRetry: false });" in trigger_body
1316: 
1317:     fit_body = js.split("function attemptInitialCameraFit", 1)[1].split("function triggerInitialCameraFitAfterSceneReady", 1)[0]
1318:     assert "if (!fit || fit.done || fit.userControlled || fit.sceneKey !== stableSceneCameraKey()) return false;" in fit_body

---

1469:         text=True,
1470:     )
1471:     assert result.returncode == 0, result.stdout + result.stderr
1472:     assert "Three.js 160" in result.stdout
1473: 
1474: 
1475: def test_primary_mesh_backed_target_bin_keeps_product_visibility_scale_color_and_readiness():
1476:     js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
1477: 
1478:     grouping = js.split("function viewerGroupFor(item)", 1)[1].split("const DEBUG_OVERLAY_TOKEN_RE", 1)[0]
1479:     assert grouping.index("if (isPrimaryAuthoredPhysicalMesh(item)) return 'environment/layout';") < grouping.index("return 'zones';")
1480:     primary_physical = js.split("function isPrimaryAuthoredPhysicalMesh(item)", 1)[1].split("function readinessCategoryForItem", 1)[0]
1481:     assert "contractCategory === 'object'" in primary_physical

---

1474: 
1475: def test_primary_mesh_backed_target_bin_keeps_product_visibility_scale_color_and_readiness():
1476:     js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
1477: 
1478:     grouping = js.split("function viewerGroupFor(item)", 1)[1].split("const DEBUG_OVERLAY_TOKEN_RE", 1)[0]
1479:     assert grouping.index("if (isPrimaryAuthoredPhysicalMesh(item)) return 'environment/layout';") < grouping.index("return 'zones';")
1480:     primary_physical = js.split("function isPrimaryAuthoredPhysicalMesh(item)", 1)[1].split("function readinessCategoryForItem", 1)[0]
1481:     assert "contractCategory === 'object'" in primary_physical
1482:     assert "target bin" in primary_physical
1483: 
1484:     overlay_filter = js.split("function isDebugOverlayItem(item)", 1)[1].split("function isSensor(item)", 1)[0]
1485:     assert "if (viewerGroupFor(item) === 'zones') return true;" in overlay_filter
1486:     assert "if (isOverlayPolicyItem(item)) return true;" in overlay_filter

---

1491:     root_scale = js.split("function scaleOf(item)", 1)[1].split("function transformOf", 1)[0]
1492:     mesh_scale = js.split("function meshLocalTransformOf(item)", 1)[1].split("function cloneTransform", 1)[0]
1493:     assert "item.scale || [1, 1, 1]" in root_scale
1494:     assert "item.scale || item.mesh_scale" not in root_scale
1495:     assert "transform.scale || item?.mesh_scale" in mesh_scale
1496: 
1497:     readiness = js.split("function beginWeb3dSceneReadiness(items)", 1)[1].split("function requiredReadinessCompleteForItem", 1)[0]
1498:     assert "pending.add(readinessKey(category, item))" not in readiness
1499:     assert "required[category] = true" in readiness
1500:     category = js.split("function readinessCategoryForItem(item)", 1)[1].split("function readinessKey", 1)[0]
1501:     assert "return 'authored_physical_mesh';" in category
1502:     load = js.split("async function tryLoadMesh", 1)[1].split("function collectItems", 1)[0]
1503:     assert load.index("setRenderInfo(rendered, 'mesh_loaded'") < load.index("completePhysicalMeshAttempt(attempt)")

---

1492:     mesh_scale = js.split("function meshLocalTransformOf(item)", 1)[1].split("function cloneTransform", 1)[0]
1493:     assert "item.scale || [1, 1, 1]" in root_scale
1494:     assert "item.scale || item.mesh_scale" not in root_scale
1495:     assert "transform.scale || item?.mesh_scale" in mesh_scale
1496: 
1497:     readiness = js.split("function beginWeb3dSceneReadiness(items)", 1)[1].split("function requiredReadinessCompleteForItem", 1)[0]
1498:     assert "pending.add(readinessKey(category, item))" not in readiness
1499:     assert "required[category] = true" in readiness
1500:     category = js.split("function readinessCategoryForItem(item)", 1)[1].split("function readinessKey", 1)[0]
1501:     assert "return 'authored_physical_mesh';" in category
1502:     load = js.split("async function tryLoadMesh", 1)[1].split("function collectItems", 1)[0]
1503:     assert load.index("setRenderInfo(rendered, 'mesh_loaded'") < load.index("completePhysicalMeshAttempt(attempt)")
1504:     assert "failPhysicalMeshAttempt(attempt, item, loadUrl, `loaded mesh bounds validation failed" in load

---

1493:     assert "item.scale || [1, 1, 1]" in root_scale
1494:     assert "item.scale || item.mesh_scale" not in root_scale
1495:     assert "transform.scale || item?.mesh_scale" in mesh_scale
1496: 
1497:     readiness = js.split("function beginWeb3dSceneReadiness(items)", 1)[1].split("function requiredReadinessCompleteForItem", 1)[0]
1498:     assert "pending.add(readinessKey(category, item))" not in readiness
1499:     assert "required[category] = true" in readiness
1500:     category = js.split("function readinessCategoryForItem(item)", 1)[1].split("function readinessKey", 1)[0]
1501:     assert "return 'authored_physical_mesh';" in category
1502:     load = js.split("async function tryLoadMesh", 1)[1].split("function collectItems", 1)[0]
1503:     assert load.index("setRenderInfo(rendered, 'mesh_loaded'") < load.index("completePhysicalMeshAttempt(attempt)")
1504:     assert "failPhysicalMeshAttempt(attempt, item, loadUrl, `loaded mesh bounds validation failed" in load
1505: 

---

1494:     assert "item.scale || item.mesh_scale" not in root_scale
1495:     assert "transform.scale || item?.mesh_scale" in mesh_scale
1496: 
1497:     readiness = js.split("function beginWeb3dSceneReadiness(items)", 1)[1].split("function requiredReadinessCompleteForItem", 1)[0]
1498:     assert "pending.add(readinessKey(category, item))" not in readiness
1499:     assert "required[category] = true" in readiness
1500:     category = js.split("function readinessCategoryForItem(item)", 1)[1].split("function readinessKey", 1)[0]
1501:     assert "return 'authored_physical_mesh';" in category
1502:     load = js.split("async function tryLoadMesh", 1)[1].split("function collectItems", 1)[0]
1503:     assert load.index("setRenderInfo(rendered, 'mesh_loaded'") < load.index("completePhysicalMeshAttempt(attempt)")
1504:     assert "failPhysicalMeshAttempt(attempt, item, loadUrl, `loaded mesh bounds validation failed" in load
1505: 
1506:     material = js.split("function materialFor(item)", 1)[1].split("function materialHasUsableAppearance", 1)[0]

---

1529:   mesh_uri: 'build/workcell_studio_web_scene/assets/ur5_2f_test/target_bin.stl'
1530: };
1531: const placeZone = { id: 'place_zone_default', category: 'place_zone', render_policy: 'overlay' };
1532: assert.strictEqual(isPrimaryAuthoredPhysicalMesh(targetBin), true);
1533: assert.strictEqual(isDebugOverlayItem(targetBin), false);
1534: assert.strictEqual(viewerGroupFor(targetBin), 'environment/layout');
1535: assert.strictEqual(readinessCategoryForItem(targetBin), 'authored_physical_mesh');
1536: assert.strictEqual(isPrimaryAuthoredPhysicalMesh(placeZone), false);
1537: assert.strictEqual(isDebugOverlayItem(placeZone), true);
1538: assert.strictEqual(viewerGroupFor(placeZone), 'zones');
1539: assert.strictEqual(false || !isDebugOverlayItem(targetBin), true);
1540: assert.strictEqual(false || !isDebugOverlayItem(placeZone), false);
1541: state.sceneJson = { assets: [targetBin, placeZone] };

---

1537: assert.strictEqual(isDebugOverlayItem(placeZone), true);
1538: assert.strictEqual(viewerGroupFor(placeZone), 'zones');
1539: assert.strictEqual(false || !isDebugOverlayItem(targetBin), true);
1540: assert.strictEqual(false || !isDebugOverlayItem(placeZone), false);
1541: state.sceneJson = { assets: [targetBin, placeZone] };
1542: beginWeb3dSceneReadiness(collectItems(state.sceneJson));
1543: const readinessKeyForTarget = readinessKey('authored_physical_mesh', targetBin);
1544: assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), false);
1545: assert.strictEqual(state.web3dReadiness.required.authored_physical_mesh, true);
1546: const targetOperation = registerReadinessOperation([readinessKeyForTarget]);
1547: assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), true);
1548: targetBin.mesh_status = 'loaded';
1549: completeReadinessOperation(targetOperation);

---

1538: assert.strictEqual(viewerGroupFor(placeZone), 'zones');
1539: assert.strictEqual(false || !isDebugOverlayItem(targetBin), true);
1540: assert.strictEqual(false || !isDebugOverlayItem(placeZone), false);
1541: state.sceneJson = { assets: [targetBin, placeZone] };
1542: beginWeb3dSceneReadiness(collectItems(state.sceneJson));
1543: const readinessKeyForTarget = readinessKey('authored_physical_mesh', targetBin);
1544: assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), false);
1545: assert.strictEqual(state.web3dReadiness.required.authored_physical_mesh, true);
1546: const targetOperation = registerReadinessOperation([readinessKeyForTarget]);
1547: assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), true);
1548: targetBin.mesh_status = 'loaded';
1549: completeReadinessOperation(targetOperation);
1550: assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), false);

---

1540: assert.strictEqual(false || !isDebugOverlayItem(placeZone), false);
1541: state.sceneJson = { assets: [targetBin, placeZone] };
1542: beginWeb3dSceneReadiness(collectItems(state.sceneJson));
1543: const readinessKeyForTarget = readinessKey('authored_physical_mesh', targetBin);
1544: assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), false);
1545: assert.strictEqual(state.web3dReadiness.required.authored_physical_mesh, true);
1546: const targetOperation = registerReadinessOperation([readinessKeyForTarget]);
1547: assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), true);
1548: targetBin.mesh_status = 'loaded';
1549: completeReadinessOperation(targetOperation);
1550: assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), false);
1551: `, context);
1552: """

---

1541: state.sceneJson = { assets: [targetBin, placeZone] };
1542: beginWeb3dSceneReadiness(collectItems(state.sceneJson));
1543: const readinessKeyForTarget = readinessKey('authored_physical_mesh', targetBin);
1544: assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), false);
1545: assert.strictEqual(state.web3dReadiness.required.authored_physical_mesh, true);
1546: const targetOperation = registerReadinessOperation([readinessKeyForTarget]);
1547: assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), true);
1548: targetBin.mesh_status = 'loaded';
1549: completeReadinessOperation(targetOperation);
1550: assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), false);
1551: `, context);
1552: """
1553:     subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

---

1544: assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), false);
1545: assert.strictEqual(state.web3dReadiness.required.authored_physical_mesh, true);
1546: const targetOperation = registerReadinessOperation([readinessKeyForTarget]);
1547: assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), true);
1548: targetBin.mesh_status = 'loaded';
1549: completeReadinessOperation(targetOperation);
1550: assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), false);
1551: `, context);
1552: """
1553:     subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
1554: 
1555: 
1556: def test_viewer_visual_bounds_diagnostics_and_fit_bounds_contract_are_source_guarded():

---

3046:   link: 'gripper_base_link',
3047:   url: 'build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq/robotiq_85_gripper_visual.dae',
3048:   uri: 'package://robotiq_85_description/meshes/visual/robotiq_85_gripper_visual.dae',
3049:   path: 'package://robotiq_85_description/meshes/visual/robotiq_85_gripper_visual.dae',
3050: });
3051: let status = updateViewerStatus();
3052: assert.strictEqual(status.web3d_readiness_state, 'scene_failed');
3053: assert.strictEqual(status.web3dReadinessState, 'scene_failed');
3054: assert.strictEqual(status.viewer_boot_state, 'scene_failed');
3055: assert.strictEqual(status.final_lifecycle_state, 'scene_failed');
3056: assert.strictEqual(status.tool_status, 'failed');
3057: assert.strictEqual(status.failed_required_item_count > 0, true);
3058: assert.strictEqual(status.final_failed_url, 'build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq/robotiq_85_gripper_visual.dae');

---

3055: assert.strictEqual(status.final_lifecycle_state, 'scene_failed');
3056: assert.strictEqual(status.tool_status, 'failed');
3057: assert.strictEqual(status.failed_required_item_count > 0, true);
3058: assert.strictEqual(status.final_failed_url, 'build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq/robotiq_85_gripper_visual.dae');
3059: assert.strictEqual(status.finalFailedUrl, status.final_failed_url);
3060: assert.strictEqual(status.final_failed_link, 'gripper_base_link');
3061: assert.strictEqual(status.readiness_failure.link, 'gripper_base_link');
3062: assert.strictEqual(status.readiness_failure.url, status.final_failed_url);
3063: assert.ok(status.readiness_failure.robot_missing_meshes[0].includes('robotiq_85_gripper_visual.dae'));
3064: assert.ok(status.readiness_failure.reason.includes('Robotiq tool mesh'));
3065: emitWeb3dReadinessState('server_ready', { http_status: 200 });
3066: status = updateViewerStatus();
3067: assert.strictEqual(status.web3d_readiness_state, 'scene_failed');

---

3056: assert.strictEqual(status.tool_status, 'failed');
3057: assert.strictEqual(status.failed_required_item_count > 0, true);
3058: assert.strictEqual(status.final_failed_url, 'build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq/robotiq_85_gripper_visual.dae');
3059: assert.strictEqual(status.finalFailedUrl, status.final_failed_url);
3060: assert.strictEqual(status.final_failed_link, 'gripper_base_link');
3061: assert.strictEqual(status.readiness_failure.link, 'gripper_base_link');
3062: assert.strictEqual(status.readiness_failure.url, status.final_failed_url);
3063: assert.ok(status.readiness_failure.robot_missing_meshes[0].includes('robotiq_85_gripper_visual.dae'));
3064: assert.ok(status.readiness_failure.reason.includes('Robotiq tool mesh'));
3065: emitWeb3dReadinessState('server_ready', { http_status: 200 });
3066: status = updateViewerStatus();
3067: assert.strictEqual(status.web3d_readiness_state, 'scene_failed');
3068: assert.strictEqual(status.web3dReadinessState, 'scene_failed');

---

3057: assert.strictEqual(status.failed_required_item_count > 0, true);
3058: assert.strictEqual(status.final_failed_url, 'build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq/robotiq_85_gripper_visual.dae');
3059: assert.strictEqual(status.finalFailedUrl, status.final_failed_url);
3060: assert.strictEqual(status.final_failed_link, 'gripper_base_link');
3061: assert.strictEqual(status.readiness_failure.link, 'gripper_base_link');
3062: assert.strictEqual(status.readiness_failure.url, status.final_failed_url);
3063: assert.ok(status.readiness_failure.robot_missing_meshes[0].includes('robotiq_85_gripper_visual.dae'));
3064: assert.ok(status.readiness_failure.reason.includes('Robotiq tool mesh'));
3065: emitWeb3dReadinessState('server_ready', { http_status: 200 });
3066: status = updateViewerStatus();
3067: assert.strictEqual(status.web3d_readiness_state, 'scene_failed');
3068: assert.strictEqual(status.web3dReadinessState, 'scene_failed');
3069: assert.notStrictEqual(status.web3d_readiness_state, 'scene_ready');

---

3058: assert.strictEqual(status.final_failed_url, 'build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq/robotiq_85_gripper_visual.dae');
3059: assert.strictEqual(status.finalFailedUrl, status.final_failed_url);
3060: assert.strictEqual(status.final_failed_link, 'gripper_base_link');
3061: assert.strictEqual(status.readiness_failure.link, 'gripper_base_link');
3062: assert.strictEqual(status.readiness_failure.url, status.final_failed_url);
3063: assert.ok(status.readiness_failure.robot_missing_meshes[0].includes('robotiq_85_gripper_visual.dae'));
3064: assert.ok(status.readiness_failure.reason.includes('Robotiq tool mesh'));
3065: emitWeb3dReadinessState('server_ready', { http_status: 200 });
3066: status = updateViewerStatus();
3067: assert.strictEqual(status.web3d_readiness_state, 'scene_failed');
3068: assert.strictEqual(status.web3dReadinessState, 'scene_failed');
3069: assert.notStrictEqual(status.web3d_readiness_state, 'scene_ready');
3070: assert.notStrictEqual(status.web3dReadinessState, 'scene_ready');

---

3061: assert.strictEqual(status.readiness_failure.link, 'gripper_base_link');
3062: assert.strictEqual(status.readiness_failure.url, status.final_failed_url);
3063: assert.ok(status.readiness_failure.robot_missing_meshes[0].includes('robotiq_85_gripper_visual.dae'));
3064: assert.ok(status.readiness_failure.reason.includes('Robotiq tool mesh'));
3065: emitWeb3dReadinessState('server_ready', { http_status: 200 });
3066: status = updateViewerStatus();
3067: assert.strictEqual(status.web3d_readiness_state, 'scene_failed');
3068: assert.strictEqual(status.web3dReadinessState, 'scene_failed');
3069: assert.notStrictEqual(status.web3d_readiness_state, 'scene_ready');
3070: assert.notStrictEqual(status.web3dReadinessState, 'scene_ready');
3071: assert.strictEqual(status.readiness_failure.url, status.final_failed_url);
3072: `, context);
3073: """

---

3063: assert.ok(status.readiness_failure.robot_missing_meshes[0].includes('robotiq_85_gripper_visual.dae'));
3064: assert.ok(status.readiness_failure.reason.includes('Robotiq tool mesh'));
3065: emitWeb3dReadinessState('server_ready', { http_status: 200 });
3066: status = updateViewerStatus();
3067: assert.strictEqual(status.web3d_readiness_state, 'scene_failed');
3068: assert.strictEqual(status.web3dReadinessState, 'scene_failed');
3069: assert.notStrictEqual(status.web3d_readiness_state, 'scene_ready');
3070: assert.notStrictEqual(status.web3dReadinessState, 'scene_ready');
3071: assert.strictEqual(status.readiness_failure.url, status.final_failed_url);
3072: `, context);
3073: """
3074:     result = subprocess.run(
3075:         ["node", "-e", harness, str(js_path)],

---

3065: emitWeb3dReadinessState('server_ready', { http_status: 200 });
3066: status = updateViewerStatus();
3067: assert.strictEqual(status.web3d_readiness_state, 'scene_failed');
3068: assert.strictEqual(status.web3dReadinessState, 'scene_failed');
3069: assert.notStrictEqual(status.web3d_readiness_state, 'scene_ready');
3070: assert.notStrictEqual(status.web3dReadinessState, 'scene_ready');
3071: assert.strictEqual(status.readiness_failure.url, status.final_failed_url);
3072: `, context);
3073: """
3074:     result = subprocess.run(
3075:         ["node", "-e", harness, str(js_path)],
3076:         cwd=ROOT,
3077:         check=True,

---

3078:         text=True,
3079:         stdout=subprocess.PIPE,
3080:         stderr=subprocess.PIPE,
3081:     )
3082:     assert result.stderr == ""
3083: 
3084: def test_viewer_emits_explicit_web3d_readiness_states_and_required_categories():
3085:     js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
3086:     assert "'server_ready'" in js or "server_ready is infrastructure state" in (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
3087:     assert "'scene_loading'" in js
3088:     assert "'scene_ready'" in js
3089:     assert "'scene_failed'" in js
3090:     assert "const WEB3D_REQUIRED_CATEGORIES = ['robot_arm', 'attached_tool_gripper', 'workbench_support_surface', 'configured_camera']" in js

---

3087:     assert "'scene_loading'" in js
3088:     assert "'scene_ready'" in js
3089:     assert "'scene_failed'" in js
3090:     assert "const WEB3D_REQUIRED_CATEGORIES = ['robot_arm', 'attached_tool_gripper', 'workbench_support_surface', 'configured_camera']" in js
3091:     assert "function beginWeb3dSceneReadiness(items)" in js
3092:     assert "function maybeEmitSceneReady()" in js
3093:     assert "if (readinessState === 'scene_ready')" in js
3094:     assert "if (state.web3dReadiness.terminal)" in js
3095:     assert "state.web3dReadiness.terminal" in js
3096:     assert "'robot_arm:expanded_urdf_loader'" in js
3097:     assert "registerReadinessOperation" in js
3098:     assert "'attached_tool_gripper:expanded_urdf_loader'" in js
3099:     assert "emitWeb3dReadinessState('scene_loading'" in js

---

3113:     viewer = (VIEWER / "viewer.js").read_text(encoding="utf-8")
3114:     renderer = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
3115:     assert "function failWeb3dSceneReadiness(item, url, reason" in viewer
3116:     assert "emitWeb3dReadinessState('scene_failed'" in viewer
3117:     assert "url: url || displayMeshUri(item)" in viewer
3118:     assert "link: item?.link || item?.link_name || item?.object_name || ''" in viewer
3119:     assert "const eventDetail = { ...structured, ...detail, final_lifecycle_state: readinessState" in viewer
3120:     assert "if (required) failPhysicalMeshAttempt(attempt, item, preflight.url || loadUrl" in viewer
3121:     assert "http_status: preflight.http_status || null" in viewer
3122:     assert "onRobotMeshLoadError: (err, uri, detail) => { if (!callbackIsCurrent()) return ignoreStaleCallback(); finalizeRequiredLoad('failure'" in viewer
3123:     assert "function inferMeshLinkDetail(path)" in renderer
3124:     assert "'gripper_base_link'" in renderer
3125:     assert "context?.onRobotMeshLoadError?.(err, uri, { url, uri, path, source_url: path, sourceUrl: path, policy_reason:" in renderer

---

3163:   id: 'oversized_table', display_name: 'Oversized Workbench', category: 'table', mesh_uri: 'package://cell/meshes/table.stl', expected_dimensions_m: [1, 0.5, 0.2],
3164:   loaded_mesh_bounds: { min: {x:0,y:0,z:0}, max: {x:4,y:1,z:0.4}, center: {x:2,y:0.5,z:0.2}, dimensions: {x:4,y:1,z:0.4} },
3165:   loaded_mesh_world_bounds: { min: {x:10,y:20,z:0}, max: {x:14,y:21,z:0.4}, center: {x:12,y:20.5,z:0.2}, dimensions: {x:4,y:1,z:0.4} },
3166:   loaded_mesh_axis_ratios: {x:4,y:2,z:2}, loaded_mesh_maximum_ratio: 4, loaded_mesh_uniform_ratio: 2,
3167:   loaded_mesh_bounds_reason_code: 'loaded_mesh_oversized', mesh_unit_correction: { scale: 1, confidence: 'rejected_non_uniform_or_unclear_ratio' }
3168: };
3169: const attempt = { token: physicalLoadToken, navigationKey: web3dNavigationKey(), category: 'workbench_support_surface', identity: item.id, readinessKey: 'workbench_support_surface:oversized_table', operation: registerReadinessOperation(['workbench_support_surface:oversized_table']) };
3170: const extra = physicalMeshBoundsFailurePayload(item, '/assets/table.stl', 'STLLoader');
3171: assert.strictEqual(failPhysicalMeshAttempt(attempt, item, '/assets/table.stl', 'loaded mesh bounds validation failed (oversized)', extra), true);
3172: const failure = window.events.at(-1);
3173: assert.strictEqual(failure.state, 'scene_failed');
3174: assert.strictEqual(failure.scene_id, 'ur5_2f_test');
3175: assert.strictEqual(failure.item_id, 'oversized_table');

---

3184: assert.deepStrictEqual(failure.axis_ratios, {x:4,y:2,z:2});
3185: assert.strictEqual(failure.maximum_ratio, 4);
3186: assert.strictEqual(failure.uniform_ratio, 2);
3187: assert.strictEqual(failure.applied_mesh_scale, 1);
3188: assert.strictEqual(failure.unit_correction_decision, 'rejected_non_uniform_or_unclear_ratio');
3189: assert.strictEqual(failure.bounds_reason_code, 'loaded_mesh_oversized');
3190: const statusFailure = window.__WORKCELL_VIEWER_STATUS__.readiness_failure;
3191: for (const field of ['scene_id', 'item_id', 'item_display_name', 'category', 'mesh_uri', 'mesh_load_url', 'loader', 'expected_dimensions', 'loaded_local_dimensions', 'loaded_local_bounds', 'loaded_world_dimensions', 'loaded_world_bounds', 'axis_ratios', 'maximum_ratio', 'uniform_ratio', 'applied_mesh_scale', 'unit_correction_decision', 'bounds_reason_code']) {
3192:   assert.deepStrictEqual(statusFailure[field], failure[field], field + ' must survive viewer status normalization');
3193: }
3194: `, context);
3195: """
3196:     subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, capture_output=True, text=True)

---

3195: """
3196:     subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, capture_output=True, text=True)
3197: 
3198: 
3199: def test_successful_required_loads_emit_scene_ready_exactly_once_after_completion():
3200:     js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
3201:     body = _viewer_function_body(js, "function emitWeb3dReadinessState", "function readinessCategoryForItem")
3202:     maybe_body = _viewer_function_body(js, "function maybeEmitSceneReady", "function isGeneratedUrdfItem")
3203:     assert "if (state.web3dReadiness.terminal)" in body
3204:     assert "terminalEmissionCount" in body
3205:     assert "readiness.pending?.size === 0" in maybe_body
3206:     assert "emitWeb3dReadinessState('scene_ready'" in maybe_body
3207:     assert "requiredReadinessCompleteForItem(item);" in js

---

3199: def test_successful_required_loads_emit_scene_ready_exactly_once_after_completion():
3200:     js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
3201:     body = _viewer_function_body(js, "function emitWeb3dReadinessState", "function readinessCategoryForItem")
3202:     maybe_body = _viewer_function_body(js, "function maybeEmitSceneReady", "function isGeneratedUrdfItem")
3203:     assert "if (state.web3dReadiness.terminal)" in body
3204:     assert "terminalEmissionCount" in body
3205:     assert "readiness.pending?.size === 0" in maybe_body
3206:     assert "emitWeb3dReadinessState('scene_ready'" in maybe_body
3207:     assert "requiredReadinessCompleteForItem(item);" in js
3208:     assert "finalizeRequiredLoad('success'" in js
3209: 
3210: 
3211: 

---

3243:     assert "expected_robot_visual_links" in js
3244:     assert "expected_tool_visual_links" in js
3245:     assert "missing_required_robot_visuals" in js
3246:     assert "missing_required_tool_visuals" in js
3247:     assert "expanded URDF expected robot/tool visuals are missing or failed" in js
3248:     assert "expanded URDF renderer reported required robot/tool visual failure" in js
3249:     readiness_body = js.split("function failExpandedUrdfReadiness", 1)[1].split("function maybeEmitSceneReady", 1)[0]
3250:     assert "attached_tool_gripper" in readiness_body
3251:     assert "expected_tool_visual_links" in readiness_body
3252: 
3253: 
3254: def test_expanded_urdf_mode_marks_flattened_robot_tool_rows_diagnostic_only():
3255:     js = (VIEWER / "viewer.js").read_text(encoding="utf-8")

---

3244:     assert "expected_tool_visual_links" in js
3245:     assert "missing_required_robot_visuals" in js
3246:     assert "missing_required_tool_visuals" in js
3247:     assert "expanded URDF expected robot/tool visuals are missing or failed" in js
3248:     assert "expanded URDF renderer reported required robot/tool visual failure" in js
3249:     readiness_body = js.split("function failExpandedUrdfReadiness", 1)[1].split("function maybeEmitSceneReady", 1)[0]
3250:     assert "attached_tool_gripper" in readiness_body
3251:     assert "expected_tool_visual_links" in readiness_body
3252: 
3253: 
3254: def test_expanded_urdf_mode_marks_flattened_robot_tool_rows_diagnostic_only():
3255:     js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
3256:     render_body = js.split("function renderScene(items)", 1)[1].split("function loadExpandedUrdfRobotPreview", 1)[0]

---

3245:     assert "missing_required_robot_visuals" in js
3246:     assert "missing_required_tool_visuals" in js
3247:     assert "expanded URDF expected robot/tool visuals are missing or failed" in js
3248:     assert "expanded URDF renderer reported required robot/tool visual failure" in js
3249:     readiness_body = js.split("function failExpandedUrdfReadiness", 1)[1].split("function maybeEmitSceneReady", 1)[0]
3250:     assert "attached_tool_gripper" in readiness_body
3251:     assert "expected_tool_visual_links" in readiness_body
3252: 
3253: 
3254: def test_expanded_urdf_mode_marks_flattened_robot_tool_rows_diagnostic_only():
3255:     js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
3256:     render_body = js.split("function renderScene(items)", 1)[1].split("function loadExpandedUrdfRobotPreview", 1)[0]
3257:     assert "if (urdfPreviewActive) loadExpandedUrdfRobotPreview(state.sceneJson.robot_preview);" in render_body

---

3373: """,
3374:         encoding="utf-8",
3375:     )
3376:     subprocess.run(["node", str(script)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
3377: 
3378: 
3379: def test_viewer_browser_readiness_contract_lifecycle_and_terminal_dedup():
3380:     js_path = VIEWER / "viewer.js"
3381:     harness = """
3382: const fs = require('fs');
3383: const vm = require('vm');
3384: const assert = require('assert');
3385: let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\\(\\);\\s*$/, '');

---

3399: ] };
3400: state.sceneJsonLoaded = true;
3401: failIfCanonicalRequiredVisualSetInvalid = () => false;
3402: beginWeb3dSceneReadiness(collectItems(state.sceneJson));
3403: for (const item of collectItems(state.sceneJson)) requiredReadinessCompleteForItem(item);
3404: let status = window.__WORKCELL_VIEWER_STATUS__;
3405: assert.strictEqual(status.readiness_contract_version, 1);
3406: assert.strictEqual(status.lifecycle_state, 'scene_ready');
3407: assert.strictEqual(status.terminal, true);
3408: assert.strictEqual(status.scene_id, 'ur5_2f_test');
3409: assert.strictEqual(status.source_web_scene_file, state.sourceWebSceneFile);
3410: assert.strictEqual(status.builder_revision, '42');
3411: assert.ok(status.expected_physical_item_count > 0);

---

3438: beginWeb3dSceneReadiness(collectItems(state.sceneJson));
3439: const robot = collectItems(state.sceneJson)[0];
3440: failWeb3dSceneReadiness(robot, 'missing.stl', 'mesh failed');
3441: let status = window.__WORKCELL_VIEWER_STATUS__;
3442: assert.strictEqual(status.lifecycle_state, 'scene_failed');
3443: assert.strictEqual(status.terminal, true);
3444: assert.strictEqual(status.readiness_failure.required_category, 'robot_arm');
3445: assert.strictEqual(status.readiness_failure.link, 'base_link');
3446: assert.strictEqual(status.readiness_failure.url, 'missing.stl');
3447: const seq = status.status_sequence;
3448: emitWeb3dReadinessState('scene_ready');
3449: failWeb3dSceneReadiness(robot, 'missing.stl', 'mesh failed again');
3450: assert.strictEqual(window.__WORKCELL_VIEWER_STATUS__.lifecycle_state, 'scene_failed');

---

3439: const robot = collectItems(state.sceneJson)[0];
3440: failWeb3dSceneReadiness(robot, 'missing.stl', 'mesh failed');
3441: let status = window.__WORKCELL_VIEWER_STATUS__;
3442: assert.strictEqual(status.lifecycle_state, 'scene_failed');
3443: assert.strictEqual(status.terminal, true);
3444: assert.strictEqual(status.readiness_failure.required_category, 'robot_arm');
3445: assert.strictEqual(status.readiness_failure.link, 'base_link');
3446: assert.strictEqual(status.readiness_failure.url, 'missing.stl');
3447: const seq = status.status_sequence;
3448: emitWeb3dReadinessState('scene_ready');
3449: failWeb3dSceneReadiness(robot, 'missing.stl', 'mesh failed again');
3450: assert.strictEqual(window.__WORKCELL_VIEWER_STATUS__.lifecycle_state, 'scene_failed');
3451: assert.strictEqual(window.__WORKCELL_VIEWER_STATUS__.status_sequence, seq);

---

3440: failWeb3dSceneReadiness(robot, 'missing.stl', 'mesh failed');
3441: let status = window.__WORKCELL_VIEWER_STATUS__;
3442: assert.strictEqual(status.lifecycle_state, 'scene_failed');
3443: assert.strictEqual(status.terminal, true);
3444: assert.strictEqual(status.readiness_failure.required_category, 'robot_arm');
3445: assert.strictEqual(status.readiness_failure.link, 'base_link');
3446: assert.strictEqual(status.readiness_failure.url, 'missing.stl');
3447: const seq = status.status_sequence;
3448: emitWeb3dReadinessState('scene_ready');
3449: failWeb3dSceneReadiness(robot, 'missing.stl', 'mesh failed again');
3450: assert.strictEqual(window.__WORKCELL_VIEWER_STATUS__.lifecycle_state, 'scene_failed');
3451: assert.strictEqual(window.__WORKCELL_VIEWER_STATUS__.status_sequence, seq);
3452: assert.strictEqual(state.web3dReadiness.terminalEmissionCount, 1);

---

3452: assert.strictEqual(state.web3dReadiness.terminalEmissionCount, 1);
3453: `, context);
3454: """
3455:     subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
3456: 
3457: 
3458: def test_viewer_render_policy_excludes_diagnostics_from_readiness_and_editing():
3459:     js_path = VIEWER / "viewer.js"
3460:     harness = """
3461: const fs = require('fs');
3462: const vm = require('vm');
3463: const assert = require('assert');
3464: let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');

---

3464: let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
3465: const element = () => ({ hidden: false, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', classList: { toggle() {} }, setAttribute() {}, querySelector() { return { textContent: '' }; }, appendChild() {}, addEventListener() {}, getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; } });
3466: const context = { console, assert, window: { dispatched: [], location: { search: '' }, dispatchEvent(event) { this.dispatched.push(event?.detail?.state); }, parent: { postMessage() {} } }, document: { getElementById() { return element(); }, createElement() { return element(); } }, URLSearchParams, CustomEvent: function CustomEvent(type, init) { return { type, detail: init?.detail || {} }; }, requestAnimationFrame() { return 0; }, setTimeout() { return 1; }, clearTimeout() {} };
3467: vm.createContext(context);
3468: vm.runInContext(source + `
3469: state.sceneJson = { scene: { id: 'policy_scene' }, assets: [
3470:   { id: 'table', category: 'table', render_policy: 'primary', render_owner: 'editable_layout', render_identity: 'scene|layout|table', readiness_category: 'workbench_support_surface', editable: true },
3471:   { id: 'pick_zone', category: 'pick_zone', render_policy: 'overlay', render_owner: 'task_overlay', render_identity: 'scene|overlay|pick' },
3472:   { id: 'base_link_flattened', category: 'robot_static_mesh_visual', role: 'robot', mesh_uri: 'robot.dae', render_policy: 'diagnostic_only', render_owner: 'expanded_urdf_robot', render_identity: 'scene|robot|base_link|0' }
3473: ] };
3474: const items = collectItems(state.sceneJson);
3475: assert.strictEqual(items.length, 3);
3476: assert.strictEqual(readinessCategoryForItem(items.find(i => i.id === 'table')), 'workbench_support_surface');

---

3470:   { id: 'table', category: 'table', render_policy: 'primary', render_owner: 'editable_layout', render_identity: 'scene|layout|table', readiness_category: 'workbench_support_surface', editable: true },
3471:   { id: 'pick_zone', category: 'pick_zone', render_policy: 'overlay', render_owner: 'task_overlay', render_identity: 'scene|overlay|pick' },
3472:   { id: 'base_link_flattened', category: 'robot_static_mesh_visual', role: 'robot', mesh_uri: 'robot.dae', render_policy: 'diagnostic_only', render_owner: 'expanded_urdf_robot', render_identity: 'scene|robot|base_link|0' }
3473: ] };
3474: const items = collectItems(state.sceneJson);
3475: assert.strictEqual(items.length, 3);
3476: assert.strictEqual(readinessCategoryForItem(items.find(i => i.id === 'table')), 'workbench_support_surface');
3477: assert.strictEqual(readinessCategoryForItem(items.find(i => i.id === 'base_link_flattened')), '');
3478: assert.strictEqual(readinessCategoryForItem(items.find(i => i.id === 'pick_zone')), '');
3479: assert.strictEqual(isDebugOverlayItem(items.find(i => i.id === 'pick_zone')), true);
3480: assert.strictEqual(canEditItem(items.find(i => i.id === 'table')), true);
3481: assert.strictEqual(canEditItem(items.find(i => i.id === 'base_link_flattened')), false);
3482: beginWeb3dSceneReadiness(items);

---

3471:   { id: 'pick_zone', category: 'pick_zone', render_policy: 'overlay', render_owner: 'task_overlay', render_identity: 'scene|overlay|pick' },
3472:   { id: 'base_link_flattened', category: 'robot_static_mesh_visual', role: 'robot', mesh_uri: 'robot.dae', render_policy: 'diagnostic_only', render_owner: 'expanded_urdf_robot', render_identity: 'scene|robot|base_link|0' }
3473: ] };
3474: const items = collectItems(state.sceneJson);
3475: assert.strictEqual(items.length, 3);
3476: assert.strictEqual(readinessCategoryForItem(items.find(i => i.id === 'table')), 'workbench_support_surface');
3477: assert.strictEqual(readinessCategoryForItem(items.find(i => i.id === 'base_link_flattened')), '');
3478: assert.strictEqual(readinessCategoryForItem(items.find(i => i.id === 'pick_zone')), '');
3479: assert.strictEqual(isDebugOverlayItem(items.find(i => i.id === 'pick_zone')), true);
3480: assert.strictEqual(canEditItem(items.find(i => i.id === 'table')), true);
3481: assert.strictEqual(canEditItem(items.find(i => i.id === 'base_link_flattened')), false);
3482: beginWeb3dSceneReadiness(items);
3483: assert.deepStrictEqual(Array.from(state.web3dReadiness.pending), []);

---

3472:   { id: 'base_link_flattened', category: 'robot_static_mesh_visual', role: 'robot', mesh_uri: 'robot.dae', render_policy: 'diagnostic_only', render_owner: 'expanded_urdf_robot', render_identity: 'scene|robot|base_link|0' }
3473: ] };
3474: const items = collectItems(state.sceneJson);
3475: assert.strictEqual(items.length, 3);
3476: assert.strictEqual(readinessCategoryForItem(items.find(i => i.id === 'table')), 'workbench_support_surface');
3477: assert.strictEqual(readinessCategoryForItem(items.find(i => i.id === 'base_link_flattened')), '');
3478: assert.strictEqual(readinessCategoryForItem(items.find(i => i.id === 'pick_zone')), '');
3479: assert.strictEqual(isDebugOverlayItem(items.find(i => i.id === 'pick_zone')), true);
3480: assert.strictEqual(canEditItem(items.find(i => i.id === 'table')), true);
3481: assert.strictEqual(canEditItem(items.find(i => i.id === 'base_link_flattened')), false);
3482: beginWeb3dSceneReadiness(items);
3483: assert.deepStrictEqual(Array.from(state.web3dReadiness.pending), []);
3484: assert.strictEqual(state.web3dReadiness.required.workbench_support_surface, true);

---

3675: }}
3676: """,
3677:         encoding="utf-8",
3678:     )
3679:     subprocess.run(["node", str(script)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
3680: 
3681: def test_expanded_urdf_readiness_allows_legitimate_multi_visual_links_and_equivalent_records():
3682:     js_path = VIEWER / "viewer.js"
3683:     harness = r"""
3684: const fs = require('fs');
3685: const vm = require('vm');
3686: const assert = require('assert');
3687: let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');

---

3696:   { link_name: 'base_link', visual_index: 1, object_name: 'visual_1' },
3697:   { link_name: 'shoulder_link', visual_index: 0, object_name: 'visual_0' },
3698:   { link_name: 'robotiq_85_base_link', visual_index: 0, object_name: 'visual_0' },
3699:   { link_name: 'robotiq_85_base_link', visual_index: 1, object_name: 'visual_1' },
3700: ] };
3701: collectRenderedMeshDiagnostics = () => [
3702:   { id: 'table_generated', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
3703:   { id: 'table_semantic_equivalent', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
3704:   { id: 'camera_generated', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
3705:   { id: 'camera_semantic_equivalent', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
3706: ];
3707: const diagnostics = expandedUrdfVisualReadinessDiagnostics();
3708: assert.strictEqual(diagnostics.required_visual_ready, true);

---

3697:   { link_name: 'shoulder_link', visual_index: 0, object_name: 'visual_0' },
3698:   { link_name: 'robotiq_85_base_link', visual_index: 0, object_name: 'visual_0' },
3699:   { link_name: 'robotiq_85_base_link', visual_index: 1, object_name: 'visual_1' },
3700: ] };
3701: collectRenderedMeshDiagnostics = () => [
3702:   { id: 'table_generated', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
3703:   { id: 'table_semantic_equivalent', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
3704:   { id: 'camera_generated', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
3705:   { id: 'camera_semantic_equivalent', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
3706: ];
3707: const diagnostics = expandedUrdfVisualReadinessDiagnostics();
3708: assert.strictEqual(diagnostics.required_visual_ready, true);
3709: assert.deepStrictEqual(diagnostics.missing_required_visuals, []);

---

3698:   { link_name: 'robotiq_85_base_link', visual_index: 0, object_name: 'visual_0' },
3699:   { link_name: 'robotiq_85_base_link', visual_index: 1, object_name: 'visual_1' },
3700: ] };
3701: collectRenderedMeshDiagnostics = () => [
3702:   { id: 'table_generated', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
3703:   { id: 'table_semantic_equivalent', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
3704:   { id: 'camera_generated', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
3705:   { id: 'camera_semantic_equivalent', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
3706: ];
3707: const diagnostics = expandedUrdfVisualReadinessDiagnostics();
3708: assert.strictEqual(diagnostics.required_visual_ready, true);
3709: assert.deepStrictEqual(diagnostics.missing_required_visuals, []);
3710: assert.ok(diagnostics.duplicate_physical_visual_identities.length > 0);

---

3699:   { link_name: 'robotiq_85_base_link', visual_index: 1, object_name: 'visual_1' },
3700: ] };
3701: collectRenderedMeshDiagnostics = () => [
3702:   { id: 'table_generated', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
3703:   { id: 'table_semantic_equivalent', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
3704:   { id: 'camera_generated', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
3705:   { id: 'camera_semantic_equivalent', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
3706: ];
3707: const diagnostics = expandedUrdfVisualReadinessDiagnostics();
3708: assert.strictEqual(diagnostics.required_visual_ready, true);
3709: assert.deepStrictEqual(diagnostics.missing_required_visuals, []);
3710: assert.ok(diagnostics.duplicate_physical_visual_identities.length > 0);
3711: state.web3dReadiness = { state: 'scene_loading', emittedSceneReady: false, required: { robot_arm: true, attached_tool_gripper: true, workbench_support_surface: true, configured_camera: true }, pending: new Set(['robot_arm:expanded_urdf_loader', 'attached_tool_gripper:expanded_urdf_loader']), failed: false, failure: null };

---

3743:   robot_missing_meshes: [],
3744:   robot_missing_required_robot_visual_links: [],
3745:   robot_missing_required_tool_visual_links: [],
3746:   robot_visual_wrapper_world_matrices: []
3747: };
3748: collectRenderedMeshDiagnostics = () => [
3749:   { id: 'workbench_generated', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
3750:   { id: 'configured_camera_generated', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
3751: ];
3752: const diagnostics = expandedUrdfVisualReadinessDiagnostics();
3753: assert.strictEqual(diagnostics.robot_expected_visual_count, 16);
3754: assert.strictEqual(diagnostics.robot_completed_visual_count, 16);
3755: assert.strictEqual(diagnostics.robot_loaded_visual_count, 16);

---

3744:   robot_missing_required_robot_visual_links: [],
3745:   robot_missing_required_tool_visual_links: [],
3746:   robot_visual_wrapper_world_matrices: []
3747: };
3748: collectRenderedMeshDiagnostics = () => [
3749:   { id: 'workbench_generated', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
3750:   { id: 'configured_camera_generated', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
3751: ];
3752: const diagnostics = expandedUrdfVisualReadinessDiagnostics();
3753: assert.strictEqual(diagnostics.robot_expected_visual_count, 16);
3754: assert.strictEqual(diagnostics.robot_completed_visual_count, 16);
3755: assert.strictEqual(diagnostics.robot_loaded_visual_count, 16);
3756: assert.strictEqual(diagnostics.robot_failed_visual_count, 0);

---

3821: assert.strictEqual(state.web3dReadiness.terminalEmissionCount, 1);
3822: `, context);
3823: """
3824:     subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
3825: 
3826: 
3827: def test_expanded_urdf_readiness_fails_missing_required_link_and_required_mesh_failure():
3828:     js_path = VIEWER / "viewer.js"
3829:     harness = r"""
3830: const fs = require('fs');
3831: const vm = require('vm');
3832: const assert = require('assert');
3833: let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');

---

3838: state.sceneJson = { scene: { id: 'ur5_2f_test' }, robot_preview: { mode: 'expanded_urdf_loader', expected_robot_visual_links: ['base_link', 'shoulder_link'], expected_tool_visual_links: ['robotiq_85_base_link'] } };
3839: state.robotUrdfPreviewDiagnostics = { robot_preview_lifecycle_state: 'failed', robot_preview_loaded: false, robot_failed_visual_count: 0, robot_missing_required_robot_visual_links: ['shoulder_link'], robot_missing_required_tool_visual_links: [], robot_visual_wrapper_world_matrices: [
3840:   { link_name: 'base_link', visual_index: 0 },
3841:   { link_name: 'robotiq_85_base_link', visual_index: 0 },
3842: ] };
3843: collectRenderedMeshDiagnostics = () => [
3844:   { id: 'table', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
3845:   { id: 'camera', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
3846: ];
3847: let diagnostics = expandedUrdfVisualReadinessDiagnostics();
3848: assert.deepStrictEqual(diagnostics.missing_required_robot_visuals, ['shoulder_link']);
3849: assert.strictEqual(diagnostics.required_visual_ready, false);
3850: assert.strictEqual(failIfExpandedUrdfExpectedVisualSetInvalid(), true);

---

3839: state.robotUrdfPreviewDiagnostics = { robot_preview_lifecycle_state: 'failed', robot_preview_loaded: false, robot_failed_visual_count: 0, robot_missing_required_robot_visual_links: ['shoulder_link'], robot_missing_required_tool_visual_links: [], robot_visual_wrapper_world_matrices: [
3840:   { link_name: 'base_link', visual_index: 0 },
3841:   { link_name: 'robotiq_85_base_link', visual_index: 0 },
3842: ] };
3843: collectRenderedMeshDiagnostics = () => [
3844:   { id: 'table', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
3845:   { id: 'camera', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
3846: ];
3847: let diagnostics = expandedUrdfVisualReadinessDiagnostics();
3848: assert.deepStrictEqual(diagnostics.missing_required_robot_visuals, ['shoulder_link']);
3849: assert.strictEqual(diagnostics.required_visual_ready, false);
3850: assert.strictEqual(failIfExpandedUrdfExpectedVisualSetInvalid(), true);
3851: assert.strictEqual(state.web3dReadiness.state, 'scene_failed');

---

3882: state.sceneJson = { scene: { id: 'ur5_2f_test' }, robot_preview: { mode: 'expanded_urdf_loader', robot_instance_id: 'ur5_2f', expected_robot_visual_links: robotLinks, expected_tool_visual_links: toolLinks } };
3883: state.robotUrdfPreviewDiagnostics = { robot_preview_lifecycle_state: 'ready', robot_preview_loaded: true, robot_failed_visual_count: 0, robot_visual_wrapper_world_matrices: robotLinks.concat(toolLinks).flatMap(link => [
3884:   { link_name: link, visual_index: 0, object_name: 'visual_0' },
3885:   { link_name: link, visual_index: 1, object_name: 'visual_1' },
3886: ]) };
3887: collectRenderedMeshDiagnostics = () => [
3888:   { id: 'workbench_generated', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
3889:   { id: 'configured_camera_generated', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
3890: ];
3891: const diagnostics = expandedUrdfVisualReadinessDiagnostics();
3892: assert.strictEqual(diagnostics.required_visual_ready, true);
3893: assert.ok(Array.isArray(diagnostics.duplicate_physical_visual_identities));
3894: assert.strictEqual(failIfExpandedUrdfExpectedVisualSetInvalid(), false);

---

3883: state.robotUrdfPreviewDiagnostics = { robot_preview_lifecycle_state: 'ready', robot_preview_loaded: true, robot_failed_visual_count: 0, robot_visual_wrapper_world_matrices: robotLinks.concat(toolLinks).flatMap(link => [
3884:   { link_name: link, visual_index: 0, object_name: 'visual_0' },
3885:   { link_name: link, visual_index: 1, object_name: 'visual_1' },
3886: ]) };
3887: collectRenderedMeshDiagnostics = () => [
3888:   { id: 'workbench_generated', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
3889:   { id: 'configured_camera_generated', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
3890: ];
3891: const diagnostics = expandedUrdfVisualReadinessDiagnostics();
3892: assert.strictEqual(diagnostics.required_visual_ready, true);
3893: assert.ok(Array.isArray(diagnostics.duplicate_physical_visual_identities));
3894: assert.strictEqual(failIfExpandedUrdfExpectedVisualSetInvalid(), false);
3895: assert.notStrictEqual(state.web3dReadiness?.state, 'scene_failed');
```
