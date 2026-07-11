import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';

const ROBOT_RENDER_MODE = 'expanded_urdf_loader';

function repoUrl(context, uri) {
  return context?.repoRootRelativeUrl ? context.repoRootRelativeUrl(uri) : uri;
}

function normalizeMeshUri(path, context, diagnostics) {
  const raw = String(path || '').trim();
  if (!raw) return '';
  if (raw.startsWith('package://')) {
    diagnostics.robot_missing_meshes.push(`${raw}: package:// URI was not staged for static web loading`);
    return '';
  }
  const diagnostic = context?.meshUriDiagnostic?.({ mesh_uri: raw, mesh_staging_status: 'staged' });
  return diagnostic?.uri || raw;
}

function meshExtension(uri) {
  const path = String(uri || '').split(/[?#]/, 1)[0];
  return path.includes('.') ? path.slice(path.lastIndexOf('.') + 1).toLowerCase() : '';
}

function applyFallbackMaterial(object, material) {
  if (!material || !object?.traverse) return object;
  object.traverse(child => {
    if (child?.isMesh && !child.material) child.material = material;
  });
  return object;
}

function setLifecycleState(diagnostics, state) {
  diagnostics.robot_preview_lifecycle_state = state;
  diagnostics.robotPreviewLifecycleState = state;
}

function loadMesh(path, manager, material, done, context, diagnostics) {
  const uri = normalizeMeshUri(path, context, diagnostics);
  diagnostics.robot_expected_visual_count += 1;
  diagnostics.robotExpectedVisualCount = diagnostics.robot_expected_visual_count;
  if (!uri) {
    diagnostics.robot_failed_visual_count += 1;
    diagnostics.robotFailedVisualCount = diagnostics.robot_failed_visual_count;
    done(null, new Error(`unloadable URDF mesh URI: ${path}`));
    return;
  }
  const url = repoUrl(context, uri);
  const ext = meshExtension(uri);
  const onDone = object => {
    diagnostics.robot_loaded_visual_count += 1;
    diagnostics.robot_completed_visual_count = diagnostics.robot_loaded_visual_count;
    diagnostics.robotCompletedVisualCount = diagnostics.robot_completed_visual_count;
    done(applyFallbackMaterial(object, material));
    context?.onRobotMeshLoaded?.();
  };
  const onError = err => {
    diagnostics.robot_failed_visual_count += 1;
    diagnostics.robotFailedVisualCount = diagnostics.robot_failed_visual_count;
    diagnostics.robot_missing_meshes.push(`${uri}: ${err?.message || err || 'load failed'}`);
    done(null, err);
    context?.onRobotMeshLoadError?.(err, uri);
  };
  if (ext === 'stl') new STLLoader(manager).load(url, geom => onDone(new THREE.Mesh(geom, material || new THREE.MeshPhongMaterial())), undefined, onError);
  else if (ext === 'dae') new ColladaLoader(manager).load(url, dae => onDone(dae.scene), undefined, onError);
  else if (ext === 'obj') new OBJLoader(manager).load(url, obj => onDone(obj), undefined, onError);
  else onError(new Error(`unsupported mesh format .${ext || 'unknown'}`));
}

function buildLookupMap(source) {
  return new Map(Object.entries(source || {}));
}

function linkRootDiagnostics(robot, links) {
  const entries = Object.entries(links || {});
  const roots = entries.filter(([, link]) => link?.parent === robot || !link?.parent).map(([name]) => name);
  const disconnected = entries.filter(([, link]) => {
    let node = link;
    const seen = new Set();
    while (node) {
      if (node === robot) return false;
      if (seen.has(node)) return true;
      seen.add(node);
      node = node.parent;
    }
    return true;
  }).map(([name]) => name);
  const seenNames = new Set();
  const duplicateLinks = [];
  for (const [name] of entries) {
    if (seenNames.has(name)) duplicateLinks.push(name);
    seenNames.add(name);
  }
  return { roots, disconnected, duplicateLinks };
}

function jointTypeCounts(joints) {
  const counts = { fixed: 0, revolute: 0, continuous: 0, prismatic: 0, mimic: 0, other: 0 };
  for (const joint of Object.values(joints || {})) {
    const type = String(joint?.jointType || '').toLowerCase();
    if (type in counts) counts[type] += 1;
    else counts.other += 1;
    if (joint?.isURDFMimicJoint || joint?.mimicJoint) counts.mimic += 1;
  }
  return counts;
}

export function loadRobotPreview(previewConfig, rendererContext = {}) {
  const diagnostics = {
    robot_render_mode: ROBOT_RENDER_MODE,
    robot_preview_loaded: false,
    robot_urdf_url: previewConfig?.urdf_url || '',
    robot_loaded_link_count: 0,
    robot_loaded_joint_count: 0,
    robot_loaded_visual_count: 0,
    robot_missing_meshes: [],
    robot_joint_values_applied: previewConfig?.joint_values || {},
    robot_joint_type_counts: {},
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
    skipped_legacy_generated_urdf_visual_count: rendererContext?.skippedLegacyGeneratedUrdfVisualCount || 0,
  };
  const result = { root: null, links: new Map(), joints: new Map(), diagnostics, ready: null };

  result.ready = (async () => {
    setLifecycleState(diagnostics, 'loading_urdf');
    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loader.parseVisual = true;
    loader.parseCollision = false;
    loader.packages = '';
    loader.workingPath = '';
    loader.loadMeshCb = (path, meshManager, material, done) => loadMesh(path, meshManager, material, done, rendererContext, diagnostics);

    const urdfUrl = repoUrl(rendererContext, previewConfig?.urdf_url || '');
    const robot = await loader.loadAsync(urdfUrl);
    setLifecycleState(diagnostics, 'loading_meshes');
    robot.name = rendererContext?.rootName || 'workcell_studio_urdf_loader_robot';
    robot.userData.robot_render_mode = ROBOT_RENDER_MODE;
    robot.userData.robot_preview_source = 'gkjohnson/urdf-loaders urdf-loader@0.13.0';

    const jointValues = previewConfig?.joint_values || {};
    // Delegate all supported fixed/revolute/continuous/prismatic and mimic propagation
    // to urdf-loader's joint API; do not manually transform child link groups here.
    robot.setJointValues(jointValues);
    robot.updateMatrixWorld(true);

    result.root = robot;
    result.links = buildLookupMap(robot.links);
    result.joints = buildLookupMap(robot.joints);
    diagnostics.robot_loaded_link_count = result.links.size;
    diagnostics.robot_loaded_joint_count = result.joints.size;
    diagnostics.robot_joint_type_counts = jointTypeCounts(robot.joints);
    diagnostics.robot_hierarchy_links = Array.from(result.links.keys());
    const rootDiagnostics = linkRootDiagnostics(robot, robot.links);
    diagnostics.robot_root_links = rootDiagnostics.roots;
    diagnostics.robotRootLinks = rootDiagnostics.roots;
    diagnostics.robot_root_link_count = rootDiagnostics.roots.length;
    diagnostics.robotRootLinkCount = rootDiagnostics.roots.length;
    diagnostics.robot_disconnected_links = rootDiagnostics.disconnected;
    diagnostics.robotDisconnectedLinks = rootDiagnostics.disconnected;
    diagnostics.robot_duplicate_links = rootDiagnostics.duplicateLinks;
    diagnostics.robotDuplicateLinks = rootDiagnostics.duplicateLinks;
    diagnostics.robot_hierarchy_missing_links = Array.isArray(previewConfig?.expected_links)
      ? previewConfig.expected_links.filter(link => !result.links.has(link))
      : [];
    diagnostics.robot_preview_loaded = diagnostics.robot_failed_visual_count === 0 && diagnostics.robot_hierarchy_missing_links.length === 0;
    setLifecycleState(diagnostics, diagnostics.robot_preview_loaded ? 'ready' : 'failed');
    rendererContext?.scene?.add?.(robot);
    rendererContext?.assemblyRoots?.push?.(robot);
    rendererContext?.onRobotLoaded?.(result);
    return result;
  })().catch(err => {
    diagnostics.robot_preview_loaded = false;
    setLifecycleState(diagnostics, 'failed');
    diagnostics.robot_missing_meshes.push(err?.message || String(err));
    rendererContext?.onRobotError?.(err, diagnostics);
    return result;
  });

  window.__WORKCELL_ROBOT_PREVIEW_READY__ = result.ready;
  return result;
}
