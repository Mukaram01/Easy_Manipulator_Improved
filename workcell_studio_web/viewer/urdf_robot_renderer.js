import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';

const ROBOT_RENDER_MODE = 'expanded_urdf_loader';
const ROS_TO_THREE_CONVERSION_BOUNDARY = 'URDFLoader owns the ROS visual frame and applies the one ROS-to-Three orientation boundary; DAE, STL, assembled URDF, and flattened fallback diagnostics must not add per-link or per-mesh 90-degree corrections after ColladaLoader.';

function repoUrl(context, uri) {
  return context?.repoRootRelativeUrl ? context.repoRootRelativeUrl(uri) : uri;
}

function safeDecodeUriSegment(segment) {
  try {
    return decodeURIComponent(segment);
  } catch (_) {
    return null;
  }
}

function rejectPackageMeshUri(reason, diagnostics) {
  diagnostics.robot_missing_meshes.push(`URDF package mesh rejected: ${reason}`);
  return '';
}

function resolvePackageMeshUri(uri, sceneId, diagnostics) {
  const raw = String(uri || '').trim();
  if (!raw.startsWith('package://')) return '';
  const scene = String(sceneId || '').trim();
  if (!scene) return rejectPackageMeshUri('missing scene ID', diagnostics);
  if (!/^[A-Za-z0-9][A-Za-z0-9_.-]*$/.test(scene) || scene.includes('%')) return rejectPackageMeshUri(`malformed scene ID: ${scene}`, diagnostics);
  const packagePath = raw.slice('package://'.length);
  if (packagePath.startsWith('/') || /^(?:file|https?):\/\//i.test(packagePath) || /^[A-Za-z]:[\\/]/.test(packagePath)) {
    return rejectPackageMeshUri(raw, diagnostics);
  }
  const parts = packagePath.split('/');
  const packageName = parts.shift() || '';
  if (!/^[A-Za-z][A-Za-z0-9_]*$/.test(packageName)) return rejectPackageMeshUri(raw, diagnostics);
  if (!parts.length) return rejectPackageMeshUri(raw, diagnostics);
  const safeParts = [];
  for (const part of parts) {
    if (!part) return rejectPackageMeshUri(raw, diagnostics);
    const decoded = safeDecodeUriSegment(part);
    if (!decoded || decoded === '.' || decoded === '..' || decoded.includes('\0') || decoded.includes('/') || decoded.includes('\\') || /^[A-Za-z]:[\\/]/.test(decoded) || /^[A-Za-z][A-Za-z0-9+.-]*:/.test(decoded)) {
      return rejectPackageMeshUri(raw, diagnostics);
    }
    if (decoded.includes('%')) {
      const decodedAgain = safeDecodeUriSegment(decoded);
      if (decodedAgain !== decoded) return rejectPackageMeshUri(raw, diagnostics);
    }
    safeParts.push(encodeURIComponent(decoded));
  }
  const resolved = `build/workcell_studio_web_scene/assets/${scene}/${packageName}/${safeParts.join('/')}`;
  diagnostics.robot_package_mesh_resolutions.push(`URDF package mesh resolved: ${raw} -> ${resolved}`);
  return resolved;
}


function packageRootResolver(sceneId, diagnostics) {
  const scene = String(sceneId || '').trim();
  return targetPackage => {
    const packageName = String(targetPackage || '').trim();
    if (!/^[A-Za-z0-9][A-Za-z0-9_.-]*$/.test(scene) || scene.includes('%') || !/^[A-Za-z][A-Za-z0-9_]*$/.test(packageName)) {
      rejectPackageMeshUri(`package resolver rejected ${packageName || '<empty>'} for scene ${scene || '<empty>'}`, diagnostics);
      return 'build/workcell_studio_web_scene/assets/__rejected_package_uri__';
    }
    return `build/workcell_studio_web_scene/assets/${scene}/${packageName}`;
  };
}

function configureUrdfPackageResolution(loader, manager, context, diagnostics) {
  const resolver = packageRootResolver(context?.sceneId, diagnostics);
  if ('packages' in loader) loader.packages = resolver;
  else loader.packages = resolver;
  if (manager?.setURLModifier) {
    manager.setURLModifier(url => {
      const raw = String(url || '').trim();
      if (raw.startsWith('package://')) return resolvePackageMeshUri(raw, context?.sceneId, diagnostics);
      if (/^\/(?:[A-Za-z][A-Za-z0-9_]*)(?:\/|$)/.test(raw)) {
        return rejectPackageMeshUri(`bare package-root URL: ${raw}`, diagnostics);
      }
      return url;
    });
  }
}

function normalizeMeshUri(path, context, diagnostics) {
  const raw = String(path || '').trim();
  if (!raw) return '';
  if (raw.startsWith('package://')) {
    return resolvePackageMeshUri(raw, context?.sceneId, diagnostics);
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

function isIdentityTransform(object, eps = 1e-9) {
  if (!object) return true;
  return Math.abs(object.position?.x || 0) <= eps
    && Math.abs(object.position?.y || 0) <= eps
    && Math.abs(object.position?.z || 0) <= eps
    && Math.abs((object.quaternion?.x || 0)) <= eps
    && Math.abs((object.quaternion?.y || 0)) <= eps
    && Math.abs((object.quaternion?.z || 0)) <= eps
    && Math.abs((object.quaternion?.w ?? 1) - 1) <= eps
    && Math.abs((object.scale?.x ?? 1) - 1) <= eps
    && Math.abs((object.scale?.y ?? 1) - 1) <= eps
    && Math.abs((object.scale?.z ?? 1) - 1) <= eps;
}

function objectLocalTransformDiagnostics(object) {
  if (!object) return null;
  object.updateMatrix?.();
  return {
    position: vector3ToDiagnostics(object.position),
    quaternion: object.quaternion ? { x: object.quaternion.x, y: object.quaternion.y, z: object.quaternion.z, w: object.quaternion.w } : null,
    scale: vector3ToDiagnostics(object.scale),
    matrix: Array.from(object.matrix?.elements || []).map(value => Number(value)),
  };
}

function countDescendantMeshes(object) {
  let count = 0;
  object?.traverse?.(child => {
    if (child?.isMesh) count += 1;
  });
  return count;
}

function summarizeColladaDiagnostic(diagnostics, detail) {
  const key = [detail.up_axis || '', detail.unit_meter ?? '', detail.root_transform_normalized ? 'normalized' : 'unchanged'].join('|');
  diagnostics.robot_collada_summary_keys = diagnostics.robot_collada_summary_keys || [];
  if (!diagnostics.robot_collada_summary_keys.includes(key)) diagnostics.robot_collada_summary_keys.push(key);
  diagnostics.robot_collada_summary = diagnostics.robot_collada_summary || [];
  const row = diagnostics.robot_collada_summary.find(entry => entry.key === key);
  if (row) { row.count += 1; return; }
  diagnostics.robot_collada_summary.push({ key, count: 1, severity: 'debug', ...detail });
}

function normalizeRosColladaScene(dae, uri, diagnostics) {
  const scene = dae?.scene;
  if (!scene) return scene;
  const asset = dae?.asset || {};
  const upAxis = String(asset.upAxis || asset.up_axis || '').toUpperCase();
  const unitMeter = Number(asset.unit || asset.unitMeter || asset.meter || 1);
  const before = objectLocalTransformDiagnostics(scene);
  const meshCount = countDescendantMeshes(scene);
  const rootHasMesh = Boolean(scene.isMesh);

  // ROS-to-Three conversion boundary: URDFLoader owns the ROS visual frame for
  // every mesh format (DAE, STL, assembled URDF, and flattened fallback rows).
  // Three's ColladaLoader may expose file-level Z_UP or unit metadata as a
  // transform on the returned dae.scene root before URDFLoader places the visual
  // wrapper. Neutralize only that loader root metadata transform. Do not add a
  // second hardcoded 90-degree correction to any link, mesh, baked world matrix,
  // assembled URDF node, or flattened fallback diagnostic row.
  const hasLoaderRootConversion = !rootHasMesh
    && !isIdentityTransform(scene)
    && (upAxis === 'Z_UP' || (Number.isFinite(unitMeter) && Math.abs(unitMeter - 1) > 1e-9));
  if (!hasLoaderRootConversion) {
    const detail = {
      uri,
      up_axis: upAxis || null,
      unit_meter: Number.isFinite(unitMeter) ? unitMeter : null,
      descendant_mesh_count: meshCount,
      root_transform_normalized: false,
      root_transform_before: before,
    };
    diagnostics.robot_collada_mesh_diagnostics.push(detail);
    summarizeColladaDiagnostic(diagnostics, detail);
    return scene;
  }

  scene.position.set(0, 0, 0);
  scene.quaternion.identity();
  scene.rotation.set(0, 0, 0);
  scene.scale.set(1, 1, 1);
  scene.updateMatrix();
  scene.updateMatrixWorld(true);
  diagnostics.robot_collada_root_normalization_count += 1;
  diagnostics.robotColladaRootNormalizationCount = diagnostics.robot_collada_root_normalization_count;
  const detail = {
    uri,
    up_axis: upAxis || null,
    unit_meter: Number.isFinite(unitMeter) ? unitMeter : null,
    descendant_mesh_count: meshCount,
    root_transform_normalized: true,
    root_transform_before: before,
    root_transform_after: objectLocalTransformDiagnostics(scene),
  };
  diagnostics.robot_collada_mesh_diagnostics.push(detail);
  summarizeColladaDiagnostic(diagnostics, detail);
  return scene;
}

function loadMesh(path, manager, material, done, context, diagnostics) {
  const uri = normalizeMeshUri(path, context, diagnostics);
  diagnostics.robot_expected_visual_count += 1;
  diagnostics.robotExpectedVisualCount = diagnostics.robot_expected_visual_count;
  if (!uri) {
    diagnostics.robot_failed_visual_count += 1;
    diagnostics.robotFailedVisualCount = diagnostics.robot_failed_visual_count;
    const err = new Error(`unloadable URDF mesh URI: ${path}`);
    done(null, err);
    context?.onRobotMeshLoadError?.(err, '', { path, ...inferMeshLinkDetail(path) });
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
    diagnostics.robot_missing_meshes.push(`${url}: ${err?.message || err || 'load failed'}`);
    done(null, err);
    context?.onRobotMeshLoadError?.(err, uri, { url, uri, path, ...inferMeshLinkDetail(path) });
  };
  if (ext === 'stl') new STLLoader(manager).load(url, geom => onDone(new THREE.Mesh(geom, material || new THREE.MeshPhongMaterial())), undefined, onError);
  else if (ext === 'dae') new ColladaLoader(manager).load(url, dae => onDone(normalizeRosColladaScene(dae, uri, diagnostics)), undefined, onError);
  else if (ext === 'obj') new OBJLoader(manager).load(url, obj => onDone(obj), undefined, onError);
  else onError(new Error(`unsupported mesh format .${ext || 'unknown'}`));
}

function inferMeshLinkDetail(path) {
  const text = String(path || '').toLowerCase();
  const known = ['gripper_base_link', 'tool0', 'wrist_3_link', 'wrist_2_link', 'wrist_1_link', 'forearm_link', 'upper_arm_link', 'shoulder_link', 'base_link_inertia', 'base_link'];
  const link = known.find(name => text.includes(name.toLowerCase())) || (/gripper|robotiq|finger|suction|airpick/.test(text) ? 'gripper_base_link' : '');
  return { link, link_name: link, item: link || String(path || '') };
}


function meshCompletionPromise(diagnostics, manager) {
  let managerComplete = false;
  const waitTick = callback => (typeof window !== 'undefined' && window.setTimeout ? window.setTimeout(callback, 0) : setTimeout(callback, 0));
  const managerDone = new Promise(resolve => {
    const previousOnLoad = manager.onLoad;
    manager.onLoad = () => {
      managerComplete = true;
      previousOnLoad?.();
      resolve();
    };
  });
  const countsSettled = () => diagnostics.robot_expected_visual_count === (diagnostics.robot_loaded_visual_count + diagnostics.robot_failed_visual_count);
  return {
    async wait() {
      if (diagnostics.robot_expected_visual_count === 0) return;
      if (!managerComplete) await managerDone;
      await new Promise(resolve => {
        const check = () => {
          if (countsSettled()) resolve();
          else waitTick(check);
        };
        check();
      });
    },
  };
}

function matrix4ToDiagnostics(object) {
  if (!object?.matrixWorld) return null;
  object.updateMatrixWorld?.(true);
  return Array.from(object.matrixWorld.elements).map(value => Number(value));
}

function vector3ToDiagnostics(value) {
  return value ? { x: Number(value.x), y: Number(value.y), z: Number(value.z) } : null;
}


function finiteMatrix16(value) {
  if (Array.isArray(value) && value.length === 4 && value.every(row => Array.isArray(row) && row.length === 4)) {
    const out = [];
    for (let col = 0; col < 4; col += 1) {
      for (let row = 0; row < 4; row += 1) out.push(Number(value[row][col]));
    }
    return out.every(Number.isFinite) ? out : null;
  }
  if (Array.isArray(value) && value.length === 16) {
    const out = value.map(Number);
    return out.every(Number.isFinite) ? out : null;
  }
  const matrix = value?.matrix_world || value?.matrixWorld || value?.expected_matrix || value?.expectedMatrix || null;
  return matrix && matrix !== value ? finiteMatrix16(matrix) : null;
}

function matrixMaxAbsDelta(a, b) {
  const ma = finiteMatrix16(a);
  const mb = finiteMatrix16(b);
  if (!ma || !mb) return Infinity;
  return Math.max(...ma.map((value, index) => Math.abs(value - mb[index])));
}

function expectedMatrixMap(...sources) {
  const out = new Map();
  for (const source of sources) {
    if (!source) continue;
    if (Array.isArray(source)) {
      for (const entry of source) {
        const key = String(entry?.link_name || entry?.linkName || entry?.link || entry?.name || '').trim();
        const matrix = finiteMatrix16(entry);
        if (key && matrix) out.set(key, matrix);
      }
    } else if (typeof source === 'object') {
      for (const [key, value] of Object.entries(source)) {
        const matrix = finiteMatrix16(value);
        if (key && matrix) out.set(key, matrix);
      }
    }
  }
  return out;
}

function compareMatrixDiagnostics(actualByLink, expectedByLink, tolerance = 1e-5) {
  const comparisons = [];
  let pass = true;
  for (const [linkName, expectedMatrix] of expectedByLink.entries()) {
    const actualEntry = actualByLink instanceof Map ? actualByLink.get(linkName) : actualByLink?.[linkName];
    const actualMatrix = finiteMatrix16(actualEntry);
    const maxAbsDelta = matrixMaxAbsDelta(expectedMatrix, actualMatrix);
    const ok = Number.isFinite(maxAbsDelta) && maxAbsDelta <= tolerance;
    pass = pass && ok;
    comparisons.push({
      link_name: linkName,
      linkName,
      max_abs_delta: maxAbsDelta,
      maxAbsDelta,
      tolerance,
      pass: ok,
      expected_matrix_world: expectedMatrix,
      expectedMatrixWorld: expectedMatrix,
      actual_matrix_world: actualMatrix,
      actualMatrixWorld: actualMatrix,
    });
  }
  return { pass, tolerance, compared_count: comparisons.length, comparedCount: comparisons.length, comparisons };
}

function collectMatrixParityDiagnostics(previewConfig, diagnostics) {
  const tolerance = Number(previewConfig?.matrix_diagnostic_tolerance ?? previewConfig?.matrixDiagnosticTolerance ?? 1e-5);
  const linkExpected = expectedMatrixMap(
    previewConfig?.expected_robot_link_world_matrices,
    previewConfig?.expectedRobotLinkWorldMatrices,
    previewConfig?.robot_link_world_matrices_expected,
    previewConfig?.robotLinkWorldMatricesExpected
  );
  const visualExpected = expectedMatrixMap(
    previewConfig?.expected_robot_visual_wrapper_world_matrices,
    previewConfig?.expectedRobotVisualWrapperWorldMatrices,
    previewConfig?.robot_visual_wrapper_world_matrices_expected,
    previewConfig?.robotVisualWrapperWorldMatricesExpected
  );
  const linkParity = compareMatrixDiagnostics(diagnostics.robot_link_world_matrices || {}, linkExpected, tolerance);
  const visualActual = new Map((diagnostics.robot_visual_wrapper_world_matrices || []).map(entry => [String(entry?.link_name || entry?.linkName || '').trim(), entry]).filter(([key]) => key));
  const visualParity = compareMatrixDiagnostics(visualActual, visualExpected, tolerance);
  diagnostics.robot_matrix_world_parity = { tolerance, link: linkParity, visual_wrapper: visualParity, pass: linkParity.pass && visualParity.pass };
  diagnostics.robotMatrixWorldParity = diagnostics.robot_matrix_world_parity;
  return diagnostics.robot_matrix_world_parity;
}

function collectLinkMatrixDiagnostics(robot, links) {
  robot?.updateMatrixWorld?.(true);
  const out = {};
  for (const [name, link] of Object.entries(links || {})) {
    if (!link?.matrixWorld) continue;
    const position = new THREE.Vector3();
    const quaternion = new THREE.Quaternion();
    const scale = new THREE.Vector3();
    link.updateMatrixWorld?.(true);
    link.matrixWorld.decompose(position, quaternion, scale);
    out[name] = {
      link_name: name,
      name: link.name || name,
      parent_name: link.parent?.name || '',
      parent_link_name: Object.entries(links || {}).find(([, candidate]) => candidate === link.parent)?.[0] || '',
      matrix_world: matrix4ToDiagnostics(link),
      world_position: vector3ToDiagnostics(position),
      world_quaternion: { x: quaternion.x, y: quaternion.y, z: quaternion.z, w: quaternion.w },
      world_scale: vector3ToDiagnostics(scale),
    };
  }
  return out;
}

function isVisualWrapperCandidate(child, links) {
  if (!child || Object.values(links || {}).includes(child)) return false;
  const type = String(child.type || '').toLowerCase();
  const name = String(child.name || '').toLowerCase();
  return child.isMesh || type.includes('mesh') || type.includes('group') || name.includes('visual') || child.children?.some?.(desc => desc?.isMesh);
}

function collectVisualWrapperMatrixDiagnostics(links) {
  const out = [];
  for (const [linkName, link] of Object.entries(links || {})) {
    let visualIndex = 0;
    for (const child of link.children || []) {
      if (!isVisualWrapperCandidate(child, links)) continue;
      child.updateMatrixWorld?.(true);
      const position = new THREE.Vector3();
      const quaternion = new THREE.Quaternion();
      const scale = new THREE.Vector3();
      child.matrixWorld.decompose(position, quaternion, scale);
      out.push({
        link_name: linkName,
        linkName,
        visual_index: visualIndex,
        visualIndex,
        object_name: child.name || `visual_${visualIndex}`,
        objectName: child.name || `visual_${visualIndex}`,
        parent_name: child.parent?.name || '',
        parentName: child.parent?.name || '',
        matrix_world: matrix4ToDiagnostics(child),
        matrixWorld: matrix4ToDiagnostics(child),
        world_position: vector3ToDiagnostics(position),
        worldPosition: vector3ToDiagnostics(position),
        world_quaternion: { x: quaternion.x, y: quaternion.y, z: quaternion.z, w: quaternion.w },
        worldQuaternion: { x: quaternion.x, y: quaternion.y, z: quaternion.z, w: quaternion.w },
        world_scale: vector3ToDiagnostics(scale),
        worldScale: vector3ToDiagnostics(scale),
      });
      visualIndex += 1;
    }
  }
  return out;
}

function collectDescendantRenderMeshDiagnostics(links) {
  const out = [];
  for (const [linkName, link] of Object.entries(links || {})) {
    let visualIndex = 0;
    for (const visual of link.children || []) {
      if (!isVisualWrapperCandidate(visual, links)) continue;
      let meshIndex = 0;
      visual.updateMatrixWorld?.(true);
      visual.traverse?.(mesh => {
        if (!mesh?.isMesh) return;
        mesh.updateMatrixWorld?.(true);
        const position = new THREE.Vector3();
        const quaternion = new THREE.Quaternion();
        const scale = new THREE.Vector3();
        mesh.matrixWorld.decompose(position, quaternion, scale);
        const box = new THREE.Box3().setFromObject(mesh);
        const size = new THREE.Vector3();
        const center = new THREE.Vector3();
        const validBox = Number.isFinite(box.min.x) && Number.isFinite(box.max.x) && !box.isEmpty();
        if (validBox) {
          box.getSize(size);
          box.getCenter(center);
        }
        out.push({
          link_name: linkName,
          linkName,
          visual_index: visualIndex,
          visualIndex,
          mesh_index: meshIndex,
          meshIndex,
          object_name: mesh.name || `mesh_${meshIndex}`,
          objectName: mesh.name || `mesh_${meshIndex}`,
          parent_name: mesh.parent?.name || '',
          parentName: mesh.parent?.name || '',
          matrix_world: matrix4ToDiagnostics(mesh),
          matrixWorld: matrix4ToDiagnostics(mesh),
          world_position: vector3ToDiagnostics(position),
          worldPosition: vector3ToDiagnostics(position),
          world_quaternion: { x: quaternion.x, y: quaternion.y, z: quaternion.z, w: quaternion.w },
          worldQuaternion: { x: quaternion.x, y: quaternion.y, z: quaternion.z, w: quaternion.w },
          world_scale: vector3ToDiagnostics(scale),
          worldScale: vector3ToDiagnostics(scale),
          bounds_center: validBox ? vector3ToDiagnostics(center) : null,
          boundsCenter: validBox ? vector3ToDiagnostics(center) : null,
          bounds_size: validBox ? vector3ToDiagnostics(size) : null,
          boundsSize: validBox ? vector3ToDiagnostics(size) : null,
          bounds_valid: validBox,
          boundsValid: validBox,
        });
        meshIndex += 1;
      });
      visualIndex += 1;
    }
  }
  return out;
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

export function applyRobotJointPreview(result, jointValues = {}) {
  const robot = result?.root;
  if (!robot?.setJointValues) throw new Error('Product View is not ready');
  robot.setJointValues(jointValues);
  robot.updateMatrixWorld?.(true);
  const links = Object.fromEntries(result.links || []);
  result.diagnostics.robot_joint_values_applied = { ...jointValues };
  result.diagnostics.robot_link_world_matrices = collectLinkMatrixDiagnostics(robot, links);
  result.diagnostics.robotLinkWorldMatrices = result.diagnostics.robot_link_world_matrices;
  result.diagnostics.robot_visual_wrapper_world_matrices = collectVisualWrapperMatrixDiagnostics(links);
  result.diagnostics.robotVisualWrapperWorldMatrices = result.diagnostics.robot_visual_wrapper_world_matrices;
  result.diagnostics.robot_descendant_render_mesh_diagnostics = collectDescendantRenderMeshDiagnostics(links);
  result.diagnostics.robotDescendantRenderMeshDiagnostics = result.diagnostics.robot_descendant_render_mesh_diagnostics;
  collectMatrixParityDiagnostics(result?.previewConfig || {}, result.diagnostics);
  return result.diagnostics;
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

export { normalizeRosColladaScene, ROS_TO_THREE_CONVERSION_BOUNDARY };

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
    robot_collada_root_normalization_count: 0,
    robotColladaRootNormalizationCount: 0,
    robot_collada_mesh_diagnostics: [],
    robotColladaMeshDiagnostics: [],
    robot_collada_summary: [],
    robotColladaSummary: [],
    robot_package_mesh_resolutions: [],
    robot_descendant_render_mesh_diagnostics: [],
    robotDescendantRenderMeshDiagnostics: [],
    skipped_legacy_generated_urdf_visual_count: rendererContext?.skippedLegacyGeneratedUrdfVisualCount || 0,
  };
  const result = { root: null, links: new Map(), joints: new Map(), diagnostics, ready: null, previewConfig };

  result.ready = (async () => {
    setLifecycleState(diagnostics, 'loading_urdf');
    const manager = new THREE.LoadingManager();
    const meshCompletion = meshCompletionPromise(diagnostics, manager);
    const loader = new URDFLoader(manager);
    loader.parseVisual = true;
    loader.parseCollision = false;
    configureUrdfPackageResolution(loader, manager, rendererContext, diagnostics);
    loader.workingPath = '';
    loader.loadMeshCb = (path, meshManager, material, done) => loadMesh(path, meshManager, material, done, rendererContext, diagnostics);

    const urdfUrl = repoUrl(rendererContext, previewConfig?.urdf_url || '');
    const robot = await loader.loadAsync(urdfUrl);
    setLifecycleState(diagnostics, 'loading_meshes');
    robot.name = rendererContext?.rootName || 'workcell_studio_urdf_loader_robot';
    robot.userData.robot_render_mode = ROBOT_RENDER_MODE;
    robot.userData.robot_preview_source = 'gkjohnson/urdf-loaders urdf-loader@0.13.0';

    await meshCompletion.wait();

    const jointValues = previewConfig?.joint_values || {};
    // Transform chain for expanded previews is intentionally single-application:
    // world/root fixed chain -> URDF joint origin -> joint value -> link frame
    // -> URDF visual origin -> URDF mesh scale -> loader asset-coordinate conversion.
    // Delegate fixed/revolute/continuous/prismatic and mimic propagation to
    // urdf-loader's joint API; do not manually transform child link groups here.
    robot.setJointValues(jointValues);
    robot.updateMatrixWorld(true);

    result.root = robot;
    result.links = buildLookupMap(robot.links);
    result.joints = buildLookupMap(robot.joints);
    diagnostics.robot_loaded_link_count = result.links.size;
    diagnostics.robot_loaded_joint_count = result.joints.size;
    diagnostics.robot_joint_type_counts = jointTypeCounts(robot.joints);
    diagnostics.robot_hierarchy_links = Array.from(result.links.keys());
    diagnostics.robot_link_world_matrices = collectLinkMatrixDiagnostics(robot, robot.links);
    diagnostics.robotLinkWorldMatrices = diagnostics.robot_link_world_matrices;
    diagnostics.robot_visual_wrapper_world_matrices = collectVisualWrapperMatrixDiagnostics(robot.links);
    diagnostics.robotVisualWrapperWorldMatrices = diagnostics.robot_visual_wrapper_world_matrices;
    diagnostics.robot_descendant_render_mesh_diagnostics = collectDescendantRenderMeshDiagnostics(robot.links);
    diagnostics.robotDescendantRenderMeshDiagnostics = diagnostics.robot_descendant_render_mesh_diagnostics;
    collectMatrixParityDiagnostics(previewConfig, diagnostics);
    diagnostics.robot_collada_mesh_diagnostics = diagnostics.robot_collada_mesh_diagnostics || [];
    diagnostics.robotColladaMeshDiagnostics = diagnostics.robot_collada_mesh_diagnostics;
    diagnostics.robot_collada_summary = diagnostics.robot_collada_summary || [];
    diagnostics.robotColladaSummary = diagnostics.robot_collada_summary;
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
    diagnostics.robot_link_world_matrices = collectLinkMatrixDiagnostics(robot, robot.links);
    diagnostics.robotLinkWorldMatrices = diagnostics.robot_link_world_matrices;
    diagnostics.robot_visual_wrapper_world_matrices = collectVisualWrapperMatrixDiagnostics(robot.links);
    diagnostics.robotVisualWrapperWorldMatrices = diagnostics.robot_visual_wrapper_world_matrices;
    diagnostics.robot_descendant_render_mesh_diagnostics = collectDescendantRenderMeshDiagnostics(robot.links);
    diagnostics.robotDescendantRenderMeshDiagnostics = diagnostics.robot_descendant_render_mesh_diagnostics;
    diagnostics.robot_mesh_callbacks_complete = diagnostics.robot_expected_visual_count === (diagnostics.robot_loaded_visual_count + diagnostics.robot_failed_visual_count);
    diagnostics.robotMeshCallbacksComplete = diagnostics.robot_mesh_callbacks_complete;
    diagnostics.robot_preview_loaded = diagnostics.robot_failed_visual_count === 0 && diagnostics.robot_hierarchy_missing_links.length === 0 && diagnostics.robot_mesh_callbacks_complete;
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
