let THREE;
let OrbitControls;
let STLLoader;
let ColladaLoader;
let OBJLoader;
let TransformControls;

const SUPPORTED_SCHEMA_VERSION = 'workcell_studio_web_scene/v1';
const EDIT_PATCH_SCHEMA_VERSION = 'workcell_studio_web_scene_edit_patch/v1';
const VIEWER_VERSION = 'static_web_viewer_edit_patch_v1';
const LOCKED_EDIT_REASON = 'Locked/generated preview item; edit source layout/environment instead.';
const MIN_FRAME_RADIUS = 1.2;
const EMPTY_SCENE_MESSAGE = 'Scene contains no renderable robots, tools, assets, sensors, zones, items, or objects.';
const FRAME_DISTANCE_MULTIPLIER = 2.7;
const state = { sceneJson: null, sourceWebSceneFile: '', objects: [], selected: null, three: {}, animationId: null, lastFrameBounds: null, runtimeWarnings: [], labelsVisible: false, dirtyTransforms: new Map(), undoStack: [], redoStack: [], gizmoDragStart: null };
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
  canvas: document.getElementById('scene-canvas'),
  labelLayer: document.getElementById('label-layer'),
  empty: document.getElementById('empty-state'),
  error: document.getElementById('error-state'),
  list: document.getElementById('object-list'),
  inspector: document.getElementById('inspector'),
  warnings: document.getElementById('warnings'),
};

function showError(message) {
  el.error.textContent = message;
  el.error.hidden = false;
}
function clearError() { el.error.hidden = true; el.error.textContent = ''; }
function valueOrDash(value) { return value === undefined || value === null || value === '' ? '—' : value; }
function asArray(value) { return Array.isArray(value) ? value : []; }
function escapeHtml(value) { return String(valueOrDash(value)).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c])); }
function vector3(value, fallback = [0, 0, 0]) {
  const arr = Array.isArray(value) ? value : fallback;
  return new THREE.Vector3(Number(arr[0] || 0), Number(arr[1] || 0), Number(arr[2] || 0));
}
function poseOf(item) {
  const pose = item.pose || item.world_pose || item.baked_world_visual_pose || {};
  const xyz = item.pose_xyz || pose.xyz || pose.position || pose.translation || (Array.isArray(pose) ? pose.slice(0, 3) : [0, 0, 0]);
  const rpy = item.pose_rpy || pose.rpy || pose.rotation_rpy || (Array.isArray(pose) ? pose.slice(3, 6) : [0, 0, 0]);
  return { xyz: vector3(xyz), rpy: vector3(rpy) };
}
function scaleOf(item) {
  const scale = item.scale || item.mesh_scale || [1, 1, 1];
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
function cloneTransform(transform) { return JSON.parse(JSON.stringify(transform)); }
function sameTransform(a, b) { return JSON.stringify(a) === JSON.stringify(b); }
function renderedById(id) { return state.objects.find(obj => obj.item.id === id); }
function transformFromObject(object) {
  return { pose: { xyz: { x: object.position.x, y: object.position.y, z: object.position.z }, rpy: { x: object.rotation.x, y: object.rotation.y, z: object.rotation.z } }, scale: { x: object.scale.x, y: object.scale.y, z: object.scale.z } };
}
function translationSnapValue() { const v = Number(el.translationSnap?.value || 0); return Number.isFinite(v) && v > 0 ? v : null; }
function rotationSnapRadians() { const v = Number(el.rotationSnap?.value || 0); return Number.isFinite(v) && v > 0 ? THREE.MathUtils.degToRad(v) : null; }
function snapTransform(transform) {
  if (!el.snapToggle?.checked) return transform;
  const out = cloneTransform(transform);
  const t = translationSnapValue();
  if (t) for (const axis of ['x', 'y', 'z']) out.pose.xyz[axis] = Math.round(out.pose.xyz[axis] / t) * t;
  const r = rotationSnapRadians();
  if (r) for (const axis of ['x', 'y', 'z']) out.pose.rpy[axis] = Math.round(out.pose.rpy[axis] / r) * r;
  return out;
}
function applyTransformToObject(object, transform) {
  object.position.set(transform.pose.xyz.x, transform.pose.xyz.y, transform.pose.xyz.z);
  object.rotation.set(transform.pose.rpy.x, transform.pose.rpy.y, transform.pose.rpy.z, 'XYZ');
  object.scale.set(transform.scale.x, transform.scale.y, transform.scale.z);
}
function markDirtyTransform(rendered, next, { pushHistory = true, oldTransform = null } = {}) {
  if (!rendered || !canEditItem(rendered.item)) return;
  const previous = oldTransform || state.dirtyTransforms.get(rendered.item.id)?.newTransform || transformFromObject(rendered.object3d);
  const snapped = snapTransform(next);
  if (pushHistory && !sameTransform(previous, snapped)) {
    state.undoStack.push({ itemId: rendered.item.id, before: cloneTransform(previous), after: cloneTransform(snapped) });
    state.redoStack = [];
  }
  applyTransformToObject(rendered.object3d, snapped);
  if (sameTransform(rendered.originalTransform, snapped)) state.dirtyTransforms.delete(rendered.item.id);
  else state.dirtyTransforms.set(rendered.item.id, { oldTransform: cloneTransform(rendered.originalTransform), newTransform: cloneTransform(snapped) });
  syncInspectorTransformFields(rendered);
  updateDirtyState();
  updateLabels();
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
  rendered.object3d?.traverse?.(child => { child.userData.renderInfo = info; });
  return info;
}
function appendRuntimeWarning(item, meshUri, reason) {
  state.runtimeWarnings.push({
    source: 'runtime_mesh',
    code: 'mesh_primitive_fallback',
    object_id: item?.id || itemLabel(item || {}),
    link: item?.link || item?.object_name || item?.visual || '',
    object_name: item?.object_name || item?.link || itemLabel(item || {}),
    original_mesh_uri: item?.original_mesh_uri || item?.package_uri || item?.source_path || item?.mesh_path || meshUri || '',
    mesh_uri: meshUri || '',
    reason: reason || 'mesh loading skipped',
    message: `Primitive fallback for ${item?.id || itemLabel(item || {})} (${item?.link || item?.object_name || itemLabel(item || {})}): ${reason || 'mesh loading skipped'}`,
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
  const allowedRoots = [
    'build/workcell_studio_web_scene/assets/',
    'workcell_studio_web/',
    'assets/',
  ];
  if (!allowedRoots.some(root => pathOnly.startsWith(root))) {
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
function itemType(item) { return item.type || item.category || item.role || item.source_kind || 'asset'; }
function itemLabel(item) { return item.label || item.display_name || item.name || item.id || 'unnamed'; }
function viewerGroupIdentity(item) {
  return [
    item?.source_kind,
    item?.type,
    item?.category,
    item?.role,
    item?.id,
    itemLabel(item || {}),
  ].map(value => String(value || '').toLowerCase().replace(/[_-]+/g, ' ')).join(' ');
}
function viewerGroupFor(item) {
  const identity = viewerGroupIdentity(item);
  if (/\b(zone|pick zone|place zone|spawn zone|safety zone|work envelope|reachability|collision)\b/.test(identity)) return 'zones';
  if (/\b(camera|sensor|realsense|depth camera|rgbd|lidar|vision)\b/.test(identity)) return 'sensors';
  if (/\b(robot|arm|manipulator|ur3|ur5|ur10|tool|gripper|end effector|eef|suction|vacuum|robotiq|airpick|generated|generated preview|urdf|moveit)\b/.test(identity)) return 'robot/tool/generated';
  if (/\b(environment|layout|asset|object|item|table|workbench|fixture|bin|tray|conveyor|shelf|rack|pallet|floor|wall|part|product)\b/.test(identity)) return 'environment/layout';
  return 'unknown';
}
function isZone(item) { return viewerGroupFor(item) === 'zones'; }
function isSensor(item) { return viewerGroupFor(item) === 'sensors'; }
function shouldLabelItem(item) { return viewerGroupFor(item) !== 'zones'; }
function materialFor(item) {
  if (isZone(item)) return new THREE.MeshBasicMaterial({ color: 0xffc857, transparent: true, opacity: 0.22, side: THREE.DoubleSide });
  if (isSensor(item)) return new THREE.MeshStandardMaterial({ color: 0x62d2ff, roughness: 0.65 });
  if (item.locked || item.source_kind === 'generated_preview') return new THREE.MeshStandardMaterial({ color: 0x8794aa, roughness: 0.78, metalness: 0.05 });
  return new THREE.MeshStandardMaterial({ color: 0x7bd88f, roughness: 0.72 });
}
function dimensionsFromPrimitive(primitive) {
  if (!primitive) return [0.25, 0.25, 0.25];
  if (Array.isArray(primitive)) return primitive.slice(0, 3);
  return primitive.size || primitive.dimensions || primitive.extents || [primitive.x || primitive.width || 0.25, primitive.y || primitive.depth || 0.25, primitive.z || primitive.height || 0.25];
}
function makePrimitiveMesh(item) {
  const primitive = primitiveOf(item);
  const kind = String(item.geometry_type || item.primitive_geometry_type || primitive?.type || primitive?.shape || '').toLowerCase();
  const material = materialFor(item);
  let geometry;
  if (kind.includes('sphere')) geometry = new THREE.SphereGeometry(Number(primitive?.radius || 0.12), 24, 16);
  else if (kind.includes('cylinder')) geometry = new THREE.CylinderGeometry(Number(primitive?.radius || 0.08), Number(primitive?.radius || 0.08), Number(primitive?.height || 0.25), 24);
  else {
    const dims = dimensionsFromPrimitive(primitive);
    geometry = new THREE.BoxGeometry(Number(dims[0] || 0.25), Number(dims[1] || 0.25), Number(dims[2] || 0.25));
  }
  return new THREE.Mesh(geometry, material);
}
function makeSensorMarker(item) {
  const group = new THREE.Group();
  group.add(new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.08, 0.08), materialFor(item)));
  const frustum = new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0,0,0), new THREE.Vector3(0.35,0.22,-0.18), new THREE.Vector3(0,0,0), new THREE.Vector3(0.35,-0.22,-0.18),
      new THREE.Vector3(0,0,0), new THREE.Vector3(0.35,0.22,0.18), new THREE.Vector3(0,0,0), new THREE.Vector3(0.35,-0.22,0.18),
      new THREE.Vector3(0.35,0.22,-0.18), new THREE.Vector3(0.35,-0.22,-0.18), new THREE.Vector3(0.35,-0.22,-0.18), new THREE.Vector3(0.35,-0.22,0.18),
      new THREE.Vector3(0.35,-0.22,0.18), new THREE.Vector3(0.35,0.22,0.18), new THREE.Vector3(0.35,0.22,0.18), new THREE.Vector3(0.35,0.22,-0.18),
    ]),
    new THREE.LineBasicMaterial({ color: 0x62d2ff })
  );
  group.add(frustum);
  return group;
}
function applyPose(object, item) {
  const pose = poseOf(item);
  object.position.copy(pose.xyz);
  object.rotation.set(pose.rpy.x, pose.rpy.y, pose.rpy.z, 'XYZ');
  const s = scaleOf(item);
  object.scale.set(s.x, s.y, s.z);
}

function assignItemUserData(object, item) {
  object.userData.item = item;
  object.traverse?.(child => { child.userData.item = item; });
}
function materializeLoadedMesh(item, uri, loaded) {
  const ext = uri.split(/[?#]/, 1)[0].slice(uri.split(/[?#]/, 1)[0].lastIndexOf('.') + 1).toLowerCase();
  let object;
  if (ext === 'stl') object = new THREE.Mesh(loaded, materialFor(item));
  else if (ext === 'dae') object = loaded.scene;
  else object = loaded;
  object.name = `${item.id || itemLabel(item)}_mesh`;
  assignItemUserData(object, item);
  return object;
}
async function tryLoadMesh(item, rendered, fallback) {
  const diagnostic = meshUriDiagnostic(item);
  const uri = diagnostic.uri;
  const requestedUri = displayMeshUri(item);
  item.mesh_status = uri ? 'loading' : diagnostic.status;
  if (!uri) {
    setRenderInfo(rendered, rendered.renderInfo?.render_status || 'box_fallback', requestedUri, diagnostic.reason);
    if (requestedUri) appendRuntimeWarning(item, requestedUri, diagnostic.reason);
    if (state.selected === item.id) populateInspector(rendered);
    return;
  }
  try {
    const ext = uri.split(/[?#]/, 1)[0].slice(uri.split(/[?#]/, 1)[0].lastIndexOf('.') + 1).toLowerCase();
    let loaded;
    if (ext === 'stl') loaded = await new STLLoader().loadAsync(uri);
    else if (ext === 'dae') loaded = await new ColladaLoader().loadAsync(uri);
    else loaded = await new OBJLoader().loadAsync(uri);
    const meshObject = materializeLoadedMesh(item, uri, loaded);
    fallback.visible = false;
    rendered.object3d.add(meshObject);
    rendered.meshObject = meshObject;
    item.mesh_status = 'loaded';
    item.mesh_load_error = '';
    setRenderInfo(rendered, 'mesh_loaded', uri, '');
    const bounds = computeRenderedBounds();
    if (bounds) frameScene(bounds);
    if (state.selected === item.id) populateInspector(rendered);
  } catch (err) {
    fallback.visible = true;
    item.mesh_status = 'load_error';
    item.mesh_load_error = err?.message || String(err);
    const reason = `mesh loader failed: ${item.mesh_load_error}`;
    setRenderInfo(rendered, rendered.renderInfo?.render_status || 'box_fallback', uri, reason);
    appendRuntimeWarning(item, uri, reason);
    if (state.selected === item.id) populateInspector(rendered);
  }
}
function collectItems(sceneJson) {
  const buckets = ['robots', 'tools', 'assets', 'sensors', 'zones', 'items', 'objects'];
  const byId = new Map();
  for (const bucket of buckets) for (const item of asArray(sceneJson[bucket])) if (item && typeof item === 'object') byId.set(item.id || `${bucket}_${byId.size}`, { ...item, id: item.id || `${bucket}_${byId.size}` });
  return Array.from(byId.values());
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
    const renderer = new THREE.WebGLRenderer({ canvas: el.canvas, antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b1018);
    const camera = new THREE.PerspectiveCamera(55, 1, 0.01, 100);
    camera.position.set(2.4, -2.8, 1.8);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    scene.add(new THREE.GridHelper(5, 20, 0x3a4a5e, 0x263445));
    scene.add(new THREE.AxesHelper(0.75));
    scene.add(new THREE.HemisphereLight(0xffffff, 0x223344, 1.2));
    const light = new THREE.DirectionalLight(0xffffff, 1.5); light.position.set(2, -3, 4); scene.add(light);
    const transformControls = new TransformControls(camera, renderer.domElement);
    transformControls.setMode('translate');
    transformControls.addEventListener('dragging-changed', event => {
      controls.enabled = !event.value;
      const rendered = renderedById(state.selected);
      if (!rendered || !canEditItem(rendered.item)) return;
      if (event.value) state.gizmoDragStart = cloneTransform(state.dirtyTransforms.get(rendered.item.id)?.newTransform || transformFromObject(rendered.object3d));
      else { markDirtyTransform(rendered, transformFromObject(rendered.object3d), { pushHistory: true, oldTransform: state.gizmoDragStart }); state.gizmoDragStart = null; }
    });
    transformControls.addEventListener('objectChange', () => {
      const rendered = renderedById(state.selected);
      if (!rendered || !canEditItem(rendered.item)) return;
      const snapped = snapTransform(transformFromObject(rendered.object3d));
      applyTransformToObject(rendered.object3d, snapped);
      syncInspectorTransformFields(rendered);
      updateLabels();
    });
    scene.add(transformControls);
    state.three = { renderer, scene, camera, controls, transformControls, raycaster: new THREE.Raycaster(), pointer: new THREE.Vector2() };
    resize();
    window.addEventListener('resize', resize);
    el.canvas.addEventListener('pointerdown', pickObject);
    animate();
  } catch (err) {
    showError(`Three.js/CDN load failure: ${err.message || err}`);
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

function computeRenderedBounds() {
  const bounds = new THREE.Box3();
  for (const rendered of state.objects) {
    if (!rendered.object3d) continue;
    rendered.object3d.updateWorldMatrix(true, true);
    bounds.expandByObject(rendered.object3d);
  }
  return bounds.isEmpty() ? null : bounds;
}
function frameScene(bounds) {
  const { camera, controls } = state.three;
  if (!camera || !controls || !bounds || bounds.isEmpty()) return false;
  const center = new THREE.Vector3();
  const sphere = new THREE.Sphere();
  bounds.getCenter(center);
  bounds.getBoundingSphere(sphere);
  const radius = Math.max(sphere.radius, MIN_FRAME_RADIUS);
  const direction = new THREE.Vector3(1.35, -1.65, 1.05).normalize();
  const distance = Math.max(radius * FRAME_DISTANCE_MULTIPLIER, MIN_FRAME_RADIUS * FRAME_DISTANCE_MULTIPLIER);
  camera.position.copy(center).addScaledVector(direction, distance);
  camera.near = Math.max(0.01, radius / 100);
  camera.far = Math.max(100, distance + radius * 6);
  camera.updateProjectionMatrix();
  controls.target.copy(center);
  controls.update();
  state.lastFrameBounds = bounds.clone();
  if (el.resetView) el.resetView.disabled = false;
  return true;
}
function resetView() {
  const bounds = state.lastFrameBounds || computeRenderedBounds();
  if (bounds) frameScene(bounds);
}

function clearLabels() {
  if (el.labelLayer) el.labelLayer.innerHTML = '';
}
function clearSceneObjects() {
  const scene = state.three.scene;
  if (!scene) return;
  for (const rendered of state.objects) scene.remove(rendered.object3d);
  clearLabels();
  state.objects = [];
  state.lastFrameBounds = null;
  if (el.resetView) el.resetView.disabled = true;
}
function renderScene(items) {
  clearSceneObjects();
  state.dirtyTransforms.clear();
  state.undoStack = [];
  state.redoStack = [];
  detachTransformGizmo();
  updateDirtyState();
  const scene = state.three.scene;
  for (const item of items) {
    const object3d = new THREE.Group();
    const primitive = primitiveOf(item);
    const fallback = isSensor(item) ? makeSensorMarker(item) : makePrimitiveMesh(item);
    fallback.name = `${item.id || itemLabel(item)}_fallback`;
    assignItemUserData(fallback, item);
    object3d.add(fallback);
    applyPose(object3d, item);
    assignItemUserData(object3d, item);
    scene.add(object3d);
    const rendered = { item, object3d, fallback, labelEl: createLabelElement(item), originalTransform: transformOf(item) };
    const fallbackStatus = primitive || isSensor(item) ? 'primitive_fallback' : 'box_fallback';
    const fallbackReason = primitive || isSensor(item) ? 'primitive geometry rendered while mesh loads or is unavailable' : 'no primitive geometry or mesh was provided; using box fallback';
    setRenderInfo(rendered, fallbackStatus, displayMeshUri(item), fallbackReason);
    state.objects.push(rendered);
    tryLoadMesh(item, rendered, fallback);
  }
  populateObjectList();
  updateLabels();
  const bounds = computeRenderedBounds();
  if (bounds) frameScene(bounds);
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
    'robot/tool/generated': 'Robot, tool & generated',
    'environment/layout': 'Environment & layout',
    sensors: 'Sensors',
    zones: 'Zones',
    unknown: 'Unknown',
  };
  const grouped = new Map(Object.keys(groupLabels).map(group => [group, []]));
  for (const rendered of state.objects) {
    const group = viewerGroupFor(rendered.item);
    if (!grouped.has(group)) grouped.set(group, []);
    grouped.get(group).push(rendered);
  }
  for (const [group, renderedItems] of grouped.entries()) {
    if (!renderedItems.length) continue;
    const heading = document.createElement('li');
    heading.className = 'object-group-heading';
    heading.textContent = groupLabels[group] || group;
    el.list.appendChild(heading);
    for (const rendered of renderedItems) {
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

      const meta = document.createElement('span');
      meta.className = 'meta';
      meta.textContent = `${group} · ${rendered.item.locked ? 'locked/generated' : 'editable/environment'}${state.dirtyTransforms.has(rendered.item.id) ? ' · edited' : ''}`;
      li.appendChild(meta);
      li.addEventListener('click', () => selectObject(rendered.item.id));
      el.list.appendChild(li);
    }
  }
}
function selectObject(id) {
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
  if (rendered) { populateInspector(rendered); attachTransformGizmo(rendered); }
}
function pickObject(event) {
  const rect = el.canvas.getBoundingClientRect();
  state.three.pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  state.three.pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  state.three.raycaster.setFromCamera(state.three.pointer, state.three.camera);
  const hits = state.three.raycaster.intersectObjects(state.objects.map(o => o.object3d), true);
  const hit = hits.find(h => h.object?.parent || h.object);
  const item = hit?.object?.userData?.item || hit?.object?.parent?.userData?.item;
  if (item?.id) selectObject(item.id);
}

function attachTransformGizmo(rendered) {
  const gizmo = state.three.transformControls;
  if (!gizmo) return;
  if (rendered && canEditItem(rendered.item)) {
    gizmo.attach(rendered.object3d);
    gizmo.visible = true;
    gizmo.enabled = true;
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
  const entry = state.undoStack.pop();
  if (!entry) return;
  applyHistoryEntry(entry, 'undo');
  state.redoStack.push(entry);
  updateDirtyState();
}
function redoPreviewEdit() {
  const entry = state.redoStack.pop();
  if (!entry) return;
  applyHistoryEntry(entry, 'redo');
  state.undoStack.push(entry);
  updateDirtyState();
}
function sceneId() { return state.sceneJson?.scene?.id || state.sceneJson?.scene_id || ''; }
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
    id: item.id, label: itemLabel(item), type: itemType(item), source: item.source_kind || item.source || valueOrDash(item.provenance && Object.values(item.provenance)[0]),
    'pose xyz': [pose.xyz.x, pose.xyz.y, pose.xyz.z].map(n => n.toFixed(3)).join(', '),
    'pose rpy': [pose.rpy.x, pose.rpy.y, pose.rpy.z].map(n => n.toFixed(3)).join(', '),
    scale: JSON.stringify(item.scale || item.mesh_scale || [1, 1, 1]), editable: String(Boolean(item.editable)), locked: String(Boolean(item.locked)),
    render_status: renderInfo.render_status, mesh_uri: renderInfo.mesh_uri || displayMeshUri(item), fallback_reason: renderInfo.fallback_reason,
    mesh_status: item.mesh_status, mesh_load_error: item.mesh_load_error,
    original_mesh_uri: item.original_mesh_uri, mesh_staging_status: item.mesh_staging_status,
    mesh_staged_path: item.mesh_staged_path, mesh_resolve_warning: item.mesh_resolve_warning,
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
  if (!warnings.length) { el.warnings.className = 'warnings state empty'; el.warnings.textContent = 'No JSON or runtime mesh warnings.'; return; }
  el.warnings.className = 'warnings';
  el.warnings.innerHTML = warnings.map(w => {
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
    const items = validateSceneJson(json);
    state.sceneJson = json;
    state.sourceWebSceneFile = file.name || '';
    state.runtimeWarnings = [];
    state.dirtyTransforms.clear();
    state.undoStack = [];
    state.redoStack = [];
    state.selected = null;
    detachTransformGizmo();
    el.empty.hidden = true;
    if (items.length) renderScene(items);
    else renderScene([]);
    refreshWarnings(json);
    el.inspector.className = 'state empty';
    el.inspector.textContent = items.length ? 'Select an object from the list or canvas.' : EMPTY_SCENE_MESSAGE;
  } catch (err) {
    showError(err.message || String(err));
  }
}

if (el.resetView) el.resetView.addEventListener('click', resetView);
if (el.labelsToggle) el.labelsToggle.addEventListener('change', event => setLabelsVisible(event.target.checked));
if (el.undoEdit) el.undoEdit.addEventListener('click', undoPreviewEdit);
if (el.redoEdit) el.redoEdit.addEventListener('click', redoPreviewEdit);
if (el.clearEdits) el.clearEdits.addEventListener('click', clearPreviewEdits);
if (el.snapToggle) el.snapToggle.addEventListener('change', refreshGizmoSnap);
if (el.translationSnap) el.translationSnap.addEventListener('input', refreshGizmoSnap);
if (el.rotationSnap) el.rotationSnap.addEventListener('input', refreshGizmoSnap);
if (el.exportEditPatch) el.exportEditPatch.addEventListener('click', exportEditPatch);

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
    THREE = threeModule;
    OrbitControls = controlsModule.OrbitControls;
    TransformControls = transformControlsModule.TransformControls;
    STLLoader = stlModule.STLLoader;
    ColladaLoader = colladaModule.ColladaLoader;
    OBJLoader = objModule.OBJLoader;
    initThree();
    setLabelsVisible(el.labelsToggle?.checked || false);
  } catch (err) {
    showError(`Three.js/CDN load failure: ${err.message || err}`);
  }
}

boot();
