let THREE;
let OrbitControls;

const SUPPORTED_SCHEMA_VERSION = 'workcell_studio_web_scene/v1';
const state = { sceneJson: null, objects: [], selected: null, three: {}, animationId: null };
const el = {
  file: document.getElementById('scene-file'),
  canvas: document.getElementById('scene-canvas'),
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
function primitiveOf(item) {
  return item.primitive || item.primitive_details || item.dimensions || item.geometry || item.primitive_geometry || null;
}
function itemType(item) { return item.type || item.category || item.role || item.source_kind || 'asset'; }
function itemLabel(item) { return item.label || item.display_name || item.name || item.id || 'unnamed'; }
function isZone(item) { return /zone|pick|place|spawn|safety/i.test(`${itemType(item)} ${item.id || ''} ${itemLabel(item)}`); }
function isSensor(item) { return /camera|sensor|realsense/i.test(`${itemType(item)} ${item.id || ''} ${itemLabel(item)}`); }
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
  object.scale.multiply(s);
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
  if (!items.length) throw new Error('Missing/empty assets: web_scene.json contains no robots, tools, assets, sensors, zones, items, or objects to render.');
  return items;
}
function initThree() {
  try {
    if (!THREE?.Scene || !OrbitControls) throw new Error('Three.js modules were not available.');
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
    state.three = { renderer, scene, camera, controls, raycaster: new THREE.Raycaster(), pointer: new THREE.Vector2() };
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
}
function clearSceneObjects() {
  const scene = state.three.scene;
  if (!scene) return;
  for (const rendered of state.objects) scene.remove(rendered.object3d);
  state.objects = [];
}
function renderScene(items) {
  clearSceneObjects();
  const scene = state.three.scene;
  for (const item of items) {
    let object3d = isSensor(item) ? makeSensorMarker(item) : makePrimitiveMesh(item);
    if (!primitiveOf(item) && item.mesh_uri) object3d.userData.viewerWarning = 'Mesh loading is intentionally excluded; showing box fallback.';
    applyPose(object3d, item);
    object3d.userData.item = item;
    scene.add(object3d);
    state.objects.push({ item, object3d });
  }
  populateObjectList();
}
function populateObjectList() {
  el.list.innerHTML = '';
  for (const rendered of state.objects) {
    const li = document.createElement('li');
    li.dataset.id = rendered.item.id;
    li.textContent = `${rendered.item.id} — ${itemLabel(rendered.item)}`;
    const meta = document.createElement('span');
    meta.className = 'meta';
    meta.textContent = `${itemType(rendered.item)} · ${rendered.item.locked ? 'locked/generated' : 'editable/environment'}`;
    li.appendChild(meta);
    li.addEventListener('click', () => selectObject(rendered.item.id));
    el.list.appendChild(li);
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
  }
  const rendered = state.objects.find(obj => obj.item.id === id);
  if (rendered) populateInspector(rendered.item);
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
function populateInspector(item) {
  const pose = poseOf(item);
  const rows = {
    id: item.id, label: itemLabel(item), type: itemType(item), source: item.source_kind || item.source || valueOrDash(item.provenance && Object.values(item.provenance)[0]),
    'pose xyz': [pose.xyz.x, pose.xyz.y, pose.xyz.z].map(n => n.toFixed(3)).join(', '),
    'pose rpy': [pose.rpy.x, pose.rpy.y, pose.rpy.z].map(n => n.toFixed(3)).join(', '),
    scale: JSON.stringify(item.scale || item.mesh_scale || [1, 1, 1]), editable: String(Boolean(item.editable)), locked: String(Boolean(item.locked)),
    mesh_uri: item.mesh_uri || item.package_uri || item.mesh_path || item.source_path, primitive: JSON.stringify(primitiveOf(item) || 'box fallback'),
  };
  el.inspector.className = '';
  el.inspector.innerHTML = `<table class="inspector-table"><tbody>${Object.entries(rows).map(([k,v]) => `<tr><th>${k}</th><td><code>${String(valueOrDash(v)).replace(/[&<>]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}</code></td></tr>`).join('')}</tbody></table>`;
}
function populateWarnings(sceneJson) {
  const warnings = asArray(sceneJson.warnings).concat(asArray(sceneJson.notes_warnings));
  if (!warnings.length) { el.warnings.className = 'warnings state empty'; el.warnings.textContent = 'No warnings in JSON.'; return; }
  el.warnings.className = 'warnings';
  el.warnings.innerHTML = warnings.map(w => `<div class="warning-item"><strong>${valueOrDash(w.code || w.source || 'warning')}</strong><br>${valueOrDash(w.message || JSON.stringify(w))}</div>`).join('');
}
async function loadFile(file) {
  clearError();
  try {
    const text = await file.text();
    let json;
    try { json = JSON.parse(text); } catch (err) { throw new Error(`Invalid JSON in ${file.name}: ${err.message}`); }
    const items = validateSceneJson(json);
    state.sceneJson = json;
    el.empty.hidden = true;
    renderScene(items);
    populateWarnings(json);
    el.inspector.className = 'state empty';
    el.inspector.textContent = 'Select an object from the list or canvas.';
  } catch (err) {
    showError(err.message || String(err));
  }
}

el.file.addEventListener('change', event => {
  const file = event.target.files?.[0];
  if (file) loadFile(file);
});

async function boot() {
  try {
    const threeModule = await import('three');
    const controlsModule = await import('three/addons/controls/OrbitControls.js');
    THREE = threeModule;
    OrbitControls = controlsModule.OrbitControls;
    initThree();
  } catch (err) {
    showError(`Three.js/CDN load failure: ${err.message || err}`);
  }
}

boot();
