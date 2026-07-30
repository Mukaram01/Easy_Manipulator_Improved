import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web" / "viewer"


def test_selection_identity_index_and_expanded_urdf_picks_behave_end_to_end(tmp_path):
    harness = r"""
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
const element = () => ({ hidden:false, checked:false, disabled:false, textContent:'', className:'', innerHTML:'', classList:{toggle(){}}, querySelector(){return {textContent:''}}, appendChild(){}, addEventListener(){}, setAttribute(){} });
const context = { console, assert, window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}}}, document:{getElementById(){return element()},createElement(){return element()}}, URLSearchParams, CustomEvent:function(){}, requestAnimationFrame(){return 0}, setTimeout(){return 0}, clearTimeout(){} };
vm.createContext(context);
vm.runInContext(source + `
const cameraOwner = { id:'camera_owner', category:'camera' };
const tableOwner = { id:'table_owner', category:'table' };
const robotOwner = { id:'configured_robot', category:'robot', readiness_category:'robot_arm', locked:true, editable:false };
const toolOwner = { id:'configured_tool', category:'tool', readiness_category:'attached_tool_gripper', locked:true, editable:false };
const cameraMapped = { id:'camera_payload_visual', link_name:'fixture_camera_link', canonical_scene_item_id:'camera_owner' };
const cameraClicked = { id:'generated_urdf::fixture_camera_link::visual_17::17', link_name:'fixture_camera_link' };
const tableMapped = { id:'table_payload_visual', final_render_link:'fixture_table_link', support_surface_ref:'table_owner' };
const tableClicked = { id:'generated_urdf::fixture_table_link::visual_4::4', canonical_link_name:'fixture_table_link' };
state.sceneJson = { scene:{id:'selection_scene'}, objects:[cameraOwner, tableOwner, robotOwner, toolOwner, cameraMapped, cameraClicked, tableMapped, tableClicked], robot_preview:{ mode:'expanded_urdf_loader', robot_instance_id:'configured_robot', tool_id:'configured_tool', expected_robot_visual_links:['arm_link'], expected_tool_visual_links:['finger_link'] } };
rebuildSelectionIdentityIndex();
let camera = uiSelectionIdentity({item:cameraClicked, pickRecordSource:'payload_item'});
assert.deepStrictEqual(JSON.parse(JSON.stringify(camera)), {id:'camera_owner', resolution:'exact_link_explicit_ref', linkName:'fixture_camera_link', pickRecordSource:'payload_item'});
let table = uiSelectionIdentity({item:tableClicked, pickRecordSource:'payload_item'});
assert.strictEqual(table.id, 'table_owner');
assert.strictEqual(table.resolution, 'exact_link_explicit_ref');

state.sceneJson.objects.push({id:'other_owner'}, {id:'ambiguous_mapping', link_name:'fixture_camera_link', canonical_item_id:'other_owner'});
rebuildSelectionIdentityIndex();
assert.strictEqual(explicitUiSelectionItemId({item:cameraClicked}), cameraClicked.id);
state.sceneJson.objects.splice(-2);
rebuildSelectionIdentityIndex();

let callbacks = [];
loadRobotPreview = (_preview, options) => { callbacks.push(options); return {diagnostics:{}, links:new Map()}; };
state.three = {scene:{add(){}}};
state.web3dReadiness = {state:'scene_loading', required:{}, pending:new Set(), failed:false};
renderSceneSummary = () => {};
refreshInitialPoseActionState = () => {};
failIfExpandedUrdfExpectedVisualSetInvalid = () => false;
completeExpandedUrdfReadiness = () => {};
loadExpandedUrdfRobotPreview(state.sceneJson.robot_preview);
const armObject = {visible:true};
const fingerObject = {visible:true};
callbacks[0].onRobotLoaded({root:{}, links:new Map([['arm_link',armObject],['finger_link',fingerObject]]), diagnostics:{}});
assert.strictEqual(state.pickRecords.length, 2);
const armPick = state.pickRecords.find(record => record.item.link_name === 'arm_link');
const fingerPick = state.pickRecords.find(record => record.item.link_name === 'finger_link');
assert.strictEqual(armPick.pickRecordSource, 'expanded_urdf_inspection');
assert.strictEqual(uiSelectionIdentity(armPick).id, 'configured_robot');
assert.strictEqual(uiSelectionIdentity(armPick).resolution, 'robot_owner');
assert.strictEqual(uiSelectionIdentity(fingerPick).id, 'configured_tool');
assert.strictEqual(uiSelectionIdentity(fingerPick).resolution, 'tool_owner');
state.selected = armPick.item.id;
let diagnostic = currentSelectionDiagnostics();
assert.strictEqual(diagnostic.selectedItemId, armPick.item.id);
assert.strictEqual(diagnostic.uiSelectionItemId, 'configured_robot');
assert.strictEqual(diagnostic.pickRecordSource, 'expanded_urdf_inspection');
assert.strictEqual(state.objects.some(record => record.item.id === armPick.item.id), false);
assert.strictEqual(state.dirtyTransforms.has(armPick.item.id), false);
assert.strictEqual(buildEditPatch().edits.some(edit => edit.id === armPick.item.id), false);

const stale = callbacks[0];
state.sceneJson = {scene:{id:'next_scene'}, objects:[]};
resetSceneLifecycleState();
const before = state.pickRecords.length;
stale.onRobotLoaded({root:{}, links:new Map([['arm_link',{visible:true}]]), diagnostics:{}});
assert.strictEqual(state.pickRecords.length, before);
`, context);
"""
    subprocess.run(
        ["node", "-e", harness, str(VIEWER / "viewer.js")],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def test_static_viewer_files_exist():
    assert (VIEWER / "index.html").is_file()
    assert (VIEWER / "viewer.js").is_file()
    assert (VIEWER / "style.css").is_file()
    assert (VIEWER / "README.md").is_file()


def test_index_references_static_assets():
    index = (VIEWER / "index.html").read_text(encoding="utf-8")
    assert 'href="style.css"' in index
    assert 'src="./dist/viewer.bundle.js"' in index or 'src="dist/viewer.bundle.js"' in index
    assert 'id="scene-file"' in index
    assert 'scene-file' in index


def test_viewer_js_schema_and_inspector_hooks():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    assert "SUPPORTED_SCHEMA_VERSION" in js
    assert "workcell_studio_web_scene/v1" in js
    assert "schema_version" in js
    assert "populateInspector" in js
    assert "mesh_uri" in js
    assert "primitive" in js
    assert "editable" in js
    assert "locked" in js
    assert "original_mesh_uri" in js
    assert "mesh_staging_status" in js
    assert "mesh_staged_path" in js
    assert "mesh_resolve_warning" in js


def test_viewer_records_render_status_for_inspector():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    assert "setRenderInfo" in js
    for token in [
        "render_status",
        "fallback_reason",
        "mesh_loaded",
        "primitive_fallback",
        "box_fallback",
        "load_error",
    ]:
        assert token in js
    assert "child.userData.renderInfo" in js
    assert "rendered.renderInfo" in js


def test_viewer_exposes_browser_acceptance_status_hook():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "window.__WORKCELL_VIEWER_STATUS__",
        "scene_name",
        "renderable_count",
        "mesh_loaded_count",
        "required_mesh_failed_count",
        "fallback_count",
        "runtime_warnings",
        "updateViewerStatus",
    ]:
        assert token in js


def test_viewer_allows_valid_empty_scene_with_empty_message():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    assert "EMPTY_SCENE_MESSAGE" in js
    assert "Scene contains no renderable robots, tools, assets, sensors, zones, items, or objects." in js
    validate_body = js.split("function validateSceneJson(json)", 1)[1].split("function initThree()", 1)[0]
    assert "return items;" in validate_body
    assert "!items.length" not in validate_body
    assert "else renderScene([]);" in js
    assert "li.textContent = EMPTY_SCENE_MESSAGE" in js
    assert "items.length ? 'Select an object from the list or canvas.' : EMPTY_SCENE_MESSAGE" in js


def test_viewer_keeps_invalid_json_and_schema_errors_clear():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    assert "Invalid web_scene.json: expected a JSON object." in js
    assert "Invalid JSON in ${file.name}: ${err.message}" in js
    assert "Unsupported schema_version" in js
    assert "Expected ${SUPPORTED_SCHEMA_VERSION}" in js


def test_viewer_includes_auto_frame_and_reset_view_helpers():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "MIN_FRAME_RADIUS",
        "FRAME_DISTANCE_MULTIPLIER",
        "lastFrameBounds",
        "computeRenderedBounds",
        "frameScene",
        "resetView",
        "finiteBounds.getBoundingSphere",
        "controls.target.copy(center)",
        "el.resetView.addEventListener('click', resetView)",
    ]:
        assert token in js


def test_viewer_includes_render_status_strings_and_fallback_reasons():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "render_status",
        "fallback_reason",
        "mesh_loaded",
        "primitive_fallback",
        "box_fallback",
        "load_error",
        "loading",
        "missing_file",
        "unsafe_path",
        "unsupported_format",
        "unresolved_package_uri",
        "primitive geometry rendered while mesh loads or is unavailable",
        "no mesh or physical primitive dimensions were provided; Product View box fallback suppressed",
        "unsafe mesh_uri rejected by viewer policy",
        "no mesh_uri provided",
        "loader_failure",
    ]:
        assert token in js


def test_product_view_physical_dimension_semantics_do_not_require_overlays():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "PHYSICAL_SEMANTIC_TOKEN_RE",
        "conveyor",
        "workpiece",
        "bin",
        "support surface",
        "fixture",
        "pallet",
        "physical safety barrier",
        "function isPhysicalSemanticItem(item)",
        "function hasDimensionBackedPhysicalPrimitive(item)",
        "isPhysicalSemanticItem(item) && !/\\b(safety zone|pick zone|place zone|observation zone|spawn zone|work envelope|robot reach|camera fov|fov|home pose|transform anchor|warning marker|warning anchor|warning badge)\\b/",
        "object3d.visible = state.debugOverlaysVisible || !isDebugOverlayItem(item)",
    ]:
        assert token in js


def test_product_view_helper_semantics_stay_overlay_gated():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    overlay_body = js.split("const DEBUG_OVERLAY_TOKEN_RE", 1)[1].split("function isSensor(item)", 1)[0]
    for token in [
        "robot reach",
        "camera fov",
        "pick zone",
        "place zone",
        "observation zone",
        "home pose",
        "transform anchor",
        "warning marker",
        "viewerGroupFor(item) === 'zones'",
        "return DEBUG_OVERLAY_TOKEN_RE.test(identity)",
    ]:
        assert token in overlay_body


def test_product_view_suppresses_missing_mesh_box_without_physical_dimensions():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "if (!primitive) return null",
        "return numeric.every(value => Number.isFinite(value) && value > 0) ? numeric : null",
        "if (!dims) return null",
        "no_physical_dimensions",
        "no mesh or physical primitive dimensions were provided; Product View box fallback suppressed",
        "if (fallback) fallback.visible = !requiredMesh",
    ]:
        assert token in js


def test_viewer_includes_mesh_loader_references_and_safe_mesh_uri_logic():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "STLLoader",
        "ColladaLoader",
        "OBJLoader",
        "three/addons/loaders/STLLoader.js",
        "three/addons/loaders/ColladaLoader.js",
        "three/addons/loaders/OBJLoader.js",
        "safeMeshUri",
        "displayMeshUri",
        "loadAsync(loadUrl)",
        "materializeLoadedMesh",
        "appendRuntimeWarning",
        "build/workcell_studio_web_scene/assets/",
        "workcell_studio_web/",
        "assets/",
        "['stl', 'dae', 'obj'].includes(ext)",
    ]:
        assert token in js
    for unsafe_token in [
        "lower.startsWith('package://')",
        "lower.startsWith('http://')",
        "lower.startsWith('https://')",
        "lower.startsWith('file://')",
        "lower.startsWith('data:')",
        "lower.startsWith('//')",
        "uri.startsWith('/')",
        r"uri.startsWith('\\')",
        "part === '..'",
    ]:
        assert unsafe_token in js


def test_urdf_renderer_resolves_package_meshes_to_staged_scene_assets():
    js = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    assert "function resolvePackageMeshUri(uri, sceneId, diagnostics)" in js
    assert "raw.startsWith('package://')" in js
    assert "context?.sceneId" in js
    assert "${STAGED_MESH_ASSET_ROOT}${scene}/${packageName}/${safeParts.join('/')}" in js
    assert "package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae" not in js
    assert "URDF package mesh resolved:" in js
    assert "URDF package mesh rejected:" in js


def test_expanded_urdf_robot_preview_callbacks_are_scene_current_guarded():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    clear_body = js.split("function clearSceneObjects()", 1)[1].split("function renderScene(items)", 1)[0]
    load_body = js.split("function loadExpandedUrdfRobotPreview(preview)", 1)[1].split("function linkNameOfItem(item)", 1)[0]

    assert "let robotPreviewLoadToken = 0;" in js
    assert "robotPreviewLoadToken += 1;" in js
    assert "state.assemblyRoots = [];" in js
    assert "state.objects = [];" in js
    assert "state.robotPreviewResult = null;" in js
    assert "state.robotUrdfPreviewDiagnostics = {};" in js

    assert "const loadToken = ++robotPreviewLoadToken;" in load_body
    assert "const loadSceneId = sceneId();" in load_body
    assert "loadToken === robotPreviewLoadToken && loadSceneId === sceneId()" in load_body
    assert "sceneId: loadSceneId" in load_body
    assert "if (callbackIsCurrent()) state.three.scene?.add?.(root);" in load_body
    assert "if (callbackIsCurrent()) state.assemblyRoots.push(root);" in load_body
    assert "Ignored stale robot preview callback: callback_scene=${loadSceneId} active_scene=${sceneId()}" in load_body

    guard_token = "if (!callbackIsCurrent()) return ignoreStaleCallback();"
    callback_mutations = {
        "onRobotLoaded: result => {": "state.robotPreviewResult = result;",
        "onRobotMeshLoaded: () => {": "renderSceneSummary();",
        "onRobotMeshLoadError: (err, uri, detail) => {": "failExpandedUrdfReadiness(err, state.robotUrdfPreviewDiagnostics, detail || { uri });",
        "onRobotError: (err, diagnostics) => {": "failExpandedUrdfReadiness(err, diagnostics || state.robotUrdfPreviewDiagnostics);",
    }
    for callback, mutation in callback_mutations.items():
        section = load_body.split(callback, 1)[1].split("},", 1)[0]
        assert guard_token in section
        assert section.find(guard_token) < section.find(mutation)



def test_expanded_urdf_stale_callbacks_cannot_mutate_new_scene_runtime_state():
    js_path = VIEWER / "viewer.js"
    harness = r'''
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
const element = () => ({ hidden: false, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', classList: { toggle() {} }, setAttribute() {}, querySelector() { return { textContent: '' }; }, appendChild() {}, addEventListener() {}, getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; } });
const context = { console: { ...console, warn() {} }, assert, window: { dispatched: [], location: { search: '' }, dispatchEvent(event) { this.dispatched.push(event?.detail?.state); }, parent: { postMessage() {} } }, document: { getElementById() { return element(); }, createElement() { return element(); } }, URLSearchParams, CustomEvent: function CustomEvent(type, init) { return { type, detail: init?.detail || {} }; }, requestAnimationFrame() { return 0; }, setTimeout() { return 1; }, clearTimeout() {} };
vm.createContext(context);
vm.runInContext(source + `
renderSceneSummary = () => updateViewerStatus();
setInitialPosePreviewUi = () => {};
refreshInitialPoseActionState = () => {};
refreshWarnings = () => {};
appendRuntimeWarning = () => {};
failIfCanonicalRequiredVisualSetInvalid = () => false;
let captured = [];
loadRobotPreview = (preview, rendererContext) => {
  const root = { name: preview.rootName || 'old_root', children: [], geometry: { dispose() {} }, material: { dispose() {} } };
  captured.push({ preview, rendererContext, root });
  return { root, links: new Map([['old_link', {}]]), joints: new Map([['old_joint', {}]]), diagnostics: { robot_preview_lifecycle_state: 'loading', robotPreviewLifecycleState: 'loading', scene: sceneId() }, ready: Promise.resolve(root) };
};
function makeSceneRecorder(id) { return { id, added: [], removed: [], add(root) { this.added.push(root); }, remove(root) { this.removed.push(root); } }; }
function beginScene(id, sceneExists = true) {
  state.sceneJson = { scene: { id }, robot_preview: { urdf_url: id + '.urdf', rootName: id + '_root' } };
  state.three.scene = sceneExists ? makeSceneRecorder(id) : null;
  state.web3dReadiness = { state: 'scene_loading', emittedSceneReady: false, required: { robot_arm: true, attached_tool_gripper: true, workbench_support_surface: true, configured_camera: true }, pending: new Set(['robot_arm:expanded_urdf_loader', 'attached_tool_gripper:expanded_urdf_loader']), failed: false, failure: null };
}
function dirtyOldState() {
  state.robotPreviewResult = { stale: true };
  state.robotUrdfPreviewDiagnostics = { stale: true };
  state.robotAssemblyDiagnostics = [{ stale: true }];
  state.robotAssemblyRenderDiagnostics = { stale: true };
  state.resolvedFramePoses.set('old_frame', {});
  state.frameLookup.set('old_link', {});
  state.assemblyRoots = [{ name: 'previous_assembly_root' }];
  state.objects = [{ object3d: { name: 'previous_flattened_fallback' }, item: { id: 'old' } }];
}
function fireOldCallbacks(capture) {
  capture.rendererContext.scene.add(capture.root);
  capture.rendererContext.assemblyRoots.push(capture.root);
  capture.rendererContext.onRobotLoaded({ root: capture.root, diagnostics: { robot_preview_lifecycle_state: 'loaded', robotPreviewLifecycleState: 'loaded' } });
  capture.rendererContext.onRobotMeshLoaded(capture.root);
  capture.rendererContext.onRobotMeshLoadError(new Error('old mesh failed'), 'old.dae', { uri: 'old.dae' });
  capture.rendererContext.onRobotError(new Error('old robot failed'), { robot_urdf_url: 'old.urdf' });
}
function assertOldCallbacksRejected(targetSceneId) {
  assert.strictEqual(state.robotPreviewResult, null);
  assert.deepStrictEqual(state.robotUrdfPreviewDiagnostics, {});
  assert.deepStrictEqual(state.robotAssemblyDiagnostics, []);
  assert.deepStrictEqual(state.robotAssemblyRenderDiagnostics, {});
  assert.deepStrictEqual(Array.from(state.resolvedFramePoses.keys()), []);
  assert.deepStrictEqual(Array.from(state.frameLookup.keys()), []);
  assert.deepStrictEqual(state.assemblyRoots, []);
  assert.deepStrictEqual(state.objects, []);
  assert.strictEqual(state.web3dReadiness.state, 'scene_loading');
  assert.strictEqual(state.web3dReadiness.failed, false);
  assert.strictEqual(state.web3dReadiness.terminal, false);
  assert.strictEqual(state.sceneJson.scene.id, targetSceneId);
  assert.ok(!window.dispatched.includes('scene_ready'));
  assert.ok(!window.dispatched.includes('scene_failed'));
}
function runSwitchScenario(targetSceneId) {
  window.dispatched.length = 0; captured = [];
  beginScene('ur5_2f_test', true);
  loadExpandedUrdfRobotPreview(state.sceneJson.robot_preview);
  const old = captured[0];
  beginScene(targetSceneId, true);
  dirtyOldState();
  clearSceneObjects();
  fireOldCallbacks(old);
  assert.strictEqual(state.three.scene.id, targetSceneId);
  assert.deepStrictEqual(state.three.scene.added, []);
  assertOldCallbacksRejected(targetSceneId);
}
runSwitchScenario('suction_test');
runSwitchScenario('ur5_3f_test');
window.dispatched.length = 0; captured = [];
beginScene('ur5_2f_test', true);
loadExpandedUrdfRobotPreview(state.sceneJson.robot_preview);
const oldWithoutScene = captured[0];
beginScene('suction_test', false);
dirtyOldState();
clearSceneObjects();
fireOldCallbacks(oldWithoutScene);
assert.strictEqual(state.three.scene, null);
assertOldCallbacksRejected('suction_test');
`, context);
'''
    result = subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert result.stderr == ""

def test_urdf_renderer_rejects_unsafe_package_mesh_sources():
    js = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    package_body = js.split("function resolvePackageMeshUri", 1)[1].split("function normalizeMeshUri", 1)[0]
    for token in [
        "if (!scene)",
        "malformed scene ID",
        "safeSceneAssetId(scene)",
        "safePackageAssetId(packageName)",
        "if (!parts.length)",
        "if (!part)",
        "decoded === '.'",
        "decoded === '..'",
        "decoded.includes('\\0')",
        "decoded.includes('/')",
        "decoded.includes('\\\\')",
        "decodedAgain !== decoded",
        "/^(?:file|https?):\\/\\//i.test(packagePath)",
        "/^[A-Za-z]:[\\\\/]/.test(packagePath)",
        "/^[A-Za-z][A-Za-z0-9+.-]*:/.test(decoded)",
    ]:
        assert token in package_body
    for rejected_uri in [
        "package://pkg/../secret",
        "package://pkg/%2e%2e/secret",
        "package://pkg/%2Fetc/passwd",
        "package://pkg/C:/secret",
        "file:///tmp/mesh.dae",
        "http://example.com/mesh.dae",
    ]:
        assert rejected_uri not in js


def test_urdf_renderer_passes_loaded_scene_id_without_changing_staged_meshes():
    renderer = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    viewer = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    normalize_body = renderer.split("function normalizeMeshUri", 1)[1].split("function meshExtension", 1)[0]
    assert "return resolvePackageMeshUri(raw, context?.sceneId, diagnostics);" in normalize_body
    assert "const diagnostic = context?.meshUriDiagnostic?.({ mesh_uri: raw, mesh_staging_status: 'staged' });" in normalize_body
    assert "return diagnostic?.uri || raw;" in normalize_body
    preview_call = viewer.split("const previewResult = loadRobotPreview(preview, {", 1)[1].split("onRobotLoaded", 1)[0]
    assert "sceneId: loadSceneId," in preview_call



def test_load_robot_preview_resolves_urdf_package_mesh_requests_through_runtime_loader_path(tmp_path):
    script = tmp_path / "capture_robot_preview_mesh_requests.mjs"
    script.write_text(
        f"""
import assert from 'node:assert/strict';
import * as THREE from {str((VIEWER / 'node_modules/three/build/three.module.js')).__repr__()};
import URDFLoader from {str((VIEWER / 'node_modules/urdf-loader/src/URDFLoader.js')).__repr__()};
import {{ ColladaLoader }} from {str((VIEWER / 'node_modules/three/examples/jsm/loaders/ColladaLoader.js')).__repr__()};
import {{ STLLoader }} from {str((VIEWER / 'node_modules/three/examples/jsm/loaders/STLLoader.js')).__repr__()};

globalThis.window = {{}};

const requestedUrls = [];
globalThis.fetch = async () => ({{ ok: true, status: 200, statusText: 'OK', text: async () => '<robot name="test"/>' }});
const originalUrdfParse = URDFLoader.prototype.parse;
const originalColladaLoad = ColladaLoader.prototype.load;
const originalStlLoad = STLLoader.prototype.load;

URDFLoader.prototype.parse = function parseStub() {{
  assert.equal(this.constructor, URDFLoader);
  assert.equal(this.parseVisual, true);
  assert.equal(this.parseCollision, false);
  this.loadMeshCb(
    'package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae',
    this.manager,
    new THREE.MeshPhongMaterial(),
    (mesh, error) => {{ if (error) throw error; }}
  );
  const robot = new THREE.Group();
  robot.links = {{ base_link: new THREE.Group() }};
  robot.joints = {{}};
  robot.setJointValues = () => {{}};
  return robot;
}};

ColladaLoader.prototype.load = function loadStub(url, onLoad, onProgress, onError) {{
  requestedUrls.push(url);
  this.manager?.itemStart?.(url);
  onLoad({{ scene: new THREE.Group(), asset: {{}} }});
  this.manager?.itemEnd?.(url);
}};

STLLoader.prototype.load = function loadStub(url, onLoad, onProgress, onError) {{
  requestedUrls.push(url);
  this.manager?.itemStart?.(url);
  onLoad({{}});
  this.manager?.itemEnd?.(url);
}};

try {{
  const {{ loadRobotPreview }} = await import({str((VIEWER / 'urdf_robot_renderer.js')).__repr__()});
  const result = loadRobotPreview(
    {{ urdf_url: 'build/workcell_studio_web_scene/assets/ur5_2f_test/robot.urdf' }},
    {{ sceneId: 'ur5_2f_test' }}
  );
  await result.ready;
  assert.deepEqual(requestedUrls, [
    '/build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq_85_description/meshes/visual/robotiq_85_base_link.dae',
  ]);
  for (const url of requestedUrls) {{
    for (const forbidden of [
      '/robotiq_85_description/',
      '/ur_description/',
      '/single_suction_description/',
      '/robotiq_3f_gripper_description/',
    ]) {{
      assert.equal(url.startsWith(forbidden), false, `${{url}} unexpectedly starts with ${{forbidden}}`);
    }}
  }}
}} finally {{
  URDFLoader.prototype.parse = originalUrdfParse;
  ColladaLoader.prototype.load = originalColladaLoad;
  STLLoader.prototype.load = originalStlLoad;
}}
""",
        encoding="utf-8",
    )

    subprocess.run(
        ["node", str(script)],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

def test_viewer_applies_mesh_local_transform_only_below_object_roots():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "meshLocalTransformOf",
        "const source = item?.mesh_local_transform",
        "visualOriginOf",
        "meshLocalVector(transform.xyz || transform.origin || transform.position",
        "meshLocalVector(transform.rpy || transform.rotation_rpy",
        "meshLocalVector(scaleSource",
        "Number.isFinite(number)",
        "applyMeshLocalTransform",
        "applyLoadedMeshScaleHandling",
        "invalid_mesh_local_transform",
        "applied_mesh_local_transform",
    ]:
        assert token in js
    make_visual_body = js.split("function makeMeshVisualRoot", 1)[1].split("function applyMeshLocalTransform", 1)[0]
    assert make_visual_body.index("applyMeshLocalTransform(visualRoot, item)") < make_visual_body.index("applyLoadedMeshScaleHandling(meshObject, item)")
    assert make_visual_body.index("applyLoadedMeshScaleHandling(meshObject, item)") < make_visual_body.index("visualRoot.add(meshObject)")
    try_load_body = js.split("async function tryLoadMesh", 1)[1].split("function collectItems", 1)[0]
    assert try_load_body.index("materializeLoadedMesh(item, uri, loaded)") < try_load_body.index("makeMeshVisualRoot(item, meshObject)")
    assert try_load_body.index("makeMeshVisualRoot(item, meshObject)") < try_load_body.index("rendered.object3d.add(visualRoot)")
    apply_pose_body = js.split("function applyPose", 1)[1].split("function assignItemUserData", 1)[0]
    assert "mesh_local_transform" not in apply_pose_body
    assert "visual_local_transform" not in apply_pose_body
    assert "visual_origin" not in apply_pose_body


def test_viewer_generated_urdf_baked_pose_mode_prefers_baked_visual_pose_chain():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")

    assert "workcell_web_render_pose_mode === 'baked_visible_world_pose'" in js
    assert "function bakedVisibleWorldPoseSource(item)" in js
    assert "function usesBakedVisibleWorldPose(item)" in js

    baked_source_body = js.split("function bakedVisibleWorldPoseSource(item)", 1)[1].split("function usesBakedVisibleWorldPose(item)", 1)[0]
    assert "item?.baked_world_visual_pose || item?.expected_visual_pose || item?.final_transform || item?.world_from_visual || null" in baked_source_body

    mode_check_body = js.split("function usesBakedVisibleWorldPose(item)", 1)[1].split("function effectiveWorkcellWebRenderPoseMode(item)", 1)[0]
    assert "item?.workcell_web_render_pose_mode === 'baked_visible_world_pose'" in mode_check_body
    assert "hasFinitePoseBlock(bakedVisibleWorldPoseSource(item))" in mode_check_body
    assert "return isGeneratedUrdfMeshVisualItem(item);" in mode_check_body

    final_pose_body = js.split("function canonicalFinalPose(item)", 1)[1].split("function poseOf(item)", 1)[0]
    assert "if (isGeneratedUrdfItem(item))" in final_pose_body
    assert "if (usesBakedVisibleWorldPose(item))" in final_pose_body
    assert "return item.baked_world_visual_pose || item.expected_visual_pose || item.final_transform" in final_pose_body


def test_viewer_generated_urdf_unflagged_roots_still_use_frame_or_link_world_pose():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")

    frame_source_body = js.split("function generatedUrdfFramePoseSource(item)", 1)[1].split("function framePoseOf(item)", 1)[0]
    assert "if (item?.frame_world_pose) return item.frame_world_pose;" in frame_source_body
    assert "if (item?.link_world_pose) return item.link_world_pose;" in frame_source_body

    final_pose_body = js.split("function canonicalFinalPose(item)", 1)[1].split("function poseOf(item)", 1)[0]
    assert final_pose_body.index("if (usesBakedVisibleWorldPose(item))") < final_pose_body.index("return generatedUrdfFramePoseSource(item);")


def test_viewer_generated_urdf_baked_pose_mode_uses_identity_mesh_local_transform():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")

    mesh_transform_body = js.split("function applyMeshLocalTransform", 1)[1].split("async function tryLoadMesh", 1)[0]
    assert "const generatedUrdf = isGeneratedUrdfItem(item);" in mesh_transform_body
    assert "usesBakedVisibleWorldPose(item)" in mesh_transform_body
    assert "poseBlockOf({ xyz: [0, 0, 0], rpy: [0, 0, 0] })" in mesh_transform_body
    assert "visualOriginOf(item)" in mesh_transform_body
    assert mesh_transform_body.index("usesBakedVisibleWorldPose(item)") < mesh_transform_body.index("visualOriginOf(item)")
    assert "meshObject.position.copy(visualOrigin.xyz)" in mesh_transform_body
    assert "applyRosRpy(meshObject, visualOrigin.rpy)" in mesh_transform_body


def test_viewer_generated_urdf_unflagged_mesh_wrapper_still_applies_visual_origin():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")

    assert "function visualOriginOf(item)" in js
    visual_origin_body = js.split("function visualOriginOf(item)", 1)[1].split("function canonicalFinalPose(item)", 1)[0]
    assert "item?.visual_origin || item?.visual_local_transform" in visual_origin_body

    mesh_transform_body = js.split("function applyMeshLocalTransform", 1)[1].split("async function tryLoadMesh", 1)[0]
    assert "? (usesBakedVisibleWorldPose(item) ? poseBlockOf({ xyz: [0, 0, 0], rpy: [0, 0, 0] }) : visualOriginOf(item))" in mesh_transform_body
    assert "if (generatedUrdf) meshObject.scale.set(1, 1, 1);" in mesh_transform_body


def test_viewer_declares_explicit_ros_z_up_convention():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    init_body = js.split("function initThree()", 1)[1].split("function animate()", 1)[0]

    assert "const ROS_Z_UP = new THREE.Vector3(0, 0, 1);" in init_body
    assert "THREE.Object3D.DEFAULT_UP.copy(ROS_Z_UP);" in init_body
    assert "scene.up.copy(ROS_Z_UP);" in init_body
    assert "camera.up.copy(ROS_Z_UP);" in init_body


def test_viewer_ground_grid_is_ros_xy_not_threejs_default_xz_y_up():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    init_body = js.split("function initThree()", 1)[1].split("function animate()", 1)[0]

    assert "new THREE.GridHelper" in init_body
    assert "grid.name = 'ros_xy_ground_grid';" in init_body
    assert "grid.up.copy(ROS_Z_UP);" in init_body
    assert "grid.rotation.x = Math.PI / 2;" in init_body



def test_viewer_product_workspace_uses_central_light_palette_for_scene_and_css():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    css = (VIEWER / "style.css").read_text(encoding="utf-8")
    init_body = js.split("function initThree()", 1)[1].split("function animate()", 1)[0]

    assert "const PRODUCT_VIEW_LIGHT_PALETTE = Object.freeze({" in js
    for token in [
        "workspaceBackground: 0xeef1f4",
        "rendererClearColor: 0xeef1f4",
        "gridMajor: 0x7f8b98",
        "gridMinor: 0xb6c0ca",
        "ambientSky: 0xffffff",
        "ambientGround: 0xcbd3dc",
        "keyLight: 0xffffff",
        "fillLight: 0xdcefff",
        "labelText: 0x123040",
        "labelSurface: 0xf8fafc",
        "overlaySurface: 0xfff6dd",
        "errorSurface: 0xffeef0",
        "errorAccent: 0xc43434",
    ]:
        assert token in js
    assert "scene.background = new THREE.Color(PRODUCT_VIEW_LIGHT_PALETTE.workspaceBackground);" in init_body
    assert "renderer.setClearColor(PRODUCT_VIEW_LIGHT_PALETTE.rendererClearColor, 1);" in init_body
    assert "scene.background = new THREE.Color(0xe9edf1);" not in js
    assert "setClearColor(0xe9edf1" not in js
    assert "new THREE.Fog(0xe9edf1" not in js
    assert "scene.background = new THREE.Color(0x0b1018);" not in js
    assert "setClearColor(0x0b1018" not in js
    assert "new THREE.Fog(0x0b1018" not in js
    assert "Fog" not in js and ".fog" not in js

    assert "--workspace-bg: #eef1f4;" in css
    assert "body {" in css and "background: var(--bg);" in css
    assert ".app-shell" in css and "background: var(--workspace-bg);" in css
    assert ".viewport-panel" in css and "background: var(--workspace-bg);" in css
    assert "#scene-canvas" in css and "background: var(--workspace-bg);" in css
    assert "color: #123040;" in css
    assert "--panel" in css
    assert "background: var(--error-surface);" in css
    assert "background: rgba(139, 30, 45, 0.94);" not in css


def test_viewer_product_grid_uses_visible_major_minor_palette_colours():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    init_body = js.split("function initThree()", 1)[1].split("function animate()", 1)[0]

    assert "workspaceBackground: 0xeef1f4" in js
    assert "gridMajor: 0x7f8b98" in js
    assert "gridMinor: 0xb6c0ca" in js
    assert "new THREE.GridHelper(5, 20, PRODUCT_VIEW_LIGHT_PALETTE.gridMajor, PRODUCT_VIEW_LIGHT_PALETTE.gridMinor)" in init_body
    assert "new THREE.GridHelper(5, 20, 0x3a4a5e, 0x263445)" not in js
    assert "grid.name = 'ros_xy_ground_grid';" in init_body
    assert "material.opacity = 0.34;" in init_body


def test_viewer_product_visual_baseline_preserves_materials_and_selection_detail():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    assert "function materialHasUsableAppearance(material)" in js
    assert "applyNeutralFallbackToUnmaterialedMeshes(object, item);" in js
    assert "if (child?.isMesh && !materialHasUsableAppearance(child.material)) child.material = materialFor(item);" in js
    assert "child.material?.emissive" not in js
    assert "function refreshSelectionHighlight(rendered)" in js
    assert "helper.userData.selection_outline = true;" in js
    assert "function removeSelectionHighlight()" in js


def test_viewer_product_lights_are_named_bounded_and_reload_safe():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    assert "function installProductViewLights(scene)" in js
    assert "if (child?.userData?.product_view_light === true) scene.remove(child);" in js
    assert "product_view_balanced_ambient_light" in js
    assert "product_view_key_light" in js
    assert "product_view_fill_light" in js
    assert "key.shadow.mapSize.set(1024, 1024);" in js
    assert "Object.assign(key.shadow.camera, { near: 0.1, far: 12, left: -4, right: 4, top: 4, bottom: -4 });" in js


def test_viewer_product_theme_keeps_grid_controls_and_bounds_exclusions():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")

    assert "if (el.resetView) el.resetView.addEventListener('click', resetView);" in js
    assert "object.isGridHelper || object.isAxesHelper" in js
    assert "object.userData?.selection_outline === true" in js
    assert "camera.position.set(2.4, -2.8, 1.8);" in js
    assert "function fitSelection()" in js
    assert "fitSelection: () => { fitSelection(); return editorState(); }" in js


def test_viewer_product_view_camera_presets_and_grid_are_workcell_focused():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    html = (VIEWER / "index.html").read_text(encoding="utf-8")

    assert 'id="camera-preset"' in html
    for preset in ["isometric", "front", "top", "robot"]:
        assert f'value="{preset}"' in html
    assert "const CAMERA_PRESET_DIRECTIONS = Object.freeze" in js
    for token in ["isometric", "front", "top", "robot"]:
        assert token in js
    assert "function applyCameraClipping(camera, radius, distance)" in js
    assert "function applyCameraPreset(preset)" in js
    assert "applyCameraPreset: preset =>" in js
    assert "el.cameraPreset.addEventListener('change'" in js
    assert "material.opacity = 0.34" in js
    assert "grid.userData.exclude_from_fit_bounds = true" in js
    assert "grid.userData.exclude_from_physical_bounds = true" in js
    assert "if (el.cameraPreset) el.cameraPreset.disabled = false" in js


def test_viewer_product_theme_is_scene_independent():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    init_body = js.split("function initThree()", 1)[1].split("function animate()", 1)[0]

    for scene_name in ["ur5_2f_test", "ur5_3f_test", "ur3_suction_test", "ur10_2f_test", "suction_test"]:
        assert scene_name not in init_body
    assert "sceneId()" not in init_body


def test_viewer_initial_fit_waits_for_terminal_scene_ready_once():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")

    readiness_body = js.split("function emitWeb3dReadinessState", 1)[1].split("function readinessCategoryForItem", 1)[0]
    assert "readinessState === 'scene_ready' || readinessState === 'scene_failed'" in readiness_body
    assert "if (state.web3dReadiness.terminal)" in readiness_body
    assert "terminalEmissionCount" in readiness_body
    assert "if (readinessState === 'scene_ready') triggerInitialCameraFitAfterSceneReady();" in readiness_body

    scene_ready_body = js.split("function maybeEmitSceneReady()", 1)[1].split("const el = {", 1)[0]
    assert "if (readiness.pending?.size === 0) emitWeb3dReadinessState('scene_ready'" in scene_ready_body

    trigger_body = js.split("function triggerInitialCameraFitAfterSceneReady()", 1)[1].split("function scheduleInitialCameraFitRetry", 1)[0]
    assert "return attemptInitialCameraFit({ allowRetry: false });" in trigger_body

    fit_body = js.split("function attemptInitialCameraFit", 1)[1].split("function triggerInitialCameraFitAfterSceneReady", 1)[0]
    assert "if (!fit || fit.done || fit.userControlled || fit.sceneKey !== stableSceneCameraKey()) return false;" in fit_body
    assert "fit.done = true;" in fit_body


def test_viewer_initial_fit_not_reframed_per_mesh_load_but_manual_fit_remains():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")

    try_load_body = js.split("async function tryLoadMesh", 1)[1].split("function collectItems", 1)[0]
    assert "loadAsync(loadUrl)" in try_load_body
    assert "frameScene(bounds)" not in try_load_body
    assert "attemptInitialCameraFit" not in try_load_body
    assert "scheduleInitialCameraFitRetry" not in try_load_body

    expanded_body = js.split("function loadExpandedUrdfRobotPreview", 1)[1].split("function renderFrameDebugOverlays", 1)[0]
    assert "onRobotLoaded" in expanded_body
    assert "onRobotMeshLoaded" in expanded_body
    assert "attemptInitialCameraFit" not in expanded_body
    assert "scheduleInitialCameraFitRetry" not in expanded_body

    render_scene_body = js.split("function renderScene(items)", 1)[1].split("function loadExpandedUrdfRobotPreview", 1)[0]
    assert "tryLoadMesh(item, rendered, fallback);" in render_scene_body
    assert "maybeEmitSceneReady();" in render_scene_body
    assert "attemptInitialCameraFit" not in render_scene_body

    reset_body = js.split("function resetView", 1)[1].split("function reportFitSelectionFallback", 1)[0]
    assert "if (userInitiated) markCameraUserControlled();" in reset_body
    assert "const physical = collectPhysicalVisibleBounds(state.three.scene);" in reset_body
    assert "frameScene(physical.bounds, { preset })" in reset_body
    assert "el.resetView.addEventListener('click', resetView)" in js
    assert "fitScene: () => { resetView(); return editorState(); }" in js


def test_viewer_initial_fit_bounds_use_visible_physical_geometry_only():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")

    assert "function isInitialFitPhysicalGeometryItem(item, identity = '')" in js
    physical_body = js.split("function isInitialFitPhysicalGeometryItem", 1)[1].split("function isPhysicalBoundsHelperObject", 1)[0]
    for token in ["'robot'", "'tool'", "'table'", "'camera'", "'object'", "conveyor", "bin", "workbench", "configured camera"]:
        assert token in physical_body

    helper_body = js.split("function isPhysicalBoundsHelperObject", 1)[1].split("function isRobotPrimitiveFallbackObject", 1)[0]
    for token in [
        "object.isGridHelper || object.isAxesHelper",
        "selection_outline",
        "exclude_from_fit_bounds",
        "DEBUG_OVERLAY_TOKEN_RE.test(identity)",
        "return !isInitialFitPhysicalGeometryItem(item, identity);",
    ]:
        assert token in helper_body
    for token in ["robot reach", "camera fov", "pick zone", "place zone", "warning badge", "helper", "diagnostic"]:
        assert token in js

def test_repository_does_not_track_generated_web_scene_outputs_under_source_paths():
    result = subprocess.run(
        ["git", "ls-files", "*.web_scene.json"],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    tracked = [line for line in result.stdout.splitlines() if line.strip()]
    assert tracked == []


def test_viewer_validates_renderable_transforms_and_required_mesh_fallbacks():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "validateRenderableTransform",
        "invalid_renderable_transform",
        "final xyz contains non-finite values",
        "final rpy contains non-finite values",
        "scale contains non-finite values",
        "scale must be positive on every axis",
        "renderable skipped before applyPose because transform validation failed",
        "itemRequiresMeshBackedVisual",
        "required_mesh_failed_debug_fallback",
        "requires_mesh_backed_visual",
        "parent_link",
        "immediate_parent_link",
        "baked_world_visual_pose",
        "visual_origin",
        "final_pose",
        "final_scale",
        "mesh_scale",
        "fallback_or_skip_reason",
    ]:
        assert token in js
    assert "if (!applyPose(object3d, item)) continue;" in js
    assert "warnRequiredMeshFallback(item," in js


def test_viewer_mesh_preflight_surfaces_distinct_failure_reasons():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "unsupported_format",
        "unsupported mesh format",
        "preflightMeshUrl",
        "fetch(url, { method: 'HEAD' })",
        "method: 'GET'",
        "url_not_served",
        "mesh_url_not_served",
        "file_access_blocked",
        "mesh_file_access_blocked",
        "loader_failure",
        "mesh_loader_failure",
        "meshLoaderNameForExtension",
        "extension: ext",
        "loader: loaderName",
        "invalid_renderable_transform",
        "scale must be positive on every axis",
        "scale contains non-finite values",
    ]:
        assert token in js
    preflight_body = js.split("async function preflightMeshUrl", 1)[1].split("function itemType", 1)[0]
    assert "response.status" in preflight_body
    assert "!response.ok" in preflight_body
    try_load_body = js.split("async function tryLoadMesh", 1)[1].split("function collectItems", 1)[0]
    assert try_load_body.index("await preflightMeshUrl") < try_load_body.index("new STLLoader().loadAsync")
    assert "item.mesh_status = preflight.status || 'url_not_served'" in try_load_body
    assert "item.mesh_status = 'loader_failure'" in try_load_body


def test_viewer_local_mesh_transform_is_applied_to_mesh_object_not_parent_pose():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "mesh_local_transform",
        "visual_local_transform",
        "function applyMeshLocalTransform",
        "applyMeshLocalTransform(meshObject, item)",
    ]:
        assert token in js

    local_transform_body = js.split("function applyMeshLocalTransform", 1)[1].split("async function tryLoadMesh", 1)[0]
    assert "meshObject" in local_transform_body
    assert ".position" in local_transform_body
    assert "applyRosRpy(meshObject, visualOrigin.rpy)" in local_transform_body
    assert ".scale" in local_transform_body

    apply_pose_body = js.split("function applyPose", 1)[1].split("function assignItemUserData", 1)[0]
    assert "mesh_local_transform" not in apply_pose_body
    assert "visual_local_transform" not in apply_pose_body
    assert "visual_origin" not in apply_pose_body


def test_viewer_mesh_local_ros_rpy_order_regression():
    result = subprocess.run(
        [str(ROOT / "tests" / "test_web3d_mesh_ros_rpy_order.mjs")],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Three.js 160" in result.stdout


def test_primary_mesh_backed_target_bin_keeps_product_visibility_scale_color_and_readiness():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")

    grouping = js.split("function viewerGroupFor(item)", 1)[1].split("const DEBUG_OVERLAY_TOKEN_RE", 1)[0]
    assert grouping.index("if (isPrimaryAuthoredPhysicalMesh(item)) return 'environment/layout';") < grouping.index("return 'zones';")
    primary_physical = js.split("function isPrimaryAuthoredPhysicalMesh(item)", 1)[1].split("function readinessCategoryForItem", 1)[0]
    assert "contractCategory === 'object'" in primary_physical
    assert "target bin" in primary_physical

    overlay_filter = js.split("function isDebugOverlayItem(item)", 1)[1].split("function isSensor(item)", 1)[0]
    assert "if (viewerGroupFor(item) === 'zones') return true;" in overlay_filter
    assert "if (isOverlayPolicyItem(item)) return true;" in overlay_filter
    assert overlay_filter.index("if (isPrimaryAuthoredPhysicalMesh(item)) return false;") < overlay_filter.index("const identity")
    render_scene = js.split("function renderScene(items)", 1)[1].split("function loadExpandedUrdfRobotPreview", 1)[0]
    assert "object3d.visible = state.debugOverlaysVisible || !isDebugOverlayItem(item);" in render_scene

    root_scale = js.split("function scaleOf(item)", 1)[1].split("function transformOf", 1)[0]
    mesh_scale = js.split("function meshLocalTransformOf(item)", 1)[1].split("function cloneTransform", 1)[0]
    assert "item.scale || [1, 1, 1]" in root_scale
    assert "item.scale || item.mesh_scale" not in root_scale
    assert "transform.scale || item?.mesh_scale" in mesh_scale

    readiness = js.split("function beginWeb3dSceneReadiness(items)", 1)[1].split("function requiredReadinessCompleteForItem", 1)[0]
    assert "pending.add(readinessKey(category, item))" in readiness
    category = js.split("function readinessCategoryForItem(item)", 1)[1].split("function readinessKey", 1)[0]
    assert "return 'authored_physical_mesh';" in category
    load = js.split("async function tryLoadMesh", 1)[1].split("function collectItems", 1)[0]
    assert load.index("setRenderInfo(rendered, 'mesh_loaded'") < load.rindex("requiredReadinessCompleteForItem(item)")
    assert "failWeb3dSceneReadiness(item, loadUrl, `loaded mesh bounds validation failed" in load

    material = js.split("function materialFor(item)", 1)[1].split("function materialHasUsableAppearance", 1)[0]
    assert "item?.material?.color" in material
    assert "material.color.setRGB" in material


def test_primary_target_bin_place_zone_identity_remains_physical_until_staged_mesh_loads():
    js_path = VIEWER / "viewer.js"
    harness = r"""
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
const element = () => ({ hidden: false, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', classList: { toggle() {} }, setAttribute() {}, querySelector() { return { textContent: '' }; }, appendChild() {}, addEventListener() {}, getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; } });
const context = { console, assert, window: { location: { search: '' }, dispatchEvent() {}, parent: { postMessage() {} } }, document: { getElementById() { return element(); }, createElement() { return element(); } }, URLSearchParams, CustomEvent: function CustomEvent(type, init) { return { type, detail: init?.detail || {} }; }, requestAnimationFrame() { return 0; }, setTimeout() { return 1; }, clearTimeout() {} };
vm.createContext(context);
vm.runInContext(source + `
const targetBin = {
  id: 'target_bin_default',
  display_name: 'Target bin',
  category: 'place_zone',
  render_policy: 'primary',
  mesh_contract_category: 'object',
  mesh_load_required: true,
  mesh_uri: 'build/workcell_studio_web_scene/assets/ur5_2f_test/target_bin.stl'
};
const placeZone = { id: 'place_zone_default', category: 'place_zone', render_policy: 'overlay' };
assert.strictEqual(isPrimaryAuthoredPhysicalMesh(targetBin), true);
assert.strictEqual(isDebugOverlayItem(targetBin), false);
assert.strictEqual(viewerGroupFor(targetBin), 'environment/layout');
assert.strictEqual(readinessCategoryForItem(targetBin), 'authored_physical_mesh');
assert.strictEqual(isPrimaryAuthoredPhysicalMesh(placeZone), false);
assert.strictEqual(isDebugOverlayItem(placeZone), true);
assert.strictEqual(viewerGroupFor(placeZone), 'zones');
assert.strictEqual(false || !isDebugOverlayItem(targetBin), true);
assert.strictEqual(false || !isDebugOverlayItem(placeZone), false);
state.sceneJson = { assets: [targetBin, placeZone] };
beginWeb3dSceneReadiness(collectItems(state.sceneJson));
const readinessKeyForTarget = readinessKey('authored_physical_mesh', targetBin);
assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), true);
assert.strictEqual(state.web3dReadiness.required.authored_physical_mesh, true);
targetBin.mesh_status = 'loaded';
requiredReadinessCompleteForItem(targetBin);
assert.strictEqual(state.web3dReadiness.pending.has(readinessKeyForTarget), false);
`, context);
"""
    subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def test_viewer_visual_bounds_diagnostics_and_fit_bounds_contract_are_source_guarded():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "loaded_mesh_oversized",
        "loaded_mesh_bounds_invalid",
        "required_mesh_failed_debug_fallback",
        "mesh_unit_correction",
        "auto_detected_mm_to_m",
        "auto_detected_cm_to_m",
    ]:
        assert token in js

    fit_bounds_body = js.split("function computeFitBounds", 1)[1].split("function computeRenderedBounds", 1)[0]
    assert "required_mesh_failed_debug_fallback" in fit_bounds_body
    assert "includeDebugFallbacks" in fit_bounds_body
    assert "continue" in fit_bounds_body
    assert "excluded from normal camera fit bounds" in fit_bounds_body

    unit_autoscale_body = js.split("function maybeApplyMeshUnitAutoscale", 1)[1].split("function isCoreMeshContractItem", 1)[0]
    assert "mesh_unit_correction" in unit_autoscale_body
    assert "auto_detected_mm_to_m" in unit_autoscale_body
    assert "auto_detected_cm_to_m" in unit_autoscale_body



def test_viewer_does_not_apply_expected_dimensions_as_per_axis_render_scale():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")

    assert "maybeApplyExpectedDimensionScale" not in js

    render_scale_bodies = [
        js.split("function scaleOf(item)", 1)[1].split("function transformOf", 1)[0],
        js.split("function meshLocalTransformOf(item)", 1)[1].split("function cloneTransform", 1)[0],
        js.split("function applyMeshLocalTransform", 1)[1].split("async function tryLoadMesh", 1)[0],
        js.split("function applyLoadedMeshScaleHandling", 1)[1].split("async function tryLoadMesh", 1)[0],
    ]
    for body in render_scale_bodies:
        assert "expected_dimensions_m" not in body
        assert "expected_dimensions" not in body
        assert "dimensions_m" not in body

    for forbidden in [
        ".scale.set(expected",
        ".scale.set(dimensions",
        ".scale.set(dims",
        "scale.x = expected",
        "scale.y = expected",
        "scale.z = expected",
        "meshObject.scale.set(axisRatios",
        "meshObject.scale.set(scale.x, scale.y, scale.z)",
    ]:
        assert forbidden not in js


def test_viewer_expected_dimensions_are_limited_to_diagnostics_and_uniform_unit_correction():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")

    assert "function expectedDimensionsOf(item)" in js
    assert "function maybeApplyMeshUnitAutoscale" in js
    unit_autoscale_body = js.split("function maybeApplyMeshUnitAutoscale", 1)[1].split("function isCoreMeshContractItem", 1)[0]
    assert "expected_dimensions_m" in unit_autoscale_body
    assert "axisRatios" in unit_autoscale_body
    assert "uniformRatio" in unit_autoscale_body
    assert "[1000, 100].find" in unit_autoscale_body
    assert "uniformRatio <= 1.25" in unit_autoscale_body
    assert "meshObject.scale.multiplyScalar(scale);" in unit_autoscale_body
    assert "meshObject.scale.set" not in unit_autoscale_body
    assert "targetRatio === 1000 ? 0.001 : 0.01" in unit_autoscale_body
    assert "rejected_non_uniform_or_unclear_ratio" in unit_autoscale_body

    expected_dimension_references = [line for line in js.splitlines() if "expected_dimensions_m" in line]
    assert expected_dimension_references
    allowed_context_tokens = (
        "expectedDimensionsOf",
        "item?.expected_dimensions_m",
        "meshUnitAutoscaleAllowed",
        "maybeApplyMeshUnitAutoscale",
        "viewer_expected_dimensions_m",
        "appendRuntimeWarning",
        "warnLoadedMeshBounds",
        "loaded_mesh_oversized",
        "mesh_unit_autoscale",
        "expected_dimensions_m:",
        "to expected_dimensions_m",
    )
    for line in expected_dimension_references:
        assert any(token in line for token in allowed_context_tokens), line


def test_viewer_unit_autoscale_is_uniform_only_for_clear_100_or_1000_ratios():
    js_path = VIEWER / "viewer.js"
    harness = r'''
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
class MockVector3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
}
class MockBox3 {
  constructor(size) {
    this.min = { x: 0, y: 0, z: 0 };
    this.max = { x: size?.x || 0, y: size?.y || 0, z: size?.z || 0 };
  }
  isEmpty() { return false; }
  getSize(target) {
    target.x = this.max.x - this.min.x;
    target.y = this.max.y - this.min.y;
    target.z = this.max.z - this.min.z;
    return target;
  }
  getCenter(target) {
    target.x = (this.min.x + this.max.x) / 2;
    target.y = (this.min.y + this.max.y) / 2;
    target.z = (this.min.z + this.max.z) / 2;
    return target;
  }
  setFromObject(object) { return new MockBox3(object.mockCorrectedSize || object.mockSize || { x: 1, y: 1, z: 1 }); }
}
const element = () => ({
  hidden: false,
  checked: false,
  disabled: false,
  textContent: '',
  className: '',
  innerHTML: '',
  classList: { toggle() {} },
  setAttribute() {},
  querySelector() { return { textContent: '' }; },
  appendChild() {},
  addEventListener() {},
  getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; },
});
const sandbox = {
  console,
  assert,
  MockVector3,
  MockBox3,
  window: { location: { search: '' } },
  document: { getElementById() { return element(); }, createElement() { return element(); } },
  URLSearchParams,
  requestAnimationFrame() { return 0; },
};
vm.createContext(sandbox);
vm.runInContext(source + `
THREE = { Vector3: MockVector3, Box3: MockBox3 };
state.runtimeWarnings = [];
function meshObject(size) {
  return {
    mockSize: size,
    mockCorrectedSize: size,
    updateMatrixWorld() {},
    scale: {
      scalar: 1,
      setCalled: false,
      multiplyScalar(value) { this.scalar *= value; },
      set() { this.setCalled = true; throw new Error('per-axis scale must not be used for unit correction'); },
    },
  };
}
let mmItem = { id: 'metric_table', category: 'table', allow_mesh_unit_autoscale: true, expected_dimensions_m: [1, 0.5, 0.25] };
let mmMesh = meshObject({ x: 1000, y: 500, z: 250 });
assert.strictEqual(maybeApplyMeshUnitAutoscale(mmItem, mmMesh, new MockBox3(mmMesh.mockSize), 'table.stl'), true);
assert.strictEqual(mmMesh.scale.scalar, 0.001);
assert.strictEqual(mmMesh.scale.setCalled, false);
assert.strictEqual(mmItem.mesh_unit_correction.target_ratio, 1000);

let cmItem = { id: 'cm_table', category: 'table', allow_mesh_unit_autoscale: true, expected_dimensions_m: [1, 0.5, 0.25] };
let cmMesh = meshObject({ x: 100, y: 50, z: 25 });
assert.strictEqual(maybeApplyMeshUnitAutoscale(cmItem, cmMesh, new MockBox3(cmMesh.mockSize), 'table.stl'), true);
assert.strictEqual(cmMesh.scale.scalar, 0.01);
assert.strictEqual(cmMesh.scale.setCalled, false);
assert.strictEqual(cmItem.mesh_unit_correction.target_ratio, 100);

let fittedItem = { id: 'bad_fit_table', category: 'table', allow_mesh_unit_autoscale: true, expected_dimensions_m: [1, 0.5, 0.25] };
let fittedMesh = meshObject({ x: 4, y: 7, z: 11 });
assert.strictEqual(maybeApplyMeshUnitAutoscale(fittedItem, fittedMesh, new MockBox3(fittedMesh.mockSize), 'table.stl'), false);
assert.strictEqual(fittedMesh.scale.scalar, 1);
assert.strictEqual(fittedMesh.scale.setCalled, false);
assert.strictEqual(fittedItem.mesh_unit_correction.confidence, 'rejected_non_uniform_or_unclear_ratio');
assert.strictEqual(fittedItem.mesh_unit_correction.scale, 1.0);
assert.strictEqual(fittedItem.mesh_unit_correction.target_ratio, null);
`, sandbox);
'''
    result = subprocess.run(
        ["node", "-e", harness, str(js_path)],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert result.stderr == ""

def test_viewer_hides_camera_framing_exclusions_from_user_warning_panel_but_keeps_status_raw():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "function isUserFacingWarning(w)",
        "w.code === 'camera_framing_blocker_excluded'",
        "return false",
        "required_mesh_failed",
        "invalid_dimension",
        "visual_contract",
        "unsafe_path",
        "unsupported_format",
        "missing_file",
        "loader_failure",
        "load_error",
    ]:
        assert token in js

    refresh_body = js.split("function refreshWarnings", 1)[1].split("function populateWarnings", 1)[0]
    assert "const userFacingWarnings = warnings.filter(isUserFacingWarning);" in refresh_body
    assert "userFacingWarnings.map" in refresh_body
    assert "warnings.map" not in refresh_body

    status_body = js.split("function updateViewerStatus", 1)[1].split("function renderSceneSummary", 1)[0]
    assert "runtime_warnings: warnings" in status_body
    assert "runtimeWarnings: warnings" in status_body
    assert "filter(isUserFacingWarning)" not in status_body


def _viewer_function_body(js: str, signature: str, next_signature: str) -> str:
    assert signature in js
    assert next_signature in js
    return js.split(signature, 1)[1].split(next_signature, 1)[0]


def test_viewer_load_contract_keeps_selection_empty_until_manual_pick():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    prompt = "Select an object from the list or canvas."

    load_file_body = _viewer_function_body(js, "async function loadFile(file)", "function safeRelativeSceneUrl")
    load_url_body = _viewer_function_body(js, "async function loadSceneUrl(rawUrl)", "if (el.resetView)")
    render_scene_body = _viewer_function_body(js, "function renderScene(items)", "function createLabelElement")
    object_list_body = _viewer_function_body(js, "function populateObjectList()", "function selectObject(id)")
    append_object_list_body = _viewer_function_body(js, "function appendObjectListRow(rendered, group)", "function populateObjectList()")
    select_body = _viewer_function_body(js, "function selectObject(id)", "function pickObject(event)")
    pick_body = _viewer_function_body(js, "function pickObject(event)", "function attachTransformGizmo")

    for body in [load_file_body, load_url_body]:
        assert "state.selected = null;" in body
        assert "detachTransformGizmo();" in body
        assert "renderScene(" in body
        assert "el.inspector.className = 'state empty';" in body
        assert f"items.length ? '{prompt}' : EMPTY_SCENE_MESSAGE" in body
        assert "selectObject(" not in body

    assert "selectObject(" not in render_scene_body
    assert "state.selected =" not in render_scene_body
    assert "populateObjectList();" in render_scene_body

    assert "li.addEventListener('click', () => selectObject(rendered.item.id));" in append_object_list_body
    assert "const selected = rendered.item.id === id;" in select_body
    assert "rendered.item.locked" not in select_body
    assert "canEditItem(rendered.item)" not in select_body
    assert "rankedPickingCandidates(hits)" in pick_body
    assert "candidates.find(candidate => Number.isFinite(candidate.priority)" in pick_body
    ranking_body = _viewer_function_body(js, "function rankedPickingCandidates(hits)", "function selectObject(id)")
    assert "inspectionSelectionRendered" in ranking_body
    assert "canonicalEditOwnerRendered" not in ranking_body
    assert "inspectionSelectionRendered" in select_body
    assert "selectObject(selectedCandidate.rendered.item.id);" in pick_body
    assert "return selectedCandidate.rendered.item.id;" in pick_body


def test_raycast_registry_ancestry_distance_overlays_reload_and_empty_select():
    js_path = VIEWER / "viewer.js"
    harness = r"""
const fs = require('fs'); const vm = require('vm'); const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
const element = () => ({hidden:false,checked:false,disabled:false,textContent:'',className:'',innerHTML:'',classList:{toggle(){}},querySelector(){return null;},querySelectorAll(){return[];},addEventListener(){},setAttribute(){},appendChild(){},getBoundingClientRect(){return {left:0,top:0,width:100,height:100};}});
const context={console,assert,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}}},document:{getElementById(){return element();},querySelectorAll(){return[];},createElement(){return element();}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context);
vm.runInContext(source + `
updateLabels=()=>{}; populateInspector=()=>{}; attachTransformGizmo=()=>{}; detachTransformGizmo=()=>{}; refreshSelectionHighlight=()=>{}; removeSelectionHighlight=()=>{};
const node = (name='', parent=null) => ({name,parent,visible:true,userData:{},children:[],traverse(fn){fn(this); this.children.forEach(child=>child.traverse(fn));}});
const rendered = item => { const object3d=node(item.id); object3d.userData.item=item; return {item,object3d}; };
const bin=rendered({id:'target_bin_default',editable:true,locked:false,render_policy:'primary',mesh_load_required:true,mesh_contract_category:'object',category:'place_zone',source_layer:'editable_layout',transform_group:'default_drop_destination'});
const zone=rendered({id:'place_zone_default',editable:true,locked:false,render_policy:'overlay',role:'place_zone',source_layer:'editable_layout',transform_group:'default_drop_destination'});
const marker=rendered({id:'commissioning_object',editable:false,locked:true,render_policy:'primary',role:'task_marker',source_layer:'task_preview'});
state.sceneJson={scene:{id:'ur5_2f_test'}}; state.objects=[bin,zone,marker]; state.editorEvents=[]; state.debugOverlaysVisible=false;
const table=rendered({id:'support_surface_table',role:'table',render_policy:'primary'});
const camera=rendered({id:'realsense_overhead',role:'sensor',render_policy:'primary'});
const ur5={id:'ur5_generated',role:'robot',source_layer:'locked_generated_urdf_visual'};
const tool={id:'robotiq_generated',role:'tool',source_layer:'locked_generated_urdf_visual'};
state.objects.push(table,camera);
const colladaChild=node('nested-collada',table.object3d); table.object3d.children.push(colladaChild);
assert.strictEqual(itemFromRaycastHit({object:colladaChild}).item.id,'support_surface_table');
const urdfRoot=node('urdf-root'), urdfLink=node('shoulder_link',urdfRoot), urdfMesh=node('mesh',urdfLink); urdfRoot.children=[urdfLink]; urdfLink.children=[urdfMesh];
registerPickRecord(ur5,urdfLink,urdfRoot); assert.strictEqual(itemFromRaycastHit({object:urdfMesh}).item.id,'ur5_generated'); assert.strictEqual(state.objects.some(record=>record.item.id==='ur5_generated'),false); assert.strictEqual(selectionIsEditable(renderedById('ur5_generated')),false);
const toolLink=node('robotiq_link',urdfRoot), toolMesh=node('tool-mesh',toolLink); toolLink.children=[toolMesh]; urdfRoot.children.push(toolLink);
registerPickRecord(tool,toolLink,urdfRoot); assert.strictEqual(itemFromRaycastHit({object:toolMesh}).item.id,'robotiq_generated');
const hits=[{object:table.object3d,distance:1},{object:bin.object3d,distance:3}];
assert.strictEqual(rankedPickingCandidates(hits)[0].rendered.item.id,'support_surface_table');
state.three.pointer={}; state.three.camera={}; state.three.raycaster={setFromCamera(){},intersectObjects(){return hits;}};
assert.strictEqual(pickObject({clientX:10,clientY:10}),'support_surface_table');
for (const candidate of [table,camera,{object3d:urdfMesh},{object3d:toolMesh}]) {
  hits.splice(0,hits.length,{object:candidate.object3d,distance:1},{object:bin.object3d,distance:2});
  assert.notStrictEqual(pickObject({clientX:10,clientY:10}),'target_bin_default');
}
assert.strictEqual(isNormalSelectableRendered(marker),false);
hits.splice(0,hits.length,{object:zone.object3d,distance:1},{object:bin.object3d,distance:2});
state.debugOverlaysVisible=false; assert.strictEqual(pickObject({clientX:10,clientY:10}),'target_bin_default');
state.debugOverlaysVisible=true; assert.strictEqual(pickObject({clientX:10,clientY:10}),'place_zone_default'); assert.strictEqual(pickingPriority(marker),4);
const pickZone=rendered({id:'pick_zone_commissioning',role:'pick_zone',render_policy:'overlay'}); state.objects.push(pickZone);
for (const overlay of [pickZone,zone]) { hits.splice(0,hits.length,{object:overlay.object3d,distance:1},{object:bin.object3d,distance:2}); assert.notStrictEqual(pickObject({clientX:10,clientY:10}),'target_bin_default'); }
assert.strictEqual(isNormalSelectableRendered(renderedById('ur5_generated')),true); assert.strictEqual(pickingPriority(renderedById('ur5_generated')),3);
assert.strictEqual(editableTransformGroupMembers(bin).map(r=>r.item.id).sort().join(','),'place_zone_default,target_bin_default');
const staleIdentity=state.pickIdentityByObject; resetSceneLifecycleState(); assert.strictEqual(state.pickRecords.length,0); assert.notStrictEqual(state.pickIdentityByObject,staleIdentity); assert.strictEqual(renderedById('ur5_generated'),undefined);
state.editorMode='select'; state.selected='target_bin_default'; state.three.raycaster={setFromCamera(){},intersectObjects(){return[];}}; pickObject({clientX:10,clientY:10}); assert.strictEqual(state.selected,''); assert.strictEqual(state.lastCanvasPickReason,'empty_select_click');
`, context);
"""
    subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, capture_output=True, text=True)


def test_selection_rejects_late_helper_without_clearing_valid_physical_item():
    js_path = VIEWER / "viewer.js"
    harness = r"""
const fs = require('fs'); const vm = require('vm'); const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
const element = () => ({ hidden:false, checked:false, disabled:false, textContent:'', className:'', innerHTML:'', classList:{toggle(){}}, querySelector(){return null;}, querySelectorAll(){return[];}, addEventListener(){}, setAttribute(){}, appendChild(){} });
const context = { console, assert, window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}}}, document:{getElementById(){return element();},querySelectorAll(){return[];},createElement(){return element();}}, URLSearchParams, CustomEvent:function(){}, requestAnimationFrame(){}, setTimeout(){}, clearTimeout(){} };
vm.createContext(context);
vm.runInContext(source + `
updateLabels=()=>{}; populateInspector=()=>{}; attachTransformGizmo=()=>{}; detachTransformGizmo=()=>{}; refreshSelectionHighlight=()=>{}; removeSelectionHighlight=()=>{};
const rendered = item => ({ item, object3d:{} });
const target = rendered({id:'target_bin_default', editable:true, render_policy:'primary', mesh_load_required:true, mesh_contract_category:'object', category:'place_zone_bin', source_layer:'editable_layout'});
const robot = rendered({id:'ur5_generated', editable:false, locked:true, render_policy:'primary', role:'robot', source_layer:'locked_generated_urdf_visual'});
const helper = rendered({id:'debug_frame_axes_tool0', editable:false, render_policy:'primary', role:'helper', source_layer:'debug_overlay'});
state.sceneJson={scene:{id:'ur5_2f_test'}}; state.objects=[target,robot,helper]; state.editorEvents=[];
selectObject('target_bin_default');
assert.strictEqual(state.selected,'target_bin_default');
assert.strictEqual(state.editorEvents.filter(e=>e.type==='selection_changed'&&e.itemId==='target_bin_default').length,1);
selectObject('debug_frame_axes_tool0');
assert.strictEqual(state.selected,'target_bin_default');
assert.strictEqual(state.editorEvents.filter(e=>e.type==='selection_changed').length,1);
assert.strictEqual(state.editorEvents.at(-1).type,'selection_ignored');
selectObject('commissioning_object'); assert.strictEqual(state.selected,'target_bin_default');
selectObject('commissioning_object'); assert.strictEqual(state.selected,'target_bin_default');
assert.strictEqual(state.editorEvents.filter(e=>e.type==='selection_ignored'&&e.itemId==='commissioning_object').length,1);
selectObject('ur5_generated'); assert.strictEqual(state.selected,'ur5_generated'); assert.strictEqual(canEditItem(robot.item),false);
selectObject('target_bin_default'); assert.strictEqual(state.selected,'target_bin_default'); assert.strictEqual(isNormalSelectableRendered(target),true);
clearSelection(); assert.strictEqual(state.selected,''); assert.strictEqual(state.editorEvents.at(-1).itemId,'');
`, context);
"""
    subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, capture_output=True, text=True)


def test_selection_identity_is_separate_from_transform_edit_ownership():
    js_path = VIEWER / "viewer.js"
    harness = r"""
const fs = require('fs'); const vm = require('vm'); const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
const element = () => ({hidden:false,checked:false,disabled:false,textContent:'',className:'',innerHTML:'',classList:{toggle(){}},querySelector(){return null;},querySelectorAll(){return[];},addEventListener(){},setAttribute(){},appendChild(){},getBoundingClientRect(){return {left:0,top:0,width:100,height:100};}});
const context={console,assert,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}}},document:{getElementById(){return element();},querySelectorAll(){return[];},createElement(){return element();}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context);
vm.runInContext(source + `
updateLabels=()=>{}; populateInspector=()=>{}; attachTransformGizmo=()=>{}; detachTransformGizmo=()=>{}; refreshSelectionHighlight=()=>{}; removeSelectionHighlight=()=>{};
const rendered=item=>({item,object3d:{userData:{item}}});
const target=rendered({id:'target_bin_default',role:'target_bin',source_layer:'editable_layout',editable:true});
const zone=rendered({id:'place_zone_default',role:'place_zone',render_policy:'overlay',target_ref:'target_bin_default',source_layer:'editable_layout',editable:true});
const robot=rendered({id:'ur5_generated',role:'robot',source_layer:'locked_generated_urdf_visual',locked:true});
const commissioning=rendered({id:'commissioning_object',role:'task_marker',source_layer:'task_preview',locked:true});
state.objects=[target,zone,robot,commissioning]; state.sceneJson={scene:{id:'ur5_2f_test'}}; state.debugOverlaysVisible=false;
selectObject('place_zone_default');
assert.strictEqual(editorState().selectedItemId,'place_zone_default');
assert.strictEqual(editorState().uiSelectionItemId,'place_zone_default');
assert.strictEqual(editorState().selectedEditable,false);
assert.strictEqual(editorState().editOwnerItemId,'target_bin_default');
assert.strictEqual(editorState().selectionDiagnostics.uiSelectionItemId,'place_zone_default');
assert.strictEqual(editorState().selectionDiagnostics.editOwnerItemId,'target_bin_default');
assert.strictEqual(canonicalEditOwnerRendered(zone).item.id,'target_bin_default');
assert.strictEqual(inspectionSelectionRendered(zone).item.id,'place_zone_default');
state.debugOverlaysVisible=true; assert.strictEqual(rankedPickingCandidates([{object:zone.object3d,distance:1}])[0].rendered.item.id,'place_zone_default');
selectObject('target_bin_default'); assert.strictEqual(editorState().selectedItemId,'target_bin_default'); assert.strictEqual(editorState().selectedEditable,true);
selectObject('ur5_generated'); assert.strictEqual(editorState().selectedItemId,'ur5_generated'); assert.strictEqual(editorState().editOwnerItemId,'ur5_generated');
selectObject('commissioning_object'); assert.strictEqual(editorState().selectedItemId,'commissioning_object');
clearSelection(); assert.strictEqual(editorState().selectedItemId,'');
assert.strictEqual(editorState().uiSelectionItemId,'');
`,context);
"""
    subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, capture_output=True, text=True)

    select_body = _viewer_function_body(js_path.read_text(encoding="utf-8"), "function selectObject(id)", "function clearSelection()")
    dirty_body = _viewer_function_body(js_path.read_text(encoding="utf-8"), "function markDirtyTransform(rendered", "function editPatchEntryFor")
    assert "inspectionSelectionRendered(rawRequested)" in select_body
    assert "canonicalEditOwnerRendered(rendered)" in dirty_body


def test_explicit_ui_selection_identity_preserves_render_and_edit_identities():
    js_path = VIEWER / "viewer.js"
    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const element=()=>({hidden:false,checked:false,className:'',textContent:'',classList:{toggle(){}},querySelector(){return null;},querySelectorAll(){return[];},addEventListener(){},setAttribute(){},appendChild(){}});
const context={console,assert,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}}},document:{getElementById(){return element();},querySelectorAll(){return[];},createElement(){return element();}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context); vm.runInContext(source+`
updateLabels=()=>{}; populateInspector=()=>{}; attachTransformGizmo=()=>{}; detachTransformGizmo=()=>{}; refreshSelectionHighlight=()=>{}; removeSelectionHighlight=()=>{};
const rendered=item=>({item,object3d:{userData:{item}}});
const authored=[{id:'realsense_overhead'},{id:'support_surface_table'},{id:'target_bin_default'},{id:'place_zone_default'},{id:'pick_zone_commissioning'}];
const camera=rendered({id:'generated_urdf::camera_link::visual_17::17',camera_id:'realsense_overhead'});
const table=rendered({id:'generated_urdf::table_link::visual_2::2',support_surface_ref:'support_surface_table'});
const bin=rendered({id:'target_bin_default',editable:true});
const zone=rendered({id:'place_zone_default',target_ref:'target_bin_default',render_policy:'overlay'});
const invalid=rendered({id:'generated_robot_visual',canonical_item_id:'missing_robot'});
state.sceneJson={scene:{id:'ur5_2f_test'},items:authored}; state.objects=[camera,table,bin,zone,invalid,...authored.map(rendered)]; state.editorEvents=[];
selectObject(camera.item.id); assert.strictEqual(editorState().selectedItemId,camera.item.id); assert.strictEqual(editorState().uiSelectionItemId,'realsense_overhead'); assert.strictEqual(state.editorEvents.at(-1).uiItemId,'realsense_overhead');
selectObject(table.item.id); assert.strictEqual(editorState().selectedItemId,table.item.id); assert.strictEqual(editorState().uiSelectionItemId,'support_surface_table');
selectObject(bin.item.id); assert.strictEqual(editorState().uiSelectionItemId,'target_bin_default');
selectObject(zone.item.id); assert.strictEqual(editorState().uiSelectionItemId,'place_zone_default'); assert.strictEqual(editorState().editOwnerItemId,'target_bin_default');
selectObject(invalid.item.id); assert.strictEqual(editorState().uiSelectionItemId,invalid.item.id);
`,context);
"""
    subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, capture_output=True, text=True)


def test_hierarchy_selection_keeps_single_product_view_command_path():
    cpp = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")
    selection = cpp.split("void ScenePreviewWidget::select_preview_item", 1)[1].split("void ScenePreviewWidget::", 1)[0]
    for item_id in ["target_bin_default", "realsense_overhead", "support_surface_table", "pick_zone_commissioning", "commissioning_pick_pose", "place_zone_default"]:
        assert item_id not in selection
    assert selection.count("run_embedded_editor_command") == 2
    assert selection.count(".selectItem(") == 1
    assert selection.count(".clearSelection()") == 1


def test_viewer_status_reporting_ignores_hidden_helper_overlay_counters(tmp_path):
    js_path = VIEWER / "viewer.js"
    harness = r"""
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
const element = () => ({
  hidden: false,
  checked: false,
  disabled: false,
  textContent: '',
  className: '',
  innerHTML: '',
  classList: { toggle() {} },
  setAttribute() {},
  querySelector() { return { textContent: '' }; },
  appendChild() {},
  addEventListener() {},
  getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; },
});
const context = {
  console,
  assert,
  window: { location: { search: '' } },
  document: { getElementById() { return element(); }, createElement() { return element(); } },
  URLSearchParams,
  requestAnimationFrame() { return 0; },
};
vm.createContext(context);
vm.runInContext(source + `
populateObjectList = () => {};
updateLabels = () => {};
resetView = () => {};
renderSceneSummary = () => updateViewerStatus();
state.sceneJson = { scene: { id: 'status_test' }, frames: [{ name: 'tool0', link: 'tool0', parent_link: 'wrist_3_link', role: 'transform_anchor', type: 'frame', source_layer: 'generated_urdf', provenance: { source: 'payload.frames' }, world_pose: { xyz: [0.10, 0, 0], rpy: [0, 0, 0] } }] };
state.frameLookup = parseSceneFrames(state.sceneJson);
state.runtimeWarnings = [{ code: 'camera_framing_blocker_excluded', reason: 'hidden helper excluded' }];
class MockVector3 {
  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }
}
THREE = { Vector3: MockVector3 };
state.resolvedFramePoses.set('wrist_3_link', { xyz: new MockVector3(0, 0, 0), rpy: new MockVector3(0, 0, 0) });
state.resolvedFramePoses.set('tool0', { xyz: new MockVector3(0.10, 0, 0), rpy: new MockVector3(0, 0, 0) });
state.resolvedFramePoses.set('gripper_base_link', { xyz: new MockVector3(0.10, 0.20, 0), rpy: new MockVector3(0, 0, 0) });
state.objects = [
  { item: { id: 'robot_link', role: 'robot' }, renderInfo: { render_status: 'mesh_loaded' }, object3d: { visible: true } },
  { item: { id: 'workbench', category: 'environment' }, renderInfo: { render_status: 'mesh_loaded' }, object3d: { visible: true } },
  { item: { id: 'camera_fov_helper', role: 'camera_fov', category: 'overlay' }, renderInfo: { render_status: 'mesh_loaded' }, object3d: { visible: false } },
  { item: { id: 'pick_zone_bounds', role: 'pick_zone', category: 'safety_zone', renderInfo: { render_status: 'required_mesh_failed_debug_fallback' } }, renderInfo: { render_status: 'required_mesh_failed_debug_fallback' }, object3d: { visible: false } },
];
let status = updateViewerStatus();
assert.strictEqual(status.meshLoadedCount, 2);
assert.strictEqual(status.mesh_loaded_count, 2);
assert.strictEqual(status.requiredMeshFailedCount, 0);
assert.strictEqual(status.required_mesh_failed_count, 0);
assert.deepStrictEqual(status.resolvedFramePositions.wrist_3_link, [0, 0, 0]);
assert.deepStrictEqual(status.resolvedFramePositions.tool0, [0.10, 0, 0]);
assert.deepStrictEqual(status.resolvedFramePositions.gripper_base_link, [0.10, 0.20, 0]);
assert.deepStrictEqual(status.resolved_frame_positions, status.resolvedFramePositions);
assert.strictEqual(typeof status.viewer_resolved_distances_m['wrist_3_link -> tool0'], 'number');
assert.strictEqual(typeof status.viewer_resolved_distances_m['tool0 -> gripper_base_link'], 'number');
assert.strictEqual(typeof status.viewer_resolved_distances_m['wrist_3_link -> gripper_base_link'], 'number');
assert.strictEqual(status.viewer_resolved_distances_m['wrist_3_link -> tool0'], 0.10);
assert.strictEqual(status.viewer_resolved_distances_m['tool0 -> gripper_base_link'], 0.20);
assert.strictEqual(status.viewer_resolved_distances_m['wrist_3_link -> gripper_base_link'], Math.hypot(0.10, 0.20, 0));
assert.strictEqual(status.runtimeWarnings.length, 1);
assert.strictEqual(status.runtimeWarnings[0].code, 'camera_framing_blocker_excluded');
assert.strictEqual(isUserFacingWarning(status.runtimeWarnings[0]), false);
assert.deepStrictEqual(status.renderedMeshDiagnostics, []);
assert.deepStrictEqual(status.rendered_mesh_diagnostics, []);
assert.strictEqual(status.frameDiagnostics, status.frame_diagnostics);
const toolFrameDiagnostic = status.frameDiagnostics.find(entry => entry.name === 'tool0');
assert.ok(toolFrameDiagnostic, 'expected tool0 frame diagnostic even without rendered object');
assert.strictEqual(toolFrameDiagnostic.link, 'tool0');
assert.strictEqual(toolFrameDiagnostic.parent_link, 'wrist_3_link');
assert.strictEqual(toolFrameDiagnostic.role, 'transform_anchor');
assert.strictEqual(toolFrameDiagnostic.render_expected, false);
assert.strictEqual(toolFrameDiagnostic.mesh_available, false);
assert.deepStrictEqual(toolFrameDiagnostic.resolved_world_position, [0.10, 0, 0]);
setDebugOverlaysVisible(true);
assert.strictEqual(window.__WORKCELL_VIEWER_STATUS__.meshLoadedCount, 2);
assert.strictEqual(window.__WORKCELL_VIEWER_STATUS__.requiredMeshFailedCount, 0);
setDebugOverlaysVisible(false);
assert.strictEqual(window.__WORKCELL_VIEWER_STATUS__.meshLoadedCount, 2);
assert.strictEqual(window.__WORKCELL_VIEWER_STATUS__.requiredMeshFailedCount, 0);
`, context);
"""
    result = subprocess.run(
        ["node", "-e", harness, str(js_path)],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert result.stderr == ""


def test_viewer_status_static_contract_counts_physical_not_debug_overlay_visibility():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    summary_body = js.split("function computeSceneSummary", 1)[1].split("function isUserFacingWarning", 1)[0]
    status_body = js.split("function updateViewerStatus", 1)[1].split("function renderSceneSummary", 1)[0]
    debug_toggle_body = js.split("function setDebugOverlaysVisible", 1)[1].split("function updateLabels", 1)[0]

    assert "function statusCountedRenderables()" in js
    assert "!isDebugOverlayItem(obj.item)" in js
    assert "const statusRendered = statusCountedRenderables();" in summary_body
    assert "meshLoadedCount: statusRendered.filter" in summary_body
    assert "fallbackCount: statusRendered.filter" in summary_body
    assert "meshFailedCount: statusRendered.filter" in summary_body
    assert "requiredMeshFailedCount: statusCountedRenderables().filter(isRequiredMeshFailureStatus).length" in status_body
    assert "renderSceneSummary();" in debug_toggle_body
    assert "rendered.object3d.visible = state.debugOverlaysVisible" in debug_toggle_body


def test_viewer_status_exports_generated_urdf_mesh_diagnostics():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    diagnostics_body = _viewer_function_body(js, "function collectRenderedMeshDiagnostics()", "function updateViewerStatus")
    status_body = _viewer_function_body(js, "function updateViewerStatus()", "function renderSceneSummary")

    assert "state.three?.scene?.updateMatrixWorld?.(true);" in diagnostics_body
    assert "rendered.object3d.updateMatrixWorld(true);" in diagnostics_body
    assert "rendered.object3d.getWorldPosition(linkFrameWorldPosition);" in diagnostics_body
    assert "rendered.meshObject.getWorldPosition(visualWrapperWorld);" in diagnostics_body
    assert "new THREE.Box3().setFromObject(object)" in js
    for token in [
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
        "tool0",
        "gripper_base_link",
    ]:
        assert token in js
    for token in [
        "id:",
        "link:",
        "link_name:",
        "frame:",
        "display_name:",
        "linkFrameWorldPosition",
        "visualWrapperWorldPosition",
        "loadedMeshBoundingBoxCenter",
        "loadedMeshBoundingBoxSize",
    ]:
        assert token in diagnostics_body
    assert "const renderedMeshDiagnostics = collectRenderedMeshDiagnostics();" in status_body
    assert "renderedMeshDiagnostics" in status_body
    assert "rendered_mesh_diagnostics: renderedMeshDiagnostics" in status_body
    assert "const frameDiagnostics = collectFrameDiagnostics();" in status_body
    assert "frameDiagnostics" in status_body
    assert "frame_diagnostics: frameDiagnostics" in status_body


def test_viewer_support_surface_display_types_and_metadata_diagnostics():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "function supportSurfaceDisplayType(item)",
        "support_surface_kind",
        "supportSurfaceKind",
        "workbench_body",
        "Workbench / support surface",
        "table_surface",
        "tabletop",
        "Tabletop / support surface",
        "top_surface_z_m",
        "topSurfaceZM",
        "support_surface_height_m",
        "supportSurfaceHeightM",
        "expected_support_footprint_m",
        "expectedSupportFootprintM",
        "support_surface_display_type",
        "supportSurfaceDisplayType",
    ]:
        assert token in js

    collect_body = js.split("function collectRenderedMeshDiagnostics", 1)[1].split("function updateViewerStatus", 1)[0]
    assert "...supportSurfaceMetadata(item)" in collect_body
    status_body = js.split("const renderedObjectStatuses = statusCountedRenderables().map", 1)[1].split("window.__WORKCELL_VIEWER_STATUS__", 1)[0]
    assert "support_surface_kind" in status_body
    assert "supportSurfaceKind" in status_body
    assert "support_surface_display_type" in status_body
    assert "supportSurfaceDisplayType" in status_body


def test_viewer_support_surface_ui_logic_and_semantic_warnings():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    item_type_body = js.split("function itemType(item)", 1)[1].split("function itemLabel(item)", 1)[0]
    assert "const supportSurfaceType = supportSurfaceDisplayType(item);" in item_type_body
    assert "if (supportSurfaceType) return supportSurfaceType;" in item_type_body

    item_label_body = js.split("function itemLabel(item)", 1)[1].split("function viewerGroupIdentity", 1)[0]
    assert "supportSurfaceType" in item_label_body
    assert "(${supportSurfaceType})" in item_label_body

    inspector_body = js.split("function populateInspector", 1)[1].split("function refreshWarnings", 1)[0]
    assert "type: itemType(item)" in inspector_body
    assert "details: supportSurfaceDisplayType(item)" in inspector_body
    assert "support_surface_height_m" in inspector_body
    assert "expected_support_footprint_m" in inspector_body

    list_body = js.split("function appendObjectListRow", 1)[1].split("function populateObjectList", 1)[0]
    assert "supportSurfaceDisplayType(rendered.item) || meshStatusLabel(rendered)" in list_body

    for token in [
        "function maybeWarnSupportSurfaceSemantics",
        "support_surface_semantics_missing_height",
        "workbench_body support surface is missing top/support height metadata",
        "support_surface_semantics_non_thin_tabletop",
        "tabletop/table_surface support surface has non-thin loaded bounds",
        "maybeWarnSupportSurfaceSemantics(item, dims)",
    ]:
        assert token in js


def test_viewer_builds_assembled_urdf_hierarchy_instead_of_flattened_baked_roots():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "function buildRobotAssemblies(items)",
        "assembly_group",
        "robot_instance_id",
        "robot_render_mode: 'assembled_urdf_hierarchy'",
        "nodes.get(parent).add(node)",
        "applyPoseBlockToObject(node, jointOriginOfItem(representative))",
        "state.three.scene.add(root)",
        "state.assemblyRoots.push(root)",
    ]:
        assert token in js
    body = js.split("function buildRobotAssemblies(items)", 1)[1].split("function createLabelElement", 1)[0]
    assert body.index("nodes.get(parent).add(node)") < body.index("tryLoadMesh(item, rendered, fallback)")
    assert "generatedUrdfFramePoseSource(representative)" in body


def test_viewer_visual_origin_is_local_child_in_assembled_hierarchy_not_baked_twice():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    assert "function usesAssembledUrdfHierarchy(item)" in js
    uses_baked = js.split("function usesBakedVisibleWorldPose(item)", 1)[1].split("function effectiveWorkcellWebRenderPoseMode", 1)[0]
    assert "if (usesAssembledUrdfHierarchy(item)) return false;" in uses_baked
    mesh_local = js.split("function applyMeshLocalTransform(meshObject, item)", 1)[1].split("function applyLoadedMeshScaleHandling", 1)[0]
    assert "? (usesBakedVisibleWorldPose(item) ? poseBlockOf({ xyz: [0, 0, 0], rpy: [0, 0, 0] }) : visualOriginOf(item))" in mesh_local
    assert "meshObject.position.copy(visualOrigin.xyz)" in mesh_local


def test_viewer_includes_tool0_meshless_frame_in_assembly_items():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    assert "'frames'" in js
    body = js.split("function collectItems(sceneJson)", 1)[1].split("function frameNameOf", 1)[0]
    assert "'frames'" in body
    assembly = js.split("function buildRobotAssemblies(items)", 1)[1].split("function createLabelElement", 1)[0]
    assert "'wrist_3_link','tool0'" in assembly
    assert "'tool0','gripper_base_link'" in assembly
    assert "robot_hierarchy_missing_links" in assembly


def test_viewer_reports_hierarchy_acceptance_diagnostics():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "robot_hierarchy_links",
        "robot_hierarchy_missing_links",
        "robot_hierarchy_missing_parents",
        "robot_hierarchy_mesh_count",
        "assembled_link_world_positions",
        "assembled_link_adjacency_distances_m",
        "base_link_inertia','shoulder_link",
        "tool0','gripper_base_link",
    ]:
        assert token in js


def test_viewer_has_expanded_urdf_loader_robot_preview_path():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    index = (VIEWER / "index.html").read_text(encoding="utf-8")
    assert "expanded_urdf_loader" in js
    assert "function loadExpandedUrdfRobotPreview" in js
    assert "const urdfPreviewActive = isExpandedUrdfRobotPreview(state.sceneJson?.robot_preview);" in js
    assert "new Set(robotToolGeneratedUrdfItems)" in js
    assert "robot_preview_loaded" in js
    assert "robot_loaded_link_count" in js
    assert "robot_loaded_visual_count" in js
    assert "robot_missing_meshes" in js
    assert "skipped_legacy_generated_urdf_visual_count" in js
    assert "Robot preview: expanded URDF loader" in js
    assert 'data-summary-field="robot-preview-mode"' in index


def test_urdf_renderer_waits_for_mesh_completion_before_ready():
    js = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    assert "function meshCompletionPromise" in js
    assert "await meshCompletion.wait();" in js
    assert "robot_mesh_callbacks_complete" in js
    assert "robot_expected_visual_count === (diagnostics.robot_loaded_visual_count + diagnostics.robot_failed_visual_count)" in js
    ready_section = js.split("await meshCompletion.wait();", 1)[1].split("setLifecycleState(diagnostics, diagnostics.robot_preview_loaded ? 'ready' : 'failed')", 1)[0]
    assert "robot.setJointValues(jointValues)" in ready_section
    assert "robot.updateMatrixWorld(true)" in ready_section
    assert "collectDescendantRenderMeshDiagnostics(robot.links)" in ready_section


def test_urdf_renderer_fetches_and_validates_urdf_before_parse():
    js = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    body = _viewer_function_body(js, "export async function fetchValidatedUrdfRobotElement", "function collectLinkMatrixDiagnostics")
    assert "fetch(urdfUrl, { cache: 'no-store' })" in body
    assert "if (!response?.ok)" in body
    assert "http_status=${statusText}" in js
    assert "new DOMParser().parseFromString(text, 'application/xml')" in body
    assert "getElementsByTagName?.('parsererror')" in body
    assert "stage}: url=${url" in js
    assert "validate_robot_root" in body
    assert "expected top-level <robot> element" in body


def test_urdf_renderer_parse_path_uses_validated_xml_element_not_load_async():
    js = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    ready_body = js.split("result.ready = (async () => {", 1)[1].split("setLifecycleState(diagnostics, 'loading_meshes');", 1)[0]
    assert "const urdfUrl = repoUrl(rendererContext, previewConfig?.urdf_url || '');" in ready_body
    assert "const urdfRobotElement = await fetchValidatedUrdfRobotElement(urdfUrl);" in ready_body
    assert "const robot = loader.parse(urdfRobotElement" in ready_body
    assert "loader.loadAsync(urdfUrl)" not in js


def test_urdf_renderer_reports_html_malformed_xml_and_missing_robot_root_stages():
    js = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    body = _viewer_function_body(js, "export async function fetchValidatedUrdfRobotElement", "function collectLinkMatrixDiagnostics")
    # HTTP 200 HTML and malformed XML both surface through parsererror / parse_xml instead of URDFLoader children crashes.
    assert "parse_xml" in body
    assert "parserError.textContent?.trim() || 'malformed XML'" in body
    # XML without a top-level robot is rejected before URDFLoader.parse.
    assert "String(root.tagName || '').toLowerCase() !== 'robot'" in body
    assert "got <${root?.tagName || 'none'}>" in body


def test_urdf_renderer_diagnostics_skip_undefined_links_and_use_optional_children():
    js = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    visual_body = _viewer_function_body(js, "function collectVisualWrapperMatrixDiagnostics", "function collectDescendantRenderMeshDiagnostics")
    descendant_body = _viewer_function_body(js, "function collectDescendantRenderMeshDiagnostics", "function buildLookupMap")
    assert "if (!link) continue;" in visual_body
    assert "for (const child of link?.children || [])" in visual_body
    assert "if (!link) continue;" in descendant_body
    assert "for (const visual of link?.children || [])" in descendant_body


def test_urdf_renderer_adds_successful_robot_only_after_meshes_joints_and_diagnostics():
    js = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    ready_body = js.split("result.ready = (async () => {", 1)[1].split("})().catch(err =>", 1)[0]
    assert ready_body.index("await meshCompletion.wait();") < ready_body.index("robot.setJointValues(jointValues)")
    assert ready_body.index("robot.setJointValues(jointValues)") < ready_body.index("diagnostics.robot_preview_loaded =")
    assert ready_body.index("diagnostics.robot_preview_loaded =") < ready_body.index("rendererContext?.scene?.add?.(robot)")
    assert "rendererContext?.scene?.add?.(robot);" in ready_body
    assert "rendererContext?.onRobotLoaded?.(result);" in ready_body
    assert ready_body.index("rendererContext?.scene?.add?.(robot)") < ready_body.index("rendererContext?.onRobotLoaded?.(result)")


def test_urdf_renderer_optional_diagnostics_do_not_scene_fail_successful_render():
    js = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    ready_body = js.split("result.ready = (async () => {", 1)[1].split("})().catch(err =>", 1)[0]
    assert "collectDescendantRenderMeshDiagnostics(robot.links)" in ready_body
    assert "for (const visual of link?.children || [])" in js
    diagnostics_tail = ready_body.split("diagnostics.robot_preview_loaded =", 1)[1]
    assert "rendererContext?.onRobotError" not in diagnostics_tail
    assert "throw" not in diagnostics_tail


def test_urdf_renderer_normalizes_collada_loader_root_transform_generically():
    js = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    assert "function normalizeRosColladaScene" in js
    assert "loadColladaWithSceneScopedZUpDiagnostic(new ColladaLoader(manager), url, dae => onDone(normalizeRosColladaScene(dae, uri, diagnostics))" in js
    assert "upAxis === 'Z_UP' || (Number.isFinite(unitMeter)" in js
    assert "upAxis === 'Z_UP' || upAxis === 'Y_UP'" not in js
    assert "robot_collada_root_normalization_count" in js
    assert "robot_descendant_render_mesh_diagnostics" in js
    collada_section = js.split("function normalizeRosColladaScene", 1)[1].split("function loadMesh", 1)[0]
    assert "shoulder_link" not in collada_section
    assert "Math.PI / 2" not in collada_section
    assert "1.5707963267948966" not in collada_section
    assert "rotateX" not in collada_section
    assert "rotateY" not in collada_section
    assert "rotateZ" not in collada_section


def test_urdf_renderer_documents_single_ros_to_three_conversion_boundary():
    js = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    assert "ROS_TO_THREE_CONVERSION_BOUNDARY" in js
    assert "DAE, STL, assembled URDF, and flattened fallback diagnostics" in js
    assert "must not add per-link or per-mesh 90-degree corrections after ColladaLoader" in js
    assert "Do not add a\n  // second hardcoded 90-degree correction to any link, mesh, baked world matrix" in js
    assert "assembled URDF node, or flattened fallback diagnostic row" in js


def test_urdf_renderer_collada_root_normalization_is_diagnostic_only_with_matrix_parity(tmp_path):
    script = tmp_path / "collada_root_boundary.mjs"
    script.write_text(
        f"""
import assert from 'node:assert/strict';
import * as THREE from {str((VIEWER / 'node_modules/three/build/three.module.js')).__repr__()};
import {{ normalizeRosColladaScene, ROS_TO_THREE_CONVERSION_BOUNDARY }} from {str((VIEWER / 'urdf_robot_renderer.js')).__repr__()};

const diagnostics = {{
  robot_collada_root_normalization_count: 0,
  robotColladaRootNormalizationCount: 0,
  robot_collada_mesh_diagnostics: [],
}};
const scene = new THREE.Group();
scene.rotation.x = -Math.PI / 2;
scene.add(new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshBasicMaterial()));
const normalized = normalizeRosColladaScene({{ scene, asset: {{ upAxis: 'Z_UP', unit: 1 }} }}, 'base.dae', diagnostics);
assert.equal(normalized, scene);
assert.equal(diagnostics.robot_collada_root_normalization_count, 1);
assert.equal(diagnostics.robotColladaRootNormalizationCount, 1);
assert.equal(diagnostics.robot_collada_mesh_diagnostics[0].root_transform_normalized, true);
assert.equal(scene.rotation.x, 0);
assert.equal(scene.quaternion.x, 0);

const visualWrapper = new THREE.Group();
visualWrapper.position.set(1, 2, 3);
visualWrapper.add(scene);
visualWrapper.updateMatrixWorld(true);
const expectedWrapper = new THREE.Matrix4().makeTranslation(1, 2, 3).elements;
assert.deepEqual(Array.from(visualWrapper.matrixWorld.elements), Array.from(expectedWrapper));
assert.equal(ROS_TO_THREE_CONVERSION_BOUNDARY.includes('DAE, STL, assembled URDF, and flattened fallback'), true);

const authoredYUp = new THREE.Group();
authoredYUp.rotation.x = -Math.PI / 2;
normalizeRosColladaScene({{ scene: authoredYUp, asset: {{ upAxis: 'Y_UP', unit: 1 }} }}, 'authored-y-up.dae', diagnostics);
assert.equal(diagnostics.robot_collada_root_normalization_count, 1);
assert.equal(authoredYUp.rotation.x, -Math.PI / 2);
""",
        encoding="utf-8",
    )
    subprocess.run(["node", str(script)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def test_viewer_expanded_urdf_preview_helper_accepts_canonical_and_legacy_modes():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    helper = js.split("function isExpandedUrdfRobotPreview(preview)", 1)[1].split("function robotPreviewSummaryMode", 1)[0]
    assert "expanded_urdf_loader" in helper
    assert "expanded_urdf_robot_subtree" in helper
    render_body = js.split("function renderScene(items)", 1)[1].split("function loadExpandedUrdfRobotPreview", 1)[0]
    assert "isExpandedUrdfRobotPreview(state.sceneJson?.robot_preview)" in render_body
    assert "state.sceneJson?.robot_preview?.mode === 'expanded_urdf_loader'" not in render_body


def test_viewer_urdf_preview_suppression_is_limited_to_robot_tool_generated_rows():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    helper = js.split("function isRobotToolGeneratedUrdfMeshVisualItem(item)", 1)[1].split("function itemAssemblyGroup", 1)[0]
    identity_helper = js.split("function viewerGroupIdentity(item)", 1)[1].split("function isGeneratedPreviewIdentity", 1)[0]
    assert "isGeneratedUrdfMeshVisualItem(item)" in helper
    assert "isGeneratedRobotItem(item) || isGeneratedToolOrGripperItem(item)" in helper
    for semantic_field in [
        "robot_instance_id",
        "link_name",
        "visual_index",
        "category",
        "role",
        "renderer_owner",
        "renderer_ownership",
    ]:
        assert semantic_field in identity_helper
    render_body = js.split("function renderScene(items)", 1)[1].split("function loadExpandedUrdfRobotPreview", 1)[0]
    assert "items.filter(isRobotToolGeneratedUrdfMeshVisualItem)" in render_body
    assert "items.filter(isGeneratedUrdfMeshVisualItem)" not in render_body
    assert "new Set(robotToolGeneratedUrdfItems)" in render_body
    assert "if (assemblyBuild.handled.has(item)) continue" in render_body
    assert "mesh_uri" not in helper
    assert "mesh_path" not in helper
    assert "source_path" not in helper


def test_expanded_urdf_loader_keeps_environment_urdf_rows_generic_renderable():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    render_body = js.split("function renderScene(items)", 1)[1].split("function loadExpandedUrdfRobotPreview", 1)[0]

    assert "const urdfPreviewActive = isExpandedUrdfRobotPreview(state.sceneJson?.robot_preview)" in render_body
    assert "const robotToolGeneratedUrdfItems = items.filter(isRobotToolGeneratedUrdfMeshVisualItem)" in render_body
    assert "handled: new Set(robotToolGeneratedUrdfItems)" in render_body
    assert "if (assemblyBuild.handled.has(item)) continue" in render_body

    helper = js.split("function isRobotToolGeneratedUrdfMeshVisualItem(item)", 1)[1].split("function itemAssemblyGroup", 1)[0]
    assert "isGeneratedRobotItem(item) || isGeneratedToolOrGripperItem(item)" in helper
    for environment_semantic in ["table", "camera", "workbench", "environment"]:
        assert environment_semantic not in helper
    # The expanded-URDF ownership set is intentionally object-identity based so
    # generated robot/tool rows that share Robotiq mesh filenames remain distinct,
    # while generated table/camera/workbench URDF rows are never suppressed merely
    # because they are mesh-backed urdf_flattened rows.
    assert "new Set(robotToolGeneratedUrdfItems)" in render_body
    assert "displayMeshUri(item)" not in helper


def test_viewer_summary_reports_expanded_urdf_loader_for_aliases():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    summary = js.split("function robotPreviewSummaryMode(preview)", 1)[1].split("function isGeneratedUrdfMeshVisualItem", 1)[0]
    assert "Robot preview: expanded URDF loader" in summary
    assert "isExpandedUrdfRobotPreview(preview)" in summary
    summary_fields = js.split("const fields = {", 1)[1].split("};", 1)[0]
    assert "'robot-preview-mode': summary.robotPreviewMode" in summary_fields
    assert "summary.robotPreviewMode === 'expanded_urdf_loader'" not in summary_fields


def test_expanded_urdf_assembly_roots_are_first_class_physical_fit_bounds():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    fit_bounds_body = js.split("function computeFitBounds", 1)[1].split("function computeRenderedBounds", 1)[0]
    collect_body = js.split("function collectPhysicalAssemblyBounds", 1)[1].split("function itemHiddenForFit", 1)[0]
    status_body = js.split("function collectAssemblyRenderDiagnostics", 1)[1].split("function isUserFacingWarning", 1)[0]

    assert "function collectPhysicalAssemblyBounds" in js
    assert "for (const root of state.assemblyRoots || [])" in collect_body
    assert "visibleRenderableBounds(root)" in collect_body
    assert "if (!rootBounds) continue" in collect_body
    assert "assemblyRootHiddenForFit(root)" in collect_body
    assert "normalBounds.union(assemblyBounds.bounds)" in fit_bounds_body
    assert "state.physicalAssemblyBounds = assemblyBounds.bounds.clone()" in fit_bounds_body
    assert "state.finalPhysicalFitBounds = finiteNormal.clone()" in fit_bounds_body
    for token in [
        "physical_assembly_root_count",
        "physical_assembly_bounds",
        "physical_fit_included_robot_preview",
        "final_physical_fit_bounds",
        "physical_renderable_count",
    ]:
        assert token in status_body


def test_expanded_urdf_robot_preview_defers_initial_fit_until_scene_ready():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    preview_body = js.split("function loadExpandedUrdfRobotPreview", 1)[1].split("function linkNameOfItem", 1)[0]
    loaded_body = preview_body.split("onRobotLoaded: result =>", 1)[1].split("onRobotMeshLoaded", 1)[0]
    mesh_loaded_body = preview_body.split("onRobotMeshLoaded: () =>", 1)[1].split("onRobotMeshLoadError", 1)[0]

    assert "completeExpandedUrdfReadiness(result)" in loaded_body
    assert "attemptInitialCameraFit" not in loaded_body
    assert "renderSceneSummary()" in mesh_loaded_body
    assert "scheduleInitialCameraFitRetry" not in mesh_loaded_body
    assert "attemptInitialCameraFit" not in mesh_loaded_body


def test_expanded_urdf_robot_assembly_stays_locked_single_root_and_legacy_rows_suppressed():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    render_body = js.split("function renderScene(items)", 1)[1].split("function loadExpandedUrdfRobotPreview", 1)[0]
    assert "const urdfPreviewActive = isExpandedUrdfRobotPreview" in render_body
    assert "handled: new Set(robotToolGeneratedUrdfItems)" in render_body
    assert "skipped_legacy_generated_urdf_visual_count: robotToolGeneratedUrdfItems.length" in render_body
    assert "if (assemblyBuild.handled.has(item)) continue" in render_body
    assert "source.includes('generated')" in js
    renderer = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    ready_body = renderer.split("await meshCompletion.wait();", 1)[1].split("return result;", 1)[0]
    assert ready_body.count("rendererContext?.scene?.add?.(robot)") == 1
    assert ready_body.count("rendererContext?.assemblyRoots?.push?.(robot)") == 1
    assert "world/root fixed chain -> URDF joint origin -> joint value -> link frame" in renderer


def test_collect_physical_visible_bounds_filters_product_view_geometry():
    js_path = VIEWER / "viewer.js"
    harness = r'''
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
class MockVector3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
}
class MockBox3 {
  constructor(min, max) {
    if (max) { this.min = { ...min }; this.max = { ...max }; }
    else { this.min = { x: Infinity, y: Infinity, z: Infinity }; this.max = { x: -Infinity, y: -Infinity, z: -Infinity }; }
  }
  isEmpty() { return this.min.x > this.max.x || this.min.y > this.max.y || this.min.z > this.max.z; }
  union(box) {
    this.min.x = Math.min(this.min.x, box.min.x); this.min.y = Math.min(this.min.y, box.min.y); this.min.z = Math.min(this.min.z, box.min.z);
    this.max.x = Math.max(this.max.x, box.max.x); this.max.y = Math.max(this.max.y, box.max.y); this.max.z = Math.max(this.max.z, box.max.z);
    return this;
  }
  setFromObject(object) { this.min = { ...object.mockBounds.min }; this.max = { ...object.mockBounds.max }; return this; }
  getSize(target) { target.x = this.max.x - this.min.x; target.y = this.max.y - this.min.y; target.z = this.max.z - this.min.z; return target; }
  getCenter(target) { target.x = (this.min.x + this.max.x) / 2; target.y = (this.min.y + this.max.y) / 2; target.z = (this.min.z + this.max.z) / 2; return target; }
  clone() { return new MockBox3(this.min, this.max); }
}
const element = () => ({ hidden: false, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', classList: { toggle() {} }, setAttribute() {}, querySelector() { return { textContent: '' }; }, appendChild() {}, addEventListener() {}, getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; } });
const sandbox = { console, assert, MockVector3, MockBox3, window: { location: { search: '' } }, document: { getElementById() { return element(); }, createElement() { return element(); } }, URLSearchParams, requestAnimationFrame() { return 0; } };
vm.createContext(sandbox);
vm.runInContext(source + `
THREE = { Vector3: MockVector3, Box3: MockBox3 };
function root(children = []) { return { visible: true, children, userData: {}, updateWorldMatrix() {} }; }
function mesh(id, min, max, userData = {}) { return { isMesh: true, visible: true, children: [], name: id, mockBounds: { min, max }, userData }; }
const physical = mesh('table', { x: 1, y: 2, z: 3 }, { x: 2, y: 3, z: 4 }, { item: { id: 'workbench', category: 'table', source_layer: 'editable_layout' } });
let result = collectPhysicalVisibleBounds(root([physical]));
assert.strictEqual(result.count, 1);
assert.deepStrictEqual(result.bounds_json.min, { x: 1, y: 2, z: 3 });
assert.deepStrictEqual(result.bounds_json.max, { x: 2, y: 3, z: 4 });

const hidden = mesh('hidden', { x: -100, y: -100, z: -100 }, { x: -90, y: -90, z: -90 }, { item: { id: 'hidden_part', category: 'object' } });
hidden.visible = false;
const grid = mesh('grid', { x: -50, y: -50, z: 0 }, { x: 50, y: 50, z: 0 }, {}); grid.isGridHelper = true;
const axes = mesh('axes', { x: -50, y: -50, z: 0 }, { x: 50, y: 50, z: 50 }, {}); axes.isAxesHelper = true;
const zone = mesh('pick_zone', { x: -20, y: -20, z: 0 }, { x: 20, y: 20, z: 1 }, { item: { id: 'pick_zone', category: 'safety_zone', source_layer: 'editable_layout' } });
const outline = mesh('selection', { x: -10, y: -10, z: -10 }, { x: 10, y: 10, z: 10 }, { selection_outline: true, item: { id: 'selection_outline', role: 'selection_highlight' } });
result = collectPhysicalVisibleBounds(root([physical, hidden, grid, axes, zone, outline]));
assert.strictEqual(result.count, 1);
assert.deepStrictEqual(result.bounds_json.min, { x: 1, y: 2, z: 3 });

const fallback = mesh('robot_box', { x: -100, y: -1, z: 0 }, { x: -90, y: 1, z: 2 }, { item: { id: 'robot_fallback', role: 'robot', source_layer: 'primitive_fallback', active_visual_source: 'primitive_fallback' } });
const generated = mesh('robot_mesh', { x: 0, y: 0, z: 0 }, { x: 1, y: 1, z: 1 }, { item: { id: 'ur5_mesh', role: 'robot', source_layer: 'locked_generated_urdf_visual', active_visual_source: 'mesh_preview' } });
result = collectPhysicalVisibleBounds(root([fallback, generated]));
assert.strictEqual(result.count, 1);
assert.deepStrictEqual(result.bounds_json.min, { x: 0, y: 0, z: 0 });

result = collectPhysicalVisibleBounds(root([]));
assert.strictEqual(result.count, 0);
assert.strictEqual(result.bounds, null);
assert.strictEqual(result.bounds_json, null);

const finite = collectPhysicalVisibleBounds(root([physical])).bounds_json;
for (const section of ['min', 'max', 'center', 'dimensions']) for (const value of Object.values(finite[section])) assert.strictEqual(Number.isFinite(value), true);
`, sandbox);
'''
    result = subprocess.run(
        ["node", "-e", harness, str(js_path)],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert result.stderr == ""


def test_fit_cell_uses_physical_visible_bounds_and_preserves_camera_on_empty():
    js_path = VIEWER / "viewer.js"
    harness = r'''
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
class MockVector3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
  copy(v) { this.x = v.x; this.y = v.y; this.z = v.z; return this; }
  addScaledVector(v, s) { this.x += v.x * s; this.y += v.y * s; this.z += v.z * s; return this; }
  normalize() { const l = Math.hypot(this.x, this.y, this.z) || 1; this.x /= l; this.y /= l; this.z /= l; return this; }
}
class MockSphere { constructor() { this.radius = 0; } }
class MockBox3 {
  constructor(min, max) {
    if (max) { this.min = { ...min }; this.max = { ...max }; }
    else { this.min = { x: Infinity, y: Infinity, z: Infinity }; this.max = { x: -Infinity, y: -Infinity, z: -Infinity }; }
  }
  isEmpty() { return this.min.x > this.max.x || this.min.y > this.max.y || this.min.z > this.max.z; }
  union(box) { for (const k of ['x','y','z']) { this.min[k] = Math.min(this.min[k], box.min[k]); this.max[k] = Math.max(this.max[k], box.max[k]); } return this; }
  setFromObject(object) { this.min = { ...object.mockBounds.min }; this.max = { ...object.mockBounds.max }; return this; }
  getSize(target) { target.x = this.max.x - this.min.x; target.y = this.max.y - this.min.y; target.z = this.max.z - this.min.z; return target; }
  getCenter(target) { target.x = (this.min.x + this.max.x) / 2; target.y = (this.min.y + this.max.y) / 2; target.z = (this.min.z + this.max.z) / 2; return target; }
  getBoundingSphere(sphere) { const dx = this.max.x - this.min.x, dy = this.max.y - this.min.y, dz = this.max.z - this.min.z; sphere.radius = Math.hypot(dx, dy, dz) / 2; return sphere; }
  clone() { return new MockBox3(this.min, this.max); }
}
const element = () => ({ hidden: true, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', classList: { toggle() {} }, setAttribute() {}, querySelector() { return { textContent: '' }; }, appendChild() {}, addEventListener() {}, getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; } });
const sandbox = { console, assert, MockVector3, MockSphere, MockBox3, window: { location: { search: '' } }, document: { getElementById() { return element(); }, createElement() { return element(); } }, URLSearchParams, requestAnimationFrame() { return 0; } };
vm.createContext(sandbox);
vm.runInContext(source + `
THREE = { Vector3: MockVector3, Sphere: MockSphere, Box3: MockBox3 };
function root(children = []) { return { visible: true, children, userData: {}, updateWorldMatrix() {} }; }
function mesh(id, min, max, userData = {}) { return { isMesh: true, visible: true, children: [], name: id, mockBounds: { min, max }, userData }; }
state.three.camera = { position: new MockVector3(9, 8, 7), near: 0.5, far: 50, updateProjectionMatrix() { this.updated = true; } };
state.three.controls = { target: new MockVector3(1, 1, 1), update() { this.updated = true; } };
const physical = mesh('robot_mesh', { x: 0, y: 0, z: 0 }, { x: 1, y: 1, z: 1 }, { item: { id: 'ur5_mesh', role: 'robot', source_layer: 'locked_generated_urdf_visual', active_visual_source: 'mesh_preview' } });
const overlay = mesh('work_envelope', { x: -100, y: -100, z: -100 }, { x: 100, y: 100, z: 100 }, { item: { id: 'work_envelope', role: 'helper', category: 'reachability' } });
const hidden = mesh('hidden_table', { x: 30, y: 30, z: 30 }, { x: 40, y: 40, z: 40 }, { item: { id: 'hidden_table', category: 'table' } }); hidden.visible = false;
state.three.scene = root([physical, overlay, hidden]);
let helperCalls = 0;
const originalHelper = collectPhysicalVisibleBounds;
collectPhysicalVisibleBounds = rootArg => { helperCalls += 1; return originalHelper(rootArg); };
resetView();
assert.strictEqual(helperCalls, 1);
assert.strictEqual(JSON.stringify(state.lastFrameBounds.min), JSON.stringify({ x: 0, y: 0, z: 0 }));
assert.strictEqual(JSON.stringify(state.lastFrameBounds.max), JSON.stringify({ x: 1, y: 1, z: 1 }));
for (const value of [state.three.camera.position.x, state.three.camera.position.y, state.three.camera.position.z, state.three.camera.near, state.three.camera.far]) assert.strictEqual(Number.isFinite(value), true);
const fittedPosition = { ...state.three.camera.position };
updateViewerStatus(); refreshWarnings({ warnings: [] }); renderSceneSummary();
assert.strictEqual(helperCalls, 1);
assert.strictEqual(JSON.stringify(state.three.camera.position), JSON.stringify(fittedPosition));
state.three.camera.position.copy(new MockVector3(3, 4, 5));
state.three.scene = root([overlay]);
resetView();
assert.strictEqual(helperCalls, 2);
assert.strictEqual(JSON.stringify(state.three.camera.position), JSON.stringify({ x: 3, y: 4, z: 5 }));
assert.strictEqual(state.editorError, 'No visible physical geometry to frame');
assert.strictEqual(state.editorEvents.at(-1).type, 'fit_cell_unavailable');
`, sandbox);
'''
    result = subprocess.run(
        ["node", "-e", harness, str(js_path)],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert result.stderr == ""



def test_fit_selection_frames_shared_selected_physical_root_and_safe_fallbacks():
    js_path = VIEWER / "viewer.js"
    harness = r"""
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
class MockVector3 { constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; } copy(v) { this.x = v.x; this.y = v.y; this.z = v.z; return this; } addScaledVector(v, s) { this.x += v.x * s; this.y += v.y * s; this.z += v.z * s; return this; } normalize() { const l = Math.hypot(this.x, this.y, this.z) || 1; this.x /= l; this.y /= l; this.z /= l; return this; } }
class MockSphere { constructor() { this.radius = 0; } }
class MockBox3 {
  constructor(min, max) { if (max) { this.min = { ...min }; this.max = { ...max }; } else { this.min = { x: Infinity, y: Infinity, z: Infinity }; this.max = { x: -Infinity, y: -Infinity, z: -Infinity }; } }
  isEmpty() { return this.min.x > this.max.x || this.min.y > this.max.y || this.min.z > this.max.z; }
  union(box) { for (const k of ['x','y','z']) { this.min[k] = Math.min(this.min[k], box.min[k]); this.max[k] = Math.max(this.max[k], box.max[k]); } return this; }
  setFromObject(object) { this.min = { ...object.mockBounds.min }; this.max = { ...object.mockBounds.max }; return this; }
  getCenter(target) { target.x = (this.min.x + this.max.x) / 2; target.y = (this.min.y + this.max.y) / 2; target.z = (this.min.z + this.max.z) / 2; return target; }
  getSize(target) { target.x = this.max.x - this.min.x; target.y = this.max.y - this.min.y; target.z = this.max.z - this.min.z; return target; }
  getBoundingSphere(sphere) { sphere.radius = Math.hypot(this.max.x - this.min.x, this.max.y - this.min.y, this.max.z - this.min.z) / 2; return sphere; }
  clone() { return new MockBox3(this.min, this.max); }
}
const element = () => ({ hidden: true, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', classList: { toggle() {} }, setAttribute() {}, querySelector() { return element(); }, querySelectorAll() { return []; }, appendChild() {}, addEventListener() {}, getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; } });
const sandbox = { console, assert, MockVector3, MockSphere, MockBox3, window: { location: { search: '' } }, document: { getElementById() { return element(); }, querySelectorAll() { return []; }, createElement() { return element(); } }, URLSearchParams, requestAnimationFrame() { return 0; } };
vm.createContext(sandbox);
vm.runInContext(source + `
THREE = { Vector3: MockVector3, Sphere: MockSphere, Box3: MockBox3 };
function group(item, children = []) { return { visible: true, children, userData: { item }, traverse(fn) { fn(this); for (const c of children) c.traverse ? c.traverse(fn) : fn(c); }, updateWorldMatrix() {} }; }
function mesh(min, max, userData = {}) { return { isMesh: true, visible: true, children: [], mockBounds: { min, max }, userData, traverse(fn) { fn(this); } }; }
state.three.camera = { position: new MockVector3(9, 8, 7), near: 0.5, far: 50, updateProjectionMatrix() {} };
state.three.controls = { target: new MockVector3(1, 1, 1), update() {} };
const tableRoot = group({ id: 'table', category: 'table', source_layer: 'editable_layout' }, [
  mesh({ x: 10, y: 0, z: 0 }, { x: 12, y: 2, z: 1 }, { item: { id: 'table', category: 'table', source_layer: 'editable_layout' } }),
  mesh({ x: -99, y: -99, z: -99 }, { x: 99, y: 99, z: 99 }, { selection_outline: true, item: { id: 'outline', role: 'helper' } })
]);
const robotRoot = group({ id: 'robot', role: 'robot', locked: true, source_layer: 'locked_generated_urdf_visual' }, [
  mesh({ x: 0, y: 0, z: 0 }, { x: 1, y: 1, z: 1 }, { item: { id: 'base_link', role: 'robot', active_visual_source: 'mesh_preview' } }),
  mesh({ x: 1, y: 0, z: 0 }, { x: 2, y: 1, z: 1 }, { item: { id: 'tool_link', role: 'tool', active_visual_source: 'mesh_preview' } })
]);
const helperRoot = group({ id: 'helper', role: 'helper', category: 'work_envelope' }, [mesh({ x: -50, y: -50, z: -50 }, { x: 50, y: 50, z: 50 }, { item: { id: 'work_envelope', role: 'helper', category: 'reachability' } })]);
state.three.scene = group({}, [tableRoot, robotRoot, helperRoot, mesh({ x: -5, y: -5, z: 0 }, { x: -4, y: -4, z: 1 }, { item: { id: 'bin', category: 'bin' } })]);
state.objects = [{ item: tableRoot.userData.item, object3d: tableRoot }, { item: robotRoot.userData.item, object3d: robotRoot }, { item: helperRoot.userData.item, object3d: helperRoot }];
selectObject('table');
const beforeSelectOnly = JSON.stringify(state.three.camera.position);
selectObject('robot');
assert.strictEqual(JSON.stringify(state.three.camera.position), beforeSelectOnly);
fitSelection();
assert.strictEqual(JSON.stringify(state.lastFrameBounds.min), JSON.stringify({ x: 0, y: 0, z: 0 }));
assert.strictEqual(JSON.stringify(state.lastFrameBounds.max), JSON.stringify({ x: 2, y: 1, z: 1 }));
assert.strictEqual(state.editorEvents.at(-1).type, 'fit_selection');
selectObject('table');
fitSelection();
assert.strictEqual(JSON.stringify(state.lastFrameBounds.min), JSON.stringify({ x: 10, y: 0, z: 0 }));
assert.strictEqual(JSON.stringify(state.lastFrameBounds.max), JSON.stringify({ x: 12, y: 2, z: 1 }));
clearSelection();
fitSelection();
assert.ok(state.editorEvents.some(event => event.type === 'fit_selection_fallback'));
assert.strictEqual(state.editorError, 'No physical item selected; fitting the workcell');
selectObject('helper');
assert.strictEqual(state.selected, '');
state.three.camera.position.copy(new MockVector3(3, 4, 5));
fitSelection();
assert.strictEqual(state.editorError, 'No physical item selected; fitting the workcell');
for (const value of [state.three.camera.position.x, state.three.camera.position.y, state.three.camera.position.z, state.three.camera.near, state.three.camera.far]) assert.strictEqual(Number.isFinite(value), true);
`, sandbox);
"""
    result = subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert result.stderr == ""

def test_initial_camera_fit_runs_once_per_stable_scene_and_respects_manual_camera():
    js_path = VIEWER / "viewer.js"
    harness = r'''
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
class MockVector3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
  copy(v) { this.x = v.x; this.y = v.y; this.z = v.z; return this; }
  addScaledVector(v, s) { this.x += v.x * s; this.y += v.y * s; this.z += v.z * s; return this; }
  normalize() { const l = Math.hypot(this.x, this.y, this.z) || 1; this.x /= l; this.y /= l; this.z /= l; return this; }
}
class MockSphere { constructor() { this.radius = 0; } }
class MockBox3 {
  constructor(min, max) {
    if (max) { this.min = { ...min }; this.max = { ...max }; }
    else { this.min = { x: Infinity, y: Infinity, z: Infinity }; this.max = { x: -Infinity, y: -Infinity, z: -Infinity }; }
  }
  isEmpty() { return this.min.x > this.max.x || this.min.y > this.max.y || this.min.z > this.max.z; }
  union(box) { for (const k of ['x','y','z']) { this.min[k] = Math.min(this.min[k], box.min[k]); this.max[k] = Math.max(this.max[k], box.max[k]); } return this; }
  setFromObject(object) { this.min = { ...object.mockBounds.min }; this.max = { ...object.mockBounds.max }; return this; }
  getSize(target) { target.x = this.max.x - this.min.x; target.y = this.max.y - this.min.y; target.z = this.max.z - this.min.z; return target; }
  getCenter(target) { target.x = (this.min.x + this.max.x) / 2; target.y = (this.min.y + this.max.y) / 2; target.z = (this.min.z + this.max.z) / 2; return target; }
  getBoundingSphere(sphere) { sphere.radius = Math.hypot(this.max.x - this.min.x, this.max.y - this.min.y, this.max.z - this.min.z) / 2; return sphere; }
  clone() { return new MockBox3(this.min, this.max); }
}
const element = () => ({ hidden: true, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', classList: { toggle() {} }, setAttribute() {}, querySelector() { return { textContent: '' }; }, appendChild() {}, addEventListener() {}, getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; } });
const timers = [];
const sandbox = { console, assert, MockVector3, MockSphere, MockBox3, timers, window: { location: { search: '' } }, document: { getElementById() { return element(); }, createElement() { return element(); } }, URLSearchParams, requestAnimationFrame() { return 0; }, setTimeout(fn) { timers.push(fn); return fn; }, clearTimeout(fn) { const index = timers.indexOf(fn); if (index >= 0) timers.splice(index, 1); } };
vm.createContext(sandbox);
vm.runInContext(source + `
THREE = { Vector3: MockVector3, Sphere: MockSphere, Box3: MockBox3 };
function root(children = []) { return { visible: true, children, userData: {}, updateWorldMatrix() {} }; }
function mesh(id, min, max) { return { isMesh: true, visible: true, children: [], name: id, mockBounds: { min, max }, userData: { item: { id, category: 'table' } } }; }
state.sceneJson = { schema_version: SUPPORTED_SCHEMA_VERSION, scene: { id: 'scene_a', root: '/cells/scene_a', generation_version: 'v1' } };
state.sourceWebSceneFile = 'build/workcell_studio_web_scene/scene_a.web_scene.json';
state.three.camera = { position: new MockVector3(9, 8, 7), near: 0.5, far: 50, updateProjectionMatrix() { this.updated = true; } };
state.three.controls = { target: new MockVector3(1, 1, 1), update() { this.updated = true; } };
state.three.scene = root([mesh('table', { x: 0, y: 0, z: 0 }, { x: 1, y: 1, z: 1 })]);
let fitCalls = 0;
const originalFrame = frameScene;
frameScene = bounds => { fitCalls += 1; return originalFrame(bounds); };
beginInitialCameraFitForCurrentScene();
assert.strictEqual(attemptInitialCameraFit(), true);
assert.strictEqual(fitCalls, 1);
const fitted = JSON.stringify(state.three.camera.position);
attemptInitialCameraFit(); refreshWarnings({ warnings: [] }); renderSceneSummary(); setDebugOverlaysVisible(false);
assert.strictEqual(fitCalls, 1);
assert.strictEqual(JSON.stringify(state.three.camera.position), fitted);
resetView();
assert.strictEqual(fitCalls, 2);
markCameraUserControlled();
state.initialCameraFit.done = false;
attemptInitialCameraFit();
assert.strictEqual(fitCalls, 2);
state.sceneJson = { schema_version: SUPPORTED_SCHEMA_VERSION, scene: { id: 'scene_b', root: '/cells/scene_b', generation_version: 'v1' } };
state.sourceWebSceneFile = 'build/workcell_studio_web_scene/scene_b.web_scene.json';
beginInitialCameraFitForCurrentScene();
state.three.scene = root([mesh('camera', { x: 2, y: 0, z: 0 }, { x: 3, y: 1, z: 1 })]);
assert.strictEqual(attemptInitialCameraFit(), true);
assert.strictEqual(fitCalls, 3);
for (const value of [state.three.camera.position.x, state.three.camera.position.y, state.three.camera.position.z, state.three.camera.near, state.three.camera.far, state.three.controls.target.x, state.three.controls.target.y, state.three.controls.target.z]) assert.strictEqual(Number.isFinite(value), true);
state.sceneJson = { schema_version: SUPPORTED_SCHEMA_VERSION, scene: { id: 'empty', root: '/cells/empty' } };
state.sourceWebSceneFile = 'build/workcell_studio_web_scene/empty.web_scene.json';
beginInitialCameraFitForCurrentScene();
state.three.scene = root([]);
attemptInitialCameraFit();
assert.strictEqual(timers.length, 1);
state.sceneJson = { schema_version: SUPPORTED_SCHEMA_VERSION, scene: { id: 'after_empty', root: '/cells/after_empty' } };
beginInitialCameraFitForCurrentScene();
assert.strictEqual(timers.length, 0);
attemptInitialCameraFit();
assert.strictEqual(timers.length, 1);
timers.shift()();
assert.strictEqual(state.editorError, 'No visible physical geometry available for initial framing');
`, sandbox);
'''
    result = subprocess.run(
        ["node", "-e", harness, str(js_path)],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert result.stderr == ""


def test_viewer_world_z_rotate_gizmo_transaction_contract():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    assert "directRotateDrag: null" in js
    assert "beginDirectRotateDrag(rendered)" in js
    assert "finishDirectRotateDrag(rendered)" in js
    assert "cancelDirectRotateDrag('Rotation cancelled')" in js
    assert "pushEditorEvent('status', { message: `Rotated ${itemLabel(rendered.item)}` })" in js
    assert "pushEditorEvent('status', { message: message || 'Rotation cancelled' })" in js
    assert "gizmo.setSpace('world')" in js
    assert "gizmo.showX = false; gizmo.showY = false; gizmo.showZ = true" in js
    assert "const next = cloneTransform(drag.start);" in js
    assert "next.pose.rpy.z = transformFromObject(rendered.object3d).pose.rpy.z" in js
    assert "snapTransform(next, { translationAxes: [], rotationAxes: ['z'] })" in js
    assert "markDirtyTransform(rendered, finalTransform, { pushHistory: true, oldTransform: drag.start, snapOptions: { translationAxes: [], rotationAxes: ['z'] }, memberStarts: drag.groupStart })" in js
    assert "if (sameTransform(drag.start, finalTransform))" in js


def test_linked_transform_group_moves_rotates_and_records_atomic_two_edit_patch():
    js_path = VIEWER / "viewer.js"
    harness = r"""
const fs = require('fs'); const vm = require('vm'); const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
const element = () => ({ hidden:false, checked:false, disabled:false, textContent:'', innerHTML:'', classList:{toggle(){}}, querySelector(){return null;}, addEventListener(){}, setAttribute(){}, appendChild(){}, remove(){} });
const context = { console, assert, window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}}}, document:{getElementById(){return element();},createElement(){return element();}}, URLSearchParams, CustomEvent:function(){}, requestAnimationFrame(){}, setTimeout(){}, clearTimeout(){} };
vm.createContext(context);
vm.runInContext(source + `
const object = (x,y,z,yaw=0) => ({ position:{x,y,z,set(a,b,c){this.x=a;this.y=b;this.z=c;}}, rotation:{x:0,y:0,z:yaw,set(a,b,c){this.x=a;this.y=b;this.z=c;}}, scale:{x:1,y:1,z:1,set(a,b,c){this.x=a;this.y=b;this.z=c;}} });
const rendered = (id,x,y,z,group='destination') => ({ item:{id,editable:true,locked:false,source_layer:'editable_layout',render_policy:'primary',transform_group:group}, object3d:object(x,y,z), originalTransform:null });
const bin=rendered('bin',1,2,0.2), zone=rendered('zone',1,2,0.1), free=rendered('free',4,5,0,'');
for (const row of [bin,zone,free]) row.originalTransform=transformFromObject(row.object3d);
state.objects=[bin,zone,free]; state.sceneJson={scene:{id:'group_test'}}; state.dirtyTransforms=new Map(); state.undoStack=[]; state.redoStack=[];
updateLabels=()=>{}; updateDirtyState=()=>{}; emitDirtyChanged=()=>{}; populateObjectList=()=>{};
let next=cloneTransform(bin.originalTransform); next.pose.xyz.x+=0.4; next.pose.xyz.y-=0.3;
assert.strictEqual(markDirtyTransform(bin,next),true); assert.strictEqual(zone.object3d.position.x,1.4); assert.strictEqual(zone.object3d.position.y,1.7); assert.ok(Math.abs((bin.object3d.position.z-zone.object3d.position.z)-0.1)<1e-12);
assert.strictEqual(state.undoStack.length,1); assert.strictEqual(state.undoStack[0].changes.length,2); assert.strictEqual(buildEditPatch().edits.length,2);
undoPreviewEdit(); assert.strictEqual(bin.object3d.position.x,1); assert.strictEqual(zone.object3d.position.x,1); redoPreviewEdit(); assert.strictEqual(bin.object3d.position.x,1.4); assert.strictEqual(zone.object3d.position.x,1.4);
next=transformFromObject(zone.object3d); next.pose.rpy.z=Math.PI/2; markDirtyTransform(zone,next);
assert.ok(Math.abs(bin.object3d.rotation.z-Math.PI/2)<1e-12); assert.ok(Math.abs(bin.object3d.position.z-zone.object3d.position.z-0.1)<1e-12);
let freeNext=cloneTransform(free.originalTransform); freeNext.pose.xyz.x=8; markDirtyTransform(free,freeNext); assert.strictEqual(bin.object3d.position.x,1.4);
`, context);
"""
    subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, capture_output=True, text=True)


def test_viewer_rotate_cancels_on_selection_mode_and_scene_change_without_yaml_writes():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    select_body = _viewer_function_body(js, "function selectObject(id)", "function clearSelection()")
    mode_body = _viewer_function_body(js, "function setEditorMode(mode)", "function setEditorSnap")
    load_file_body = _viewer_function_body(js, "async function loadFile(file)", "function safeRelativeSceneUrl")
    load_url_body = _viewer_function_body(js, "async function loadSceneUrl(rawUrl)", "if (el.resetView)")
    object_change_body = js.split("transformControls.addEventListener('objectChange'", 1)[1].split("controls.addEventListener('start'", 1)[0]
    assert "state.directRotateDrag && state.directRotateDrag.itemId !== selectionId" in select_body
    assert "cancelDirectRotateDrag('Rotation cancelled')" in select_body
    assert "if (state.editorMode !== normalized)" in mode_body
    assert "cancelDirectRotateDrag('Rotation cancelled')" in mode_body
    assert "cancelDirectRotateDrag('Rotation cancelled')" in load_file_body
    assert "cancelDirectRotateDrag('Rotation cancelled')" in load_url_body
    assert "previewDirectRotateDrag(rendered); return;" in object_change_body
    assert "state.undoStack.push" not in object_change_body
    assert "yaml" not in object_change_body.lower()
    assert "metadata" not in object_change_body.lower()



def test_expanded_urdf_tool_mesh_failure_keeps_status_scene_failed_not_shell_ready():
    js_path = VIEWER / "viewer.js"
    harness = r"""
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
const element = () => ({
  hidden: false,
  checked: false,
  disabled: false,
  textContent: '',
  className: '',
  innerHTML: '',
  classList: { toggle() {} },
  setAttribute() {},
  querySelector() { return { textContent: '' }; },
  appendChild() {},
  addEventListener() {},
  getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; },
});
const context = {
  console,
  assert,
  window: { location: { search: '' }, dispatchEvent() {}, parent: { postMessage() {} } },
  document: { getElementById() { return element(); }, createElement() { return element(); } },
  CustomEvent: function CustomEvent(type, init) { return { type, ...init }; },
  URLSearchParams,
  requestAnimationFrame() { return 0; },
};
vm.createContext(context);
vm.runInContext(source + `
populateObjectList = () => {};
updateLabels = () => {};
resetView = () => {};
renderSceneSummary = () => updateViewerStatus();
state.sourceWebSceneFile = 'build/workcell_studio_web_scene/ur5_2f_test.web_scene.json';
state.sceneJson = {
  scene: { id: 'ur5_2f_test' },
  robot_preview: { mode: 'expanded_urdf_loader', urdf_url: 'assets/ur5_2f_test/scene.urdf' },
  robots: [{ id: 'ur5', role: 'robot', category: 'robot', mesh_contract_category: 'robot' }],
  tools: [{ id: 'robotiq_2f', role: 'end_effector', category: 'tool', display_name: 'Robotiq 2F', mesh_contract_category: 'tool', mesh_load_required: true, mesh_uri: 'assets/ur5_2f_test/robotiq/robotiq_85_gripper_visual.dae' }],
  assets: [{ id: 'workbench', category: 'environment', mesh_contract_category: 'table', mesh_uri: 'assets/ur5_2f_test/table.stl' }],
  sensors: [{ id: 'camera', category: 'camera', mesh_contract_category: 'camera', mesh_uri: 'assets/ur5_2f_test/realsense.stl' }],
};
const items = collectItems(state.sceneJson);
beginWeb3dSceneReadiness(items);
failExpandedUrdfReadiness(new Error('HTTP 404 loading required Robotiq tool mesh'), {
  robot_urdf_url: 'assets/ur5_2f_test/scene.urdf',
  robot_missing_meshes: ['build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq/robotiq_85_gripper_visual.dae: HTTP 404'],
  robot_failed_visual_count: 1,
  robotFailedVisualCount: 1,
}, {
  required_category: 'attached_tool_gripper',
  link: 'gripper_base_link',
  url: 'build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq/robotiq_85_gripper_visual.dae',
  uri: 'package://robotiq_85_description/meshes/visual/robotiq_85_gripper_visual.dae',
  path: 'package://robotiq_85_description/meshes/visual/robotiq_85_gripper_visual.dae',
});
let status = updateViewerStatus();
assert.strictEqual(status.web3d_readiness_state, 'scene_failed');
assert.strictEqual(status.web3dReadinessState, 'scene_failed');
assert.strictEqual(status.viewer_boot_state, 'scene_failed');
assert.strictEqual(status.final_lifecycle_state, 'scene_failed');
assert.strictEqual(status.tool_status, 'failed');
assert.strictEqual(status.failed_required_item_count > 0, true);
assert.strictEqual(status.final_failed_url, 'build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq/robotiq_85_gripper_visual.dae');
assert.strictEqual(status.finalFailedUrl, status.final_failed_url);
assert.strictEqual(status.final_failed_link, 'gripper_base_link');
assert.strictEqual(status.readiness_failure.link, 'gripper_base_link');
assert.strictEqual(status.readiness_failure.url, status.final_failed_url);
assert.ok(status.readiness_failure.robot_missing_meshes[0].includes('robotiq_85_gripper_visual.dae'));
assert.ok(status.readiness_failure.reason.includes('Robotiq tool mesh'));
emitWeb3dReadinessState('server_ready', { http_status: 200 });
status = updateViewerStatus();
assert.strictEqual(status.web3d_readiness_state, 'scene_failed');
assert.strictEqual(status.web3dReadinessState, 'scene_failed');
assert.notStrictEqual(status.web3d_readiness_state, 'scene_ready');
assert.notStrictEqual(status.web3dReadinessState, 'scene_ready');
assert.strictEqual(status.readiness_failure.url, status.final_failed_url);
`, context);
"""
    result = subprocess.run(
        ["node", "-e", harness, str(js_path)],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert result.stderr == ""

def test_viewer_emits_explicit_web3d_readiness_states_and_required_categories():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    assert "'server_ready'" in js or "server_ready is infrastructure state" in (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
    assert "'scene_loading'" in js
    assert "'scene_ready'" in js
    assert "'scene_failed'" in js
    assert "const WEB3D_REQUIRED_CATEGORIES = ['robot_arm', 'attached_tool_gripper', 'workbench_support_surface', 'configured_camera']" in js
    assert "function beginWeb3dSceneReadiness(items)" in js
    assert "function maybeEmitSceneReady()" in js
    assert "if (readinessState === 'scene_ready')" in js
    assert "if (state.web3dReadiness.terminal)" in js
    assert "state.web3dReadiness.terminal" in js
    assert "pending.add('robot_arm:expanded_urdf_loader')" in js
    assert "pending.add('attached_tool_gripper:expanded_urdf_loader')" in js
    assert "emitWeb3dReadinessState('scene_loading'" in js
    assert "emitWeb3dReadinessState('scene_ready'" in js
    assert "scene_id: sceneId()" in js
    assert "expected_physical_item_count" in js
    assert "rendered_physical_item_count" in js
    assert "failed_required_item_count" in js
    assert "robot_status" in js
    assert "tool_status" in js
    assert "environment_status" in js
    assert "camera_status" in js
    assert "final_lifecycle_state" in js


def test_required_gripper_mesh_failures_transition_scene_failed_with_url_and_link_details():
    viewer = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    renderer = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    assert "function failWeb3dSceneReadiness(item, url, reason" in viewer
    assert "emitWeb3dReadinessState('scene_failed'" in viewer
    assert "url: url || displayMeshUri(item)" in viewer
    assert "link: item?.link || item?.link_name || item?.object_name || ''" in viewer
    assert "const eventDetail = { ...structured, ...detail, final_lifecycle_state: readinessState" in viewer
    assert "if (required) failWeb3dSceneReadiness(item, preflight.url || loadUrl" in viewer
    assert "http_status: preflight.http_status || null" in viewer
    assert "onRobotMeshLoadError: (err, uri, detail) => { if (!callbackIsCurrent()) return ignoreStaleCallback(); failExpandedUrdfReadiness" in viewer
    assert "function inferMeshLinkDetail(path)" in renderer
    assert "'gripper_base_link'" in renderer
    assert "context?.onRobotMeshLoadError?.(err, uri, { url, uri, path, source_url: path, sourceUrl: path, policy_reason:" in renderer
    assert "policyReason: err?.message || 'loader failure'" in renderer
    assert "diagnostics.robot_missing_meshes.push(`${url}:" in renderer


def test_gripper_mesh_404_scene_failed_payload_includes_structured_url_and_link_detail():
    viewer = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    preflight_body = _viewer_function_body(viewer, "async function preflightMeshUrl", "function supportSurfaceKindOf")
    try_load_body = _viewer_function_body(viewer, "async function tryLoadMesh", "function collectItems")
    failure_body = _viewer_function_body(viewer, "function failWeb3dSceneReadiness", "function completeExpandedUrdfReadiness")
    assert "HTTP ${response.status}" in preflight_body
    assert "http_status: response.status" in preflight_body
    assert "if (required) failWeb3dSceneReadiness(item, preflight.url || loadUrl" in try_load_body
    assert "http_status: preflight.http_status || null" in try_load_body
    assert "url: url || displayMeshUri(item)" in failure_body
    assert "link: item?.link || item?.link_name || item?.object_name || ''" in failure_body
    assert "required_category: category" in failure_body
    assert "scene_id: sceneId()" in viewer


def test_successful_required_loads_emit_scene_ready_exactly_once_after_completion():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    body = _viewer_function_body(js, "function emitWeb3dReadinessState", "function readinessCategoryForItem")
    maybe_body = _viewer_function_body(js, "function maybeEmitSceneReady", "function isGeneratedUrdfItem")
    assert "if (state.web3dReadiness.terminal)" in body
    assert "terminalEmissionCount" in body
    assert "readiness.pending?.size === 0" in maybe_body
    assert "emitWeb3dReadinessState('scene_ready'" in maybe_body
    assert "requiredReadinessCompleteForItem(item);" in js
    assert "completeExpandedUrdfReadiness(result);" in js



def test_urdf_renderer_configures_active_loader_package_resolver_before_loading():
    js = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    ready_body = js.split("result.ready = (async () => {", 1)[1].split("const urdfUrl =", 1)[0]
    assert ready_body.index("const loader = new URDFLoader(manager);") < ready_body.index("configureUrdfPackageResolution(loader, manager, rendererContext, diagnostics);")
    assert ready_body.index("configureUrdfPackageResolution(loader, manager, rendererContext, diagnostics);") < ready_body.index("loader.loadMeshCb =")
    load_body = js.split("export function loadRobotPreview", 1)[1]
    assert load_body.index("loader.loadMeshCb =") < load_body.index("fetchValidatedUrdfRobotElement(urdfUrl)")
    assert load_body.index("fetchValidatedUrdfRobotElement(urdfUrl)") < load_body.index("loader.parse(urdfRobotElement")
    assert "loader.loadAsync(urdfUrl)" not in js
    assert "loader.packages = resolver" in js
    assert "manager.setURLModifier(url =>" in js


def test_urdf_renderer_active_package_resolution_avoids_bare_package_root_requests():
    js = (VIEWER / "urdf_robot_renderer.js").read_text(encoding="utf-8")
    assert "${STAGED_MESH_ASSET_ROOT}${scene}/${packageName}" in js
    assert "${STAGED_MESH_ASSET_ROOT}${scene}/${packageName}/${safeParts.join('/')}" in js
    assert "bare package-root URL" in js
    expected_final_requests = [
        "/build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq_85_description/meshes/visual/robotiq_85_base_link.dae",
        "/build/workcell_studio_web_scene/assets/ur5_2f_test/ur_description/meshes/ur5/visual/base.dae",
    ]
    assert expected_final_requests[0].startswith("/build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq_85_description/")
    assert expected_final_requests[1].startswith("/build/workcell_studio_web_scene/assets/ur5_2f_test/ur_description/")
    for request in expected_final_requests:
        assert not request.startswith(("/robotiq_85_description/", "/ur_description/"))

def test_viewer_uses_expanded_urdf_expected_visual_metadata_not_ur5_2f_constants():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    assert "CANONICAL_UR5_2F_REQUIRED" not in js
    assert "expandedUrdfExpectedVisualSet" in js
    assert "expected_robot_visual_links" in js
    assert "expected_tool_visual_links" in js
    assert "missing_required_robot_visuals" in js
    assert "missing_required_tool_visuals" in js
    assert "expanded URDF expected robot/tool visuals are missing or failed" in js
    assert "expanded URDF renderer reported required robot/tool visual failure" in js
    readiness_body = js.split("function failExpandedUrdfReadiness", 1)[1].split("function maybeEmitSceneReady", 1)[0]
    assert "attached_tool_gripper" in readiness_body
    assert "expected_tool_visual_links" in readiness_body


def test_expanded_urdf_mode_marks_flattened_robot_tool_rows_diagnostic_only():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    render_body = js.split("function renderScene(items)", 1)[1].split("function loadExpandedUrdfRobotPreview", 1)[0]
    assert "if (urdfPreviewActive) loadExpandedUrdfRobotPreview(state.sceneJson.robot_preview);" in render_body
    assert "new Set(robotToolGeneratedUrdfItems)" in render_body
    assert "expanded_urdf_loader_skip_flattened_row" in render_body
    assert "expanded URDF mode renders robot/tool only through loadRobotPreview and URDFLoader" in render_body
    assert render_body.index("loadExpandedUrdfRobotPreview(state.sceneJson.robot_preview)") < render_body.index("for (const item of items)")


def test_urdf_renderer_compares_browser_matrix_world_to_exported_expected_matrices(tmp_path):
    script = tmp_path / "matrix_parity.mjs"
    script.write_text(
        f"""
import assert from 'node:assert/strict';
import * as THREE from {str((VIEWER / 'node_modules/three/build/three.module.js')).__repr__()};
import URDFLoader from {str((VIEWER / 'node_modules/urdf-loader/src/URDFLoader.js')).__repr__()};

globalThis.window = {{}};
globalThis.fetch = async () => ({{ ok: true, status: 200, statusText: 'OK', text: async () => '<robot name="test"/>' }});
const originalUrdfParse = URDFLoader.prototype.parse;
URDFLoader.prototype.parse = function parseStub() {{
  const robot = new THREE.Group();
  const base = new THREE.Group();
  base.name = 'base_link';
  base.position.set(1, 2, 3);
  const visual = new THREE.Group();
  visual.name = 'visual_0';
  visual.position.set(0.25, 0, 0);
  visual.add(new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshBasicMaterial()));
  base.add(visual);
  robot.add(base);
  robot.links = {{ base_link: base }};
  robot.joints = {{}};
  robot.setJointValues = () => {{}};
  return robot;
}};

try {{
  const {{ loadRobotPreview }} = await import({str((VIEWER / 'urdf_robot_renderer.js')).__repr__()});
  const expectedLink = new THREE.Matrix4().makeTranslation(1, 2, 3).elements;
  const expectedVisual = new THREE.Matrix4().makeTranslation(1.25, 2, 3).elements;
  const result = loadRobotPreview({{
    urdf_url: 'robot.urdf',
    expected_robot_link_world_matrices: {{ base_link: expectedLink }},
    expected_robot_visual_wrapper_world_matrices: {{ base_link: expectedVisual }},
  }});
  await result.ready;
  assert.equal(result.diagnostics.robot_matrix_world_parity.pass, true);
  assert.equal(result.diagnostics.robot_matrix_world_parity.tolerance, 1e-5);
  assert.equal(result.diagnostics.robot_matrix_world_parity.link.compared_count, 1);
  assert.equal(result.diagnostics.robot_matrix_world_parity.visual_wrapper.compared_count, 1);
  assert.equal(result.diagnostics.robot_matrix_world_parity.link.comparisons[0].max_abs_delta <= 1e-5, true);
  assert.equal(result.diagnostics.robot_matrix_world_parity.visual_wrapper.comparisons[0].max_abs_delta <= 1e-5, true);
}} finally {{
  URDFLoader.prototype.parse = originalUrdfParse;
}}
""",
        encoding="utf-8",
    )
    subprocess.run(["node", str(script)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)



def test_urdf_renderer_detects_expanded_visual_wrappers_with_deep_descendant_meshes(tmp_path):
    script = tmp_path / "deep_visual_wrapper.mjs"
    script.write_text(
        f"""
import assert from 'node:assert/strict';
import * as THREE from {str((VIEWER / 'node_modules/three/build/three.module.js')).__repr__()};
import URDFLoader from {str((VIEWER / 'node_modules/urdf-loader/src/URDFLoader.js')).__repr__()};

globalThis.window = {{}};
globalThis.fetch = async () => ({{ ok: true, status: 200, statusText: 'OK', text: async () => '<robot name="test"/>' }});
const originalUrdfParse = URDFLoader.prototype.parse;
URDFLoader.prototype.parse = function parseStub() {{
  const robot = new THREE.Group();
  const base = new THREE.Group();
  base.name = 'base_link';
  const tool = new THREE.Group();
  tool.name = 'robotiq_85_base_link';

  for (const link of [base, tool]) {{
    const wrapper = new THREE.Object3D();
    const importedScene = new THREE.Scene();
    const nestedGroup = new THREE.Object3D();
    nestedGroup.add(new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshBasicMaterial()));
    importedScene.add(nestedGroup);
    wrapper.add(importedScene);
    link.add(wrapper);
    robot.add(link);
  }}

  robot.links = {{ base_link: base, robotiq_85_base_link: tool }};
  robot.joints = {{}};
  robot.setJointValues = () => {{}};
  return robot;
}};

try {{
  const {{ loadRobotPreview }} = await import({str((VIEWER / 'urdf_robot_renderer.js')).__repr__()});
  const result = loadRobotPreview({{
    urdf_url: 'robot.urdf',
    expected_robot_visual_links: ['base_link'],
    expected_tool_visual_links: ['robotiq_85_base_link'],
  }});
  await result.ready;
  assert.equal(result.diagnostics.robot_preview_loaded, true);
  assert.equal(result.diagnostics.robot_preview_lifecycle_state, 'ready');
  assert.deepEqual(result.diagnostics.robot_missing_required_robot_visual_links, []);
  assert.deepEqual(result.diagnostics.robot_missing_required_tool_visual_links, []);
  assert.deepEqual(
    result.diagnostics.robot_visual_wrapper_world_matrices.map(entry => entry.link_name).sort(),
    ['base_link', 'robotiq_85_base_link']
  );
  assert.equal(result.diagnostics.robot_descendant_render_mesh_diagnostics.length, 2);
}} finally {{
  URDFLoader.prototype.parse = originalUrdfParse;
}}
""",
        encoding="utf-8",
    )
    subprocess.run(["node", str(script)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def test_viewer_browser_readiness_contract_lifecycle_and_terminal_dedup():
    js_path = VIEWER / "viewer.js"
    harness = """
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\\(\\);\\s*$/, '');
const element = () => ({ hidden: false, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', classList: { toggle() {} }, setAttribute() {}, querySelector() { return { textContent: '' }; }, appendChild() {}, addEventListener() {}, getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; } });
const context = { console, assert, window: { dispatched: [], location: { search: '' }, dispatchEvent(event) { this.dispatched.push(event?.detail?.state); }, parent: { postMessage() {} } }, document: { getElementById() { return element(); }, createElement() { return element(); } }, URLSearchParams, CustomEvent: function CustomEvent(type, init) { return { type, detail: init?.detail || {} }; }, requestAnimationFrame() { return 0; }, setTimeout() { return 1; }, clearTimeout() {} };
vm.createContext(context);
vm.runInContext(source + `
assert.strictEqual(state.web3dReadiness.state, 'booting');
state.sourceWebSceneFile = 'build/workcell_studio_web_scene/ur5_2f_test.web_scene.json';
state.builderRevision = '42';
emitWeb3dReadinessState('scene_loading');
state.sceneJson = { scene: { id: 'ur5_2f_test' }, objects: [
 { id: 'robot', category: 'robot', mesh_uri: 'robot.stl' },
 { id: 'tool', category: 'tool', mesh_uri: 'tool.stl' },
 { id: 'table', category: 'table' },
 { id: 'camera', category: 'camera' }
] };
state.sceneJsonLoaded = true;
failIfCanonicalRequiredVisualSetInvalid = () => false;
beginWeb3dSceneReadiness(collectItems(state.sceneJson));
for (const item of collectItems(state.sceneJson)) requiredReadinessCompleteForItem(item);
let status = window.__WORKCELL_VIEWER_STATUS__;
assert.strictEqual(status.readiness_contract_version, 1);
assert.strictEqual(status.lifecycle_state, 'scene_ready');
assert.strictEqual(status.terminal, true);
assert.strictEqual(status.scene_id, 'ur5_2f_test');
assert.strictEqual(status.source_web_scene_file, state.sourceWebSceneFile);
assert.strictEqual(status.builder_revision, '42');
assert.ok(status.expected_physical_item_count > 0);
const seq = status.status_sequence;
emitWeb3dReadinessState('scene_loading', { infrastructure: 'server_ready' });
assert.strictEqual(window.__WORKCELL_VIEWER_STATUS__.lifecycle_state, 'scene_ready');
emitWeb3dReadinessState('scene_ready');
assert.strictEqual(window.__WORKCELL_VIEWER_STATUS__.status_sequence, seq);
assert.strictEqual(state.web3dReadiness.terminalEmissionCount, 1);
`, context);
"""
    subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def test_viewer_required_failure_contract_is_terminal_and_deduplicated():
    js_path = VIEWER / "viewer.js"
    harness = """
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\\(\\);\\s*$/, '');
const element = () => ({ hidden: false, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', classList: { toggle() {} }, setAttribute() {}, querySelector() { return { textContent: '' }; }, appendChild() {}, addEventListener() {}, getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; } });
const context = { console, assert, window: { dispatched: [], location: { search: '' }, dispatchEvent(event) { this.dispatched.push(event?.detail?.state); }, parent: { postMessage() {} } }, document: { getElementById() { return element(); }, createElement() { return element(); } }, URLSearchParams, CustomEvent: function CustomEvent(type, init) { return { type, detail: init?.detail || {} }; }, requestAnimationFrame() { return 0; }, setTimeout() { return 1; }, clearTimeout() {} };
vm.createContext(context);
vm.runInContext(source + `
state.sourceWebSceneFile = 'build/workcell_studio_web_scene/ur5_2f_test.web_scene.json';
state.builderRevision = '7';
state.sceneJson = { scene: { id: 'ur5_2f_test' }, objects: [{ id: 'robot', category: 'robot', mesh_uri: 'missing.stl', link: 'base_link' }] };
state.sceneJsonLoaded = true;
beginWeb3dSceneReadiness(collectItems(state.sceneJson));
const robot = collectItems(state.sceneJson)[0];
failWeb3dSceneReadiness(robot, 'missing.stl', 'mesh failed');
let status = window.__WORKCELL_VIEWER_STATUS__;
assert.strictEqual(status.lifecycle_state, 'scene_failed');
assert.strictEqual(status.terminal, true);
assert.strictEqual(status.readiness_failure.required_category, 'robot_arm');
assert.strictEqual(status.readiness_failure.link, 'base_link');
assert.strictEqual(status.readiness_failure.url, 'missing.stl');
const seq = status.status_sequence;
emitWeb3dReadinessState('scene_ready');
failWeb3dSceneReadiness(robot, 'missing.stl', 'mesh failed again');
assert.strictEqual(window.__WORKCELL_VIEWER_STATUS__.lifecycle_state, 'scene_failed');
assert.strictEqual(window.__WORKCELL_VIEWER_STATUS__.status_sequence, seq);
assert.strictEqual(state.web3dReadiness.terminalEmissionCount, 1);
`, context);
"""
    subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def test_viewer_render_policy_excludes_diagnostics_from_readiness_and_editing():
    js_path = VIEWER / "viewer.js"
    harness = """
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
const element = () => ({ hidden: false, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', classList: { toggle() {} }, setAttribute() {}, querySelector() { return { textContent: '' }; }, appendChild() {}, addEventListener() {}, getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; } });
const context = { console, assert, window: { dispatched: [], location: { search: '' }, dispatchEvent(event) { this.dispatched.push(event?.detail?.state); }, parent: { postMessage() {} } }, document: { getElementById() { return element(); }, createElement() { return element(); } }, URLSearchParams, CustomEvent: function CustomEvent(type, init) { return { type, detail: init?.detail || {} }; }, requestAnimationFrame() { return 0; }, setTimeout() { return 1; }, clearTimeout() {} };
vm.createContext(context);
vm.runInContext(source + `
state.sceneJson = { scene: { id: 'policy_scene' }, assets: [
  { id: 'table', category: 'table', render_policy: 'primary', render_owner: 'editable_layout', render_identity: 'scene|layout|table', readiness_category: 'workbench_support_surface', editable: true },
  { id: 'pick_zone', category: 'pick_zone', render_policy: 'overlay', render_owner: 'task_overlay', render_identity: 'scene|overlay|pick' },
  { id: 'base_link_flattened', category: 'robot_static_mesh_visual', role: 'robot', mesh_uri: 'robot.dae', render_policy: 'diagnostic_only', render_owner: 'expanded_urdf_robot', render_identity: 'scene|robot|base_link|0' }
] };
const items = collectItems(state.sceneJson);
assert.strictEqual(items.length, 3);
assert.strictEqual(readinessCategoryForItem(items.find(i => i.id === 'table')), 'workbench_support_surface');
assert.strictEqual(readinessCategoryForItem(items.find(i => i.id === 'base_link_flattened')), '');
assert.strictEqual(readinessCategoryForItem(items.find(i => i.id === 'pick_zone')), '');
assert.strictEqual(isDebugOverlayItem(items.find(i => i.id === 'pick_zone')), true);
assert.strictEqual(canEditItem(items.find(i => i.id === 'table')), true);
assert.strictEqual(canEditItem(items.find(i => i.id === 'base_link_flattened')), false);
beginWeb3dSceneReadiness(items);
assert.deepStrictEqual(Array.from(state.web3dReadiness.pending), []);
assert.strictEqual(state.web3dReadiness.required.workbench_support_surface, true);
assert.strictEqual(state.web3dReadiness.required.robot_arm, false);
assert.strictEqual(physicalReadinessItems().length, 1);
`, context);
"""
    subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def test_staged_mesh_urls_resolve_from_expanded_urdf_without_doubling(tmp_path):
    script = tmp_path / "browser_url_resolution.mjs"
    script.write_text(
        """
import assert from 'node:assert/strict';
const base = 'http://127.0.0.1:8765/build/workcell_studio_web_scene/expanded_robot.urdf';
const cases = [
  '/build/workcell_studio_web_scene/assets/ur5_2f_test/ur_description/meshes/ur5/visual/base.dae',
  '/build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq_85_description/meshes/visual/robotiq_85_base_link.dae',
];
for (const url of cases) {
  const href = new URL(url, base).href;
  assert.equal(href, `http://127.0.0.1:8765${url}`);
  assert.equal((href.match(/build\/workcell_studio_web_scene/g) || []).length, 1, href);
  assert.ok(!href.includes('/build/workcell_studio_web_scene/build/workcell_studio_web_scene/'), href);
  assert.ok(!href.replace('http://127.0.0.1:8765', '').includes('//build/workcell_studio_web_scene/assets'), href);
}
""",
        encoding="utf-8",
    )
    subprocess.run(["node", str(script)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)




def test_load_robot_preview_preserves_canonical_mesh_urls_after_normalize_and_repo_url(tmp_path):
    script = tmp_path / "capture_final_staged_mesh_urls.mjs"
    script.write_text(
        f"""
import assert from 'node:assert/strict';
import * as THREE from {str((VIEWER / 'node_modules/three/build/three.module.js')).__repr__()};
import URDFLoader from {str((VIEWER / 'node_modules/urdf-loader/src/URDFLoader.js')).__repr__()};
import {{ ColladaLoader }} from {str((VIEWER / 'node_modules/three/examples/jsm/loaders/ColladaLoader.js')).__repr__()};

globalThis.window = {{}};
const requestedUrls = [];
const repoRootRelativeCalls = [];
globalThis.fetch = async () => ({{ ok: true, status: 200, statusText: 'OK', text: async () => '<robot name="test"/>' }});
const originalUrdfParse = URDFLoader.prototype.parse;
const originalColladaLoad = ColladaLoader.prototype.load;

URDFLoader.prototype.parse = function parseStub() {{
  for (const path of [
    'package://ur_description/meshes/ur5/visual/base.dae',
    'package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae',
  ]) {{
    this.loadMeshCb(path, this.manager, new THREE.MeshPhongMaterial(), (mesh, error) => {{ if (error) throw error; }});
  }}
  const robot = new THREE.Group();
  robot.links = {{ base_link: new THREE.Group(), gripper_base_link: new THREE.Group() }};
  robot.joints = {{}};
  robot.setJointValues = () => {{}};
  return robot;
}};

ColladaLoader.prototype.load = function loadStub(url, onLoad) {{
  requestedUrls.push(url);
  this.manager?.itemStart?.(url);
  onLoad({{ scene: new THREE.Group(), asset: {{}} }});
  this.manager?.itemEnd?.(url);
}};

try {{
  const {{ loadRobotPreview }} = await import({str((VIEWER / 'urdf_robot_renderer.js')).__repr__()});
  const result = loadRobotPreview(
    {{ urdf_url: '/build/workcell_studio_web_scene/assets/ur5_2f_test/robot.urdf' }},
    {{
      sceneId: 'ur5_2f_test',
      repoRootRelativeUrl: (value) => {{
        repoRootRelativeCalls.push(value);
        return `/build/workcell_studio_web_scene/${{value}}`;
      }},
    }}
  );
  await result.ready;
  assert.deepEqual(requestedUrls, [
    '/build/workcell_studio_web_scene/assets/ur5_2f_test/ur_description/meshes/ur5/visual/base.dae',
    '/build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq_85_description/meshes/visual/robotiq_85_base_link.dae',
  ]);
  assert.deepEqual(repoRootRelativeCalls, []);
  for (const url of requestedUrls) {{
    assert.equal((url.match(/build\/workcell_studio_web_scene/g) || []).length, 1, url);
    assert.equal(url.includes('//build/workcell_studio_web_scene/assets'), false, url);
    assert.equal(url.includes('/build/workcell_studio_web_scene//build/workcell_studio_web_scene/'), false, url);
  }}
}} finally {{
  URDFLoader.prototype.parse = originalUrdfParse;
  ColladaLoader.prototype.load = originalColladaLoad;
}}
""",
        encoding="utf-8",
    )
    subprocess.run(["node", str(script)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def test_urdf_renderer_accepts_only_canonical_staged_mesh_urls(tmp_path):
    script = tmp_path / "canonical_staged_mesh_policy.mjs"
    script.write_text(
        f"""
import assert from 'node:assert/strict';
const {{ canonicalStagedMeshUrl }} = await import({str((VIEWER / 'urdf_robot_renderer.js')).__repr__()});
const diagnostics = {{ robot_missing_meshes: [], robot_package_mesh_resolutions: [] }};
const context = {{ sceneId: 'ur5_2f_test' }};
assert.equal(
  canonicalStagedMeshUrl('/build/workcell_studio_web_scene/assets/ur5_2f_test/ur_description/meshes/visual/base.dae', context, diagnostics).uri,
  '/build/workcell_studio_web_scene/assets/ur5_2f_test/ur_description/meshes/visual/base.dae'
);
assert.equal(
  canonicalStagedMeshUrl('build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq_85_description/meshes/visual/finger.stl', context, diagnostics).uri,
  '/build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq_85_description/meshes/visual/finger.stl'
);
for (const rejected of [
  '/ur_description/meshes/visual/base.dae',
  'https://example.invalid/mesh.dae',
  'file:///tmp/mesh.dae',
  '/tmp/mesh.dae',
  'C:/tmp/mesh.dae',
  '/build/workcell_studio_web_scene/assets/ur5_2f_test/ur_description/../secret.dae',
  '/build/workcell_studio_web_scene/assets/ur5_2f_test/ur_description/%2e%2e/secret.dae',
  String.raw`/build/workcell_studio_web_scene/assets/ur5_2f_test/ur_description/meshes\\visual\\base.dae`,
  '/build/workcell_studio_web_scene/assets/ur5_3f_test/ur_description/meshes/visual/base.dae',
  '/build/workcell_studio_web_scene/assets/ur5_2f_test/../meshes/visual/base.dae',
]) {{
  const result = canonicalStagedMeshUrl(rejected, context, diagnostics);
  assert.equal(result.uri, '', rejected);
  assert.ok(result.reason, rejected);
}}
""",
        encoding="utf-8",
    )
    subprocess.run(["node", str(script)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def test_load_robot_preview_accepts_canonical_leading_slash_and_relative_staged_urls(tmp_path):
    script = tmp_path / "capture_canonical_staged_mesh_requests.mjs"
    script.write_text(
        f"""
import assert from 'node:assert/strict';
import * as THREE from {str((VIEWER / 'node_modules/three/build/three.module.js')).__repr__()};
import URDFLoader from {str((VIEWER / 'node_modules/urdf-loader/src/URDFLoader.js')).__repr__()};
import {{ ColladaLoader }} from {str((VIEWER / 'node_modules/three/examples/jsm/loaders/ColladaLoader.js')).__repr__()};

globalThis.window = {{}};
const requestedUrls = [];
globalThis.fetch = async () => ({{ ok: true, status: 200, statusText: 'OK', text: async () => '<robot name="test"/>' }});
const originalUrdfParse = URDFLoader.prototype.parse;
const originalColladaLoad = ColladaLoader.prototype.load;
URDFLoader.prototype.parse = function parseStub() {{
  for (const path of [
    '/build/workcell_studio_web_scene/assets/ur5_2f_test/ur_description/meshes/visual/base.dae',
    'build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq_85_description/meshes/visual/gripper.dae',
    'package://ur_description/meshes/ur5/visual/shoulder.dae',
    'package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae',
  ]) {{
    this.loadMeshCb(path, this.manager, new THREE.MeshPhongMaterial(), (mesh, error) => {{ if (error) throw error; }});
  }}
  const robot = new THREE.Group();
  robot.links = {{ base_link: new THREE.Group(), gripper_base_link: new THREE.Group() }};
  robot.joints = {{}};
  robot.setJointValues = () => {{}};
  return robot;
}};
ColladaLoader.prototype.load = function loadStub(url, onLoad) {{
  requestedUrls.push(url);
  this.manager?.itemStart?.(url);
  onLoad({{ scene: new THREE.Group(), asset: {{}} }});
  this.manager?.itemEnd?.(url);
}};
try {{
  const {{ loadRobotPreview }} = await import({str((VIEWER / 'urdf_robot_renderer.js')).__repr__()});
  const result = loadRobotPreview({{ urdf_url: 'robot.urdf' }}, {{ sceneId: 'ur5_2f_test' }});
  await result.ready;
  assert.deepEqual(requestedUrls, [
    '/build/workcell_studio_web_scene/assets/ur5_2f_test/ur_description/meshes/visual/base.dae',
    '/build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq_85_description/meshes/visual/gripper.dae',
    '/build/workcell_studio_web_scene/assets/ur5_2f_test/ur_description/meshes/ur5/visual/shoulder.dae',
    '/build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq_85_description/meshes/visual/robotiq_85_base_link.dae',
  ]);
  for (const requested of requestedUrls) assert.ok(requested.startsWith('/build/workcell_studio_web_scene/assets/'), requested);
  assert.equal(result.diagnostics.robot_failed_visual_count, 0);
}} finally {{
  URDFLoader.prototype.parse = originalUrdfParse;
  ColladaLoader.prototype.load = originalColladaLoad;
}}
""",
        encoding="utf-8",
    )
    subprocess.run(["node", str(script)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def test_expanded_urdf_readiness_allows_legitimate_multi_visual_links_and_equivalent_records():
    js_path = VIEWER / "viewer.js"
    harness = r"""
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
const element = () => ({ hidden: false, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', classList: { toggle() {} }, setAttribute() {}, querySelector() { return { textContent: '' }; }, appendChild() {}, addEventListener() {}, getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; } });
const context = { console, assert, window: { dispatched: [], location: { search: '' }, dispatchEvent(event) { this.dispatched.push(event?.detail?.state); }, parent: { postMessage() {} } }, document: { getElementById() { return element(); }, createElement() { return element(); } }, URLSearchParams, CustomEvent: function CustomEvent(type, init) { return { type, detail: init?.detail || {} }; }, requestAnimationFrame() { return 0; }, setTimeout() { return 1; }, clearTimeout() {} };
vm.createContext(context);
vm.runInContext(source + `
state.sceneJson = { scene: { id: 'ur5_2f_test' }, robot_preview: { mode: 'expanded_urdf_loader', robot_instance_id: 'ur5', expected_robot_visual_links: ['base_link', 'shoulder_link'], expected_tool_visual_links: ['robotiq_85_base_link'] } };
state.robotUrdfPreviewDiagnostics = { robot_preview_lifecycle_state: 'ready', robot_preview_loaded: true, robot_failed_visual_count: 0, robot_visual_wrapper_world_matrices: [
  { link_name: 'base_link', visual_index: 0, object_name: 'visual_0' },
  { link_name: 'base_link', visual_index: 0, object_name: 'visual_0_duplicate_wrapper' },
  { link_name: 'base_link', visual_index: 1, object_name: 'visual_1' },
  { link_name: 'shoulder_link', visual_index: 0, object_name: 'visual_0' },
  { link_name: 'robotiq_85_base_link', visual_index: 0, object_name: 'visual_0' },
  { link_name: 'robotiq_85_base_link', visual_index: 1, object_name: 'visual_1' },
] };
collectRenderedMeshDiagnostics = () => [
  { id: 'table_generated', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
  { id: 'table_semantic_equivalent', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
  { id: 'camera_generated', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
  { id: 'camera_semantic_equivalent', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
];
const diagnostics = expandedUrdfVisualReadinessDiagnostics();
assert.strictEqual(diagnostics.required_visual_ready, true);
assert.deepStrictEqual(diagnostics.missing_required_visuals, []);
assert.ok(diagnostics.duplicate_physical_visual_identities.length > 0);
state.web3dReadiness = { state: 'scene_loading', emittedSceneReady: false, required: { robot_arm: true, attached_tool_gripper: true, workbench_support_surface: true, configured_camera: true }, pending: new Set(['robot_arm:expanded_urdf_loader', 'attached_tool_gripper:expanded_urdf_loader']), failed: false, failure: null };
assert.strictEqual(failIfExpandedUrdfExpectedVisualSetInvalid(), false);
completeExpandedUrdfReadiness({ diagnostics: state.robotUrdfPreviewDiagnostics });
assert.strictEqual(state.web3dReadiness.state, 'scene_ready');
`, context);
"""
    subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)



def test_expanded_urdf_successful_16_of_16_callback_accounting_is_authoritative():
    js_path = VIEWER / "viewer.js"
    harness = r"""
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
const element = () => ({ hidden: false, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', classList: { toggle() {} }, setAttribute() {}, querySelector() { return { textContent: '' }; }, appendChild() {}, addEventListener() {}, getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; } });
const context = { console, assert, window: { dispatched: [], location: { search: '' }, dispatchEvent(event) { this.dispatched.push(event?.detail?.state); }, parent: { postMessage() {} } }, document: { getElementById() { return element(); }, createElement() { return element(); } }, URLSearchParams, CustomEvent: function CustomEvent(type, init) { return { type, detail: init?.detail || {} }; }, requestAnimationFrame() { return 0; }, setTimeout() { return 1; }, clearTimeout() {} };
vm.createContext(context);
vm.runInContext(source + `
const falseMissingRobotLinks = ['base_link_inertia', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link'];
state.sceneJson = { scene: { id: 'ur5_2f_test' }, robot_preview: { mode: 'expanded_urdf_loader', robot_instance_id: 'ur5_2f', expected_robot_visual_links: falseMissingRobotLinks, expected_tool_visual_links: ['robotiq_85_base_link'] } };
state.robotUrdfPreviewDiagnostics = {
  robot_preview_lifecycle_state: 'failed',
  robot_preview_loaded: false,
  robot_expected_visual_count: 16,
  robot_completed_visual_count: 16,
  robot_loaded_visual_count: 16,
  robot_failed_visual_count: 0,
  robot_mesh_callbacks_complete: true,
  robot_missing_meshes: [],
  robot_missing_required_robot_visual_links: falseMissingRobotLinks,
  robot_missing_required_tool_visual_links: [],
  robot_visual_wrapper_world_matrices: []
};
collectRenderedMeshDiagnostics = () => [
  { id: 'workbench_generated', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
  { id: 'configured_camera_generated', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
];
const diagnostics = expandedUrdfVisualReadinessDiagnostics();
assert.strictEqual(diagnostics.robot_expected_visual_count, 16);
assert.strictEqual(diagnostics.robot_completed_visual_count, 16);
assert.strictEqual(diagnostics.robot_loaded_visual_count, 16);
assert.strictEqual(diagnostics.robot_failed_visual_count, 0);
assert.strictEqual(diagnostics.robot_mesh_callbacks_complete, true);
assert.strictEqual(diagnostics.robot_successful_callback_accounting, true);
assert.deepStrictEqual(diagnostics.robot_missing_meshes, []);
assert.deepStrictEqual(diagnostics.missing_required_visuals, []);
assert.strictEqual(diagnostics.expanded_urdf_required_visual_counts.shoulder_link, undefined);
state.web3dReadiness = { state: 'scene_loading', emittedSceneReady: false, required: { robot_arm: true, attached_tool_gripper: true, workbench_support_surface: true, configured_camera: true }, pending: new Set(['robot_arm:expanded_urdf_loader', 'attached_tool_gripper:expanded_urdf_loader']), failed: false, failure: null };
assert.strictEqual(failIfExpandedUrdfExpectedVisualSetInvalid(), false);
completeExpandedUrdfReadiness({ diagnostics: state.robotUrdfPreviewDiagnostics });
assert.strictEqual(state.web3dReadiness.state, 'scene_ready');
assert.ok(window.dispatched.includes('scene_ready'));
assert.ok(!window.dispatched.includes('scene_failed'));
`, context);
"""
    subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def test_expanded_urdf_readiness_fails_missing_required_link_and_required_mesh_failure():
    js_path = VIEWER / "viewer.js"
    harness = r"""
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
const element = () => ({ hidden: false, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', classList: { toggle() {} }, setAttribute() {}, querySelector() { return { textContent: '' }; }, appendChild() {}, addEventListener() {}, getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; } });
const context = { console, assert, window: { dispatched: [], location: { search: '' }, dispatchEvent(event) { this.dispatched.push(event?.detail?.state); }, parent: { postMessage() {} } }, document: { getElementById() { return element(); }, createElement() { return element(); } }, URLSearchParams, CustomEvent: function CustomEvent(type, init) { return { type, detail: init?.detail || {} }; }, requestAnimationFrame() { return 0; }, setTimeout() { return 1; }, clearTimeout() {} };
vm.createContext(context);
vm.runInContext(source + `
state.sceneJson = { scene: { id: 'ur5_2f_test' }, robot_preview: { mode: 'expanded_urdf_loader', expected_robot_visual_links: ['base_link', 'shoulder_link'], expected_tool_visual_links: ['robotiq_85_base_link'] } };
state.robotUrdfPreviewDiagnostics = { robot_preview_lifecycle_state: 'failed', robot_preview_loaded: false, robot_failed_visual_count: 0, robot_missing_required_robot_visual_links: ['shoulder_link'], robot_missing_required_tool_visual_links: [], robot_visual_wrapper_world_matrices: [
  { link_name: 'base_link', visual_index: 0 },
  { link_name: 'robotiq_85_base_link', visual_index: 0 },
] };
collectRenderedMeshDiagnostics = () => [
  { id: 'table', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
  { id: 'camera', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
];
let diagnostics = expandedUrdfVisualReadinessDiagnostics();
assert.deepStrictEqual(diagnostics.missing_required_robot_visuals, ['shoulder_link']);
assert.strictEqual(diagnostics.required_visual_ready, false);
assert.strictEqual(failIfExpandedUrdfExpectedVisualSetInvalid(), true);
assert.strictEqual(state.web3dReadiness.state, 'scene_failed');
assert.ok(state.web3dReadiness.failure.missing_required_robot_visuals.includes('shoulder_link'));
state.web3dReadiness = null;
state.robotUrdfPreviewDiagnostics.robot_visual_wrapper_world_matrices.push({ link_name: 'shoulder_link', visual_index: 0 });
state.robotUrdfPreviewDiagnostics.robot_failed_visual_count = 1;
state.robotUrdfPreviewDiagnostics.robot_missing_meshes = ['/assets/robotiq_85_base_link.dae: 404'];
diagnostics = expandedUrdfVisualReadinessDiagnostics();
assert.strictEqual(diagnostics.required_visual_ready, false);
assert.ok(diagnostics.failed_required_links.includes('expanded_urdf_loader'));
assert.ok(diagnostics.failed_mesh_urls.includes('/assets/robotiq_85_base_link.dae'));
assert.strictEqual(failIfExpandedUrdfExpectedVisualSetInvalid(), true);
assert.strictEqual(state.web3dReadiness.state, 'scene_failed');
assert.ok(state.web3dReadiness.failure.failed_mesh_urls.includes('/assets/robotiq_85_base_link.dae'));
`, context);
"""
    subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def test_expanded_urdf_visible_ur5_2f_multi_visual_diagnostics_no_generic_false_failure():
    js_path = VIEWER / "viewer.js"
    harness = r"""
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
let source = fs.readFileSync(process.argv[1], 'utf8').replace(/boot\(\);\s*$/, '');
const element = () => ({ hidden: false, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', classList: { toggle() {} }, setAttribute() {}, querySelector() { return { textContent: '' }; }, appendChild() {}, addEventListener() {}, getBoundingClientRect() { return { width: 800, height: 600, left: 0, top: 0 }; } });
const context = { console, assert, window: { dispatched: [], location: { search: '' }, dispatchEvent(event) { this.dispatched.push(event?.detail?.state); }, parent: { postMessage() {} } }, document: { getElementById() { return element(); }, createElement() { return element(); } }, URLSearchParams, CustomEvent: function CustomEvent(type, init) { return { type, detail: init?.detail || {} }; }, requestAnimationFrame() { return 0; }, setTimeout() { return 1; }, clearTimeout() {} };
vm.createContext(context);
vm.runInContext(source + `
const robotLinks = ['base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link'];
const toolLinks = ['robotiq_85_base_link', 'robotiq_85_left_finger_link', 'robotiq_85_right_finger_link'];
state.sceneJson = { scene: { id: 'ur5_2f_test' }, robot_preview: { mode: 'expanded_urdf_loader', robot_instance_id: 'ur5_2f', expected_robot_visual_links: robotLinks, expected_tool_visual_links: toolLinks } };
state.robotUrdfPreviewDiagnostics = { robot_preview_lifecycle_state: 'ready', robot_preview_loaded: true, robot_failed_visual_count: 0, robot_visual_wrapper_world_matrices: robotLinks.concat(toolLinks).flatMap(link => [
  { link_name: link, visual_index: 0, object_name: 'visual_0' },
  { link_name: link, visual_index: 1, object_name: 'visual_1' },
]) };
collectRenderedMeshDiagnostics = () => [
  { id: 'workbench_generated', category: 'table', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'workbench_support_surface' },
  { id: 'configured_camera_generated', category: 'camera', render_policy: 'primary', render_status: 'mesh_loaded', mesh_loaded: true, readiness_category: 'configured_camera' },
];
const diagnostics = expandedUrdfVisualReadinessDiagnostics();
assert.strictEqual(diagnostics.required_visual_ready, true);
assert.ok(Array.isArray(diagnostics.duplicate_physical_visual_identities));
assert.strictEqual(failIfExpandedUrdfExpectedVisualSetInvalid(), false);
assert.notStrictEqual(state.web3dReadiness?.state, 'scene_failed');
`, context);
"""
    subprocess.run(["node", "-e", harness, str(js_path)], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
