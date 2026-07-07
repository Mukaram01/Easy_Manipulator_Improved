import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web" / "viewer"


def test_static_viewer_files_exist():
    assert (VIEWER / "index.html").is_file()
    assert (VIEWER / "viewer.js").is_file()
    assert (VIEWER / "style.css").is_file()
    assert (VIEWER / "README.md").is_file()


def test_index_references_static_assets():
    index = (VIEWER / "index.html").read_text(encoding="utf-8")
    assert 'href="style.css"' in index
    assert 'src="viewer.js"' in index
    assert 'id="scene-file"' in index
    assert 'web_scene.json' in index


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
        "bounds.getBoundingSphere",
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
        "no primitive geometry or mesh was provided; using box fallback",
        "unsafe mesh_uri rejected by viewer policy",
        "no mesh_uri provided",
        "loader_failure",
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


def test_viewer_applies_mesh_local_transform_only_to_loaded_meshes():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "meshLocalTransformOf",
        "item?.mesh_local_transform || item?.visual_local_transform",
        "meshLocalVector(transform.xyz, [0, 0, 0], 'xyz', reasons)",
        "meshLocalVector(transform.rpy, [0, 0, 0], 'rpy', reasons)",
        "meshLocalVector(transform.scale, [1, 1, 1], 'scale', reasons)",
        "Number.isFinite(number)",
        "applyMeshLocalTransform",
        "invalid_mesh_local_transform",
        "applied_mesh_local_transform",
    ]:
        assert token in js
    try_load_body = js.split("async function tryLoadMesh", 1)[1].split("function collectItems", 1)[0]
    assert try_load_body.index("materializeLoadedMesh(item, uri, loaded)") < try_load_body.index("applyMeshLocalTransform(meshObject, item)")
    assert try_load_body.index("applyMeshLocalTransform(meshObject, item)") < try_load_body.index("rendered.object3d.add(meshObject)")
    apply_pose_body = js.split("function applyPose", 1)[1].split("function assignItemUserData", 1)[0]
    assert "mesh_local_transform" not in apply_pose_body
    assert "visual_local_transform" not in apply_pose_body

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
    assert ".rotation" in local_transform_body
    assert ".scale" in local_transform_body

    apply_pose_body = js.split("function applyPose", 1)[1].split("function assignItemUserData", 1)[0]
    assert "mesh_local_transform" not in apply_pose_body
    assert "visual_local_transform" not in apply_pose_body
    assert "applyMeshLocalTransform" not in apply_pose_body


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
    assert "frameScene(bounds);" in render_scene_body

    assert "li.addEventListener('click', () => selectObject(rendered.item.id));" in append_object_list_body
    assert "const selected = rendered.item.id === id;" in select_body
    assert "rendered.item.locked" not in select_body
    assert "canEditItem(rendered.item)" not in select_body
    assert "if (item?.id) selectObject(item.id);" in pick_body


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
state.sceneJson = { scene: { id: 'status_test' } };
state.runtimeWarnings = [{ code: 'camera_framing_blocker_excluded', reason: 'hidden helper excluded' }];
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
assert.strictEqual(status.runtimeWarnings.length, 1);
assert.strictEqual(status.runtimeWarnings[0].code, 'camera_framing_blocker_excluded');
assert.strictEqual(isUserFacingWarning(status.runtimeWarnings[0]), false);
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
