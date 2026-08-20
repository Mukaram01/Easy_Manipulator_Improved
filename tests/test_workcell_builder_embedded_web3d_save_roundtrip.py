import hashlib
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
GUI = ROOT / "workcell_builder/workcell_builder/gui"
CONTROLLER = GUI / "embedded_web_edit_save_controller.hpp"
MAINWINDOW = GUI / "mainwindow.cpp"
MAINWINDOW_HEADER = GUI / "mainwindow.h"
WORKFLOW = ROOT / "scripts/run_workcell_studio_web_edit_workflow.py"
APPLICATOR = ROOT / "scripts/apply_workcell_studio_web_scene_edit_patch.py"
EXPORTER = ROOT / "scripts/export_workcell_studio_web_scene.py"


def _assert_transform_numerically_equal(actual, expected, *, tolerance=1e-12):
    assert set(actual) == set(expected)
    for field in ("xyz", "rpy", "scale"):
        assert len(actual[field]) == len(expected[field])
        for actual_value, expected_value in zip(actual[field], expected[field]):
            assert abs(float(actual_value) - float(expected_value)) <= tolerance


def _run_production_owner_binding_assertions(web_scene: Path, owner_id: str, expect_generated_pose_delta: bool) -> None:
    """Execute the production ownership binding against an exported payload."""
    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const THREE_IMPL=require('./workcell_studio_web/viewer/node_modules/three/build/three.cjs');
const payload=JSON.parse(fs.readFileSync(process.argv[2],'utf8')),ownerId=process.argv[3],expectDelta=process.argv[4]==='true',tableStl=process.argv[5];
const element=()=>({hidden:false,checked:false,disabled:false,textContent:'',innerHTML:'',classList:{toggle(){}},querySelector(){return null},querySelectorAll(){return[]},addEventListener(){},setAttribute(){},appendChild(){},remove(){}});
const context={console,assert,fs,THREE_IMPL,payload,ownerId,expectDelta,tableStl,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}}},document:{getElementById(){return element()},createElement(){return element()}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context);vm.runInContext(source+`
THREE=THREE_IMPL;createLabelElement=()=>null;state.sceneJson=payload;state.three.scene=new THREE.Scene();state.objects=[];state.physicalEditBindings=new Map();rebuildSelectionIdentityIndex(payload);
const visual=[...asArray(payload.sensors),...asArray(payload.assets)].find(item=>(item.camera_id===ownerId||item.support_surface_ref===ownerId)&&item.owner_relative_visual_transform);
assert(visual,'generated physical visual missing');const root=new THREE.Group();applyTransformToObject(root,transformOf(visual));state.three.scene.add(root);let meshObject=new THREE.Group();
if(ownerId==='support_surface_table'){
  assert.strictEqual(JSON.stringify(visual.scale),'[0.001,0.001,0.001]','production workbench must retain its URDF mesh-unit scale');
  const bytes=fs.readFileSync(tableStl),triangles=bytes.readUInt32LE(80),positions=new Float32Array(triangles*9);
  for(let triangle=0;triangle<triangles;triangle++)for(let vertex=0;vertex<3;vertex++)for(let axis=0;axis<3;axis++)positions[triangle*9+vertex*3+axis]=bytes.readFloatLE(84+triangle*50+12+vertex*12+axis*4);
  const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));geometry.computeBoundingBox();
  meshObject=materializeLoadedMesh(visual,visual.original_mesh_uri,geometry);
  const visualRoot=makeMeshVisualRoot(visual,meshObject);root.add(visualRoot);
  assert.strictEqual(meshObject.scale.toArray().join(','),'0.001,0.001,0.001','loaded workbench mesh must receive 0.001 exactly once');
}
const rendered={item:visual,object3d:root,meshObject,originalTransform:transformOf(visual)};state.objects.push(rendered);
const generatedWorld=root.position.clone();const bindings=bindExportedPhysicalTransformOwnership();assert.strictEqual(bindings.length,1);const binding=bindings[0];
assert.strictEqual(binding.owner.object3d.scale.toArray().join(','),'1,1,1','authored physical owner root must remain unit scale');assert.strictEqual(root.scale.toArray().join(','),'1,1,1','generated physical visual root must remain unit scale');
const expected=new THREE.Matrix4().compose(binding.owner.object3d.position,binding.owner.object3d.quaternion,binding.owner.object3d.scale).multiply(new THREE.Matrix4().compose(root.position,root.quaternion,root.scale));root.updateWorldMatrix(true,true);for(let i=0;i<16;i++)assert(Math.abs(root.matrixWorld.elements[i]-expected.elements[i])<1e-10,'owner world x stable local must equal visual world');const actual=new THREE.Vector3().setFromMatrixPosition(root.matrixWorld);
if(ownerId==='support_surface_table'){const bounds=new THREE.Box3().setFromObject(root),size=bounds.getSize(new THREE.Vector3());assert(!bounds.isEmpty(),'physical workbench bounds must be non-empty');assert(size.x>0.5&&size.x<2&&size.y>0.3&&size.y<2&&size.z>0.5&&size.z<1.5,'physical workbench bounds must be realistic metres, got '+size.toArray());}
assert.strictEqual(actual.distanceTo(generatedWorld)>1e-8,expectDelta,'reopen must move from stale generated pose iff owner was edited');const beforeAsync=root.matrixWorld.clone();visual.mesh_status='loaded';suppressOwnedAuthoredFallback(rendered);root.updateWorldMatrix(true,true);for(let i=0;i<16;i++)assert(Math.abs(root.matrixWorld.elements[i]-beforeAsync.elements[i])<1e-10,'async mesh completion changed visual pose');
`,context);
"""
    result = subprocess.run(
        ["node", "-e", harness, str(ROOT / "workcell_studio_web/viewer/viewer.js"), str(web_scene), owner_id, str(expect_generated_pose_delta).lower(),
         str(ROOT / "assets/environment/workbench_description/meshes/visual/table.stl")],
        cwd=ROOT, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr


def test_mainwindow_installs_controller_on_existing_top_save_action():
    controller = CONTROLLER.read_text(encoding="utf-8")
    mainwindow = MAINWINDOW.read_text(encoding="utf-8")

    assert '#include "embedded_web_edit_save_controller.hpp"' in mainwindow
    assert "installEmbeddedWebEditSaveController(" in mainwindow
    assert "scene_preview_widget_, save_layout_button_, layout_state_label_" in mainwindow
    assert 'findChild<QWebEngineView *>(QStringLiteral("embeddedWeb3dProductView"))' in controller
    assert 'property("workcell_embedded_save_controller")' in controller
    assert "ScenePreviewWidget::embedded_authoring_save_requested" in controller
    assert 'QPushButton(QStringLiteral("Save layout")' not in controller
    assert "embeddedSaveLayoutButton" not in controller
    assert "button->text()" not in controller


def test_mainwindow_web3d_state_drives_existing_save_and_dirty_label():
    source = CONTROLLER.read_text(encoding="utf-8")
    assert "native_dirty || (ready && matching_scene && dirty && valid_dirty_transforms)" in source
    assert "host_dirty_ && host_dirty_()" in source
    assert 'QStringLiteral("Unsaved Layout Edits: %1 (Web3D)")' in source
    for phase in ["save requested", "checking edits", "validation failed", "saved", "failed closed without regeneration"]:
        assert phase in source


def test_native_metadata_and_web_transforms_use_one_stable_id_transaction():
    controller = CONTROLLER.read_text(encoding="utf-8")
    mainwindow = MAINWINDOW.read_text(encoding="utf-8")
    serializer = (GUI / "layout_item_serializer.hpp").read_text(encoding="utf-8")

    assert "cannot safely compose native structural edits" not in controller
    assert "native_save_(patch, &native_error)" in controller
    assert "unified authored transaction complete" in controller
    assert "Validation is deliberately complete before mutating the native session" in mainwindow
    assert "Browser position/rotation wins; native state supplies metadata and structure" in mainwindow
    assert "RoleMetadataExplicitlyEdited" in mainwindow
    assert 'item["display_name"] = layout_string_scalar(state.display_name)' in serializer
    assert 'item["role"] = state.role' in serializer
    assert 'item["id"] = layout_string_scalar(state.id)' in serializer  # new records only
    assert "stable ID missing after write" in mainwindow
    assert "Save Layout persistence verified" in mainwindow


def test_qt_metadata_preview_updates_user_surfaces_without_renaming_canonical_id():
    mainwindow = MAINWINDOW.read_text(encoding="utf-8")
    preview = (GUI / "scene_preview_widget.cpp").read_text(encoding="utf-8")
    viewer = (ROOT / "workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")

    assert "refresh_scene_hierarchy_tree_from_current_items();" in mainwindow
    assert "set_authoring_item_metadata(" in mainwindow
    assert "setItemMetadata(" in preview
    assert "function setItemMetadataFromBridge(id, displayName, semanticRole)" in viewer
    metadata_bridge = viewer.split(
        "function setItemMetadataFromBridge(id, displayName, semanticRole)", 1
    )[1].split("window.__WORKCELL_EDITOR_API_V1__", 1)[0]
    assert "rendered.item.display_name = name" in metadata_bridge
    assert "rendered.item.role = role" in metadata_bridge
    assert "rendered.item.id =" not in metadata_bridge
    assert "populateObjectList();" in metadata_bridge
    assert "populateInspector(rendered);" in metadata_bridge
    assert "TreeRoleIsGroup, TreeRoleDisplayName" in mainwindow
    assert "state.display_name = item->data(0, TreeRoleDisplayName).toString().trimmed()" in mainwindow
    assert "state.display_name = item->text(0).trimmed()" not in mainwindow


def test_live_authoring_crud_api_updates_product_view_without_scene_regeneration():
    mainwindow = MAINWINDOW.read_text(encoding="utf-8")
    preview_header = (GUI / "scene_preview_widget.h").read_text(encoding="utf-8")
    preview = (GUI / "scene_preview_widget.cpp").read_text(encoding="utf-8")
    viewer = (ROOT / "workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")

    assert "schemaVersion: 'workcell_studio_live_authoring_capabilities/v1'" in viewer
    assert "apiVersion: '1.1.0'" in viewer
    assert "getCapabilities: ()" in viewer
    assert "addItem: item => addItemFromBridge(item)" in viewer
    assert "removeItem: id => removeItemFromBridge(id)" in viewer
    assert "duplicateItem: (id, item) => duplicateItemFromBridge(id, item)" in viewer
    assert "state.three.scene?.remove(rendered.object3d)" in viewer
    assert "state.objects = state.objects.filter(record => record !== rendered)" in viewer
    assert "state.three.scene.add(object3d)" in viewer

    for method in ("add_authoring_item", "duplicate_authoring_item", "remove_authoring_item"):
        assert method in preview_header
        assert f"ScenePreviewWidget::{method}" in preview
        assert method in mainwindow
    assert 'live_item.insert(QStringLiteral("mesh_uri"), mesh_source)' in preview
    assert 'mesh_source.startsWith(QStringLiteral("scenes/"))' in preview

    delete_body = mainwindow.split("void MainWindow::delete_selected_item()", 1)[1].split(
        "double MainWindow::current_nudge_step_m", 1
    )[0]
    assert "remove_authoring_item(id)" in delete_body
    assert "request_embedded_web_product_view_refresh" not in delete_body


def test_executable_live_crud_mutates_three_scene_in_place():
    viewer = ROOT / "workcell_studio_web/viewer/viewer.js"
    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const THREE_IMPL=require('./workcell_studio_web/viewer/node_modules/three/build/three.cjs');
const element=()=>({hidden:false,checked:false,disabled:false,textContent:'',innerHTML:'',value:'',dataset:{},classList:{toggle(){},add(){},remove(){}},querySelector(){return null},querySelectorAll(){return[]},addEventListener(){},setAttribute(){},appendChild(){},remove(){}});
const context={console,assert,THREE_IMPL,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}}},document:{getElementById(){return element()},createElement(){return element()}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context);vm.runInContext(source+`
THREE=THREE_IMPL;state.sceneJson={scene:{id:'ur5_2f_test'}};state.three.scene=new THREE.Scene();state.objects=[];state.pickRecords=[];state.dirtyTransforms=new Map();state.undoStack=[];state.redoStack=[];
createLabelElement=()=>null;populateObjectList=()=>{};updateLabels=()=>{};renderSceneSummary=()=>{};bindExportedPhysicalTransformOwnership=()=>[];tryLoadMesh=()=>{};selectObject=id=>{state.selected=String(id||'')};clearSelection=()=>{state.selected=''};detachTransformGizmo=()=>{};updateDirtyState=()=>{};
const item={id:'object_02',display_name:'Fixture Plate',role:'fixture',category:'object',editable:true,locked:false,source_layer:'editable_layout',world_pose:{xyz:[.95,.45,.16],rpy:[0,1.48352986419518,0]},dimensions:[.2,.2,.2],primitive:{type:'box',dimensions:[.2,.2,.2]}};
let result=window.__WORKCELL_EDITOR_API_V1__.addItem(item);assert.strictEqual(state.objects.length,1);assert.strictEqual(state.three.scene.children.length,1);assert.strictEqual(result.selectedItemId,'object_02');
result=window.__WORKCELL_EDITOR_API_V1__.setItemMetadata('object_02','Fixture Plate','fixture');assert.strictEqual(state.objects[0].item.display_name,'Fixture Plate');assert.strictEqual(state.objects[0].item.id,'object_02');
result=window.__WORKCELL_EDITOR_API_V1__.duplicateItem('object_02',{...item,id:'object_02_copy',display_name:'Fixture Plate copy',world_pose:{xyz:[1.05,.55,.16],rpy:[0,1.48352986419518,0]}});assert.strictEqual(state.objects.length,2);assert.strictEqual(state.three.scene.children.length,2);assert.strictEqual(result.selectedItemId,'object_02_copy');
result=window.__WORKCELL_EDITOR_API_V1__.setItemMetadata('object_02_copy','Inspection Fixture','fixture');assert.strictEqual(state.objects.find(x=>x.item.id==='object_02_copy').item.display_name,'Inspection Fixture');
result=window.__WORKCELL_EDITOR_API_V1__.removeItem('object_02');assert.strictEqual(state.objects.length,1);assert.strictEqual(state.three.scene.children.length,1);assert.strictEqual(state.objects[0].item.id,'object_02_copy');
result=window.__WORKCELL_EDITOR_API_V1__.addItem(item);assert.strictEqual(state.objects.length,2,'native undo restores exactly one item');
result=window.__WORKCELL_EDITOR_API_V1__.removeItem('object_02');assert.strictEqual(state.objects.length,1,'native redo removes it again');
result=window.__WORKCELL_EDITOR_API_V1__.addItem({...item,display_name:'Fixture Plate'});assert.strictEqual(state.objects.length,2,'second undo restores the same stable ID');
assert.strictEqual(window.__WORKCELL_EDITOR_API_V1__.apiVersion,'1.1.0');
const capabilities=window.__WORKCELL_EDITOR_API_V1__.getCapabilities();assert.strictEqual(capabilities.schemaVersion,'workcell_studio_live_authoring_capabilities/v1');assert.ok(capabilities.operations.includes('duplicateItem'));
`,context);
"""
    result = subprocess.run(
        ["node", "-e", harness, str(viewer)], cwd=ROOT, capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr


def test_versioned_capability_handshake_fails_closed_for_stale_bundle():
    preview = (GUI / "scene_preview_widget.cpp").read_text(encoding="utf-8")
    viewer = (ROOT / "workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")
    assert "verify_embedded_editor_contract(identity)" in preview
    assert "typeof api.getCapabilities !== 'function'" in preview
    assert "capabilities?.apiVersion !== '1.1.0'" in preview
    assert "Live authoring disabled" in preview
    assert "no scene regeneration was attempted" in preview
    assert "getCapabilities: ()" in viewer
    assert "schemaVersion: 'workcell_studio_live_authoring_capabilities/v1'" in viewer


def test_successful_live_save_rebases_in_place_without_product_view_regeneration():
    controller = CONTROLLER.read_text(encoding="utf-8")
    mainwindow = MAINWINDOW.read_text(encoding="utf-8")

    request_save = controller.split("void requestSave()", 1)[1].split("void startWorkflow", 1)[0]
    assert "live authored session retained; no Product View regeneration required" in request_save
    assert "native_save_(patch, &native_error)" in request_save
    assert "startWorkflow(" not in request_save
    assert "requestPostSaveProductViewRefresh" not in request_save
    rebase_success = controller.split("if (browser_rebase_succeeded_)", 1)[1].split(
        "} else {", 1
    )[0]
    assert "browser baseline rebased in place; no Product View regeneration required" in rebase_success
    assert "requestPostSaveProductViewRefresh" not in rebase_success
    assert "live authored session retained without Product View regeneration" in mainwindow
    assert "Save Layout persisted authored YAML only" in mainwindow


def test_open_authoring_session_pins_one_canonical_scene_directory():
    mainwindow = MAINWINDOW.read_text(encoding="utf-8")
    header = (GUI / "mainwindow.h").read_text(encoding="utf-8")

    assert "QString authoring_session_scene_dir_;" in header
    assert "authoring_session_scene_dir_ = canonical_scene_path_string(requested_scene.scene_dir)" in mainwindow
    assert "selected_scene_state_.path = authoring_session_scene_dir_" in mainwindow
    assert "Scene discovery alias ignored for active authoring session" in mainwindow
    assert "const fs::path scene_dir = fs::path(selected_scene_path().toStdString())" in mainwindow


def test_qt_reads_patch_from_existing_browser_editor_api_and_checks_identity():
    source = CONTROLLER.read_text(encoding="utf-8")
    for token in [
        "window.__WORKCELL_EDITOR_API_V1__",
        "api.getState()",
        "api.getEditPatch()",
        'kPatchSchema = "workcell_studio_web_scene_edit_patch/v1"',
        'kPatchCreator = "static_web_viewer"',
        'patch.value(QStringLiteral("scene_id")).toString() != scene_id_',
        "view_->url() != expected_url_",
        "sceneIdFromViewerUrl(view_->url()) != scene_id_",
        "Scene changed—reload required",
        "No changes",
        "Validation failed",
        "Saved",
    ]:
        assert token in source


def test_preview_context_is_the_primary_bounded_save_path_source():
    source = CONTROLLER.read_text(encoding="utf-8")
    header = (GUI / "scene_preview_widget.h").read_text(encoding="utf-8")
    implementation = (GUI / "scene_preview_widget.cpp").read_text(encoding="utf-8")

    assert "PreviewContext preview_context() const;" in header
    assert "ScenePreviewWidget::PreviewContext ScenePreviewWidget::preview_context() const" in implementation
    for field in ["context.absolute_repo_root", "context.absolute_scene_dir", "context.scene_id"]:
        assert field in source
    assert "QDir::currentPath()" not in source
    assert "applicationDirPath()" not in source
    assert "if (!dir.cdUp())" not in source
    assert 'value(QStringLiteral("WORKCELL_STUDIO_REPO_ROOT"))' in source


def test_mainwindow_preview_context_uses_canonical_repository_root_not_extractor_file():
    mainwindow = MAINWINDOW.read_text(encoding="utf-8")
    header = MAINWINDOW_HEADER.read_text(encoding="utf-8")
    wiring = mainwindow.split("ScenePreviewWidget::PreviewContext preview_context;", 1)[1].split(
        "scene_preview_widget_->set_preview_context(preview_context);", 1
    )[0]

    assert "resolve_workcell_studio_repo_root" in header
    assert "preview_context.scene_id = selected_scene_state_.name" in wiring
    assert "selected_scene_info.canonicalFilePath()" in wiring
    assert "preview_context.absolute_repo_root = resolve_workcell_studio_repo_root" in wiring
    assert "resolve_scene3d_extractor_script_path" not in wiring
    assert "extract_scene_urdf_visual_mesh_index.py" not in wiring
    assert 'QStringLiteral("scenes/%1").arg(preview_context.scene_id)' in wiring
    assert "expected_scene_dir" in wiring
    assert "preview_context.absolute_repo_root.clear()" in wiring


def test_preview_repository_root_resolver_requires_product_markers_and_directory():
    source = MAINWINDOW.read_text(encoding="utf-8")
    resolver = source.split("QString MainWindow::resolve_workcell_studio_repo_root", 1)[1].split(
        "QString MainWindow::selected_scene_name", 1
    )[0]

    assert "root_info.exists()" in resolver
    assert "root_info.isDir()" in resolver
    assert 'QStringLiteral("scenes")' in resolver
    assert 'QStringLiteral("scripts/run_workcell_studio_web_edit_workflow.py")' in resolver
    assert 'QStringLiteral("workcell_studio_web/viewer/index.html")' in resolver
    assert "extract_scene_urdf_visual_mesh_index.py" not in resolver


def test_save_rejects_url_mismatch_stale_context_and_scene_path_escape():
    source = CONTROLLER.read_text(encoding="utf-8")
    assert "scene_id_ != url_scene_id" in source
    assert "expected_scene_path" in source
    assert "preview_->preview_context()" in source
    assert "canonicalPath(current.absolute_repo_root.trimmed()) == repo_root_" in source
    assert "canonicalPath(current.absolute_scene_dir.trimmed()) == scene_dir_" in source
    assert "Web3D edits preserved" in source
    assert "QFileInfo(scene_dir_).absolutePath()) != scenes_root" in source


def test_qt_save_uses_native_authored_session_without_legacy_patch_workflow():
    source = CONTROLLER.read_text(encoding="utf-8")
    request_save = source.split("void requestSave()", 1)[1].split("void startWorkflow", 1)[0]
    assert "native_save_(patch, &native_error)" in request_save
    assert "startWorkflow(" not in request_save
    assert "writePatchAtomically(" not in request_save
    assert "run_workcell_studio_web_edit_workflow.py" not in request_save


def test_qt_logs_patch_summary_before_unified_native_save():
    source = CONTROLLER.read_text(encoding="utf-8")
    for token in [
        "patch summary: scene_id=%1 dirty_edit_count=%2",
        "patch edit: item_id=%1 operation=%2",
        "persistence_source=%5",
    ]:
        assert token in source
    request_save = source.split("void requestSave()", 1)[1].split("void startWorkflow", 1)[0]
    assert request_save.index("logPatchSummary(patch)") < request_save.index("native_save_(patch, &native_error)")


def test_successful_qt_save_does_not_invoke_legacy_workflow_or_refresh():
    controller = CONTROLLER.read_text(encoding="utf-8")
    request_save = controller.split("void requestSave()", 1)[1].split("void startWorkflow", 1)[0]
    assert "scripts/run_workcell_studio_web_edit_workflow.py" not in request_save
    assert "request_post_save_product_view_refresh" not in request_save
    assert "requestPostSaveProductViewRefresh" not in request_save
    assert "native_save_(patch, &native_error)" in request_save


def test_post_save_refresh_renews_all_lifecycle_identities_and_rejects_stale_ready():
    controller = CONTROLLER.read_text(encoding="utf-8")
    header = (GUI / "scene_preview_widget.h").read_text(encoding="utf-8")
    preview = (GUI / "scene_preview_widget.cpp").read_text(encoding="utf-8")
    refresh = preview.split("int ScenePreviewWidget::request_post_save_product_view_refresh()", 1)[1].split(
        "bool ScenePreviewWidget::preview_payload_matches", 1
    )[0]

    assert "++preview_payload_revision_" in refresh
    assert "++preview_payload_generation_" in refresh
    assert 'true, QStringLiteral("post_save"), EmbeddedWebSourcePolicy::PersistedCanonical' in refresh
    assert "post_save_refresh_generation_ = embedded_web_request_generation_" in refresh
    assert "post_save_refresh_payload_revision_ = preview_payload_revision_" in refresh
    assert "post_save_product_view_refresh_finished" in header
    assert "revision != saved_reload_revision_" in controller
    assert "builder_revision != expected_builder_revision" in preview
    assert "navigation_token != embedded_web_navigation_token_" in preview


def test_post_save_refresh_failure_keeps_rebased_browser_or_blocks_repeat_save():
    controller = CONTROLLER.read_text(encoding="utf-8")
    preview = (GUI / "scene_preview_widget.cpp").read_text(encoding="utf-8")
    failure = preview.split("void ScenePreviewWidget::show_embedded_web_preparation_failure", 1)[1].split(
        "void ScenePreviewWidget::clear_embedded_editor_state_for_scene_handoff", 1
    )[0]

    assert "finish_post_save_product_view_refresh(identity, 0, false, detail)" in failure
    assert failure.index("finish_post_save_product_view_refresh") < failure.index("embedded_web_view_->setHtml")
    assert "persisted YAML and the browser edit baseline agree" in controller
    assert "another save is blocked until reload" in controller
    assert "reload_required_after_save_" in controller


def test_source_yaml_write_is_allowlisted_backed_up_and_atomic():
    source = APPLICATOR.read_text(encoding="utf-8")
    for token in [
        '"layout/workcell_studio_layout.yaml"',
        '"environment.yaml"',
        'FORBIDDEN_TARGET_PARTS = {"generated"}',
        'FORBIDDEN_TARGET_NAMES = {"cell_definition.yaml", "scene_manifest.yaml"}',
        "shutil.copy2(path, path.with_name",
        "tempfile.mkstemp",
        "stream.flush()",
        "os.fsync(stream.fileno())",
        "os.replace(temporary, path)",
        "temporary.unlink()",
    ]:
        assert token in source
    assert "path.write_text(yaml.safe_dump" not in source


def test_save_roundtrip_has_no_browser_source_writes_or_robot_motion():
    combined = "\n".join(
        path.read_text(encoding="utf-8").lower()
        for path in (CONTROLLER, WORKFLOW, APPLICATOR)
    )
    for forbidden in [
        "execute_trajectory",
        "getmotionplan",
        "/plan_kinematic_path",
        "real_hardware_enabled: true",
        "move_group.execute",
        "ros2 launch",
        "qwebchannel",
    ]:
        assert forbidden not in combined
    assert "no robot motion is started" in combined
    assert "does not launch controllers" in combined
    assert "move real hardware" in combined


def test_roundtrip_change_stays_focused():
    assert len(CONTROLLER.read_text(encoding="utf-8").splitlines()) < 950
    assert len(WORKFLOW.read_text(encoding="utf-8").splitlines()) < 370
    assert len(APPLICATOR.read_text(encoding="utf-8").splitlines()) < 340


def test_linked_group_patch_uses_existing_two_edit_save_path():
    viewer = (ROOT / "workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")
    controller = CONTROLLER.read_text(encoding="utf-8")
    assert "state.undoStack.push({ changes:" in viewer
    assert "for (const rendered of state.objects)" in viewer
    assert "api.getEditPatch()" in controller
    assert "scripts/run_workcell_studio_web_edit_workflow.py" in controller


def _rendered_items(payload):
    if isinstance(payload, dict):
        if isinstance(payload.get("id"), str) and isinstance(payload.get("pose"), dict):
            yield payload
        for value in payload.values():
            yield from _rendered_items(value)
    elif isinstance(payload, list):
        for value in payload:
            yield from _rendered_items(value)


def _transform(item):
    vector = lambda values: dict(zip(("x", "y", "z"), map(float, values)))
    pose = item["pose"]
    scale = item.get("scale", [1.0, 1.0, 1.0])
    return {"pose": {"xyz": vector(pose["xyz"]), "rpy": vector(pose["rpy"])}, "scale": vector(scale)}


def test_executable_target_bin_linked_save_and_reload_roundtrip(tmp_path):
    """Exercise the same validated dry-run/write/re-export path used by Qt."""
    scene = tmp_path / "ur5_2f_test"
    shutil.copytree(ROOT / "scenes/ur5_2f_test", scene)
    output = tmp_path / "web"
    before_path = output / "ur5_2f_test.before.web_scene.json"
    output.mkdir()
    subprocess.run(
        [sys.executable, str(EXPORTER), "--scene", str(scene), "--output", str(before_path)],
        cwd=ROOT, check=True, capture_output=True, text=True,
    )
    before = json.loads(before_path.read_text(encoding="utf-8"))
    items = {item["id"]: item for item in _rendered_items(before)}
    old_bin = _transform(items["target_bin_default"])
    old_zone = _transform(items["place_zone_default"])
    delta = {"x": 0.08, "y": -0.03, "z": 0.0}

    def moved(transform):
        result = json.loads(json.dumps(transform))
        for axis, amount in delta.items():
            result["pose"]["xyz"][axis] += amount
        result["pose"]["rpy"]["z"] += 0.1
        return result

    patch = {
        "schema_version": "workcell_studio_web_scene_edit_patch/v1",
        "scene_id": "ur5_2f_test",
        "source_scene_schema_version": before["schema_version"],
        "created_at": "2026-07-28T00:00:00Z",
        "created_by": "static_web_viewer",
        "provenance": {"source_web_scene_file": before_path.name},
        "edits": [{"item_id": "target_bin_default", "operation": "update_transform", "editable_required": True,
                   "locked_required": False, "old_transform": old_bin, "new_transform": moved(old_bin)}],
    }
    patch_path = output / "edit_patch.json"
    patch_path.write_text(json.dumps(patch), encoding="utf-8")
    assert {edit["item_id"] for edit in patch["edits"]} == {"target_bin_default"}

    layout_path = scene / "layout/workcell_studio_layout.yaml"
    layout_before = layout_path.read_bytes()
    protected_before = {
        path.relative_to(scene): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in scene.rglob("*") if path.is_file() and path != layout_path
        and ".bak" not in path.name and "generated" not in path.relative_to(scene).parts
        and path.relative_to(scene) != Path("config/moveit_collision_objects.yaml")
    }
    result = subprocess.run(
        [sys.executable, str(WORKFLOW), "--scene", str(scene), "--patch", str(patch_path),
         "--output-dir", str(output), "--write"],
        cwd=ROOT, check=True, capture_output=True, text=True,
    )
    assert "persistence verification result: PASS" in result.stdout
    assert layout_path.read_bytes() != layout_before
    protected_after = {
        path.relative_to(scene): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in scene.rglob("*") if path.is_file() and path != layout_path and ".bak" not in path.name
        and "generated" not in path.relative_to(scene).parts
        and path.relative_to(scene) != Path("config/moveit_collision_objects.yaml")
    }
    assert protected_after == protected_before  # authored non-layout inputs and hardware stay untouched

    layout = yaml.safe_load(layout_path.read_text(encoding="utf-8"))
    layout_items = {item["id"]: item for item in layout["items"]}
    after = json.loads((output / "ur5_2f_test.after.web_scene.json").read_text(encoding="utf-8"))
    reloaded = {item["id"]: item for item in _rendered_items(after)}
    expected = moved(old_bin)
    assert {edit["item_id"] for edit in patch["edits"]} == {"target_bin_default"}
    assert layout_items["target_bin_default"]["pose"]["xyz"] == list(expected["pose"]["xyz"].values())
    assert layout_items["target_bin_default"]["pose"]["rpy"] == list(expected["pose"]["rpy"].values())
    assert _transform(reloaded["target_bin_default"])["pose"] == expected["pose"]
    # The authored overlay pose is not persisted independently. Export derives
    # both its transform and footprint from the destination asset.
    assert layout_items["place_zone_default"]["pose"]["xyz"] == list(old_zone["pose"]["xyz"].values())
    assert layout_items["place_zone_default"]["pose"]["rpy"] == list(old_zone["pose"]["rpy"].values())
    assert _transform(reloaded["place_zone_default"])["pose"] == expected["pose"]
    assert reloaded["place_zone_default"]["dimensions"][:2] == reloaded["target_bin_default"]["dimensions"][:2]
    assert 0 < reloaded["place_zone_default"]["dimensions"][2] <= 0.01


def test_executable_production_camera_table_save_roundtrips(tmp_path):
    """Use real exported owner records and the exact Qt dry-run/write workflow."""
    for edited_ids in [
        ("realsense_overhead",),
        ("support_surface_table",),
        ("realsense_overhead", "support_surface_table"),
    ]:
        case = "_and_".join(edited_ids)
        scene = tmp_path / case / "ur5_2f_test"
        shutil.copytree(ROOT / "scenes/ur5_2f_test", scene)
        output = tmp_path / case / "web"
        before_path = output / "ur5_2f_test.before.web_scene.json"
        output.mkdir()
        subprocess.run(
            [sys.executable, str(EXPORTER), "--scene", str(scene), "--output", str(before_path)],
            cwd=ROOT, check=True, capture_output=True, text=True,
        )
        before = json.loads(before_path.read_text(encoding="utf-8"))
        owners = {item["id"]: item for item in before["ui_selection_owners"]}
        before_visuals = {
            item.get("camera_id") or item.get("support_surface_ref"): item
            for section in ("sensors", "assets") for item in before[section]
            if item.get("owner_relative_visual_transform")
        }
        assert set(before_visuals) == {"realsense_overhead", "support_surface_table"}
        assert all(visual["owner_relative_visual_transform"]["scale"] == [1.0, 1.0, 1.0] for visual in before_visuals.values())
        for owner_id in before_visuals:
            _run_production_owner_binding_assertions(before_path, owner_id, False)
        assert owners["realsense_overhead"]["provenance"]["pose"] == "layout/workcell_studio_layout.yaml"
        assert owners["support_surface_table"]["provenance"]["pose"] == "layout/workcell_studio_layout.yaml"
        edits = []
        for item_id in edited_ids:
            old = _transform(owners[item_id])
            new = json.loads(json.dumps(old))
            new["pose"]["xyz"]["x"] += 0.04
            new["pose"]["rpy"]["z"] += 0.12
            edits.append({
                "item_id": item_id, "operation": "update_transform",
                "editable_required": True, "locked_required": False,
                "old_transform": old, "new_transform": new,
            })
        patch = {
            "schema_version": "workcell_studio_web_scene_edit_patch/v1",
            "scene_id": "ur5_2f_test", "source_scene_schema_version": before["schema_version"],
            "created_at": "2026-08-03T00:00:00Z", "created_by": "static_web_viewer",
            "edits": edits,
        }
        patch_path = output / "edit_patch.json"
        patch_path.write_text(json.dumps(patch), encoding="utf-8")
        protected_before = {
            path.relative_to(scene): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in scene.rglob("*") if path.is_file() and path.name != "workcell_studio_layout.yaml"
            and ".bak" not in path.name and "generated" not in path.relative_to(scene).parts
            and path.relative_to(scene) != Path("config/moveit_collision_objects.yaml")
        }
        for mode in ("--dry-run-apply", "--write"):
            result = subprocess.run(
                [sys.executable, str(WORKFLOW), "--scene", str(scene), "--patch", str(patch_path),
                 "--output-dir", str(output), mode], cwd=ROOT, capture_output=True, text=True,
            )
            assert result.returncode == 0, result.stdout + result.stderr
        assert "persistence verification result: PASS" in result.stdout
        after = json.loads((output / "ur5_2f_test.web_scene.json").read_text(encoding="utf-8"))
        after_owners = {item["id"]: item for item in after["ui_selection_owners"]}
        after_visuals = {
            item.get("camera_id") or item.get("support_surface_ref"): item
            for section in ("sensors", "assets") for item in after[section]
            if item.get("owner_relative_visual_transform")
        }
        for edit in edits:
            assert _transform(after_owners[edit["item_id"]]) == edit["new_transform"]
            assert edit["old_transform"]["scale"] == edit["new_transform"]["scale"] == {"x": 1.0, "y": 1.0, "z": 1.0}
        # Re-export derives the browser's exact owner-local transform from the
        # refreshed generated world pose. With compounded RPY rotations this
        # local representation can legitimately differ after an owner edit;
        # the invariant is that binding reconstructs the exported world pose
        # without applying the edit a second time.
        for owner_id in before_visuals:
            relative = after_visuals[owner_id]["owner_relative_visual_transform"]
            assert relative["scale"] == [1.0, 1.0, 1.0]
            assert all(math.isfinite(float(value)) for field in ("xyz", "rpy") for value in relative[field])
            provenance = after_visuals[owner_id]["provenance"]["owner_relative_visual_transform"]
            assert provenance["source_owner_pose_provenance"] == after_owners[owner_id]["provenance"]["pose"]
            assert provenance["source_owner_pose_provenance"] == "layout/workcell_studio_layout.yaml"
            assert provenance["generated_visual_pose"]
            # The write workflow regenerates the scene before reopen, so
            # the generated physical visual is already expressed at the
            # edited owner pose. Ownership binding must preserve that world
            # pose; an additional delta here would apply the edit twice.
            _run_production_owner_binding_assertions(
                output / "ur5_2f_test.web_scene.json", owner_id, False,
            )
        protected_after = {
            path.relative_to(scene): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in scene.rglob("*") if path.is_file() and path.name != "workcell_studio_layout.yaml" and ".bak" not in path.name
            and "generated" not in path.relative_to(scene).parts
            and path.relative_to(scene) != Path("config/moveit_collision_objects.yaml")
        }
        assert protected_after == protected_before


def test_exact_workflow_rejects_identity_table_baseline_without_scene_mutation(tmp_path):
    scene = tmp_path / "ur5_2f_test"
    shutil.copytree(ROOT / "scenes/ur5_2f_test", scene)
    output = tmp_path / "web"
    output.mkdir()
    before_path = output / "ur5_2f_test.before.web_scene.json"
    subprocess.run([sys.executable, str(EXPORTER), "--scene", str(scene), "--output", str(before_path)], cwd=ROOT, check=True)
    before = json.loads(before_path.read_text(encoding="utf-8"))
    table = {item["id"]: item for item in before["ui_selection_owners"]}["support_surface_table"]
    new = _transform(table)
    new["pose"]["rpy"]["z"] -= 0.61086524
    identity = {"pose": {"xyz": {"x": 0.0, "y": 0.0, "z": 0.0}, "rpy": {"x": 0.0, "y": 0.0, "z": 0.0}}, "scale": {"x": 1.0, "y": 1.0, "z": 1.0}}
    patch = {"schema_version": "workcell_studio_web_scene_edit_patch/v1", "scene_id": "ur5_2f_test", "source_scene_schema_version": before["schema_version"], "created_at": "2026-08-03T00:00:00Z", "created_by": "static_web_viewer", "edits": [{"item_id": "support_surface_table", "operation": "update_transform", "editable_required": True, "locked_required": False, "old_transform": identity, "new_transform": new}]}
    patch_path = output / "identity-old-transform.json"
    patch_path.write_text(json.dumps(patch), encoding="utf-8")
    hashes = {path.relative_to(scene): hashlib.sha256(path.read_bytes()).hexdigest() for path in scene.rglob("*") if path.is_file()}
    for mode in ("--dry-run-apply", "--write"):
        result = subprocess.run([sys.executable, str(WORKFLOW), "--scene", str(scene), "--patch", str(patch_path), "--output-dir", str(output), mode], cwd=ROOT, capture_output=True, text=True)
        assert result.returncode == 1
        assert "old_transform precondition" in result.stderr
        assert "write/apply is blocked" in result.stderr
        assert {path.relative_to(scene): hashlib.sha256(path.read_bytes()).hexdigest() for path in scene.rglob("*") if path.is_file()} == hashes

def test_executable_linked_edit_undo_redo_preserves_canonical_selection():
    viewer = ROOT / "workcell_studio_web/viewer/viewer.js"
    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const element=()=>({hidden:false,checked:false,disabled:false,textContent:'',innerHTML:'',value:'0',classList:{toggle(){},add(){},remove(){}},querySelector(){return null;},querySelectorAll(){return[];},addEventListener(){},setAttribute(){},appendChild(){},remove(){}});
const context={console,assert,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}}},document:{getElementById(){return element();},createElement(){return element();}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context); vm.runInContext(source+`
const object=(x,y,z)=>({position:{x,y,z,set(a,b,c){this.x=a;this.y=b;this.z=c;}},rotation:{x:0,y:0,z:0,set(a,b,c){this.x=a;this.y=b;this.z=c;}},scale:{x:1,y:1,z:1,set(a,b,c){this.x=a;this.y=b;this.z=c;}}});
const row=(id,z)=>({item:{id,editable:true,locked:false,source_layer:'editable_layout',render_policy:'primary',transform_group:'default_drop_destination',...(id==='place_zone_default'?{role:'place_zone',target_ref:'target_bin_default'}:{role:'target_bin'})},object3d:object(.55,-.28,z),originalTransform:null});
const bin=row('target_bin_default',.2),zone=row('place_zone_default',.105); for(const item of [bin,zone]) item.originalTransform=transformFromObject(item.object3d);
state.objects=[bin,zone];state.sceneJson={scene:{id:'ur5_2f_test'}};state.dirtyTransforms=new Map();state.undoStack=[];state.redoStack=[];state.selected='target_bin_default';
updateLabels=()=>{};updateDirtyState=()=>{};emitDirtyChanged=()=>{};populateObjectList=()=>{};populateInspector=()=>{};
let savedTransform=cloneTransform(bin.originalTransform);savedTransform.pose.xyz.x+=.08;assert.strictEqual(markDirtyTransform(bin,savedTransform),true);assert.strictEqual(buildEditPatch().edits.length,1);assert.strictEqual(buildEditPatch().edits[0].item_id,'target_bin_default');
undoPreviewEdit();assert.strictEqual(state.selected,'target_bin_default');assert.strictEqual(state.dirtyTransforms.size,0);
redoPreviewEdit();assert.strictEqual(state.selected,'target_bin_default');assert.strictEqual(state.dirtyTransforms.size,1);assert.strictEqual(zone.object3d.position.x,.63);
`,context);
"""
    result = subprocess.run(["node", "-e", harness, str(viewer)], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
