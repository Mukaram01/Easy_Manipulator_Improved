import hashlib
import json
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


def _run_production_owner_binding_assertions(web_scene: Path, owner_id: str, expect_generated_pose_delta: bool) -> None:
    """Execute the production ownership binding against an exported payload."""
    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const THREE_IMPL=require('./workcell_studio_web/viewer/node_modules/three/build/three.cjs');
const payload=JSON.parse(fs.readFileSync(process.argv[2],'utf8')),ownerId=process.argv[3],expectDelta=process.argv[4]==='true';
const element=()=>({hidden:false,checked:false,disabled:false,textContent:'',innerHTML:'',classList:{toggle(){}},querySelector(){return null},querySelectorAll(){return[]},addEventListener(){},setAttribute(){},appendChild(){},remove(){}});
const context={console,assert,THREE_IMPL,payload,ownerId,expectDelta,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}}},document:{getElementById(){return element()},createElement(){return element()}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context);vm.runInContext(source+`
THREE=THREE_IMPL;createLabelElement=()=>null;state.sceneJson=payload;state.three.scene=new THREE.Scene();state.objects=[];state.physicalEditBindings=new Map();rebuildSelectionIdentityIndex(payload);
const visual=[...asArray(payload.sensors),...asArray(payload.assets)].find(item=>(item.camera_id===ownerId||item.support_surface_ref===ownerId)&&item.owner_relative_visual_transform);
assert(visual,'generated physical visual missing');const root=new THREE.Group();applyTransformToObject(root,transformOf(visual));state.three.scene.add(root);const rendered={item:visual,object3d:root,meshObject:new THREE.Group(),originalTransform:transformOf(visual)};root.add(rendered.meshObject);state.objects.push(rendered);
const generatedWorld=root.position.clone();const bindings=bindExportedPhysicalTransformOwnership();assert.strictEqual(bindings.length,1);const binding=bindings[0];
const expected=new THREE.Matrix4().compose(binding.owner.object3d.position,binding.owner.object3d.quaternion,binding.owner.object3d.scale).multiply(new THREE.Matrix4().compose(root.position,root.quaternion,root.scale));root.updateWorldMatrix(true,true);for(let i=0;i<16;i++)assert(Math.abs(root.matrixWorld.elements[i]-expected.elements[i])<1e-10,'owner world x stable local must equal visual world');const actual=new THREE.Vector3().setFromMatrixPosition(root.matrixWorld);
assert.strictEqual(actual.distanceTo(generatedWorld)>1e-8,expectDelta,'reopen must move from stale generated pose iff owner was edited');const beforeAsync=root.matrixWorld.clone();visual.mesh_status='loaded';suppressOwnedAuthoredFallback(rendered);root.updateWorldMatrix(true,true);for(let i=0;i<16;i++)assert(Math.abs(root.matrixWorld.elements[i]-beforeAsync.elements[i])<1e-10,'async mesh completion changed visual pose');
`,context);
"""
    result = subprocess.run(
        ["node", "-e", harness, str(ROOT / "workcell_studio_web/viewer/viewer.js"), str(web_scene), owner_id, str(expect_generated_pose_delta).lower()],
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
    assert "ready && dirty && valid_dirty_transforms && matching_scene" in source
    assert 'QStringLiteral("Unsaved Layout Edits: %1 (Web3D)")' in source
    for phase in ["save requested", "checking edits", "validation started", "validation failed", "saving", "saved", "reload"]:
        assert phase in source


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


def test_qt_writes_patch_atomically_then_runs_dry_run_before_confirmation_and_write():
    source = CONTROLLER.read_text(encoding="utf-8")
    assert "QSaveFile output(patch_path_)" in source
    assert "output.commit()" in source
    assert 'QStringLiteral("--dry-run-apply")' in source
    assert 'arguments << QStringLiteral("--write");' in source
    assert "WorkflowPhase::DryRun" in source
    assert "WorkflowPhase::Write" in source
    assert "Confirm Save Product View Layout" in source
    assert source.index("startWorkflow(WorkflowPhase::DryRun)") < source.index("Confirm Save Product View Layout")
    assert source.index("Confirm Save Product View Layout") < source.index("startWorkflow(WorkflowPhase::Write)")
    assert "QProcess::MergedChannels" in source
    assert "waitForFinished" not in source


def test_qt_logs_bounded_patch_summary_and_failed_subprocess_details():
    source = CONTROLLER.read_text(encoding="utf-8")
    for token in [
        "patch summary: scene_id=%1 dirty_edit_count=%2",
        "patch edit: item_id=%1 operation=%2",
        "persistence_source=%5",
        "exit_code=%2 subprocess_output=%3; Web3D edits preserved",
        "subprocess output truncated",
        "QProcess::MergedChannels",
    ]:
        assert token in source
    assert source.index("logPatchSummary(patch)") < source.index("validation started")


def test_successful_qt_save_reuses_backend_refreshes_and_reloads_product_view():
    controller = CONTROLLER.read_text(encoding="utf-8")
    workflow = WORKFLOW.read_text(encoding="utf-8")

    assert "scripts/run_workcell_studio_web_edit_workflow.py" in controller
    assert "scripts/apply_workcell_studio_web_scene_edit_patch.py" not in controller
    assert "scripts/validate_workcell_studio_web_scene_edit_patch.py" not in controller
    assert "request_post_save_product_view_refresh()" in controller
    assert "view_->reload()" not in controller
    assert "post_save_product_view_refresh_finished" in controller
    assert "matching regenerated Product View scene_ready" in controller

    assert 'write_cmd = [*dry_cmd, "--write", "--backup"]' in workflow
    assert "persistence verification" in workflow
    assert "_product_view_refresh_cmd" in workflow
    refresh = workflow.split("def _product_view_refresh_cmd", 1)[1].split("def _generated_summary_paths", 1)[0]
    assert "ensure_workcell_studio_web_scene_fresh.py" in refresh
    assert '"--stage-assets"' in refresh
    assert '"--force"' in refresh
    assert "Product View refresh result" in workflow


def test_post_save_refresh_renews_all_lifecycle_identities_and_rejects_stale_ready():
    controller = CONTROLLER.read_text(encoding="utf-8")
    header = (GUI / "scene_preview_widget.h").read_text(encoding="utf-8")
    preview = (GUI / "scene_preview_widget.cpp").read_text(encoding="utf-8")
    refresh = preview.split("int ScenePreviewWidget::request_post_save_product_view_refresh()", 1)[1].split(
        "bool ScenePreviewWidget::preview_payload_matches", 1
    )[0]

    assert "++preview_payload_revision_" in refresh
    assert "++preview_payload_generation_" in refresh
    assert 'request_embedded_web_product_view_refresh(true, QStringLiteral("post_save"))' in refresh
    assert "post_save_refresh_generation_ = embedded_web_request_generation_" in refresh
    assert "post_save_refresh_payload_revision_ = preview_payload_revision_" in refresh
    assert "post_save_product_view_refresh_finished" in header
    assert "revision != saved_reload_revision_" in controller
    assert "builder_revision != expected_builder_revision" in preview
    assert "navigation_token != embedded_web_navigation_token_" in preview


def test_post_save_refresh_failure_keeps_browser_edits_and_never_rewrites_yaml():
    controller = CONTROLLER.read_text(encoding="utf-8")
    preview = (GUI / "scene_preview_widget.cpp").read_text(encoding="utf-8")
    failure = preview.split("void ScenePreviewWidget::show_embedded_web_preparation_failure", 1)[1].split(
        "void ScenePreviewWidget::clear_embedded_editor_state_for_scene_handoff", 1
    )[0]

    assert "finish_post_save_product_view_refresh(identity, 0, false, detail)" in failure
    assert failure.index("finish_post_save_product_view_refresh") < failure.index("embedded_web_view_->setHtml")
    assert "persisted YAML was not written again and browser edits were preserved" in controller
    assert "choose Save Layout again; the YAML will not be rewritten automatically" in controller


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
    assert len(CONTROLLER.read_text(encoding="utf-8").splitlines()) < 630
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
        and path.relative_to(scene) != Path("generated/scene_visual_mesh_index.json")
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
        and path.relative_to(scene) != Path("generated/scene_visual_mesh_index.json")
    }
    assert protected_after == protected_before  # generated files, robot transforms, and hardware stay untouched

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
            and path.relative_to(scene) != Path("generated/scene_visual_mesh_index.json")
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
        # Re-export must retain the original generated mesh-origin/orientation
        # relationship rather than deriving a new local pose from the edited owner.
        for owner_id in before_visuals:
            assert after_visuals[owner_id]["owner_relative_visual_transform"] == before_visuals[owner_id]["owner_relative_visual_transform"]
            provenance = after_visuals[owner_id]["provenance"]["owner_relative_visual_transform"]
            assert provenance["source_owner_pose_provenance"] == "environment.yaml"
            assert provenance["generated_visual_pose"]
            _run_production_owner_binding_assertions(
                output / "ur5_2f_test.web_scene.json", owner_id, owner_id in edited_ids,
            )
        protected_after = {
            path.relative_to(scene): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in scene.rglob("*") if path.is_file() and path.name != "workcell_studio_layout.yaml" and ".bak" not in path.name
                and path.relative_to(scene) != Path("generated/scene_visual_mesh_index.json")
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
