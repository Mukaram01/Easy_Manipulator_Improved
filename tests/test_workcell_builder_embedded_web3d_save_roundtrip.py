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
WORKFLOW = ROOT / "scripts/run_workcell_studio_web_edit_workflow.py"
APPLICATOR = ROOT / "scripts/apply_workcell_studio_web_scene_edit_patch.py"
EXPORTER = ROOT / "scripts/export_workcell_studio_web_scene.py"


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
        "view_->url() == expected_url_",
        "sceneIdFromViewerUrl(view_->url()) == scene_id_",
        "Scene changed—reload required",
        "No changes",
        "Validation failed",
        "Saved",
    ]:
        assert token in source


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


def test_successful_qt_save_reuses_backend_refreshes_and_reloads_product_view():
    controller = CONTROLLER.read_text(encoding="utf-8")
    workflow = WORKFLOW.read_text(encoding="utf-8")

    assert "scripts/run_workcell_studio_web_edit_workflow.py" in controller
    assert "scripts/apply_workcell_studio_web_scene_edit_patch.py" not in controller
    assert "scripts/validate_workcell_studio_web_scene_edit_patch.py" not in controller
    assert "if (view_) view_->reload();" in controller

    assert 'write_cmd = [*dry_cmd, "--write", "--backup"]' in workflow
    assert "persistence verification" in workflow
    assert "_product_view_refresh_cmd" in workflow
    assert "export_workcell_studio_web_scene.py" in workflow
    assert '"--stage-assets"' not in workflow.split("def _product_view_refresh_cmd", 1)[1].split("def _generated_summary_paths", 1)[0]
    assert "Product View refresh result" in workflow


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
    assert len(CONTROLLER.read_text(encoding="utf-8").splitlines()) < 520
    assert len(WORKFLOW.read_text(encoding="utf-8").splitlines()) < 340
    assert len(APPLICATOR.read_text(encoding="utf-8").splitlines()) < 290


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
        "edits": [
            {"item_id": item_id, "operation": "update_transform", "editable_required": True,
             "locked_required": False, "old_transform": old, "new_transform": moved(old)}
            for item_id, old in (("target_bin_default", old_bin), ("place_zone_default", old_zone))
        ],
    }
    patch_path = output / "edit_patch.json"
    patch_path.write_text(json.dumps(patch), encoding="utf-8")
    assert {edit["item_id"] for edit in patch["edits"]} == {"target_bin_default", "place_zone_default"}

    layout_path = scene / "layout/workcell_studio_layout.yaml"
    layout_before = layout_path.read_bytes()
    protected_before = {
        path.relative_to(scene): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in scene.rglob("*") if path.is_file() and path != layout_path
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
    }
    assert protected_after == protected_before  # generated files, robot transforms, and hardware stay untouched

    layout = yaml.safe_load(layout_path.read_text(encoding="utf-8"))
    layout_items = {item["id"]: item for item in layout["items"]}
    after = json.loads((output / "ur5_2f_test.after.web_scene.json").read_text(encoding="utf-8"))
    reloaded = {item["id"]: item for item in _rendered_items(after)}
    for item_id, old in (("target_bin_default", old_bin), ("place_zone_default", old_zone)):
        expected = moved(old)
        assert layout_items[item_id]["pose"]["xyz"] == list(expected["pose"]["xyz"].values())
        assert _transform(reloaded[item_id])["pose"] == expected["pose"]


def test_executable_linked_edit_undo_redo_preserves_canonical_selection():
    viewer = ROOT / "workcell_studio_web/viewer/viewer.js"
    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const element=()=>({hidden:false,checked:false,disabled:false,textContent:'',innerHTML:'',value:'0',classList:{toggle(){},add(){},remove(){}},querySelector(){return null;},querySelectorAll(){return[];},addEventListener(){},setAttribute(){},appendChild(){},remove(){}});
const context={console,assert,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}}},document:{getElementById(){return element();},createElement(){return element();}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context); vm.runInContext(source+`
const object=(x,y,z)=>({position:{x,y,z,set(a,b,c){this.x=a;this.y=b;this.z=c;}},rotation:{x:0,y:0,z:0,set(a,b,c){this.x=a;this.y=b;this.z=c;}},scale:{x:1,y:1,z:1,set(a,b,c){this.x=a;this.y=b;this.z=c;}}});
const row=(id,z)=>({item:{id,editable:true,locked:false,source_layer:'editable_layout',render_policy:'primary',transform_group:'default_drop_destination'},object3d:object(.55,-.28,z),originalTransform:null});
const bin=row('target_bin_default',.2),zone=row('place_zone_default',.105); for(const item of [bin,zone]) item.originalTransform=transformFromObject(item.object3d);
state.objects=[bin,zone];state.sceneJson={scene:{id:'ur5_2f_test'}};state.dirtyTransforms=new Map();state.undoStack=[];state.redoStack=[];state.selected='target_bin_default';
updateLabels=()=>{};updateDirtyState=()=>{};emitDirtyChanged=()=>{};populateObjectList=()=>{};populateInspector=()=>{};
let savedTransform=cloneTransform(bin.originalTransform);savedTransform.pose.xyz.x+=.08;assert.strictEqual(markDirtyTransform(bin,savedTransform),true);assert.strictEqual(buildEditPatch().edits.length,2);
undoPreviewEdit();assert.strictEqual(state.selected,'target_bin_default');assert.strictEqual(state.dirtyTransforms.size,0);
redoPreviewEdit();assert.strictEqual(state.selected,'target_bin_default');assert.strictEqual(state.dirtyTransforms.size,2);
`,context);
"""
    result = subprocess.run(["node", "-e", harness, str(viewer)], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
