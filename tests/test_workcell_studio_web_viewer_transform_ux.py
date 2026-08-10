import subprocess
import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
VIEWER = REPO_ROOT / "workcell_studio_web" / "viewer" / "viewer.js"
INDEX = REPO_ROOT / "workcell_studio_web" / "viewer" / "index.html"


def _viewer_text():
    return VIEWER.read_text(encoding="utf-8")


def _index_text():
    return INDEX.read_text(encoding="utf-8")


def test_viewer_has_edit_mode_guard_for_editable_and_unlocked_items():
    viewer = _viewer_text()
    assert "function canEditItem" in viewer
    assert "item.locked === true" in viewer
    assert "item.editable !== true" in viewer
    assert "generated_authority_layer" in viewer
    assert "editAuthoritySource" in viewer
    assert "active_visual_source" in viewer
    assert "Edit mode active for editable/unlocked item" in viewer


def test_locked_generated_edit_message_exists():
    assert "Locked/generated preview item; edit source layout/environment instead." in _viewer_text()


def test_transform_gizmo_hook_is_loaded_without_build_system():
    viewer = _viewer_text()
    assert "TransformControls" in viewer
    assert "three/addons/controls/TransformControls.js" in viewer
    assert "attachTransformGizmo" in viewer
    # The editable owner may be attached directly or through the permitted
    # transient visual pivot.  In either case the authored item remains the
    # canonical edit/history/persistence identity.
    assert "gizmo.attach(rendered.object3d)" in viewer
    assert "state.gizmoPivot?.owner === rendered" in viewer
    assert "transformControls.object === state.gizmoPivot.group" in viewer


def test_transform_controls_lifecycle_regression_harness():
    """Execute the production TransformControls callbacks without a browser.

    This is callback-level Node/VM evidence, not proof of the complete Product
    View runtime.  The test double records the TransformControls contract while
    the assertions exercise the viewer's actual lifecycle callback bodies.
    """
    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const element=()=>({hidden:false,checked:false,disabled:false,value:'0',textContent:'',className:'',innerHTML:'',dataset:{},classList:{add(){},remove(){},toggle(){}},querySelector(){return null},querySelectorAll(){return[]},addEventListener(){},setAttribute(){},appendChild(){},remove(){},getBoundingClientRect(){return {left:0,top:0,width:100,height:100}}});
const context={console,assert,process,source,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}},addEventListener(){}},document:{getElementById(){return element()},querySelectorAll(){return[]},createElement(){return element()}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context);vm.runInContext(source+`
const copy=value=>JSON.parse(JSON.stringify(value));
const pose=(x=0,y=0,z=0,rx=.11,ry=-.22,rz=.33)=>({pose:{xyz:{x,y,z},rpy:{x:rx,y:ry,z:rz}},scale:{x:1,y:1,z:1}});
class TransformControlsDouble {
  constructor(){this.object=null;this.axis=null;this.rotationAngle=0;this.listeners={};this.translationSnap=undefined;this.rotationSnap=undefined;}
  addEventListener(type,listener){this.listeners[type]=listener}
  emit(type,event={}){assert(this.listeners[type],type+' listener missing');this.listeners[type](event)}
  attach(object){this.object=object} detach(){this.object=null} reset(){} setMode(){} setSpace(){}
  setTranslationSnap(value){this.translationSnap=value} setRotationSnap(value){this.rotationSnap=value}
}
const gizmo=new TransformControlsDouble();
const transformControls=gizmo;
const makeOwner=(id,start=pose())=>({item:{id,display_name:id,editable:true,locked:false,persistence_source:'layout/workcell_studio_layout.yaml'},object3d:{name:id,t:copy(start)}});
const bin=makeOwner('target_bin_default',pose(.4,-.2,.8));
const table=makeOwner('support_surface_table',pose(.55,0,.06));
const camera=makeOwner('realsense_overhead',pose(.35,0,.85,0,1.5708,0));
const lockedRobot={item:{id:'ur5_generated',editable:false,locked:true},object3d:{name:'ur5_generated',t:pose()}};
const lockedTool={item:{id:'robotiq_generated',editable:false,locked:true},object3d:{name:'robotiq_generated',t:pose()}};
const helpers=['fallback_edges','camera_frustum','selection_outline','urdf_descendant','transient_pivot'].map(id=>({item:{id,editable:false,locked:true,helper:true},object3d:{name:id,t:pose()}}));
state.objects=[bin,table,camera,lockedRobot,lockedTool,...helpers];state.selected=bin.item.id;state.editorMode='move';
state.web3dReadiness={state:'scene_ready',terminal:true,pending:new Set(),required:{},failed:false};
state.three={transformControls:gizmo,controls:{enabled:false},scene:{add(){}}};
let inspector=[],commits=[];
renderedById=id=>state.objects.find(record=>record.item.id===id)||null;
canonicalTransformOwner=value=>typeof value==='string'?renderedById(value):value;
selectionIsEditable=record=>Boolean(record?.item?.editable&&!record?.item?.locked);
canEditItem=item=>Boolean(item?.editable&&!item?.locked);
cloneTransform=copy;transformFromObject=object=>copy(object.t);applyTransformToObject=(object,value)=>{object.t=copy(value)};
captureTransformGroup=record=>new Map([[record.item.id,copy(record.object3d.t)]]);
linkedTransformChanges=(record,before,after)=>[{rendered:record,before:copy(before),after:copy(after)}];
applyTransformChanges=changes=>{for(const change of changes)change.rendered.object3d.t=copy(change.after);return true};
syncInspectorTransformFields=record=>inspector.push({id:record.item.id,value:copy(record.object3d.t)});updateLabels=()=>{};syncOrbitControlsForEditorMode=()=>{};
emitTransformCommitted=record=>commits.push(record.item.id);pushEditorEvent=()=>{};populateInspector=record=>inspector.push({id:record.item.id,value:copy(record.object3d.t)});refreshSelectionHighlight=()=>{};removeSelectionHighlight=()=>{};
resolveCanonicalPhysicalEditBinding=()=>null;refreshTransientGizmoPivot=()=>true;attachTransformGizmo=()=>{};
inspectionSelectionRendered=value=>value;explicitUiSelectionItemId=value=>value?.item?.id||'';isCanvasSelectableRendered=()=>true;resolveSelectionOwner=id=>({record:renderedById(id)});itemType=()=>'';sceneId=()=> 'harness';
snapTransform=value=>copy(value);isFiniteTransform=()=>true;
const callbackBlock=source.split("transformControls.addEventListener('dragging-changed', event => {",2)[1];
const listenerBoundary="    });"+String.fromCharCode(10)+"    transformControls.addEventListener('objectChange'";
const draggingBody=callbackBlock.split(listenerBoundary,1)[0];
const objectBody=callbackBlock.split("transformControls.addEventListener('objectChange', () => {",2)[1].split("    });"+String.fromCharCode(10)+"    controls.addEventListener('start'",1)[0];
gizmo.addEventListener('dragging-changed',eval('(event)=>{'+draggingBody+'}'));
gizmo.addEventListener('objectChange',eval('()=>{'+objectBody+'}'));

const ready=()=>assert.strictEqual(state.web3dReadiness.state,'scene_ready');
const exact=(actual,expected,label)=>assert.deepStrictEqual(actual,expected,label);
const begin=(owner,mode,axis)=>{state.selected=owner.item.id;state.editorMode=mode;gizmo.object=owner.object3d;gizmo.axis=axis;gizmo.rotationAngle=0;gizmo.emit('dragging-changed',{value:true})};
const end=()=>gizmo.emit('dragging-changed',{value:false});
const assertIdentity=owner=>{assert.strictEqual(state.selected,owner.item.id);assert.strictEqual(gizmo.object,owner.object3d);assert(!state.dirtyTransforms.has('transient_pivot'));for(const entry of state.undoStack)assert(entry.changes.every(change=>change.itemId===owner.item.id));};

// Move X/Y/Z: preview through objectChange, commit exactly once at drag end.
for(const [axis,key] of [['X','x'],['Y','y'],['Z','z']]){
  state.undoStack=[];state.redoStack=[];state.dirtyTransforms.clear();commits=[];inspector=[];bin.object3d.t=pose(.4,-.2,.8);
  const start=copy(bin.object3d.t);begin(bin,'move',axis);bin.object3d.t.pose.xyz[key]+=.10;gizmo.emit('objectChange');
  assert.strictEqual(bin.object3d.t.pose.xyz[key],start.pose.xyz[key]+.10);for(const other of ['x','y','z'].filter(v=>v!==key))assert.strictEqual(bin.object3d.t.pose.xyz[other],start.pose.xyz[other]);exact(bin.object3d.t.pose.rpy,start.pose.rpy,axis+' RPY');
  assert(inspector.length>0);assert.strictEqual(commits.length,0);assert.strictEqual(state.undoStack.length,0);end();assert.strictEqual(commits.length,1);assert.strictEqual(state.undoStack.length,1);assertIdentity(bin);ready();
}

// Roll/pitch/yaw isolation for both signs, with XYZ fixed.
for(const [axis,key] of [['X','x'],['Y','y'],['Z','z']])for(const angle of [.27,-.31]){
  state.undoStack=[];state.redoStack=[];state.dirtyTransforms.clear();commits=[];inspector=[];bin.object3d.t=pose(.4,-.2,.8);const start=copy(bin.object3d.t);
  begin(bin,'rotate',axis);gizmo.rotationAngle=angle;gizmo.emit('objectChange');const preview=copy(bin.object3d.t);assert.strictEqual(preview.pose.rpy[key],start.pose.rpy[key]+angle);for(const other of ['x','y','z'].filter(v=>v!==key))assert.strictEqual(preview.pose.rpy[other],start.pose.rpy[other]);exact(preview.pose.xyz,start.pose.xyz,axis+' XYZ');assert.strictEqual(commits.length,0);end();assert.strictEqual(commits.length,1);assert.strictEqual(state.undoStack.length,1);ready();
}

// Existing snap choices, including Off, are forwarded without adding UI values.
for(const value of [null,.01,.05,.10]){gizmo.setTranslationSnap(value);assert.strictEqual(gizmo.translationSnap,value)}
for(const value of [null,5*Math.PI/180,15*Math.PI/180]){gizmo.setRotationSnap(value);assert.strictEqual(gizmo.rotationSnap,value)}

// One transform is one history entry; undo/redo restore exact poses and inspector.
state.undoStack=[];state.redoStack=[];state.dirtyTransforms.clear();commits=[];inspector=[];bin.object3d.t=pose(.4,-.2,.8);const historyStart=copy(bin.object3d.t);begin(bin,'move','X');bin.object3d.t.pose.xyz.x+=.10;gizmo.emit('objectChange');const historyFinal=copy(bin.object3d.t);end();assert.strictEqual(state.undoStack.length,1);undoPreviewEdit();exact(bin.object3d.t,historyStart,'undo');exact(inspector.at(-1).value,historyStart,'undo inspector');redoPreviewEdit();exact(bin.object3d.t,historyFinal,'redo');exact(inspector.at(-1).value,historyFinal,'redo inspector');ready();

// Escape, pointercancel, and mode switch restore previews and commit nothing.
for(const [label,cancel] of [['Escape',()=>onEditorKeyDown({key:'Escape'})],['pointercancel',()=>onCanvasPointerCancel()],['mode',()=>setEditorMode('rotate')]]){
  state.undoStack=[];state.redoStack=[];state.dirtyTransforms.clear();commits=[];bin.object3d.t=pose(.4,-.2,.8);const start=copy(bin.object3d.t);begin(bin,'move','X');bin.object3d.t.pose.xyz.x+=.10;gizmo.emit('objectChange');cancel();exact(bin.object3d.t,start,label);assert.strictEqual(commits.length,0);assert.strictEqual(state.undoStack.length,0);ready();
}

// Canonical identity is stable for direct attachment or a transient visual pivot.
for(const owner of [bin,table,camera]){state.selected=owner.item.id;state.undoStack=[];state.dirtyTransforms.clear();const pivot={name:owner.item.id+'_transient_gizmo_pivot'};state.gizmoPivot=owner===bin?null:{owner,group:pivot};gizmo.object=state.gizmoPivot?.group||owner.object3d;const attachedOwner=gizmo.object===owner.object3d?owner:state.gizmoPivot.owner;assert.strictEqual(attachedOwner.item.id,owner.item.id);assert.strictEqual(state.selected,owner.item.id);assert.strictEqual(state.dirtyTransforms.has(pivot.name),false);ready();}
state.gizmoPivot=null;

// Locked/generated and helper visuals remain inspectable but never editable owners.
for(const record of [lockedRobot,lockedTool,...helpers]){state.selected=record.item.id;assert.strictEqual(renderedById(record.item.id),record);assert.strictEqual(selectionIsEditable(record),false);const before=copy(record.object3d.t);state.undoStack=[];state.dirtyTransforms.clear();commits=[];gizmo.object=record.object3d;gizmo.emit('dragging-changed',{value:true});gizmo.emit('objectChange');gizmo.emit('dragging-changed',{value:false});exact(record.object3d.t,before,record.item.id);assert.strictEqual(state.dirtyTransforms.size,0);assert.strictEqual(state.undoStack.length,0);assert.strictEqual(commits.length,0);ready();}

// CAD-style keyboard transforms use the same canonical commit/history path.
THREE={MathUtils:{degToRad:value=>value*Math.PI/180}};
const keyEvent=(code,options={})=>({code,key:code==='Escape'?'Escape':'',repeat:false,shiftKey:false,ctrlKey:false,metaKey:false,altKey:false,target:null,prevented:0,preventDefault(){this.prevented++},...options});
const keyboardStart=pose(.41,-.23,.87,.17,-.29,.38);
const resetKeyboard=()=>{state.selected=bin.item.id;state.undoStack=[];state.redoStack=[];state.dirtyTransforms.clear();state.gizmoDragStart=null;state.gizmoDragGroupStart=null;state.gizmoPivotDragStart=null;state.directMoveDrag=null;state.directRotateDrag=null;bin.object3d.t=copy(keyboardStart);commits=[];inspector=[];el.snapToggle.checked=false;el.translationSnap.value='0';el.rotationSnap.value='0'};
for(const [code,kind,component,direction] of [['KeyW','xyz','y',1],['KeyS','xyz','y',-1],['KeyA','xyz','x',1],['KeyD','xyz','x',-1],['PageUp','xyz','z',1],['PageDown','xyz','z',-1],['KeyQ','rpy','z',-1],['KeyE','rpy','z',1],['KeyR','rpy','y',1],['KeyF','rpy','y',-1],['KeyZ','rpy','x',-1],['KeyC','rpy','x',1]]){
  resetKeyboard();const event=keyEvent(code);onEditorKeyDown(event);const result=bin.object3d.t;const expected=copy(keyboardStart);expected.pose[kind][component]+=direction*(kind==='xyz'?.01:5*Math.PI/180);exact(result,expected,code+' isolated transform');assert.strictEqual(state.undoStack.length,1);assert.strictEqual(commits.length,1);assert.strictEqual(inspector.length,1);assert.strictEqual(state.selected,bin.item.id);assert.strictEqual(event.prevented,1);
}
resetKeyboard();el.snapToggle.checked=true;el.translationSnap.value='.05';onEditorKeyDown(keyEvent('KeyD'));assert.strictEqual(bin.object3d.t.pose.xyz.x,keyboardStart.pose.xyz.x-.05);
resetKeyboard();el.snapToggle.checked=true;el.rotationSnap.value='15';onEditorKeyDown(keyEvent('KeyE'));assert.strictEqual(bin.object3d.t.pose.rpy.z,keyboardStart.pose.rpy.z+15*Math.PI/180);
resetKeyboard();el.snapToggle.checked=true;el.translationSnap.value='.10';onEditorKeyDown(keyEvent('KeyW',{shiftKey:true}));assert.strictEqual(bin.object3d.t.pose.xyz.y,keyboardStart.pose.xyz.y+.001);assert.strictEqual(el.translationSnap.value,'.10');
resetKeyboard();el.snapToggle.checked=true;el.rotationSnap.value='15';onEditorKeyDown(keyEvent('KeyR',{shiftKey:true}));assert.strictEqual(bin.object3d.t.pose.rpy.y,keyboardStart.pose.rpy.y+Math.PI/180);assert.strictEqual(el.rotationSnap.value,'15');

// One key transaction is exactly undoable/redoable through every scene shortcut.
resetKeyboard();onEditorKeyDown(keyEvent('KeyD'));const keyboardFinal=copy(bin.object3d.t);const undoEvent=keyEvent('KeyZ',{ctrlKey:true});onEditorKeyDown(undoEvent);exact(bin.object3d.t,keyboardStart,'keyboard undo');assert.strictEqual(undoEvent.prevented,1);const redoY=keyEvent('KeyY',{ctrlKey:true});onEditorKeyDown(redoY);exact(bin.object3d.t,keyboardFinal,'keyboard Ctrl+Y redo');assert.strictEqual(redoY.prevented,1);onEditorKeyDown(keyEvent('KeyZ',{ctrlKey:true}));const redoShiftZ=keyEvent('KeyZ',{ctrlKey:true,shiftKey:true});onEditorKeyDown(redoShiftZ);exact(bin.object3d.t,keyboardFinal,'keyboard Ctrl+Shift+Z redo');assert.strictEqual(redoShiftZ.prevented,1);

// Repeats, active mouse transactions, locked/helpers, and editable controls are inert.
const assertKeyboardIgnored=(label,event,record=bin,armDrag=false)=>{resetKeyboard();state.selected=record.item.id;record.object3d.t=copy(keyboardStart);if(armDrag)state.gizmoDragStart=copy(keyboardStart);onEditorKeyDown(event);exact(record.object3d.t,keyboardStart,label);assert.strictEqual(state.undoStack.length,0);assert.strictEqual(commits.length,0);assert.strictEqual(event.prevented,0)};
assertKeyboardIgnored('repeat',keyEvent('KeyD',{repeat:true}));
assertKeyboardIgnored('active TransformControls drag',keyEvent('KeyD'),bin,true);
for(const record of [lockedRobot,lockedTool,helpers[0]])assertKeyboardIgnored(record.item.id,keyEvent('KeyD'),record);
for(const target of [{tagName:'INPUT'},{tagName:'TEXTAREA'},{tagName:'DIV',isContentEditable:true}])assertKeyboardIgnored(target.tagName+' focus',keyEvent('KeyD',{target}));
resetKeyboard();state.undoStack.push({changes:[]});const inputUndo=keyEvent('KeyZ',{ctrlKey:true,target:{tagName:'INPUT'}});onEditorKeyDown(inputUndo);assert.strictEqual(state.undoStack.length,1);assert.strictEqual(inputUndo.prevented,0);
`,context);
"""
    subprocess.run(
        ["node", "-e", harness, str(VIEWER)],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )


def test_snap_controls_exist():
    index = _index_text()
    viewer = _viewer_text()
    assert 'id="snap-toggle"' in index
    assert 'id="translation-snap"' in index
    assert 'id="rotation-snap"' in index
    assert "setTranslationSnap" in viewer
    assert "setRotationSnap" in viewer
    assert "snapTransform" in viewer


def test_undo_redo_clear_preview_edit_controls_exist():
    index = _index_text()
    viewer = _viewer_text()
    assert "Undo" in index
    assert "Redo" in index
    assert "Clear Preview Edits" in index
    assert "undoPreviewEdit" in viewer
    assert "redoPreviewEdit" in viewer
    assert "clearPreviewEdits" in viewer
    assert "undoStack" in viewer
    assert "redoStack" in viewer


def test_patch_export_still_exists_and_remains_preview_only():
    index = _index_text()
    viewer = _viewer_text()
    assert "Export Edit Patch" in index
    assert "buildEditPatch" in viewer
    assert "schema_version: EDIT_PATCH_SCHEMA_VERSION" in viewer
    assert "Preview-only browser transform edit. Source scene files were not modified." in viewer
    assert "canEditItem" in viewer
    assert "item.editable !== true" in viewer
    assert "item.locked === true" in viewer


def test_no_direct_yaml_write_or_browser_apply_logic_added():
    combined = (_viewer_text() + "\n" + _index_text()).lower()
    forbidden = ["environment.yaml", "workcell_studio_layout.yaml", "yaml.safe_dump", "js-yaml", "apply_workcell_studio_web_scene_edit_patch"]
    for token in forbidden:
        assert token not in combined


def test_no_generated_scene_json_committed():
    tracked = subprocess.run(
        ["git", "ls-files", "*.web_scene.json", "*web_scene.json", "*.edit_patch.json", "*edit_patch.json"],
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        check=True,
    ).stdout.splitlines()
    assert not tracked


def test_status_summary_reports_loaded_fallback_and_failed_meshes():
    index = _index_text()
    viewer = _viewer_text()
    assert "meshLoadedCount" in viewer
    assert "fallbackCount" in viewer
    assert "meshFailedCount" in viewer
    assert "render_status === 'mesh_loaded'" in viewer
    assert "isRuntimeFallbackStatus" in viewer
    assert "isMissingOrFailedMeshStatus" in viewer
    assert 'data-summary-field="mesh-loaded-count"' in index
    assert 'data-summary-field="fallback-count"' in index
    assert 'data-summary-field="mesh-failed-count"' in index


def test_camera_fit_constants_cover_ur5_2f_sized_workcell_without_clipping():
    viewer = _viewer_text()
    min_radius = float(re.search(r"const MIN_FRAME_RADIUS = ([0-9.]+);", viewer).group(1))
    distance_multiplier = float(re.search(r"const FRAME_DISTANCE_MULTIPLIER = ([0-9.]+);", viewer).group(1))
    near_formula = re.search(r"camera\.near = Math\.max\(0\.01, radius / ([0-9.]+)\);", viewer)
    far_formula = re.search(r"camera\.far = Math\.max\(100, distance \+ radius \* ([0-9.]+)\);", viewer)
    assert near_formula
    assert far_formula

    # Roughly ur5_2f_test-sized bounds: workbench, robot, camera, and bins fit in
    # about a 2.0 m x 1.6 m x 0.85 m envelope. The fitted near/far values should
    # have a wide margin around the workcell rather than clipping table/camera.
    span = (2.0, 1.6, 0.85)
    scene_radius = max((sum((axis / 2.0) ** 2 for axis in span)) ** 0.5, min_radius)
    distance = max(scene_radius * distance_multiplier, min_radius * distance_multiplier)
    near = max(0.01, scene_radius / float(near_formula.group(1)))
    far = max(100.0, distance + scene_radius * float(far_formula.group(1)))

    assert near <= 0.02
    assert far >= 100.0
    assert far > distance + scene_radius
    assert far / near >= 5000
