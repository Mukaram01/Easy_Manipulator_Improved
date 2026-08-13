from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEWER = (ROOT / "workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")
INDEX = (ROOT / "workcell_studio_web/viewer/index.html").read_text(encoding="utf-8")
STYLE = (ROOT / "workcell_studio_web/viewer/style.css").read_text(encoding="utf-8")
CPP = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")
HDR = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.h").read_text(encoding="utf-8")
VIEWPORT_CPP = (ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")


def test_browser_editor_api_v1_contract_and_bounded_events():
    assert "window.__WORKCELL_EDITOR_API_V1__" in VIEWER
    for method in ["getState", "selectItem", "clearSelection", "setMode", "setSnap", "undo", "redo", "fitScene", "getEditPatch", "drainEvents"]:
        assert f"{method}:" in VIEWER
    for field in ["ready", "sceneId", "selectedItemId", "uiSelectionItemId", "selectedItemType", "selectedEditable", "dirty", "dirtyCount", "canUndo", "canRedo", "mode", "error"]:
        assert field in VIEWER
    for event in ["selection_changed", "dirty_changed", "transform_committed", "editor_error"]:
        assert event in VIEWER
    assert "state.editorEvents.length > 100" in VIEWER
    assert "splice(0, state.editorEvents.length - 100)" in VIEWER


def test_browser_placement_api_raycast_state_and_single_request_contract():
    import subprocess

    for method in ["placementPointFromViewport", "armPlacement", "cancelPlacement", "getPlacementState"]:
        assert f"{method}:" in VIEWER

    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const canvas={style:{},classList:{add(){},remove(){}},setAttribute(){},getBoundingClientRect(){return {left:10,top:20,width:100,height:80}}};
const element=()=>({hidden:false,checked:false,disabled:false,textContent:'',className:'',innerHTML:'',dataset:{},style:{},classList:{add(){},remove(){},toggle(){}},querySelector(){return null},querySelectorAll(){return[]},addEventListener(){},setAttribute(){},appendChild(){},remove(){},getBoundingClientRect(){return {left:0,top:0,width:100,height:100}}});
const document={getElementById(id){return id==='scene-canvas'?canvas:element()},querySelectorAll(){return[]},createElement(){return element()}};
const context={console,assert,process,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}},addEventListener(){}},document,URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context);
vm.runInContext(source+`
class Vector3 { constructor(x=0,y=0,z=0){this.x=x;this.y=y;this.z=z} }
class Plane { constructor(normal,constant){this.normal=normal;this.constant=constant} }
THREE={Vector3,Plane};
const ray={intersectPlane(plane,out){out.x=1.25;out.y=-.5;out.z=0;return out}};
const raycaster={ray,hits:[],setFromCamera(pointer,camera){this.last={x:pointer.x,y:pointer.y,camera}},intersectObjects(){return this.hits}};
state.three={pointer:{x:0,y:0},camera:{id:'camera'},raycaster};
const helperItem={id:'warning',role:'warning_marker',source_layer:'debug_overlay',debug_overlay:true};
const helperNode={name:'warning_helper',userData:{item:helperItem},material:{},visible:true,parent:null};
const helper={item:helperItem,object3d:helperNode};
state.objects=[helper];

assert.strictEqual(placementPointFromViewport(null),null);
assert.strictEqual(placementPointFromViewport({clientX:NaN,clientY:30}),null);
assert.strictEqual(placementPointFromViewport({clientX:Infinity,clientY:30}),null);
assert.strictEqual(placementPointFromViewport({clientX:9,clientY:30}),null);
assert.strictEqual(placementPointFromViewport({clientX:110,clientY:30}),null);
raycaster.hits=[];
assert.deepStrictEqual(placementPointFromViewport({clientX:60,clientY:60}),{x:1.25,y:-.5,z:0});
raycaster.hits=[{object:helperNode,point:new Vector3(9,9,9)}];
assert.deepStrictEqual(placementPointFromViewport({clientX:60,clientY:60}),{x:1.25,y:-.5,z:0},'helper hit must be transparent to ground fallback');

const tableItem={id:'table',role:'support_surface',category:'table'};
const tableNode={name:'tabletop',userData:{item:tableItem},material:{wireframe:false},visible:true,parent:null};
state.objects.push({item:tableItem,object3d:tableNode});
raycaster.hits=[{object:helperNode,point:new Vector3(9,9,9)},{object:tableNode,point:new Vector3(.55,.10,.42)}];
assert.deepStrictEqual(placementPointFromViewport({clientX:60,clientY:60}),{x:.55,y:.10,z:.42});

assert.deepStrictEqual(armPlacement(),{armed:true,persistent:false});
assert.deepStrictEqual(getPlacementState(),{armed:true,persistent:false});
onEditorKeyDown({key:'Escape',preventDefault(){}});
assert.deepStrictEqual(getPlacementState(),{armed:false,persistent:false});
assert.strictEqual(state.editorEvents.filter(event=>event.type==='placement_requested').length,0);

armPlacement();
onCanvasPointerDown({button:0,clientX:60,clientY:60,preventDefault(){},stopPropagation(){}});
assert.strictEqual(state.editorEvents.filter(event=>event.type==='placement_requested').length,1);
const request=state.editorEvents.find(event=>event.type==='placement_requested');
assert.deepStrictEqual({x:request.x,y:request.y,z:request.z},{x:.55,y:.10,z:.42});
assert.deepStrictEqual(getPlacementState(),{armed:false,persistent:false});
`,context);
"""
    # Avoid depending on a browser runner while still executing viewer.js itself.
    subprocess.run(
        ["node", "-e", harness, str(ROOT / "workcell_studio_web/viewer/viewer.js")],
        cwd=ROOT, check=True, capture_output=True, text=True,
    )


def test_webengine_asset_drop_uses_browser_placement_contract_and_typed_xyz_path():
    drop_handler = CPP.split("class AssetDropWebEngineView", 1)[1].split("private:", 1)[0]

    assert "window.__WORKCELL_EDITOR_API_V1__" in drop_handler
    assert "api.placementPointFromViewport({clientX:%1,clientY:%2})" in drop_handler
    assert "pointerToWorldPlane" not in drop_handler
    assert "[hit.x,hit.y,hit.z].every(Number.isFinite)" in drop_handler
    assert "std::isfinite(x)" in drop_handler
    assert "std::isfinite(y)" in drop_handler
    assert "std::isfinite(z)" in drop_handler
    assert "placement_requested(asset_id, x, y, z, configure_transform)" in drop_handler

    forwarding = CPP.split("asset_drop_web_view->placement_requested", 1)[1].split("embedded_web_view_->setObjectName", 1)[0]
    assert "emit asset_placement_requested(asset_id, x, y, z, configure_transform)" in forwarding

    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    placement = main.split("bool MainWindow::place_catalog_asset_at_world_position", 1)[1].split("bool MainWindow::configure_asset_placement_transform", 1)[0]
    assert "armed_asset_x_m_ = world_x_m" in placement
    assert "armed_asset_y_m_ = world_y_m" in placement
    assert "armed_asset_z_m_ = world_z_m" in placement
    assert "commit_armed_asset_placement(QPointF(world_x_m * 100.0, world_y_m * 100.0))" in placement


def test_browser_api_reuses_existing_editor_functions_and_preserves_locks():
    for token in ["selectObject(String(id || ''))", "refreshGizmoSnap()", "undoPreviewEdit()", "redoPreviewEdit()", "resetView()", "buildEditPatch()"]:
        assert token in VIEWER
    assert "function canEditItem(item)" in VIEWER
    assert "function editAuthorityForItem(item)" in VIEWER
    assert "item.locked === true" in VIEWER
    assert "item.editable !== true" in VIEWER
    assert "active_visual_source and renderer/mesh provenance deliberately do not" in VIEWER
    assert "emitTransformCommitted(rendered)" in VIEWER
    assert "dragging-changed" in VIEWER


def test_embedded_mode_hides_standalone_browser_ui_only_when_requested():
    assert "params.get('embedded') === '1'" in VIEWER
    assert "document.body.classList.add('embedded-mode')" in VIEWER
    assert "body.embedded-mode .topbar" in STYLE
    assert "body.embedded-mode .object-panel" in STYLE
    assert "body.embedded-mode .details-panel" in STYLE
    assert "body.embedded-mode .toolbar" in STYLE
    assert "body.embedded-mode .app-shell" in STYLE
    assert "class=\"topbar\"" in INDEX


def test_qt_loads_embedded_view_and_routes_controls_without_reload():
    assert 'viewer_query.addQueryItem(QStringLiteral("embedded"), QStringLiteral("1"))' in CPP
    for helper in ["run_embedded_editor_command", "poll_embedded_editor_events", "apply_embedded_editor_state", "embedded_snap_command"]:
        assert helper in CPP and helper in HDR
    for command in ["setMode", "setSnap", "fitScene", "undo()", "redo()", "selectItem"]:
        assert command in CPP
    assert "QTimer::singleShot(200, this, [this, identity]()" in CPP
    assert "refresh_embedded_web_product_view();" not in CPP.split('connect(gizmo_mode_selector_', 1)[1].split('connect(mesh_preview_mode_selector_', 1)[0]


def test_outer_qt_authoring_controls_use_typed_web3d_bridge():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    for method in ["set_authoring_mode", "undo_authoring_edit", "redo_authoring_edit", "request_authoring_save"]:
        assert method in HDR
        assert f"scene_preview_widget_->{method}" in main
    assert "authoring_mode_changed" in CPP
    assert "authoring_mode_changed" in main
    assert 'make_primary_button("Rotate")' in main


def test_authoring_mode_round_trip_cleanup_and_selection_preservation():
    mode_body = VIEWER.split("function setEditorMode", 1)[1].split("function setEditorSnap", 1)[0]
    for mode in ["'move'", "'rotate'", "'select'"]:
        assert mode in mode_body
    assert "cancelActiveTransformOperation('Mode changed')" in mode_body
    assert "gizmo.reset?.()" in mode_body
    assert "gizmo.detach()" in mode_body
    assert "syncOrbitControlsForEditorMode()" in mode_body
    assert "state.selected =" not in mode_body
    assert "return state.editorMode" in mode_body


def test_transform_controls_cancellation_restores_canonical_state_without_commit_side_effects():
    import subprocess

    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const element=()=>({hidden:false,checked:false,disabled:false,textContent:'',className:'',innerHTML:'',dataset:{},classList:{add(){},remove(){},toggle(){}},querySelector(){return null},querySelectorAll(){return[]},addEventListener(){},setAttribute(){},appendChild(){},remove(){},getBoundingClientRect(){return {left:0,top:0,width:100,height:100}}});
const context={console,assert,process,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}},addEventListener(){}},document:{getElementById(){return element()},querySelectorAll(){return[]},createElement(){return element()}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context);
vm.runInContext(source+`
const copy=value=>JSON.parse(JSON.stringify(value));
const transform=(x,r=0)=>({pose:{xyz:{x,y:0,z:0},rpy:{x:0,y:0,z:r}},scale:{x:1,y:1,z:1}});
const owner={item:{id:'old',editable:true,locked:false,display_name:'Old'},object3d:{t:transform(1)}};
const member={item:{id:'member',editable:true,locked:false},object3d:{t:transform(2)}};
const next={item:{id:'next',editable:true,locked:false,display_name:'Next'},object3d:{t:transform(3)}};
state.objects=[owner,member,next]; state.selected='old'; state.editorMode='move';
let committed=0,dirtyChanged=0;
state.three={controls:{enabled:false},scene:{add(){}},transformControls:{object:owner.object3d,axis:'Z',rotationAngle:.5,setMode(){},setSpace(){},setTranslationSnap(){},setRotationSnap(){},attach(object){this.object=object},detach(){this.object=null},reset(){ if(state.cancellingTransformOperation){ /* models synchronous TransformControls re-entry */ } }}};
renderedById=id=>state.objects.find(value=>value.item.id===id)||null;
canonicalTransformOwner=value=>typeof value==='string'?renderedById(value):value;
selectionIsEditable=value=>Boolean(value?.item?.editable);
canEditItem=item=>Boolean(item?.editable);
cloneTransform=copy; transformFromObject=object=>copy(object.t); applyTransformToObject=(object,value)=>{object.t=copy(value)};
applyTransformChanges=changes=>changes.forEach(change=>{if(change.rendered)change.rendered.object3d.t=copy(change.after)});
syncInspectorTransformFields=()=>{}; updateLabels=()=>{}; pushEditorEvent=(type)=>{if(type==='dirty_changed')dirtyChanged++};
emitTransformCommitted=()=>committed++; refreshTransientGizmoPivot=()=>true; resolveCanonicalPhysicalEditBinding=()=>null;
populateInspector=()=>{}; attachTransformGizmo=()=>{}; refreshSelectionHighlight=()=>{}; removeSelectionHighlight=()=>{};
inspectionSelectionRendered=value=>value; explicitUiSelectionItemId=value=>value?.item?.id||''; isCanvasSelectableRendered=()=>true;
resolveSelectionOwner=id=>({record:renderedById(id)}); itemType=()=>''; sceneId=()=> 'scene';
const startOwner=transform(1),startMember=transform(2),previewOwner=transform(9,.7),previewMember=transform(8,.4);
function arm(kind,{pivot=false}={}){
  owner.object3d.t=copy(previewOwner); member.object3d.t=copy(previewMember); state.selected='old';
  state.editorMode=kind; state.gizmoDragStart=copy(startOwner); state.gizmoDragGroupStart=new Map([['old',copy(startOwner)],['member',copy(startMember)]]);
  state.gizmoPivotDragStart=pivot?{axis:'Z'}:null; state.directMoveDrag=null; state.directRotateDrag=null;
  if(kind==='rotate'&&!pivot)state.directRotateDrag={itemId:'old',start:copy(startOwner),groupStart:state.gizmoDragGroupStart,last:copy(previewOwner)};
  if(pivot)state.gizmoPivot={owner,group:{}}; else state.gizmoPivot=null;
}
function verify(label,undo,redo,dirty){
  assert.deepStrictEqual(owner.object3d.t,startOwner,label+' owner'); assert.deepStrictEqual(member.object3d.t,startMember,label+' member');
  assert.strictEqual(state.undoStack.length,undo);assert.strictEqual(state.redoStack.length,redo);assert.strictEqual(JSON.stringify([...state.dirtyTransforms]),dirty);
  assert.strictEqual(committed,0);assert.strictEqual(dirtyChanged,0);
}
state.undoStack=[{kept:true}];state.redoStack=[{kept:true}];state.dirtyTransforms=new Map([['existing',{kept:true}]]);
const undo=state.undoStack.length,redo=state.redoStack.length,dirty=JSON.stringify([...state.dirtyTransforms]);
for(const kind of ['move','rotate']){
  arm(kind);onEditorKeyDown({key:'Escape'});verify(kind+' Escape',undo,redo,dirty);
  arm(kind);onCanvasPointerCancel();verify(kind+' pointercancel',undo,redo,dirty);
  arm(kind);setEditorMode(kind==='move'?'rotate':'select');verify(kind+' mode',undo,redo,dirty);assert.strictEqual(state.selected,'old');
}
arm('move');selectObject('next');verify('selection',undo,redo,dirty);assert.strictEqual(state.selected,'next');
arm('rotate',{pivot:true});onEditorKeyDown({key:'Escape'});verify('transient pivot',undo,redo,dirty);
`,context);
"""
    subprocess.run(
        ["node", "-e", harness, str(ROOT / "workcell_studio_web/viewer/viewer.js")],
        cwd=ROOT, check=True, capture_output=True, text=True,
    )


def test_transform_controls_zero_delta_completion_is_an_exact_no_op():
    import subprocess

    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const element=()=>({hidden:false,checked:false,disabled:false,textContent:'',className:'',innerHTML:'',dataset:{},classList:{add(){},remove(){},toggle(){}},querySelector(){return null},querySelectorAll(){return[]},addEventListener(){},setAttribute(){},appendChild(){},remove(){},getBoundingClientRect(){return {left:0,top:0,width:100,height:100}}});
const context={console,assert,process,source,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}},addEventListener(){}},document:{getElementById(){return element()},querySelectorAll(){return[]},createElement(){return element()}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context);
vm.runInContext(source+`
const copy=value=>JSON.parse(JSON.stringify(value));
const transform=(x,y,z,rx,ry,rz)=>({pose:{xyz:{x,y,z},rpy:{x:rx,y:ry,z:rz}},scale:{x:1,y:1,z:1}});
const ownerStart=transform(.4,-.2,.8,.1,-.3,.5),memberStart=transform(1.4,.7,.2,-.2,.4,-.6);
const owner={item:{id:'owner',editable:true,locked:false,transform_group:'group',display_name:'Owner'},object3d:{t:copy(ownerStart)}};
const member={item:{id:'member',editable:true,locked:false,transform_group:'group'},object3d:{t:copy(memberStart)}};
state.objects=[owner,member];state.selected='owner';
const transformControls={object:owner.object3d,axis:'X',rotationAngle:0};
state.three={controls:{enabled:false},transformControls};
let marked=0,committed=0,dirtyChanged=0;
renderedById=id=>state.objects.find(value=>value.item.id===id)||null;
canonicalTransformOwner=()=>owner;canEditItem=item=>Boolean(item?.editable);selectionIsEditable=value=>value===owner;
cloneTransform=copy;transformFromObject=object=>copy(object.t);applyTransformToObject=(object,value)=>{object.t=copy(value)};
captureTransformGroup=()=>new Map([['owner',copy(owner.object3d.t)],['member',copy(member.object3d.t)]]);
linkedTransformChanges=(rendered,before,after,starts)=>[{rendered:owner,before:copy(before),after:copy(after)},{rendered:member,before:copy(starts.get('member')),after:copy(starts.get('member'))}];
applyTransformChanges=changes=>{for(const change of changes)change.rendered.object3d.t=copy(change.after);return true};
syncInspectorTransformFields=()=>{};updateLabels=()=>{};syncOrbitControlsForEditorMode=()=>{};isFiniteTransform=()=>true;
markDirtyTransform=()=>{marked++;return true};emitTransformCommitted=()=>{committed++};pushEditorEvent=type=>{if(type==='dirty_changed')dirtyChanged++};
const listenerSource=source.split("transformControls.addEventListener('dragging-changed', event => {",2)[1].split("    });\\n    transformControls.addEventListener('objectChange'",1)[0];
const completeTransformControlsDrag=eval('(event)=>{'+listenerSource+'}');
state.undoStack=[{kept:'undo'}];state.redoStack=[{kept:'redo'}];state.dirtyTransforms=new Map([['existing',{kept:true}]]);
const baseline={undo:JSON.stringify(state.undoStack),redo:JSON.stringify(state.redoStack),dirty:JSON.stringify([...state.dirtyTransforms])};
function resetPose(){owner.object3d.t=copy(ownerStart);member.object3d.t=copy(memberStart)}
function verify(label){
  assert.deepStrictEqual(owner.object3d.t,ownerStart,label+' owner pose');assert.deepStrictEqual(member.object3d.t,memberStart,label+' linked pose');
  assert.strictEqual(JSON.stringify(state.undoStack),baseline.undo,label+' undo');assert.strictEqual(JSON.stringify(state.redoStack),baseline.redo,label+' redo');assert.strictEqual(JSON.stringify([...state.dirtyTransforms]),baseline.dirty,label+' dirty transforms');
  assert.strictEqual(marked,0,label+' markDirtyTransform');assert.strictEqual(committed,0,label+' transform_committed');assert.strictEqual(dirtyChanged,0,label+' dirty transition');
}
for(const axis of ['X','Y','Z']){
  resetPose();state.editorMode='move';state.gizmoPivot=null;transformControls.object=owner.object3d;transformControls.axis=axis;
  completeTransformControlsDrag({value:true});completeTransformControlsDrag({value:false});verify('Move '+axis);
}
for(const axis of ['X','Y','Z']){
  resetPose();state.editorMode='rotate';state.gizmoPivot=null;transformControls.object=owner.object3d;transformControls.axis=axis;transformControls.rotationAngle=0;
  completeTransformControlsDrag({value:true});completeTransformControlsDrag({value:false});verify('Rotate '+axis);
}
resetPose();state.editorMode='rotate';transformControls.axis='Y';transformControls.rotationAngle=0;
const identityMatrix={clone(){return this},invert(){return this},multiply(){return this}};
owner.object3d.updateWorldMatrix=()=>{};owner.object3d.matrixWorld=identityMatrix;
const pivotGroup={updateWorldMatrix(){},matrixWorld:identityMatrix};state.gizmoPivot={owner,group:pivotGroup};transformControls.object=pivotGroup;
completeTransformControlsDrag({value:true});completeTransformControlsDrag({value:false});verify('transient pivot Rotate Y');
`,context);
"""
    subprocess.run(
        ["node", "-e", harness, str(ROOT / "workcell_studio_web/viewer/viewer.js")],
        cwd=ROOT, check=True, capture_output=True, text=True,
    )


def test_orbit_controls_remain_available_across_modes_except_during_gizmo_drags():
    import subprocess

    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const element=()=>({hidden:false,checked:false,disabled:false,textContent:'',className:'',innerHTML:'',dataset:{},classList:{add(){},remove(){},toggle(){}},querySelector(){return null},querySelectorAll(){return[]},addEventListener(){},setAttribute(){},appendChild(){},remove(){},getBoundingClientRect(){return {left:0,top:0,width:100,height:100}},setPointerCapture(){},releasePointerCapture(){}});
const context={console,assert,process,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}},addEventListener(){}},document:{getElementById(){return element()},querySelectorAll(){return[]},createElement(){return element()}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context);
vm.runInContext(source+`
(async()=>{
const copy=value=>JSON.parse(JSON.stringify(value));
const start={pose:{xyz:{x:0,y:0,z:0},rpy:{r:0,p:0,y:0}},scale:{x:1,y:1,z:1}};
const rendered={item:{id:'editable',editable:true,locked:false,display_name:'Editable'},object3d:{}};
state.objects=[rendered]; state.selected='editable'; state.three={controls:{enabled:true},transformControls:{axis:'Z',rotationAngle:0,dragging:false,setMode(){},setSpace(){},attach(){},detach(){},reset(){this.dragging=false}},pointer:{},raycaster:{}};
cloneTransform=copy; transformFromObject=()=>copy(start); selectionIsEditable=value=>value===rendered; renderedById=id=>id==='editable'?rendered:null; captureTransformGroup=()=>new Map([['editable',copy(start)]]); applyTransformChanges=()=>{}; syncInspectorTransformFields=()=>{}; linkedTransformChanges=()=>[]; markDirtyTransform=()=>true; emitTransformCommitted=()=>{}; pushEditorEvent=()=>{}; showError=message=>{throw new Error(message)}; updateLabels=()=>{}; attachTransformGizmo=()=>{}; canonicalTransformOwner=()=>rendered; detachTransformGizmo=()=>{}; applyTransformToObject=()=>{}; sameTransform=()=>false; snapTransform=value=>copy(value); isFiniteTransform=()=>true;

for(const mode of ['select','move','rotate']){setEditorMode(mode);assert.strictEqual(state.three.controls.enabled,true,mode+' must allow CAD camera navigation while idle')}
for(const mode of ['move','rotate']){
  setEditorMode(mode);state.three.transformControls.dragging=true;syncOrbitControlsForEditorMode();assert.strictEqual(state.three.controls.enabled,false,mode+' gizmo drag must own the pointer exclusively');
  state.three.transformControls.dragging=false;syncOrbitControlsForEditorMode();assert.strictEqual(state.three.controls.enabled,true,mode+' drag completion must restore camera navigation');
  state.three.transformControls.dragging=true;state.gizmoDragStart=copy(start);state.gizmoDragGroupStart=new Map([['editable',copy(start)]]);onCanvasPointerCancel();assert.strictEqual(state.three.controls.enabled,true,mode+' pointercancel must restore camera navigation');
  state.three.transformControls.dragging=true;state.gizmoDragStart=copy(start);state.gizmoDragGroupStart=new Map([['editable',copy(start)]]);onEditorKeyDown({key:'Escape'});assert.strictEqual(state.three.controls.enabled,true,mode+' Escape must restore camera navigation');
}

validateSceneJson=()=>[]; renderScene=()=>{}; refreshWarnings=()=>{}; renderSceneSummary=()=>{}; beginInitialCameraFitForCurrentScene=()=>{}; emitWeb3dReadinessState=()=>{}; clearError=()=>{};
state.three.controls={enabled:false}; await loadFile({name:'replacement.json',text:async()=>'{"scene_id":"replacement"}'}); assert.strictEqual(state.three.controls.enabled,true,'scene replacement must restore idle OrbitControls');
for(const mode of ['select','move','rotate']){setEditorMode(mode);state.three.controls={enabled:false};syncOrbitControlsForEditorMode();assert.strictEqual(state.three.controls.enabled,true,'new OrbitControls must be available in '+mode)}
})().catch(error=>{console.error(error);process.exitCode=1});
`,context);
"""
    subprocess.run(
        ["node", "-e", harness, str(ROOT / "workcell_studio_web/viewer/viewer.js")],
        cwd=ROOT, check=True, capture_output=True, text=True,
    )

def test_browser_mode_state_synchronizes_all_qt_controls_without_stale_callbacks():
    apply_body = CPP.split("void ScenePreviewWidget::apply_embedded_editor_state", 1)[1].split("void ScenePreviewWidget::poll_embedded_editor_events", 1)[0]
    assert "QSignalBlocker blocker(gizmo_mode_selector_)" in apply_body
    assert "QSignalBlocker blocker(interaction_mode_selector_)" in apply_body
    assert "emit authoring_mode_changed(mode)" in apply_body
    assert "state_request_token != embedded_editor_state_request_token_" in CPP
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    for control in ["select_mode_button", "move_mode_button", "rotate_mode_button"]:
        assert f"QSignalBlocker {control.removesuffix('_mode_button')}_blocker({control})" in main


def test_move_mode_body_click_only_selects_without_starting_direct_drag():
    handler = VIEWER.split("function onCanvasPointerDown", 1)[1].split("function onCanvasPointerMove", 1)[0]
    assert "pickObject(event)" in handler
    assert "event.button === 0" in handler
    assert "beginDirectMoveDrag" not in handler
    assert "updateDirectMoveDrag" not in VIEWER.split("function onCanvasPointerMove", 1)[1].split("function onCanvasPointerUp", 1)[0]
    assert "finishDirectMoveDrag" not in VIEWER.split("function onCanvasPointerUp", 1)[1].split("function onCanvasPointerCancel", 1)[0]
    for helper in ["beginDirectMoveDrag", "updateDirectMoveDrag", "finishDirectMoveDrag"]:
        assert f"function {helper}" in VIEWER


def test_move_transform_controls_use_world_axes_and_commit_once_after_preview():
    listeners = VIEWER.split("transformControls.addEventListener('dragging-changed'", 1)[1].split("controls.addEventListener('start'", 1)[0]
    object_change = listeners.split("transformControls.addEventListener('objectChange'", 1)[1]
    assert "transformControls.setSpace('world')" in VIEWER
    assert "transformControls.object === rendered?.object3d" in listeners
    assert "state.gizmoDragStart = cloneTransform" in listeners
    assert "state.gizmoDragGroupStart = captureTransformGroup(rendered)" in listeners
    assert "!sameTransform(state.gizmoDragStart, finalTransform)" in listeners
    assert listeners.count("markDirtyTransform(rendered, finalTransform") == 1
    assert listeners.count("emitTransformCommitted(rendered)") == 1
    assert "applyTransformChanges(linkedTransformChanges" in object_change
    assert "syncInspectorTransformFields(rendered)" in object_change
    assert "markDirtyTransform" not in object_change
    assert "emitTransformCommitted" not in object_change


def test_rotate_transform_controls_preserve_canonical_components_and_commit_once():
    import subprocess

    harness = r"""
const fs=require('fs'),assert=require('assert');const source=fs.readFileSync(process.argv[1],'utf8');
const helper=source.slice(source.indexOf('function canonicalRotatePreviewTransform'),source.indexOf('function directRotatePreviewTransform'));
const cloneTransform=value=>JSON.parse(JSON.stringify(value));eval(helper);
const start={pose:{xyz:{x:.45,y:-.28,z:.19},rpy:{x:.10,y:-.20,z:.30}},scale:{x:1,y:1,z:1}};
for(const [axis,component] of [['X','x'],['Y','y'],['Z','z']])for(const angle of [.37,-.41]){
  const next=canonicalRotatePreviewTransform(start,axis,angle);assert(next);assert.strictEqual(next.pose.rpy[component],start.pose.rpy[component]+angle);
  for(const other of ['x','y','z'].filter(value=>value!==component))assert.strictEqual(next.pose.rpy[other],start.pose.rpy[other]);
  assert.deepStrictEqual(next.pose.xyz,start.pose.xyz);assert.deepStrictEqual(next.scale,start.scale);
}
for(const axis of ['E','XYZE','XYZ',null])assert.strictEqual(canonicalRotatePreviewTransform(start,axis,.2),null);
assert.strictEqual(canonicalRotatePreviewTransform(start,'X',NaN),null);
"""
    subprocess.run(
        ["node", "-e", harness, str(ROOT / "workcell_studio_web/viewer/viewer.js")],
        cwd=ROOT, check=True, capture_output=True, text=True,
    )

    attach = VIEWER.split("function attachTransformGizmo", 1)[1].split("function detachTransformGizmo", 1)[0]
    mode = VIEWER.split("function setEditorMode", 1)[1].split("function setEditorSnap", 1)[0]
    for body in [attach, mode]:
        rotate = body.split("setMode('rotate')", 1)[1].split("}", 1)[0]
        for axis in "XYZ":
            assert f"show{axis} = true" in rotate

    direct = VIEWER.split("function directRotatePreviewTransform", 1)[1].split("function beginDirectRotateDrag", 1)[0]
    transient = VIEWER.split("function previewTransientPivotDrag", 1)[1].split("function finishTransientPivotDrag", 1)[0]
    assert "canonicalRotatePreviewTransform(drag.start, drag.axis" in direct
    assert "canonicalRotatePreviewTransform(state.gizmoDragStart, start.axis" in transient
    assert "if (state.editorMode === 'rotate')" in transient
    move_branch = transient.split("if (state.editorMode === 'rotate')", 1)[1].split("pivot.group.updateWorldMatrix", 1)[1]
    assert "matrixWorld.clone().multiply" in move_branch
    assert "ownerLocal.decompose" in move_branch

    listeners = VIEWER.split("transformControls.addEventListener('dragging-changed'", 1)[1].split("controls.addEventListener('start'", 1)[0]
    object_change = listeners.split("transformControls.addEventListener('objectChange'", 1)[1]
    assert "markDirtyTransform" not in object_change
    assert "emitTransformCommitted" not in object_change
    direct_finish = VIEWER.split("function finishDirectRotateDrag", 1)[1].split("function endDirectRotateDrag", 1)[0]
    pivot_finish = VIEWER.split("function finishTransientPivotDrag", 1)[1].split("function attachTransformGizmo", 1)[0]
    assert direct_finish.count("markDirtyTransform(") == 1
    assert direct_finish.count("emitTransformCommitted(") == 1
    assert pivot_finish.count("markDirtyTransform(") == 1
    assert pivot_finish.count("emitTransformCommitted(") == 1
    assert "snapOptions: null" in direct_finish
    assert "snapOptions: state.editorMode === 'rotate' ? null : undefined" in pivot_finish


def test_rotation_snap_radians_converts_arbitrary_degree_values():
    import subprocess

    script = r"""
const fs=require('fs'),assert=require('assert');const source=fs.readFileSync(process.argv[1],'utf8');
const body=source.match(/function rotationSnapRadians\(\) \{([^}]*)\}/)[1];
const THREE={MathUtils:{degToRad:value=>value*Math.PI/180}};let el={rotationSnap:{value:'5'}};
const rotationSnapRadians=new Function('el','THREE',`return function(){${body}}`)(el,THREE);
assert(Math.abs(rotationSnapRadians()-5*Math.PI/180)<1e-12);el.rotationSnap.value='15';assert(Math.abs(rotationSnapRadians()-15*Math.PI/180)<1e-12);
"""
    subprocess.run(["node", "-e", script, str(ROOT / "workcell_studio_web/viewer/viewer.js")], cwd=ROOT, check=True)


def test_qt_compact_toolbar_for_embedded_web3d():
    for member in ["embedded_undo_button_", "embedded_redo_button_", "embedded_fit_button_"]:
        assert member in CPP and member in HDR
    assert "set_visible(view_actions_label_, !embedded_web_active)" in CPP
    assert "set_visible(view_actions_selector_, !embedded_web_active)" in CPP
    assert "set_visible(labels_label_, !embedded_web_active)" in CPP
    assert "set_visible(labels_selector_, !embedded_web_active)" in CPP
    assert "set_visible(mesh_preview_mode_label_, !embedded_web_active)" in CPP
    assert "set_visible(mesh_preview_mode_selector_, !embedded_web_active)" in CPP
    assert "set_visible(interaction_mode_label_, !embedded_web_active)" in CPP
    assert "set_visible(interaction_mode_selector_, !embedded_web_active)" in CPP
    assert "set_visible(overlays_selector_, !embedded_web_active)" in CPP
    assert "set_visible(gizmo_mode_label_, embedded_web_active)" in CPP
    assert "set_visible(gizmo_mode_selector_, embedded_web_active)" in CPP
    assert "set_visible(snap_mode_selector_, embedded_web_active)" in CPP
    assert 'state.value(QStringLiteral("canUndo")).toBool()' in CPP
    assert 'state.value(QStringLiteral("canRedo")).toBool()' in CPP


def test_product_view_selection_uses_stable_identity_and_filters_helpers():
    assert "function isExpandedUrdfInspectionPick(rendered)" in VIEWER
    assert "function isCanvasSelectableRendered(rendered)" in VIEWER
    assert "state.pickRecords.includes(rendered)" in VIEWER
    assert "rendered?.authoritativePhysicalPick === true" in VIEWER
    assert "function isNormalSelectableRendered(rendered)" in VIEWER
    assert "item.selectable !== false" in VIEWER
    assert "state.debugOverlaysVisible && (isTaskOnlyHelperItem(item)" in VIEWER
    assert "!isTaskOnlyHelperItem(item) && !isOverlayPolicyItem(item)" in VIEWER
    assert "!isDebugOverlayItem(item)" in VIEWER
    assert "renderedById(requestedId)" in VIEWER
    assert "missing_render_identity" in VIEWER
    assert "diagnostic_helper_or_non_selectable" in VIEWER
    assert "canonicalSelectionRendered" in VIEWER
    assert "selectObjectFromRender(canonicalId, selectedCandidate.rendered);" in VIEWER
    assert "itemLabel(item)" not in VIEWER.split("function pickObject", 1)[1].split("function beginDirectMoveDrag", 1)[0]


def test_expanded_urdf_inspection_selection_policy_executes_in_browser_harness(tmp_path):
    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const element=()=>({hidden:false,checked:false,disabled:false,textContent:'',className:'',innerHTML:'',classList:{toggle(){}},querySelector(){return null;},querySelectorAll(){return[];},addEventListener(){},setAttribute(){},appendChild(){},getBoundingClientRect(){return {left:0,top:0,width:100,height:100}}});
const context={console,assert,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}}},document:{getElementById(){return element()},querySelectorAll(){return[]},createElement(){return element()}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context); vm.runInContext(source+`
updateLabels=()=>{}; populateInspector=()=>{}; attachTransformGizmo=()=>{}; detachTransformGizmo=()=>{}; removeSelectionHighlight=()=>{};
const highlighted=[]; refreshSelectionHighlight=record=>highlighted.push(record);
const object=name=>({name,visible:true,userData:{},children:[],parent:null});
const normal=id=>{const item={id,editable:true,source_layer:'editable_layout'};const object3d=object(id);object3d.userData.item=item;return {item,object3d};};
const table=normal('support_surface_table'),camera=normal('realsense_overhead'),bin=normal('target_bin_default');
state.objects=[table,camera,bin]; state.sceneJson={scene:{id:'ur5_2f_test'},ui_selection_owners:[{id:'ur5'},{id:'robotiq_85_gripper'}]}; rebuildSelectionIdentityIndex();
for(const rendered of [table,camera,bin]){assert.strictEqual(itemFromRaycastHit({object:rendered.object3d}),rendered);selectObject(rendered.item.id);assert.strictEqual(state.selected,rendered.item.id);}
const register=(id,link,owner)=>{const node=object(link);return registerPickRecord({id,link_name:link,locked:true,editable:false,selectable:true},node,node,{pickRecordSource:'expanded_urdf_inspection',uiSelectionOwnerId:owner});};
const wrist=register('scene::inspection::wrist_3_link','wrist_3_link','ur5');
const finger=register('scene::inspection::robotiq_85_right_finger_link','robotiq_85_right_finger_link','robotiq_85_gripper');
for(const [record,owner] of [[wrist,'ur5'],[finger,'robotiq_85_gripper']]){assert.strictEqual(isCanvasSelectableRendered(record),true);selectObject(record.item.id);assert.strictEqual(editorState().selectedItemId,owner);assert.strictEqual(editorState().uiSelectionItemId,owner);assert.strictEqual(editorState().selectedEditable,false);assert.strictEqual(highlighted.at(-1),record);}
const excluded=object('selection_subtle_bounds_highlight'); excluded.userData.selection_highlight=true;
state.three={pointer:{},camera:{},raycaster:{setFromCamera(){},intersectObjects(){return this.hits;},hits:[]}};
for(const [record,canonical,editable] of [[wrist,'ur5',false],[finger,'robotiq_85_gripper',false],[camera,'realsense_overhead',true],[table,'support_surface_table',true],[bin,'target_bin_default',true]]){
  state.three.raycaster.hits=[{object:excluded,distance:0.5},{object:record.object3d,distance:1}];
  assert.strictEqual(pickObject({clientX:5,clientY:5}),canonical,'excluded first hit must not reject '+canonical);
  assert.strictEqual(editorState().selectedItemId,canonical);
  assert.strictEqual(editorState().selectedEditable,editable);
}
const diagnostic=normal('ordinary_diagnostic'); diagnostic.item.diagnostic_only=true; state.objects.push(diagnostic); assert.strictEqual(isCanvasSelectableRendered(diagnostic),false); assert.strictEqual(selectObject(diagnostic.item.id),'');
assert.strictEqual(state.objects.includes(wrist),false);assert.strictEqual(state.objects.includes(finger),false);assert.strictEqual(state.dirtyTransforms.size,0);assert.strictEqual(buildEditPatch().edits.length,0);assert.strictEqual(state.undoStack.length,0);assert.strictEqual(state.redoStack.length,0);
`,context);
"""
    import subprocess
    subprocess.run(["node", "-e", harness, str(ROOT / "workcell_studio_web/viewer/viewer.js")], cwd=ROOT, check=True, capture_output=True, text=True)


def test_qt_selection_clears_stale_scene_and_missing_id_callbacks():
    assert "selected_preview_item_id_.clear();" in CPP
    assert "scene_context_changed" in CPP
    assert "selection_missing_after_refresh" in CPP
    assert "Preview selection cleared after refresh (id missing):" in CPP
    assert "if (!embedded_web_identity_is_current(identity)) return;" in CPP
    assert "if (valid_browser_selection && browser_ui_selected_id != selected_preview_item_id_)" in CPP
    assert "selection_update_guard_" in (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")


def test_missing_nonempty_selection_is_preserved_and_warning_is_bounded():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    body = main.split("void MainWindow::apply_scene_selection", 1)[1].split("void MainWindow::mark_layout_dirty", 1)[0]
    assert "Ignored selection id absent from active scene payload; existing selection preserved:" in body
    assert "emitted_scene_diagnostic_log_keys_.contains(warning_key)" in body
    assert body.find("if (!active_scene_item_present)") < body.find("current_selected_scene_item_id_ = selected_id")
    assert "apply_scene_selection(QString(), selected_role, true, false)" not in body


def test_selection_diagnostics_are_exposed_to_qt_bridge():
    assert "function currentSelectionDiagnostics()" in VIEWER
    assert "selectionDiagnostics: currentSelectionDiagnostics()" in VIEWER
    assert "selectionDiagnostics: () => currentSelectionDiagnostics()" in VIEWER
    for field in ["renderIdentity", "sourceLayer", "activeVisualSource", "diagnosticOnly", "helperOrOverlay", "objectPresent", "canonicalSelectedId", "editAuthoritySource", "canonicalOwnerId", "gizmoAttachedTargetId", "gizmoAttachedObjectName", "gizmoVisible", "gizmoEnabled", "gizmoAttachmentReason", "physicalEditBinding", "ownerRecordSource", "physicalVisualId", "pivotWorldCentre", "attachedObjectWorldPosition", "distance", "attachmentReason"]:
        assert field in VIEWER


def test_authored_camera_and_table_generated_visuals_attach_to_canonical_edit_owners(tmp_path):
    import json
    import subprocess

    web_scene = tmp_path / "ur5_2f_test.web_scene.json"
    subprocess.run(
        [
            "python3", "scripts/ensure_workcell_studio_web_scene_fresh.py",
            "--scene", "scenes/ur5_2f_test", "--output", str(web_scene),
            "--stage-assets", "--force",
        ],
        cwd=ROOT, check=True, capture_output=True, text=True,
    )
    payload = json.loads(web_scene.read_text(encoding="utf-8"))
    assert any(item.get("camera_id") == "realsense_overhead" for item in payload["sensors"])
    assert any(item.get("support_surface_ref") == "support_surface_table" for item in payload["assets"])
    physical_visuals = [
        item for section in ("sensors", "assets") for item in payload[section]
        if item.get("mesh_contract_category") in {"camera", "table"}
    ]
    assert physical_visuals
    assert all("owner_relative_visual_transform" in item for item in physical_visuals)
    assert all(item["provenance"]["owner_relative_visual_transform"]["source_owner_pose"] for item in physical_visuals)
    assert "Object3D.attach" not in VIEWER.split("function bindExportedPhysicalTransformOwnership", 1)[1].split("function suppressOwnedAuthoredFallback", 1)[0]
    assert "applyExportedOwnerRelativeVisualTransform(rendered, owner)" in VIEWER

    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const THREE_IMPL=require('./workcell_studio_web/viewer/node_modules/three/build/three.cjs');
const payload=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
const element=()=>({hidden:false,checked:false,disabled:false,textContent:'',className:'',innerHTML:'',dataset:{},classList:{toggle(){}},querySelector(){return null;},querySelectorAll(){return[];},addEventListener(){},setAttribute(){},appendChild(){},remove(){},getBoundingClientRect(){return {left:0,top:0,width:100,height:100}}});
const context={console,assert,THREE_IMPL,inputPayload:payload,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}}},document:{getElementById(){return element()},querySelectorAll(){return[]},createElement(){return element()}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context); vm.runInContext(source+`
THREE=THREE_IMPL; updateLabels=()=>{}; populateObjectList=()=>{}; renderSceneSummary=()=>{}; maybeEmitSceneReady=()=>{}; updateDirtyState=()=>{}; renderFrameDebugOverlays=()=>{}; beginWeb3dSceneReadiness=()=>{}; registerReadinessOperation=()=>null; maybeWarnSupportSurfaceSemantics=()=>{}; isExpandedUrdfRobotPreview=()=>false;buildRobotAssemblies=()=>({handled:new Set(),assemblies:[],renderDiagnostics:{}});const productionUsesAssembly=usesAssembledUrdfHierarchy;usesAssembledUrdfHierarchy=item=>item.editable===true?false:productionUsesAssembly(item);
tryLoadMesh=(item,rendered,fallback)=>{if(!item.mesh_uri)return;const mesh=new THREE.Mesh(new THREE.BoxGeometry(.04,.03,.02),new THREE.MeshBasicMaterial());mesh.name=item.id+'_physical_mesh';rendered.object3d.add(mesh);rendered.meshObject=mesh;item.mesh_status='loaded';if(fallback)fallback.visible=false;suppressOwnedAuthoredFallback(rendered);};
state.sceneJson=inputPayload;state.three.scene=new THREE.Scene();const exportedItems=validateSceneJson(inputPayload);assert(!exportedItems.some(item=>item.id==='realsense_overhead'));assert(!exportedItems.some(item=>item.id==='support_surface_table'));renderScene(exportedItems);
const byId=id=>state.objects.find(record=>record.item.id===id);
const camera=byId('realsense_overhead'),table=byId('support_surface_table'),bin=byId('target_bin_default');
const cameraVisual=state.objects.find(record=>record.item.camera_id==='realsense_overhead'&&record.item.locked===true);
const tableVisual=state.objects.find(record=>record.item.support_surface_ref==='support_surface_table'&&record.item.locked===true);
assert(camera&&table&&bin&&cameraVisual&&tableVisual,JSON.stringify({camera:!!camera,table:!!table,bin:!!bin,cameraVisual:!!cameraVisual,tableVisual:!!tableVisual}));
assert.strictEqual(camera.physicalEditRoot,true);assert.strictEqual(table.physicalEditRoot,true);
assert.deepStrictEqual(transformFromObject(camera.object3d).pose.xyz,{x:.35,y:0,z:.85});assert(Math.abs(transformFromObject(camera.object3d).pose.rpy.y-1.5708)<1e-12);
assert.deepStrictEqual(transformFromObject(table.object3d).pose.xyz,{x:.55,y:0,z:.06});
assert.strictEqual(cameraVisual.object3d.parent,camera.object3d);assert.strictEqual(tableVisual.object3d.parent,table.object3d);
const worldPose=o=>{o.updateWorldMatrix(true,true);return {p:new THREE.Vector3().setFromMatrixPosition(o.matrixWorld),q:new THREE.Quaternion().setFromRotationMatrix(o.matrixWorld)}};
const gizmo={object:null,visible:false,enabled:false,showX:false,showY:false,showZ:false,axis:'Z',rotationAngle:0,attach(o){this.object=o},detach(){this.object=null},setMode(){},setSpace(){},setTranslationSnap(){},setRotationSnap(){},reset(){}};
let hits=[];state.three={transformControls:gizmo,pointer:{},camera:{},controls:{enabled:true},raycaster:{setFromCamera(){},intersectObjects(){return hits}}};state.editorMode='move';
// Match the production ordering that exposed PR #2957: same-ID authored-looking
// records precede the exact physical edit owners after the binding pass.
const competingCamera={item:{...camera.item,source_layer:'competing_same_id_record'},object3d:new THREE.Group()};
const competingTable={item:{...table.item,source_layer:'competing_same_id_record'},object3d:new THREE.Group()};
state.objects.unshift(competingCamera,competingTable);
assert.strictEqual(renderedById('realsense_overhead'),competingCamera);assert.strictEqual(renderedById('support_surface_table'),competingTable);
assert.strictEqual(resolveCanonicalPhysicalEditBinding('realsense_overhead').owner,camera);assert.strictEqual(resolveCanonicalPhysicalEditBinding('support_surface_table').owner,table);
assert.strictEqual(canonicalTransformOwner('target_bin_default'),bin);
const assertCanonicalGizmoIdentity=(canonical,owner)=>{const diagnostic=currentSelectionDiagnostics();assert.strictEqual(diagnostic.selectedItemId,canonical);assert.strictEqual(diagnostic.canonicalOwnerId,canonical);assert.strictEqual(diagnostic.editOwnerItemId,canonical);assert.strictEqual(diagnostic.gizmoAttachedTargetId,canonical);assert.strictEqual(state.gizmoPivot?.group===gizmo.object?state.gizmoPivot.owner:owner,owner);assert(!state.objects.some(record=>record.object3d===state.gizmoPivot?.group));assert(!state.pickRecords.some(record=>record.object3d===state.gizmoPivot?.group));assert.strictEqual(state.dirtyTransforms.has(state.gizmoPivot?.group?.name),false);assert(!state.undoStack.some(entry=>JSON.stringify(entry).includes('_transient_gizmo_pivot')));assert(!state.redoStack.some(entry=>JSON.stringify(entry).includes('_transient_gizmo_pivot')));assert(!buildEditPatch().edits.some(edit=>edit.item_id?.includes('_transient_gizmo_pivot')));};
const pick=(visual,canonical,owner)=>{hits=[{object:visual.object3d,distance:1}];assert.strictEqual(pickObject({clientX:5,clientY:5}),canonical);assert.strictEqual(state.selected,canonical);assert.strictEqual(editorState().selectedEditable,true);assert(gizmo.object===owner.object3d||(gizmo.object===state.gizmoPivot?.group&&state.gizmoPivot.owner===owner));assert.strictEqual(gizmo.visible,true);assert.strictEqual(gizmo.enabled,true);assert.strictEqual(gizmo.showX&&gizmo.showY&&gizmo.showZ,true);assertCanonicalGizmoIdentity(canonical,owner);};
setEditorMode('select');hits=[{object:cameraVisual.object3d,distance:1}];assert.strictEqual(pickObject({clientX:5,clientY:5}),'realsense_overhead');assert.strictEqual(gizmo.object,null);assert.strictEqual(gizmo.visible,false);assert.strictEqual(gizmo.enabled,false);
const pendingCameraMesh=cameraVisual.meshObject;cameraVisual.meshObject=null;cameraVisual.item.mesh_status='loading';setEditorMode('move');assert.strictEqual(gizmo.object,null,'registered physical binding must wait rather than attach at authored origin');cameraVisual.meshObject=pendingCameraMesh;cameraVisual.item.mesh_status='loaded';suppressOwnedAuthoredFallback(cameraVisual);assert.strictEqual(gizmo.object,state.gizmoPivot.group,'mesh completion must attach the registered owner at its physical centre');
hits=[{object:cameraVisual.object3d,distance:1}];assert.strictEqual(pickObject({clientX:5,clientY:5}),'realsense_overhead');assert.strictEqual(gizmo.object,state.gizmoPivot.group);const cameraPivot=state.gizmoPivot.group;
assertCanonicalGizmoIdentity('realsense_overhead',camera);assert.strictEqual(currentSelectionDiagnostics().gizmoAttachedObjectName,'realsense_overhead_transient_gizmo_pivot');
const cameraMeshCentre=()=>{const mesh=cameraVisual.meshObject;mesh.updateWorldMatrix(true,true);mesh.geometry.computeBoundingBox();return mesh.geometry.boundingBox.clone().applyMatrix4(mesh.matrixWorld).getCenter(new THREE.Vector3())};assert(cameraPivot.getWorldPosition(new THREE.Vector3()).distanceTo(cameraMeshCentre())<1e-12);
const helper=new THREE.Mesh(new THREE.BoxGeometry(1,1,1),new THREE.MeshBasicMaterial());helper.name='camera_fov_frustum_helper';helper.position.set(50,50,50);cameraVisual.meshObject.add(helper);assert(authoritativePhysicalMeshCentre(camera).distanceTo(cameraMeshCentre())<1e-12,'camera helper must not affect physical centre');
let bindingDiagnostic=currentSelectionDiagnostics().physicalEditBinding;assert.strictEqual(bindingDiagnostic.canonicalOwnerId,'realsense_overhead');assert.strictEqual(bindingDiagnostic.ownerRecordSource,'synthetic_authored_selection_owner');assert.strictEqual(bindingDiagnostic.physicalVisualId,cameraVisual.item.id);assert(bindingDiagnostic.distance<1e-6);
const cameraBeforeAttach=worldPose(camera.object3d),cameraMeshBeforeAttach=worldPose(cameraVisual.meshObject);attachTransformGizmo(camera);assert(worldPose(camera.object3d).p.distanceTo(cameraBeforeAttach.p)<1e-12);assert(worldPose(cameraVisual.meshObject).p.distanceTo(cameraMeshBeforeAttach.p)<1e-12);
hits=[{object:tableVisual.object3d,distance:1}];assert.strictEqual(pickObject({clientX:5,clientY:5}),'support_surface_table');assert.strictEqual(gizmo.object,state.gizmoPivot.group);assert(state.gizmoPivot.group.position.distanceTo(new THREE.Box3().setFromObject(tableVisual.meshObject).getCenter(new THREE.Vector3()))<1e-12);
assertCanonicalGizmoIdentity('support_surface_table',table);assert.strictEqual(currentSelectionDiagnostics().gizmoAttachedObjectName,'support_surface_table_transient_gizmo_pivot');
hits=[{object:bin.object3d,distance:1}];assert.strictEqual(pickObject({clientX:5,clientY:5}),'target_bin_default');assert.strictEqual(gizmo.object,bin.object3d);assert.strictEqual(state.gizmoPivot,null);assertCanonicalGizmoIdentity('target_bin_default',bin);assert.strictEqual(currentSelectionDiagnostics().gizmoAttachedObjectName,bin.object3d.name||'');
assert.strictEqual(byId('ur5'),undefined);assert.strictEqual(byId('robotiq_85_gripper'),undefined);for(const record of state.objects.filter(record=>record!==cameraVisual&&record!==tableVisual&&record.item.locked===true)){assert.strictEqual(record.object3d.parent.type,'Scene');assert.strictEqual(selectionIsEditable(record),false);attachTransformGizmo(record);assert.strictEqual(gizmo.object,null,'URDF descendants, fallback geometry, and generated visual rows must remain read-only');assert.strictEqual(currentSelectionDiagnostics().gizmoAttachedTargetId,'');}
setEditorMode('move');hits=[{object:cameraVisual.object3d,distance:1}];pickObject({clientX:5,clientY:5});assert.strictEqual(gizmo.object,state.gizmoPivot.group);
const before=cloneTransform(transformFromObject(camera.object3d)),meshBefore=worldPose(cameraVisual.meshObject).p;beginTransientPivotDrag(camera);state.gizmoPivot.group.position.x+=.04;previewTransientPivotDrag(camera);assert(Math.abs(transformFromObject(camera.object3d).pose.xyz.x-before.pose.xyz.x-.04)<1e-10);assert(Math.abs(worldPose(cameraVisual.meshObject).p.x-meshBefore.x-.04)<1e-10);finishTransientPivotDrag(camera);undoPreviewEdit();redoPreviewEdit();
setEditorMode('rotate');assert.strictEqual(gizmo.object,state.gizmoPivot.group);const ownerBeforeRotation=worldPose(camera.object3d).p;gizmo.axis='Z';beginTransientPivotDrag(camera);gizmo.rotationAngle=.2;previewTransientPivotDrag(camera);const ownerAfterRotation=worldPose(camera.object3d).p;assert(ownerAfterRotation.distanceTo(ownerBeforeRotation)<1e-12,'canonical XYZ must not drift during pivot rotation');assert(Math.abs(transformFromObject(camera.object3d).pose.rpy.z-before.pose.rpy.z)>.1,'canonical owner rotation must change');finishTransientPivotDrag(camera);
setEditorMode('select');assert.strictEqual(gizmo.object,null);setEditorMode('rotate');assert.strictEqual(state.gizmoPivot.owner,camera);assert(state.gizmoPivot.group.getWorldPosition(new THREE.Vector3()).distanceTo(cameraMeshCentre())<1e-6,'mode switching must not revert to competing authored origin');
hits=[{object:tableVisual.object3d,distance:1}];pickObject({clientX:5,clientY:5});const tableBefore=cloneTransform(transformFromObject(table.object3d));gizmo.axis='Z';beginTransientPivotDrag(table);gizmo.rotationAngle=-.61086524;previewTransientPivotDrag(table);assert.deepStrictEqual(transformFromObject(table.object3d).pose.xyz,tableBefore.pose.xyz);finishTransientPivotDrag(table);undoPreviewEdit();redoPreviewEdit();
assert.strictEqual(state.dirtyTransforms.has(cameraVisual.item.id),false);assert.strictEqual(state.dirtyTransforms.has(tableVisual.item.id),false);
const patch=window.__WORKCELL_EDITOR_API_V1__.getEditPatch(),edits=new Map(patch.edits.map(edit=>[edit.item_id,edit]));
assert.deepStrictEqual([...edits.keys()].sort(),['realsense_overhead','support_surface_table']);
assert.deepStrictEqual(edits.get('realsense_overhead').old_transform.pose.xyz,{x:.35,y:0,z:.85});
assert(Math.abs(edits.get('realsense_overhead').old_transform.pose.rpy.y-1.5708)<1e-12);
assert.deepStrictEqual(edits.get('support_surface_table').old_transform.pose.xyz,{x:.55,y:0,z:.06});
assert(Math.abs(edits.get('support_surface_table').new_transform.pose.xyz.z-.06)<1e-12);assert(Math.abs(edits.get('support_surface_table').new_transform.pose.rpy.z)>.6);
for(const edit of edits.values()){assert.strictEqual(edit.operation,'update_transform');assert.strictEqual(edit.persistence_source,'layout/workcell_studio_layout.yaml');assert.deepStrictEqual(edit.old_transform.scale,{x:1,y:1,z:1});assert.deepStrictEqual(edit.new_transform.scale,{x:1,y:1,z:1});assert(!edit.item_id.startsWith('urdf_'));}
const raycastCandidates=[...state.objects.map(o=>o.object3d),...state.pickRecords.map(o=>o.pickRoot||o.object3d)].filter((root,index,all)=>root?.visible!==false&&!excludedPickNode(root)&&all.indexOf(root)===index),candidateSet=new Set(raycastCandidates),roots=raycastCandidates.filter(root=>{for(let p=root.parent;p;p=p.parent)if(candidateSet.has(p))return false;return true});for(const root of roots){for(let p=root.parent;p;p=p.parent)assert.strictEqual(roots.includes(p),false,'raycast roots must be top-level');}assert(!roots.includes(cameraVisual.object3d)&&!roots.includes(tableVisual.object3d));
`,context);
"""
    subprocess.run(["node", "-e", harness, str(ROOT / "workcell_studio_web/viewer/viewer.js"), str(web_scene)], cwd=ROOT, check=True, text=True)


def test_qt_poll_accepts_only_final_valid_browser_selection_and_explicit_clear():
    poll = CPP.split("void ScenePreviewWidget::poll_embedded_editor_events()", 1)[1].split("#else", 1)[0]
    for token in [
        'editor_state.value(QStringLiteral("selectedItemId"))',
        'editor_state.value(QStringLiteral("uiSelectionItemId"))',
        'editor_state.value(QStringLiteral("selectionDiagnostics"))',
        'editor_state.value(QStringLiteral("sceneId"))',
        'selection_diagnostics.value(QStringLiteral("objectPresent")).toBool()',
        'preview_item_by_id(browser_ui_selected_id) != nullptr',
        'browser_scene_id == identity.scene_id',
    ]:
        assert token in poll
    assert "selected_preview_item_id_ = id" not in poll
    assert 'event.value(QStringLiteral("uiItemId"))' in poll
    assert "browser_ui_selected_id = browser_selected_id" in poll
    assert "selected_preview_item_id_ = browser_ui_selected_id" in poll
    assert "browser_selected_id.isEmpty()" in poll
    assert "selected_preview_item_id_.clear();" in poll


def test_qt_poll_treats_current_browser_state_as_authoritative_without_same_cycle_event():
    poll = CPP.split("void ScenePreviewWidget::poll_embedded_editor_events()", 1)[1].split("#else", 1)[0]
    assert "matching_selection_event" not in poll
    assert "const bool valid_browser_selection = scene_identity_matches && !browser_ui_selected_id.isEmpty()" in poll
    assert 'editor_state.value(QStringLiteral("selectedItemType"))' in poll
    assert "browser_ui_selected_id != selected_preview_item_id_" in poll
    assert "emit preview_item_selected(browser_ui_selected_id, matching_item_type);" in poll
    assert "browser_selected_id.isEmpty() && scene_identity_matches" in poll
    assert "state_request_token != embedded_editor_state_request_token_" in poll
    assert "preview_item_by_id(browser_ui_selected_id) != nullptr" in poll
    assert "Embedded Product View selection rejected:" in poll
    assert "emitted_scene_diagnostic_keys_.contains(rejection_key)" in poll


def test_qt_inventory_preserves_explicit_locked_robot_and_tool_owner_rows():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    body = main.split("QString selection_robot_owner_id", 1)[1].split("const bool filtered_payload_changed", 1)[0]
    assert 'value(QStringLiteral("selection_robot_owner_id"))' in body
    assert 'value(QStringLiteral("selection_tool_owner_id"))' in body
    assert "item.id.trimmed() == owner_id" in body
    assert "filtered_items.push_back(*owner)" in body
    assert "selection_robot_owner_id.isEmpty() && !explicit_robot_owner" in body
    assert "selection_tool_owner_id.isEmpty() && !explicit_tool_owner" in body
    assert "used unique category fallback" in body
    assert "item.locked && !item.editable" in body
    assert 'source_layer != QStringLiteral("locked_generated_urdf_visual")' in body
    assert "if (candidates.size() != 1) return;" in body
    assert 'QStringLiteral("robot_arm")' in body
    assert 'QStringLiteral("end_effector")' in body
    assert "filtered_items.push_back(candidates.front())" in body


def test_qt_reconciles_exported_selection_owner_registry_as_identity_only_rows():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    body = main.split("const QString selection_owner_contract_path", 1)[1].split(
        "apply_scene3d_product_view_layer_defaults_and_commit();", 1
    )[0]
    for token in [
        'value(QStringLiteral("ui_selection_owners"))',
        'value(QStringLiteral("robot_preview"))',
        'value(QStringLiteral("selection_robot_owner_id"))',
        'value(QStringLiteral("selection_tool_owner_id"))',
        'QStringLiteral("robot")',
        'QStringLiteral("end_effector")',
    ]:
        assert token in body
    assert 'source_layer == QStringLiteral("selection_owner_registry")' in body
    assert "all_scene_preview_items_.removeAt(i)" in body
    assert "item.id.trimmed() == declaration.id" in body
    assert "if (id_already_owned) continue;" in body
    assert 'owner.source_layer = QStringLiteral("selection_owner_registry")' in body
    assert "owner.locked = true" in body
    assert "owner.editable = false" in body
    assert "owner.mesh_available = false" in body
    assert "owner.has_mesh_metadata = false" in body
    assert "owner.primitive_geometry_type.clear()" in body
    assert "owner.sx = owner.sy = owner.sz = 0.0" in body
    assert "all_scene_preview_items_.push_back(owner)" in body
    assert "add_tree_node(owner)" in body
    assert "retained usable preview state" in body


def test_selection_owner_registry_is_forwarded_for_lookup_but_not_geometry():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    filters = main.split("void MainWindow::apply_scene3d_preview_layer_filters", 1)[1].split(
        "void MainWindow::refresh_scene3d_product_view_status_and_audit", 1
    )[0]
    registry_branch = filters.split('p.source_layer == QStringLiteral("selection_owner_registry")', 1)[1]
    registry_branch = registry_branch.split("if (workcell_builder::include_preview_item_for_scene3d", 1)[0]
    assert "filtered_items.push_back(p)" in registry_branch
    assert "continue;" in registry_branch
    assert "preview_item_by_id(browser_ui_selected_id) != nullptr" in CPP
    ingest = VIEWPORT_CPP.split("void Scene3DViewportWidget::ingest_preview_items", 1)[1]
    ingest = ingest.split("void Scene3DViewportWidget::", 1)[0]
    assert 'item.source_layer == QStringLiteral("selection_owner_registry")' in ingest
    assert "continue;" in ingest


def test_qt_exact_ur5_and_robotiq_owner_ids_remain_selectable_in_preview_inventory():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    environment = (ROOT / "scenes/ur5_2f_test/environment.yaml").read_text(encoding="utf-8")
    owner_body = main.split("auto preserve_explicit_inspection_owner", 1)[1].split("// Compatibility fallback", 1)[0]
    poll = CPP.split("void ScenePreviewWidget::poll_embedded_editor_events()", 1)[1].split("#else", 1)[0]
    assert "all_scene_preview_items_.cbegin()" in owner_body
    assert "item.id.trimmed() == owner_id" in owner_body
    assert "filtered_items.push_back(*owner)" in owner_body
    for owner_id in ("ur5", "robotiq_85_gripper"):
        assert f"id: {owner_id}" in environment
    assert "preview_item_by_id(browser_ui_selected_id) != nullptr" in poll


def test_qt_web3d_selected_transform_inspector_round_trip_contract():
    main = (
        ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
    ).read_text(encoding="utf-8")

    # Web3D -> Qt: the live selected transform is exported through getState().
    get_state = VIEWER.split(
        "window.__WORKCELL_EDITOR_API_V1__ = {", 1
    )[1].split("selectItem:", 1)[0]
    assert "selectedTransform" in get_state
    assert "canonicalTransformOwner" in get_state
    assert "dirtyTransforms.get(transformOwner.item.id)" in get_state

    assert "preview_item_transform_changed" in HDR
    assert "preview_item_transform_changed" in CPP
    assert "selectedTransform" in CPP
    assert "embeddedSelectedTransformSignature" in CPP
    assert "refresh_selection_transform_editor_from_state(refreshed)" in main

    # Qt -> Web3D: inspector Apply uses the live editor instead of reloading.
    bridge = VIEWER.split(
        "function setItemPoseFromBridge", 1
    )[1].split("window.__WORKCELL_EDITOR_API_V1__", 1)[0]
    assert "canonicalEditOwnerRendered" in bridge
    assert "selectionIsEditable(rendered)" in bridge
    assert "markDirtyTransform(rendered, next" in bridge
    assert "emitTransformCommitted(rendered)" in bridge

    assert "setItemPose:" in VIEWER
    assert "set_authoring_item_pose" in HDR
    assert "set_authoring_item_pose" in CPP
    assert ".setItemPose(" in CPP

    inspector = main.split(
        "void MainWindow::apply_inspector_pose_to_item()", 1
    )[1].split(
        "void MainWindow::revert_selection_transform_editor()", 1
    )[0]

    assert "embedded_web_authoring_active()" in inspector
    assert "set_authoring_item_pose(" in inspector

    embedded_section = inspector.split(
        "scene_preview_widget_->embedded_web_authoring_active()", 1
    )[1]
    direct_part, fallback_part = embedded_section.split("} else {", 1)

    assert "set_authoring_item_pose(" in direct_part
    assert "apply_scene3d_preview_layer_filters(false);" not in direct_part
    assert "apply_scene3d_preview_layer_filters(false);" in fallback_part


def test_embedded_asset_drop_is_identity_only_and_uses_typed_shared_request():
    header = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.h").read_text(encoding="utf-8")
    drop_view = CPP.split("class AssetDropWebEngineView", 1)[1].split("#endif", 1)[0]
    assert 'application/x-workcell-studio-asset' in drop_view
    assert 'mime->formats() != QStringList{kMimeType}' in drop_view
    assert 'payload.size() != 1' in drop_view
    assert 'payload.contains(QStringLiteral("asset_id"))' in drop_view
    assert 'source_path' not in drop_view and 'text/uri-list' not in drop_view
    assert drop_view.count('placement_requested(asset_id, x, y, z, configure_transform)') == 1
    assert 'api.placementPointFromViewport({clientX:%1,clientY:%2})' in drop_view
    assert 'pointerToWorldPlane' not in drop_view
    assert 'void asset_placement_requested(' in header


def test_embedded_and_native_frontends_connect_to_same_mainwindow_backend():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    assert 'ScenePreviewWidget::asset_placement_requested' in main
    assert main.count('place_catalog_asset_at_world_position(') >= 3  # definition plus both frontends


def test_qt_arms_and_cancels_existing_browser_placement_api():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    arm = main.split("bool MainWindow::arm_place_asset_mode", 1)[1].split(
        "void MainWindow::commit_armed_asset_placement", 1
    )[0]
    handoff = CPP.split("void ScenePreviewWidget::clear_embedded_editor_state_for_scene_handoff", 1)[1].split(
        "void ScenePreviewWidget::show_embedded_web_loading_document", 1
    )[0]
    assert "arm_embedded_asset_placement" in HDR
    assert "cancel_embedded_asset_placement" in HDR
    assert "ScenePreviewWidget::ProductViewBackend::EmbeddedWeb3D" in arm
    assert "scene_preview_widget_->arm_embedded_asset_placement(" in arm
    assert "__WORKCELL_EDITOR_API_V1__.armPlacement({persistent:%1})" in CPP
    assert "__WORKCELL_EDITOR_API_V1__.cancelPlacement()" in CPP
    assert "cancel_embedded_asset_placement();" in handoff


def test_polled_browser_placement_is_typed_finite_and_identity_free():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    poll = CPP.split("void ScenePreviewWidget::poll_embedded_editor_events()", 1)[1].split("#else", 1)[0]
    connection = main.split(
        "ScenePreviewWidget::embedded_asset_placement_requested", 1
    )[1].split("auto * scene3d_viewport", 1)[0]
    assert 'event_type == QStringLiteral("placement_requested")' in poll
    assert "std::isfinite(x) && std::isfinite(y) && std::isfinite(z)" in poll
    assert "emit embedded_asset_placement_requested(x, y, z);" in poll
    assert "asset_id" not in poll.split('event_type == QStringLiteral("placement_requested")', 1)[1].split(
        '} else if (event_type == QStringLiteral("selection_changed"))', 1
    )[0]
    assert "place_asset_armed_" in connection
    assert "armed_asset_x_m_ = x;" in connection
    assert "armed_asset_y_m_ = y;" in connection
    assert "armed_asset_z_m_ = z;" in connection
    assert connection.count("commit_armed_asset_placement(") == 1
    assert "place_catalog_asset_at_world_position" not in connection


def test_qt_cancellation_does_not_commit_and_native_placement_remains_wired():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    mode = main.split("void MainWindow::set_canvas_interaction_mode", 1)[1].split(
        "bool MainWindow::eventFilter", 1
    )[0]
    clear = main.split("void MainWindow::clear_armed_asset_placement", 1)[1].split(
        "void MainWindow::reset_armed_asset_transform_to_defaults", 1
    )[0]
    assert "clear_armed_asset_placement();" in mode
    assert "scene_preview_widget_->cancel_embedded_asset_placement();" in clear
    assert "commit_armed_asset_placement" not in mode
    assert "place_asset_armed_ = false;" in clear
    assert "scene3d_viewport->asset_drop_cb" in main
    assert "digital_twin_canvas_" in main and "commit_armed_asset_placement(scene_pos);" in main


def test_click_and_embedded_drop_share_canonical_b_placement_backend():
    """A1 steps 6 and 8: both frontends converge before arm/commit."""
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    click = main.split("connect_button(add_to_canvas_button_", 1)[1].split(
        "connect_button(add_asset_button_", 1
    )[0]
    embedded_drop = main.split("ScenePreviewWidget::asset_placement_requested", 1)[1].split(
        "ScenePreviewWidget::embedded_asset_placement_requested", 1
    )[0]
    backend = main.split("bool MainWindow::place_catalog_asset_at_world_position", 1)[1].split(
        "bool MainWindow::configure_asset_placement_transform", 1
    )[0]

    assert 'data(0, CatalogRoleAssetId).toString().trimmed()' in click
    assert click.count("place_catalog_asset_at_world_position(") == 1
    assert embedded_drop.count("place_catalog_asset_at_world_position(") == 1
    for frontend in (click, embedded_drop):
        assert "arm_place_asset_mode(" not in frontend
        assert "commit_armed_asset_placement(" not in frontend
    assert backend.count("arm_place_asset_mode(asset_id)") == 1
    assert backend.count("commit_armed_asset_placement(") == 1


def test_inspector_pose_edit_preserves_authored_mesh_scale():
    main = (
        ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
    ).read_text(encoding="utf-8")

    inspector = main.split(
        "void MainWindow::apply_inspector_pose_to_item()", 1
    )[1].split(
        "void MainWindow::revert_selection_transform_editor()", 1
    )[0]

    # Inspector dimensions remain dimensions.
    assert "p.sx = refreshed_state.dim_x;" in inspector
    assert "p.sy = refreshed_state.dim_y;" in inspector
    assert "p.sz = refreshed_state.dim_z;" in inspector

    # Pose/dimension editing must never overwrite the authored/imported
    # mesh scale, e.g. 0.001 from asset_manifest.yaml.
    assert "p.mesh_scale_x = refreshed_state.dim_x;" not in inspector
    assert "p.mesh_scale_y = refreshed_state.dim_y;" not in inspector
    assert "p.mesh_scale_z = refreshed_state.dim_z;" not in inspector


def test_browser_placement_prefers_authored_support_surface_before_ground_plane():
    viewer = (
        ROOT / "workcell_studio_web/viewer/viewer.js"
    ).read_text(encoding="utf-8")

    helper = viewer.split(
        "function placementPointOnAuthoredSupportSurface", 1
    )[1].split(
        "function placementPointFromViewport", 1
    )[0]

    placement = viewer.split(
        "function placementPointFromViewport", 1
    )[1].split(
        "function getPlacementState", 1
    )[0]

    assert "state.sceneJson?.ui_selection_owners" in helper
    assert "dimensions[2] / 2" in helper
    assert "role !== 'support surface'" in helper
    assert "category !== 'work surface'" in helper

    support_fallback = placement.index(
        "placementPointOnAuthoredSupportSurface(raycaster)"
    )
    ground_fallback = placement.index(
        "new THREE.Plane(new THREE.Vector3(0, 0, 1), 0)"
    )

    assert support_fallback < ground_fallback
