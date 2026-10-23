// Combined #3206 + #3207 harness on the production Web3D render/pick path.
// Usage (cwd = repo root): node production_path_isolation.cjs <viewer.js> <fixture.web_scene.json> <result.json>
// Renders the exported payload with renderScene (mesh loads replaced by boxes, as in
// the repository's owner-binding test), then isolates via the editor API and picks with
// mateRaycast / pickObject through a real three.js camera and raycaster.
const fs = require('fs'), vm = require('vm'), assert = require('assert');
const [viewerPath, payloadPath, resultPath] = process.argv.slice(2);
let source = fs.readFileSync(viewerPath, 'utf8').replace(/boot\(\);\s*$/, '');
const THREE_IMPL = require(process.cwd() + '/workcell_studio_web/viewer/node_modules/three/build/three.cjs');
const payload = JSON.parse(fs.readFileSync(payloadPath, 'utf8'));
const element = () => ({ hidden: false, checked: false, disabled: false, textContent: '', className: '', innerHTML: '', value: '', style: {}, dataset: {},
  classList: { add() {}, remove() {}, toggle() {} }, querySelector() { return null; }, querySelectorAll() { return []; }, addEventListener() {},
  setAttribute() {}, appendChild() {}, remove() {}, getBoundingClientRect() { return { left: 0, top: 0, width: 100, height: 100 }; } });
const results = {};
const context = { console, assert, THREE_IMPL, inputPayload: payload, results,
  window: { location: { search: '' }, dispatchEvent() {}, parent: { postMessage() {} }, addEventListener() {}, __WORKCELL_VIEWER_LIFECYCLE__: { registerCleanup() {} } },
  document: { getElementById() { return element(); }, querySelectorAll() { return []; }, createElement() { return element(); } },
  URLSearchParams, CustomEvent: function () {}, requestAnimationFrame() {}, setTimeout() {}, clearTimeout() {} };
vm.createContext(context);
vm.runInContext(source + `
THREE=THREE_IMPL; updateLabels=()=>{}; populateObjectList=()=>{}; renderSceneSummary=()=>{}; maybeEmitSceneReady=()=>{}; updateDirtyState=()=>{}; renderFrameDebugOverlays=()=>{}; beginWeb3dSceneReadiness=()=>{}; registerReadinessOperation=()=>null; maybeWarnSupportSurfaceSemantics=()=>{}; isExpandedUrdfRobotPreview=()=>false;buildRobotAssemblies=()=>({handled:new Set(),assemblies:[],renderDiagnostics:{}});
const productionUsesAssembly=usesAssembledUrdfHierarchy,productionDiagnosticOnly=isDiagnosticOnlyItem;
usesAssembledUrdfHierarchy=item=>(item.editable===true||item.owner_relative_visual_transform)?false:productionUsesAssembly(item);
isDiagnosticOnlyItem=item=>item.owner_relative_visual_transform?false:productionDiagnosticOnly(item);
tryLoadMesh=(item,rendered,fallback)=>{if(!item.mesh_uri)return;const mesh=new THREE.Mesh(new THREE.BoxGeometry(.04,.03,.02),new THREE.MeshBasicMaterial());mesh.name=item.id+'_physical_mesh';rendered.object3d.add(mesh);rendered.meshObject=mesh;item.mesh_status='loaded';if(fallback)fallback.visible=false;suppressOwnedAuthoredFallback(rendered);};
state.sceneJson=inputPayload;state.three.scene=new THREE.Scene();renderScene(validateSceneJson(inputPayload));
const ros=([r,p,y])=>new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(r,p,y,'ZYX'));
const rotErr=(a,b)=>{const q=new THREE.Quaternion().setFromRotationMatrix(a).invert().multiply(new THREE.Quaternion().setFromRotationMatrix(b));return 2*Math.atan2(Math.hypot(q.x,q.y,q.z),Math.abs(q.w));};
const owners=Object.fromEntries(inputPayload.ui_selection_owners.map(o=>[o.id,o]));
const camera=resolveCanonicalPhysicalEditBinding('realsense_overhead').owner, table=resolveCanonicalPhysicalEditBinding('support_surface_table').owner;
const cameraVisual=state.objects.find(r=>r.item.id.includes('_camera_link_')&&r.item.locked===true);
const tableVisual=state.objects.find(r=>r.item.id==='fixture_generated_table_top_visual');
assert(camera&&table&&cameraVisual&&tableVisual,'owners and reparented visuals rendered');
assert.strictEqual(cameraVisual.object3d.parent,camera.object3d,'D435 visual reparented under the camera owner');
assert.strictEqual(tableVisual.object3d.parent,table.object3d,'table visual reparented under the table owner');
state.three.scene.updateMatrixWorld(true);
// Drawn vs authored: owner pose and the generated visual's exported world pose.
const authoredCamera=owners.realsense_overhead.pose, cameraItem=inputPayload.sensors.find(i=>i.id===cameraVisual.item.id);
results.camera_authored_rpy=authoredCamera.rpy;
results.camera_owner_drawn_rotation_error_rad=rotErr(camera.object3d.matrixWorld,ros(authoredCamera.rpy));
results.camera_owner_drawn_rpy=mateRosRpyFromMatrix(camera.object3d.matrixWorld);
const cameraVisualWorld=cameraItem.final_transform||cameraItem.world_from_visual||cameraItem.pose;
results.camera_visual_drawn_rotation_error_rad=rotErr(cameraVisual.object3d.matrixWorld,ros(cameraVisualWorld.rpy));
results.camera_visual_drawn_position_error_m=new THREE.Vector3().setFromMatrixPosition(cameraVisual.object3d.matrixWorld).distanceTo(new THREE.Vector3(...cameraVisualWorld.xyz));
results.table_visual_drawn_rotation_error_rad=rotErr(tableVisual.object3d.matrixWorld,ros(tableVisual.item.final_transform.rpy));
results.table_visual_drawn_position_error_m=new THREE.Vector3().setFromMatrixPosition(tableVisual.object3d.matrixWorld).distanceTo(new THREE.Vector3(...tableVisual.item.final_transform.xyz));
for(const k of ['camera_owner_drawn_rotation_error_rad','camera_visual_drawn_rotation_error_rad','camera_visual_drawn_position_error_m','table_visual_drawn_rotation_error_rad','table_visual_drawn_position_error_m'])assert(results[k]<1e-6,k+' '+results[k]);
// Roots and a root that is already hidden before any isolation.
const roots=()=>[...state.objects,...state.pickRecords].filter(r=>r?.object3d);
const bin=resolveCanonicalPhysicalEditBinding('target_bin_default')?.owner||state.objects.find(r=>r.item.id==='target_bin_default');
assert(bin,'bin rendered');bin.object3d.visible=false;
const before=new Map(roots().map(r=>[r,r.object3d.visible]));
results.root_count=before.size;results.visible_roots_before=[...before.values()].filter(Boolean).length;
const onChain=(from,target)=>{for(let n=from;n;n=n.parent)if(n===target)return true;return false;};
const api=window.__WORKCELL_EDITOR_API_V1__;
const isolate=id=>api.setMateMode({active:true,step:'pick_child',isolate_asset:id,markers:[]});
const checkIsolated=(id,keepRoot,label)=>{
  assert.strictEqual(api.getMateState().isolatedAsset,id,label);
  let hidden=0,kept=0;
  for(const r of roots()){
    const related=r.object3d===keepRoot||onChain(keepRoot.parent,r.object3d)||onChain(r.object3d.parent,keepRoot);
    if(related&&before.get(r)){assert.strictEqual(r.object3d.visible,true,label+': '+r.item.id+' must stay visible');kept++;}
    else{assert.strictEqual(r.object3d.visible,false,label+': '+r.item.id+' must be hidden');if(before.get(r))hidden++;}
  }
  return {kept,hidden};
};
// A real camera and raycaster aimed through the centre of a mesh (50,50 of a 100x100 canvas).
const view=new THREE.PerspectiveCamera(30,1,0.01,20);
state.three.camera=view;state.three.raycaster=new THREE.Raycaster();state.three.pointer=new THREE.Vector2();
const aim=(mesh,dir)=>{const c=new THREE.Box3().setFromObject(mesh).getCenter(new THREE.Vector3());view.position.copy(c).addScaledVector(new THREE.Vector3(...dir).normalize(),1.0);view.up.set(0,0,1);view.lookAt(c);view.updateMatrixWorld(true);view.updateProjectionMatrix();};
const mateHit=()=>mateRaycast({clientX:50,clientY:50});
// 1. Camera isolated: D435 visual visible and pickable, resolves to the camera owner.
isolate('realsense_overhead');results.camera_isolation=checkIsolated('realsense_overhead',camera.object3d,'camera isolated');
assert.strictEqual(cameraVisual.object3d.visible,true);assert.strictEqual(hiddenPickSubtree(cameraVisual.meshObject),null);
aim(cameraVisual.meshObject,[1,0.3,0.4]);let hit=mateHit();results.camera_pick=hit;
assert.strictEqual(hit.asset_id,'realsense_overhead');assert(hit.geometry_source.startsWith(cameraVisual.item.id+'_physical_mesh'),hit.geometry_source);
assert(rotErr(ros(hit.object_pose_world.slice(3)),ros(authoredCamera.rpy))<1e-9,'mate pick reports the authored camera rotation');
// The table is hidden while the camera is isolated: a ray through its visual hits nothing.
aim(tableVisual.meshObject,[0,0,1]);hit=mateHit();results.table_ray_while_camera_isolated=hit;assert.strictEqual(hit.asset_id,'','hidden table must not pick');
// 2. Switch isolation to the table: its reparented visual is visible and pickable.
isolate('support_surface_table');results.table_isolation=checkIsolated('support_surface_table',table.object3d,'table isolated');
assert.strictEqual(tableVisual.object3d.visible,true);assert.strictEqual(camera.object3d.visible,false,'camera hidden after switching');
aim(tableVisual.meshObject,[0,0,1]);hit=mateHit();results.table_pick=hit;
assert.strictEqual(hit.asset_id,'support_surface_table');assert(hit.geometry_source.startsWith('fixture_generated_table_top_visual_physical_mesh'),hit.geometry_source);
assert(rotErr(ros(hit.object_pose_world.slice(3)),ros(owners.support_surface_table.pose.rpy))<1e-9,'mate pick reports the authored table rotation');
// Isolating the reparented visual itself keeps its owner (ancestor) visible.
isolate(cameraVisual.item.id);checkIsolated(cameraVisual.item.id,cameraVisual.object3d,'visual isolated');assert.strictEqual(camera.object3d.visible,true);
// 3. Clear: every root back exactly as before; the pre-hidden bin stays hidden.
api.clearMateMode();assert.strictEqual(api.getMateState().isolatedAsset,'');
for(const [r,v] of before)assert.strictEqual(r.object3d.visible,v,'restored '+r.item.id);
assert.strictEqual(bin.object3d.visible,false,'a root hidden before isolation stays hidden');
results.restored_exactly=true;
// Editor selection (not Mate) through the same real raycaster resolves to the physical edit owners.
aim(cameraVisual.meshObject,[1,0.3,0.4]);results.select_camera=pickObject({clientX:50,clientY:50});assert.strictEqual(results.select_camera,'realsense_overhead');
aim(tableVisual.meshObject,[0,0,1]);results.select_table=pickObject({clientX:50,clientY:50});assert.strictEqual(results.select_table,'support_surface_table');
// Selecting compound-rotated owners reads back exactly (no pose echo).
selectObject('realsense_overhead');const sel=window.__WORKCELL_EDITOR_API_V1__.getState().selectedTransform;results.selected_camera_rpy=sel.pose.rpy;
[sel.pose.rpy.x,sel.pose.rpy.y,sel.pose.rpy.z].forEach((v,i)=>assert.strictEqual(v,authoredCamera.rpy[i],'selected camera rpy['+i+'] reads back exactly'));
assert.strictEqual(state.dirtyTransforms.size,0);assert.strictEqual(buildEditPatch().edits.length,0);
results.result='PASS';
`, context);
fs.writeFileSync(resultPath, JSON.stringify(results, null, 1));
console.log('PASS', JSON.stringify({ camera_isolation: results.camera_isolation, table_isolation: results.table_isolation, select: [results.select_camera, results.select_table] }));
