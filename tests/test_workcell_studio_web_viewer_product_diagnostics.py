from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web" / "viewer"


def test_scene_health_ui_remains_visible_in_embedded_product_view() -> None:
    index = (VIEWER / "index.html").read_text(encoding="utf-8")
    css = (VIEWER / "style.css").read_text(encoding="utf-8")

    for element_id in [
        "scene-health",
        "scene-health-title",
        "scene-health-summary",
        "scene-health-count",
        "scene-health-items",
    ]:
        assert f'id="{element_id}"' in index
    assert "What happened and what to do next" in index
    assert "body.embedded-mode #scene-health" not in css
    assert ".health-loading" in css
    assert ".health-ready" in css
    assert ".health-warning" in css
    assert ".health-failed" in css


def test_product_diagnostics_classify_failures_and_give_next_actions() -> None:
    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const element=()=>({hidden:false,open:false,checked:false,disabled:false,value:'',textContent:'',className:'',innerHTML:'',dataset:{},style:{},classList:{add(){},remove(){},toggle(){}},querySelector(){return null},querySelectorAll(){return[]},addEventListener(){},setAttribute(){},appendChild(){},remove(){},getBoundingClientRect(){return {left:0,top:0,width:100,height:100}}});
const context={console,assert,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}},addEventListener(){}},document:{body:{classList:{add(){}}},getElementById(){return element()},querySelectorAll(){return[]},createElement(){return element()},addEventListener(){}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){return 0},clearTimeout(){}};
vm.createContext(context);vm.runInContext(source+`
const classify=(raw,expected)=>assert.strictEqual(productDiagnosticKind(raw),expected);
classify({code:'missing_file',message:'Mesh file not found'},'missing_mesh');
classify({code:'unsupported_format',reason:'unsupported mesh format .step'},'unsupported_format');
classify({code:'unresolved_package_uri',reason:'package URI was not staged for browser loading: package://demo/part.dae'},'unresolved_package_uri');
classify({code:'loaded_mesh_collapsed',visual_bounds_status:'loaded_mesh_collapsed'},'invalid_scale');
classify({code:'loader_failure',reason:'physical asset failed to load'},'mesh_load_failed');
classify({code:'artifact_stale',message:'Generated artifact is stale'},'generated_stale');
classify({code:'collision',message:'Collision warning'},'collision');
assert.strictEqual(productDiagnosticKind({code:'optional_file_missing',message:'Optional input file is missing'}),'');

for(const kind of Object.keys(PRODUCT_DIAGNOSTIC_DEFINITIONS)){
  const diagnostic=normalizeProductDiagnostic({code:kind,reason:'details',object_id:'part'},kind);
  assert(diagnostic.title.length>5,kind+' title');
  assert(diagnostic.action.length>12,kind+' action');
  assert(['error','warning'].includes(diagnostic.severity),kind+' severity');
}

renderedPhysicalItemCount=()=>4;
state.sceneJson={scene:{id:'ur5_2f_test'},warnings:[{code:'artifact_stale',message:'Generated artifact stale after layout save'}]};
state.runtimeWarnings=[{code:'missing_file',object_id:'fixture_1',mesh_uri:'assets/fixture.stl',message:'Mesh file not found'}];
state.objects=[
  {item:{id:'fixture_1',render_policy:'primary',mesh_status:'missing_file',mesh_uri:'assets/fixture.stl'},renderInfo:{render_status:'required_mesh_failed_debug_fallback',mesh_uri:'assets/fixture.stl',fallback_reason:'Mesh file not found'}},
  {item:{id:'bad_scale',render_policy:'primary',mesh_status:'loaded',mesh_scale:[1,0,1]},renderInfo:{render_status:'mesh_loaded',mesh_uri:'assets/bad.obj'}},
];
state.web3dReadiness={state:'scene_ready',terminal:true,pending:new Set(),required:{},failed:false,failure:null};
let model=sceneHealthModel();
assert.strictEqual(model.state,'failed');
assert(model.diagnostics.some(item=>item.kind==='missing_mesh'&&item.itemId==='fixture_1'));
assert(model.diagnostics.some(item=>item.kind==='invalid_scale'&&item.itemId==='bad_scale'));
assert(model.diagnostics.some(item=>item.kind==='generated_stale'));
assert.strictEqual(model.diagnostics.filter(item=>item.kind==='missing_mesh'&&item.itemId==='fixture_1').length,1,'dedupe per item/category');

state.placement={armed:true,collision:true,collidingOwnerIds:['table_main'],asset:{id:'new_guard'}};
model=sceneHealthModel();
assert(model.diagnostics.some(item=>item.kind==='collision'&&item.itemId==='new_guard'));

state.sceneJson={scene:{id:'ur5_2f_test'},warnings:[]};state.runtimeWarnings=[];state.objects=[];
state.placement={armed:false,collision:false,collidingOwnerIds:[]};
state.web3dReadiness={state:'scene_ready',terminal:true,pending:new Set(),required:{},failed:false,failure:null};
state.editorError='';
model=sceneHealthModel();assert.strictEqual(model.state,'ready');assert.strictEqual(model.title,'Scene ready');assert(model.summary.includes('safe to edit'));
const rendered=renderSceneHealth(model);assert.strictEqual(rendered.state,'ready');

state.web3dReadiness={state:'scene_failed',terminal:true,pending:new Set(),required:{},failed:true,failure:{reason:'camera mesh failed',item_id:'camera'}};
model=sceneHealthModel();assert.strictEqual(model.state,'failed');assert(model.diagnostics.some(item=>item.kind==='scene_failed'));
`,context);
"""
    subprocess.run(
        ["node", "-e", harness, str(VIEWER / "viewer.js")],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def test_machine_readable_viewer_status_exposes_product_diagnostics() -> None:
    source = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    for token in [
        "scene_health_state",
        "product_diagnostics",
        "product_diagnostic_error_count",
        "product_diagnostic_warning_count",
    ]:
        assert token in source
