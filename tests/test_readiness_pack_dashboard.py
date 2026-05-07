from __future__ import annotations
import json, subprocess, sys, shutil
from pathlib import Path
from tools.workcell_studio_streamlit import backend


def _mk_scene(tmp_path: Path) -> Path:
    scene = tmp_path / 'scene'
    shutil.copytree('scenes/ur5_2f_test', scene)
    gen = scene / 'generated'
    gen.mkdir(parents=True, exist_ok=True)
    subprocess.run([
        sys.executable,'scripts/create_or_update_builder_task_intent.py','--scene-package',str(scene),'--task-id','sorting_task_001','--task-type','pick_place','--pick-source','pick_zone_main','--place-target','bin_red','--grasp-strategy','finger_pinch_basic','--approach-axis','z_down','--approach-distance-m','0.12','--retreat-axis','z_up','--retreat-distance-m','0.10','--release-strategy','tool_release','--object-class','any','--object-color','red','--output',str(gen/'workcell_builder_task_intent.yaml'),'--validate','--json'
    ], check=True)
    return scene


def test_generate_dashboard_from_pack(tmp_path: Path):
    out = tmp_path/'pack'
    scene = _mk_scene(tmp_path)
    subprocess.run([sys.executable,'scripts/workcell_studio.py','generate-readiness-pack','--scene-package',str(scene),'--output-dir',str(out),'--project-name','demo','--validate','--smoke-dry-run','--force','--json'],check=False)
    dash = out/'readiness_dashboard.html'
    assert dash.exists()
    txt=dash.read_text(encoding='utf-8')
    for k in ['pick_zone_main','bin_red','finger_pinch_basic','task_flow_summary.json']:
        assert k in txt
    assert 'pick source: missing' not in txt
    assert "href='/tmp/" not in txt and 'href="/tmp/' not in txt
    assert 'preview/static_preview.svg' in txt
    assert 'preview/static_preview.html' in txt
    assert 'No next_commands.md found' not in txt


def test_generate_dashboard_missing_artifacts(tmp_path: Path):
    m=tmp_path/'manifest.json'
    m.write_text(json.dumps({'schema':'workcell_studio_readiness_pack/v1','source':{'project_name':'x'},'artifacts':{},'results':{'final_readiness':'WARN'},'safety':{},'summary':{}}),encoding='utf-8')
    dash=tmp_path/'d.html'
    rc=subprocess.run([sys.executable,'scripts/generate_readiness_pack_dashboard.py','--manifest',str(m),'--output',str(dash)],check=False)
    assert rc.returncode==0 and dash.exists()
    assert 'MISSING' in dash.read_text(encoding='utf-8')


def test_streamlit_backend_dashboard_helpers(tmp_path: Path):
    m=tmp_path/'m.json'; out=tmp_path/'x.html'
    m.write_text(json.dumps({'schema':'workcell_studio_readiness_pack/v1','source':{'project_name':'x'},'artifacts':{},'results':{'final_readiness':'WARN'},'safety':{},'summary':{}}),encoding='utf-8')
    run=backend.generate_readiness_dashboard(m,out)
    assert run['returncode']==0
    assert backend.read_readiness_dashboard(out)
    assert backend.dashboard_path_from_manifest({'artifacts':{'readiness_dashboard':'abc'}})=='abc'
