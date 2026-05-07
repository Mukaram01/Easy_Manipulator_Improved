from __future__ import annotations
import json, subprocess, sys
from pathlib import Path
from tools.workcell_studio_streamlit import backend


def test_generate_dashboard_from_pack(tmp_path: Path):
    out = tmp_path/'pack'
    subprocess.run([sys.executable,'scripts/workcell_studio.py','generate-readiness-pack','--scene-package','scenes/ur5_2f_test','--output-dir',str(out),'--project-name','demo','--validate','--smoke-dry-run','--force','--json'],check=False)
    dash = out/'readiness_dashboard.html'
    assert dash.exists()
    txt=dash.read_text(encoding='utf-8')
    for k in ['Workcell Studio','Final readiness','No robot motion','not a safety certificate','Artifact status']:
        assert k in txt


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
