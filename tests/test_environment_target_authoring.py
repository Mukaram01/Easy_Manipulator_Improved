from __future__ import annotations
import json, subprocess, sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]

def _run(cmd):
    return subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, check=False)

def test_environment_target_authoring_flow(tmp_path):
    layout = tmp_path/'environment_layout.yaml'
    r1=_run([sys.executable,'scripts/create_or_update_environment_target.py','--environment-layout',str(layout),'--target-id','pick_zone_main','--target-type','pick_zone','--label','Main pick zone','--frame','world','--xyz','0.45','0','0.08','--rpy','0','0','0','--size','0.3','0.2','0.1','--output',str(layout),'--json'])
    assert r1.returncode==0 and layout.exists()
    assert json.loads(r1.stdout)['updated_existing'] is False
    r2=_run([sys.executable,'scripts/create_or_update_environment_target.py','--environment-layout',str(layout),'--target-id','bin_red','--target-type','place_target','--label','Red bin','--frame','world','--xyz','0.35','0.35','0.10','--rpy','0','0','0','--size','0.2','0.2','0.15','--output',str(layout),'--json'])
    assert r2.returncode==0 and json.loads(r2.stdout)['updated_existing'] is False
    r3=_run([sys.executable,'scripts/create_or_update_environment_target.py','--environment-layout',str(layout),'--target-id','pick_zone_main','--target-type','pick_zone','--label','Main pick zone','--frame','world','--xyz','0.50','0','0.08','--rpy','0','0','0','--size','0.3','0.2','0.1','--output',str(layout),'--json'])
    assert json.loads(r3.stdout)['updated_existing'] is True
