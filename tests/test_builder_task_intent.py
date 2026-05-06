from __future__ import annotations
import json, subprocess
from pathlib import Path
REPO_ROOT=Path(__file__).resolve().parents[1]
CLI=REPO_ROOT/'scripts'/'validate_builder_task_intent.py'

def _run(f:str):
    return subprocess.run(['python3',str(CLI),str(REPO_ROOT/'tests'/'fixtures'/f),'--grasp-strategies-dir',str(REPO_ROOT/'catalog'/'grasp_strategies'),'--json'],capture_output=True,text=True,check=False)

def test_valid():
    r=_run('builder_task_intent_valid.yaml'); assert r.returncode==0; assert json.loads(r.stdout)['status'] in {'PASS','WARN'}

def test_missing_pick_fails():
    r=_run('builder_task_intent_missing_pick.yaml'); assert r.returncode!=0

def test_missing_place_fails():
    r=_run('builder_task_intent_missing_place.yaml'); assert r.returncode!=0

def test_unknown_grasp_warns():
    p=REPO_ROOT/'tests'/'fixtures'/'builder_task_intent_valid.yaml'
    t=p.read_text(); tmp=p.parent/'_tmp_unknown.yaml'; tmp.write_text(t.replace('suction_top_basic','unknown_strategy'))
    r=subprocess.run(['python3',str(CLI),str(tmp),'--grasp-strategies-dir',str(REPO_ROOT/'catalog'/'grasp_strategies'),'--json'],capture_output=True,text=True,check=False)
    payload=json.loads(r.stdout); assert payload['status'] in {'WARN','PASS'}

def test_safety_conservative_required():
    payload=json.loads(_run('builder_task_intent_valid.yaml').stdout)
    s=payload['safety']; assert s['metadata_only'] is True and s['runtime_io_applied'] is False and s['motion_started'] is False and s['ros_launch_started'] is False
