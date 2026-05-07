from __future__ import annotations
import json, subprocess, sys
from pathlib import Path
from tools.workcell_studio_streamlit import backend

REPO = Path(__file__).resolve().parents[1]
SCENE = REPO / 'scenes' / 'ur5_2f_test'

def _run(cmd):
    return subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, check=False)

def test_authoring_flow(tmp_path):
    out = tmp_path/'workcell_builder_task_intent.yaml'
    r=_run([sys.executable,'scripts/create_or_update_builder_task_intent.py','--scene-package',str(SCENE),'--task-id','sorting_task_001','--task-type','pick_place','--pick-source','pick_zone_main','--place-target','bin_red','--grasp-strategy','finger_pinch_basic','--approach-axis','z_down','--approach-distance-m','0.12','--retreat-axis','z_up','--retreat-distance-m','0.10','--release-strategy','tool_release','--object-class','any','--object-color','red','--output',str(out),'--validate','--json'])
    assert r.returncode==0
    payload=json.loads(r.stdout)
    assert payload['pick_source']=='pick_zone_main'
    data=backend.load_builder_task_intent(out)
    assert data['place']['target']['id']=='bin_red'
    assert data['safety']['metadata_only'] is True and data['safety']['motion_started'] is False
    rv=_run([sys.executable,'scripts/validate_builder_task_intent.py',str(out),'--scene-package',str(SCENE),'--json'])
    assert rv.returncode==0
    recipe=tmp_path/'recipe.yaml'
    rc=_run([sys.executable,'scripts/convert_builder_task_intent_to_task_recipe.py','--task-intent',str(out),'--output',str(recipe),'--validate','--json'])
    assert rc.returncode==0
    sm=_run([sys.executable,'scripts/summarize_task_flow.py','--task-recipe',str(recipe),'--json'])
    assert sm.returncode==0
    assert json.loads(sm.stdout)['pick_source_id']=='pick_zone_main'

def test_list_and_wrappers(tmp_path):
    r=_run([sys.executable,'scripts/list_builder_scene_authoring_targets.py','--scene-package',str(SCENE),'--json'])
    assert r.returncode==0 and json.loads(r.stdout)['result']=='PASS'
    r2=_run([sys.executable,'scripts/workcell_studio.py','list-builder-targets','--scene-package',str(SCENE),'--json'])
    assert r2.returncode==0
    out=tmp_path/'intent.yaml'
    r3=_run([sys.executable,'scripts/workcell_studio.py','author-builder-task','--scene-package',str(SCENE),'--task-id','sorting_task_001','--task-type','pick_place','--pick-source','pick_zone_main','--place-target','bin_red','--grasp-strategy','finger_pinch_basic','--output',str(out),'--validate','--json'])
    assert r3.returncode==0
    b1=backend.list_builder_scene_authoring_targets(SCENE)
    assert b1['returncode']==0
    b2=backend.create_or_update_builder_task_intent(SCENE,'sorting_task_001','pick_place','pick_zone_main','bin_red','finger_pinch_basic',output_path=out)
    assert b2['returncode']==0
