from __future__ import annotations
import json, subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CHECK = REPO_ROOT / 'scripts' / 'check_planning_scene_readiness.py'
VALIDATE = REPO_ROOT / 'scripts' / 'validate_planning_scene_readiness_report.py'
CLI = REPO_ROOT / 'scripts' / 'workcell_studio.py'

def run(*args):
    return subprocess.run(['python3', *map(str,args)], capture_output=True, text=True, check=False)

def test_physical_scene_only_warn(tmp_path):
    out=tmp_path/'out'
    proc=run(CHECK,'--scene-package',REPO_ROOT/'scenes/ur5_2f_test','--output-dir',out,'--json')
    assert proc.returncode==0
    rep=json.loads((out/'planning_scene_readiness_report.json').read_text())
    assert rep['result']['readiness']=='WARN'

def test_with_task_and_request(tmp_path):
    recipe=REPO_ROOT/'tests/fixtures/task_recipes/valid_pick_place.yaml'
    req=tmp_path/'req.json'
    req.write_text(json.dumps({
        "schema": "offline_plan_preview_request/v1",
        "request": {
            "pick": {"source_id": "pick_zone"},
            "place": {"target_id": "bin_a"},
            "tool": {"grasp_strategy": "finger_pinch_basic"},
            "waypoints": ["w1"],
        },
    }), encoding="utf-8")
    out=tmp_path/'out'
    proc=run(CHECK,'--scene-package',REPO_ROOT/'scenes/ur5_2f_test','--output-dir',out,'--task-recipe',recipe,'--plan-preview-request',req,'--json')
    rep=json.loads((out/'planning_scene_readiness_report.json').read_text())
    assert 'plan_preview_request_ready' in rep['result']['classification']
    assert rep['checks']['task']['waypoint_count'] >= 0

def test_validate_cli(tmp_path):
    out=tmp_path/'out'; run(CHECK,'--scene-package',REPO_ROOT/'scenes/ur5_2f_test','--output-dir',out,'--json')
    proc=run(VALIDATE,out/'planning_scene_readiness_report.json','--json')
    assert proc.returncode==0
    proc2=run(CLI,'check-planning-scene-readiness','--scene-package',REPO_ROOT/'scenes/ur5_2f_test','--output-dir',out,'--json')
    assert proc2.returncode==0
