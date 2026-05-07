from __future__ import annotations
import json, subprocess, tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

def _run(*args:str):
    return subprocess.run(["python3", *args], cwd=REPO_ROOT, capture_output=True, text=True, check=False)

def test_generate_and_validate_session():
    with tempfile.TemporaryDirectory() as d:
        recipe=Path(d)/'task_recipe.yaml'
        recipe.write_text((REPO_ROOT/'tests/fixtures/task_recipes/valid_pick_place.yaml').read_text(), encoding='utf-8')
        req=Path(d)/'offline.yaml'
        r1=_run('scripts/generate_offline_plan_preview_request.py','--task-recipe',str(recipe),'--output',str(req),'--allow-incomplete','--json')
        assert r1.returncode==0
        out=Path(d)/'sess'
        r2=_run('scripts/generate_rviz_moveit_plan_preview_session.py','--scene-package','scenes/ur5_2f_test','--plan-preview-request',str(req),'--output-dir',str(out),'--allow-missing-launch','--json')
        assert r2.returncode==0
        assert (out/'rviz_moveit_plan_preview_session.json').is_file()
        assert (out/'rviz_moveit_plan_preview_session.md').is_file()
        cmd=(out/'suggested_commands.sh').read_text()
        assert 'use_fake_hardware:=false' not in cmd
        assert 'use_fake_hardware:=true' in cmd
        assert 'smoke-launch-preview' in cmd
        r3=_run('scripts/validate_rviz_moveit_plan_preview_session.py',str(out/'rviz_moveit_plan_preview_session.json'),'--json')
        assert r3.returncode==0

def test_unsafe_command_fails_validation():
    with tempfile.TemporaryDirectory() as d:
        p=Path(d)/'s.json'
        p.write_text(json.dumps({'schema':'rviz_moveit_plan_preview_session/v1','session':{'launch_allowed':False,'generated_commands_only':True},'safety':{'motion_started':False,'moveit_service_called':False,'ros_launch_started':False,'runtime_io_applied':False,'fake_hardware_required':True},'rviz_moveit':{'suggested_launch':{'command':'ros2 launch x demo.launch.py use_fake_hardware:=false'}}}, indent=2))
        r=_run('scripts/validate_rviz_moveit_plan_preview_session.py',str(p),'--json')
        assert r.returncode!=0
