from __future__ import annotations
import json, subprocess, tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CLI = REPO_ROOT / 'scripts' / 'summarize_task_flow.py'


def run(*args:str):
    p=subprocess.run(['python3',str(CLI),*args],capture_output=True,text=True,check=False)
    return p, json.loads(p.stdout)


def test_summarize_valid_builder_task_intent():
    p,j=run('--task-intent',str(REPO_ROOT/'tests/fixtures/builder_task_intent_valid.yaml'),'--json')
    assert p.returncode==0
    assert j['readiness_classification']=='task_intent_ready_offline'
    assert j['pick_source_id'] and j['place_target_id'] and j['grasp_strategy'] and j['release_strategy']
    assert 'pick_source_type' in j


def test_summarize_task_recipe():
    with tempfile.TemporaryDirectory() as d:
        out=Path(d)/'recipe.yaml'
        subprocess.run(['python3',str(REPO_ROOT/'scripts/convert_builder_task_intent_to_task_recipe.py'),'--task-intent',str(REPO_ROOT/'tests/fixtures/builder_task_intent_valid.yaml'),'--output',str(out),'--validate'],check=False)
        p,j=run('--task-recipe',str(out),'--json')
        assert j['readiness_classification']=='task_recipe_generated'


def test_missing_pick_place_fail():
    p,j=run('--task-intent',str(REPO_ROOT/'tests/fixtures/builder_task_intent_missing_pick.yaml'),'--json')
    assert j['status']=='FAIL'
    assert 'pick.source.id' in j['missing_required_fields']


def test_physical_scene_only():
    with tempfile.TemporaryDirectory() as d:
        p,j=run('--scene-package',d,'--json')
        assert j['readiness_classification']=='physical_scene_only'


def test_visual_resolution_flags():
    layout=REPO_ROOT/'tests/fixtures/environment_layouts/ur5_table_bins_existing_assets.layout.yaml'
    p,j=run('--task-intent',str(REPO_ROOT/'tests/fixtures/builder_task_intent_valid.yaml'),'--environment-layout',str(layout),'--json')
    assert 'visual_resolution' in j
