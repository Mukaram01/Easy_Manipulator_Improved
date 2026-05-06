from __future__ import annotations
import json, subprocess, tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def test_generate_and_validate_request() -> None:
    with tempfile.TemporaryDirectory() as d:
        out = Path(d)/'req.yaml'
        recipe = Path(d)/'recipe.yaml'
        recipe.write_text("""schema_version: task_recipe/v1\ntask:\n  id: pick_place_demo\n  source: src\n  destinations:\n    - id: bin\ngrasp:\n  strategy_ref: suction_top_basic\n""", encoding='utf-8')
        run = subprocess.run(['python3', str(REPO/'scripts'/'generate_offline_plan_preview_request.py'),'--task-recipe',str(recipe),'--output',str(out),'--validate','--json'],capture_output=True,text=True,check=False)
        assert run.returncode == 0
        data = json.loads(run.stdout)
        assert data['status'] in {'PASS','WARN'}
        assert out.exists()
        v = subprocess.run(['python3', str(REPO/'scripts'/'validate_offline_plan_preview_request.py'), str(out), '--json'],capture_output=True,text=True,check=False)
        assert v.returncode == 0


def test_allow_incomplete() -> None:
    with tempfile.TemporaryDirectory() as d:
        recipe = Path(d)/'bad.yaml'
        recipe.write_text("""schema_version: task_recipe/v1\ntask:\n  id: x\ngrasp: {}\n""", encoding='utf-8')
        out = Path(d)/'req.yaml'
        run = subprocess.run(['python3', str(REPO/'scripts'/'generate_offline_plan_preview_request.py'),'--task-recipe',str(recipe),'--output',str(out),'--allow-incomplete','--json'],capture_output=True,text=True,check=False)
        assert run.returncode == 0
        assert json.loads(run.stdout)['readiness_classification'] == 'plan_preview_request_incomplete'
