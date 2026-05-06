from __future__ import annotations
import json, subprocess, sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CLI = REPO_ROOT / 'scripts' / 'convert_builder_task_intent_to_task_recipe.py'


def test_valid_conversion(tmp_path: Path) -> None:
    out = tmp_path / 'task_recipe.yaml'
    proc = subprocess.run([sys.executable, str(CLI), '--task-intent', str(REPO_ROOT/'tests/fixtures/builder_task_intent_valid.yaml'), '--output', str(out), '--validate', '--json'], capture_output=True, text=True)
    assert proc.returncode == 0
    assert out.is_file()
    payload = out.read_text()
    assert 'pick_zone_main' in payload
    assert 'bin_red' in payload
    assert 'suction_top_basic' in payload
    assert 'tool_release' in payload


def test_missing_pick_place_fails(tmp_path: Path) -> None:
    out = tmp_path / 'task_recipe.yaml'
    proc = subprocess.run([sys.executable, str(CLI), '--task-intent', str(REPO_ROOT/'tests/fixtures/builder_task_intent_missing_pick.yaml'), '--output', str(out), '--validate'], capture_output=True, text=True)
    assert proc.returncode != 0


def test_json_output_fields(tmp_path: Path) -> None:
    out = tmp_path / 'task_recipe.yaml'
    proc = subprocess.run([sys.executable, str(CLI), '--task-intent', str(REPO_ROOT/'tests/fixtures/builder_task_intent_valid.yaml'), '--output', str(out), '--validate', '--json'], capture_output=True, text=True)
    data = json.loads(proc.stdout)
    for k in ['status','generated_task_recipe_path','pick_source','place_target','grasp_strategy','routing_rule_count']:
        assert k in data
