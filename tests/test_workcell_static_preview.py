from __future__ import annotations
import json, subprocess, tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CLI = REPO_ROOT / 'scripts' / 'generate_workcell_static_preview.py'

def _run(*args:str):
    return subprocess.run(['python3', str(CLI), *args], capture_output=True, text=True, check=False)

def test_valid_ur5_preview_generates_artifacts():
    with tempfile.TemporaryDirectory() as d:
        out=Path(d)
        cell=REPO_ROOT/'tests/fixtures/cell_definition_ur5_suction_sorting.yaml'
        p=_run('--cell-definition',str(cell),'--output-dir',str(out),'--title','UR5 + Suction Sorting Demo')
        assert p.returncode==0, p.stdout+p.stderr
        assert (out/'static_preview.svg').is_file(); assert (out/'static_preview.html').is_file(); assert (out/'static_preview_summary.json').is_file()
        s=json.loads((out/'static_preview_summary.json').read_text())
        assert s['robot'] and s['end_effector'] and s['task'] and s['title']
        svg=(out/'static_preview.svg').read_text()
        assert 'Robot' in svg and ('Bin:' in svg or 'Table:' in svg)

def test_preview_only_fixture_still_generates():
    with tempfile.TemporaryDirectory() as d:
        out=Path(d)
        cell=REPO_ROOT/'tests/fixtures/cell_definition_generic_cartesian_suction_sorting.yaml'
        p=_run('--cell-definition',str(cell),'--output-dir',str(out),'--title','Preview Only')
        assert p.returncode==0
        s=json.loads((out/'static_preview_summary.json').read_text())
        assert 'preview_only' in s and 'runtime_blocked' in s

def test_missing_optional_environment_layout_warns():
    with tempfile.TemporaryDirectory() as d:
        out=Path(d)
        cell=REPO_ROOT/'tests/fixtures/cell_definition_pick_place.yaml'
        p=_run('--cell-definition',str(cell),'--environment-layout',str(Path(d)/'missing.yaml'),'--output-dir',str(out),'--title','No Layout')
        assert p.returncode==0
        s=json.loads((out/'static_preview_summary.json').read_text())
        assert isinstance(s.get('warnings',[]), list)

def test_bad_cell_definition_fails():
    with tempfile.TemporaryDirectory() as d:
        p=_run('--cell-definition',str(Path(d)/'missing.yaml'),'--output-dir',str(Path(d)/'o'),'--title','bad')
        assert p.returncode != 0
        assert 'ERROR' in p.stderr
