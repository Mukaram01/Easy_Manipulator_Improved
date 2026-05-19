from __future__ import annotations
import json, subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT/'scripts/validate_all_scene_builder_reproducibility.py'
FIX = ROOT/'tests/fixtures/all_scene_builder_reproducibility'

def test_reproducibility_report_contains_all_scenes_and_statuses(tmp_path:Path):
    out = tmp_path/'report.json'
    res = subprocess.run([sys.executable, str(SCRIPT), '--scenes-root', str(FIX), '--json', '--output', str(out)], cwd=ROOT, capture_output=True, text=True)
    assert res.returncode == 0
    data=json.loads(out.read_text())
    names={s['scene_name']:s for s in data['per_scene']}
    assert set(names) >= {'valid_scene','preview_only','malformed','missing_meta'}
    assert names['valid_scene']['status'] in {'PASS','WARN','BLOCKED'}
    assert names['preview_only']['starter_layout_created'] is True
    assert names['malformed']['status'] == 'BLOCKED'
    assert names['missing_meta']['status'] == 'BLOCKED'


def test_source_scenes_not_mutated_default():
    assert not (FIX/'preview_only/layout/workcell_studio_layout.yaml').exists()
