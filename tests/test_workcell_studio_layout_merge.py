import json, subprocess, sys
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[1]
CLI=ROOT/'scripts'/'workcell_studio_layout_merge.py'

def test_layout_merge_helper_exists_and_outputs_report(tmp_path: Path):
    scene=tmp_path/'scene'; (scene/'layout').mkdir(parents=True); (scene/'config').mkdir()
    (scene/'environment.yaml').write_text('objects: [{id: obj1, pose: {xyz: [0,0,0]}}]\n',encoding='utf-8')
    (scene/'scene_manifest.yaml').write_text('objects: [{id: obj1, metadata: m}]\n',encoding='utf-8')
    (scene/'layout'/'workcell_studio_layout.yaml').write_text('saved_at_utc: 2026-01-01T00:00:00Z\nitems: [{id: obj1, pose: {xyz: [1,2,3]}}]\n',encoding='utf-8')
    (scene/'config'/'workcell_builder_task_intent.yaml').write_text('safety: {}\n',encoding='utf-8')
    (scene/'config'/'task_recipe.yaml').write_text('safety: {}\n',encoding='utf-8')
    proc=subprocess.run([sys.executable,str(CLI),str(scene),'--json'],capture_output=True,text=True,check=False)
    assert proc.returncode==0
    rep=json.loads(proc.stdout)
    assert rep['layout_applied'] is True
    assert (scene/'generated'/'workcell_studio_layout_merge_report.json').exists()
