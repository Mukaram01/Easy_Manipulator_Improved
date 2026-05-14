from pathlib import Path
import yaml, subprocess, sys, json
ROOT=Path(__file__).resolve().parents[1]
MERGE=ROOT/'scripts'/'workcell_studio_layout_merge.py'

def test_added_asset_and_missing_paths_warning(tmp_path: Path):
    s=tmp_path/'s'; (s/'layout').mkdir(parents=True); (s/'config').mkdir()
    (s/'environment.yaml').write_text('objects: []\n'); (s/'scene_manifest.yaml').write_text('objects: []\n')
    (s/'layout'/'workcell_studio_layout.yaml').write_text('items: [{id: bin_new, type: bin}]\n')
    (s/'config'/'workcell_builder_task_intent.yaml').write_text('{}\n'); (s/'config'/'task_recipe.yaml').write_text('{}\n')
    subprocess.run([sys.executable,str(MERGE),str(s)],check=True)
    mm=yaml.safe_load((s/'generated'/'workcell_studio_merged_scene_manifest.yaml').read_text())
    assert any(o.get('id')=='bin_new' for o in mm['objects'])
    rep=json.loads((s/'generated'/'workcell_studio_layout_merge_report.json').read_text())
    assert rep['merge_warnings']
