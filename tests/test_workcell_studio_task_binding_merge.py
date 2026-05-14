from pathlib import Path
import yaml, subprocess, sys
ROOT=Path(__file__).resolve().parents[1]
MERGE=ROOT/'scripts'/'workcell_studio_layout_merge.py'

def test_task_bindings_propagate(tmp_path: Path):
    s=tmp_path/'s'; (s/'layout').mkdir(parents=True); (s/'config').mkdir()
    (s/'environment.yaml').write_text('objects: []\n'); (s/'scene_manifest.yaml').write_text('objects: []\n')
    (s/'layout'/'workcell_studio_layout.yaml').write_text('task_bindings: {pick_source: pick_zone_main, place_target: bin_red, camera: cam1}\n')
    (s/'config'/'workcell_builder_task_intent.yaml').write_text('{}\n'); (s/'config'/'task_recipe.yaml').write_text('{}\n')
    subprocess.run([sys.executable,str(MERGE),str(s)],check=True)
    ti=yaml.safe_load((s/'generated'/'workcell_builder_task_intent.yaml').read_text())
    assert ti['pick']['source']['id']=='pick_zone_main'
    assert ti['place']['target']['id']=='bin_red'
