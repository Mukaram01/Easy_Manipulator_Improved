from pathlib import Path
import yaml, json, subprocess, sys
ROOT=Path(__file__).resolve().parents[1]
MERGE=ROOT/'scripts'/'workcell_studio_layout_merge.py'

def test_layout_pose_overrides_environment(tmp_path: Path):
    s=tmp_path/'s'; (s/'layout').mkdir(parents=True); (s/'config').mkdir()
    (s/'environment.yaml').write_text('objects: [{id: obj1, pose: {xyz: [0,0,0]}}]\n')
    (s/'scene_manifest.yaml').write_text('objects: []\n')
    (s/'layout'/'workcell_studio_layout.yaml').write_text('items: [{id: obj1, pose: {xyz: [9,9,9]}}]\n')
    (s/'config'/'workcell_builder_task_intent.yaml').write_text('{}\n'); (s/'config'/'task_recipe.yaml').write_text('{}\n')
    subprocess.run([sys.executable,str(MERGE),str(s)],check=True)
    merged=yaml.safe_load((s/'generated'/'workcell_studio_merged_environment.yaml').read_text())
    assert merged['objects'][0]['pose']['xyz']==[9,9,9]
