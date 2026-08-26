import json, shutil, subprocess, sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
SCRIPT=ROOT/'scripts/validate_workcell_studio_generated_scene.py'

def _run(scene:Path):
    p=subprocess.run([sys.executable,str(SCRIPT),str(scene),'--json'],capture_output=True,text=True,check=False)
    return p, json.loads(p.stdout)

def test_acceptance_on_existing_scene(tmp_path: Path):
    src=next((ROOT/'scenes').iterdir())
    scene=tmp_path/src.name
    shutil.copytree(src,scene)
    p, out=_run(scene)
    assert p.returncode in (0,1)
    assert out['status'] in {'PASS','WARNINGS','PREVIEW_ONLY','BLOCKED'}
    a=scene/'acceptance'
    assert (a/'generated_scene_acceptance.json').is_file()
    assert (a/'generated_scene_acceptance.html').is_file()
    assert (a/'generated_scene_acceptance_summary.txt').is_file()


def test_missing_environment_blocked(tmp_path: Path):
    scene=tmp_path/'ur5_demo'
    scene.mkdir(); (scene/'package.xml').write_text('x'); (scene/'CMakeLists.txt').write_text('x')
    p,out=_run(scene)
    assert out['status'] in {'MISSING_ENVIRONMENT_YAML','BLOCKED'}
    assert p.returncode==1


def test_acceptance_fingerprint_tracks_authored_inputs_not_checkout_timestamps(tmp_path: Path):
    scene = tmp_path / 'fingerprint_scene'
    (scene / 'config').mkdir(parents=True)
    (scene / 'environment.yaml').write_text('scene: {name: Fingerprint}\n', encoding='utf-8')
    (scene / 'config/task_recipe.yaml').write_text('task: {type: pick_place}\n', encoding='utf-8')
    _, first = _run(scene)
    fingerprint = first['authored_input_fingerprint']
    assert len(fingerprint) == 16

    _, unchanged = _run(scene)
    assert unchanged['authored_input_fingerprint'] == fingerprint

    with (scene / 'config/task_recipe.yaml').open('a', encoding='utf-8') as stream:
        stream.write('notes: changed\n')
    _, changed = _run(scene)
    assert changed['authored_input_fingerprint'] != fingerprint
