from pathlib import Path
import json, zipfile, subprocess

def test_scripts_exist_and_markers():
    for p in ['scripts/export_workcell_scene_bundle.py','scripts/import_workcell_scene_bundle.py','scripts/validate_workcell_scene_bundle.py']:
        assert Path(p).exists()
    txt=Path('scripts/validate_workcell_scene_bundle.py').read_text(encoding='utf-8')
    assert 'WORKCELL_SCENE_BUNDLE: PASS' in txt
    assert 'WORKCELL_SCENE_BUNDLE: WARN' in txt
    assert 'WORKCELL_SCENE_BUNDLE: FAIL' in txt

def test_export_and_manifest(tmp_path):
    scene=tmp_path/'s'; (scene/'config').mkdir(parents=True); (scene/'preview').mkdir();
    (scene/'environment.yaml').write_text('schema_version: workcell_scene/v1\nsafety:\n  fake_hardware_first: true\n',encoding='utf-8')
    (scene/'config'/'task_recipe.yaml').write_text('schema_version: workcell_task/v1\n',encoding='utf-8')
    (scene/'preview'/'workcell_preview.svg').write_text('<svg/>',encoding='utf-8')
    out=tmp_path/'s.workcell.zip'
    subprocess.check_call(['python3','scripts/export_workcell_scene_bundle.py','--scene-dir',str(scene),'--output',str(out)])
    with zipfile.ZipFile(out,'r') as zf:
        assert 'environment.yaml' in zf.namelist()
        assert 'config/task_recipe.yaml' in zf.namelist()
        assert 'preview/workcell_preview.svg' in zf.namelist()
        mf=json.loads(zf.read('manifest.json').decode('utf-8'))
        assert mf['bundle_format']=='workcell_bundle/v1'

def test_import_rejects_unsafe(tmp_path):
    bad=tmp_path/'bad.zip'
    with zipfile.ZipFile(bad,'w') as zf:
        zf.writestr('../escape.txt','x')
        zf.writestr('manifest.json',json.dumps({'bundle_format':'workcell_bundle/v1'}))
    proc=subprocess.run(['python3','scripts/import_workcell_scene_bundle.py','--bundle',str(bad),'--target-scenes-dir',str(tmp_path/'out')],text=True,capture_output=True)
    assert proc.returncode!=0

def test_import_dry_run_and_overwrite_flags_present():
    txt=Path('scripts/import_workcell_scene_bundle.py').read_text(encoding='utf-8')
    assert '--dry-run' in txt and '--overwrite' in txt and '--print-summary' in txt

