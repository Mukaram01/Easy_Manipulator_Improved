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


def test_duplicate_delete_save_reload_does_not_resurrect_environment_or_manifest(tmp_path):
    scene = tmp_path / 'scene'
    (scene / 'layout').mkdir(parents=True)
    original = {'id': 'bin', 'type': 'target_bin', 'pose': {'xyz': [1, 2, 3]}}
    duplicate = {**original, 'id': 'bin_copy', 'layout_item_ref': 'bin_copy'}
    unrelated = {'id': 'camera', 'pose_xyz': [4, 5, 6], 'custom_metadata': 'preserve'}
    environment = {'environment': {'assets': [original, duplicate, unrelated]},
                   'assets': [original, duplicate, unrelated]}
    (scene / 'environment.yaml').write_text(yaml.safe_dump(environment))
    (scene / 'scene_manifest.yaml').write_text(yaml.safe_dump({'objects': [duplicate, unrelated]}))
    layout = scene / 'layout/workcell_studio_layout.yaml'
    layout.write_text(yaml.safe_dump({'items': [original, duplicate]}))
    subprocess.run([sys.executable, str(CLI), str(scene)], check=True)
    # Native delete filters the saved items and passes its existing tombstones.
    layout.write_text(yaml.safe_dump({'items': [original]}))
    subprocess.run([sys.executable, str(CLI), str(scene), '--deleted-item-id', 'bin_copy'], check=True)
    # Next Generate/reopen no longer has native tombstones.
    subprocess.run([sys.executable, str(CLI), str(scene)], check=True)
    for path in [layout, scene / 'environment.yaml', scene / 'scene_manifest.yaml',
                 scene / 'generated/workcell_studio_merged_environment.yaml',
                 scene / 'generated/workcell_studio_merged_scene_manifest.yaml']:
        assert 'bin_copy' not in path.read_text(), path
    saved = yaml.safe_load((scene / 'environment.yaml').read_text())
    assert saved['environment']['assets'] == [original, unrelated]
    assert saved['assets'] == [original, unrelated]


def test_native_save_passes_tombstones_before_clearing_them():
    source = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()
    projection = source.split('bool MainWindow::save_authored_environment_from_layout', 1)[1].split('void MainWindow::', 1)[0]
    assert 'deleted_layout_item_ids_' in projection
    assert '--deleted-item-id' in projection
    save = source.split('bool MainWindow::save_native_layout_changes', 1)[1]
    assert save.index('save_authored_environment_from_layout(error)') < save.index('deleted_layout_item_ids_.clear()')
