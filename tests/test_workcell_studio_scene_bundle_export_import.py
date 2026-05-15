from pathlib import Path
import json, subprocess, zipfile

ROOT=Path(__file__).resolve().parents[1]


def test_ui_bundle_button_labels_and_handlers_present():
    cpp=(ROOT/'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    hpp=(ROOT/'workcell_builder/workcell_builder/gui/mainwindow.h').read_text(encoding='utf-8')
    for marker in ['Export Scene Bundle','Import Scene Bundle','Open Export Folder']:
        assert marker in cpp
    for fn in ['export_scene_bundle_for_selected_scene','import_scene_bundle_into_scenes_root','open_scene_bundle_export_folder']:
        assert fn in cpp and fn in hpp


def test_export_manifest_safety_and_expected_files(tmp_path: Path):
    scene=tmp_path/'demo_scene'; (scene/'config').mkdir(parents=True); (scene/'preview').mkdir(); (scene/'validation').mkdir()
    (scene/'environment.yaml').write_text('name: demo\n',encoding='utf-8')
    (scene/'scene_manifest.yaml').write_text('scene: demo\n',encoding='utf-8')
    (scene/'task_recipe.yaml').write_text('task: pick_place\n',encoding='utf-8')
    (scene/'config/workcell_builder_task_intent.yaml').write_text('preview_only: true\n',encoding='utf-8')
    (scene/'preview/static_preview.svg').write_text('<svg/>',encoding='utf-8')
    out=tmp_path/'demo_workcell_studio_bundle.zip'
    subprocess.check_call(['python3',str(ROOT/'scripts/export_workcell_scene_bundle.py'),'--scene-dir',str(scene),'--output',str(out)])
    with zipfile.ZipFile(out,'r') as zf:
        mf=json.loads(zf.read('manifest.json').decode('utf-8'))
        assert mf['preview_only'] is True and mf['use_fake_hardware_default'] is True and mf['no_robot_motion'] is True
        names=set(zf.namelist())
        assert 'environment.yaml' in names and 'scene_manifest.yaml' in names and 'task_recipe.yaml' in names
        assert 'config/workcell_builder_task_intent.yaml' in names


def test_import_avoids_destructive_overwrite(tmp_path: Path):
    scenes=tmp_path/'scenes'; scenes.mkdir()
    (scenes/'my_scene').mkdir()
    bundle=tmp_path/'bundle'; bundle.mkdir()
    (bundle/'manifest.json').write_text(json.dumps({'bundle_format':'workcell_studio_scene_bundle/v1','source_scene_name':'my_scene'}),encoding='utf-8')
    (bundle/'environment.yaml').write_text('name: x\n',encoding='utf-8')
    subprocess.check_call(['python3',str(ROOT/'scripts/import_workcell_scene_bundle.py'),'--bundle',str(bundle),'--target-scenes-dir',str(scenes)])
    assert (scenes/'my_scene').exists()
    assert (scenes/'my_scene_imported').exists()


def test_no_runtime_motion_command_added():
    cpp=(ROOT/'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    assert 'run_grasp_execution' not in cpp
