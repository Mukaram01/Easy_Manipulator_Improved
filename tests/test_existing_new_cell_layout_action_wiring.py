from pathlib import Path
import json
import shutil
import subprocess
import yaml

MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_new_cell_actions_are_wired_to_runtime_file_outputs():
    for token in [
        'Use Recommended Layout: wrote',
        'workcell_studio_layout.yaml',
        'existing_new_cell_flow',
        'task/workcell_builder_task_intent.yaml',
        'task/task_recipe_from_builder_intent.yaml',
        'offline_plan_preview_request.yaml',
    ]:
        assert token in MAIN


def test_save_layout_contract_tokens_present():
    for token in [
        'schema_version"] = "workcell_studio_layout/v1"',
        'schema"] = "workcell_studio_layout/v1"',
        'fake_hardware_first',
        'task_zones',
    ]:
        assert token in MAIN


def test_task_intent_script_writes_zone_bound_intent(tmp_path: Path):
    scene = tmp_path / 'scene_a'
    (scene / 'generated').mkdir(parents=True)
    (scene / 'environment.yaml').write_text(yaml.safe_dump({
        'scene_name': 'scene_a',
        'task_zones': [
            {'id': 'pick_zone', 'type': 'pick'},
            {'id': 'place_zone', 'type': 'place'},
        ],
    }, sort_keys=False), encoding='utf-8')
    out = scene / 'generated' / 'workcell_builder_task_intent.yaml'
    cmd = [
        'python3', 'scripts/create_or_update_builder_task_intent.py',
        '--scene-package', str(scene),
        '--task-id', 'scene_a_pick_place',
        '--task-type', 'pick_place',
        '--task-template', 'pick_place',
        '--pick-source', 'pick_zone',
        '--place-target', 'place_zone',
        '--grasp-strategy', 'top_grasp_2f',
        '--output', str(out),
        '--json',
    ]
    subprocess.run(cmd, check=True)
    payload = yaml.safe_load(out.read_text(encoding='utf-8'))
    assert payload['task']['pick']['source']['id'] == 'pick_zone'
    assert payload['task']['place']['target']['id'] == 'place_zone'
    text = out.read_text(encoding='utf-8').lower()
    for banned in ['use_fake_hardware:=false', 'fake_hardware:=false', 'ur_robot_driver', 'ethercat', 'canopen']:
        assert banned not in text


def test_new_cell_camera_metadata_canonical_and_legacy_traceability_tokens_present():
    txt = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    for token in [
        'Detection mode: %6',
        'mapped from legacy: %2',
        'perception_legacy_source',
        'snapshot_overlay',
        'epd_optional',
        'none (camera metadata only)',
    ]:
        assert token in txt


def test_use_recommended_layout_fixture_populates_scene3d_data_immediately(tmp_path: Path):
    fixture = Path('tests/fixtures/new_cell_recommended_layout')
    scene = tmp_path / 'new_cell_recommended_layout'
    shutil.copytree(fixture, scene)

    subprocess.run([
        'python3',
        'scripts/generate_workcell_studio_scene_artifacts.py',
        '--root', str(tmp_path),
        '--scene', scene.name,
        '--overwrite',
    ], check=True)

    layout_preview = yaml.safe_load((scene / 'layout' / 'workcell_studio_layout.generated.yaml').read_text(encoding='utf-8'))
    preview_items = layout_preview.get('preview_items', [])
    assert len(preview_items) > 0

    mesh_index = json.loads((scene / 'generated' / 'scene_visual_mesh_index.json').read_text(encoding='utf-8'))
    items = mesh_index.get('items', [])
    by_id = {str(i.get('id')): i for i in items if isinstance(i, dict)}
    for required in ['robot_base', 'workbench', 'pick_zone', 'place_zone']:
        assert required in by_id

    for required in ['robot_base', 'workbench', 'pick_zone', 'place_zone']:
        item = by_id[required]
        assert item.get('id')
        assert item.get('source_layer')
        assert item.get('active_visual_source')
        assert item.get('editable') is (not item.get('locked'))

    assert int(mesh_index.get('visible_after_filters', 0)) > 0
