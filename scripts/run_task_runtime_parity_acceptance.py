#!/usr/bin/env python3
"""Exercise real task consumers on retained New Cell/Qt acceptance scenes.

Run after the existing Qt New Cell acceptance and a Humble build/install of
those scene packages. This runner never sends execution goals. Use disposable
acceptance scenes: destination edit/restore intentionally exercises authored Save.
"""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import sys

import yaml

from export_builder_scene_to_cell_definition import export_scene
from generate_workcell_from_cell_definition import generate_package
from validate_builder_generated_scene import validate_scene
from export_workcell_studio_web_scene_impl import build_web_scene
from task_intent_resolver import read_scene_task, scene_resolution
from workcell_studio_layout_merge import merge


def identity(result):
    resolution = result.get('task_intent_resolution', result)
    destination = resolution['place_resolution']['destination']
    return {'normalized_intent_sha256': resolution['normalized_intent_sha256'],
            'resolution_sha256': resolution['resolution_sha256'],
            'destination': destination}


def generate(scene):
    export_scene(scene, scene, validate=True)
    handoff = yaml.safe_load((scene / 'task_recipe_from_builder_intent.yaml').read_text())
    if handoff.get('enabled') is False:
        assert not (scene / 'offline_plan_preview_request.yaml').exists()
    if generate_package(scene / 'cell_definition.yaml', scene.parent, scene.name,
                        False, False, existing_package_dir=scene):
        raise RuntimeError('Generation failed')


def runtime(scene, output, domain, resolve=False, blocked=False):
    command = [sys.executable, str(Path(__file__).with_name('run_r14_plan_only_acceptance.py')),
               '--scene-dir', str(scene), '--output-dir', str(output),
               '--timeout', '240', '--domain-id', str(next(domain))]
    if resolve:
        command.append('--resolve-task')
    with output.with_suffix('.log').open('w') as log:
        completed = subprocess.run(command, check=False, stdout=log, stderr=subprocess.STDOUT)
    result = json.loads((output / 'acceptance.json').read_text())
    assert completed.returncode == (1 if blocked else 0), result.get('failure', result)
    assert result['result'] == ('FAIL' if blocked else 'PASS')
    assert result['execution_action_goals'] == []
    assert result['execution_attempted'] is False
    assert result['shutdown_clean'] is True
    return result


def stage_chain(scene, output, domain):
    output.mkdir(parents=True)
    generate(scene)  # Physical bootstrap/current authored handoff, not claimed READY.
    resolved = runtime(scene, output / 'resolve', domain, resolve=True)
    generate(scene)  # Consume actual MoveIt-backed resolution.
    cell = yaml.safe_load((scene / 'cell_definition.yaml').read_text())
    recipe = yaml.safe_load((scene / 'config/task_recipe.yaml').read_text())
    compatibility_recipe = yaml.safe_load((scene / 'task_recipe_from_builder_intent.yaml').read_text())
    offline_preview = yaml.safe_load((scene / 'offline_plan_preview_request.yaml').read_text())
    validation = validate_scene(scene)
    assert validation['ok'], validation['errors']
    from validate_task_recipe import validate_path
    assert validate_path(scene / 'config/task_recipe.yaml').ok
    assert identity(validation['task_flow_summary']) == identity(resolved)
    web = build_web_scene(scene)
    planned = runtime(scene, output / 'plan', domain)
    stages = {name: identity(value) for name, value in (
        ('resolver', resolved), ('generate', cell), ('recipe', recipe),
        ('compatibility_recipe', compatibility_recipe), ('offline_preview', offline_preview),
        ('validate', validation), ('product_view', web), ('plan_simulate', planned))}
    assert all(value == stages['resolver'] for value in stages.values()), stages
    intent, physical, document = read_scene_task(scene)
    assert identity(scene_resolution(scene, intent, physical, document, require_ready=True)) == stages['resolver']
    result = {'status': 'PASS', 'stages': stages,
              'fake_hardware': planned['fake_hardware_guard'],
              'execution_action_goals': planned['execution_action_goals'],
              'shutdown_clean': planned['shutdown_clean']}
    (output / 'parity.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, default=191)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('choose a new output directory')
    args.output.mkdir(parents=True)
    evidence = {}
    domains = iter(range(args.domain_id, 233))  # Fresh DDS domain per owned launch.
    for scene in args.scene:
        scene = scene.resolve()
        baseline = stage_chain(scene, args.output / scene.name / 'baseline', domains)
        layout_path = scene / 'layout/workcell_studio_layout.yaml'
        environment_path = scene / 'environment.yaml'
        original_layout, original_environment = layout_path.read_bytes(), environment_path.read_bytes()
        destination = baseline['stages']['resolver']['destination']
        try:
            layout = yaml.safe_load(original_layout)
            environment = yaml.safe_load(original_environment)['environment']
            zone = next(z for z in environment['task_zones'] if z['id'] == destination['id'])
            item = next(i for i in layout['items'] if i['id'] == zone.get('layout_item_ref', zone['id']))
            item['pose']['xyz'][0] += .01
            layout_path.write_text(yaml.safe_dump(layout, sort_keys=False))
            merge(scene, save_authored=True)
            edited = stage_chain(scene, args.output / scene.name / 'edited', domains)
            assert edited['stages']['resolver']['resolution_sha256'] != baseline['stages']['resolver']['resolution_sha256']
            assert edited['stages']['resolver']['destination']['pose_xyz'][0] == destination['pose_xyz'][0] + .01
        finally:
            layout_path.write_bytes(original_layout)
            environment_path.write_bytes(original_environment)
        restored = stage_chain(scene, args.output / scene.name / 'restored', domains)
        assert restored['stages'] == baseline['stages']
        task_path = scene / 'config/workcell_builder_task_intent.yaml'
        original_task = task_path.read_bytes()
        try:
            # Existing v2 policy fields, persisted as normal authored input.
            preferred_intent = yaml.safe_load(original_task)
            preferred_intent['pick']['grasp'].update(policy='PREFERRED', strategy_ref='side_grip_basic')
            task_path.write_text(yaml.safe_dump(preferred_intent, sort_keys=False))
            preferred = stage_chain(scene, args.output / scene.name / 'preferred', domains)
            resolution = scene_resolution(scene, *read_scene_task(scene))
            assert resolution['readiness_status'] == 'WARNING'
            assert resolution['grasp_resolution']['fallback']['used'] is True
            exact_intent = copy.deepcopy(preferred_intent)
            exact_intent['pick']['grasp']['policy'] = 'EXACT'
            task_path.write_text(yaml.safe_dump(exact_intent, sort_keys=False))
            generate(scene)
            exact = runtime(scene, args.output / scene.name / 'exact', domains, resolve=True, blocked=True)
            resolution = exact['task_intent_resolution']
            assert resolution['readiness_status'] == 'BLOCKED'
            assert resolution['grasp_resolution']['fallback']['used'] is False
            assert resolution['place_resolution']['fallback']['used'] is False
            assert len(resolution['grasp_resolution']['attempts']) == 1
        finally:
            task_path.write_bytes(original_task)
        final = stage_chain(scene, args.output / scene.name / 'final', domains)
        assert final['stages'] == baseline['stages']
        evidence[scene.name] = {'baseline': baseline, 'edited': edited, 'restored': restored,
            'preferred': preferred, 'exact': {'status': 'BLOCKED', 'resolution': resolution,
                'execution_action_goals': exact['execution_action_goals'], 'shutdown_clean': exact['shutdown_clean']},
            'final': final}
        (args.output / 'acceptance.json').write_text(json.dumps(evidence, indent=2) + '\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
