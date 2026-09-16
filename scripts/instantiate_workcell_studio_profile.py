#!/usr/bin/env python3
"""Materialize New Cell selections using reviewed physical defaults.

This is the wizard's adapter to the existing authored scene and package generator,
not a scene clone: only equipment/physical sections and the equipment xacro are
reused. Identity, TaskIntent, manifests and caches belong to the new cell.
"""
import argparse
import copy
import json
from pathlib import Path
import sys
import shutil
import tempfile
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
XACRO = '{http://www.ros.org/wiki/xacro}'


def load_profile(profile_id):
    if Path(profile_id).name != profile_id:
        raise ValueError('Invalid recommended profile ID')
    profile = yaml.safe_load((ROOT / 'catalog/workcell_studio_profiles' / (profile_id + '.yaml')).read_text())
    source = yaml.safe_load((ROOT / profile['environment_source']).read_text())
    from physical_destination import resolve_destination
    resolve_destination(source['environment'], profile['place_region'])
    return profile, source


def describe(profile_id):
    profile, source = load_profile(profile_id)
    physical = source['environment']
    rows = []
    for key in ('support_surfaces', 'assets', 'task_zones'):
        for item in physical.get(key, []):
            if key == 'task_zones' and item['id'] != profile['pick_zone']:
                continue
            role = ('pick_source' if item['id'] == profile['pick_zone'] else
                    'place_target' if item['id'] == profile['place_asset'] else item.get('role', ''))
            rows.append(dict(id=item['id'], asset_type=item['type'], role=role,
                             pose=dict(xyz=item['pose_xyz'], rpy=item['pose_rpy'])))
    return dict(profile, rows=rows, robot=source['robot'], tool=source['tool'])


def materialize(scene, profile_id, destination):
    from authored_yaml import write_preserving
    from task_intent_authoring import load_authoring
    from export_builder_scene_to_cell_definition import export_scene
    import generate_workcell_from_cell_definition as generator
    from ensure_workcell_studio_web_scene_fresh_impl import require_real_xacro_mesh_index
    import subprocess

    if destination.exists():
        raise ValueError('New Cell destination already exists; refusing to replace it')
    profile, defaults = load_profile(profile_id)
    authored = yaml.safe_load((scene / 'environment.yaml').read_text())
    studio = authored['workcell_studio']
    studio['recommended_profile'] = profile_id
    if (studio['robot']['model'] != profile['robot_model'] or
            studio['tool']['model'] != profile['tool_model']):
        raise ValueError('Recommended physical profile does not match selected robot/tool')
    expected = {row['id']: row for row in describe(profile_id)['rows']}
    selected = {row['id']: row for row in studio['environment_objects']}
    if selected.keys() != expected.keys():
        raise ValueError('Recommended profile assets changed; reselect Use Recommended Layout')
    # Do not silently ignore custom attachment/pose edits or invent their physical meaning.
    for name, row in selected.items():
        supported_fields = {'asset_type': expected[name]['asset_type'],
                            'semantic_role': expected[name]['role'], 'parent_object': 'world',
                            'parent_link': 'world', 'child_link': name, 'joint_type': 'fixed'}
        if any(row.get(key) != value for key, value in supported_fields.items()) or any(
                len(row['pose'][axis]) != 3 for axis in ('xyz', 'rpy')) or any(
                abs(a-b) > 1e-6 for axis in ('xyz', 'rpy')
                for a, b in zip(row['pose'][axis], expected[name]['pose'][axis])):
            raise ValueError(f'Recommended asset {name} differs from its reviewed pose/asset binding; edit it in Studio after creation')
    for key in ('robot', 'tool', 'end_effector', 'environment', 'camera', 'safety'):
        authored[key] = copy.deepcopy(defaults[key])
    authored['schema_version'] = 'workcell_scene/v1'
    authored['scene'] = {'id': scene.name, 'name': scene.name}
    authored['scaffold_only'] = False
    robot, tool = authored['robot'], authored['tool']
    frames = studio['frames']
    robot['pose_xyz'], robot['pose_rpy'] = frames['robot_mount_pose']['xyz'], frames['robot_mount_pose']['rpy']
    robot['base_frame'] = studio['robot']['base']
    robot['tool_mount_link'] = studio['robot']['tip']
    robot['planning_group'] = studio['robot']['planning_group']
    tool['mount_link'] = studio['robot']['tip']
    tool['grasp_frame'] = studio['tool']['tcp']
    tool['mount_pose_xyz'], tool['mount_pose_rpy'] = frames['tool_mount_pose']['xyz'], frames['tool_mount_pose']['rpy']
    authored['end_effector'] = copy.deepcopy(tool)
    camera = authored['camera']
    physical_camera = next(item for item in authored['environment']['assets'] if item['id'] == camera['camera_id'])
    camera['pose'] = physical_camera['pose_xyz'] + physical_camera['pose_rpy']
    source = studio['pick_zone']['object_source']
    if source['mode'] != 'manual_simulated':
        # Consume the existing EPD contract; its implementation stays in EPD.
        perception = copy.deepcopy(defaults['perception'])
        perception['mode'] = 'replay' if source['mode'] == 'recorded_perception' else 'live'
        perception['epd_input']['topic'] = source['perception_binding']
        perception['required_object_classes'] = [source['required_class']] if source['required_class'] else []
        perception['confidence_threshold'] = source.get('minimum_confidence')
        authored['perception'] = perception
    # TaskIntent is authored from this wizard, never from the profile source task.
    studio['place_zone']['region'] = profile['place_region']
    if studio['place_zone']['target'] != profile['place_asset']:
        raise ValueError('Select the recommended physical destination bin')
    if studio['pick_zone']['object_source']['source_zone_ref'] != profile['pick_zone']:
        raise ValueError('Select the recommended pick region')
    write_preserving(scene / 'environment.yaml', authored)
    intent = load_authoring(scene)['task_intent']
    intent['scene_package'] = str(destination)
    (scene / 'config/workcell_builder_task_intent.yaml').write_text(json.dumps(intent, indent=2) + '\n')

    # Project the authoritative physical objects into the existing editable layout.
    items = []
    for key in ('support_surfaces', 'assets', 'task_zones'):
        for raw in authored['environment'].get(key, []):
            item = copy.deepcopy(raw)
            item['id'] = item.get('layout_item_ref', item['id'])
            if any(previous['id'] == item['id'] for previous in items):
                continue
            item['pose'] = {'xyz': item.pop('pose_xyz'), 'rpy': item.pop('pose_rpy')}
            item.update(editable=True, locked=False, source='environment.yaml', source_layer='editable_layout')
            items.append(item)
    (scene / 'layout').mkdir(exist_ok=True)
    write_preserving(scene / 'layout/workcell_studio_layout.yaml', {
        'schema_version': 'workcell_studio_layout/v1', 'scene_name': scene.name,
        'scene_path': '.', 'metadata': {'physical_state_source': 'environment.yaml'}, 'items': items})

    # Reuse the reviewed equipment xacro as a profile template. All scene-dependent
    # arguments read this cell's authored values; there is no reference-scene lookup.
    ET.register_namespace('xacro', XACRO[1:-1])
    model = ET.fromstring((ROOT / profile['model_source']).read_text())
    model.set('name', scene.name)
    for arg in model.findall(XACRO + 'arg'):
        if arg.get('name') == 'environment_file':
            arg.set('default', '../environment.yaml')
        elif arg.get('name') == 'tool_mount_link':
            arg.set('default', tool['mount_link'])
        elif arg.get('name') == 'grasp_frame':
            arg.set('default', tool['grasp_frame'])
    origins = profile['model_origins']
    # Profile metadata identifies the existing macro fields, not a second URDF renderer.
    for macro, path in origins.items():
        model_origins = model.findall(XACRO + macro + '/origin')
        if len(model_origins) != 1:
            raise ValueError(f'Reviewed model must define exactly one {macro} origin')
        for origin in model_origins:
            for axis, selection in (('xyz', '[:3]'), ('rpy', '[3:]')):
                expression = path.format(axis=axis, slice=selection)
                origin.set(axis, "${' '.join([str(v) for v in workcell_environment" + expression + "])}")
    (scene / 'urdf').mkdir(exist_ok=True)
    (scene / 'urdf/scene.urdf.xacro').write_bytes(ET.tostring(model, encoding='utf-8', xml_declaration=True))
    # Full initial publication uses the same generator as later in-place refresh.
    # Its initial model is this cell's authored equipment definition, never a
    # reference scene's generated package. In-place refresh cannot create missing
    # curated package files, so it is deliberately not used for initialization.
    export_scene(scene, scene, validate=True)
    with tempfile.TemporaryDirectory(prefix='.initial-package-', dir=scene.parent) as output:
        result = generator.generate_package(scene / 'cell_definition.yaml', Path(output),
                                            scene.name, False, False, authored_model_dir=scene, published_package_dir=destination)
        if result:
            raise ValueError(f'Initial scene package generation failed ({result})')
        shutil.copytree(Path(output) / scene.name, scene, dirs_exist_ok=True)
    result = subprocess.run([sys.executable, str(ROOT / 'scripts/extract_scene_urdf_visual_mesh_index.py'),
                             '--scene', str(scene), '--prefer-xacro', '--require-xacro'], capture_output=True, text=True)
    if result.returncode:
        raise ValueError(result.stderr or result.stdout or 'Required visual extraction failed')
    require_real_xacro_mesh_index(scene / 'generated/scene_visual_mesh_index.json')
    (scene / 'config/workcell_builder_task_intent.yaml').write_text(json.dumps(intent, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--describe', action='store_true')
    parser.add_argument('--scene', type=Path)
    parser.add_argument('--destination', type=Path)
    args = parser.parse_args()
    try:
        if args.describe:
            print(json.dumps(describe(args.profile)))
        else:
            materialize(args.scene, args.profile, args.destination)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
