#!/usr/bin/env python3
"""Read-only Qt authoring adapter for the existing task-intent v2 contract."""
import argparse
import json
from pathlib import Path
import sys

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
from task_intent_v2 import migrate_v1, parse_validate_normalize, normalized_intent_hash, TOOL_CAPABILITY_MAP
from validate_builder_task_intent import validate_payload


def _wizard_intent(scene, env):
    """Materialize the existing Create New Cell selections, without editing them."""
    studio = env.get('workcell_studio', {})
    if studio.get('scenario', {}).get('id') != 'static_table_pick_place':
        raise ValueError('Configure Pick & Place in the existing Create New Cell workflow first.')
    pick = studio.get('pick_zone', {})
    source = pick.get('object_source', {})
    task = studio.get('task_intent', {})
    authored_strategy = task.get('grasp_strategy', 'auto')
    strategy = 'top_2f' if authored_strategy == 'auto' else authored_strategy
    strategy = {'finger_top': 'top_2f', 'top_grasp_2f': 'top_2f', 'top_down_2f': 'top_2f'}.get(strategy, strategy)
    catalog_path = SCRIPT_DIR.parent / 'catalog/grasp_strategies' / (strategy + '.yaml')
    if not catalog_path.is_file():
        raise ValueError(f'Unknown wizard grasp strategy {strategy!r}; select a supported 2F strategy.')
    catalog = yaml.safe_load(catalog_path.read_text())['grasp_strategy']
    target = studio.get('place_zone', {}).get('target', '')
    regions = [z['id'] for z in env.get('task_zones', []) if z.get('target_ref') == target]
    pick_zones = [z['id'] for z in env.get('task_zones', []) if z.get('type') == 'pick_zone']
    return {
        'schema': 'workcell_builder_task_intent/v2', 'scene_package': str(scene),
        'task': {'id': scene.name + '_pick_place', 'type': 'pick_place', 'template': 'pick_place'},
        'pick': {
            'selection': {'source_ref': source.get('source_zone_ref', pick.get('pick_object_source', '')),
                          'source_type': source.get('mode', 'manual_simulated'),
                          'zone_ref': pick_zones[0] if len(pick_zones) == 1 else pick.get('pick_object_source', ''),
                          'object_filter': {'class_id': source.get('required_class', ''), 'color': None,
                                            'min_confidence': source.get('minimum_confidence'), 'max_age_seconds': 2.0}},
            'grasp': {'policy': 'AUTO' if authored_strategy == 'auto' else 'EXACT', 'required_capability': 'two_finger_parallel', 'strategy_ref': None if authored_strategy == 'auto' else strategy,
                      'approach': {'axis': task.get('approach_axis', catalog['approach_axis']),
                                   'distance_m': task.get('approach_distance_m', catalog['approach_distance_m'])},
                      'orientation': {'mode': catalog['orientation_mode'],
                                      'allowed_roll_deg': catalog.get('allowed_roll_angles_deg', [0]),
                                      'allowed_yaw_deg': catalog.get('allowed_yaw_angles_deg', [0]),
                                      'tolerance_rad': [0.0, 0.0, 0.0]},
                      'tcp_offset_xyz_m': catalog.get('tool_frame_offset_xyz', [0.0, 0.0, 0.0]),
                      'tcp_offset_rpy_rad': catalog.get('tool_frame_offset_rpy', [0.0, 0.0, 0.0]),
                      'contact': {'required': True, 'min_quality': 0.0},
                      'aperture': {'min_m': 0.0, 'max_m': 0.085},
                      'lift': {'axis': 'z_up', 'distance_m': task.get('retreat_distance_m', catalog['retreat_distance_m'])}}},
        'place': {'target': {'asset_ref': target, 'region_ref': studio.get('place_zone', {}).get('region') or (regions[0] if len(regions) == 1 else target)},
                  'placement': {'policy': 'AUTO', 'requested_local_pose': None,
                                'orientation': {'mode': 'target_default', 'rpy_rad': [0.0, 0.0, 0.0], 'tolerance_rad': [0.0, 0.0, 0.0]},
                                'approach': {'axis': 'z_down', 'distance_m': 0.1}, 'clearance_m': 0.01,
                                'retreat': {'axis': 'z_up', 'distance_m': 0.1}},
                  'release': {'strategy': 'tool_release'}},
        'safety': {'execution_mode': 'simulation_preview', 'require_fake_hardware': True,
                   'real_hardware_enabled': False, 'preview_policy': 'diagnostic_if_unresolved'},
    }


def load_authoring(scene, draft=None):
    scene = Path(scene)
    scene_doc = yaml.safe_load((scene / 'environment.yaml').read_text()) or {}
    env = scene_doc.get('environment', scene_doc)
    path = scene / 'config/workcell_builder_task_intent.yaml'
    if draft is not None:
        payload = draft
    elif path.is_file():
        payload = yaml.safe_load(path.read_text())
        if payload.get('schema') == 'workcell_builder_task_intent/v1':
            payload = migrate_v1(payload, env)
    else:
        payload = _wizard_intent(scene, {**env, 'workcell_studio': scene_doc.get('workcell_studio', {})})
    parsed = parse_validate_normalize(payload)
    report = validate_payload(payload, scene)
    # A physical blocker does not prevent saving the authored request unchanged.
    if parsed['normalized'] is not None:
        report['task_intent'] = parsed['normalized']
        report['normalized_intent_sha256'] = normalized_intent_hash(payload)
    tool = env.get('end_effector') or env.get('tool') or scene_doc.get('end_effector') or scene_doc.get('tool') or {}
    tool_id = (tool.get('id') or tool.get('name')) if isinstance(tool, dict) else tool
    report['strategies'] = (['top_2f', 'side_grip_basic', 'finger_pinch_basic']
                            if 'two_finger_parallel' in TOOL_CAPABILITY_MAP.get(tool_id, set()) else [])
    report['assets'] = [a['id'] for a in env.get('assets', [])]
    report['regions'] = {z['id']: z.get('target_ref') for z in env.get('task_zones', []) if z.get('target_ref')}
    report['pick_zones'] = [z['id'] for z in env.get('task_zones', []) if z.get('type') == 'pick_zone']
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('scene', type=Path)
    parser.add_argument('--stdin', action='store_true', help='Validate an unsaved v2 draft from stdin')
    args = parser.parse_args()
    try:
        print(json.dumps(load_authoring(args.scene, json.load(sys.stdin) if args.stdin else None)))
    except (ValueError, KeyError, TypeError, OSError) as exc:
        print(json.dumps({'status': 'FAIL', 'errors': [str(exc)]}))
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
