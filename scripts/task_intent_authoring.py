#!/usr/bin/env python3
"""Read-only Qt authoring adapter for the existing task-intent v2 contract."""
import argparse
import json
from pathlib import Path
import sys
import time

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
            'selection': {'source_ref': ('detected_objects/v1' if source.get('mode') in ('recorded_perception', 'live_epd_realsense') else source.get('source_zone_ref', pick.get('pick_object_source', ''))),
                          'source_type': ('perception' if source.get('mode') in ('recorded_perception', 'live_epd_realsense') else 'manual_simulated'),
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
        'perception': {
            'mode': {'recorded_perception': 'replayed_snapshot', 'live_epd_realsense': 'live_epd'}.get(source.get('mode'), 'off'),
            'replay_path': '',
            **({'camera_source': 'realsense_d435i'} if source.get('mode') == 'live_epd_realsense' else {}),
        },
        'safety': {'execution_mode': 'simulation_preview', 'require_fake_hardware': True,
                   'real_hardware_enabled': False, 'preview_policy': 'diagnostic_if_unresolved'},
    }


def planning_input(scene, intent, override=None):
    """Consume the saved input binding; never substitute observations for new tasks.

    Existing tasks without an explicit perception binding retain their reviewed
    commissioning replay. Live snapshots are never stamped as fresh replay data.
    """
    binding = intent.get('perception')
    if binding is not None and not isinstance(binding, dict):
        raise ValueError('TASK_INPUT_INVALID: perception binding must be a mapping.')
    if binding is None or 'mode' not in binding:
        return Path(override) if override else SCRIPT_DIR.parent / 'config/runtime/r1_4_replay.yaml', True
    mode = binding['mode']
    if mode not in ('off', 'replayed_snapshot', 'live_epd'):
        raise ValueError('PERCEPTION_MODE_UNSUPPORTED: choose off, replay or live.')
    name = binding.get('replay_path')
    if not isinstance(name, str) or not name.strip():
        raise ValueError('TASK_INPUT_UNAVAILABLE: choose a commissioning replay or normalized live snapshot in Task settings.')
    path = Path(name).expanduser()
    if not path.is_absolute():
        path = Path(scene) / path
    if override is not None and Path(override).resolve() != path.resolve():
        raise ValueError('TASK_INPUT_OVERRIDE_REJECTED: use the saved Task input; edit it in Builder to change recordings.')
    if not path.is_file():
        raise ValueError(f'TASK_INPUT_UNAVAILABLE: snapshot {path} is missing; select an existing input in Task settings.')
    snapshot = yaml.safe_load(path.read_text())
    if not isinstance(snapshot, dict) or snapshot.get('schema_version') != 'detected_objects/v1' or not snapshot.get('objects'):
        raise ValueError('TASK_INPUT_INVALID: input must contain detected_objects/v1 observations with measured geometry.')
    if not isinstance(snapshot['objects'], list) or any(not isinstance(obj, dict) for obj in snapshot['objects']):
        raise ValueError('TASK_INPUT_INVALID: observations must be a list of object mappings.')
    if not isinstance(snapshot.get('source'), dict):
        raise ValueError('TASK_INPUT_INVALID: snapshot source must be a mode mapping.')
    replay = mode != 'live_epd'
    if not replay and snapshot.get('source', {}).get('mode') == 'replayed_snapshot':
        raise ValueError('LIVE_INPUT_REQUIRED: live mode needs current normalized observations, not a replay template.')
    if replay and snapshot.get('source', {}).get('mode') != 'replayed_snapshot':
        raise ValueError('REPLAY_INPUT_REQUIRED: select a snapshot declaring source.mode=replayed_snapshot.')
    if not replay:
        chosen_camera = binding.get('camera_source')
        if chosen_camera not in ('simulated_rgbd', 'realsense_d435i'):
            raise ValueError('CAMERA_SOURCE_REQUIRED: choose Simulated RGB-D or Intel RealSense D435i in Task settings.')
        reported_camera = snapshot['source'].get('camera_source')
        if reported_camera != chosen_camera:
            raise ValueError(f'CAMERA_SOURCE_MISMATCH: selected {chosen_camera}, but the normalized live observation declares {reported_camera!r}; verify the EPD adapter and camera connection.')
    # Reuse the runtime normalizer and shared selection/zone authority. This is
    # input eligibility, never a replacement for collision-aware cycle planning.
    from runtime_pick_inputs import normalize, replay_snapshot
    import perceived_object_grasp_plan as geometry
    from task_intent_resolver import select_observations
    now = time.time()
    observations = normalize(replay_snapshot(snapshot, now) if replay else snapshot, now, geometry)
    document = yaml.safe_load((Path(scene) / 'environment.yaml').read_text())
    physical = document.get('environment', document)
    if not select_observations(intent, physical, observations, now):
        raise ValueError('OBSERVATION_UNAVAILABLE: no current workpiece matches the class, confidence, age and pick-zone constraints.')
    return path, replay


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
    try:
        input_path, replay = planning_input(scene, payload)
        report['planning_input'] = {'path': str(input_path), 'replay': replay}
    except (ValueError, OSError, yaml.YAMLError) as exc:
        report['planning_input_blocker'] = str(exc)
    report['pick_zones'] = [z['id'] for z in env.get('task_zones', []) if z.get('type') == 'pick_zone']
    intent = parsed['normalized']
    if intent is not None and intent.get('sequence') is not None:
        # The same review the queue preview and the full-sequence check apply to this (draft) task.
        from simulation_batch_preview import review_sequence
        try:
            report['sequence_review'] = review_sequence(scene, intent)
        except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as exc:  # never hide the task itself
            report['sequence_review'] = {'status': 'BLOCKED', 'items': [], 'order': [], 'placements': {},
                                         'summary': f'BLOCKED: the saved sequence cannot be checked: {exc}'}
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
