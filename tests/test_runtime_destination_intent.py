"""Canonical authored task -> place intent, without ROS or motion."""
import copy
import itertools
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import yaml

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import runtime_pick_inputs as inputs
import perceived_object_grasp_execute as executor

PACKAGE = ROOT / 'scenes/ur5_2f_test'


def cell():
    return yaml.safe_load((PACKAGE / 'cell_definition.yaml').read_text())


def test_canonical_task_place_intent_order_independent():
    config = cell()
    task = inputs.task_request(yaml.safe_load((ROOT / 'config/runtime/r1_4_task.yaml').read_text()), config)
    snapshot = inputs.replay_snapshot(yaml.safe_load((ROOT / 'config/runtime/r1_4_replay.yaml').read_text()), 100.)
    authored = next(z for z in config['environment']['task_zones'] if z['id'] == task['destination_zone'])
    environment = yaml.safe_load((PACKAGE / 'environment.yaml').read_text())
    # The generated runtime handoff preserves the authored zone geometry.
    def find_zone(node):
        if isinstance(node, dict):
            if node.get('id') == authored['id'] and node.get('type') == 'place_zone':
                return node
            for value in node.values():
                found = find_zone(value)
                if found is not None: return found
        if isinstance(node, list):
            for value in node:
                found = find_zone(value)
                if found is not None: return found
    source = find_zone(environment)
    assert source is not None
    for key in ('pose_xyz', 'pose_rpy', 'dimensions'):
        assert authored[key] == source[key]
    destinations = []
    for permutation in itertools.permutations(snapshot['objects']):
        current = dict(snapshot, objects=list(permutation))
        objects = inputs.normalize(current, 100., executor._PLANNER)
        selected, _ = inputs.filter_targets(objects, task, config, 100., executor._PLANNER)
        target = selected[0]
        assert target['class_id'] == 'cup'
        destination = executor.load_canonical_place_target(PACKAGE, task['destination_zone'])
        assert destination['id'] == 'default_drop_zone' != task['source_zone']
        assert destination['pose_xyz'] == authored['pose_xyz']
        assert destination['dimensions'] == authored['dimensions']
        assert destination['pose_rpy'] == authored['pose_rpy']
        geometry = executor.eligible_grasp_target(target, destination)
        assert len(executor.generate_box_grasp_candidates(geometry)) == 8
        # A held tool/object offset is retained by the production transfer/place helper.
        tool = NS(pose=NS(position=NS(x=target['pose'][0], y=target['pose'][1],
                                     z=target['pose'][2] + .1),
                          orientation=NS(x=1., y=0., z=0., w=0.)))
        original = copy.deepcopy(tool)
        contract = executor._PLANNER.load_grasp_contract(PACKAGE)
        transfer, place = executor.place_motion_targets(tool, target, destination, contract['retreat_distance_m'])
        assert [place.pose.position.x, place.pose.position.y, place.pose.position.z - .1] == pytest.approx(authored['pose_xyz'])
        assert transfer.pose.position.z - place.pose.position.z == pytest.approx(contract['retreat_distance_m'])
        assert place.pose.orientation == tool.pose.orientation
        assert tool == original
        destinations.append(destination)
    assert all(d == destinations[0] for d in destinations)


@pytest.mark.parametrize('change,error', [
    ('missing', 'missing or ambiguous'), ('source', 'not a place_zone'),
    ('dimensions', 'dimensions are invalid'), ('pose', 'pose is invalid'),
    ('orientation', 'orientation is invalid'), ('frame', 'world transform'),
    ('duplicate', 'missing or ambiguous'),
])
def test_invalid_destination_fails_before_motion(tmp_path, change, error):
    config = cell()
    zone = next(z for z in config['environment']['task_zones'] if z['id'] == 'default_drop_zone')
    destination_id = zone['id']
    if change == 'missing': destination_id = 'absent_zone'
    if change == 'source': destination_id = 'pick_zone_main'
    if change == 'dimensions': zone['dimensions'] = [0., .1, .1]
    if change == 'pose': zone['pose_xyz'] = [float('nan'), 0., 0.]
    if change == 'orientation': zone['pose_rpy'] = [0., 0.]
    if change == 'frame': zone['frame'] = 'camera'
    if change == 'duplicate': config['environment']['task_zones'].append(copy.deepcopy(zone))
    (tmp_path / 'cell_definition.yaml').write_text(yaml.safe_dump(config))
    with pytest.raises(RuntimeError, match=error):
        executor.load_canonical_place_target(tmp_path, destination_id)


def test_explicit_destination_never_uses_legacy_fallback(tmp_path):
    with pytest.raises(RuntimeError, match='requires generated cell handoff'):
        executor.load_canonical_place_target(tmp_path, 'default_drop_zone')
