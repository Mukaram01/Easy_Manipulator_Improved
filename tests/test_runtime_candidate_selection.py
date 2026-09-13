"""Headless selection/grasp-boundary probe; no claim of MoveIt feasibility."""
import copy
import itertools
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import runtime_pick_inputs as inputs
import perceived_object_grasp_execute as executor


def scenario():
    region = dict(frame='world', pose_xyz=[0., 0., 0.], dimensions=[2., 2., 2.])
    cell = dict(environment=dict(task_zones=[dict(region, id='source'), dict(region, id='place')]))
    task = inputs.task_request(dict(action='pick_and_place', target_class='cup',
        source_zone='source', destination_zone='place'), cell)
    def obj(oid, label, width, confidence):
        return dict(object_id=oid, class_id=label, confidence=confidence, timestamp=100.,
            pose=dict(frame_id='world', xyz=[0., 0., 0.], rpy=[0., 0., 0.]),
            dimensions=[width, width, 0.1])
    return cell, task, [obj('cup_A', 'cup', .06, .8),
                        obj('cup_B', 'cup', .12, .9),
                        obj('bottle_A', 'bottle', .06, .99)]


@pytest.mark.parametrize('order', list(itertools.permutations(range(3))))
def test_real_aperture_failure_falls_back_independent_of_order(order):
    cell, task, raw = scenario()
    objects = inputs.normalize(dict(schema_version='detected_objects/v1',
        objects=[raw[i] for i in order]), 100., executor._PLANNER)
    eligible, rejected = inputs.filter_targets(objects, task, cell, 100., executor._PLANNER)
    assert rejected == {'runtime::bottle_A': 'class_mismatch'}
    assert [o['id'] for o in eligible] == ['runtime::cup_B', 'runtime::cup_A']
    attempts = []
    def grasp_probe(target, index, record):
        geometry = executor.eligible_grasp_target(target, inputs.zone(cell, 'place'))
        candidates = executor.generate_box_grasp_candidates(geometry)
        return dict(object_id=target['id'], geometry=geometry, candidates=candidates)
    result = executor.choose_cycle(eligible, executor.candidate_indices(8), grasp_probe, attempts)
    assert result['object_id'] == 'runtime::cup_A'
    assert result['geometry']['perceived_object_id'] == 'runtime::cup_A'
    assert len(result['candidates']) == 8
    assert len(attempts) == 9
    assert all(a['reason'] == 'target exceeds Robotiq aperture' and
               a['failed_stage'] == 'GENERATE_GRASPS' for a in attempts[:-1])
    assert attempts[-1]['object_id'] == 'runtime::cup_A'


def test_equal_confidence_uses_stable_identity():
    cell, task, raw = scenario()
    raw[1]['confidence'] = raw[0]['confidence']
    for perm in itertools.permutations(raw):
        objects = inputs.normalize(dict(schema_version='detected_objects/v1', objects=list(perm)),
                                  100., executor._PLANNER)
        eligible, _ = inputs.filter_targets(objects, task, cell, 100., executor._PLANNER)
        assert [o['id'] for o in eligible] == ['runtime::cup_A', 'runtime::cup_B']


def test_destination_bounds_uses_existing_constraint():
    _, _, raw = scenario()
    target = inputs.normalize(dict(schema_version='detected_objects/v1', objects=[raw[0]]),
                              100., executor._PLANNER)[0]
    with pytest.raises(executor.CandidateFailure, match='destination bounds'):
        executor.eligible_grasp_target(target, dict(dimensions=[.05, .05, .05]))


@pytest.mark.parametrize('order', list(itertools.permutations(range(3))))
def test_invalid_geometry_fails_closed_before_selection(order):
    _, _, raw = scenario()
    raw[1]['dimensions'][0] = 0.
    with pytest.raises(ValueError, match='cup_B: dimensions must be positive'):
        inputs.normalize(dict(schema_version='detected_objects/v1', objects=[raw[i] for i in order]),
                         100., executor._PLANNER)
