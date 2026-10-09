import copy
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from pile_extraction import ExtractionFailure, audit_extraction, extraction_intents


def box(name, xyz=(0., 0., .01), yaw=0.):
    return dict(id=name, shape='BOX', frame_id='world', dimensions=[.02]*3,
                pose=[*xyz, 0., 0., math.sin(yaw/2), math.cos(yaw/2)])


def test_vertical_below_support_clears_but_lateral_contact_needs_away():
    target = box('target-A')
    support = box('support', (0., 0., -.01))
    vertical = extraction_intents(target, [support], 'grasp::1', .1)[0]
    assert audit_extraction(target, [support], vertical)['success']
    neighbor = box('side', (.02, 0., .01))
    variants = extraction_intents(target, [neighbor, support], 'grasp::1', .1)
    assert variants[0]['variant_id'] == 'vertical'
    with pytest.raises(ExtractionFailure, match='NO_VALID_EXTRACTION') as failure:
        audit_extraction(target, [neighbor, support], variants[0])
    assert failure.value.reason_code == 'NO_VALID_EXTRACTION'
    assert failure.value.details['neighbor'] == 'side'
    assert failure.value.details['failure_kind'] == 'extraction'
    away = next(v for v in variants if v['offset_xyz_m'][0] < 0)
    evidence = audit_extraction(target, [neighbor, support], away)
    assert evidence['success'] and evidence['expiry']['side']['height_m'] <= .01
    assert evidence['initial_contacts'] == ['side', 'support']


def test_near_only_pair_is_not_contact_authority_and_must_clear():
    target = box('target-A'); near = box('near', (.020006, 0., .01))
    variants = extraction_intents(target, [near], 'grasp::1', .1)
    with pytest.raises(ExtractionFailure):
        audit_extraction(target, [near], variants[0])
    proof = audit_extraction(target, [near], variants[1])
    assert proof['initial_contacts'] == []
    assert proof['near_neighbors'] == ['near']
    assert proof['expiry']['near']['gap_m'] > .0001
    # A positive-gap neighbor never gets the numerical contact allowance.
    with pytest.raises(ExtractionFailure) as failure:
        audit_extraction(target, [near], variants[1],
                         poses=[target['pose'], [.000007, 0., .011, 0., 0., 0., 1.]])
    assert failure.value.details['extraction_reason_code'] == 'EXTRACTION_NEW_CONTACT'


def test_intents_bounded_deterministic_rotated_geometry_and_input_immutable():
    yaw = .7; target = box('target-A', yaw=yaw)
    a = box('side-A', (.020002*math.cos(yaw), .020002*math.sin(yaw), .01), yaw)
    b = box('side-B', (-.020002*math.sin(yaw), .020002*math.cos(yaw), .01), yaw)
    original = copy.deepcopy([target, a, b])
    intents = extraction_intents(target, [a, b], 'grasp::1', .1)
    assert intents == extraction_intents(target, [b, a], 'grasp::1', .1)
    assert 2 <= len(intents) <= 4
    assert all(math.hypot(*v['offset_xyz_m'][:2]) <= .002 + 1e-15 for v in intents)
    assert all(v['offset_xyz_m'][2] == .1 for v in intents)
    assert any(v['offset_xyz_m'][0] < 0 and v['offset_xyz_m'][1] < 0 for v in intents)
    assert [target, a, b] == original


def test_above_and_later_foreign_collisions_are_rejected():
    target = box('target-A'); above = box('above', (0., 0., .03))
    intent = extraction_intents(target, [above], 'grasp::1', .1)[0]
    with pytest.raises(ExtractionFailure):
        audit_extraction(target, [above], intent)
    obstacle = box('obstacle', (0., 0., .07))
    with pytest.raises(ExtractionFailure) as failure:
        audit_extraction(target, [obstacle], intent)
    assert failure.value.details['extraction_reason_code'] == 'EXTRACTION_NEW_CONTACT'


def test_expiry_is_irreversible_even_when_gap_only_returns_below_threshold():
    target = box('target-A'); neighbor = box('side', (.02, 0., .01))
    intent = extraction_intents(target, [neighbor], 'grasp::1', .1)[1]
    poses = [target['pose'], [-.0002, 0., .015, 0., 0., 0., 1.],
             [-.00005, 0., .016, 0., 0., 0., 1.]]
    with pytest.raises(ExtractionFailure) as failure:
        audit_extraction(target, [neighbor], intent, poses=poses)
    assert failure.value.details['extraction_reason_code'] == 'EXTRACTION_CLEARANCE_REVERSED'


@pytest.mark.parametrize('pose,reason', [
    ([0., .0026, .011, 0., 0., 0., 1.], 'EXTRACTION_CORRIDOR'),
    ([0., 0., .00999, 0., 0., 0., 1.], 'EXTRACTION_CORRIDOR'),
    ([0., 0., .011, 0., 0., math.sin(.011/2), math.cos(.011/2)], 'EXTRACTION_CORRIDOR'),
])
def test_actual_pose_audit_preserves_corridor(pose, reason):
    target = box('target-A'); neighbor = box('side', (.02, 0., .01))
    intent = extraction_intents(target, [neighbor], 'grasp::1', .1)[0]
    with pytest.raises(ExtractionFailure) as failure:
        audit_extraction(target, [neighbor], intent, poses=[target['pose'], pose])
    assert failure.value.details['extraction_reason_code'] == reason


def test_overdeep_and_invalid_geometry_fail_closed():
    target = box('target-A'); overdeep = box('deep', (.0198, 0., .01))
    with pytest.raises(ExtractionFailure) as failure:
        extraction_intents(target, [overdeep], 'grasp::1', .1)
    assert failure.value.details['extraction_reason_code'] == 'EXTRACTION_INITIAL_DEPTH'
    for broken in [dict(target, shape='SPHERE'), dict(target, dimensions=[float('nan'), .02, .02]),
                   dict(target, pose=[0., 0., .01, 0., 0., 0., 0.])]:
        with pytest.raises(ExtractionFailure):
            extraction_intents(broken, [], 'grasp::1', .1)


def test_densification_catches_obstacle_between_supplied_endpoints():
    target = box('target-A'); obstacle = box('obstacle', (0., 0., .06))
    intent = extraction_intents(target, [obstacle], 'grasp::1', .1)[0]
    with pytest.raises(ExtractionFailure) as failure:
        audit_extraction(target, [obstacle], intent,
                         poses=[target['pose'], [0., 0., .11, 0., 0., 0., 1.]])
    assert failure.value.details['extraction_reason_code'] == 'EXTRACTION_NEW_CONTACT'
    assert .02 < failure.value.details['height_m'] < .06


def test_almost_vertical_support_normal_does_not_create_lateral_intent():
    angle = .001
    target = box('target-A')
    target['pose'][3:] = [0., math.sin(angle/2), 0., math.cos(angle/2)]
    support = box('support', (-.020002*math.sin(angle), 0., .01-.020002*math.cos(angle)))
    support['pose'][3:] = list(target['pose'][3:])
    assert len(extraction_intents(target, [support], 'grasp::1', .1)) == 1
    side = box('side', (0., .020002, .01))
    side['pose'][3:] = list(target['pose'][3:])
    variants = extraction_intents(target, [support, side], 'grasp::1', .1)
    assert len(variants) == 2
    assert abs(variants[1]['offset_xyz_m'][0]) < 1e-10
    assert variants[1]['offset_xyz_m'][1] == pytest.approx(-.002)
    assert audit_extraction(target, [support, side], variants[1])['success']


def test_malformed_neighbor_and_intent_are_structured_failures():
    target = box('target-A')
    with pytest.raises(ExtractionFailure):
        extraction_intents(target, [None], 'grasp::1', .1)
    intent = extraction_intents(target, [], 'grasp::1', .1)[0]
    for changed in [dict(intent, object_id='different'), dict(intent, offset_xyz_m=[.003, 0., .1]),
                    dict(intent, offset_xyz_m=[0., 0., float('nan')])]:
        with pytest.raises(ExtractionFailure):
            audit_extraction(target, [], changed)
    with pytest.raises(ExtractionFailure):
        audit_extraction(target, [], intent, poses=[])
