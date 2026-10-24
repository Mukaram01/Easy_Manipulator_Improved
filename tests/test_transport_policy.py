"""Authored carried-payload transport policy (transport_policy/v1)."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import transport_policy as tp  # noqa: E402


def policy(**fields):
    return dict({'schema': tp.SCHEMA, 'orientation': 'keep_level', 'tilt_tolerances_rad': [0.05, 0.1]}, **fields)


def test_absent_policy_keeps_legacy_free_transfer():
    assert tp.parse_transport_policy(None) is None
    assert tp.policy_from_contract({}) is None and tp.policy_from_contract({'transport_policy': None}) is None


def test_keep_level_yields_a_bounded_ladder_and_an_optional_unconstrained_fallback():
    parsed = tp.parse_transport_policy(policy())
    assert [a.as_dict() for a in parsed.attempts()] == [
        {'label': 'keep_level_0', 'tilt_tolerance_rad': .05}, {'label': 'keep_level_1', 'tilt_tolerance_rad': .1}]
    fallback = tp.parse_transport_policy(policy(allow_unconstrained_fallback=True))
    assert [a.tilt_tolerance_rad for a in fallback.attempts()] == [.05, .1, None]
    assert len(fallback.attempts()) <= tp.MAX_ATTEMPTS
    assert [a.tilt_tolerance_rad for a in tp.parse_transport_policy(
        {'schema': tp.SCHEMA, 'orientation': 'free'}).attempts()] == [None]
    assert tp.policy_from_contract({'transport_policy': parsed.as_contract()}) == parsed


def test_destination_approach_height_never_lowers_the_authored_approach():
    parsed = tp.parse_transport_policy(policy(destination_approach_height_m=0.2))
    assert parsed.place_approach_m(0.1) == 0.2 and parsed.place_approach_m(0.3) == 0.3
    assert tp.parse_transport_policy(policy()).place_approach_m(0.1) == 0.1


@pytest.mark.parametrize('fields,message', [
    ({'schema': 'transport_policy/v0'}, 'schema'),
    ({'orientation': 'tilt_free'}, 'orientation'),
    ({'tilt_tolerances_rad': []}, 'strictly increasing'),
    ({'tilt_tolerances_rad': [0.1, 0.05]}, 'strictly increasing'),
    ({'tilt_tolerances_rad': [0.05, 0.6]}, 'strictly increasing'),
    ({'tilt_tolerances_rad': [0.01, 0.02, 0.03], 'allow_unconstrained_fallback': True}, 'strictly increasing'),
    ({'tilt_tolerances_rad': [math.nan]}, 'strictly increasing'),
    ({'allow_unconstrained_fallback': 'yes'}, 'true or false'),
    ({'destination_approach_height_m': 0}, 'destination_approach_height_m'),
    ({'via_joint_path': [0, 1, 2]}, 'unknown transport policy fields'),
])
def test_invalid_policies_fail_closed(fields, message):
    with pytest.raises(tp.TransportPolicyError, match=message):
        tp.parse_transport_policy(policy(**fields))
    with pytest.raises(tp.TransportPolicyError, match='no tolerance ladder'):
        tp.parse_transport_policy({'schema': tp.SCHEMA, 'orientation': 'free', 'tilt_tolerances_rad': [.1]})


def test_keep_level_constraint_bounds_tilt_and_frees_yaw():
    constraint = tp.keep_level_constraint('tool0', 'world', [1.0, 0.0, 0.0, 0.0], 0.05)
    assert constraint.link_name == 'tool0' and constraint.header.frame_id == 'world'
    assert (constraint.orientation.x, constraint.orientation.w) == (1.0, 0.0)
    assert (constraint.absolute_x_axis_tolerance, constraint.absolute_y_axis_tolerance) == (0.05, 0.05)
    assert constraint.absolute_z_axis_tolerance == pytest.approx(math.pi) and constraint.weight == 1.0
