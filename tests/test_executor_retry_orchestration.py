"""Exercise production retry orchestration; planning success is explicitly stubbed."""
import itertools
import sys
from pathlib import Path
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import perceived_object_grasp_execute as executor


def targets(width=.06):
    return [dict(id='runtime::cup_A', confidence=.9, frame_id='world', shape='BOX',
                 pose=[0., 0., 0., 0., 0., 0., 1.], dimensions=[width, width, .1]),
            dict(id='runtime::cup_B', confidence=.8, frame_id='world', shape='BOX',
                 pose=[0., 0., 0., 0., 0., 0., 1.], dimensions=[.06, .06, .1])]


def accepted(target, index):
    # The established preplan callback contract, not evidence of a MoveIt plan.
    return dict(object_id=target['id'], grasp_index=index, full_cycle_prevalidated=True)


@pytest.mark.parametrize('reverse', [False, True])
def test_grasp_retry_planning_result_seam(reverse):
    objects = targets()
    candidates = executor.generate_box_grasp_candidates(executor.build_grasp_target(objects[0]))
    assert len(candidates) == 8
    indices = executor.candidate_indices(len(candidates))
    planning_result = Mock(side_effect=[
        executor.CandidateFailure('PREPLAN_APPROACH', 'MoveIt returned empty trajectory'),
        accepted(objects[0], indices[1])])
    attempts = []
    result = executor.choose_cycle(objects[::-1] if reverse else objects, indices, planning_result, attempts)
    assert [(a['object_id'], a['grasp_index']) for a in attempts] == [
        ('runtime::cup_A', 3), ('runtime::cup_A', 0)]
    assert attempts[0]['reason'] == 'MoveIt returned empty trajectory'
    assert attempts[0]['failed_stage'] == 'PREPLAN_APPROACH'
    assert attempts[0]['full_cycle_prevalidated'] is False
    assert attempts[1]['full_cycle_prevalidated'] is True
    assert result == accepted(objects[0], 0)
    assert planning_result.call_count == 2
    # Plan-only must still forbid execution even after accepted stubbed planning.
    with pytest.raises(RuntimeError, match='execution requires'):
        executor.require_prevalidated_execution(False, result)


@pytest.mark.parametrize('reverse', [False, True])
def test_object_exhaustion_from_real_aperture_gate(reverse):
    objects = targets(width=.12)
    attempts = []
    def preplan(target, index, record):
        geometry = executor.eligible_grasp_target(target, dict(dimensions=[1., 1., 1.]))
        assert len(executor.generate_box_grasp_candidates(geometry)) == 8
        return accepted(target, index)
    result = executor.choose_cycle(objects[::-1] if reverse else objects,
                                   executor.candidate_indices(8), preplan, attempts)
    assert [(a['object_id'], a['grasp_index']) for a in attempts[:8]] == [
        ('runtime::cup_A', i) for i in executor.candidate_indices(8)]
    assert all(a['reason'] == 'target exceeds Robotiq aperture' and
               not a['full_cycle_prevalidated'] for a in attempts[:8])
    assert len(attempts) == 9
    assert attempts[-1]['object_id'] == 'runtime::cup_B'
    assert result == accepted(objects[1], 3)


@pytest.mark.parametrize('reverse', [False, True])
def test_all_pairs_exhausted_without_execution(reverse):
    objects = targets(width=.12)
    objects[1]['dimensions'] = [.12, .12, .1]
    attempts = []
    execution = Mock()
    def preplan(target, index, record):
        executor.eligible_grasp_target(target, dict(dimensions=[1., 1., 1.]))
        return accepted(target, index)
    with pytest.raises(executor.CandidateFailure, match='no target/grasp has a feasible complete cycle') as error:
        cycle = executor.choose_cycle(objects[::-1] if reverse else objects,
                                      executor.candidate_indices(8), preplan, attempts)
        execution(cycle)
    execution.assert_not_called()
    assert error.value.stage == 'GENERATE_GRASPS'
    assert len(attempts) == 16
    assert [(a['object_id'], a['grasp_index']) for a in attempts] == list(itertools.product(
        ['runtime::cup_A', 'runtime::cup_B'], executor.candidate_indices(8)))
    assert all(a['reason'] == 'target exceeds Robotiq aperture' and
               not a['full_cycle_prevalidated'] for a in attempts)


def test_empty_candidates_and_nonretryable_callback_error():
    attempts = []
    plan = Mock()
    with pytest.raises(executor.CandidateFailure, match='no eligible targets'):
        executor.choose_cycle([], executor.candidate_indices(8), plan, attempts)
    plan.assert_not_called()
    assert attempts == []
    # choose_cycle catches only CandidateFailure; other callback errors escape.
    plan.side_effect = RuntimeError('unexpected callback error')
    with pytest.raises(RuntimeError, match='unexpected callback error'):
        executor.choose_cycle(targets(), executor.candidate_indices(8), plan, attempts)
    assert plan.call_count == 1
