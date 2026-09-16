"""Top strategy extraction: pose/order drift would change the selected grasp."""
import copy
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))


def test_top_2f_matches_legacy_top_down_candidate_order():
    from grasp_strategy_candidates import generate_strategy_candidates
    from perceived_object_grasp_plan import build_grasp_target, generate_box_grasp_candidates
    observation = dict(id='arbitrary-box', frame_id='world', dimensions=[.04, .06, .1],
                       pose=[.4, -.2, .3, 0., 0., .7071067811865475, .7071067811865476])
    intent = {'approach_distance_m': .17}
    saved = copy.deepcopy((observation, intent))
    candidates = generate_strategy_candidates('top_2f', observation, intent)
    # Existing geometry is the extraction oracle; hand-checked height and IDs
    # independently constrain accidental shared-helper changes.
    geometry = build_grasp_target(observation)
    assert [list(c.grasp_pose) for c in candidates] == generate_box_grasp_candidates(geometry, 0.)
    assert [list(c.approach_pose) for c in candidates] == generate_box_grasp_candidates(geometry, .17)
    assert len(candidates) == 8
    assert [c.candidate_id for c in candidates] == [f'top_2f::{i:03}' for i in range(8)]
    assert all(c.object_id == 'arbitrary-box' and c.strategy_ref == 'top_2f' for c in candidates)
    assert candidates[0].grasp_pose == pytest.approx([.4, -.2, .35, 1., 0., 0., 0.])
    assert candidates[3].approach_pose == pytest.approx([.4, -.2, .52, -.7071067811865475, .7071067811865476, 0., 0.])
    assert candidates[0].effective == {'approach_distance_m': .17}
    assert (observation, intent) == saved


@pytest.mark.parametrize('strategy', ['unknown', 'side_grip_basic', 'finger_pinch_basic'])
def test_unimplemented_strategy_fails_closed(strategy):
    from grasp_strategy_candidates import generate_strategy_candidates
    with pytest.raises(ValueError, match='unsupported'):
        generate_strategy_candidates(strategy, {}, {})


def test_top_2f_does_not_claim_unimplemented_v2_constraints():
    from grasp_strategy_candidates import generate_strategy_candidates
    with pytest.raises(ValueError, match='unsupported'):
        generate_strategy_candidates('top_2f', {}, {'orientation': {'mode': 'EXACT'}})


@pytest.mark.parametrize('distance', [float('nan'), float('inf'), -.1, [.1, .2]])
def test_top_2f_rejects_invalid_approach_distance(distance):
    from grasp_strategy_candidates import generate_strategy_candidates
    observation = dict(id='box', frame_id='world', dimensions=[.04, .04, .1], pose=[0., 0., 0., 0., 0., 0., 1.])
    with pytest.raises(ValueError, match='approach distance'):
        generate_strategy_candidates('top_2f', observation, {'approach_distance_m': distance})


def test_top_2f_catalog_is_vocabulary_validation_not_a_distance_override(tmp_path):
    from grasp_strategy_candidates import generate_strategy_candidates
    observation = dict(id='box', frame_id='world', dimensions=[.04, .04, .1], pose=[0., 0., 0., 0., 0., 0., 1.])
    catalog = tmp_path / 'top_2f.yaml'
    catalog.write_text('grasp_strategy:\n  id: wrong\n')
    with pytest.raises(ValueError, match='catalog'):
        generate_strategy_candidates('top_2f', observation, {'approach_distance_m': .17}, tmp_path)
    catalog.write_text('grasp_strategy:\n  id: top_2f\n  approach_distance_m: 99\n')
    assert generate_strategy_candidates('top_2f', observation, {'approach_distance_m': .17}, tmp_path)[0].approach_pose[2] == pytest.approx(.22)
