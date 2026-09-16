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


def test_side_grip_basic_uses_horizontal_x_plus_approach():
    from grasp_strategy_candidates import generate_strategy_candidates
    observation = dict(id='moving-box', frame_id='world', dimensions=[.04, .08, .10],
                       pose=[.4, -.2, .3, 0., 0., .7071067811865475, .7071067811865476])
    intent = {'approach_distance_m': .08}
    saved = copy.deepcopy((observation, intent))

    candidates = generate_strategy_candidates('side_grip_basic', observation, intent)

    assert len(candidates) == 1
    candidate = candidates[0]
    assert candidate.candidate_id == 'side_grip_basic::000'
    assert candidate.object_id == 'moving-box'
    # The 90-degree object yaw projects the local 8 cm Y dimension onto world X.
    # Contact is on the +X face; pregrasp is a further authored 8 cm along +X.
    assert candidate.grasp_pose == pytest.approx(
        [.44, -.2, .3, 0., -.7071067811865475, 0., .7071067811865476])
    assert candidate.approach_pose == pytest.approx(
        [.52, -.2, .3, 0., -.7071067811865475, 0., .7071067811865476])
    assert candidate.effective == {
        'approach_axis': 'x_plus',
        'orientation_mode': 'horizontal',
        'approach_distance_m': .08,
    }
    assert (observation, intent) == saved


def test_side_grip_basic_changes_with_live_geometry_and_is_deterministic():
    from grasp_strategy_candidates import generate_strategy_candidates
    observation = dict(id='another-box', frame_id='world', dimensions=[.12, .03, .05],
                       pose=[-.1, .25, .4, 0., 0., 0., 1.])
    intent = {'approach_distance_m': .03,
              'approach_axis': 'x_plus', 'orientation_mode': 'horizontal'}

    first = generate_strategy_candidates('side_grip_basic', observation, intent)
    second = generate_strategy_candidates('side_grip_basic', copy.deepcopy(observation), copy.deepcopy(intent))

    assert first == second
    assert first[0].grasp_pose[:3] == pytest.approx([-.04, .25, .4])
    assert first[0].approach_pose[:3] == pytest.approx([-.01, .25, .4])


def test_side_grip_contact_intersects_the_actual_rotated_box_surface():
    from grasp_strategy_candidates import generate_strategy_candidates
    observation = dict(id='yawed-box', frame_id='world', dimensions=[.04, .08, .10],
                       pose=[.4, -.2, .3, 0., 0., .3826834323650898, .9238795325112867])

    candidate = generate_strategy_candidates(
        'side_grip_basic', observation, {'approach_distance_m': .08})[0]

    # A world +X ray through the centre reaches the local 20 mm X half-face
    # after 20 mm / cos(45 degrees), not at the enclosing AABB's +X extent.
    assert candidate.grasp_pose[:3] == pytest.approx(
        [.4282842712474619, -.2, .3])
    assert candidate.approach_pose[:3] == pytest.approx(
        [.5082842712474619, -.2, .3])


@pytest.mark.parametrize('constraint,value', [
    ('approach_axis', 'z_down'),
    ('orientation_mode', 'vertical'),
    ('retreat_axis', 'x_plus'),
])
def test_side_grip_basic_rejects_incompatible_or_unconsumed_constraints(constraint, value):
    from grasp_strategy_candidates import generate_strategy_candidates
    observation = dict(id='box', frame_id='world', dimensions=[.04, .04, .1],
                       pose=[0., 0., 0., 0., 0., 0., 1.])
    intent = {'approach_distance_m': .08, constraint: value}
    with pytest.raises(ValueError, match='unsupported|incompatible'):
        generate_strategy_candidates('side_grip_basic', observation, intent)


def test_side_grip_catalog_geometry_cannot_be_relabelled(tmp_path):
    from grasp_strategy_candidates import generate_strategy_candidates
    observation = dict(id='box', frame_id='world', dimensions=[.04, .04, .1],
                       pose=[0., 0., 0., 0., 0., 0., 1.])
    (tmp_path / 'side_grip_basic.yaml').write_text(
        'grasp_strategy:\n  id: side_grip_basic\n  approach_axis: z_down\n'
        '  orientation_mode: horizontal\n')
    with pytest.raises(ValueError, match='catalog'):
        generate_strategy_candidates('side_grip_basic', observation,
                                     {'approach_distance_m': .08}, tmp_path)


def test_side_grip_finds_catalog_from_installed_runtime_layout(tmp_path, monkeypatch):
    import grasp_strategy_candidates as module
    prefix = tmp_path / 'install' / 'workcell_builder'
    installed_script = prefix / 'lib' / 'workcell_builder' / 'grasp_strategy_candidates.py'
    installed_catalog = prefix / 'share' / 'workcell_builder' / 'catalog' / 'grasp_strategies'
    installed_catalog.mkdir(parents=True)
    (installed_catalog / 'side_grip_basic.yaml').write_text(
        'grasp_strategy:\n  id: side_grip_basic\n  approach_axis: x_plus\n'
        '  orientation_mode: horizontal\n')
    monkeypatch.setattr(module, '__file__', str(installed_script))
    observation = dict(id='box', frame_id='world', dimensions=[.04, .04, .1],
                       pose=[0., 0., 0., 0., 0., 0., 1.])

    candidate = module.generate_strategy_candidates(
        'side_grip_basic', observation, {'approach_distance_m': .08})[0]

    assert candidate.effective['approach_axis'] == 'x_plus'


@pytest.mark.parametrize('strategy', ['unknown'])
def test_unimplemented_strategy_fails_closed(strategy):
    from grasp_strategy_candidates import generate_strategy_candidates
    with pytest.raises(ValueError, match='unsupported'):
        generate_strategy_candidates(strategy, {}, {})


def test_top_2f_does_not_claim_unimplemented_v2_constraints():
    from grasp_strategy_candidates import generate_strategy_candidates
    with pytest.raises(ValueError, match='unsupported'):
        generate_strategy_candidates('top_2f', {}, {'orientation': {'mode': 'EXACT'}})


@pytest.mark.parametrize('distance', [float('nan'), float('inf'), -.1, True, [.1, .2]])
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


def test_finger_pinch_basic_tracks_tilted_object_and_catalog_yaws():
    from grasp_strategy_candidates import generate_strategy_candidates
    observation = dict(id='tilted-part', frame_id='world', dimensions=[.04, .06, .10],
                       pose=[.4, -.2, .3, 0., .7071067811865475, 0., .7071067811865476])
    intent = {'approach_distance_m': .07, 'approach_axis': 'tool_z',
              'orientation_mode': 'tool_aligned'}
    saved = copy.deepcopy((observation, intent))
    candidates = generate_strategy_candidates('finger_pinch_basic', observation, intent)
    assert candidates == generate_strategy_candidates('finger_pinch_basic', observation, intent)
    assert [c.candidate_id for c in candidates] == [f'finger_pinch_basic::{i:03}' for i in range(4)]
    assert all(c.object_id == 'tilted-part' and c.effective == intent for c in candidates)
    # Local +Z is world +X. All approaches follow the tilted face, not world Z.
    assert all(c.grasp_pose[:3] == pytest.approx([.45, -.2, .3]) for c in candidates)
    assert all(c.approach_pose[:3] == pytest.approx([.52, -.2, .3]) for c in candidates)
    assert candidates[0].grasp_pose[3:] == pytest.approx([.7071067811865476, 0., -.7071067811865475, 0.])
    assert candidates[1].grasp_pose[3:] == pytest.approx([.5, .5, -.5, -.5])
    assert (observation, intent) == saved


@pytest.mark.parametrize('field,value', [('approach_axis', 'x_plus'),
                                         ('orientation_mode', 'horizontal'),
                                         ('force_limit_n', 4.)])
def test_finger_pinch_basic_rejects_unconsumed_authored_constraints(field, value):
    from grasp_strategy_candidates import generate_strategy_candidates
    with pytest.raises(ValueError, match='unsupported|incompatible'):
        generate_strategy_candidates('finger_pinch_basic', {},
                                     {'approach_distance_m': .07, field: value})
