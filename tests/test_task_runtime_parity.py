"""Semantic identity and real consumer parity; evaluator fixtures are unit evidence only."""
import copy
import json
from pathlib import Path

import pytest
import yaml

from tests.test_task_intent_resolver import valid_intent, environment, cell, observations, pass_cycle
from scripts.task_intent_resolver import (
    resolve_task_intent, consume_resolution, resolution_hash, write_resolution_artifacts,
    resolved_recipe,
)
from scripts.task_intent_v2 import normalized_intent_hash


def test_null_confidence_has_the_authored_unrestricted_semantics():
    intent = valid_intent()
    intent['pick']['selection']['object_filter']['min_confidence'] = None
    objects = observations()
    objects[0]['confidence'] = None
    result = resolve_task_intent(intent, environment(), cell(), objects, pass_cycle, now=100.)
    assert result['readiness_status'] == 'READY'
    intent['pick']['selection']['object_filter']['min_confidence'] = .5
    assert resolve_task_intent(intent, environment(), cell(), objects, pass_cycle, now=100.)['readiness_status'] == 'BLOCKED'


def test_intent_hash_ignores_paths_and_migration_provenance_but_not_policy():
    intent = valid_intent()
    same = copy.deepcopy(intent)
    same.update(scene_package='/different/workspace/cell', provenance={'migration': {'note': 'historical'}})
    assert normalized_intent_hash(same) == normalized_intent_hash(intent)
    same['pick']['grasp']['policy'] = 'PREFERRED'
    same['pick']['grasp']['strategy_ref'] = 'top_2f'
    assert normalized_intent_hash(same) != normalized_intent_hash(intent)


def test_resolution_identity_roundtrip_staleness_destination_edit_and_restore(tmp_path):
    intent, physical, installed = valid_intent(), environment(), cell()
    first = resolve_task_intent(intent, physical, installed, observations(), pass_cycle, now=100.)
    assert first['resolution_sha256'] == resolution_hash(first)
    write_resolution_artifacts(first, tmp_path)
    loaded = json.loads((tmp_path / 'task_intent_resolution.json').read_text())
    assert loaded == yaml.safe_load((tmp_path / 'task_intent_resolution.yaml').read_text())
    assert consume_resolution(loaded, intent, physical, installed) == first
    recipe = resolved_recipe(intent, first)
    assert recipe['resolution_sha256'] == first['resolution_sha256']
    original = copy.deepcopy(physical)
    physical['assets'][0]['pose_xyz'][0] += .01
    physical['task_zones'][1]['pose_xyz'][0] += .01
    with pytest.raises(ValueError, match='STALE'):
        consume_resolution(first, intent, physical, installed)
    edited = resolve_task_intent(intent, physical, installed, observations(), pass_cycle, now=100.)
    assert edited['normalized_intent_sha256'] == first['normalized_intent_sha256']
    assert edited['resolution_sha256'] != first['resolution_sha256']
    assert edited['place_resolution']['destination']['pose_xyz'][0] == pytest.approx(1.06)
    restored = resolve_task_intent(intent, original, installed, observations(), pass_cycle, now=100.)
    assert restored == first


def test_no_evaluator_cannot_manufacture_ready_or_a_recipe():
    intent = valid_intent()
    unresolved = resolve_task_intent(intent, environment(), cell(), [], None, now=0.)
    assert unresolved['readiness_status'] == 'BLOCKED'
    assert unresolved['readiness']['primary_code'] == 'CYCLE_EVALUATION_REQUIRED'
    assert unresolved['place_resolution']['destination']['target_id'] == 'bin'
    with pytest.raises(ValueError, match='CYCLE_EVALUATION_REQUIRED'):
        resolved_recipe(intent, unresolved)


def test_observation_replay_timestamp_is_not_resolution_identity():
    first = resolve_task_intent(valid_intent(), environment(), cell(), observations(), pass_cycle, now=100.)
    replay = observations()
    replay[0]['timestamp'] += 10
    second = resolve_task_intent(valid_intent(), environment(), cell(), replay, pass_cycle, now=110.)
    assert first['resolution_sha256'] == second['resolution_sha256']
    replay[0]['pose'][0] += .01
    moved = resolve_task_intent(valid_intent(), environment(), cell(), replay, pass_cycle, now=110.)
    assert moved['resolution_sha256'] != first['resolution_sha256']


def test_preferred_unreachable_placement_attempts_default_but_exact_never_does():
    intent = valid_intent('EXACT', 'PREFERRED')
    intent['place']['placement']['requested_local_pose']['xyz_m'][0] = .06
    calls = []
    def evaluate(request):
        x = request['destination']['pose_xyz'][0]
        calls.append(x)
        if x > 1.055:
            return {'success': False, 'reason_code': 'PLACE_UNREACHABLE', 'reason': 'requested place unreachable'}
        return pass_cycle(request)
    preferred = resolve_task_intent(intent, environment(), cell(), observations(), evaluate, now=100.)
    assert calls == pytest.approx([1.06, 1.05])
    assert preferred['readiness_status'] == 'WARNING'
    assert preferred['place_resolution']['fallback']['reason_code'] == 'PLACE_UNREACHABLE'
    calls.clear()
    intent['place']['placement']['policy'] = 'EXACT'
    exact = resolve_task_intent(intent, environment(), cell(), observations(), evaluate, now=100.)
    assert calls == pytest.approx([1.06])
    assert exact['readiness_status'] == 'BLOCKED'
    assert exact['place_resolution']['fallback']['used'] is False


def test_consuming_resolution_replans_selected_candidate_without_new_auto_search():
    intent = valid_intent()
    def initial(request):
        if request['candidate'].candidate_id.endswith('000'):
            return {'success': False, 'reason_code': 'PLAN_FAILED', 'reason': 'transient failure'}
        return pass_cycle(request)
    chosen = resolve_task_intent(intent, environment(), cell(), observations(), initial, now=100.)
    calls = []
    def consume(request):
        calls.append(request['candidate'].candidate_id)
        return pass_cycle(request)
    checked = resolve_task_intent(intent, environment(), cell(), observations(), consume,
                                  now=100., resolved=chosen)
    assert calls == ['top_2f::001']
    assert checked['resolution_sha256'] == chosen['resolution_sha256']
    calls.clear()
    def fails(request):
        calls.append(request['candidate'].candidate_id)
        return {'success': False, 'reason_code': 'PLAN_FAILED', 'reason': 'blocked'}
    blocked = resolve_task_intent(intent, environment(), cell(), observations(), fails,
                                  now=100., resolved=chosen)
    assert calls == ['top_2f::001']
    assert blocked['readiness_status'] == 'BLOCKED'


def test_direct_task_recipe_works_without_optional_routing():
    from scripts.validate_task_recipe import validate_task_recipe_doc
    from scripts.dry_run_task_recipe import _evaluate_rules
    intent = valid_intent()
    del intent['routing']
    result = resolve_task_intent(intent, environment(), cell(), observations(), pass_cycle, now=100.)
    recipe = resolved_recipe(intent, result)
    assert validate_task_recipe_doc(recipe, Path('recipe.yaml'), 'yaml', []).ok
    rule, error = _evaluate_rules(recipe['decision_rules'], {})
    assert error is None and rule['destination'] == 'drop'


def test_conditional_routing_and_unsupported_source_cannot_be_ignored():
    intent = valid_intent()
    intent['routing']['rules'][0]['when'] = {'class_id': 'other'}
    assert resolve_task_intent(intent, environment(), cell(), observations(), pass_cycle, now=100.)['readiness_status'] == 'BLOCKED'
    intent = valid_intent()
    intent['pick']['selection']['source_ref'] = 'another_stream'
    assert resolve_task_intent(intent, environment(), cell(), observations(), pass_cycle, now=100.)['readiness_status'] == 'BLOCKED'


def test_preferred_rotation_only_request_falls_back_explicitly():
    intent = valid_intent('EXACT', 'PREFERRED')
    intent['place']['placement']['requested_local_pose']['rpy_rad'][2] = .01
    calls = []
    def evaluate(request):
        yaw = request['destination']['pose_rpy'][2]
        calls.append(yaw)
        return {'success': False, 'reason_code': 'ROTATION_BLOCKED', 'reason': 'orientation'} if yaw else pass_cycle(request)
    result = resolve_task_intent(intent, environment(), cell(), observations(), evaluate, now=100.)
    assert calls == [0.01, 0.0]
    assert result['readiness_status'] == 'WARNING'
    assert result['place_resolution']['fallback']['used'] is True


def test_authored_aperture_uses_actual_closing_axis_and_rotation():
    from tests.test_full_cycle_preplanner import fixture
    from scripts.grasp_strategy_candidates import generate_strategy_candidates
    from scripts.full_cycle_preplanner import preplan_full_cycle
    from scripts.perceived_object_grasp_plan import quaternion_from_rpy
    import math
    for rotation, index in ((0., 0), (math.pi / 2, 2)):
        kwargs, _, goals, _ = fixture()
        kwargs['observation']['dimensions'] = [.04, .08, .03]
        kwargs['observation']['pose'][3:] = quaternion_from_rpy([0., 0., rotation])
        kwargs['candidate'] = generate_strategy_candidates('top_2f', kwargs['observation'], {'approach_distance_m': .12})[index]
        intent = valid_intent()
        intent['pick']['grasp']['aperture']['max_m'] = .05
        kwargs['contract']['task_intent'] = intent
        result = preplan_full_cycle(**kwargs)
        assert not result.success and 'aperture' in result.reason
        assert goals == []


@pytest.mark.parametrize('strategy,axis,mode', [('side_grip_basic', 'x_plus', 'horizontal'), ('finger_pinch_basic', 'tool_z', 'tool_aligned')])
def test_candidate_yaw_restrictions_fail_before_any_plan(strategy, axis, mode):
    from tests.test_full_cycle_preplanner import fixture
    from scripts.grasp_strategy_candidates import generate_strategy_candidates
    from scripts.full_cycle_preplanner import preplan_full_cycle
    kwargs, _, goals, _ = fixture()
    kwargs['candidate'] = generate_strategy_candidates(strategy, kwargs['observation'], {'approach_distance_m': .12})[0]
    intent = valid_intent()
    intent['pick']['grasp']['approach']['axis'] = axis
    intent['pick']['grasp']['orientation'].update(mode=mode, allowed_yaw_deg=[90])
    kwargs['contract']['task_intent'] = intent
    result = preplan_full_cycle(**kwargs)
    assert not result.success and 'yaw' in result.reason
    assert goals == []


@pytest.mark.parametrize('block', ['grasp', 'placement'])
def test_custom_orientation_tolerance_is_explicitly_blocked(block):
    from tests.test_full_cycle_preplanner import fixture
    from scripts.full_cycle_preplanner import preplan_full_cycle
    kwargs, _, goals, _ = fixture()
    intent = valid_intent()
    orientation = intent['pick']['grasp']['orientation'] if block == 'grasp' else intent['place']['placement']['orientation']
    orientation['tolerance_rad'] = [.001, .001, .001]
    kwargs['contract']['task_intent'] = intent
    result = preplan_full_cycle(**kwargs)
    assert not result.success and 'custom orientation tolerances' in result.reason
    assert goals == []


def test_resolution_consumer_rejects_changed_observations_and_tampered_identity():
    intent = valid_intent()
    chosen = resolve_task_intent(intent, environment(), cell(), observations(), pass_cycle, now=100.)
    changed = observations()
    changed[0]['pose'][0] += .001
    def never(_):
        pytest.fail('changed context must block before planning')
    blocked = resolve_task_intent(intent, environment(), cell(), changed, never, now=100., resolved=chosen)
    assert blocked['readiness']['primary_code'] == 'TASK_OBSERVATIONS_CHANGED'
    chosen['place_resolution']['destination']['pose_xyz'][0] += .001
    with pytest.raises(ValueError, match='CORRUPT'):
        consume_resolution(chosen, intent, environment(), cell())


def test_new_resolution_cannot_be_consumed_with_old_generated_handoff():
    intent = valid_intent()
    first = resolve_task_intent(intent, environment(), cell(), observations(), pass_cycle, now=100.)
    def changed_feasibility(request):
        return {'success': False, 'reason_code': 'PLAN_FAILED'} if request['candidate'].candidate_id.endswith('000') else pass_cycle(request)
    second = resolve_task_intent(intent, environment(), cell(), observations(), changed_feasibility, now=100.)
    assert first['resolution_sha256'] != second['resolution_sha256']
    with pytest.raises(ValueError, match='TASK_HANDOFF_STALE'):
        consume_resolution(second, intent, environment(), cell(), generated_cell={'resolution_sha256': first['resolution_sha256']})
    assert consume_resolution(second, intent, environment(), cell(), generated_cell={'resolution_sha256': second['resolution_sha256']}) == second


def test_offline_generated_preview_projects_authored_motion_not_legacy_defaults():
    from scripts.generate_offline_plan_preview_request import generate
    intent = valid_intent()
    intent['pick']['grasp']['approach']['distance_m'] = .17
    intent['pick']['grasp']['lift']['distance_m'] = .23
    intent['place']['placement']['approach']['distance_m'] = .19
    intent['place']['placement']['retreat']['distance_m'] = .14
    result = resolve_task_intent(intent, environment(), cell(), observations(), pass_cycle, now=100.)
    recipe = resolved_recipe(intent, result)
    generated = {**cell(), 'builder_task_intent': intent, 'task_intent_resolution': result,
                 'normalized_intent_sha256': result['normalized_intent_sha256'],
                 'resolution_sha256': result['resolution_sha256']}
    preview, _, missing = generate(recipe, Path('recipe.yaml'), {}, generated, {})
    assert not missing
    assert preview['resolution_sha256'] == result['resolution_sha256']
    assert preview['request']['pick']['approach_distance_m'] == .17
    assert preview['request']['pick']['retreat_distance_m'] == .23
    assert preview['request']['place']['retreat_distance_m'] == .14
    assert preview['request']['place']['pose_xyz'] == result['place_resolution']['destination']['pose_xyz']
    assert preview['request']['place']['place_offset_xyz'] == [0, 0, 0]
    distances = {w['id']: w['distance_m'] for w in preview['request']['waypoints'] if 'distance_m' in w}
    assert distances == {'pre_pick': .17, 'post_pick': .23, 'pre_place': .19, 'post_place': .14}


def test_resolved_recipe_without_cell_cannot_enter_legacy_preview_defaults():
    from scripts.generate_offline_plan_preview_request import generate
    intent = valid_intent()
    result = resolve_task_intent(intent, environment(), cell(), observations(), pass_cycle, now=100.)
    with pytest.raises(ValueError, match='TASK_HANDOFF_REQUIRED'):
        generate(resolved_recipe(intent, result), Path('recipe.yaml'), {}, {}, {})
