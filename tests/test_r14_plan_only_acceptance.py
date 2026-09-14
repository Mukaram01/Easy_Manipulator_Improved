import copy
import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('r14_acceptance', Path(__file__).parents[1]/'scripts/run_r14_plan_only_acceptance.py')
acceptance = importlib.util.module_from_spec(spec)
spec.loader.exec_module(acceptance)


def successful_result():
    return dict(result='PLAN_ONLY', full_cycle_prevalidated=True, execution_attempted=False,
                prevalidation_left_live_scene_unchanged=True, selected_object_id='runtime::cup',
                inserted_object_ids=['runtime::cup'],
                plan_metadata=[dict(stage=s, success=True, moveit_code=1, points=2) for s in acceptance.STAGES])


def test_acceptance_requires_the_complete_plan_and_no_execution():
    result = successful_result()
    acceptance.check_result(result, set())
    for stage in acceptance.STAGES:
        incomplete = copy.deepcopy(result)
        incomplete['plan_metadata'] = [p for p in incomplete['plan_metadata'] if p['stage'] != stage]
        with pytest.raises(RuntimeError):
            acceptance.check_result(incomplete, set())
    with pytest.raises(RuntimeError, match='execution action'):
        acceptance.check_result(result, {('/execute_trajectory', 'goal')})


@pytest.mark.parametrize('key,value', [('result', 'PASS'), ('full_cycle_prevalidated', False),
    ('execution_attempted', True), ('prevalidation_left_live_scene_unchanged', False)])
def test_acceptance_rejects_incomplete_or_executed_results(key, value):
    result = successful_result()
    result[key] = value
    with pytest.raises(RuntimeError):
        acceptance.check_result(result, set())


def test_process_exit_is_not_clean_if_moveit_crashed():
    clean = '[ERROR] [node]: process has died [pid 123, exit code -2, cmd /node]'
    crash = '[ERROR] [move_group]: process has died [pid 124, exit code -11, cmd /move_group]'
    assert acceptance.process_failures(clean) == []
    assert acceptance.process_failures(clean + '\n' + crash) == [crash]


def executed_result():
    result = successful_result()
    result.update(result='PASS', selected_object_id='runtime::sample-cup', selected_grasp_index=3,
        execution_attempted=True, full_cycle_execution_success=True, attach_verified=True,
        detach_verified=True, home_verified=True, destination_verified=True,
        final_collision_valid=True, baseline_acm_restored=True, shutdown_clean=True,
        execution_results=[dict(stage=s.replace('PREPLAN_', 'EXECUTE_'), code=1, action_status=4)
                           for s in acceptance.STAGES],
        fake_hardware_guard=dict(move_group_use_fake_hardware=True, real_hardware=False,
                                 hardware_classes=['mock_components/GenericSystem'] * 2),
        final_planning_scene=dict(world_ids=['runtime::sample-cup', 'runtime::sample-bottle'], attached_ids=[]))
    statuses = {(topic, str(i)): 4 for topic, count in [('/execute_trajectory', 9),
        ('/ur5_arm_controller/follow_joint_trajectory', 7),
        ('/ur5_gripper_controller/follow_joint_trajectory', 2)] for i in range(count)}
    return result, statuses


def test_execution_requires_all_nine_successful_results_and_controller_completion():
    result, statuses = executed_result()
    acceptance.check_execution_result(result, statuses)
    for index in range(9):
        failed = copy.deepcopy(result)
        failed['execution_results'][index]['code'] = -1
        with pytest.raises(RuntimeError, match='action results'):
            acceptance.check_execution_result(failed, statuses)
        failed['execution_results'].pop(index)
        with pytest.raises(RuntimeError, match='action results'):
            acceptance.check_execution_result(failed, statuses)
    for key in statuses:
        accepted_only = dict(statuses)
        accepted_only[key] = 2
        with pytest.raises(RuntimeError, match='terminal success'):
            acceptance.check_execution_result(result, accepted_only)


@pytest.mark.parametrize('key', ['full_cycle_prevalidated', 'prevalidation_left_live_scene_unchanged',
    'full_cycle_execution_success', 'attach_verified', 'detach_verified', 'home_verified',
    'destination_verified', 'final_collision_valid', 'baseline_acm_restored', 'shutdown_clean'])
def test_execution_rejects_missing_state_proof(key):
    result, statuses = executed_result()
    del result[key]
    with pytest.raises(RuntimeError):
        acceptance.check_execution_result(result, statuses)


def test_execution_rejects_lost_bottle_duplicate_cup_and_non_mock_hardware():
    result, statuses = executed_result()
    for world in [['runtime::sample-cup'], ['runtime::sample-cup'] * 2 + ['runtime::sample-bottle']]:
        bad = copy.deepcopy(result)
        bad['final_planning_scene']['world_ids'] = world
        with pytest.raises(RuntimeError, match='target/distractor'):
            acceptance.check_execution_result(bad, statuses)
    result['fake_hardware_guard']['hardware_classes'].append('custom/Hardware')
    with pytest.raises(RuntimeError, match='mock hardware'):
        acceptance.check_execution_result(result, statuses)
