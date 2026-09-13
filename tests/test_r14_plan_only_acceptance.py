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
