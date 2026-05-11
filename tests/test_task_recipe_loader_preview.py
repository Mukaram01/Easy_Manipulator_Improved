from pathlib import Path
import importlib.util

MODULE_PATH = Path('easy_manipulation_deployment/emd_demo_nodes/run_grasp_execution/run_grasp_execution/task_recipe.py')


def _load_module():
    spec = importlib.util.spec_from_file_location('task_recipe', MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


def _valid_recipe():
    return {
        'schema_version': 'workcell_task/v1',
        'task': {'type': 'pick_place', 'pick_source': 'bin_a', 'place_target': 'bin_b'},
        'grasp': {'strategy': 'finger_top', 'approach_distance_m': 0.1, 'retreat_distance_m': 0.1},
        'place': {'clearance_m': 0.02},
        'release': {'strategy': 'open_gripper'},
        'safety': {'fake_hardware_first': True, 'motion_command_sent': False, 'runtime_execution_enabled': False},
    }


def test_loader_module_and_functions_exist():
    assert MODULE_PATH.exists()
    mod = _load_module()
    for name in ['load_task_recipe', 'validate_task_recipe', 'normalize_task_recipe', 'build_offline_task_plan', 'write_task_plan_report', 'write_task_plan_markdown', 'write_task_plan_json']:
        assert hasattr(mod, name)


def test_schema_required_and_validation_blocks_invalid_values():
    mod = _load_module()
    r = _valid_recipe(); r['schema_version'] = 'bad'
    assert not mod.validate_task_recipe(r)['valid']
    r = _valid_recipe(); r['grasp']['approach_distance_m'] = -0.1
    assert not mod.validate_task_recipe(r)['valid']
    r = _valid_recipe(); del r['task']['pick_source']
    assert not mod.validate_task_recipe(r)['valid']
    r = _valid_recipe(); del r['task']['place_target']
    assert not mod.validate_task_recipe(r)['valid']


def test_safety_markers_required():
    mod = _load_module()
    r = _valid_recipe(); r['safety']['fake_hardware_first'] = False
    assert not mod.validate_task_recipe(r)['valid']
    r = _valid_recipe(); r['safety']['motion_command_sent'] = True
    assert not mod.validate_task_recipe(r)['valid']
    r = _valid_recipe(); r['safety']['runtime_execution_enabled'] = True
    assert not mod.validate_task_recipe(r)['valid']


def test_plan_order_and_safety_markers_present():
    mod = _load_module()
    plan = mod.build_offline_task_plan(_valid_recipe())
    assert [s['name'] for s in plan['steps']] == [
      'validate_recipe','resolve_pick_source','resolve_place_target','approach_pick','apply_grasp_strategy','retreat_from_pick','transfer_placeholder','approach_place','release_object','retreat_from_place','complete'
    ]
    txt = str(plan)
    for marker in ['OFFLINE_ONLY', 'NO_MOTION_COMMAND', 'NO_MOVEIT_PLAN', 'NO_REAL_HARDWARE']:
        assert marker in txt


def test_cli_and_launch_template_passive_and_safe_strings():
    cli = Path('scripts/preview_task_recipe.py').read_text(encoding='utf-8')
    assert 'WORKCELL_TASK_RECIPE_PREVIEW: PASS' in cli
    launch = Path('workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py').read_text(encoding='utf-8')
    assert 'task_recipe_path' in launch
    assert 'Task recipe loaded for offline preview' in launch
    lowered = (cli + launch).lower()
    assert 'moveit_msgs/srv/getmotionplan' not in lowered
    assert 'execute_trajectory' not in lowered
