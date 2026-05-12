from pathlib import Path
import importlib.util
import yaml

ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / 'scripts' / 'workcell_builder_gui_workflow.py'
spec = importlib.util.spec_from_file_location('workflow', MODULE_PATH)
workflow = importlib.util.module_from_spec(spec)
spec.loader.exec_module(workflow)


def _sample_params(topic='/camera/points'):
    return {
        'grasp_output_service': '/grasp',
        'execution_in_progress_gate_enabled': False,
        'table_to_camera_height': 0.7,
        'grasp_planner_allow_target_fingertip_contact': True,
        'grasp_planner_allowed_target_touch_links': ['a'],
        'grasp_planner_allowed_target_touch_link_patterns': ['b'],
        'easy_perception_deployment': {'epd_enabled': True, 'epd_localization_topic': '/easy_perception_deployment/epd_localize_output'},
        'camera_parameters': {'camera_frame': 'camera_color_optical_frame', 'robot_base_frame': 'base_link', 'fx': 1.0, 'fy': 1.0, 'ppx': 0.0, 'ppy': 0.0},
        'point_cloud_params': {'point_cloud_topic': topic, 'passthrough_x_min': 0.1, 'passthrough_x_max': 1.0, 'passthrough_y_min': -1.0, 'passthrough_y_max': 1.0, 'passthrough_z_min': 0.0, 'passthrough_z_max': 2.0},
        'end_effectors': {'selected_end_effector': 'robotiq_2f'},
        'visualization_params': {'point_cloud_visualization': False},
    }


def test_generated_planner_config_and_launch(tmp_path):
    out = workflow.generate_emd_planner_files({}, 'Conveyor Sorting Live EPD Preview', '2f', _sample_params(), planner_dir=tmp_path)
    config_path = Path(out['config_path'])
    launch_path = Path(out['launch_path'])
    assert config_path.name == 'params_conveyor_sorting_live_epd_preview_2f.yaml'
    assert launch_path.name == 'grasp_planner_conveyor_sorting_live_epd_preview.launch.py'
    payload = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    assert 'grasp_planning_node' in payload and 'ros__parameters' in payload['grasp_planning_node']
    text = launch_path.read_text(encoding='utf-8')
    assert 'run_grasp_planner' in text and 'demo_node' in text
    assert config_path.name in text


def test_validation_and_commands():
    payload = {'grasp_planning_node': {'ros__parameters': _sample_params('')}}
    status = workflow.validate_generated_emd_planner_config(payload)
    assert status['status'] == 'ERROR'
    assert any('missing point_cloud_topic' in e for e in status['errors'])
    ok = workflow.generate_emd_planner_files({}, 'demo_scene', 'suction', _sample_params())
    assert ok['planner_command'] == 'ros2 launch run_grasp_planner grasp_planner_demo_scene.launch.py'
    assert ok['execution_command'] == 'ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=demo_scene'
    assert 'with_epd.launch.py' not in ok['planner_command']
    assert 'handoff_file' not in ok['execution_command']
