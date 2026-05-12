from pathlib import Path
import importlib.util
import yaml
ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / 'scripts' / 'workcell_builder_gui_workflow.py'
spec = importlib.util.spec_from_file_location('workflow', MODULE_PATH)
workflow = importlib.util.module_from_spec(spec)
spec.loader.exec_module(workflow)


def _values(topic='/camera/custom/points'):
    return {
        'easy_perception_deployment': {'epd_enabled': True, 'epd_localization_topic': '/easy_perception_deployment/epd_localize_output', 'epd_tracking_enabled': False},
        'camera_parameters': {'point_cloud_topic': topic, 'camera_frame': 'camera_depth_optical_frame', 'robot_base_frame': 'base_link', 'fx': 1.0, 'fy': 1.0},
        'point_cloud_params': {'point_cloud_topic': topic, 'passthrough_x_min': 0.0, 'passthrough_x_max': 1.0, 'passthrough_y_min': -1.0, 'passthrough_y_max': 1.0, 'passthrough_z_min': 0.0, 'passthrough_z_max': 1.0, 'min_cluster_size': 1},
        'end_effectors': {'selected_end_effector': 'robotiq_2f'},
        'visualization_params': {'point_cloud_visualization': False},
    }

def test_defaults_load_from_params_file(tmp_path):
    params = tmp_path / 'params_2f.yaml'
    params.write_text('grasp_planning_node:\n  ros__parameters:\n    easy_perception_deployment: {}\n    camera_parameters: {}\n    point_cloud_params: {}\n    end_effectors: {}\n    visualization_params: {}\n', encoding='utf-8')
    defaults = workflow.load_emd_planner_defaults(params)
    for section in ['easy_perception_deployment','camera_parameters','point_cloud_params','end_effectors','visualization_params']:
        assert section in defaults

def test_generate_from_edited_values_and_launch(tmp_path):
    out = workflow.generate_emd_planner_files({}, 'conveyor_sorting_live_epd_preview', '2f', _values('/my/custom/topic'), planner_dir=tmp_path)
    cfg = Path(out['config_path'])
    launch = Path(out['launch_path'])
    payload = yaml.safe_load(cfg.read_text(encoding='utf-8'))
    assert payload['grasp_planning_node']['ros__parameters']['point_cloud_params']['point_cloud_topic'] == '/my/custom/topic'
    txt = launch.read_text(encoding='utf-8')
    assert cfg.name in txt
    assert 'run_grasp_planner' in txt and 'demo_node' in txt

def test_commands_and_no_legacy_defaults():
    out = workflow.generate_emd_planner_files({}, 'demo_scene', '2f', _values())
    assert out['planner_command'] == 'ros2 launch run_grasp_planner grasp_planner_demo_scene.launch.py'
    assert out['execution_command'] == 'ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=demo_scene'
    assert 'with_epd.launch.py' not in out['planner_command']
    assert 'handoff_file' not in out['execution_command']

def test_validation_and_no_overwrite(tmp_path):
    out1 = workflow.generate_emd_planner_files({}, 'demo_scene', '2f', _values(), planner_dir=tmp_path)
    cfg = Path(out1['config_path'])
    before = cfg.read_text(encoding='utf-8')
    out2 = workflow.generate_emd_planner_files({}, 'demo_scene', '2f', _values('/new/topic'), planner_dir=tmp_path, overwrite=False)
    assert cfg.read_text(encoding='utf-8') == before
    assert out2['existing_files']
    bad = {'grasp_planning_node': {'ros__parameters': _values('')}}
    bad['grasp_planning_node']['ros__parameters']['point_cloud_params']['passthrough_x_min'] = 2
    bad['grasp_planning_node']['ros__parameters']['point_cloud_params']['passthrough_x_max'] = 1
    status = workflow.validate_generated_emd_planner_config(bad)
    assert status['status'] == 'ERROR'
    good = workflow.validate_generated_emd_planner_config({'grasp_planning_node': {'ros__parameters': _values()}})
    assert good['status'] in {'OK','WARN'}
