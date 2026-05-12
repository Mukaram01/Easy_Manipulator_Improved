from pathlib import Path
import yaml

CFG = Path('easy_manipulation_deployment/emd_demo_nodes/run_grasp_planner/config')
CONFIGS = ['params.yaml','params_2f.yaml','params_3f.yaml','params_suction.yaml','params_airpick4.yaml']

def load(p):
    return yaml.safe_load(Path(p).read_text())['grasp_planning_node']['ros__parameters']

def test_configs_have_visualization_block_and_defaults_false():
    for name in CONFIGS:
        params = load(CFG / name)
        vis = params['visualization_params']
        assert vis['point_cloud_visualization'] is False
        for key in [
            'point_cloud_visualization_backend',
            'point_cloud_visualization_blocking',
            'point_cloud_visualization_timeout_ms',
            'point_cloud_visualization_save_debug_pcd',
            'point_cloud_visualization_debug_dir',
        ]:
            assert key in vis

def test_headless_guard_and_debug_artifact_logic_present():
    text = Path('easy_manipulation_deployment/emd_grasp_planner/src/grasp_scene.cpp').read_text()
    assert 'DISPLAY' in text and 'WAYLAND_DISPLAY' in text
    assert 'maybe_save_visualization_debug_artifacts' in text
    assert 'pcl::io::savePCDFileBinary' in text

def test_no_execution_calls_introduced_in_visualization_path():
    text = Path('easy_manipulation_deployment/emd_grasp_planner/src/grasp_scene.cpp').read_text()
    # Ensure visualization path does not call execution entry points.
    vis_block_idx = text.find('if (point_cloud_visualization)')
    assert vis_block_idx != -1
    block = text[vis_block_idx: vis_block_idx + 1800]
    assert 'send_to_execution' not in block
