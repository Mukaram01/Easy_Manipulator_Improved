from pathlib import Path
REPO_ROOT = Path(__file__).resolve().parents[1]

def test_marker_publish_launch_args_present():
    txt=(REPO_ROOT/'workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py').read_text(encoding='utf-8')
    assert 'publish_workcell_markers' in txt
    assert 'show_task_flow' in txt
    assert 'show_grasp_markers' in txt
    marker_script=(REPO_ROOT/'workcell_builder/workcell_builder/templates/ros2/humble/launch/workcell_visual_scene_publisher.py').read_text(encoding='utf-8')
    assert '/workcell_markers' in marker_script
