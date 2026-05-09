from pathlib import Path
REPO_ROOT = Path(__file__).resolve().parents[1]

def test_demo_rviz_contains_marker_display():
    txt=(REPO_ROOT/'workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.rviz').read_text(encoding='utf-8')
    assert 'rviz_default_plugins/MarkerArray' in txt
    assert '/scene_name/workcell_markers' in txt
