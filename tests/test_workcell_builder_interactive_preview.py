from pathlib import Path


def test_interactive_preview_generation_markers():
    cpp = Path('workcell_builder/workcell_builder/gui/placed_object_preview_writer.cpp').read_text(encoding='utf-8')
    for marker in [
        'interactive_preview.launch.py',
        'README_INTERACTIVE_PREVIEW.md',
        'workcell_builder_interactive_preview_node.py',
        'InteractiveMarkers',
        'safe_for_robot_motion: false',
    ]:
        assert marker in cpp or marker in Path('scripts/workcell_builder_interactive_preview_node.py').read_text(encoding='utf-8')


def test_interactive_preview_safety_constraints():
    txt = (
        Path('workcell_builder/workcell_builder/gui/placed_object_preview_writer.cpp').read_text(encoding='utf-8')
        + Path('scripts/workcell_builder_interactive_preview_node.py').read_text(encoding='utf-8')
    ).lower()
    for forbidden in ['controller_manager', 'move_group', 'execute_trajectory', 'followjointtrajectory', 'real hardware nodes']:
        assert forbidden not in txt


def test_safe_for_robot_motion_rejection_marker_present():
    importer = Path('workcell_builder/workcell_builder/gui/rviz_pose_feedback_importer.cpp').read_text(encoding='utf-8')
    assert 'safe_for_robot_motion' in importer
    assert 'Rejected feedback' in importer
