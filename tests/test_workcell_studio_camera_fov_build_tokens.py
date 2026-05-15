from pathlib import Path

CPP_MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
CPP_PREVIEW = Path('workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')


def test_scene_preview_widget_includes_qvector3d_and_uses_it():
    assert '#include <QVector3D>' in CPP_PREVIEW
    assert 'QVector3D' in CPP_PREVIEW


def test_qt5_compatible_polygon_construction_and_selection_regression_guards():
    assert 'QPolygonF{' not in CPP_PREVIEW
    assert 'select_preview_item(item->' not in CPP_MAIN


def test_no_live_perception_or_motion_runtime_tokens_introduced():
    forbidden = [
        'realsense2_camera',
        'easy_perception_deployment',
        'trajectory_msgs',
        'FollowJointTrajectory',
        'publish(',
        'move_group',
        'moveit::',
    ]
    for token in forbidden:
        assert token not in CPP_MAIN
