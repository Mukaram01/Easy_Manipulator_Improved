from pathlib import Path

CPP_MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
CPP_PREVIEW = Path('workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
H_PREVIEW = Path('workcell_builder/workcell_builder/gui/scene_preview_widget.h').read_text(encoding='utf-8')


def test_camera_overlay_model_tokens_exist():
    for token in [
        'CameraOverlayModel', 'camera_id', 'display_name', 'frame_id',
        'horizontal_fov_deg', 'vertical_fov_deg', 'range_min_m', 'range_max_m',
        'metadata_source', 'status', 'warnings'
    ]:
        assert token in H_PREVIEW


def test_camera_fov_pick_coverage_and_epd_tokens_exist():
    for token in [
        'Camera FOV', 'Pick Coverage', 'EPD Detections', 'Detection Labels',
        'pick zone outside camera FOV', 'camera range too short', 'camera frame unknown',
        'no camera item found', 'no pick source found'
    ]:
        assert token in CPP_MAIN or token in CPP_PREVIEW


def test_perception_status_panel_tokens_exist():
    for token in [
        'Perception Status', 'Camera:', 'Frame:', 'FOV:', 'Range:',
        'Pick coverage:', 'Detection count:', 'No EPD detection snapshot loaded',
        'Open Perception Metadata', 'Open EPD Pipeline Docs', 'Refresh Snapshot'
    ]:
        assert token in CPP_MAIN


def test_regression_and_safety_tokens():
    assert '2D Layout' in CPP_PREVIEW
    assert 'QPolygonF{' not in CPP_PREVIEW
    assert 'select_preview_item(item->' not in CPP_MAIN
    forbidden = ['realsense2_camera', 'easy_perception_deployment', 'trajectory_msgs', 'FollowJointTrajectory', 'publish(']
    for token in forbidden:
        assert token not in CPP_MAIN
