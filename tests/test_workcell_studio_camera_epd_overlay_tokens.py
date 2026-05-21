from pathlib import Path

CPP_MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
CPP_PREVIEW = Path('workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
H_PREVIEW = Path('workcell_builder/workcell_builder/gui/scene_preview_widget.h').read_text(encoding='utf-8')
CPP_VIEWPORT = Path('workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


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
        'Pick coverage:', 'Detection count:', 'Perception preview unavailable until mode/config is selected.',
        'Open Perception Metadata', 'Open EPD Pipeline Docs', 'Refresh Snapshot'
    ]:
        assert token in CPP_MAIN


def test_regression_and_safety_tokens():
    assert '2D Layout' in CPP_PREVIEW
    assert 'QPolygonF{' not in CPP_PREVIEW
    assert 'select_preview_item(item->' in CPP_MAIN
    forbidden = ['realsense2_camera', 'easy_perception_deployment', 'trajectory_msgs', 'FollowJointTrajectory', 'publish(']
    for token in forbidden:
        assert token not in CPP_MAIN


def test_scene3d_contract_tokens_for_layer_hierarchy_are_current():
    for token in [
        'source_layer', 'active_visual_source', 'editable_layout',
        'primitive_fallback', 'mesh_preview', 'generated_urdf_visual'
    ]:
        assert token in CPP_MAIN or token in CPP_PREVIEW or token in CPP_VIEWPORT


def test_scene3d_camera_and_epd_overlay_render_tokens_exist():
    for token in [
        'draw_camera_body_with_frustum', 'show_camera_fov', 'show_pick_coverage',
        'show_epd_detections', 'show_detection_labels', 'clean_label_from_item',
        'horizontal_fov_deg', 'vertical_fov_deg', 'range_min_m', 'range_max_m'
    ]:
        assert token in CPP_VIEWPORT or token in CPP_PREVIEW or token in H_PREVIEW
