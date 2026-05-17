from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def test_camera_frustum_preview_assets_present():
    src=(ROOT/'workcell_builder/workcell_builder/gui/placed_object_preview_writer.cpp').read_text()
    script=(ROOT/'scripts/workcell_builder_camera_frustum_preview_node.py').read_text()
    for n in ['camera_frustum_preview.launch.py','camera_01_link','camera_01_color_optical_frame','horizontal_fov_deg','vertical_fov_deg','near_m','far_m']:
        assert n in src
    banned=['realsense2_camera','epd','controller_manager','move_group','FollowJointTrajectory']
    joined=(src+script).lower()
    for b in banned:
        assert b.lower() not in joined
