from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()

def test_inspector_pose_fields_and_apply_handler_present():
    for needle in ['inspector_x_', 'inspector_y_', 'inspector_z_', 'inspector_roll_', 'inspector_pitch_', 'inspector_yaw_', 'apply_inspector_pose_to_item']:
        assert needle in CPP
