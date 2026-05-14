from pathlib import Path
HDR = Path('workcell_builder/workcell_builder/include/workcell_studio_canvas_model.hpp').read_text(encoding='utf-8')

def test_pose_fields_and_metadata_present():
    for token in ['x{0.0}', 'y{0.0}', 'z{0.0}', 'roll{0.0}', 'pitch{0.0}', 'yaw{0.0}', 'locked{false}']:
        assert token in HDR
