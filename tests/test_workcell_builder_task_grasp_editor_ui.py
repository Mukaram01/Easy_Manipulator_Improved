from pathlib import Path

UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')

def test_task_grasp_controls_exist():
    for t in [
        'pick_zone','conveyor_pick_zone','class_route_target','inspection_preview','machine_tending_preview',
        'Use Selected Item as Pick Source','Use Selected Item as Place Target','Use Selected Zone as Pick Zone','Use Selected Zone as Place Zone',
        'Approach Axis','Retreat Axis','Place Clearance (m)','Allowed Yaw/Roll (deg)','TCP Offset XYZ/RPY','Reset to Tool Defaults'
    ]:
        assert t in UI
