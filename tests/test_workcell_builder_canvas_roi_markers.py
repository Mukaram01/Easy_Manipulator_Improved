from scripts import workcell_builder_gui_workflow as wf


def test_roi_box_appears_and_invalid_roi_warns():
    state={"selected":{},"current_cell_assets":[],"camera_pointcloud_roi":{
        "enabled":True,"frame":"world","x_min":0.5,"x_max":0.1,"y_min":-0.1,"y_max":0.2,"z_min":0.0,"z_max":0.3}}
    model=wf.build_visual_layout_canvas_model(state)
    assert model["roi"] is not None
    assert model["roi"]["valid"] is False
    assert "ROI invalid" in model["warnings"]
