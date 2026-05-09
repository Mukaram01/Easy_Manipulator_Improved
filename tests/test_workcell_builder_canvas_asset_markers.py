from scripts import workcell_builder_gui_workflow as wf


def test_inspector_xy_updates_reflected_in_canvas_model_and_preview_badge():
    state={"selected":{},"current_cell_assets":[]}
    wf.add_asset_to_cell(state,"conv_preview","conveyor",role="conveyor",name="Conveyor")
    asset=state["current_cell_assets"][0]
    asset["pose"]["x"]=1.2
    asset["pose"]["y"]=-0.3
    model=wf.build_visual_layout_canvas_model(state)
    marker=model["markers"][0]
    assert marker["x"]==1.2 and marker["y"]==-0.3
    assert marker["warning_badge"] is True


def test_canvas_move_updates_asset_pose_for_export():
    state={"selected":{},"current_cell_assets":[]}
    wf.add_asset_to_cell(state,"cube_small","object",role="pick_object")
    wf.update_asset_xy_from_canvas_move(state,"cube_small",0.9,0.4)
    assert state["current_cell_assets"][0]["pose"]["x"]==0.9
    assert state["current_cell_assets"][0]["pose"]["y"]==0.4
