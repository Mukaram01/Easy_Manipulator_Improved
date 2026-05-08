from scripts import workcell_builder_gui_workflow as wf

def test_add_core_assets_to_scene_roles():
    state={"selected":{},"current_cell_assets":[]}
    wf.add_asset_to_cell(state,"ur5","robot",role="robot")
    wf.add_asset_to_cell(state,"robotiq_2f","gripper",role="end_effector")
    wf.add_asset_to_cell(state,"workbench","table",role="support_surface")
    wf.add_asset_to_cell(state,"cube_small","object",role="pick_object")
    wf.add_asset_to_cell(state,"small_bin","bin",role="place_target")
    roles=[a["role"] for a in state["current_cell_assets"]]
    assert "robot_base" in roles
    assert "end_effector" in roles
    assert "support_surface" in roles
    assert "pick_object" in roles
    assert "place_target" in roles
