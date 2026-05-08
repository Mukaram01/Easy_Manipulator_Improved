from scripts import workcell_builder_gui_workflow as wf

def test_duplicate_and_remove_current_cell_asset():
    state={"selected":{},"current_cell_assets":[]}
    wf.add_asset_to_cell(state,"cube_small","object",role="pick_object",name="cube")
    dup=wf.duplicate_selected_asset(state,0)
    assert dup["name"].startswith("cube_copy")
    assert len(state["current_cell_assets"])==2
    wf.remove_selected_asset(state,0)
    assert len(state["current_cell_assets"])==1
