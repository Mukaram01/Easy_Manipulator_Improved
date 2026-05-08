from scripts import workcell_builder_gui_workflow as wf

def test_import_custom_stl_action_creates_asset_entry():
    state={"selected":{},"current_cell_assets":[]}
    added=wf.import_custom_stl(state,"/tmp/my_fixture.stl")
    assert added["category"]=="custom_stl"
    assert added["collision_mode"]=="visual_only"
    assert state["current_cell_assets"]
