from scripts import workcell_builder_gui_workflow as wf


def test_canvas_model_renders_core_markers_and_reach_helper():
    state={"selected":{},"current_cell_assets":[]}
    wf.add_asset_to_cell(state,"ur5","robot",role="robot")
    wf.add_asset_to_cell(state,"workbench","table",role="support_surface",name="Table")
    wf.add_asset_to_cell(state,"bin_a","bin",role="place_target",name="Bin A")
    model=wf.build_visual_layout_canvas_model(state)
    marker_types={m["marker_type"] for m in model["markers"]}
    assert {"robot_base","table","bin"}.issubset(marker_types)
    assert model["reach_helpers"]
