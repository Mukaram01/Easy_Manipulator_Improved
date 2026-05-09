from scripts import workcell_builder_gui_workflow as wf


def test_export_layout_preview_generates_svg_and_html(tmp_path):
    state={"selected":{},"current_cell_assets":[]}
    wf.add_asset_to_cell(state,"ur5","robot",role="robot",name="UR5")
    wf.add_asset_to_cell(state,"bin_a","bin",role="place_target",name="Bin A")
    result=wf.export_layout_preview(state,tmp_path)
    assert result["ok"] is True
    assert (tmp_path/"layout_preview.svg").exists()
    assert (tmp_path/"layout_preview.html").exists()
