import json
from scripts import workcell_builder_gui_workflow as wf

def test_export_includes_current_cell_assets(tmp_path):
    state={"selected":{},"current_cell_assets":[]}
    wf.add_asset_to_cell(state,"ur5","robot",role="robot")
    wf.add_asset_to_cell(state,"workbench","table",role="support_surface")
    wf.add_asset_to_cell(state,"conv","conveyor",role="conveyor")
    out=wf.generate_canonical_files(state,tmp_path)
    payload=json.loads((tmp_path/"selected_assets.json").read_text())
    assert out["ok"]
    assert len(payload["current_cell_assets"])==3
    assert "placed_assets" in (tmp_path/"environment_layout.yaml").read_text()
    assert wf.validate_manual_cell_state(state)["status"] in {"WARN","OK","FAIL"}
