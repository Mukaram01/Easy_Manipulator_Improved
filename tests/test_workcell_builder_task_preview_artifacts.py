import json
from scripts import workcell_builder_gui_workflow as wf


def test_visual_composer_includes_pick_place_arrows(tmp_path):
    s={"selected":{"robot":"ur5","tool":"robotiq_2f"},"fake_hardware_default":True}
    wf.add_asset_to_cell(s,"pick_area_1","bin",role="pick_object",name="Pick")
    wf.add_asset_to_cell(s,"bin_1","bin",role="place_target",name="Place")
    c=wf.ensure_task_grasp_config(s)
    c["task"]["pick"]["source_ref"]="pick_area_1"
    c["task"]["place"]["target_ref"]="bin_1"
    wf.export_layout_preview(s,tmp_path)
    markers=json.loads((tmp_path/"preview_markers.json").read_text())
    assert markers["task_flow"]["arrow"]
    assert "Task:" in (tmp_path/"static_preview.svg").read_text()
