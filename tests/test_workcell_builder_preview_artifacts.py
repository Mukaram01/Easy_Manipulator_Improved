import json
from scripts import workcell_builder_gui_workflow as wf


def test_static_preview_artifacts_generated(tmp_path):
    state={"selected":{"robot":"ur5","tool":"robotiq_2f","support_surface":"table","pick_area":"bin","place_target":"tray","grasp_strategy":"top"},"fake_hardware_default":True}
    wf.add_asset_to_cell(state,"ur5","robot",role="robot_base",name="UR5")
    stl=tmp_path/"fixture.stl"
    stl.write_text("solid s\nendsolid s\n")
    wf.register_custom_stl_asset(state, str(stl), tmp_path/"scene_assets")
    res=wf.export_layout_preview(state,tmp_path)
    assert res["ok"] is True
    assert (tmp_path/"static_preview.svg").exists()
    markers=json.loads((tmp_path/"preview_markers.json").read_text())
    assert any(m["marker_type"]=="robot_base" for m in markers["markers"])
    assert any(m.get("source_file") for m in markers["markers"])
