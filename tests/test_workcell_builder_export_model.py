import json
from scripts import workcell_builder_gui_workflow as wf


def test_builder_export_model_contains_canonical_metadata(tmp_path):
    state={"scene_name":"demo_scene","selected":{"robot":"ur5","tool":"robotiq_2f","camera":"d435","task":"pick_place","support_surface":"table","pick_area":"bin_a","place_target":"bin_b","grasp_strategy":"top"},"fake_hardware_default":True}
    part=tmp_path/"part.stl"
    part.write_text("solid x\nendsolid x\n", encoding="utf-8")
    wf.register_custom_stl_asset(state, str(part), tmp_path/"assets", xyz=[1,2,3], rpy=[0.1,0.2,0.3], scale=[1.1,1.2,1.3])
    out=tmp_path/"generated"
    res=wf.generate_canonical_files(state,out)
    assert res["ok"]
    cell=json.loads((out/"cell_definition.yaml").read_text(encoding="utf-8"))
    env=json.loads((out/"environment_layout.yaml").read_text(encoding="utf-8"))
    assert cell["schema_version"]=="cell_definition/v1"
    assert env["schema_version"]=="environment_layout/v1"
    assert cell["robot"]["id"]=="ur5"
    assert cell["tool"]["id"]=="robotiq_2f"
    assert env["assets"][0]["scale"]==[1.1,1.2,1.3]
