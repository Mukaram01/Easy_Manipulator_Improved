from scripts import workcell_builder_gui_workflow as wf


def _base(tool: str, strategy: str):
    s={"selected":{"robot":"ur5","tool":tool,"support_surface":"table","pick_area":"pick_area_1","place_target":"bin_1","grasp_strategy":strategy},"fake_hardware_default":True}
    c=wf.ensure_task_grasp_config(s)
    c["task"]["pick"]["source_ref"]="pick_area_1"
    c["task"]["place"]["target_ref"]="bin_1"
    c["grasp"]["strategy"]=strategy
    return s


def test_robotiq_finger_top_validates():
    assert not any(i["code"]=="GRASP_INCOMPATIBLE" for i in wf.validate_manual_cell_state(_base("robotiq_2f","finger_top"))["issues"])


def test_suction_basic_suction_top_validates():
    assert not any(i["code"]=="GRASP_INCOMPATIBLE" for i in wf.validate_manual_cell_state(_base("suction_basic","suction_top"))["issues"])


def test_incompatible_grasp_warns():
    assert any(i["code"]=="GRASP_INCOMPATIBLE" for i in wf.validate_manual_cell_state(_base("robotiq_2f","suction_top"))["issues"])
    assert any(i["code"]=="GRASP_INCOMPATIBLE" for i in wf.validate_manual_cell_state(_base("suction_basic","finger_top"))["issues"])
