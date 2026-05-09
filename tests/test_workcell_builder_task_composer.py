import json
from scripts import workcell_builder_gui_workflow as wf


def _state():
    s={"selected":{"robot":"ur5","tool":"robotiq_2f","support_surface":"table","pick_area":"pick_area_1","place_target":"bin_1","grasp_strategy":"finger_top"},"fake_hardware_default":True}
    wf.add_asset_to_cell(s,"pick_area_1","bin",role="pick_object")
    wf.add_asset_to_cell(s,"bin_1","bin",role="place_target")
    c=wf.ensure_task_grasp_config(s)
    c["task"]["pick"]["source_ref"]="pick_area_1"
    c["task"]["place"]["target_ref"]="bin_1"
    c["grasp"]["strategy"]="finger_top"
    return s


def test_task_composer_default_pick_place_model():
    cfg=wf.default_task_grasp_config()
    assert cfg["task"]["template"]=="pick_place"


def test_task_composer_exports_task_metadata(tmp_path):
    s=_state()
    wf.generate_canonical_files(s,tmp_path)
    task=json.loads((tmp_path/"task_recipe.yaml").read_text())
    assert task["task"]["pick"]["source_ref"]=="pick_area_1"


def test_missing_pick_or_place_is_task_incomplete():
    s=_state()
    s["task_grasp_composer"]["task"]["pick"]["source_ref"]=""
    v=wf.validate_manual_cell_state(s)
    assert any(i["code"]=="TASK_INCOMPLETE" for i in v["issues"])
