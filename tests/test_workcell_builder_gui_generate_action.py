from pathlib import Path
from scripts import workcell_builder_gui_workflow as wf


def test_generation_exports_canonical_files(tmp_path:Path):
    state={"selected":{"robot":"ur5","tool":"2f","support_surface":"table","pick_area":"bin","place_target":"tray","grasp_strategy":"top"}}
    out=tmp_path/"out"
    result=wf.generate_canonical_files(state,out)
    assert result["ok"]
    for name in ["cell_definition.yaml","environment_layout.yaml","task_recipe.yaml","selected_assets.json","compatibility_report.json","builder_export_summary.json"]:
        assert (out/name).exists()
    assert result["output_dir"]==str(out)
