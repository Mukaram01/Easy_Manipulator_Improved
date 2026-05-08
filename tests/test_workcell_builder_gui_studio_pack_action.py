from pathlib import Path
from scripts import workcell_builder_gui_workflow as wf


def test_studio_pack_exports_preview_and_readiness(tmp_path:Path):
    state={"selected":{"robot":"ur5","tool":"2f","support_surface":"table","pick_area":"bin","place_target":"tray","grasp_strategy":"top"}}
    out=tmp_path/"pack"
    res=wf.generate_studio_pack(state,out)
    assert res["ok"]
    for name in ["readiness_summary.md","readiness_summary.html","environment_preview.svg","environment_preview.html","generated_launch_commands.md"]:
        assert (out/name).exists()
