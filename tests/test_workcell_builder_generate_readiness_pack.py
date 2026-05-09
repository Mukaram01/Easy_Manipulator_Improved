from scripts import workcell_builder_gui_workflow as wf


def test_generate_readiness_pack_creates_expected_files(tmp_path):
    state={"selected":{"robot":"ur5","tool":"robotiq_2f","support_surface":"table","pick_area":"bin","place_target":"tray","grasp_strategy":"top"},"fake_hardware_default":True}
    out=tmp_path/"pack"
    res=wf.generate_studio_pack(state,out)
    assert res["ok"]
    for name in ["cell_definition.yaml","environment_layout.yaml","scene_manifest.yaml","readiness_summary.md","environment_preview.svg","generated_launch_commands.md"]:
        assert (out/name).exists()


def test_placeholder_robot_is_preview_only(tmp_path):
    state={"selected":{"robot":"generic_placeholder","tool":"suction","support_surface":"table","pick_area":"bin","place_target":"tray","grasp_strategy":"top"},"preview_only_assets":True}
    out=tmp_path/"pack"
    res=wf.generate_studio_pack(state,out)
    assert res["runtime_classification"]=="preview_only"


def test_fake_hardware_default_preserved(tmp_path):
    state={"selected":{"robot":"ur5","tool":"single_suction","support_surface":"table","pick_area":"bin","place_target":"tray","grasp_strategy":"top"},"fake_hardware_default":True}
    out=tmp_path/"pack"
    wf.generate_canonical_files(state,out)
    summary=(out/"compatibility_report.json").read_text(encoding="utf-8")
    assert '"fake_hardware_default": true' in summary
