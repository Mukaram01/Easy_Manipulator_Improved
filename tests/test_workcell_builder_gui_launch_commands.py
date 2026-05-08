from scripts import workcell_builder_gui_workflow as wf


def test_fake_hardware_command_generated_for_supported_cells():
    r=wf.copy_fake_hardware_launch_command({"preview_only_assets":False})
    assert r["ok"]
    assert "use_fake_hardware:=true" in r["message"]
    assert "use_fake_hardware:=false" not in r["message"]


def test_preview_only_cell_has_no_runtime_command():
    r=wf.copy_fake_hardware_launch_command({"preview_only_assets":True})
    assert not r["ok"]
    assert "preview-only" in r["message"]
