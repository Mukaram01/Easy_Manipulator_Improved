from scripts import workcell_builder_gui_workflow as wf


def _state():
    s={"scene_name":"demo","fake_hardware_default":True,"selected":{"robot":"ur5","tool":"robotiq_2f","camera":"d435","task":"pick_place","support_surface":"table","pick_area":"bin_a","place_target":"bin_b","grasp_strategy":"top"}}
    wf.add_asset_to_cell(s,"ur5","robot",role="robot_base",name="UR5")
    wf.add_asset_to_cell(s,"table_main","table",role="support_surface",name="Table")
    wf.add_asset_to_cell(s,"bin_a","bin",role="pick_object",name="Bin A")
    return s


def test_pose_edit_updates_exported_metadata(tmp_path):
    state=_state()
    wf.edit_asset_pose(state,1,x=1.2,y=-0.4,yaw=0.5,scale=[1,2,3])
    wf.generate_canonical_files(state,tmp_path)
    text=(tmp_path/"environment_layout.yaml").read_text()
    assert '"x": 1.2' not in text
    assert '"xyz": [' in text and '1.2' in text and '-0.4' in text
    assert '"scale": [' in text and '3.0' in text


def test_add_remove_duplicate_updates_export_metadata(tmp_path):
    state=_state()
    wf.add_asset_to_cell(state,"conv_preview","conveyor",role="conveyor",name="Conveyor")
    wf.duplicate_selected_asset(state,3)
    wf.remove_selected_asset(state,3)
    wf.generate_canonical_files(state,tmp_path)
    exported=(tmp_path/"selected_assets.json").read_text()
    assert "Conveyor_copy" in exported


def test_existing_scene_load_populates_composer_model(tmp_path):
    scene=tmp_path/"ur5_2f_test"
    gen=scene/"generated"
    gen.mkdir(parents=True)
    (gen/"environment_layout.yaml").write_text('{"assets":[{"id":"ur5","type":"robot","xyz":[0,0,0],"rpy":[0,0,0]},{"id":"table","type":"table","xyz":[0.5,0,0],"rpy":[0,0,0]}]}',encoding="utf-8")
    (gen/"selected_assets.json").write_text('{"selected":{"robot":"ur5","tool":"robotiq_2f"}}',encoding="utf-8")
    state=wf.load_existing_scene_into_state(scene,{})
    assert state["selected"]["robot"]=="ur5"
    assert len(state["canvas_model"]["markers"])==2


def test_placeholder_robot_status_badges_preview_only_runtime_unsupported():
    state=_state()
    state["selected"]["robot"]="generic_placeholder"
    panel=wf.build_readiness_status_panel(state)
    assert panel["badges"]["PREVIEW_ONLY"] is True
    assert panel["badges"]["RUNTIME_UNSUPPORTED"] is True


def test_fake_hardware_default_in_preview_command():
    state=_state()
    plan=wf.build_preview_launch_plan(state,"my_scene")
    assert "use_fake_hardware:=true" in plan["command"]
