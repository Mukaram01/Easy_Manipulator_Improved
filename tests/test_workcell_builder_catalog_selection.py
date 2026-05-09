import json
from scripts.workcell_builder_gui_workflow import generate_canonical_files, register_custom_stl_asset, validate_manual_cell_state


def test_builder_export_preserves_selected_fields(tmp_path) -> None:
    state = {"selected": {"robot": "ur5", "tool": "robotiq_2f", "camera": "realsense_d435i", "task": "pick_place"}, "current_cell_assets": [], "fake_hardware_default": True}
    out = tmp_path / "export"
    generate_canonical_files(state, out)
    payload = json.loads((out / "selected_assets.json").read_text(encoding="utf-8"))
    assert payload["catalog_selection"]["robot"] == "ur5"
    assert payload["catalog_selection"]["end_effector"] == "robotiq_2f"
    assert payload["catalog_selection"]["sensor"] == "realsense_d435i"
    assert payload["catalog_selection"]["task"] == "pick_place"


def test_custom_stl_saved_with_pose_and_scale(tmp_path) -> None:
    stl = tmp_path / "fixture.stl"
    stl.write_text("solid f\nendsolid f\n", encoding="utf-8")
    state = {"current_cell_assets": [], "selected": {}}
    copied = register_custom_stl_asset(state, str(stl), tmp_path / "scene_assets", xyz=[1,2,3], rpy=[0.1,0.2,0.3], scale=[1,1,1])
    assert copied["copied_asset_path"].endswith("fixture.stl")
    assert copied["pose"]["x"] == 1
    assert copied["scale"] == [1,1,1]


def test_placeholder_combo_warn_preview_only() -> None:
    state = {"selected": {"robot": "generic_delta_placeholder", "tool": "suction_basic"}, "current_cell_assets": [], "fake_hardware_default": True}
    val = validate_manual_cell_state(state)
    codes = {i["code"] for i in val["issues"]}
    assert "placeholder_robot" in codes
    assert "preview_combo" in codes
