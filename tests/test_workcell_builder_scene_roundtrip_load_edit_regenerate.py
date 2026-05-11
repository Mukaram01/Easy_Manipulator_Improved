from pathlib import Path


def _txt(path: str) -> str:
    return Path(path).read_text(encoding="utf-8")


def test_roundtrip_helper_exists_and_wired_into_cmake():
    assert Path("workcell_builder/workcell_builder/include/workcell_scene_roundtrip.hpp").exists()
    assert Path("workcell_builder/workcell_builder/src_workcell_scene_roundtrip.cpp").exists()
    assert "src_workcell_scene_roundtrip.cpp" in _txt("workcell_builder/workcell_builder/CMakeLists.txt")


def test_roundtrip_functions_and_ui_strings_exist():
    hpp = _txt("workcell_builder/workcell_builder/include/workcell_scene_roundtrip.hpp")
    cpp = _txt("workcell_builder/workcell_builder/src_workcell_scene_roundtrip.cpp")
    scene_cpp = _txt("workcell_builder/workcell_builder/gui/scene_select.cpp")
    for token in [
        "load_workcell_scene_v1_from_file",
        "populate_builder_state_from_scene",
        "extract_builder_state_to_scene",
    ]:
        assert token in hpp and token in cpp
    for ui in ["Open Existing Scene", "Reload Scene From YAML", "Scene Round-trip Status", "Regenerate Existing Scene"]:
        assert ui in scene_cpp


def test_roundtrip_metadata_markers_and_safety_defaults_present():
    blob = _txt("workcell_builder/workcell_builder/src_workcell_scene_roundtrip.cpp")
    for marker in [
        "compatibility_status",
        "placed_objects",
        "camera",
        "task/grasp recipe metadata",
        "workspace bounds/zones",
        "meshes/generated_objects/",
        "custom_meshes",
        "fake_hardware_first",
        "real_hardware_enabled",
        "runtime_execution_enabled",
    ]:
        assert marker in blob
    assert "fake_hardware_first = true" in blob
    assert "real_hardware_enabled = false" in blob
    assert "runtime_execution_enabled = false" in blob


def test_validation_dashboard_and_forbidden_additions():
    dashboard = _txt("workcell_builder/workcell_builder/src_validation_dashboard_model.cpp")
    scene_cpp = _txt("workcell_builder/workcell_builder/gui/scene_select.cpp")
    merged = (dashboard + scene_cpp + _txt("workcell_builder/workcell_builder/src_workcell_scene_roundtrip.cpp")).lower()
    assert "loaded scene" in scene_cpp.lower() or "loaded from workcell_scene/v1" in scene_cpp.lower()
    for forbidden in ["pyyaml", "import yaml", "getmotionplan", "execute_trajectory", "followjointtrajectory"]:
        assert forbidden not in merged


def test_curated_asset_tokens_present():
    txt = Path('workcell_builder/workcell_builder/config/asset_profiles/environment_assets.json').read_text(encoding='utf-8')
    assert "table_small" in txt and "pick_box" in txt


def test_portable_bundle_markers_present():
    blob = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for m in ['Export Scene Bundle','Import Scene Bundle','Portable Scene Bundle','Bundle Validation Status','Imported Scene Ready','Exported Scene Archive']:
        assert m in blob
