from pathlib import Path


def read(p):
    return Path(p).read_text(encoding="utf-8")


def test_asset_picker_dialog_exists():
    assert Path("workcell_builder/workcell_builder/include/asset_picker_dialog.hpp").exists()
    assert Path("workcell_builder/workcell_builder/gui/asset_picker_dialog.cpp").exists()


def test_robot_picker_wiring_and_defaults():
    txt = read("workcell_builder/workcell_builder/gui/addrobot.cpp")
    assert "Select Robot Asset" in txt
    assert "discover_workcell_assets" in txt
    assert 'findText("base_link")' in txt
    assert 'findText("ee_link")' in txt


def test_end_effector_picker_wiring_and_defaults():
    txt = read("workcell_builder/workcell_builder/gui/addendeffector.cpp")
    assert "Select End Effector Asset" in txt
    assert "discover_workcell_assets" in txt
    assert 'setCurrentText("finger")' in txt
    assert 'setCurrentText("suction")' in txt
    assert "Unknown end-effector type" in txt


def test_object_stl_picker_and_warnings_and_conveyor_text():
    txt = read("workcell_builder/workcell_builder/gui/addobject.cpp")
    assert "Select Existing STL" in txt
    assert "QFileDialog::getOpenFileName" in txt
    assert "External STL path is absolute and may not work on another machine." in txt
    assert "visual/metadata only — no conveyor physics or runtime control" in txt


def test_fake_hardware_guidance_and_no_unknown_spam():
    txt = read("workcell_builder/workcell_builder/gui/scene_select.cpp")
    assert "demo.launch.py use_fake_hardware:=true" in txt
    assert "unknown_description" not in txt
    assert "unknown_moveit_config" not in txt
