from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENE_SELECT_CPP = ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp"
WIZARD_CPP = ROOT / "workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.cpp"
WIZARD_UI = ROOT / "workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.ui"


def test_scenario_button_opens_real_wizard():
    text = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    assert "ConveyorSortingScenarioWizard wizard" in text
    assert "wizard.exec()" in text


def test_routing_and_preview_defaults_present():
    text = WIZARD_CPP.read_text(encoding="utf-8") + WIZARD_UI.read_text(encoding="utf-8")
    for s in ["box", "place_zone_box", "bottle", "place_zone_bottle", "unknown", "reject_zone", "/workcell_studio/epd_detection_snapshot_json"]:
        assert s in text
