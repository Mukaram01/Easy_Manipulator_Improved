from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UI = ROOT / "workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.ui"
CPP = ROOT / "workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.cpp"


def test_wizard_ui_exists_and_has_pages_and_buttons():
    text = UI.read_text(encoding="utf-8")
    for page in ["Overview", "Hardware", "Layout", "Work Zones", "Routing", "EPD Preview", "Generate"]:
        assert page in text
    for label in ["Use Recommended Layout", "Generate Scenario", "Copy Build Command", "Copy Launch Command", "Copy Sample EPD Command", "Export Scenario Bundle"]:
        assert label in text


def test_wizard_implements_generation_and_safety_defaults():
    text = CPP.read_text(encoding="utf-8")
    for required in ["environment.yaml", "scenario.yaml", "preview", "real_hardware_ready: false", "moveit_execute_called: false", "gripper_command_sent: false", "conveyor_command_sent: false"]:
        assert required in text
