from pathlib import Path

UI = Path('workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.ui').read_text()
SCENE_UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text()

def test_ui_structure_tokens():
    for token in [
        'Scenario Templates', 'Open Wizard: Conveyor Sorting - Live EPD Preview',
        'QComboBox" name="robotCombo', 'QComboBox" name="endEffectorCombo',
        'QComboBox" name="conveyorCombo', 'QComboBox" name="cameraCombo',
        'QComboBox" name="cameraMountCombo', 'useRecommendedLayoutButton',
        'resetLayoutButton', 'addZoneButton', 'removeZoneButton', 'resetZonesButton',
        'addRouteButton', 'removeRouteButton', 'resetRoutesButton', 'epdModeCombo',
        'generateScenarioButton', 'generateFilesButton', 'openRunConsoleButton'
    ]:
        assert token in (UI + SCENE_UI)
