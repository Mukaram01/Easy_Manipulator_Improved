from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
GUI = ROOT / 'workcell_builder/workcell_builder/gui'


def _read(p):
    return (GUI / p).read_text()


def test_wizard_core_buttons_wired():
    cpp = _read('conveyor_sorting_scenario_wizard.cpp')
    for slot in [
        'onUseRecommendedLayout','onResetLayout','onValidateRouting','onAddZone','onRemoveZone',
        'onResetZones','onValidateZones','onAddRoute','onRemoveRoute','onResetDefaultRoutes',
        'onGenerateScenario','onGenerateFiles','onOpenRunConsole'
    ]:
        assert slot in cpp


def test_recommended_layout_changes_fields_not_only_status():
    cpp = _read('conveyor_sorting_scenario_wizard.cpp')
    body = re.search(r'void ConveyorSortingScenarioWizard::onUseRecommendedLayout\(\).*?\n\}', cpp, re.S).group(0)
    for field in ['robotPoseEdit','conveyorPoseEdit','cameraMountPoseEdit','cameraPoseEdit','binPoseEdit']:
        assert field in body


def test_epd_mode_handler_updates_dependent_fields():
    cpp = _read('conveyor_sorting_scenario_wizard.cpp')
    body = re.search(r'void ConveyorSortingScenarioWizard::onEpdModeChanged\(\).*?\n\}', cpp, re.S).group(0)
    for token in ['sampleCommand->setEnabled','epdTopicEdit->setText','epdDetailsLabel->setText']:
        assert token in body


def test_copy_commands_use_scene_name():
    cpp = _read('conveyor_sorting_scenario_wizard.cpp')
    assert 'sceneName()' in re.search(r'onCopyBuildCommand\(\).*?\n\}', cpp, re.S).group(0)
    assert 'sceneName()' in re.search(r'onCopyLaunchCommand\(\).*?\n\}', cpp, re.S).group(0)


def test_run_console_buttons_wired_or_disabled():
    cpp = _read('conveyor_sorting_run_console.cpp')
    for slot in ['onBuildScenario','onLaunchPreview','onStopPreview','onRestartPreview','onClearLog','onOpenLogFolder','onRefresh']:
        assert slot in cpp
