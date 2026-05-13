from pathlib import Path


def _read(path: str) -> str:
    return Path(path).read_text(encoding='utf-8')


def test_home_scene_template_and_existing_scene_groups_exist():
    text = _read('workcell_builder/workcell_builder/gui/scene_select.ui')
    assert 'name="scenario_templates_group"' in text
    assert 'Scenario Templates' in text
    assert 'Existing Scenes' in text


def test_addscene_required_groups_and_scroll_layout():
    text = _read('workcell_builder/workcell_builder/gui/addscene.ui')
    for group in ['Scene', 'Robot', 'End Effector', 'Objects / Environment', 'Work Zones / Metadata Preview']:
        assert group in text
    assert 'QScrollArea' in text
    assert 'widgetResizable' in text and '<bool>true</bool>' in text
    assert 'workzoneButtons' in text and 'QGridLayout' in text


def test_wizard_help_text_and_mode_behavior_tokens():
    ui = _read('workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.ui')
    cpp = _read('workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.cpp')
    for token in ['Overview', 'Hardware', 'Layout', 'Work Zones / Pick Areas', 'Routing', 'EPD Preview', 'Generate']:
        assert token in ui
    assert 'Sample demo feed' in ui and 'Real EPD connector' in ui
    assert 'onEpdModeChanged' in cpp


def test_run_console_sections_and_safety_state():
    ui = _read('workcell_builder/workcell_builder/gui/conveyor_sorting_run_console.ui')
    cpp = _read('workcell_builder/workcell_builder/gui/conveyor_sorting_run_console.cpp')
    for section in ['Live Preview Status', 'EMD Grasp Planner Config', 'Process Log']:
        assert section in ui
    assert 'real_hardware_ready: false' in cpp
    assert 'safetyChecks' in cpp


def test_no_stale_autoconnect_and_no_white_on_white_and_no_full_width_delete():
    addscene_cpp = _read('workcell_builder/workcell_builder/gui/addscene.cpp')
    main_cpp = _read('workcell_builder/workcell_builder/gui/main.cpp')
    utils_cpp = _read('workcell_builder/workcell_builder/gui/workcell_builder_ui_utils.cpp')
    assert 'connectSlotsByName' not in addscene_cpp
    assert 'background-color: white;\"\n    \"  color: white;' not in main_cpp
    assert "destructive_action" in utils_cpp
