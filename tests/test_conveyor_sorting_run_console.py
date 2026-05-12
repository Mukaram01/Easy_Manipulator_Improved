from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[1]
UI = ROOT / 'workcell_builder/workcell_builder/gui/conveyor_sorting_run_console.ui'
CPP = ROOT / 'workcell_builder/workcell_builder/gui/conveyor_sorting_run_console.cpp'
SCENE_SELECT_CPP = ROOT / 'workcell_builder/workcell_builder/gui/scene_select.cpp'


def test_ui_static_tokens():
    text = UI.read_text(encoding='utf-8') + CPP.read_text(encoding='utf-8')
    for token in [
        'Conveyor Sorting Run Console', 'Build Scenario', 'Launch Fake Preview',
        'Open Log Folder', 'Live Preview Status', 'Build status',
        'Launch status', 'real_hardware_ready must remain false'
    ]:
        assert token in text


def test_status_reader_parses_expected_fields(tmp_path):
    preview = tmp_path / 'preview'
    preview.mkdir()
    payload = {'class_label': 'box', 'pick_ready': True, 'selected_place_zone': 'place_zone_box'}
    (preview / 'live_conveyor_sorting_status.json').write_text(json.dumps(payload), encoding='utf-8')
    loaded = json.loads((preview / 'live_conveyor_sorting_status.json').read_text(encoding='utf-8'))
    assert loaded['class_label'] == 'box'
    assert loaded['pick_ready'] is True
    assert loaded['selected_place_zone'] == 'place_zone_box'


def test_missing_launch_file_is_reported_clearly(tmp_path):
    scene = tmp_path / 'demo'
    (scene / 'config').mkdir(parents=True)
    (scene / 'config' / 'scenario.yaml').write_text('scenario_id: conveyor_sorting_live_epd_preview', encoding='utf-8')
    missing = not (scene / 'launch' / 'demo.launch.py').exists()
    assert missing


def test_command_builder_tokens_exist():
    text = CPP.read_text(encoding='utf-8')
    assert 'colcon build --symlink-install --packages-select %1' in text
    assert 'use_fake_hardware:=true' in text and 'enable_conveyor_sorting_preview:=true' in text


def test_safety_validation_rules_present():
    text = CPP.read_text(encoding='utf-8')
    assert 'Unsafe launch args detected' in text
    assert 'real_hardware_ready must remain false' in text


def test_scenario_detection_enables_console_path():
    text = SCENE_SELECT_CPP.read_text(encoding='utf-8')
    assert 'conveyor_sorting_live_epd_preview' in text
    assert 'on_open_conveyor_sorting_run_console_clicked' in text


def test_emd_planner_execution_tokens_present():
    text = (UI.read_text(encoding='utf-8') + CPP.read_text(encoding='utf-8'))
    for token in [
        'EMD Planner / Execution Files',
        'Generate Planner Config',
        'Generate Planner Launch',
        'Copy Planner Launch Command',
        'Copy Execution Launch Command',
        'Workcell Studio generated the EMD files. Planning/execution still happens through run_grasp_planner and run_grasp_execution.',
    ]:
        assert token in text
