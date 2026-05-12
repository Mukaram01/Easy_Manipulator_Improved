from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UI = ROOT / 'workcell_builder/workcell_builder/gui/conveyor_sorting_run_console.ui'
CPP = ROOT / 'workcell_builder/workcell_builder/gui/conveyor_sorting_run_console.cpp'


def test_ui_contains_one_click_controls():
    text = UI.read_text(encoding='utf-8') + CPP.read_text(encoding='utf-8')
    for token in [
        'Build Scenario', 'Launch Fake Preview', 'Stop Preview', 'Restart Preview', 'Clear Log',
        'Build status', 'Launch status', 'use_fake_hardware:=true'
    ]:
        assert token in text


def test_command_builder_demo_tokens():
    text = CPP.read_text(encoding='utf-8')
    assert 'colcon build --symlink-install --packages-select %1' in text
    assert 'source install/setup.bash' in text
    assert 'ros2 launch %1 demo.launch.py' in text
    assert 'use_fake_hardware:=true' in text
    assert 'launch_rviz:=true' in text
    assert 'enable_conveyor_sorting_preview:=true' in text
    assert 'publish_sample_detections:=true' in text


def test_safety_guard_tokens_present():
    text = CPP.read_text(encoding='utf-8')
    assert 'scenario_id: conveyor_sorting_live_epd_preview' in text
    assert 'Missing scenario.yaml' in text
    assert 'Wrong scenario_id' in text
    assert 'Launch command missing fake hardware flag' in text
    assert 'Unsafe launch args detected' in text
    assert 'real_hardware_ready must remain false' in text


def test_log_path_under_preview_logs():
    text = CPP.read_text(encoding='utf-8')
    assert 'preview" / "logs' in text
    assert 'build_' in text
    assert 'launch_' in text


def test_process_handling_static_tokens():
    text = CPP.read_text(encoding='utf-8') + (ROOT / 'workcell_builder/workcell_builder/gui/conveyor_sorting_run_console.h').read_text(encoding='utf-8')
    for token in ['QProcess', 'readyReadStandardOutput', 'readyReadStandardError', 'terminate()', 'kill()']:
        assert token in text


def test_status_refresh_and_malformed_warning_tokens():
    text = CPP.read_text(encoding='utf-8')
    assert 'live_conveyor_sorting_status.json' in text
    assert 'class_label:' in text
    assert 'selected_place_zone:' in text
    assert 'pick_ready:' in text
    assert 'time_to_pick_s:' in text
    assert 'Warning: malformed status JSON' in text
