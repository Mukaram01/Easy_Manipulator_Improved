from pathlib import Path

def test_preview_handlers_and_qprocess_wiring_present():
    text = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    for token in [
        'run_preview_build', 'run_fake_hardware_preview', 'stop_preview_process',
        'readyReadStandardOutput', 'readyReadStandardError', 'QProcess::finished',
        'terminate()', 'kill()', 'Fake hardware only. No real hardware. No runtime execution.',
    ]:
        assert token in text
