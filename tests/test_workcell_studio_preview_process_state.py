from pathlib import Path

def test_preview_state_tokens_present():
    text = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    for token in [
        'IDLE', 'BUILD_RUNNING', 'BUILD_PASSED', 'BUILD_FAILED',
        'PREVIEW_RUNNING', 'PREVIEW_STOPPING', 'PREVIEW_STOPPED', 'PREVIEW_FAILED', 'PREVIEW_EXITED'
    ]:
        assert token in text
