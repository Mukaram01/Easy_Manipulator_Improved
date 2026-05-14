from pathlib import Path


def test_offline_smoke_buttons_and_tokens_exist():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text()
    for token in [
        'Run Offline Smoke Check', 'Open Smoke Report', 'Export Smoke Report', 'Copy Smoke Summary',
        'PASS', 'WARNINGS', 'BLOCKED', 'PREVIEW_ONLY'
    ]:
        assert token in ui
