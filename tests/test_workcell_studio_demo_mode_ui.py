from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def test_ui_tokens_present():
    text=(ROOT/'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    for token in [
        'Demo Mode','Run Demo Readiness','Run Acceptance','Run Offline Smoke Check',
        'Generate Preview Bundle','Open Demo Dashboard','Copy Build Command',
        'Copy Fake-Hardware Launch Command','Copy Demo Summary',
        'No robot motion commanded','Offline/fake-hardware preview only'
    ]:
        assert token in text
