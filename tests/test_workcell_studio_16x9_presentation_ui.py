from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_full_screen_presentation_tokens_present():
    for token in ['Full Screen', 'Exit Full Screen', 'Press Esc to exit full screen']:
        assert token in MAIN


def test_presentation_sections_present():
    for token in [
        'Studio Home', 'Scene Builder', 'Demo Mode', 'Preview Launch',
        'Fake Hardware | No Robot Motion'
    ]:
        assert token in MAIN


def test_dashboard_action_and_fullscreen_controls_are_explicit():
    for token in [
        'const QStringList action_labels = {"Studio Home", "New Cell", "Validate", "Plan & Simulate", "Generate Scene Package", "Export"}',
        'Studio Home: switched to scene manager page.',
        'Full Screen',
        'Exit Full Screen',
    ]:
        assert token in MAIN
