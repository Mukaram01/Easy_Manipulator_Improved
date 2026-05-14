from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
QSS = (ROOT / 'workcell_builder/workcell_builder/gui/resources/workcell_studio_dark.qss').read_text(encoding='utf-8')


def test_qss_has_required_selectors():
    for token in ['QMainWindow', 'QPushButton', 'QFrame', 'QTabWidget', 'QTableWidget', 'QToolBar']:
        assert token in QSS


def test_visual_tokens_and_safety_markers_present():
    for token in [
        'Fake Hardware', 'No Robot Motion', 'Demo Mode', 'Preview Launch',
        'Press Esc to exit full screen', 'READY', 'WARNINGS', 'BLOCKED', 'PREVIEW_ONLY'
    ]:
        assert token in MAIN


def test_card_and_console_tokens_present():
    for token in ['status badge', 'safety banner', 'scene overview', 'digital twin preview', 'command console']:
        assert token in MAIN
