from pathlib import Path


def test_workcell_studio_qss_exists():
    qss = Path('workcell_builder/workcell_builder/gui/resources/workcell_studio_dark.qss')
    assert qss.exists(), 'Expected Workcell Studio dark theme stylesheet to exist'


def test_workcell_studio_labels_present_in_source():
    source = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    for token in [
        'Workcell Studio',
        'Scene Builder',
        'Existing Scenes',
        'Full Screen',
        'Validate',
        'Generate Scene',
        'Asset Catalog',
        'Readiness',
    ]:
        assert token in source, f'Missing UI token: {token}'
