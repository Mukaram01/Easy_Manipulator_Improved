from pathlib import Path

CMAKE = Path('workcell_builder/workcell_builder/CMakeLists.txt').read_text(encoding='utf-8')
MAIN_CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

REQUIRED_LABELS = [
    'Workcell Studio',
    'New Cell',
    'Open Existing Scene',
    'Scene Builder',
    'Asset Browser',
    'Scenario Templates',
    'Validate',
    'Preview',
    'Generate Scene',
    'Export',
    'Full Screen',
    'No robot motion commanded',
]


def test_cmake_keeps_qt5_widgets_wiring():
    assert 'find_package(Qt5 COMPONENTS Widgets Concurrent REQUIRED)' in CMAKE
    assert 'Qt5::Widgets' in CMAKE
    assert 'Qt5::Concurrent' in CMAKE


def test_dark_qss_file_exists_and_installed():
    qss = Path('workcell_builder/workcell_builder/gui/resources/workcell_studio_dark.qss')
    assert qss.exists(), 'Expected dark theme stylesheet to exist'
    assert 'install(DIRECTORY gui/resources' in CMAKE


def test_required_studio_labels_present_in_source():
    for label in REQUIRED_LABELS:
        assert label in MAIN_CPP, f'Missing required label: {label}'


def test_required_actions_have_wiring_or_safe_fallback():
    assert 'if (label == "Open Existing Scene")' in MAIN_CPP
    assert 'if (title == "Open Existing Scene" || title == "Scene Builder")' in MAIN_CPP
    assert 'label == "Validate" || label == "Generate Scene"' in MAIN_CPP
    assert 'title == "Validate" || title == "Generate Scene"' in MAIN_CPP
    assert 'show_not_wired_message' in MAIN_CPP
    assert 'This Workcell Studio action is not wired yet. No files changed and no robot motion was commanded.' in MAIN_CPP
