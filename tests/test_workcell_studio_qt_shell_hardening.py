from pathlib import Path

CMAKE = Path('workcell_builder/workcell_builder/CMakeLists.txt').read_text(encoding='utf-8')
MAIN_CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

REQUIRED_LABELS = [
    'Workcell Studio',
    'New Cell',
    'Open Scene',
    'Validate',
    'Demo Mode',
    'Preview Launch',
    'Generate Scene',
    'Export',
    'Full Screen',
    'No robot motion commanded',
]


def test_cmake_keeps_qt5_widgets_wiring():
    assert 'find_package(Qt5 COMPONENTS Widgets Concurrent Svg OpenGL Network REQUIRED)' in CMAKE
    assert 'Qt5::Widgets' in CMAKE
    assert 'Qt5::Concurrent' in CMAKE


def test_required_studio_labels_present_in_source():
    for label in REQUIRED_LABELS:
        assert label in MAIN_CPP, f'Missing required label: {label}'


def test_top_command_bar_actions_are_wired_to_real_handlers():
    for branch in [
        'if (label == "New Cell")',
        'if (label == "Open Scene")',
        'if (label == "Validate")',
        'if (label == "Demo Mode")',
        'if (label == "Preview Launch")',
        'if (label == "Generate Scene")',
        'if (label == "Export")',
    ]:
        assert branch in MAIN_CPP


def test_top_command_bar_does_not_use_not_wired_message():
    connect_block = MAIN_CPP.split('connect(button, &QPushButton::clicked, this, [this, label]() {', 1)[1]
    connect_block = connect_block.split('});', 1)[0]
    assert 'show_not_wired_message(label)' not in connect_block


def test_helper_script_discovery_and_safety_text_remain_present():
    assert 'helper_script_search_paths("workcell_studio.py")' in MAIN_CPP
    assert '/scripts/' in MAIN_CPP and 'workcell_studio.py' in MAIN_CPP
    assert 'No robot motion commanded' in MAIN_CPP
