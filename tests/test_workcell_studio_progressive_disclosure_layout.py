from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_more_actions_and_menu_components_exist():
    for token in ['More Actions', 'QToolButton', 'QMenu', 'setPopupMode(QToolButton::InstantPopup)']:
        assert token in CPP


def test_primary_actions_visible():
    for token in [
        'Add to Canvas',
        'Save Layout',
        'Generate/Update Task Intent',
        'Run Fake-Hardware Simulation',
        'Copy Launch Command',
        'Run Offline Validation',
        'Generate Readiness Pack',
        'Export Scene Bundle',
        'Import Scene Bundle',
    ]:
        assert token in CPP


def test_advanced_actions_present_in_menus_and_not_wired_not_used():
    for token in [
        'Remove Selected Layout Item',
        'Open Validation Report',
        'Copy Validation Summary',
        'Open Readiness Dashboard',
        'Open Scene Folder',
        'Open Preview Report',
        'Open Dashboard',
        'Copy Source Command',
        'Copy Build Command',
        'Open Export Folder',
        'Copy Bundle Summary',
        'Open Bundle Docs',
    ]:
        assert token in CPP
    assert 'show_not_wired_message("Run Offline Validation")' not in CPP


def test_mode_and_safety_chips_present():
    for token in [
        'Design',
        'Plan',
        'Plan',
        'Simulate',
        'Hardware Guarded',
        'fake-hardware',
        'guarded',
        'Real robot motion: locked',
    ]:
        assert token in CPP


def test_secondary_controls_moved_to_menus_and_theme_tokens_present():
    for token in [
        'Camera / View',
        'Canvas More',
        'Snap/Grid settings',
        'More Actions',
        'QLabel#safetyPill',
        'QToolButton',
        'QMenu',
    ]:
        assert token in CPP or token in Path('workcell_builder/workcell_builder/gui/resources/workcell_studio_dark.qss').read_text(encoding='utf-8')


def test_redundant_workspace_change_controls_removed_from_studio_pages():
    assert 'Change Workspace' not in CPP
    assert 'Workcell Studio change workspace' not in CPP
