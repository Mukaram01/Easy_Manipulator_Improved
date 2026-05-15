from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_canonical_action_labels_exist():
    canonical = [
        'Add to Canvas',
        'Save Layout',
        'Remove Selected Layout Item',
        'Use Selected as Pick Zone',
        'Use Selected as Place Zone',
        'Use Selected as Camera',
        'Copy Launch Command',
        'Run Offline Validation',
        'Generate Readiness Pack',
        'Open Readiness Dashboard',
        'Export Scene Bundle',
        'Import Scene Bundle',
        'Open Export Folder',
        'Open Task File',
        'Copy Task Summary',
    ]
    for label in canonical:
        assert label in CPP


def test_removed_duplicate_execution_labels_not_present():
    removed = [
        'Run Acceptance',
        'Run Offline Smoke Check',
        'Generate Preview Bundle',
        'Open Scene Folder", demo',
        'Copy Build Command", demo',
        'Copy Launch Command", demo',
    ]
    for label in removed:
        assert label not in CPP


def test_demo_uses_navigation_instead_of_duplicate_actions():
    for label in ['Go to Validation', 'Go to Plan & Simulate', 'Go to Export']:
        assert label in CPP


def test_no_duplicate_validation_handler_connections():
    assert 'connect(run_offline_validation_button, &QPushButton::clicked, this, &MainWindow::run_offline_validation);' in CPP
    assert 'connect(validate_layout_button, &QPushButton::clicked, this, &MainWindow::run_layout_validation_only);' in CPP


def test_safety_text_remains_and_no_not_wired_usage():
    for token in ['Fake Hardware', 'No Robot Motion', 'Preview Only']:
        assert token in CPP

    forbidden = [
        'show_not_wired_message("Run Offline Validation")',
        'show_not_wired_message("Generate Readiness Pack")',
        'show_not_wired_message("Copy Launch Command")',
    ]
    for token in forbidden:
        assert token not in CPP
