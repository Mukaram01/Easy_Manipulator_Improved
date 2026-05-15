from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_preview_page_cards_present():
    for token in [
        'Selected Scene',
        'Readiness Gate',
        'Safe Commands',
        'Preview Process',
        'Reports / Artifacts',
        'Live Log',
    ]:
        assert token in CPP


def test_validation_page_cards_present():
    for token in [
        'Validation Summary',
        'Blockers',
        'Warnings',
        'Required Files',
        'Task Intent Status',
        'Scene Package Status',
        'Next Fix Suggestions',
    ]:
        assert token in CPP


def test_references_scripts_and_fake_hardware_safety():
    assert 'workcell_studio.py' in CPP
    assert 'generate_workcell_studio_readiness_pack.py' in CPP
    assert 'use_fake_hardware:=true' in CPP
    assert 'Fake Hardware' in CPP
    assert 'No Robot Motion' in CPP
    assert 'Preview Only' in CPP


def test_validation_actions_and_no_not_wired():
    for token in [
        'Run Offline Validation',
        'Open Validation Report',
        'Copy Validation Summary',
        'Generate Readiness Pack',
        'Open Readiness Dashboard',
    ]:
        assert token in CPP

    forbidden = [
        'show_not_wired_message("Run Offline Validation")',
        'show_not_wired_message("Open Validation Report")',
        'show_not_wired_message("Copy Validation Summary")',
        'show_not_wired_message("Generate Readiness Pack")',
        'show_not_wired_message("Open Readiness Dashboard")',
        'show_not_wired_message("Run Fake-Hardware Preview")',
        'show_not_wired_message("Validate")',
    ]
    for token in forbidden:
        assert token not in CPP
