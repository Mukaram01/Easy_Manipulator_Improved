from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_readiness_classifications_and_missing_assets_text_exist():
    for token in ['READY_WITH_WARNINGS', 'PREVIEW_ONLY', 'BLOCKED_MISSING_ASSETS', 'Missing/Warnings']:
        assert token in CPP

def test_preview_only_conveyor_and_safety_tokens_exist():
    for token in [
        'Conveyor Sorting + EPD Metadata Preview',
        'metadata-only by default',
        'no_runtime_motion',
        'fake_hardware_first: true',
        'runtime_execution_enabled: false',
    ]:
        assert token in CPP
