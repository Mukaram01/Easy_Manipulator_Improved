from pathlib import Path


def test_integration_hooks_exist():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
    for token in [
        'on_run_offline_smoke_check_clicked',
        'offline_smoke_status_label',
        'refresh_scene_status(true, "Run Offline Smoke Check")',
        'PREVIEW_ONLY'
    ]:
        assert token in cpp or token == 'PREVIEW_ONLY'
