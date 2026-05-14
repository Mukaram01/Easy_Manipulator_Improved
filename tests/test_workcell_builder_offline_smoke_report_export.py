from pathlib import Path


def test_report_artifact_paths_and_sections_present():
    cpp = Path('workcell_builder/workcell_builder/src_offline_smoke_check_model.cpp').read_text()
    for token in [
        'offline_smoke_report.json',
        'offline_smoke_report.html',
        'offline_smoke_summary.txt',
        'next_action',
        'Safety banner: no robot motion'
    ]:
        assert token in cpp
