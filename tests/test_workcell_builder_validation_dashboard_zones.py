from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/src_validation_dashboard_model.cpp').read_text()

def test_validation_dashboard_has_zone_checks():
    assert 'Work Zone Validation' in CPP
    for token in ['Robot reach covers pick zone','Camera ROI covers pick zone if camera enabled','Task intent matches selected pick/place zones']:
        assert token in CPP

def test_blocked_warning_preview_only_tokens_present():
    for token in ['FAIL','WARN','PREVIEW_ONLY']:
        assert token in CPP
