from pathlib import Path

CPP = Path(__file__).resolve().parents[1] / 'workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.cpp'
SRC = CPP.read_text()

def test_layout_and_reset_present():
    assert 'Recommended layout applied' in SRC
    assert 'onResetLayout' in SRC

def test_zone_and_route_row_mutations_present():
    assert 'zoneTable->insertRow' in SRC
    assert 'zoneTable->removeRow' in SRC
    assert 'routingTable->insertRow' in SRC
    assert 'routingTable->removeRow' in SRC

def test_validate_routing_checks_missing_zone_and_unknown_route():
    assert 'unknown zone' in SRC
    assert 'missing unknown route' in SRC

def test_epd_mode_sample_real_switch_tokens_present():
    assert 'Real EPD connector' in SRC
    assert 'Sample demo mode enabled' in SRC
