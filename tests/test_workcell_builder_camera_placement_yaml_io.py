from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def test_camera_yaml_io_symbols_and_fields_present():
    h=(ROOT/'workcell_builder/workcell_builder/include/object_placement_yaml_io.hpp').read_text()
    cpp=(ROOT/'workcell_builder/workcell_builder/gui/object_placement_yaml_io.cpp').read_text()
    for n in ['load_camera_placements_from_environment_yaml','save_camera_placements_to_environment_yaml','camera_placements','horizontal_fov_deg','vertical_fov_deg','near_m','far_m']:
        assert n in h+cpp
    assert 'validate_camera_placement' in cpp
