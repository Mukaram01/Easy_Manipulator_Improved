from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_yaml_io_files_and_markers_exist():
    hpp = (ROOT / 'workcell_builder/workcell_builder/include/object_placement_yaml_io.hpp').read_text(encoding='utf-8')
    cpp = (ROOT / 'workcell_builder/workcell_builder/gui/object_placement_yaml_io.cpp').read_text(encoding='utf-8')
    assert 'save_placed_objects_to_environment_yaml' in hpp
    assert 'load_placed_objects_from_environment_yaml' in hpp
    assert 'placed_objects' in cpp
    assert 'collision_mesh' in cpp
    assert 'parent_frame' in cpp


def test_pose_round_trip_markers_present():
    cpp = (ROOT / 'workcell_builder/workcell_builder/gui/object_placement_yaml_io.cpp').read_text(encoding='utf-8')
    assert 'xyz' in cpp
    assert 'rpy' in cpp
    assert 'warnings' in cpp
