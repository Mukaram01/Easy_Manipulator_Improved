from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests/fixtures/environment.yaml"


def test_yaml_fixture_contains_required_table_01_fields():
    env = yaml.safe_load(FIXTURE.read_text(encoding="utf-8"))
    table = env["placed_objects"][0]
    assert table["name"] == "table_01"
    for key in ["mesh_path", "collision_mesh", "xyz", "rpy", "scale", "parent_frame", "collision_enabled"]:
        assert key in table


def test_pose_round_trip_markers_present():
    cpp = (ROOT / "workcell_builder/workcell_builder/gui/object_placement_yaml_io.cpp").read_text(encoding="utf-8")
    assert "xyz" in cpp
    assert "rpy" in cpp
    assert "warnings" in cpp
