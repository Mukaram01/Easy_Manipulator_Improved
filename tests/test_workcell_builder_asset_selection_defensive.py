from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LOADER = ROOT / "workcell_builder" / "workcell_builder" / "gui" / "loadobjects.cpp"


def test_loader_has_defensive_bounds_checks():
    text = LOADER.read_text(encoding="utf-8")
    assert "temp_object.link_vector.empty()" in text
    assert "temp_object.ext_joint.child_link_pos < 0" in text
    assert "temp_object.ext_joint.child_link_pos >= static_cast<int>(temp_object.link_vector.size())" in text
