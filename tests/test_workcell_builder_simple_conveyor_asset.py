from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONVEYOR_YAML = ROOT / "assets" / "environment" / "simple_conveyor_description" / "simple_conveyor.yaml"
LOADER = ROOT / "workcell_builder" / "workcell_builder" / "gui" / "loadobjects.cpp"


def test_simple_conveyor_exists_and_parses():
    data = yaml.safe_load(CONVEYOR_YAML.read_text(encoding="utf-8"))
    assert "simple_conveyor" in data
    assert isinstance(data["simple_conveyor"].get("links"), dict)


def test_conveyor_loader_has_placeholder_fallback_guard():
    text = LOADER.read_text(encoding="utf-8")
    assert "Conveyor asset is incomplete; using placeholder geometry" in text
    assert "make_placeholder_link" in text
