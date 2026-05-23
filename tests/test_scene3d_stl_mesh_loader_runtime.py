from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "workcell_builder" / "workcell_builder" / "gui" / "scene3d_viewport_widget.cpp").read_text(encoding="utf-8")


def test_ascii_and_binary_stl_parsers_present():
    assert "parse_ascii_stl" in CPP
    assert "parse_binary_stl" in CPP
    assert "looks_like_ascii_stl" in CPP


def test_stl_loader_reports_errors_and_triangle_limit():
    for token in ["mesh triangle count exceeds limit", "ascii STL contains no triangles", "binary STL truncated"]:
        assert token in CPP
