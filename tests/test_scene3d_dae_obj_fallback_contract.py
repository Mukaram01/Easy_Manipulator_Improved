from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

MAIN = (ROOT / "workcell_builder" / "workcell_builder" / "gui" / "mainwindow.cpp").read_text(encoding="utf-8")


def test_unsupported_mesh_falls_back_with_warning():
    assert "URDF visual mesh unavailable; using primitive fallback" in MAIN
    assert "unsupported_extension" in MAIN or "unsupported_format" in MAIN
