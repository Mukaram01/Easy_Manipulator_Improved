#!/usr/bin/env python3
"""Documentation regression checks for Workcell Studio positioning."""

from pathlib import Path


def test_workcell_studio_docs_positioning() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    readme = repo_root / "README.md"
    roadmap = repo_root / "docs" / "manuals" / "WORKCELL_STUDIO_ROADMAP.md"

    readme_text = readme.read_text(encoding="utf-8")
    roadmap_text = roadmap.read_text(encoding="utf-8")

    assert "Workcell Studio" in readme_text
    assert "WORKCELL_STUDIO_ROADMAP.md" in readme_text

    assert roadmap.is_file()
    assert "sorting is only one scenario template" in roadmap_text.lower()
    assert "RViz + MoveIt" in roadmap_text
    assert "Gazebo Sim" in roadmap_text
    assert "Isaac Sim" in roadmap_text
    assert "Streamlit" in roadmap_text
