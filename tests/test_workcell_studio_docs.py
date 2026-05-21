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



def test_scene3d_camera_fov_overlay_contract_docs() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    contract_doc = repo_root / "docs" / "architecture" / "SCENE3D_CANVAS_CONTRACT.md"
    contract_text = contract_doc.read_text(encoding="utf-8")

    assert "## Camera, FOV, and perception overlay contract" in contract_text
    assert "camera marker source is `editable_layout`" in contract_text
    assert "FOV frustum must render on the `overlay` layer as read-only guidance visuals" in contract_text
    assert "Detection snapshot overlays" in contract_text
    assert "preview-only and non-authoritative for scene generation" in contract_text
    assert "No live EPD/RealSense startup" in contract_text


def test_new_cell_manual_camera_fov_overlay_guidance() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    manual_doc = repo_root / "docs" / "manuals" / "WORKCELL_STUDIO_NEW_CELL_FROM_SCRATCH.md"
    manual_text = manual_doc.read_text(encoding="utf-8")

    assert "## Side-pick camera metadata, Scene3D visualization, and preview warnings" in manual_text
    assert "`pick_zone` targeting remains authoritative" in manual_text
    assert "FOV frustum guidance on the overlay layer (read-only)" in manual_text
    assert "Optional detection snapshot overlays may appear as read-only preview annotations" in manual_text
    assert "outside configured `pick_zone`" in manual_text
    assert "Missing camera metadata or missing FOV/frustum metadata" in manual_text
    assert "They are not hardware approval" in manual_text
