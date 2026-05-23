from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT/'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')

def test_scene3d_tracks_generated_fallback_and_mesh_counts():
    assert 'mesh_rendered_count' in CPP
    assert 'generated_fallback_count' in CPP
    assert 'draw_truthful_item_geometry' in CPP
