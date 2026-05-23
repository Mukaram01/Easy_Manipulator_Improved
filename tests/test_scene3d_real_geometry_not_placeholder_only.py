from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text()

def test_real_geometry_counters_available():
    for token in ['mesh_rendered_count', 'mesh_backed_count', 'primitive_fallback_count']:
        assert token in CPP
