from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_imported_flags_and_mesh_urdf_persistence_present():
    for token in ['RoleImported', 'mesh_path:', 'urdf_path:', 'imported: %17', 'source_path']:
        assert token in CPP
