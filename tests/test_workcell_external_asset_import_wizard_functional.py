from pathlib import Path


def test_dialog_files_and_cmake_wiring_and_strings():
    h=Path('workcell_builder/workcell_builder/include/external_asset_import_dialog.hpp')
    c=Path('workcell_builder/workcell_builder/gui/external_asset_import_dialog.cpp')
    assert h.exists() and c.exists()
    cm=Path('workcell_builder/workcell_builder/CMakeLists.txt').read_text(encoding='utf-8')
    assert 'gui/external_asset_import_dialog.cpp' in cm
    t=c.read_text(encoding='utf-8')
    for s in ['Browse/select external .stl / .urdf / .xacro','Validate Imported Asset','Add to Asset Library','Import and Place','Custom / Imported']:
        assert s in t


def test_importer_logic_markers_and_catalog_relative_path():
    cpp=Path('workcell_builder/workcell_builder/src_external_asset_importer.cpp').read_text(encoding='utf-8')
    for s in ['sanitize_imported_asset_name','unsupported extension','path traversal rejected','symlink not allowed','copy_file','while (fs::exists(dest)','imported_environment_assets.json']:
        assert s in cpp
    cat=Path('workcell_builder/workcell_builder/config/asset_profiles/imported_environment_assets.json').read_text(encoding='utf-8')
    assert '"category": "Custom / Imported"' in cat
    assert '"mesh_path": "workcell_builder/workcell_builder/assets/imported/' in cat


def test_cli_and_validation_and_forbidden_guards_present():
    cli=Path('scripts/import_workcell_asset.py').read_text(encoding='utf-8')
    assert '--source' in cli and 'unsupported file type' in cli and 'Custom / Imported' in cli
    v=Path('scripts/validate_workcell_asset_catalog.py').read_text(encoding='utf-8')
    for s in ['imported_environment_assets', 'absolute mesh_path forbidden', 'symlink not allowed', 'license/source note required']:
        assert s in v
    all_txt='\n'.join(Path(p).read_text(encoding='utf-8',errors='ignore').lower() for p in [
      'scripts/import_workcell_asset.py','workcell_builder/workcell_builder/src_external_asset_importer.cpp','workcell_builder/workcell_builder/gui/external_asset_import_dialog.cpp'])
    for forbidden in ['streamlit','pyyaml','import yaml','getmotionplan','execute_trajectory','followjointtrajectory','bill of materials','bom']:
        assert forbidden not in all_txt
