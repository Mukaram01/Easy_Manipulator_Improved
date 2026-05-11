from pathlib import Path


def test_external_importer_files_and_cmake_wiring_exist():
    assert Path('workcell_builder/workcell_builder/include/external_asset_importer.hpp').exists()
    assert Path('workcell_builder/workcell_builder/src_external_asset_importer.cpp').exists()
    assert 'src_external_asset_importer.cpp' in Path('workcell_builder/workcell_builder/CMakeLists.txt').read_text(encoding='utf-8')


def test_external_import_ui_strings_and_category_and_managed_path_markers():
    blob = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for m in [
        'Import External Asset','External Asset Import Wizard','Select STL / URDF','Asset Name','Asset Category','Asset Type',
        'Default Dimensions','Default Pose','Default Z Hint','License / Source Note','Validate Imported Asset',
        'Add to Asset Library','Import and Place','Import Summary','Custom / Imported'
    ]:
        assert m in blob or m in Path('workcell_builder/workcell_builder/src_external_asset_importer.cpp').read_text(encoding='utf-8')
    assert Path('workcell_builder/workcell_builder/assets/imported').exists()


def test_imported_catalog_and_validator_and_bundle_markers():
    assert Path('workcell_builder/workcell_builder/config/asset_profiles/imported_environment_assets.json').exists()
    v = Path('scripts/validate_workcell_asset_catalog.py').read_text(encoding='utf-8')
    for m in ['imported_environment_assets.json','license/source note required','symlink not allowed','absolute mesh_path forbidden','invalid default_dimensions_m']:
        assert m in v
    for p in ['scripts/export_workcell_scene_bundle.py','scripts/import_workcell_scene_bundle.py','scripts/validate_workcell_scene_bundle.py']:
        assert 'imported assets bundle support' in Path(p).read_text(encoding='utf-8')


def test_no_forbidden_additions():
    txt='\n'.join(Path(p).read_text(encoding='utf-8',errors='ignore').lower() for p in [
        'workcell_builder/workcell_builder/src_external_asset_importer.cpp',
        'scripts/validate_workcell_asset_catalog.py',
        'scripts/validate_workcell_asset_catalog.py'
    ])
    for forbidden in ['pyyaml','import yaml','streamlit','getmotionplan','execute_trajectory','followjointtrajectory','bill of materials','bom']:
        assert forbidden not in txt
