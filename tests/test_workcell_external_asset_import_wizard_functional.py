import json
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
    for s in ['sanitize_imported_asset_name','unsupported extension','path traversal rejected','symlink not allowed','QSaveFile','!QFileInfo::exists(destination)','imported_environment_assets.json']:
        assert s in cpp
    # One catalog, rewritten atomically: the old importer appended objects after the JSON array.
    assert 'std::ios::app' not in cpp
    assert '"Custom / Imported"' in cpp or 'Custom / Imported' in cpp
    # The tracked catalog is the read-only legacy bootstrap; imports write the Git-ignored local catalog.
    cat=json.loads(Path('workcell_builder/workcell_builder/config/asset_profiles/imported_environment_assets.json').read_text(encoding='utf-8'))
    assert isinstance(cat, list)
    for entry in cat:
        assert entry['category'] == 'Custom / Imported'
        assert entry['mesh_path'].startswith('workcell_builder/workcell_builder/assets/imported/')
        assert Path(entry['mesh_path']).is_file() and Path(entry['mesh_path']).stat().st_size > 84


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


def test_cli_import_writes_only_the_local_catalog_atomically(tmp_path):
    import subprocess, sys
    repo = Path(__file__).resolve().parents[1]
    legacy = tmp_path / 'workcell_builder/workcell_builder/config/asset_profiles/imported_environment_assets.json'
    legacy.parent.mkdir(parents=True)
    # The old C++ importer appended objects after the array; they are recovered, and the file is never rewritten.
    legacy.write_text('[]\n{\n  "asset_id": "old_part",\n  "mesh_path": "workcell_builder/workcell_builder/assets/imported/old_part.stl"\n}\n')
    (tmp_path / 'customer').mkdir()
    for name, body in [('part.stl', 'solid a\nendsolid a\n'), ('other.stl', 'solid b\nendsolid b\n')]:
        (tmp_path / 'customer' / name).write_text(body)

    def run(name):
        out = subprocess.run([sys.executable, '-m', 'scripts.import_workcell_asset', '--source', str(tmp_path / 'customer' / name),
                              '--asset-id', 'part', '--label', 'Part', '--asset-type', 'custom', '--repo-root', str(tmp_path),
                              '--print-summary'], cwd=repo, capture_output=True, text=True, check=True)
        return json.loads(out.stdout)

    assert run('part.stl')['status'] == 'ok'
    assert run('part.stl') == {'status': 'reused', 'asset_id': 'part', 'path': 'workcell_builder/workcell_builder/assets/imported/part.stl'}
    assert run('other.stl')['asset_id'] == 'part_2'
    library = tmp_path / 'workcell_builder/workcell_builder/assets/imported'
    catalog = json.loads((library / 'imported_environment_assets.json').read_text())
    assert [entry['asset_id'] for entry in catalog] == ['old_part', 'part', 'part_2']
    assert str(tmp_path) not in (library / 'imported_environment_assets.json').read_text()
    assert legacy.read_text().startswith('[]\n{')
    assert sorted(p.name for p in library.iterdir()) == ['imported_environment_assets.json', 'part.stl', 'part_2.stl']
