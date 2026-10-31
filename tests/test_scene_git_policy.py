"""Git policy for scenes and assets: new user cells and imported CAD stay local; the reference library stays trackable."""
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def ignored(path):
    result = subprocess.run(['git', 'check-ignore', '-q', '--no-index', path], cwd=ROOT)
    assert result.returncode in (0, 1), result
    return result.returncode == 0


@pytest.fixture(scope='module', autouse=True)
def git_checkout():
    if subprocess.run(['git', 'rev-parse', '--git-dir'], cwd=ROOT, capture_output=True).returncode:
        pytest.skip('not a Git checkout')


def test_new_user_cells_and_their_assets_are_ignored():
    assert ignored('scenes/a_new_user_cell/environment.yaml')
    assert ignored('scenes/a_new_user_cell/assets/imported/customer_part.step')
    assert ignored('workcell_builder/workcell_builder/assets/imported/customer_part.stl')
    assert not ignored('workcell_builder/workcell_builder/assets/imported/.gitkeep')


def test_imported_asset_catalog_is_local_and_its_bootstrap_stays_tracked():
    library = 'workcell_builder/workcell_builder/assets/imported/'
    for local in ['imported_environment_assets.json', '.customer_part.stl.importing', '.catalog.x1y2.json']:
        assert ignored(library + local), local
    assert not ignored('workcell_builder/workcell_builder/config/asset_profiles/imported_environment_assets.json')


def test_every_tracked_scene_keeps_its_future_source_changes_trackable():
    tracked = subprocess.run(['git', 'ls-files', 'scenes'], cwd=ROOT, capture_output=True, text=True,
                             check=True).stdout.split()
    scenes = sorted({path.split('/')[1] for path in tracked if path.count('/') >= 2})
    assert {'ur5_2f_test', 'suction_test'} <= set(scenes)
    for scene in scenes:
        assert not ignored(f'scenes/{scene}/environment.yaml'), scene
        assert not ignored(f'scenes/{scene}/assets/library/0123456789abcdef.stl'), scene
        assert ignored(f'scenes/{scene}/generated/scene_visual_mesh_index.json'), scene
    assert not ignored('scenes/supported_scenes.yaml')
    assert not ignored('assets/environment/new_core_asset/meshes/model.stl')


def test_imported_cad_is_ignored_in_every_scene_and_tracked_reference_imports_stay_tracked():
    scenes = {path.split('/')[1] for path in subprocess.run(['git', 'ls-files', 'scenes'], cwd=ROOT, capture_output=True,
                                                             text=True, check=True).stdout.split() if path.count('/') >= 2}
    for scene in scenes | {'a_new_user_cell'}:
        assert ignored(f'scenes/{scene}/assets/imported/catalog/0123456789abcdef.stl'), scene
        assert ignored(f'scenes/{scene}/assets/imported/customer_part.stl'), scene
    tracked = subprocess.run(['git', 'ls-files', 'scenes/ur5_2f_test/assets/imported'], cwd=ROOT, capture_output=True,
                             text=True, check=True).stdout.split()
    assert len(tracked) == 3 and all((ROOT / path).is_file() for path in tracked)


def test_save_copies_imported_cad_to_the_ignored_folder_and_library_meshes_to_the_tracked_one(tmp_path):
    import sys
    sys.path.insert(0, str(ROOT / 'scripts'))
    from workcell_studio_layout_merge import _scene_copy_folder
    library = ROOT / 'workcell_builder/workcell_builder/assets/imported/customer_part.stl'
    assert _scene_copy_folder(library).as_posix() == 'assets/imported/catalog'
    assert _scene_copy_folder(tmp_path / 'customer_part.stl').as_posix() == 'assets/imported/catalog'
    table = next((ROOT / 'assets/environment').rglob('*.stl'))
    assert _scene_copy_folder(table).as_posix() == 'assets/library'


def test_a_tracked_scene_using_ignored_cad_is_reported_until_the_part_is_promoted(tmp_path):
    import sys
    sys.path.insert(0, str(ROOT / 'scripts'))
    from validate_workcell_studio_generated_scene import local_only_meshes
    repo = tmp_path / 'repo'
    scene = repo / 'scenes/reference_cell'
    for rel in ['assets/imported/catalog/aaaa.stl', 'assets/library/bbbb.stl']:
        (scene / rel).parent.mkdir(parents=True, exist_ok=True)
        (scene / rel).write_bytes(b'solid x\nendsolid x\n')
    (scene / 'environment.yaml').write_text(
        'environment:\n  assets:\n'
        '  - {id: part, mesh: {path: assets/imported/catalog/aaaa.stl}}\n'
        '  - {id: bin, mesh: {path: assets/library/bbbb.stl}}\n'
        '  - {id: table, mesh: {path: "package://table_description/meshes/table.stl"}}\n')
    (repo / '.gitignore').write_text('scenes/*/assets/imported/\n')
    git = lambda *args: subprocess.run(['git', '-C', str(repo), *args], capture_output=True, check=True)
    git('init', '-q')
    assert local_only_meshes(scene) == []  # an untracked cell is local by design
    git('add', '-A')
    assert local_only_meshes(scene) == ['assets/imported/catalog/aaaa.stl']
    git('add', '-f', 'scenes/reference_cell/assets/imported/catalog/aaaa.stl')
    assert local_only_meshes(scene) == []
