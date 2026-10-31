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
        assert not ignored(f'scenes/{scene}/assets/imported/new_reference_part.stl'), scene
        assert ignored(f'scenes/{scene}/generated/scene_visual_mesh_index.json'), scene
    assert not ignored('scenes/supported_scenes.yaml')
    assert not ignored('assets/environment/new_core_asset/meshes/model.stl')
