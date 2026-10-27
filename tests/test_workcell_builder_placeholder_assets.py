import json
from pathlib import Path

# Single-triangle stub meshes (1 m x 1 m x 0 flat triangles) were removed: they
# drew as distorted triangles and stood in for products that have no real model.
REMOVED_STUB_DIRS = [
    'assets/environment/bins/meshes',
    'assets/environment/conveyors/meshes',
    'assets/environment/fixtures/meshes',
    'assets/environment/machines/meshes',
    'assets/environment/safety/meshes',
    'assets/environment/cameras/meshes',
    'assets/objects/primitives/meshes',
]


def test_generated_gui_catalog_lists_only_existing_meshes():
    payload = json.loads(Path('workcell_studio_catalog/generated/workcell_builder_gui_catalog.json').read_text())
    for asset in payload['assets']:
        mesh = asset.get('mesh_path') or ''
        if mesh:
            assert Path(mesh).is_file(), f"{asset['id']} lists a missing mesh {mesh}"
        assert not any(mesh.startswith(d) for d in REMOVED_STUB_DIRS), asset['id']


def test_removed_stub_meshes_stay_removed():
    for directory in REMOVED_STUB_DIRS:
        assert not list(Path(directory).glob('*.stl')), directory
