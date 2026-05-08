import json
from pathlib import Path

def test_placeholder_assets_and_meshes_exist():
    payload=json.loads(Path('workcell_studio_catalog/generated/workcell_builder_gui_catalog.json').read_text())
    ids={a['id'] for a in payload['assets']}
    for aid in ['small_bin','medium_bin','large_bin','conveyor_1m','conveyor_2m','simple_fixture_plate','cnc_machine_placeholder','safety_fence_panel','cube_small','box_large','cylinder_small','overhead_camera_placeholder']:
        assert aid in ids
    for mesh in [
        'assets/environment/bins/meshes/small_bin.stl',
        'assets/environment/conveyors/meshes/conveyor_1m.stl',
        'assets/environment/fixtures/meshes/simple_fixture_plate.stl',
        'assets/environment/machines/meshes/cnc_machine_placeholder.stl',
        'assets/environment/safety/meshes/safety_fence_panel.stl',
        'assets/objects/primitives/meshes/cube_small.stl']:
        assert Path(mesh).exists()
