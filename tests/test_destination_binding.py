import json
from pathlib import Path

import pytest

REPO = Path(__file__).parents[1]

BIN = ('<model name="target_bin_physics"><static>true</static><link name="bin_link">'
       '<collision name="floor"><pose>0.45 0.22 %s 0 0 0</pose><geometry><box><size>0.26 0.14 0.01</size></box></geometry></collision>'
       '<collision name="wall_x_minus"><pose>0.325 0.22 0.145 0 0 0</pose><geometry><box><size>0.01 0.14 0.10</size></box></geometry></collision>'
       '</link></model>')
ITEM = {'id': 'workcell::target_bin_default', 'source_item_id': 'target_bin_default', 'semantic_role': 'target_bin', 'frame_id': 'world',
        'operation': 'ADD', 'pose': {'xyz': [.45, .22, .1], 'quaternion_xyzw': [0., 0., 0., 1.]}, 'source_geometry': {'type': 'mesh'}}


def world(floor_pose_z='0.095', extra=''):
    return '<sdf version="1.8"><world name="a0">' + (BIN % floor_pose_z).replace('<pose>0.45 0.22', extra + '<pose>0.45 0.22', 1) + '</world></sdf>'


def manifest(tmp_path, items):
    import yaml
    path = tmp_path / 'manifest.yaml'
    path.write_text(yaml.safe_dump({'objects': items}))
    return path


def test_binding_records_the_physics_floor_top_at_the_authored_destination_origin(tmp_path):
    from destination_binding import bind_destination_geometry
    from simulator_backend import digest
    m = manifest(tmp_path, [ITEM])
    binding = bind_destination_geometry(world(), m)
    assert binding['object_id'] == 'workcell::target_bin_default'
    assert (binding['model'], binding['link'], binding['collision']) == ('target_bin_physics', 'bin_link', 'floor')
    assert binding['floor_z'] == pytest.approx(.1) and binding['floor_xyz'][:2] == pytest.approx([.45, .22])
    assert binding['manifest_sha256'] == digest(m.read_bytes())


def test_binding_fails_closed(tmp_path):
    from destination_binding import bind_destination_geometry
    m = manifest(tmp_path, [ITEM])
    assert bind_destination_geometry('<sdf><world name="a0"/></sdf>', m) is None
    with pytest.raises(ValueError, match='destination geometry binding rejected.*0.095000.*0.100000'):
        bind_destination_geometry(world('0.09'), m)  # stage_a0_sparse before the fix
    with pytest.raises(ValueError, match='destination floor pose'):
        bind_destination_geometry(world().replace('<collision name="floor"><pose>', '<collision name="floor"><pose relative_to="bin_link">'), m)
    with pytest.raises(ValueError, match='ambiguous destination objects'):
        bind_destination_geometry(world(), manifest(tmp_path, [ITEM, dict(ITEM, id='workcell::other_bin')]))
    with pytest.raises(ValueError, match='collision manifest'):
        bind_destination_geometry(world(), None)


def test_augment_spec_writes_the_binding_for_the_receipt(tmp_path):
    from destination_binding import augment_spec
    (tmp_path / 'world.sdf').write_text(world())
    (tmp_path / 'spec.json').write_text(json.dumps({'world': 'a0'}))
    binding = augment_spec(tmp_path, manifest(tmp_path, [ITEM]))
    assert json.loads((tmp_path / 'spec.json').read_text())['destination_geometry_binding'] == binding


def test_committed_worlds_bin_floor_sits_at_the_authored_destination_origin():
    from destination_binding import bind_destination_geometry
    manifest_path = REPO / 'scenes/ur5_2f_test/config/moveit_collision_objects.yaml'
    for name in ('stage_a0_sparse.sdf', 'stage_a0.sdf'):
        binding = bind_destination_geometry((REPO / 'scenes/ur5_2f_test/worlds' / name).read_text(), manifest_path)
        assert binding['floor_z'] == pytest.approx(.1) and binding['object_id'] == 'workcell::target_bin_default'


def test_launch_applies_the_binding_after_prepare_without_touching_the_evidence_bound_backend():
    launch = (REPO / 'scenes/ur5_2f_test/launch/demo.launch.py').read_text()
    assert "destination_binding.augment_spec(output, collision_manifest_path)" in launch
    assert 'destination' not in (REPO / 'scripts/simulator_backend.py').read_text()
