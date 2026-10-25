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
    assert ("destination_binding.augment_spec(output, collision_manifest_path,\n"
            "            measured_floor=_SCENE_PACKAGE_DIR is not None)") in launch
    assert 'destination' not in (REPO / 'scripts/simulator_backend.py').read_text()


def test_binding_records_every_physics_wall_as_a_world_axis_aligned_box(tmp_path):
    from destination_binding import bind_destination_geometry
    binding = bind_destination_geometry(world(), manifest(tmp_path, [ITEM]))
    assert binding['walls'] == [dict(collision='wall_x_minus', xyz=[.325, .22, .145], size_m=[.01, .14, .10])]


@pytest.mark.parametrize('wall,message', [
    ('<collision name="wall_x_plus"><pose>0.575 0.22 0.145 0 0 0.1</pose><geometry><box><size>0.01 0.14 0.10</size></box></geometry></collision>', 'axis-aligned'),
    ('<collision name="wall_x_plus"><pose>0.575 0.22 0.145 0 0 0</pose><geometry><cylinder><radius>.01</radius><length>.1</length></cylinder></geometry></collision>', 'axis-aligned'),
    ('<collision name="wall_x_plus"><pose relative_to="bin_link">0.575 0.22 0.145 0 0 0</pose><geometry><box><size>0.01 0.14 0.10</size></box></geometry></collision>', 'world frame'),
    ('<collision name="wall_x_minus"><pose>0.575 0.22 0.145 0 0 0</pose><geometry><box><size>0.01 0.14 0.10</size></box></geometry></collision>', 'ambiguous'),
])
def test_wall_binding_fails_closed(tmp_path, wall, message):
    from destination_binding import bind_destination_geometry
    xml = world().replace('</link></model>', wall + '</link></model>')
    with pytest.raises(ValueError, match=message):
        bind_destination_geometry(xml, manifest(tmp_path, [ITEM]))


def test_walls_scene_diff_adds_one_convex_box_object_per_bound_wall():
    # One object per wall: the controller certificate bounds a multi-shape body by its
    # joint projections, so one four-wall object made every place descent UNCERTIFIED
    # (PRECISION_OR_DEPTH_LIMIT ~0.11 s into PREPLAN_PLACE, LENLEG 2026-10-24).
    from destination_binding import WALL_OBJECT_PREFIX, wall_object_ids, walls_scene_diff
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    walls = [dict(collision='wall_y_plus', xyz=[.45, .285, .145], size_m=[.24, .01, .10]),
             dict(collision='wall_y_minus', xyz=[.45, .155, .145], size_m=[.24, .01, .10])]
    scene = walls_scene_diff(dict(walls=walls))
    assert scene.is_diff and wall_object_ids(dict(walls=walls)) == [
        WALL_OBJECT_PREFIX + 'wall_y_plus', WALL_OBJECT_PREFIX + 'wall_y_minus']
    assert [o.id for o in scene.world.collision_objects] == wall_object_ids(dict(walls=walls))
    for item, wall in zip(scene.world.collision_objects, walls):
        assert (item.operation, item.header.frame_id) == (CollisionObject.ADD, 'world')
        assert [(p.type, list(p.dimensions)) for p in item.primitives] == [(SolidPrimitive.BOX, wall['size_m'])]
        pose, = item.primitive_poses
        assert [pose.position.x, pose.position.y, pose.position.z, pose.orientation.w] == [*wall['xyz'], 1.]
    for bad in ({}, dict(walls=[]), dict(walls=[dict(walls[0], size_m=[.24, 0., .1])]), dict(walls=[walls[0], walls[0]])):
        with pytest.raises(ValueError):walls_scene_diff(bad)


def test_committed_worlds_bind_the_walls_that_stopped_the_lenleg_full_cycle():
    # LENLEG 2026-10-23 full-cycle: during place, gripper_finger2_knuckle_link touched
    # wall_y_plus at (0.5695, 0.28, 0.1946); the authored mesh's inner +y face at that
    # height is at y~0.318, so only the bound physics wall makes MoveIt see it.
    from destination_binding import bind_destination_geometry
    manifest_path = REPO / 'scenes/ur5_2f_test/config/moveit_collision_objects.yaml'
    contact = [.5695, .28, .1946]
    for name in ('stage_a0_sparse.sdf', 'stage_a0.sdf'):
        walls = bind_destination_geometry((REPO / 'scenes/ur5_2f_test/worlds' / name).read_text(), manifest_path)['walls']
        assert {w['collision'] for w in walls} == {'wall_x_minus', 'wall_x_plus', 'wall_y_minus', 'wall_y_plus'}
        hit = [w['collision'] for w in walls if all(abs(c - x) <= s / 2 + 1e-4 for c, x, s in zip(contact, w['xyz'], w['size_m']))]
        assert hit == ['wall_y_plus']


def test_simulator_planning_adds_the_receipt_walls_before_the_initial_scene_is_read():
    source = (REPO / 'scripts/perceived_object_grasp_execute.py').read_text()
    block = source[source.index("stage('UPDATE_PLANNING_SCENE')"):source.index("evidence_scene('planning_scene_before', initial)")]
    assert "if args.backend == 'simulator':" in block
    assert block.index("digest(receipt_bytes) != summary['backend_identity']['receipt_sha256']") < block.index("apply(walls_scene_diff(destination_binding))")
    assert block.index("apply(walls_scene_diff(destination_binding))") < block.rindex("initial = scene_now()")
