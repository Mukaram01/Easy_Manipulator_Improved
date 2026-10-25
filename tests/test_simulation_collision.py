"""Primitive physics contact geometry derived from the tool's own collision meshes."""
import copy
import hashlib
import json
import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO/'scripts'))
import simulation_collision as sc  # noqa: E402

TIP_MESH = Path('/home/ros2/workcell_ws/install/robotiq_85_description/share/robotiq_85_description/meshes/collision/robotiq_85_finger_tip_link.stl')
RECORD = yaml.safe_load((REPO/'catalog/capabilities/end_effectors/ee_robotiq_2f.yaml').read_text())['end_effector']


def stl(path, triangles):
    data = bytearray(80) + struct.pack('<I', len(triangles))
    for tri in triangles:
        data += struct.pack('<12fH', 0, 0, 0, *np.asarray(tri, dtype=float).ravel(), 0)
    path.write_bytes(bytes(data))
    return path


def cuboid(lo, hi):
    """The 12 triangles of an axis-aligned box."""
    c = [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return [[c[i] for i in f] for f in faces]


# An L-shaped (concave) solid: a low proximal block and a tall distal pad block.
L_SHAPE = cuboid([0, 0, 0], [.015, .01, .02]) + cuboid([.025, 0, 0], [.04, .03, .02])
SURFACE = '<surface><friction><ode><mu>100000</mu><mu2>100000</mu2></ode></friction></surface>'


def native(tmp_path, mesh, pose='0.1 0 0 0 0 1.5707963267948966', record=None):
    sdf = (f'<sdf version="1.9"><model name="workcell_robot"><link name="tip"><collision name="tip_collision"><pose>{pose}</pose>'
           f'<geometry><mesh><uri>file://{mesh}</uri><scale>1 1 1</scale></mesh></geometry>{SURFACE}</collision></link>'
           '<link name="other"><collision name="c"><geometry><mesh><uri>file:///x.stl</uri></mesh></geometry></collision></link>'
           '</model></sdf>')
    (tmp_path/'robot.sdf').write_text(sdf)
    (tmp_path/'robot.urdf').write_text('<robot name="r"><link name="tip"><collision><geometry><mesh filename="planning.stl"/></geometry></collision></link></robot>')
    (tmp_path/'spec.json').write_text(json.dumps({'native_description_sha256': hashlib.sha256(sdf.encode()).hexdigest(), 'world': 'a0'}))
    block = dict(mode='mesh_slab_bounding_boxes', axis='x', cuts_m=[0.02], links=['tip'], evidence='test') if record is None else record
    return {'simulator_collision': block}


def test_only_the_spawned_physics_model_changes_and_the_spec_hash_follows(tmp_path):
    urdf_before = None
    ee = native(tmp_path, stl(tmp_path/'l.stl', L_SHAPE))
    urdf_before = (tmp_path/'robot.urdf').read_text()
    binding = sc.bind_native_model(tmp_path, ee)
    model = ET.parse(tmp_path/'robot.sdf').getroot()
    collisions = model.findall(".//link[@name='tip']/collision")
    assert [c.get('name') for c in collisions] == ['tip_collision_box0', 'tip_collision_box1']
    assert all(c.find('geometry/mesh') is None and c.find('geometry/box/size') is not None for c in collisions)
    assert all(c.find('surface/friction/ode/mu').text == '100000' for c in collisions)          # contact parameters kept
    pose = [float(v) for v in collisions[0].findtext('pose').split()]
    assert pose[3:] == pytest.approx([0, 0, math.pi/2])                                        # collision rotation kept
    assert model.find(".//link[@name='other']/collision/geometry/mesh") is not None              # unlisted links untouched
    assert (tmp_path/'robot.urdf').read_text() == urdf_before                                   # planning keeps the mesh
    spec = json.loads((tmp_path/'spec.json').read_text())
    assert spec['native_description_sha256'] == hashlib.sha256((tmp_path/'robot.sdf').read_bytes()).hexdigest()
    assert spec['native_description_sha256'] == binding['native_description_sha256'] != binding['source_native_description_sha256']
    assert spec['simulator_collision_primitives'] == binding and spec['world'] == 'a0'
    record, = binding['links']
    assert record['triangles'] == 24 and len(record['boxes']) == 2 and record['mesh_sha256']
    proximal, distal = record['boxes']
    assert proximal['box_size_m'] == pytest.approx([.015, .01, .02]) and distal['box_size_m'] == pytest.approx([.015, .03, .02])


def test_a_spec_that_does_not_match_the_model_is_refused_and_the_model_restored(tmp_path):
    ee = native(tmp_path, stl(tmp_path/'l.stl', L_SHAPE))
    (tmp_path/'spec.json').write_text(json.dumps({'native_description_sha256': 'stale'}))
    before = (tmp_path/'robot.sdf').read_bytes()
    with pytest.raises(sc.SimulationCollisionError, match='SIMULATOR_COLLISION_SOURCE_CHANGED'):
        sc.bind_native_model(tmp_path, ee)
    assert (tmp_path/'robot.sdf').read_bytes() == before
    assert json.loads((tmp_path/'spec.json').read_text()) == {'native_description_sha256': 'stale'}


@pytest.mark.skipif(not TIP_MESH.is_file(), reason='robotiq_85_description not installed')
def test_the_2f85_slab_boxes_keep_the_real_pad_face_and_set_the_rest_back(tmp_path):
    binding = sc.bind_native_model(tmp_path, native(tmp_path, TIP_MESH, pose='0 0 0 0 0 0', record=dict(RECORD['simulator_collision'], links=['tip'])))
    record, = binding['links']
    assert record['triangles'] == 2436 and binding['mode'] == 'mesh_slab_bounding_boxes'
    proximal, distal = record['boxes']
    face = lambda box: box['box_centre_m'][1] + box['box_size_m'][1]/2
    # The recorded pad contacts all lie on tip-frame y = 0.0078 m (the real pad face, x 7.0-44.5 mm).
    assert face(distal) == pytest.approx(.0078, abs=1e-5)
    assert face(proximal) == pytest.approx(.0060, abs=1e-5)                    # 1.8 mm behind the pad plane
    lo = distal['box_centre_m'][0] - distal['box_size_m'][0]/2
    assert 0.004 < lo < 0.007 and distal['box_size_m'][2] == pytest.approx(.022, abs=1e-6)


def test_slab_boxes_enclose_every_triangle():
    tris = np.array(L_SHAPE, dtype=float)
    boxes = sc.slab_boxes(tris, 'x', (0.02,))
    assert len(boxes) == 2
    for tri in tris:
        assert any(((tri >= lo - 1e-12) & (tri <= hi + 1e-12)).all() for lo, hi in boxes)
    assert len(sc.slab_boxes(tris)) == 1 and len(sc.slab_boxes(tris, 'x', (5.0,))) == 1     # empty slabs dropped


@pytest.mark.parametrize('sdf_link,message', [
    ('<link name="tip"/>', 'exactly one collision'),
    ('<link name="tip"><collision><geometry><box><size>1 1 1</size></box></geometry></collision></link>', 'not a mesh'),
    ('<link name="other"/>', 'exactly one collision'),
])
def test_unbindable_links_fail_closed(tmp_path, sdf_link, message):
    (tmp_path/'robot.sdf').write_text(f'<sdf><model name="m">{sdf_link}</model></sdf>')
    with pytest.raises(sc.SimulationCollisionError, match=message):
        sc.bind_native_model(tmp_path, {'simulator_collision': dict(RECORD['simulator_collision'], links=['tip'])})


def test_no_native_model_or_a_bad_mesh_fails_closed(tmp_path):
    with pytest.raises(sc.SimulationCollisionError, match='REQUIRES_NATIVE_MODEL'):
        sc.bind_native_model(tmp_path, {'simulator_collision': RECORD['simulator_collision']})
    (tmp_path/'ascii.stl').write_text('solid x\nendsolid x\n')
    with pytest.raises(sc.SimulationCollisionError, match='binary STL'):
        sc.bind_native_model(tmp_path, native(tmp_path, tmp_path/'ascii.stl'))


def test_the_capability_block_is_validated_and_absent_means_unchanged(tmp_path):
    assert sc.block_errors(RECORD) == []
    assert RECORD['simulator_collision']['links'] == ['gripper_finger1_finger_tip_link', 'gripper_finger2_finger_tip_link']
    for change, message in ((lambda b: b.update(mode='convex_hull'), 'mode'), (lambda b: b.update(links=[]), 'links'),
                            (lambda b: b.pop('evidence'), 'evidence'), (lambda b: b.update(axis='w'), 'axis'),
                            (lambda b: b.update(cuts_m=[.02, .01]), 'cuts_m'), (lambda b: b.update(cuts_m=[]), 'cuts_m'),
                            (lambda b: b.update(mode='mesh_bounding_box'), 'no axis or cuts_m')):
        bad = copy.deepcopy(RECORD); change(bad['simulator_collision'])
        assert any(message in e for e in sc.block_errors(bad))
        with pytest.raises(sc.SimulationCollisionError):
            sc.bind_native_model(tmp_path, bad)
    assert sc.bind_native_model(tmp_path, {'family': 'finger_gripper'}) is None


def test_the_launch_binds_the_physics_model_after_derivation_in_the_simulator_branch_only():
    launch = (REPO/'scenes/ur5_2f_test/launch/demo.launch.py').read_text()
    simulator = launch[launch.index('if backend == "simulator":'):launch.index('ExecuteProcess(cmd=[sys.executable, simulator_backend.__file__')]
    assert simulator.index('simulator_backend.prepare(') < simulator.index('simulation_collision.bind_native_model(output, record.payload)') \
        < simulator.index('destination_binding.augment_spec(')
    assert "simulator_spec['native_description_sha256'] = collision_binding['native_description_sha256']" in simulator
    assert launch.count('simulation_collision.') == simulator.count('simulation_collision.')       # never on the fake path
    assert 'simulation_collision' not in (REPO/'scripts/simulator_backend.py').read_text()          # evidence-bound module untouched
    # MoveIt's robot_description is the URDF prepare() returned, before the physics-only binding.
    assert simulator.index('robot_description_config, controllers_config_path, simulator_spec = simulator_backend.prepare(') < \
        simulator.index('simulation_collision.bind_native_model(')
